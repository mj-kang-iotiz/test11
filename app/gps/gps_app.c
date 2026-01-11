#include "gps_app.h"
#include "board_config.h"
#include "gps.h"
#include "gps_port.h"
#include "gps_unicore.h"
#include "ntrip_app.h"
#include "rtcm.h"
#include "led.h"
#include <string.h>
#include <stdlib.h>
#include "flash_params.h"
#include <math.h>
#include <stdio.h>
#include "base_auto_fix.h"
#include "ble_app.h"

#ifndef TAG
  #define TAG "GPS_APP"
#endif

#include "log.h"

typedef struct {
  gps_t gps;
  TaskHandle_t task;
  gps_type_t type;
  gps_id_t id;
  bool enabled;
  gps_fix_t last_fix;
} gps_instance_t;

static gps_instance_t gps_instances[GPS_ID_MAX] = {0};

/*===========================================================================
 * UM982 초기화 명령어
 *===========================================================================*/

static const char *um982_base_cmds[] = {
	// "CONFIG ANTENNA POWERON\r\n",
  // "FRESET\r\n",
  "unmask BDS\r\n",
  "unmask GPS\r\n",
  "unmask GLO\r\n",
  "unmask GAL\r\n",
  "unmask QZSS\r\n",
  
  "rtcm1033 com1 10\r\n",
  "rtcm1006 com1 10\r\n",
  "rtcm1074 com1 1\r\n", // gps msm4
  // "rtcm1124 com1 1\r\n", // beidou msm4
  // "rtcm1084 com1 1\r\n", // glonass msm4
  "rtcm1094 com1 1\r\n", // galileo msm4
  "gpgga com1 1\r\n",
  // "gpgsv com1 1\r\n",
  "BESTNAVB 1\r\n",

  // "CONFIG RTK MMPL 1\r\n",
  // "CONFIG PVTALG MULTI\r\n",
  // "CONFIG RTK RELIABILITY 3 1\r\n",
  // "MODE BASE TIME 120 0.1\r\n",
  // "mode base 37.4136149088 127.125455729 62.0923\r\n", // lat=40.07898324818,lon=116.23660197714,height=60.4265
};

static const char *um982_rover_cmds[] = {
  // "CONFIG ANTENNA POWERON\r\n",
  "unmask BDS\r\n",
  "unmask GPS\r\n",
  "unmask GLO\r\n",
  "unmask GAL\r\n",
  "unmask QZSS\r\n",
  "gpgga com1 1\r\n",
  // "gpgsv com1 1\r\n",
  "gpths com1 0.05\r\n",
  // "OBSVHA COM1 1\r\n", // slave antenna
  "BESTNAVB 0.05\r\n",
  "CONFIG HEADING FIXLENGTH\r\n"

  // "CONFIG PVTALG MULTI\r\n",
  // "CONFIG SMOOTH RTKHEIGHT 20\r\n",
  // "MASK 10\r\n",
  // "CONFIG RTK MMPL 1\r\n",
  // "CONFIG RTK CN0THD 1\r\n",
  // "CONFIG RTK RELIABILITY 3 2\r\n",
  // "config heading length 100 40\r\n",
};

/*===========================================================================
 * GPS 이벤트 핸들러
 *===========================================================================*/

static void gps_evt_handler(gps_t *gps, const gps_event_t *event) {
  gps_instance_t *inst = NULL;
  const board_config_t *config = board_get_config();

  // 해당 GPS 인스턴스 찾기
  for (uint8_t i = 0; i < GPS_CNT; i++) {
    if (gps_instances[i].enabled && &gps_instances[i].gps == gps) {
      inst = &gps_instances[i];
      break;
    }
  }

  if (!inst)
    return;

  // 이벤트 타입별 처리
  switch (event->type) {
  case GPS_EVENT_POSITION_UPDATED:
    LOG_DEBUG("GPS[%d] Position: lat=%.6f, lon=%.6f, alt=%.2f, fix=%d",
              inst->id, event->data.position.latitude, event->data.position.longitude,
              event->data.position.altitude, event->data.position.fix_type);

    // Base 모드: Fix 변경 시 base_auto_fix 모듈에 알림
    if (config->board == BOARD_TYPE_BASE_UM982) {
      if (event->data.position.fix_type != inst->last_fix) {
        base_auto_fix_on_gps_fix_changed(event->data.position.fix_type);
        inst->last_fix = event->data.position.fix_type;
      }

      // RTK Fix 시 위치 업데이트
      if (event->data.position.fix_type == GPS_FIX_RTK_FIX) {
        base_auto_fix_on_gga_update(event->data.position.latitude,
                                     event->data.position.longitude,
                                     event->data.position.altitude);
      }
    }
    break;

  case GPS_EVENT_HEADING_UPDATED:
    LOG_DEBUG("GPS[%d] Heading: %.2f deg", inst->id, event->data.heading.heading);
    break;

  case GPS_EVENT_RTCM_RECEIVED:
    // LoRa Base 모드: RTCM 데이터를 LoRa로 전송
    if (config->lora_mode == LORA_MODE_BASE) {
      rtcm_send_to_lora(gps);
    }
    break;

  default:
    break;
  }
}

/*===========================================================================
 * UM982 초기화 함수
 *===========================================================================*/

static bool gps_send_um982_cmds(gps_t *gps, const char **cmds, size_t count) {
  bool all_ok = true;

  for (size_t i = 0; i < count; i++) {
    LOG_INFO("[%d/%d] UM982 -> %s", i + 1, count, cmds[i]);

    bool result = gps_send_cmd_sync(gps, cmds[i], 2000);

    if (result) {
      LOG_INFO("[%d/%d] 성공", i + 1, count);
    } else {
      LOG_ERR("[%d/%d] 실패 (ERROR 또는 TIMEOUT)", i + 1, count);
      all_ok = false;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  return all_ok;
}

static bool gps_init_um982_base(gps_t *gps) {
  size_t cmd_count = sizeof(um982_base_cmds) / sizeof(um982_base_cmds[0]);
  LOG_INFO("UM982 Base 초기화 시작 (%d 개 명령)", cmd_count);

  bool result = gps_send_um982_cmds(gps, um982_base_cmds, cmd_count);

  if (result) {
    LOG_INFO("UM982 Base 초기화 성공");
  } else {
    LOG_ERR("UM982 Base 초기화 실패 (일부 명령 오류)");
  }

  return result;
}

static bool gps_init_um982_rover(gps_t *gps) {
  size_t cmd_count = sizeof(um982_rover_cmds) / sizeof(um982_rover_cmds[0]);
  LOG_INFO("UM982 Rover 초기화 시작 (%d 개 명령)", cmd_count);

  bool result = gps_send_um982_cmds(gps, um982_rover_cmds, cmd_count);

  if (result) {
    LOG_INFO("UM982 Rover 초기화 성공");
  } else {
    LOG_ERR("UM982 Rover 초기화 실패 (일부 명령 오류)");
  }

  return result;
}

/*===========================================================================
 * GPS 앱 태스크
 *===========================================================================*/

static void gps_app_task(void *pvParameter) {
  gps_id_t id = (gps_id_t)(uintptr_t)pvParameter;
  gps_instance_t *inst = &gps_instances[id];
  const board_config_t *config = board_get_config();

  LOG_INFO("GPS 앱 태스크[%d] 시작", id);

  // GPS 서브시스템 초기화
  if (!gps_init(&inst->gps)) {
    LOG_ERR("GPS[%d] 서브시스템 초기화 실패", id);
    inst->enabled = false;
    vTaskDelete(NULL);
    return;
  }

  // 이벤트 핸들러 등록
  gps_set_evt_handler(&inst->gps, gps_evt_handler);

  // 하드웨어 초기화
  if (gps_port_init(&inst->gps) != 0) {
    LOG_ERR("GPS[%d] 하드웨어 초기화 실패", id);
    inst->enabled = false;
    vTaskDelete(NULL);
    return;
  }

  // GPS 통신 시작
  gps_port_start(&inst->gps);
  LOG_INFO("GPS[%d] 하드웨어 초기화 완료", id);

  // 안정화 대기
  vTaskDelay(pdMS_TO_TICKS(1000));

  // UM982 초기화 명령어 전송
  if (inst->type == GPS_TYPE_UM982) {
    if (config->lora_mode == LORA_MODE_BASE) {
      gps_init_um982_base(&inst->gps);
    } else if (config->lora_mode == LORA_MODE_ROVER) {
      gps_init_um982_rover(&inst->gps);
    }
  }

  LOG_INFO("GPS[%d] 초기화 완료, 메인 루프 진입", id);

  // 메인 루프 (필요시 추가 작업 수행)
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 주기적 상태 체크 또는 유지보수 작업
    // (현재는 gps_process_task에서 데이터 처리 중)
  }

  vTaskDelete(NULL);
}

/*===========================================================================
 * GPS 앱 태스크 생성/삭제
 *===========================================================================*/

/**
 * @brief 특정 GPS 앱 태스크 생성
 */
static bool gps_app_create(gps_id_t id) {
  const board_config_t *config = board_get_config();

  if (id >= GPS_ID_MAX || id >= config->gps_cnt) {
    LOG_ERR("GPS[%d] 잘못된 ID", id);
    return false;
  }

  if (gps_instances[id].enabled) {
    LOG_WARN("GPS[%d] 이미 실행 중", id);
    return false;
  }

  gps_type_t type = config->gps[id];
  LOG_INFO("GPS[%d] 생성 시작 (타입: %s)", id,
           type == GPS_TYPE_UM982 ? "UM982" : "UNKNOWN");

  gps_instances[id].type = type;
  gps_instances[id].id = id;
  gps_instances[id].enabled = true;
  gps_instances[id].last_fix = GPS_FIX_INVALID;

  char task_name[16];
  snprintf(task_name, sizeof(task_name), "gps_app_%d", id);

  BaseType_t ret = xTaskCreate(
      gps_app_task,
      task_name,
      2048,
      (void *)(uintptr_t)id,
      tskIDLE_PRIORITY + 2,
      &gps_instances[id].task
  );

  if (ret != pdPASS) {
    LOG_ERR("GPS[%d] 태스크 생성 실패", id);
    gps_instances[id].enabled = false;
    return false;
  }

  LOG_INFO("GPS[%d] 태스크 생성 완료", id);
  return true;
}

/**
 * @brief 특정 GPS 앱 태스크 삭제
 */
static bool gps_app_destroy(gps_id_t id) {
  if (id >= GPS_ID_MAX || !gps_instances[id].enabled) {
    LOG_WARN("GPS[%d] 실행 중이 아님", id);
    return false;
  }

  LOG_INFO("GPS[%d] 종료 시작", id);

  // GPS 통신 중지
  gps_port_stop(&gps_instances[id].gps);

  // 태스크 삭제 (태스크는 자체 종료하도록 플래그 설정)
  gps_instances[id].enabled = false;

  // 태스크가 자체 종료할 때까지 대기
  if (gps_instances[id].task) {
    vTaskDelay(pdMS_TO_TICKS(100));
    // 태스크는 gps_app_task 내에서 vTaskDelete(NULL) 호출
    gps_instances[id].task = NULL;
  }

  LOG_INFO("GPS[%d] 종료 완료", id);
  return true;
}

/**
 * @brief 모든 GPS 앱 태스크 생성 및 초기화
 */
void gps_app_start_all(void) {
  const board_config_t *config = board_get_config();

  LOG_INFO("GPS 앱 전체 시작");

  for (uint8_t i = 0; i < config->gps_cnt && i < GPS_ID_MAX; i++) {
    gps_app_create((gps_id_t)i);
  }

  LOG_INFO("GPS 앱 전체 시작 완료");

  // Base Auto-Fix 초기화 (필요시)
  if (config->board == BOARD_TYPE_BASE_UM982) {
    user_params_t *params = flash_params_get_current();
    if (params->base_auto_fix_enabled) {
      LOG_INFO("Base Auto-Fix 활성화");

      if (base_auto_fix_init(GPS_ID_BASE)) {
        if (base_auto_fix_start()) {
          LOG_INFO("Base Auto-Fix 시작 성공");
        } else {
          LOG_ERR("Base Auto-Fix 시작 실패");
        }
      } else {
        LOG_ERR("Base Auto-Fix 초기화 실패");
      }
    }
  }
}

/**
 * @brief 모든 GPS 앱 태스크 종료 및 정리
 */
void gps_app_stop_all(void) {
  LOG_INFO("GPS 앱 전체 종료");

  for (uint8_t i = 0; i < GPS_ID_MAX; i++) {
    if (gps_instances[i].enabled) {
      gps_app_destroy((gps_id_t)i);
    }
  }

  LOG_INFO("GPS 앱 전체 종료 완료");
}

/*===========================================================================
 * 유틸리티 함수
 *===========================================================================*/

/**
 * @brief 특정 GPS ID의 핸들 가져오기
 */
gps_t *gps_get_instance_handle(gps_id_t id) {
  if (id >= GPS_ID_MAX || !gps_instances[id].enabled) {
    return NULL;
  }

  return &gps_instances[id].gps;
}
