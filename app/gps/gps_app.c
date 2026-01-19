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
} gps_app_t;

static gps_app_t gps_apps[GPS_ID_MAX] = {0};

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

static void gps_app_evt_handler(gps_t *gps, const gps_event_t *event) {
  gps_instance_t *inst = NULL;
  const board_config_t *config = board_get_config();

  // 해당 GPS 인스턴스 찾기
  for (uint8_t i = 0; i < GPS_CNT; i++) {
    if (gps_apps[i].enabled && &gps_apps[i].gps == gps) {
      inst = &gps_apps[i];
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
//        base_auto_fix_on_gps_fix_changed(event->data.position.fix_type);
        inst->last_fix = event->data.position.fix_type;
      }

      // RTK Fix 시 위치 업데이트
      if (event->data.position.fix_type == GPS_FIX_RTK_FIX) {
//        base_auto_fix_on_gga_update(event->data.position.latitude,
//                                     event->data.position.longitude,
//                                     event->data.position.altitude);
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

#define GPS_CMD_MAX_RETRIES     3       /* 최대 재시도 횟수 */
#define GPS_CMD_RETRY_DELAY_MS  200     /* 재시도 간 대기 시간 (ms) */
#define GPS_CMD_TIMEOUT_MS      2000    /* 명령어 타임아웃 (ms) */
#define GPS_CMD_INTERVAL_MS     100     /* 명령어 간 대기 시간 (ms) */

/**
 * @brief 단일 명령어 전송 (재시도 포함)
 *
 * @param gps GPS 핸들
 * @param cmd 명령어 문자열
 * @param cmd_idx 명령어 인덱스 (로그용)
 * @param total_cmds 전체 명령어 수 (로그용)
 * @return true: 성공, false: 재시도 후에도 실패
 */
static bool gps_send_cmd_with_retry(gps_t *gps, const char *cmd,
                                     size_t cmd_idx, size_t total_cmds) {
  for (int retry = 0; retry < GPS_CMD_MAX_RETRIES; retry++) {
    if (retry > 0) {
      LOG_WARN("[%zu/%zu] 재시도 %d/%d: %s",
               cmd_idx, total_cmds, retry, GPS_CMD_MAX_RETRIES - 1, cmd);
      vTaskDelay(pdMS_TO_TICKS(GPS_CMD_RETRY_DELAY_MS));
    } else {
      LOG_INFO("[%zu/%zu] UM982 -> %s", cmd_idx, total_cmds, cmd);
    }

    bool result = gps_send_cmd_sync(gps, cmd, GPS_CMD_TIMEOUT_MS);

    if (result) {
      LOG_INFO("[%zu/%zu] 성공%s", cmd_idx, total_cmds,
               retry > 0 ? " (재시도 후)" : "");
      return true;
    }
  }

  LOG_ERR("[%zu/%zu] 실패 - %d회 재시도 후 포기: %s",
          cmd_idx, total_cmds, GPS_CMD_MAX_RETRIES, cmd);
  return false;
}

/**
 * @brief UM982 명령어 배열 전송
 *
 * @param gps GPS 핸들
 * @param cmds 명령어 배열
 * @param count 명령어 수
 * @param failed_count 실패한 명령어 수 (출력, NULL 가능)
 * @return true: 전체 성공, false: 하나 이상 실패
 */
static bool gps_send_um982_cmds(gps_t *gps, const char **cmds, size_t count,
                                 size_t *failed_count) {
  size_t fail_cnt = 0;

  for (size_t i = 0; i < count; i++) {
    if (!gps_send_cmd_with_retry(gps, cmds[i], i + 1, count)) {
      fail_cnt++;
    }
    vTaskDelay(pdMS_TO_TICKS(GPS_CMD_INTERVAL_MS));
  }

  if (failed_count) {
    *failed_count = fail_cnt;
  }

  return (fail_cnt == 0);
}

static bool gps_init_um982_base(gps_t *gps) {
  size_t cmd_count = sizeof(um982_base_cmds) / sizeof(um982_base_cmds[0]);
  size_t failed_count = 0;

  LOG_INFO("UM982 Base 초기화 시작 (%zu 개 명령, 최대 %d회 재시도)",
           cmd_count, GPS_CMD_MAX_RETRIES);

  bool result = gps_send_um982_cmds(gps, um982_base_cmds, cmd_count, &failed_count);

  if (result) {
    LOG_INFO("UM982 Base 초기화 성공 (%zu/%zu 명령 완료)", cmd_count, cmd_count);
  } else {
    LOG_ERR("UM982 Base 초기화 실패 (%zu/%zu 명령 실패)",
            failed_count, cmd_count);
  }

  return result;
}

static bool gps_init_um982_rover(gps_t *gps) {
  size_t cmd_count = sizeof(um982_rover_cmds) / sizeof(um982_rover_cmds[0]);
  size_t failed_count = 0;

  LOG_INFO("UM982 Rover 초기화 시작 (%zu 개 명령, 최대 %d회 재시도)",
           cmd_count, GPS_CMD_MAX_RETRIES);

  bool result = gps_send_um982_cmds(gps, um982_rover_cmds, cmd_count, &failed_count);

  if (result) {
    LOG_INFO("UM982 Rover 초기화 성공 (%zu/%zu 명령 완료)", cmd_count, cmd_count);
  } else {
    LOG_ERR("UM982 Rover 초기화 실패 (%zu/%zu 명령 실패)",
            failed_count, cmd_count);
  }

  return result;
}

/*===========================================================================
 * GPS 앱 태스크
 *===========================================================================*/

static void gps_app_task(void *pvParameter) {
  gps_id_t id = (gps_id_t)(uintptr_t)pvParameter;
  gps_instance_t *inst = &gps_apps[id];
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
  gps_set_evt_handler(&inst->gps, gps_app_evt_handler);

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

  if (gps_apps[id].enabled) {
    LOG_WARN("GPS[%d] 이미 실행 중", id);
    return false;
  }

  gps_type_t type = config->gps[id];
  LOG_INFO("GPS[%d] 생성 시작 (타입: %s)", id,
           type == GPS_TYPE_UM982 ? "UM982" : "UNKNOWN");

  gps_apps[id].type = type;
  gps_apps[id].id = id;
  gps_apps[id].enabled = true;
  gps_apps[id].last_fix = GPS_FIX_INVALID;

  char task_name[16];
  snprintf(task_name, sizeof(task_name), "gps_app_%d", id);

  BaseType_t ret = xTaskCreate(
      gps_app_task,
      task_name,
      2048,
      (void *)(uintptr_t)id,
      tskIDLE_PRIORITY + 2,
      &gps_apps[id].task
  );

  if (ret != pdPASS) {
    LOG_ERR("GPS[%d] 태스크 생성 실패", id);
    gps_apps[id].enabled = false;
    return false;
  }

  LOG_INFO("GPS[%d] 태스크 생성 완료", id);
  return true;
}

/**
 * @brief 특정 GPS 앱 태스크 삭제 (내부용)
 */
static bool gps_app_destroy(gps_id_t id, bool cleanup_resources) {
  if (id >= GPS_ID_MAX || !gps_apps[id].enabled) {
    LOG_WARN("GPS[%d] 실행 중이 아님", id);
    return false;
  }

  LOG_INFO("GPS[%d] 종료 시작", id);

  gps_instance_t *inst = &gps_apps[id];

  /* 1. 하드웨어 통신 정지 (UART, DMA 등) */
  gps_port_stop(&inst->gps);

  /* 2. GPS 앱 태스크 종료 대기 */
  inst->enabled = false;
  if (inst->task) {
    /* 태스크 종료 대기 (최대 500ms) */
    uint32_t wait_count = 0;
    while (eTaskGetState(inst->task) != eDeleted && wait_count < 50) {
      vTaskDelay(pdMS_TO_TICKS(10));
      wait_count++;
    }

    /* 태스크가 종료되지 않으면 강제 삭제 */
    if (eTaskGetState(inst->task) != eDeleted) {
      LOG_WARN("GPS[%d] 앱 태스크 강제 삭제", id);
      vTaskDelete(inst->task);
    }
    inst->task = NULL;
  }

  /* 3. GPS 코어 리소스 해제 (선택적) */
  if (cleanup_resources) {
    gps_deinit(&inst->gps);
  }

  /* 4. 인스턴스 상태 초기화 */
  inst->last_fix = GPS_FIX_INVALID;

  LOG_INFO("GPS[%d] 종료 완료", id);
  return true;
}

/**
 * @brief GPS 앱 시작
 */
void gps_app_start(void) {
  const board_config_t *config = board_get_config();

  LOG_INFO("GPS 앱 시작");

  for (uint8_t i = 0; i < config->gps_cnt && i < GPS_ID_MAX; i++) {
    gps_app_create((gps_id_t)i);
  }

  LOG_INFO("GPS 앱 시작 완료");

  // Base Auto-Fix 초기화 (필요시)
  // if (config->board == BOARD_TYPE_BASE_UM982) {
  //   user_params_t *params = flash_params_get_current();
  //   if (params->base_auto_fix_enabled) {
  //     LOG_INFO("Base Auto-Fix 활성화");

  //     if (base_auto_fix_init(GPS_ID_BASE)) {
  //       if (base_auto_fix_start()) {
  //         LOG_INFO("Base Auto-Fix 시작 성공");
  //       } else {
  //         LOG_ERR("Base Auto-Fix 시작 실패");
  //       }
  //     } else {
  //       LOG_ERR("Base Auto-Fix 초기화 실패");
  //     }
  //   }
  // }
}

/**
 * @brief GPS 앱 종료 (리소스는 유지)
 *
 * 태스크와 통신만 중지하고, OS 리소스(큐, 세마포어)는 유지합니다.
 * 재시작이 필요한 경우 gps_app_start()를 다시 호출하면 됩니다.
 */
void gps_app_stop(void) {
  LOG_INFO("GPS 앱 종료");

  for (uint8_t i = 0; i < GPS_ID_MAX; i++) {
    if (gps_apps[i].enabled) {
      gps_app_destroy((gps_id_t)i, false);  /* 리소스 유지 */
    }
  }

  LOG_INFO("GPS 앱 종료 완료");
}

/*===========================================================================
 * GPS 앱 리소스 해제
 *===========================================================================*/

/**
 * @brief 특정 GPS 앱 완전 해제
 *
 * 태스크 종료, 통신 정지, OS 리소스(큐, 세마포어, 뮤텍스) 모두 해제합니다.
 * 더 이상 해당 GPS를 사용하지 않을 때 호출합니다.
 *
 * @param id GPS ID
 * @return true: 성공, false: 실패
 */
bool gps_app_deinit(gps_id_t id) {
  if (id >= GPS_ID_MAX) {
    LOG_ERR("GPS[%d] 잘못된 ID", id);
    return false;
  }

  LOG_INFO("GPS[%d] 리소스 해제 시작", id);

  if (gps_apps[id].enabled) {
    gps_app_destroy(id, true);  /* 리소스도 해제 */
  } else {
    /* 이미 정지됨 - OS 리소스만 정리 */
    gps_deinit(&gps_apps[id].gps);
  }

  LOG_INFO("GPS[%d] 리소스 해제 완료", id);
  return true;
}

/**
 * @brief 모든 GPS 앱 완전 해제
 *
 * 모든 GPS 태스크 종료, 통신 정지, OS 리소스 해제합니다.
 * 시스템 종료 또는 GPS 기능 완전 비활성화 시 호출합니다.
 */
void gps_app_deinit_all(void) {
  LOG_INFO("모든 GPS 리소스 해제 시작");

  for (uint8_t i = 0; i < GPS_ID_MAX; i++) {
    gps_app_deinit((gps_id_t)i);
  }

  LOG_INFO("모든 GPS 리소스 해제 완료");
}

/*===========================================================================
 * 유틸리티 함수
 *===========================================================================*/

/**
 * @brief 특정 GPS ID의 핸들 가져오기
 */
gps_t *gps_get_instance_handle(gps_id_t id) {
  if (id >= GPS_ID_MAX || !gps_apps[id].enabled) {
    return NULL;
  }

  return &gps_apps[id].gps;
}
