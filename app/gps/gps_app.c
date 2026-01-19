#include "gps_app.h"
#include "board_config.h"
#include "event_bus.h"
#include "gps.h"
#include "gps_port.h"
#include "gps_role.h"
#include "gps_unicore.h"
#include "ntrip_app.h"
#include "led.h"
#include <string.h>
#include <stdlib.h>
#include "flash_params.h"
#include <math.h>
#include <stdio.h>
#include "ble_app.h"

#ifndef TAG
  #define TAG "GPS_APP"
#endif

#include "log.h"

/*===========================================================================
 * GPS 앱 구조체 (싱글 인스턴스)
 *===========================================================================*/
typedef struct {
  gps_t gps;
  TaskHandle_t task;
  gps_type_t type;
  bool enabled;
  gps_fix_t last_fix;
} gps_app_ctx_t;

static gps_app_ctx_t g_gps_app = {0};

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
  gps_app_ctx_t *ctx = &g_gps_app;

  /* GPS 인스턴스 확인 */
  if (!ctx->enabled || &ctx->gps != gps) {
    return;
  }

  /* 이벤트 타입별 처리 */
  switch (event->type) {
  case GPS_EVENT_POSITION_UPDATED:
    LOG_DEBUG("GPS Position: lat=%.6f, lon=%.6f, alt=%.2f, fix=%d",
              event->data.position.latitude, event->data.position.longitude,
              event->data.position.altitude, event->data.position.fix_type);

    /* Base 모드: 이벤트 버스로 GPS 상태 발행 */
    if (gps_role_is_base()) {
      if (event->data.position.fix_type != ctx->last_fix) {
        event_t ev = {
          .type = EVENT_GPS_FIX_CHANGED,
          .data.gps_fix = {
            .fix = event->data.position.fix_type,
            .gps_id = GPS_ID_BASE
          }
        };
        event_bus_publish(&ev);
        ctx->last_fix = event->data.position.fix_type;
      }

      /* RTK Fix 시 위치 업데이트 */
      if (event->data.position.fix_type == GPS_FIX_RTK_FIX) {
        event_t ev = {
          .type = EVENT_GPS_GGA_UPDATE,
          .data.gps_gga = {
            .lat = event->data.position.latitude,
            .lon = event->data.position.longitude,
            .alt = event->data.position.altitude,
            .gps_id = GPS_ID_BASE
          }
        };
        event_bus_publish(&ev);
      }
    }
    break;

  case GPS_EVENT_HEADING_UPDATED:
    LOG_DEBUG("GPS Heading: %.2f deg", event->data.heading.heading);
    break;

  case GPS_EVENT_RTCM_RECEIVED:
    /* Base 모드: RTCM 이벤트 발행 (LoRa에서 구독) */
    if (gps_role_is_base()) {
      event_t ev = {
        .type = EVENT_RTCM_FOR_LORA,
        .data.rtcm = {
          .gps_id = GPS_ID_BASE
        }
      };
      event_bus_publish(&ev);
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
  (void)pvParameter;
  gps_app_ctx_t *ctx = &g_gps_app;

  LOG_INFO("GPS 앱 태스크 시작");

  /* GPS 서브시스템 초기화 */
  if (!gps_init(&ctx->gps)) {
    LOG_ERR("GPS 서브시스템 초기화 실패");
    ctx->enabled = false;
    vTaskDelete(NULL);
    return;
  }

  /* 이벤트 핸들러 등록 */
  gps_set_evt_handler(&ctx->gps, gps_app_evt_handler);

  /* 하드웨어 초기화 */
  if (gps_port_init(&ctx->gps) != 0) {
    LOG_ERR("GPS 하드웨어 초기화 실패");
    gps_deinit(&ctx->gps);  /* 이미 생성된 리소스 정리 */
    ctx->enabled = false;
    vTaskDelete(NULL);
    return;
  }

  /* GPS 통신 시작 */
  gps_port_start(&ctx->gps);
  LOG_INFO("GPS 하드웨어 초기화 완료");

  /* 안정화 대기 */
  vTaskDelay(pdMS_TO_TICKS(1000));

  /* UM982 초기화 명령어 전송 (역할에 따라) */
  if (ctx->type == GPS_TYPE_UM982) {
    if (gps_role_is_base()) {
      gps_init_um982_base(&ctx->gps);
    } else if (gps_role_is_rover()) {
      gps_init_um982_rover(&ctx->gps);
    }
  }

  LOG_INFO("GPS 초기화 완료, 메인 루프 진입");

  /* 메인 루프 (필요시 추가 작업 수행) */
  while (ctx->enabled) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    /* 주기적 상태 체크 또는 유지보수 작업 */
  }

  LOG_INFO("GPS 앱 태스크 종료");
  vTaskDelete(NULL);
}

/*===========================================================================
 * GPS 앱 시작/중지
 *===========================================================================*/

/**
 * @brief GPS 앱 시작 (싱글 인스턴스)
 */
void gps_app_start(void) {
  gps_app_ctx_t *ctx = &g_gps_app;
  const board_config_t *config = board_get_config();

  if (ctx->enabled) {
    LOG_WARN("GPS 이미 실행 중");
    return;
  }

  LOG_INFO("GPS 앱 시작");

  /* 역할 감지 */
  gps_role_detect();
  LOG_INFO("GPS 역할: %s", gps_role_to_string(gps_role_get()));

  /* GPS 타입 설정 */
  ctx->type = config->gps[0];
  ctx->enabled = true;
  ctx->last_fix = GPS_FIX_INVALID;

  LOG_INFO("GPS 생성 (타입: %s)",
           ctx->type == GPS_TYPE_UM982 ? "UM982" : "UNKNOWN");

  /* 태스크 생성 */
  BaseType_t ret = xTaskCreate(
      gps_app_task,
      "gps_app",
      2048,
      NULL,
      tskIDLE_PRIORITY + 2,
      &ctx->task
  );

  if (ret != pdPASS) {
    LOG_ERR("GPS 태스크 생성 실패");
    ctx->enabled = false;
    return;
  }

  LOG_INFO("GPS 앱 시작 완료");
}

/**
 * @brief GPS 앱 종료 (리소스는 유지)
 *
 * 태스크와 통신만 중지하고, OS 리소스(큐, 세마포어)는 유지합니다.
 * 재시작이 필요한 경우 gps_app_start()를 다시 호출하면 됩니다.
 */
void gps_app_stop(void) {
  gps_app_ctx_t *ctx = &g_gps_app;

  if (!ctx->enabled) {
    LOG_WARN("GPS 실행 중이 아님");
    return;
  }

  LOG_INFO("GPS 앱 종료 시작");

  /* 1. 하드웨어 통신 정지 */
  gps_port_stop(&ctx->gps);

  /* 2. GPS 코어 태스크 종료 (gps_process_task) */
  gps_stop(&ctx->gps);

  /* 3. 앱 태스크 종료 플래그 설정 */
  ctx->enabled = false;

  /* 4. 태스크 종료 대기 */
  if (ctx->task) {
    uint32_t wait_count = 0;
    while (eTaskGetState(ctx->task) != eDeleted && wait_count < 50) {
      vTaskDelay(pdMS_TO_TICKS(10));
      wait_count++;
    }

    if (eTaskGetState(ctx->task) != eDeleted) {
      LOG_WARN("GPS 앱 태스크 강제 삭제");
      vTaskDelete(ctx->task);
    }
    ctx->task = NULL;
  }

  /* 5. 상태 초기화 */
  ctx->last_fix = GPS_FIX_INVALID;

  LOG_INFO("GPS 앱 종료 완료");
}

/*===========================================================================
 * GPS 앱 리소스 해제
 *===========================================================================*/

/**
 * @brief GPS 앱 완전 해제
 *
 * 태스크 종료, 통신 정지, OS 리소스(큐, 세마포어, 뮤텍스) 모두 해제합니다.
 */
void gps_app_deinit(void) {
  gps_app_ctx_t *ctx = &g_gps_app;

  LOG_INFO("GPS 리소스 해제 시작");

  /* 앱 종료 */
  if (ctx->enabled) {
    gps_app_stop();
  }

  /* GPS 코어 리소스 해제 */
  gps_deinit(&ctx->gps);

  LOG_INFO("GPS 리소스 해제 완료");
}

/*===========================================================================
 * 유틸리티 함수
 *===========================================================================*/

/**
 * @brief GPS 핸들 가져오기
 */
gps_t *gps_get_handle(void) {
  gps_app_ctx_t *ctx = &g_gps_app;

  if (!ctx->enabled) {
    return NULL;
  }

  return &ctx->gps;
}

/**
 * @brief GPS 핸들 가져오기 (레거시 호환용)
 * @param id 무시됨 (싱글 인스턴스)
 */
gps_t *gps_get_instance_handle(gps_id_t id) {
  (void)id;
  return gps_get_handle();
}
