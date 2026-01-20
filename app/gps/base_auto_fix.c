#include "base_auto_fix.h"
#include "event_bus.h"
#include "board_config.h"
#include "gps_app.h"
#include "gps_role.h"
#include "gps_unicore.h"
#include "ntrip_app.h"
#include "gsm_port.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "ble_app.h"
#include "stdbool.h"
#include "led.h"

#ifndef TAG
#define TAG "BASE_AUTO_FIX"
#endif

#include "log.h"

/*===========================================================================
 * 설정
 *===========================================================================*/
#define AVERAGING_DURATION_SEC 50     /* 평균 계산 시간 (초) */
#define MAX_SAMPLES 50                /* 최대 샘플 수 */
#define MIN_SAMPLES 20                /* 최소 샘플 수 */
#define SIGMA_THRESHOLD 3.0           /* 이상치 제거 임계값 (3σ) */

/*===========================================================================
 * 내부 변수
 *===========================================================================*/
static base_auto_fix_state_t state = BASE_AUTO_FIX_DISABLED;
static uint8_t gps_id = 0;
static TimerHandle_t averaging_timer = NULL;
static TimerHandle_t status_timer = NULL;

/* 워커 태스크 및 이벤트 큐 */
static TaskHandle_t worker_task = NULL;
static QueueHandle_t event_queue = NULL;
static volatile bool worker_running = false;  /* 태스크 종료 플래그 */

typedef enum {
  BASE_AUTO_FIX_EVENT_AVERAGING_COMPLETE,
  BASE_AUTO_FIX_EVENT_SHUTDOWN           /* 종료 이벤트 */
} base_auto_fix_event_t;

// 좌표 샘플 버퍼
static coord_sample_t samples[MAX_SAMPLES];
static uint32_t sample_count = 0;

// 평균 좌표 결과
static coord_average_t avg_result = {0};

// 내부 함수 선언
static void averaging_timer_callback(TimerHandle_t xTimer);
static bool calculate_average_with_outlier_removal(void);
static bool switch_to_base_fixed_mode(void);
static void shutdown_ntrip_and_lte(void);
static void base_auto_fix_worker_task(void *pvParameter);
static void status_timer_callback(TimerHandle_t xTimer);

// 이벤트 핸들러 선언
static void on_gps_fix_changed(const event_t *event, void *user_data);
static void on_gps_gga_update(const event_t *event, void *user_data);
static void on_ntrip_connected(const event_t *event, void *user_data);

/**

 * @brief 모듈 초기화

 */

bool base_auto_fix_init(uint8_t id) {
  gps_id = id;
  state = BASE_AUTO_FIX_DISABLED;
  sample_count = 0;
  memset(&avg_result, 0, sizeof(avg_result));

  // 타이머 생성 (60초 원샷)
  if (averaging_timer == NULL) {
    averaging_timer = xTimerCreate("base_avg_timer",
                                    pdMS_TO_TICKS(AVERAGING_DURATION_SEC * 1000),
                                    pdFALSE,  // 원샷
                                    NULL,
                                    averaging_timer_callback);

    if (averaging_timer == NULL) {
      LOG_ERR("타이머 생성 실패");
      return false;
    }
  }

  if (status_timer == NULL) {
    status_timer = xTimerCreate("status_timer",
                                    pdMS_TO_TICKS(10 * 1000),
                                    pdTRUE,
                                    NULL,
                                    status_timer_callback);

    if (status_timer == NULL) {
      LOG_ERR("타이머 생성 실패");
      return false;
    }
  }

  xTimerStart(status_timer, 0);

  // 이벤트 큐 생성
  if (event_queue == NULL) {
    event_queue = xQueueCreate(5, sizeof(base_auto_fix_event_t));
    if (event_queue == NULL) {
      LOG_ERR("이벤트 큐 생성 실패");
      return false;
    }
  }

  // 워커 태스크 생성
  if (worker_task == NULL) {
    BaseType_t ret = xTaskCreate(base_auto_fix_worker_task,
                                  "base_auto_fix",
                                  2048,  // 스택 크기 (블로킹 작업 포함)
                                  NULL,
                                  tskIDLE_PRIORITY + 2,
                                  &worker_task);
    if (ret != pdPASS) {
      LOG_ERR("워커 태스크 생성 실패");
      vQueueDelete(event_queue);
      event_queue = NULL;
      return false;
    }
  }

  // 이벤트 버스 구독
  event_bus_subscribe(EVENT_GPS_FIX_CHANGED, on_gps_fix_changed, NULL);
  event_bus_subscribe(EVENT_GPS_GGA_UPDATE, on_gps_gga_update, NULL);
  event_bus_subscribe(EVENT_NTRIP_CONNECTED, on_ntrip_connected, NULL);

  LOG_INFO("Base Auto-Fix 모듈 초기화 완료");

  return true;
}


/**
 * @brief Auto-Fix 모드 시작
 */
bool base_auto_fix_start(void) {
  if (state != BASE_AUTO_FIX_DISABLED) {
    LOG_WARN("이미 실행 중 (state=%d)", state);
    return false;
  }

  LOG_INFO("Base Auto-Fix 모드 시작");
  state = BASE_AUTO_FIX_INIT;
  sample_count = 0;
  memset(&avg_result, 0, sizeof(avg_result));

  // NTRIP 연결 대기 상태로 전환
  state = BASE_AUTO_FIX_NTRIP_WAIT;
  LOG_INFO("NTRIP 연결 대기 중...");

  return true;
}

/**
 * @brief Auto-Fix 모드 중지
 */
void base_auto_fix_stop(void) {

  if (averaging_timer != NULL) {

    xTimerStop(averaging_timer, 0);

  }

  // 이벤트 큐 클리어
  if (event_queue != NULL) {
    xQueueReset(event_queue);
  }

  state = BASE_AUTO_FIX_DISABLED;

  sample_count = 0;

  LOG_INFO("Base Auto-Fix 모드 중지");

}



/**

 * @brief 현재 상태 조회

 */

base_auto_fix_state_t base_auto_fix_get_state(void) {

  return state;

}



/**
 * @brief GPS Fix 상태 변화 이벤트 핸들러
 */
static void on_gps_fix_changed(const event_t *event, void *user_data) {
  (void)user_data;

  if (state == BASE_AUTO_FIX_DISABLED || state == BASE_AUTO_FIX_COMPLETED) {
    return;
  }

  gps_fix_t fix = event->data.gps_fix.fix;

  // RTK Fix 진입 감지
  if (state == BASE_AUTO_FIX_WAIT_RTK_FIX && fix == GPS_FIX_RTK_FIX) {
    LOG_INFO("RTK Fix 진입! 좌표 평균 계산 시작");
    state = BASE_AUTO_FIX_AVERAGING;
    sample_count = 0;

    // 타이머 시작
    xTimerStart(averaging_timer, 0);
  }
  // RTK Fix 이탈 감지 (평균 계산 중)
  else if (state == BASE_AUTO_FIX_AVERAGING && fix != GPS_FIX_RTK_FIX) {
    LOG_WARN("RTK Fix 이탈 (fix=%d), 평균 계산 중단", fix);
    xTimerStop(averaging_timer, 0);
    state = BASE_AUTO_FIX_WAIT_RTK_FIX;
    sample_count = 0;
  }
}


/**
 * @brief GGA 데이터 업데이트 이벤트 핸들러
 */
static void on_gps_gga_update(const event_t *event, void *user_data) {
  (void)user_data;

  if (state != BASE_AUTO_FIX_AVERAGING) {
    return;
  }

  if (sample_count < MAX_SAMPLES) {
    samples[sample_count].lat = event->data.gps_gga.lat;
    samples[sample_count].lon = event->data.gps_gga.lon;
    samples[sample_count].alt = event->data.gps_gga.alt;
    sample_count++;

    char buf[30];
    sprintf(buf, "Start Averaging %u%%\n\r", sample_count * 2);
    ble_app_send(buf, strlen(buf));
  }
}


/**
 * @brief NTRIP 연결 상태 업데이트 이벤트 핸들러
 */
static void on_ntrip_connected(const event_t *event, void *user_data) {
  (void)user_data;

  bool connected = event->data.ntrip.connected;

  if (state == BASE_AUTO_FIX_NTRIP_WAIT && connected) {
    LOG_INFO("NTRIP 연결됨! RTK Fix 대기 중...");
    state = BASE_AUTO_FIX_WAIT_RTK_FIX;
  }
}



/**

 * @brief 평균 좌표 조회

 */

bool base_auto_fix_get_average_coord(coord_average_t *result) {

  if (result == NULL) {

    return false;

  }



  if (avg_result.count == 0) {

    return false;

  }



  *result = avg_result;

  return true;

}



/**

 * @brief 평균 계산 타이머 콜백 (60초 후 호출)
 *
 * 타이머 콜백에서는 블로킹 작업을 하지 않고,
 * 워커 태스크에 이벤트만 전달합니다.

 */

static void averaging_timer_callback(TimerHandle_t xTimer) {

  LOG_INFO("평균 계산 타이머 만료 (샘플 수: %lu)", sample_count);

  // 워커 태스크에 이벤트 전송
  base_auto_fix_event_t event = BASE_AUTO_FIX_EVENT_AVERAGING_COMPLETE;

  if (xQueueSend(event_queue, &event, 0) != pdTRUE) {
    LOG_ERR("워커 태스크에 이벤트 전송 실패");
    state = BASE_AUTO_FIX_FAILED;
  }
}

static void status_timer_callback(TimerHandle_t xTimer) {

  uint8_t fix = gps_get_instance_handle(0)->nmea_data.gga.fix;
  led_color_t gsm_status = led_get_color(LED_ID_1);

  bool ntrip_connected = ntrip_is_connected();

  uint8_t connect_status = 2;

  // NTRIP 실제 상태와 LED 상태를 모두 고려
  if(gsm_status == LED_COLOR_NONE || gsm_status == LED_COLOR_RED)
  {
    connect_status = 2;  // 접속 실패
  }
  else if(gsm_status == LED_COLOR_YELLOW)
  {
    connect_status = 0;  // 접속 중
  }
  else if(gsm_status == LED_COLOR_GREEN)
  {
    // GREEN LED지만 실제로 연결되지 않은 경우(태스크 삭제 등) 처리
    if(ntrip_connected)
    {
      connect_status = 1;  // 접속 성공
    }
    else
    {
      connect_status = 2;  // 접속 실패 (태스크 삭제됨)
    }
  }
  
  char buf[10];
  sprintf(buf, "%d,%d\n\r", connect_status, fix);
  ble_app_send(buf, strlen(buf));

}


/**
 * @brief 이상치 제거 후 평균 계산 (3σ 방식)
 */
static bool calculate_average_with_outlier_removal(void) {
  if (sample_count == 0) {
    return false;
  }

  // 1차 평균 계산

  double mean_lat = 0, mean_lon = 0, mean_alt = 0;

  for (uint32_t i = 0; i < sample_count; i++) {

    mean_lat += samples[i].lat;

    mean_lon += samples[i].lon;

    mean_alt += samples[i].alt;

  }

  mean_lat /= sample_count;

  mean_lon /= sample_count;

  mean_alt /= sample_count;



  // 표준편차 계산

  double var_lat = 0, var_lon = 0, var_alt = 0;

  for (uint32_t i = 0; i < sample_count; i++) {

    double diff_lat = samples[i].lat - mean_lat;

    double diff_lon = samples[i].lon - mean_lon;

    double diff_alt = samples[i].alt - mean_alt;

    var_lat += diff_lat * diff_lat;

    var_lon += diff_lon * diff_lon;

    var_alt += diff_alt * diff_alt;

  }

  double std_lat = sqrt(var_lat / sample_count);

  double std_lon = sqrt(var_lon / sample_count);

  double std_alt = sqrt(var_alt / sample_count);



  LOG_INFO("1차 평균: lat=%.9f, lon=%.9f, alt=%.3f", mean_lat, mean_lon, mean_alt);

  LOG_INFO("표준편차: lat=%.9f, lon=%.9f, alt=%.3f", std_lat, std_lon, std_alt);



  // 3σ 밖의 샘플 제외하고 재계산

  double final_lat = 0, final_lon = 0, final_alt = 0;

  uint32_t valid_count = 0;

  uint32_t rejected_count = 0;



  for (uint32_t i = 0; i < sample_count; i++) {

    double diff_lat = fabs(samples[i].lat - mean_lat);

    double diff_lon = fabs(samples[i].lon - mean_lon);

    double diff_alt = fabs(samples[i].alt - mean_alt);



    // 3σ 이내 샘플만 사용

    if (diff_lat < SIGMA_THRESHOLD * std_lat &&

        diff_lon < SIGMA_THRESHOLD * std_lon &&

        diff_alt < SIGMA_THRESHOLD * std_alt) {

      final_lat += samples[i].lat;

      final_lon += samples[i].lon;

      final_alt += samples[i].alt;

      valid_count++;

    } else {

      rejected_count++;

    }

  }



  if (valid_count < MIN_SAMPLES) {

    LOG_ERR("이상치 제거 후 유효 샘플 수 부족 (%lu < %d)", valid_count, MIN_SAMPLES);

    return false;

  }



  final_lat /= valid_count;

  final_lon /= valid_count;

  final_alt /= valid_count;



  // 결과 저장

  avg_result.lat = final_lat;

  avg_result.lon = final_lon;

  avg_result.alt = final_alt;

  avg_result.count = valid_count;

  avg_result.rejected = rejected_count;



  LOG_INFO("이상치 제거: %lu개 제거, %lu개 유효", rejected_count, valid_count);

  return true;

}


/**
 * @brief Base Fixed 모드로 전환
 *
 * 런타임에 GPS 역할과 타입을 확인하여 적절한 명령을 전송합니다.
 */
static bool switch_to_base_fixed_mode(void) {
  LOG_INFO("Base Fixed 모드로 전환 시작...");

  /* 역할 확인 */
  if (!gps_role_is_base()) {
    LOG_WARN("Base 역할이 아닙니다. 모드 전환 스킵");
    return false;
  }

  /* GPS 핸들 가져오기 */
  gps_t *gps_handle = gps_get_instance_handle(gps_id);
  if (gps_handle == NULL) {
    LOG_ERR("GPS 인스턴스 가져오기 실패");
    return false;
  }

  /* GPS 타입 확인 */
  const board_config_t *config = board_get_config();
  gps_type_t gps_type = config->gps[gps_id];

  LOG_INFO("평균 좌표:");
  LOG_INFO("  Lat: %.10f", avg_result.lat);
  LOG_INFO("  Lon: %.10f", avg_result.lon);
  LOG_INFO("  Alt: %.4f m", avg_result.alt);

  char buf[64];
  bool result = false;

  if (gps_type != GPS_TYPE_UM982) {
    LOG_ERR("지원하지 않는 GPS 타입: %d", gps_type);
    return false;
  }

  LOG_INFO("UM982 Base Fixed 모드 설정");

  char cmd[128];
  snprintf(cmd, sizeof(cmd), "MODE BASE %.10f %.10f %.4f",
           avg_result.lat, avg_result.lon, avg_result.alt);

  if (!gps_send_command_sync(gps_id, cmd, 1000)) {
    LOG_ERR("MODE BASE 명령 전송 실패");
    return false;
  }

  snprintf(buf, sizeof(buf), "Start Base (UM982)\n\r");
  ble_app_send(buf, strlen(buf));
  result = true;

  return result;

}



/**

 * @brief NTRIP 및 LTE 통신 종료

 */

static void shutdown_ntrip_and_lte(void) {

  LOG_INFO("NTRIP 및 LTE 통신 종료 시작...");



  // 1. NTRIP 종료

  ntrip_stop();

  LOG_INFO("NTRIP 종료 완료");



  // 2. EC25 Power Off (PWRKEY 핀)

  gsm_port_power_off();

  LOG_INFO("EC25 Power Off 완료");

}

/**
 * @brief Base Auto-Fix 워커 태스크 (블로킹 작업 처리)
 *
 * 타이머 콜백에서 블로킹 작업을 직접 수행하지 않고,
 * 이 워커 태스크가 큐를 통해 이벤트를 받아서 처리합니다.
 */
static void base_auto_fix_worker_task(void *pvParameter) {
  base_auto_fix_event_t event;

  LOG_INFO("Base Auto-Fix 워커 태스크 시작");
  worker_running = true;

  while (worker_running) {
    /* 큐에서 이벤트 대기 (타임아웃으로 종료 플래그 체크) */
    if (xQueueReceive(event_queue, &event, pdMS_TO_TICKS(100)) == pdTRUE) {

      /* 종료 이벤트 */
      if (event == BASE_AUTO_FIX_EVENT_SHUTDOWN) {
        LOG_INFO("워커 태스크 종료 이벤트 수신");
        break;
      }

      /* 평균 계산 완료 이벤트 */
      if (event == BASE_AUTO_FIX_EVENT_AVERAGING_COMPLETE) {
        LOG_INFO("평균 계산 완료 이벤트 수신");

        /* 샘플 수 확인 */
        if (sample_count < MIN_SAMPLES) {
          LOG_ERR("샘플 수 부족 (최소 %d개 필요, 현재 %lu개)", MIN_SAMPLES, sample_count);
          state = BASE_AUTO_FIX_FAILED;
          continue;
        }

        /* 평균 계산 (이상치 제거 포함) */
        if (!calculate_average_with_outlier_removal()) {
          LOG_ERR("평균 계산 실패");
          state = BASE_AUTO_FIX_FAILED;
          continue;
        }

        LOG_INFO("평균 좌표 계산 완료:");
        LOG_INFO("  Lat: %.9f (유효: %lu, 제거: %lu)",
                 avg_result.lat, avg_result.count, avg_result.rejected);
        LOG_INFO("  Lon: %.9f", avg_result.lon);
        LOG_INFO("  Alt: %.3f", avg_result.alt);

        /* Base Fixed 모드로 전환 */
        state = BASE_AUTO_FIX_SWITCHING;
        if (!switch_to_base_fixed_mode()) {
          LOG_ERR("Base Fixed 모드 전환 실패");
          state = BASE_AUTO_FIX_FAILED;
          continue;
        }

        xTimerStop(status_timer, 0);

        /* NTRIP/LTE 종료 (블로킹) */
        shutdown_ntrip_and_lte();

        led_set_color(1, LED_COLOR_NONE);
        led_set_state(1, false);

        state = BASE_AUTO_FIX_COMPLETED;
        LOG_INFO("Base Auto-Fix 완료!");
      }
    }
  }

  worker_running = false;
  LOG_INFO("Base Auto-Fix 워커 태스크 종료");
  vTaskDelete(NULL);
}

/*===========================================================================
 * 리소스 해제
 *===========================================================================*/

void base_auto_fix_deinit(void) {
  LOG_INFO("Base Auto-Fix 모듈 해제 시작");

  /* 1. 이벤트 구독 해제 */
  event_bus_unsubscribe(EVENT_GPS_FIX_CHANGED, on_gps_fix_changed);
  event_bus_unsubscribe(EVENT_GPS_GGA_UPDATE, on_gps_gga_update);
  event_bus_unsubscribe(EVENT_NTRIP_CONNECTED, on_ntrip_connected);

  /* 2. 동작 중지 */
  base_auto_fix_stop();

  /* 3. 워커 태스크 종료 */
  if (worker_task != NULL && worker_running) {
    base_auto_fix_event_t event = BASE_AUTO_FIX_EVENT_SHUTDOWN;
    xQueueSend(event_queue, &event, pdMS_TO_TICKS(100));

    /* 태스크 종료 대기 (최대 500ms) */
    uint32_t wait_count = 0;
    while (worker_running && wait_count < 50) {
      vTaskDelay(pdMS_TO_TICKS(10));
      wait_count++;
    }

    if (worker_running) {
      LOG_WARN("워커 태스크 강제 종료");
      vTaskDelete(worker_task);
      worker_running = false;
    }
    worker_task = NULL;
  }

  /* 4. 타이머 삭제 */
  if (averaging_timer != NULL) {
    xTimerDelete(averaging_timer, pdMS_TO_TICKS(100));
    averaging_timer = NULL;
  }

  if (status_timer != NULL) {
    xTimerDelete(status_timer, pdMS_TO_TICKS(100));
    status_timer = NULL;
  }

  /* 5. 이벤트 큐 삭제 */
  if (event_queue != NULL) {
    vQueueDelete(event_queue);
    event_queue = NULL;
  }

  /* 6. 상태 초기화 */
  state = BASE_AUTO_FIX_DISABLED;
  sample_count = 0;
  memset(&avg_result, 0, sizeof(avg_result));

  LOG_INFO("Base Auto-Fix 모듈 해제 완료");
}
