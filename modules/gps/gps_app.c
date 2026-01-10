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

#define GPS_UART_MAX_RECV_SIZE 2048

#define GGA_AVG_SIZE 1

// GPS App 이벤트 타입 (RX 태스크 → App 태스크 전달용)
typedef enum {
  GPS_APP_EVT_NONE = 0,
  GPS_APP_EVT_NMEA_GGA,
  GPS_APP_EVT_NMEA_RMC,
  GPS_APP_EVT_NMEA_THS,
  GPS_APP_EVT_UNICORE_RESPONSE,
  GPS_APP_EVT_UNICORE_BIN_BESTNAV,
  GPS_APP_EVT_RTCM,
} gps_app_evt_type_t;

typedef struct {
  gps_app_evt_type_t type;
  gps_id_t id;
  union {
    struct {
      gps_fix_t fix;
      bool gga_is_rdy;
      char gga_raw[100];
      uint8_t gga_raw_pos;
    } gga;
    struct {
      gps_unicore_resp_t response;
    } unicore_resp;
    struct {
      double lat;
      double lon;
      double height;
      double geoid;
    } bestnav;
    struct {
      uint16_t msg_type;
      uint8_t payload[GPS_PAYLOAD_SIZE];
      uint16_t payload_len;
    } rtcm;
  } data;
} gps_app_event_t;

static bool gps_init_um982_base_fixed_async_internal(gps_id_t id, double lat, double lon, double alt,

                                                      gps_init_callback_t callback, void *user_data);

static bool gps_init_um982_base_surveyin_async_internal(gps_id_t id, uint32_t time_sec, float accuracy_m,

                                                         gps_init_callback_t callback, void *user_data);


typedef struct {
  gps_t gps;

  // RX 태스크 (파싱만)
  QueueHandle_t rx_queue;      // UART IDLE 인터럽트 신호용
  TaskHandle_t rx_task;

  // App 태스크 (이벤트 핸들링)
  QueueHandle_t app_queue;     // 이벤트 전달용
  TaskHandle_t app_task;

  gps_type_t type;
  gps_id_t id;
  bool enabled;

  struct {
    double lat[GGA_AVG_SIZE];
    double lon[GGA_AVG_SIZE];
    double alt[GGA_AVG_SIZE];
    double lat_avg;
    double lon_avg;
    double alt_avg;
    uint8_t pos;
    uint8_t len;
    bool can_read;
  } gga_avg_data;

  // 동기식 명령어 전송용
  SemaphoreHandle_t tx_mutex;
  SemaphoreHandle_t response_sem;
  volatile bool waiting_response;
  volatile bool response_ok;

  gps_fix_t last_fix;
  uint8_t gga_ntrip_counter;
} gps_instance_t;

static gps_instance_t gps_instances[GPS_ID_MAX] = {0};

void _add_gga_avg_data(gps_instance_t *inst, double lat, double lon,
                       double alt) {
  uint8_t pos = inst->gga_avg_data.pos;
  double lat_temp = 0.0, lon_temp = 0.0, alt_temp = 0.0;

  inst->gga_avg_data.lat[pos] = lat;
  inst->gga_avg_data.lon[pos] = lon;
  inst->gga_avg_data.alt[pos] = alt;

  inst->gga_avg_data.pos = (inst->gga_avg_data.pos + 1) % GGA_AVG_SIZE;

  /* 정확도를 중시한 코드 */
  if (inst->gga_avg_data.len < GGA_AVG_SIZE) {
    inst->gga_avg_data.len++;

    if (inst->gga_avg_data.len == GGA_AVG_SIZE) {

      for (int i = 0; i < GGA_AVG_SIZE; i++) {
        lat_temp += inst->gga_avg_data.lat[i];
        lon_temp += inst->gga_avg_data.lon[i];
        alt_temp += inst->gga_avg_data.alt[i];
      }

      inst->gga_avg_data.lat_avg = lat_temp / (double)GGA_AVG_SIZE;
      inst->gga_avg_data.lon_avg = lon_temp / (double)GGA_AVG_SIZE;
      inst->gga_avg_data.alt_avg = alt_temp / (double)GGA_AVG_SIZE;

      inst->gga_avg_data.can_read = true;
    }
  } else {
    for (int i = 0; i < GGA_AVG_SIZE; i++) {
      lat_temp += inst->gga_avg_data.lat[i];
      lon_temp += inst->gga_avg_data.lon[i];
      alt_temp += inst->gga_avg_data.alt[i];
    }

    inst->gga_avg_data.lat_avg = lat_temp / (double)GGA_AVG_SIZE;
    inst->gga_avg_data.lon_avg = lon_temp / (double)GGA_AVG_SIZE;
    inst->gga_avg_data.alt_avg = alt_temp / (double)GGA_AVG_SIZE;
  }
}


#define GPS_INIT_MAX_RETRY 3
#define GPS_INIT_TIMEOUT_MS 1000

#define UM982_BASE_CMD_COUNT (sizeof(um982_base_cmds) / sizeof(um982_base_cmds[0]))

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

#define UM982_ROVER_CMD_COUNT (sizeof(um982_rover_cmds) / sizeof(um982_rover_cmds[0]))

typedef void (*gps_init_callback_t)(bool success, void *user_data);

typedef struct {
  gps_id_t gps_id;
  uint8_t current_step;
  uint8_t retry_count;
  const char **cmd_list;
  uint8_t cmd_count;
  gps_init_callback_t callback;
} gps_init_context_t;

/**
 * @brief UM982 Base 스테이션을 Fixed 모드로 설정 (비동기)
 */
static bool gps_init_um982_base_fixed_async_internal(gps_id_t id, double lat, double lon, double alt,
                                                      gps_init_callback_t callback, void *user_data) {
  if (id >= GPS_ID_MAX || !gps_instances[id].enabled) {
    LOG_ERR("GPS[%d] invalid or disabled", id);
    return false;
  }

  char buffer[128];

  snprintf(buffer, sizeof(buffer),
           "mode base %.10f %.10f %.4f\r\n",
           lat, lon, alt);

  LOG_INFO("GPS[%d] Setting fixed base station mode: lat=%.10f, lon=%.10f, alt=%.4f",
           id, lat, lon, alt);

  return gps_send_command_async(id, buffer, 1000, callback, user_data);
}

/**
 * @brief UM982 Base 스테이션을 Survey-in 모드로 설정 (비동기)
 */
static bool gps_init_um982_base_surveyin_async_internal(gps_id_t id, uint32_t time_sec, float accuracy_m,
                                                         gps_init_callback_t callback, void *user_data) {

  if (id >= GPS_ID_MAX || !gps_instances[id].enabled) {
    LOG_ERR("GPS[%d] invalid or disabled", id);
    return false;
  }

  char buffer[128];
  snprintf(buffer, sizeof(buffer),
           "MODE BASE TIME %u %.1f\r\n",
           time_sec, accuracy_m);

  LOG_INFO("GPS[%d] Setting survey-in base station mode: time=%us, accuracy=%.1fm",
           id, time_sec, accuracy_m);

  return gps_send_command_async(id, buffer, 1000, callback, user_data);
}


/**
 * @brief UM982 Base 스테이션 모드를 user_params 기반으로 설정 (비동기)
 */
bool gps_configure_um982_base_mode_async(gps_id_t id, gps_init_callback_t callback, void *user_data) {
  user_params_t* params = flash_params_get_current();

    // Fixed base station mode with manual position
    double lat = strtod(params->lat, NULL);
    double lon = strtod(params->lon, NULL);
    double alt = strtod(params->alt, NULL);

    return gps_init_um982_base_fixed_async_internal(id, lat, lon, alt, callback, user_data);

  // } else {
    // Survey-in mode (auto position)
    // return gps_init_um982_base_surveyin_async_internal(id, 120, 0.1f, callback, user_data);
  // }
}

static void baseline_init_complete(bool success, void *user_data) {
  gps_id_t id = (gps_id_t)(uintptr_t)user_data;
  LOG_INFO("GPS[%d] baseline mode init %s", id, success ? "succeeded" : "failed");
}


 void gps_set_heading_length()
{
  user_params_t* params = flash_params_get_current();
  gps_config_heading_length_async(0, params->baseline_len, params->baseline_len*0.2, baseline_init_complete, (void*)0);
}


#if defined(BOARD_TYPE_BASE_UNICORE) || defined(BOARD_TYPE_ROVER_UNICORE)

static void basestation_init_complete(bool success, void *user_data) {
  gps_id_t id = (gps_id_t)(uintptr_t)user_data;
  LOG_INFO("GPS[%d] base station mode init %s", id, success ? "succeeded" : "failed");

  if(success)
  {
    ble_send("Start Base manual\n\r", strlen("Start Base manual\n\r"), false);
  }
  else
  {
    ble_send("Base manual failed\n\r", strlen("Base manual failed\n\r"), false);
  }
}

static void overall_init_complete(bool success, void *user_data) {
  gps_id_t id = (gps_id_t)(uintptr_t)user_data;
  LOG_INFO("GPS[%d] Overall init %s", id, success ? "succeeded" : "failed");

  const board_config_t *config = board_get_config();
  user_params_t* params = flash_params_get_current();
  if(config->board == BOARD_TYPE_BASE_UM982)
  {
    if(params->use_manual_position)
    {
      gps_configure_um982_base_mode_async(id, basestation_init_complete, (void *)(uintptr_t)id);
    }
  }
  else if(config->board == BOARD_TYPE_ROVER_UM982)
  {
    gps_config_heading_length_async(0, params->baseline_len, params->baseline_len*0.2, baseline_init_complete, (void *)(uintptr_t)id);
  }
 }


#endif

static void gps_init_command_callback(bool success, void *user_data) {
  gps_init_context_t *ctx = (gps_init_context_t *)user_data;

  if (!ctx) {
    LOG_ERR("GPS init context is NULL");
    return;
  }

  if (success) {
    LOG_INFO("GPS[%d] Init step %d/%d OK: %s",
             ctx->gps_id, ctx->current_step + 1, ctx->cmd_count,
             ctx->cmd_list[ctx->current_step]);

    ctx->current_step++;
    ctx->retry_count = 0;

    if (ctx->current_step >= ctx->cmd_count) {
      LOG_INFO("GPS[%d] Init sequence complete!", ctx->gps_id);
      if (ctx->callback) {
        ctx->callback(true, NULL);
      }

      vPortFree(ctx);
      return;
    }


    gps_send_command_async(ctx->gps_id, ctx->cmd_list[ctx->current_step],
                           GPS_INIT_TIMEOUT_MS, gps_init_command_callback, ctx);
  } else {

    ctx->retry_count++;

    if (ctx->retry_count < GPS_INIT_MAX_RETRY) {

      LOG_WARN("GPS[%d] Init step %d/%d failed, retrying (%d/%d): %s",
               ctx->gps_id, ctx->current_step + 1, ctx->cmd_count,
               ctx->retry_count, GPS_INIT_MAX_RETRY,
               ctx->cmd_list[ctx->current_step]);

      
      gps_send_command_async(ctx->gps_id, ctx->cmd_list[ctx->current_step],
                             GPS_INIT_TIMEOUT_MS, gps_init_command_callback, ctx);
    } else {
      LOG_ERR("GPS[%d] Init failed at step %d/%d after %d retries: %s",
              ctx->gps_id, ctx->current_step + 1, ctx->cmd_count,
              GPS_INIT_MAX_RETRY, ctx->cmd_list[ctx->current_step]);

      if (ctx->callback) {
        ctx->callback(false, NULL);
      }
      
      vPortFree(ctx);
    }
  }
}

/**
 * @brief GPS UM982 Base 모드 초기화 (비동기)
 */
bool gps_init_um982_base_async(gps_id_t id, gps_init_callback_t callback) {
  if (id >= GPS_ID_MAX || !gps_instances[id].enabled) {
    LOG_ERR("GPS[%d] invalid or disabled", id);
    return false;
  }

  // 초기화 컨텍스트 생성 (동적 할당)
  gps_init_context_t *ctx = (gps_init_context_t *)pvPortMalloc(sizeof(gps_init_context_t));
  if (!ctx) {
    LOG_ERR("GPS[%d] failed to allocate init context", id);
    return false;
  }

  // 컨텍스트 초기화
  ctx->gps_id = id;
  ctx->current_step = 0;
  ctx->retry_count = 0;
  ctx->cmd_list = um982_base_cmds;
  ctx->cmd_count = UM982_BASE_CMD_COUNT;
  ctx->callback = callback;

  LOG_DEBUG("GPS[%d] Starting UM982 base init sequence (%d commands)",
           id, ctx->cmd_count);

  // 첫 번째 명령어 전송
  if (!gps_send_command_async(id, ctx->cmd_list[0], GPS_INIT_TIMEOUT_MS,
                               gps_init_command_callback, ctx)) {
    LOG_ERR("GPS[%d] failed to start init sequence", id);
    vPortFree(ctx);
    return false;
  }

  return true;
}

/**
 * @brief GPS UM982 Rover 모드 초기화 (비동기)
 */
bool gps_init_um982_rover_async(gps_id_t id, gps_init_callback_t callback) {
  if (id >= GPS_ID_MAX || !gps_instances[id].enabled) {
    LOG_ERR("GPS[%d] invalid or disabled", id);
    return false;
  }

  // 초기화 컨텍스트 생성 (동적 할당)
  gps_init_context_t *ctx = (gps_init_context_t *)pvPortMalloc(sizeof(gps_init_context_t));
  if (!ctx) {
    LOG_ERR("GPS[%d] failed to allocate init context", id);
    return false;
  }

  // 컨텍스트 초기화
  ctx->gps_id = id;
  ctx->current_step = 0;
  ctx->retry_count = 0;
  ctx->cmd_list = um982_rover_cmds;
  ctx->cmd_count = UM982_ROVER_CMD_COUNT;
  ctx->callback = callback;

  LOG_DEBUG("GPS[%d] Starting UM982 rover init sequence (%d commands)",
           id, ctx->cmd_count);

  if (!gps_send_command_async(id, ctx->cmd_list[0], GPS_INIT_TIMEOUT_MS,
                               gps_init_command_callback, ctx)) {
    LOG_ERR("GPS[%d] failed to start init sequence", id);
    vPortFree(ctx);
    return false;
  }

  return true;
}

/**
 * @brief GPS 이벤트 핸들러 (RX 태스크 컨텍스트에서 호출됨)
 *
 * 파싱 완료 이벤트를 app_queue로 전달
 * 동기식 명령 응답은 직접 처리 (즉시 응답 필요)
 */
void gps_evt_handler(gps_t *gps, gps_event_t event, gps_protocol_t protocol,
                     gps_msg_t msg) {
  gps_instance_t *inst = NULL;

  for (uint8_t i = 0; i < GPS_CNT; i++) {
    if (gps_instances[i].enabled && &gps_instances[i].gps == gps) {
      inst = &gps_instances[i];
      break;
    }
  }

  if (!inst || !inst->app_queue)
    return;

  gps_app_event_t app_evt = {0};
  app_evt.id = inst->id;

  switch (protocol) {
  case GPS_PROTOCOL_NMEA:
    if (msg.nmea == GPS_NMEA_MSG_GGA) {
      app_evt.type = GPS_APP_EVT_NMEA_GGA;
      app_evt.data.gga.fix = gps->nmea_data.gga.fix;
      app_evt.data.gga.gga_is_rdy = gps->nmea_data.gga_is_rdy;
      if (gps->nmea_data.gga_is_rdy && gps->nmea_data.gga_raw_pos < sizeof(app_evt.data.gga.gga_raw)) {
        memcpy(app_evt.data.gga.gga_raw, gps->nmea_data.gga_raw, gps->nmea_data.gga_raw_pos);
        app_evt.data.gga.gga_raw_pos = gps->nmea_data.gga_raw_pos;
      }
      xQueueSend(inst->app_queue, &app_evt, 0);
    }
    break;

  case GPS_PROTOCOL_UNICORE:
    // 동기식 명령어 응답은 즉시 처리 (세마포어 신호)
    if (inst->waiting_response) {
      gps_unicore_resp_t resp = gps_get_unicore_response(gps);
      if (resp == GPS_UNICORE_RESP_OK) {
        inst->response_ok = true;
        xSemaphoreGive(inst->response_sem);
      } else if (resp == GPS_UNICORE_RESP_ERROR || resp == GPS_UNICORE_RESP_UNKNOWN) {
        inst->response_ok = false;
        xSemaphoreGive(inst->response_sem);
      }
    }
    break;

  case GPS_PROTOCOL_UNICORE_BIN:
    if (msg.unicore_bin.msg == GPS_UNICORE_BIN_MSG_BESTNAV) {
      app_evt.type = GPS_APP_EVT_UNICORE_BIN_BESTNAV;
      app_evt.data.bestnav.lat = gps->unicore_bin_data.bestnav.lat;
      app_evt.data.bestnav.lon = gps->unicore_bin_data.bestnav.lon;
      app_evt.data.bestnav.height = gps->unicore_bin_data.bestnav.height;
      app_evt.data.bestnav.geoid = gps->unicore_bin_data.bestnav.geoid;
      xQueueSend(inst->app_queue, &app_evt, 0);
    }
    break;

  case GPS_PROTOCOL_RTCM:
    app_evt.type = GPS_APP_EVT_RTCM;
    app_evt.data.rtcm.msg_type = msg.rtcm.msg_type;
    // RTCM 페이로드 복사 (필요시)
    if (gps->pos < GPS_PAYLOAD_SIZE) {
      memcpy(app_evt.data.rtcm.payload, gps->payload, gps->pos);
      app_evt.data.rtcm.payload_len = gps->pos;
    }
    xQueueSend(inst->app_queue, &app_evt, 0);
    break;

  default:
    break;
  }
}




/**
 * @brief GPS RX 태스크 (파싱만 담당)
 *
 * UART IDLE 인터럽트 신호를 받아 DMA 버퍼에서 데이터를 읽고 파싱
 * 파싱 완료 시 이벤트 핸들러를 통해 app_queue로 이벤트 전달
 */
static void gps_rx_task(void *pvParameter) {
  gps_id_t id = (gps_id_t)(uintptr_t)pvParameter;
  gps_instance_t *inst = &gps_instances[id];

  size_t pos = 0;
  size_t old_pos = 0;
  uint8_t dummy = 0;

  gps_set_evt_handler(&inst->gps, gps_evt_handler);

  LOG_INFO("GPS[%d] RX 태스크 시작", id);

  while (1) {
    xQueueReceive(inst->rx_queue, &dummy, portMAX_DELAY);

    xSemaphoreTake(inst->gps.mutex, portMAX_DELAY);
    pos = gps_port_get_rx_pos(id);
    char *gps_recv = gps_port_get_recv_buf(id);

    if (pos != old_pos) {
      if (pos > old_pos) {
        size_t len = pos - old_pos;
        LOG_DEBUG("[%d] %d received", id, (int)len);
        LOG_DEBUG_RAW("RAW: ", &gps_recv[old_pos], len);
        gps_parse_process(&inst->gps, &gps_recv[old_pos], len);
      } else {
        // wrap around
        size_t len1 = GPS_UART_MAX_RECV_SIZE - old_pos;
        size_t len2 = pos;
        LOG_DEBUG("[%d] %d received (wrap around)", id, (int)(len1 + len2));
        LOG_DEBUG_RAW("RAW: ", &gps_recv[old_pos], len1);
        gps_parse_process(&inst->gps, &gps_recv[old_pos], len1);
        if (len2 > 0) {
          LOG_DEBUG_RAW("RAW: ", gps_recv, len2);
          gps_parse_process(&inst->gps, gps_recv, len2);
        }
      }
      old_pos = pos;
      if (old_pos == GPS_UART_MAX_RECV_SIZE) {
        old_pos = 0;
      }
    }
    xSemaphoreGive(inst->gps.mutex);
  }

  vTaskDelete(NULL);
}

/**
 * @brief GPS App 태스크 (이벤트 핸들링 담당)
 *
 * app_queue에서 이벤트를 받아 처리
 * - LED 상태 관리
 * - NTRIP GGA 전송
 * - Base Auto-Fix
 * - RTCM to LoRa
 * - 초기화 시퀀스
 */
static void gps_app_task(void *pvParameter) {
  gps_id_t id = (gps_id_t)(uintptr_t)pvParameter;
  gps_instance_t *inst = &gps_instances[id];
  gps_app_event_t evt;

  memset(&inst->gga_avg_data, 0, sizeof(inst->gga_avg_data));

  bool use_led = (id == GPS_ID_BASE ? 1 : 0);
  const board_config_t *config = board_get_config();

  if (use_led) {
    led_set_color(2, LED_COLOR_RED);
    led_set_state(2, true);
  }

  LOG_INFO("GPS[%d] App 태스크 시작", id);

  vTaskDelay(pdMS_TO_TICKS(500));

  // GPS 초기화 시퀀스 시작
#if defined(BOARD_TYPE_BASE_UNICORE)
  gps_init_um982_base_async(id, overall_init_complete);
#elif defined(BOARD_TYPE_ROVER_UNICORE)
  gps_init_um982_rover_async(id, overall_init_complete);
#endif

  while (1) {
    if (xQueueReceive(inst->app_queue, &evt, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    switch (evt.type) {
    case GPS_APP_EVT_NMEA_GGA:
      // LED 상태 관리
      if (use_led) {
        if (evt.data.gga.fix <= GPS_FIX_DGPS) {
          led_set_color(2, LED_COLOR_RED);
        } else if (evt.data.gga.fix == GPS_FIX_RTK_FLOAT) {
          led_set_color(2, LED_COLOR_YELLOW);
        } else if (evt.data.gga.fix == GPS_FIX_RTK_FIX) {
          if (config->board == BOARD_TYPE_ROVER_UM982) {
            led_set_color(2, LED_COLOR_GREEN);
          } else if (config->board == BOARD_TYPE_BASE_UM982) {
            led_set_color(2, LED_COLOR_YELLOW);
          }
        } else if (evt.data.gga.fix == GPS_FIX_MANUAL_POS) {
          led_set_color(2, LED_COLOR_GREEN);
        } else {
          led_set_color(2, LED_COLOR_NONE);
        }
        led_set_toggle(2);
      }

      // Base Auto-Fix: fix 변경 감지
      if (config->board == BOARD_TYPE_BASE_UM982) {
        if (evt.data.gga.fix != inst->last_fix) {
          base_auto_fix_on_gps_fix_changed(evt.data.gga.fix);
          inst->last_fix = evt.data.gga.fix;
        }
      }

      // NTRIP GGA 전송
      if (evt.data.gga.gga_is_rdy) {
        if (ntrip_gga_send_queue_initialized() && evt.data.gga.fix >= GPS_FIX_GPS) {
          ntrip_send_gga_data(evt.data.gga.gga_raw, evt.data.gga.gga_raw_pos);
        }
      }
      break;

    case GPS_APP_EVT_UNICORE_BIN_BESTNAV:
      // Base Auto-Fix: 위치 업데이트
      if (config->board == BOARD_TYPE_BASE_UM982) {
        if (inst->last_fix == GPS_FIX_RTK_FIX) {
          base_auto_fix_on_gga_update(evt.data.bestnav.lat,
                                       evt.data.bestnav.lon,
                                       evt.data.bestnav.height);
        }
      }
      break;

    case GPS_APP_EVT_RTCM:
      // RTCM to LoRa
      if (config->lora_mode == LORA_MODE_BASE) {
        if (inst->last_fix == GPS_FIX_MANUAL_POS) {
          // RTCM 전송 시 gps 핸들 필요
          rtcm_send_to_lora(&inst->gps);
        }
      }
      break;

    default:
      break;
    }
  }

  vTaskDelete(NULL);
}

void gps_init_all(void) {
  const board_config_t *config = board_get_config();

  for (uint8_t i = 0; i < config->gps_cnt && i < GPS_ID_MAX; i++) {
    gps_type_t type = config->gps[i];

    LOG_DEBUG("GPS[%d] 초기화 - 타입: %s", i,
             type == GPS_TYPE_UM982 ? "UM982" : "UNKNOWN");

    if (gps_instances[i].enabled) {
      LOG_WARN("GPS[%d] 이미 초기화됨, 스킵", i);
      continue;
    }      
                               
    gps_init(&gps_instances[i].gps);
    gps_instances[i].type = type;
    gps_instances[i].id = (gps_id_t)i;
    gps_instances[i].enabled = true;

    if (gps_port_init_instance(&gps_instances[i].gps, (gps_id_t)i, type) != 0) {
      LOG_ERR("GPS[%d] 포트 초기화 실패", i);
      gps_instances[i].enabled = false;
      continue;
    }

    // RX 큐 생성 (UART IDLE 인터럽트 신호용)
    gps_instances[i].rx_queue = xQueueCreate(10, sizeof(uint8_t));
    if (gps_instances[i].rx_queue == NULL) {
      LOG_ERR("GPS[%d] RX 큐 생성 실패", i);
      gps_instances[i].enabled = false;
      continue;
    }

    gps_port_set_queue((gps_id_t)i, gps_instances[i].rx_queue);

    // App 큐 생성 (이벤트 전달용)
    gps_instances[i].app_queue = xQueueCreate(10, sizeof(gps_app_event_t));
    if (gps_instances[i].app_queue == NULL) {
      LOG_ERR("GPS[%d] App 큐 생성 실패", i);
      vQueueDelete(gps_instances[i].rx_queue);
      gps_instances[i].enabled = false;
      continue;
    }

    // 동기식 명령어 전송용 세마포어 초기화
    gps_instances[i].tx_mutex = xSemaphoreCreateMutex();
    if (gps_instances[i].tx_mutex == NULL) {
      LOG_ERR("GPS[%d] TX 뮤텍스 생성 실패", i);
      gps_instances[i].enabled = false;
      continue;
    }

    gps_instances[i].response_sem = xSemaphoreCreateBinary();
    if (gps_instances[i].response_sem == NULL) {
      LOG_ERR("GPS[%d] 응답 세마포어 생성 실패", i);
      vSemaphoreDelete(gps_instances[i].tx_mutex);
      gps_instances[i].enabled = false;
      continue;
    }

    gps_instances[i].waiting_response = false;
    gps_instances[i].response_ok = false;

    gps_port_start(&gps_instances[i].gps);

    // RX 태스크 생성 (파싱만)
    char task_name[16];
    snprintf(task_name, sizeof(task_name), "gps_rx_%d", i);

    BaseType_t ret =
        xTaskCreate(gps_rx_task, task_name, 1024,
                    (void *)(uintptr_t)i,
                    tskIDLE_PRIORITY + 2, &gps_instances[i].rx_task);

    if (ret != pdPASS) {
      LOG_ERR("GPS[%d] RX 태스크 생성 실패", i);
      gps_instances[i].enabled = false;
      continue;
    }

    // App 태스크 생성 (이벤트 핸들링)
    snprintf(task_name, sizeof(task_name), "gps_app_%d", i);

    ret = xTaskCreate(gps_app_task, task_name, 1024,
                      (void *)(uintptr_t)i,
                      tskIDLE_PRIORITY + 1, &gps_instances[i].app_task);

    if (ret != pdPASS) {
      LOG_ERR("GPS[%d] App 태스크 생성 실패", i);
      vTaskDelete(gps_instances[i].rx_task);
      gps_instances[i].enabled = false;
      continue;
    }

    LOG_INFO("GPS[%d] 인스턴스 초기화 완료 (RX + App 태스크)", i);

  }

  LOG_INFO("GPS 전체 인스턴스 초기화 완료");
  if (config->board == BOARD_TYPE_BASE_UM982) {
    user_params_t *params = flash_params_get_current();
    if (params->base_auto_fix_enabled) {
      LOG_INFO("Base Auto-Fix 모드 활성화");

      if (base_auto_fix_init(GPS_ID_BASE)) {
        if (base_auto_fix_start()) {
          LOG_INFO("Base Auto-Fix 시작 성공");
        } else {
          LOG_ERR("Base Auto-Fix 시작 실패");
        }
      } else {
        LOG_ERR("Base Auto-Fix 초기화 실패");
      }
    } else {
      LOG_INFO("Base Auto-Fix 모드 비활성화 (파라미터 설정)");
    }
  }
}

/**
 * @brief 특정 GPS ID의 핸들 가져오기
 */
gps_t *gps_get_instance_handle(gps_id_t id) {
  if (id >= GPS_ID_MAX || !gps_instances[id].enabled) {
    return NULL;
  }

  return &gps_instances[id].gps;
}

/**
 * @brief GGA 평균 데이터 읽기 가능 여부
 */
bool gps_gga_avg_can_read(gps_id_t id) {
  if (id >= GPS_ID_MAX || !gps_instances[id].enabled) {
    return false;
  }

  return gps_instances[id].gga_avg_data.can_read;
}

/**
 * @brief GGA 평균 데이터 가져오기
 */
bool gps_get_gga_avg(gps_id_t id, double *lat, double *lon, double *alt) {
  if (id >= GPS_ID_MAX || !gps_instances[id].enabled) {
    return false;
  }

  if (!gps_instances[id].gga_avg_data.can_read) {
    return false;
  }

  if (lat)
    *lat = gps_instances[id].gga_avg_data.lat_avg;
  if (lon)
    *lon = gps_instances[id].gga_avg_data.lon_avg;
  if (alt)
    *alt = gps_instances[id].gga_avg_data.alt_avg;

  return true;
}

/**
 * @brief 동기식 GPS 명령어 전송
 *
 * 직접 UART로 전송하고 응답을 기다림 (TX 태스크 없음)
 */
bool gps_send_command_sync(gps_id_t id, const char *cmd, uint32_t timeout_ms) {
  if (id >= GPS_ID_MAX || !gps_instances[id].enabled) {
    LOG_ERR("GPS[%d] invalid or disabled", id);
    return false;
  }

  if (!cmd || strlen(cmd) == 0) {
    LOG_ERR("GPS[%d] empty command", id);
    return false;
  }

  gps_instance_t *inst = &gps_instances[id];

  // TX 뮤텍스 획득 (동시 전송 방지)
  if (xSemaphoreTake(inst->tx_mutex, pdMS_TO_TICKS(5000)) != pdTRUE) {
    LOG_ERR("GPS[%d] tx_mutex timeout", id);
    return false;
  }

  // 응답 대기 플래그 설정
  inst->waiting_response = true;
  inst->response_ok = false;

  LOG_INFO("GPS[%d] Sending command: %s", id, cmd);

  // 직접 UART 전송
  if (inst->gps.ops && inst->gps.ops->send) {
    inst->gps.ops->send(cmd, strlen(cmd));
  } else {
    LOG_ERR("GPS[%d] send ops not available", id);
    inst->waiting_response = false;
    xSemaphoreGive(inst->tx_mutex);
    return false;
  }

  // 응답 대기 (타임아웃 적용)
  bool result = false;
  if (xSemaphoreTake(inst->response_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
    result = inst->response_ok;
    LOG_INFO("GPS[%d] Response: %s", id, result ? "OK" : "ERROR");
  } else {
    LOG_WARN("GPS[%d] Command timeout", id);
    result = false;
  }

  inst->waiting_response = false;
  xSemaphoreGive(inst->tx_mutex);

  return result;
}

/**
 * @brief 비동기 GPS 명령어 전송 (호환성을 위해 유지)
 *
 * 내부적으로 동기식으로 실행하고 콜백 호출
 */
bool gps_send_command_async(gps_id_t id, const char *cmd, uint32_t timeout_ms,
                             gps_command_callback_t callback, void *user_data) {
  bool result = gps_send_command_sync(id, cmd, timeout_ms);

  if (callback) {
    callback(result, user_data);
  }

  return true;
}

bool gps_send_raw_data(gps_id_t id, const uint8_t *data, size_t len) {
  if (id >= GPS_ID_MAX || !gps_instances[id].enabled) {
    LOG_ERR("GPS[%d] invalid", id);
    return false;
  }

  if (!data || len == 0) {
    LOG_ERR("GPS[%d] invalid data", id);
    return false;
  }

  gps_instance_t *inst = &gps_instances[id];

  if (!inst->gps.ops || !inst->gps.ops->send) {
    LOG_ERR("GPS[%d] ops invalid", id);
    return false;
  }

  if (xSemaphoreTake(inst->gps.mutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
    inst->gps.ops->send((const char *)data, len);
    xSemaphoreGive(inst->gps.mutex);
    LOG_DEBUG("GPS[%d] 송신 %d 바이트", id, len);
    return true;
  } else {
    LOG_ERR("GPS[%d] 뮤텍스 타임아웃", id);
    return false;
  }
}





// double convert_dms(double value)
// {
//   double temp = value;
//   double deg = (temp/(double)100);
//   double min = temp - (deg * (double)100);
//   return (deg + (min / (double)60));
// }
//   val = gps_parse_double(gps);
//   deg = (double)((int)(((int)val / 100)));
//   min = val - (deg * (double)100);
//   val = deg + (min / (double)(60));

static double convert_dm(double lat_dd)
{
    int lat_deg = (int)fabs(lat_dd);
    double lat_min = (fabs(lat_dd) - lat_deg) * 60.0;
    double lat_ddm = lat_deg * 100.0 + lat_min;

    return lat_ddm;
}

/**
 * @brief GPS 위치 데이터 포맷팅
 *
 * 포맷: +GPS,lat,N/S,lon,E/W,msl_alt,ellipsoid_alt,heading,fix\r\n
 *
 * GPS 타입별 데이터 소스:
 * - Unicore UM982: BESTNAV (lat, lon, height, geoid, trk_gnd)
 */
bool gps_format_position_data(char *buffer)
{
  gps_instance_t *inst = &gps_instances[0];
  double lat, lon, geoid, ellipsoid_alt, heading;
  char ns = 'N', ew = 'E';
  int fix;
  int sat_num;

  char ns_str[2] = {ns, '\0'};
  char ew_str[2] = {ew, '\0'};

  taskENTER_CRITICAL();
  lat = convert_dm(inst->gps.unicore_bin_data.bestnav.lat);
  lon = convert_dm(inst->gps.unicore_bin_data.bestnav.lon);
  heading = inst->gps.nmea_data.ths.heading;
  ns = inst->gps.nmea_data.gga.ns;
  ew = inst->gps.nmea_data.gga.ew;
  fix = inst->gps.nmea_data.gga.fix;
  if(fix == 0)
  {
    ellipsoid_alt = 0;
    geoid = 0;
  }
  else
  {
    ellipsoid_alt = inst->gps.unicore_bin_data.bestnav.height;
    geoid = inst->gps.unicore_bin_data.bestnav.geoid;
  }
  sat_num = inst->gps.nmea_data.gga.sat_num;
  taskEXIT_CRITICAL();

  sprintf(buffer,
          "+GPS,%.9lf,%s,%.9lf,%s,%.4lf,%.4lf,%.5lf,%d,%d\n\r",
          lat, ns_str, lon, ew_str,
          ellipsoid_alt,
          geoid,
          heading,
          fix,
          sat_num);

  return true;
}


typedef struct {

  gps_id_t gps_id;

  uint8_t current_step;  // 0: config heading length, 1: CONFIG HEADING FIXLENGTH

  gps_command_callback_t user_callback;

  void *user_data;

  char cmd_buffer[128];

} gps_config_heading_context_t;

 

static void gps_config_heading_callback(bool success, void *user_data)

{

  gps_config_heading_context_t *ctx = (gps_config_heading_context_t *)user_data;

 

  if (!success) {

    LOG_ERR("GPS[%d] Heading config step %d failed", ctx->gps_id, ctx->current_step);

 

    // 실패 시 사용자 콜백 호출 후 컨텍스트 해제

    if (ctx->user_callback) {

      ctx->user_callback(false, ctx->user_data);

    }

    vPortFree(ctx);

    return;

  }

 

  LOG_INFO("GPS[%d] Heading config step %d OK", ctx->gps_id, ctx->current_step);

 

  ctx->current_step++;

 

  if (ctx->current_step == 1) {

    // 두 번째 명령어: CONFIG HEADING FIXLENGTH

    gps_send_command_async(ctx->gps_id, "CONFIG HEADING FIXLENGTH\r\n",

                          3000, gps_config_heading_callback, ctx);

  } else {

    // 모든 명령어 완료

    LOG_INFO("GPS[%d] Heading config complete!", ctx->gps_id);

 

    if (ctx->user_callback) {

      ctx->user_callback(true, ctx->user_data);

    }

    vPortFree(ctx);

  }

}

 

bool gps_config_heading_length_async(gps_id_t id, float baseline_len, float slave_distance,

                                     gps_command_callback_t callback, void *user_data)

{

  if (id >= GPS_ID_MAX || !gps_instances[id].enabled) {

    LOG_ERR("GPS[%d] invalid", id);

    return false;

  }

 

  gps_instance_t *inst = &gps_instances[id];

 

 

 

  // UM982만 지원

 

  if (inst->type != GPS_TYPE_UM982) {

 

    LOG_ERR("GPS[%d] Only UM982 supports heading config", id);

 

    return false;

 

  }

 

 

 

  // 컨텍스트 동적 할당

 

  gps_config_heading_context_t *ctx = pvPortMalloc(sizeof(gps_config_heading_context_t));

 

  if (!ctx) {

 

    LOG_ERR("GPS[%d] Failed to allocate heading config context", id);

 

    return false;

 

  }

 

 

 

  // 컨텍스트 초기화

 

  ctx->gps_id = id;

 

  ctx->current_step = 0;

 

  ctx->user_callback = callback;

 

  ctx->user_data = user_data;

 

 

 

  // 첫 번째 명령어: config heading length [baseline] [slave_distance]

 

  snprintf(ctx->cmd_buffer, sizeof(ctx->cmd_buffer),

 

           "config heading length %.4f %.4f\r\n", baseline_len, slave_distance);

 

 

 

  LOG_INFO("GPS[%d] Starting heading config: %s", id, ctx->cmd_buffer);

 

 

 

  // 첫 번째 명령어 전송

 

  if (!gps_send_command_async(id, ctx->cmd_buffer, 3000, gps_config_heading_callback, ctx)) {

 

    LOG_ERR("GPS[%d] Failed to send heading length command", id);

 

    vPortFree(ctx);

 

    return false;

 

  }

 

 

 

  return true;

 

}

 

/**

 * @brief 특정 GPS 인스턴스 정리 및 태스크 삭제

 */

bool gps_cleanup_instance(gps_id_t id)

{

  if (id >= GPS_ID_MAX) {

    LOG_ERR("GPS[%d] invalid ID", id);

    return false;

  }

 

  gps_instance_t *inst = &gps_instances[id];

 

  if (!inst->enabled) {

    LOG_WARN("GPS[%d] already disabled", id);

    return true;

  }

 

  LOG_INFO("GPS[%d] 정리 시작", id);

 

  bool use_led = (id == GPS_ID_BASE ? 1 : 0);

  if (use_led) {

    led_set_color(2, LED_COLOR_NONE);

    led_set_state(2, false);

  }

 

  gps_port_cleanup_instance(id);

  // RX 태스크 삭제
  if (inst->rx_task != NULL) {
    vTaskDelete(inst->rx_task);
    inst->rx_task = NULL;
    LOG_INFO("GPS[%d] RX 태스크 삭제 요청", id);
  }

  // App 태스크 삭제
  if (inst->app_task != NULL) {
    vTaskDelete(inst->app_task);
    inst->app_task = NULL;
    LOG_INFO("GPS[%d] App 태스크 삭제 요청", id);
  }

  vTaskDelay(pdMS_TO_TICKS(50));
  LOG_INFO("GPS[%d] 태스크 정리 대기 완료", id);

  // RX 큐 삭제
  if (inst->rx_queue != NULL) {
    vQueueDelete(inst->rx_queue);
    inst->rx_queue = NULL;
    LOG_INFO("GPS[%d] RX 큐 삭제 완료", id);
  }

  // App 큐 삭제
  if (inst->app_queue != NULL) {
    vQueueDelete(inst->app_queue);
    inst->app_queue = NULL;
    LOG_INFO("GPS[%d] App 큐 삭제 완료", id);
  }

  // 동기식 명령어 전송용 세마포어 정리
  if (inst->tx_mutex != NULL) {
    vSemaphoreDelete(inst->tx_mutex);
    inst->tx_mutex = NULL;
    LOG_INFO("GPS[%d] TX 뮤텍스 삭제 완료", id);
  }

  if (inst->response_sem != NULL) {
    vSemaphoreDelete(inst->response_sem);
    inst->response_sem = NULL;
    LOG_INFO("GPS[%d] 응답 세마포어 삭제 완료", id);
  }

  if (inst->gps.mutex != NULL) {
    vSemaphoreDelete(inst->gps.mutex);
    inst->gps.mutex = NULL;
    LOG_INFO("GPS[%d] GPS 뮤텍스 삭제 완료", id);
  }

 

  memset(inst, 0, sizeof(gps_instance_t));

  inst->enabled = false;

 

  LOG_INFO("GPS[%d] 정리 완료", id);

 

  return true;

}

 

/**

 * @brief 모든 GPS 인스턴스 정리 및 태스크 삭제

 */

void gps_cleanup_all(void)

{

  const board_config_t *config = board_get_config();

 

  LOG_INFO("모든 GPS 인스턴스 정리 시작");

 

  for (uint8_t i = 0; i < config->gps_cnt && i < GPS_ID_MAX; i++) {

    if (gps_instances[i].enabled) {

      gps_cleanup_instance((gps_id_t)i);

    }

  }

 

 vTaskDelay(pdMS_TO_TICKS(200));

  LOG_INFO("모든 GPS 인스턴스 정리 완료 (IDLE 태스크 정리 대기 완료)");

}
