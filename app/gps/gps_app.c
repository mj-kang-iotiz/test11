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

#if 0
#define GPS_UART_MAX_RECV_SIZE 2048

#define GGA_AVG_SIZE 1
#define HP_AVG_SIZE 1

typedef struct {
  gps_t gps;
  QueueHandle_t queue;
  TaskHandle_t task;
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

  QueueHandle_t cmd_queue;
  TaskHandle_t tx_task;
  gps_cmd_request_t *current_cmd_req;

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

void _add_hp_avg_data(gps_instance_t *inst) {
  gps_t *gps = &inst->gps;
  uint8_t pos = inst->ubx_hp_avg.pos;
  gps_ubx_nav_hpposllh_t *data = &gps->ubx_data.hpposllh;
  ubx_hp_avg_data_t *avg_data = &inst->ubx_hp_avg;

  int64_t lat_sum = 0, lon_sum = 0, height_sum = 0, msl_sum = 0;
  int16_t lat_hp_sum = 0, lon_hp_sum = 0, height_hp_sum = 0, msl_hp_sum = 0;

  avg_data->lon[pos] = data->lon;
  avg_data->lat[pos] = data->lat;
  avg_data->height[pos] = data->height;
  avg_data->msl[pos] = data->msl;
  avg_data->lon_hp[pos] = data->lon_hp;
  avg_data->lat_hp[pos] = data->lat_hp;
  avg_data->height_hp[pos] = data->height_hp;
  avg_data->msl_hp[pos] = data->msl_hp;
  avg_data->hacc = data->hacc;
  avg_data->vacc = data->vacc;

  avg_data->pos = (avg_data->pos + 1) % HP_AVG_SIZE;

  if (avg_data->len < HP_AVG_SIZE) {
    avg_data->len++;

    if (avg_data->len == HP_AVG_SIZE) {
      for (int i = 0; i < HP_AVG_SIZE; i++) {
        lon_sum += avg_data->lon[i];
        lat_sum += avg_data->lat[i];
        height_sum += avg_data->height[i];
        msl_sum += avg_data->msl[i];
        lon_hp_sum += avg_data->lon_hp[i];
        lat_hp_sum += avg_data->lat_hp[i];
        height_hp_sum += avg_data->height_hp[i];
        msl_hp_sum += avg_data->msl_hp[i];
      }

      avg_data->lon_avg = (lon_sum / (double)HP_AVG_SIZE) +
                          (lon_hp_sum / (double)HP_AVG_SIZE / (double)100);
      avg_data->lat_avg = (lat_sum / (double)HP_AVG_SIZE) +
                          (lat_hp_sum / (double)HP_AVG_SIZE / (double)100);
      avg_data->height_avg = (height_sum / (double)HP_AVG_SIZE) +
                             (height_hp_sum / (double)HP_AVG_SIZE / (double)10);
      avg_data->msl_avg = (msl_sum / (double)HP_AVG_SIZE) +
                          (msl_hp_sum / (double)HP_AVG_SIZE / (double)10);

      avg_data->can_read = true;
    }
  } else if (avg_data->len == HP_AVG_SIZE) {
    for (int i = 0; i < HP_AVG_SIZE; i++) {
      lon_sum += avg_data->lon[i];
      lat_sum += avg_data->lat[i];
      height_sum += avg_data->height[i];
      msl_sum += avg_data->msl[i];
      lon_hp_sum += avg_data->lon_hp[i];
      lat_hp_sum += avg_data->lat_hp[i];
      height_hp_sum += avg_data->height_hp[i];
      msl_hp_sum += avg_data->msl_hp[i];
    }

    avg_data->lon_avg = (lon_sum / (double)HP_AVG_SIZE) +
                        (lon_hp_sum / (double)HP_AVG_SIZE / (double)100);
    avg_data->lat_avg = (lat_sum / (double)HP_AVG_SIZE) +
                        (lat_hp_sum / (double)HP_AVG_SIZE / (double)100);
    avg_data->height_avg = (height_sum / (double)HP_AVG_SIZE) +
                           (height_hp_sum / (double)HP_AVG_SIZE / (double)10);
    avg_data->msl_avg = (msl_sum / (double)HP_AVG_SIZE) +
                        (msl_hp_sum / (double)HP_AVG_SIZE / (double)10);
  } else {
    LOG_ERR("HP AVG LEN mismatch");
  }
}

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


void gps_evt_handler(gps_t *gps, gps_event_t event, gps_protocol_t protocol,
                     gps_msg_t msg) {
  gps_instance_t *inst = NULL;
  const board_config_t *config = board_get_config();

  for (uint8_t i = 0; i < GPS_CNT; i++) {
    if (gps_instances[i].enabled && &gps_instances[i].gps == gps) {
      inst = &gps_instances[i];
      break;
    }
  }

  if (!inst)
    return;

  switch (protocol) {
  case GPS_PROTOCOL_NMEA:
    if (msg.nmea == GPS_NMEA_MSG_GGA) {
    	if(config->board == BOARD_TYPE_BASE_F9P || config->board == BOARD_TYPE_BASE_UM982)
    	{
          if (gps->nmea_data.gga.fix != inst->last_fix) {
    	        base_auto_fix_on_gps_fix_changed(gps->nmea_data.gga.fix);
    	        inst->last_fix = gps->nmea_data.gga.fix;
    	    }
      }

      if (gps->nmea_data.gga_is_rdy)
      {
        if(config->board == BOARD_TYPE_ROVER_F9P)
        {
          if(inst->id == GPS_ID_BASE)
          {
                  if(ntrip_gga_send_queue_initialized() && gps->nmea_data.gga.fix >= GPS_FIX_GPS)
                  {
                    ntrip_send_gga_data(gps->nmea_data.gga_raw,
                                     gps->nmea_data.gga_raw_pos);
                  }

          }
        }
        else
        {
            if(ntrip_gga_send_queue_initialized() && gps->nmea_data.gga.fix >= GPS_FIX_GPS)
            {
                ntrip_send_gga_data(gps->nmea_data.gga_raw,
                                    gps->nmea_data.gga_raw_pos);
            }
        }
      }
    }
    break;

  case GPS_PROTOCOL_UBX:
    if (msg.ubx.id == GPS_UBX_NAV_ID_HPPOSLLH) {
      if(config->board == BOARD_TYPE_BASE_F9P)
      {
        if (gps->nmea_data.gga.fix == GPS_FIX_RTK_FIX) {
          // _add_hp_avg_data(inst);
          double lat = gps->ubx_data.hpposllh.lat * 1e-7 + gps->ubx_data.hpposllh.lat_hp * 1e-9;
          double lon = gps->ubx_data.hpposllh.lon * 1e-7 + gps->ubx_data.hpposllh.lon_hp * 1e-9;
          double alt = (gps->ubx_data.hpposllh.height + gps->ubx_data.hpposllh.height * 1e-2)/(double)1000.0;
          base_auto_fix_on_gga_update(lat, lon, alt);
        }
      }
    }

  case GPS_PROTOCOL_UNICORE:
    if (inst->current_cmd_req != NULL) {
      gps_unicore_resp_t resp = gps_get_unicore_response(gps);
      if (resp == GPS_UNICORE_RESP_OK) {
        if (inst->current_cmd_req->is_async) {
          inst->current_cmd_req->async_result = true;
        } else {
          *(inst->current_cmd_req->result) = true;
        }
        xSemaphoreGive(inst->current_cmd_req->response_sem);
      } else if (resp == GPS_UNICORE_RESP_ERROR || resp == GPS_UNICORE_RESP_UNKNOWN) {
        if (inst->current_cmd_req->is_async) {
          inst->current_cmd_req->async_result = false;
        } else {
          *(inst->current_cmd_req->result) = false;
        }
        xSemaphoreGive(inst->current_cmd_req->response_sem);
      }
    }

    break;

case GPS_PROTOCOL_UNICORE_BIN:
    switch (msg.unicore_bin.msg) {
      case GPS_UNICORE_BIN_MSG_BESTNAV: {
        if(config->board == BOARD_TYPE_BASE_UM982)
        {
          if (gps->nmea_data.gga.fix == GPS_FIX_RTK_FIX)
          {
            hpd_unicore_bestnavb_t *bestnav = &gps->unicore_bin_data.bestnav;
            base_auto_fix_on_gga_update(bestnav->lat, bestnav->lon, bestnav->height);
          }
        }
      }
    }
    break;
  case GPS_PROTOCOL_RTCM:
    if(config->lora_mode == LORA_MODE_BASE)
    {
      if(gps->nmea_data.gga.fix == GPS_FIX_MANUAL_POS)
      {
        rtcm_send_to_lora(gps);
      }
      else if(config->board == BOARD_TYPE_BASE_F9P && gps->nmea_data.gga.hdop>=99.0)
      {
        rtcm_send_to_lora(gps);
      }
    }
    break;

  default:
    break;
  }
}

static void gps_tx_task(void *pvParameter) {
  gps_id_t id = (gps_id_t)(uintptr_t)pvParameter;
  gps_instance_t *inst = &gps_instances[id];
  gps_cmd_request_t cmd_req;

  LOG_INFO("GPS TX 태스크[%d] 시작", id);

  while (1) {
    if (xQueueReceive(inst->cmd_queue, &cmd_req, portMAX_DELAY) == pdTRUE) {
      LOG_INFO("GPS[%d] Sending command: %s", id, cmd_req.cmd);

      // 현재 명령어 요청 저장 (RX Task에서 응답 처리용)
      inst->current_cmd_req = &cmd_req;
      if (cmd_req.is_async) {
        cmd_req.async_result = false;
      } else {
        *(cmd_req.result) = false;
      }

      // 명령어 전송
      if (inst->gps.ops && inst->gps.ops->send) {
        xSemaphoreTake(inst->gps.mutex, pdMS_TO_TICKS(1000));
        inst->gps.ops->send(cmd_req.cmd, strlen(cmd_req.cmd));
        xSemaphoreGive(inst->gps.mutex);
      } else {
        LOG_ERR("GPS[%d] send ops not available", id);
        inst->current_cmd_req = NULL;
        if (cmd_req.is_async) {
          cmd_req.async_result = false;
          if (cmd_req.callback) {
            cmd_req.callback(false, cmd_req.user_data);
          }
          vSemaphoreDelete(cmd_req.response_sem);
        } else {
          *(cmd_req.result) = false;
          xSemaphoreGive(cmd_req.response_sem);
        }
        inst->current_cmd_req = NULL;
        continue;
      }

      // 응답 대기 (타임아웃 적용)
      if (xSemaphoreTake(cmd_req.response_sem, pdMS_TO_TICKS(cmd_req.timeout_ms)) == pdTRUE) {
        // 응답 수신 완료 (RX Task가 세마포어를 줌)
        if (cmd_req.is_async) {
          LOG_INFO("GPS[%d] Response received: %s", id,
                   cmd_req.async_result ? "OK" : "ERROR");
        } else {
          LOG_INFO("GPS[%d] Response received: %s", id,
                   *(cmd_req.result) ? "OK" : "ERROR");
        }
      } else {
        LOG_WARN("GPS[%d] Command timeout", id);
        if (cmd_req.is_async) {
          cmd_req.async_result = false;
        } else {
          *(cmd_req.result) = false;
        }
      }

      // 현재 명령어 요청 초기화
      inst->current_cmd_req = NULL;

      // 비동기: 콜백 호출
      if (cmd_req.is_async) {
        if (cmd_req.callback) {
          cmd_req.callback(cmd_req.async_result, cmd_req.user_data);
        }

        // 비동기는 세마포어를 TX Task에서 삭제
        vSemaphoreDelete(cmd_req.response_sem);
      } else {
        // 동기: 외부 호출자에게 처리 완료 알림 (세마포어 반환)
        xSemaphoreGive(cmd_req.response_sem);
      }
    }
  }
  vTaskDelete(NULL);
}
#endif

static void gps_process_task(void *pvParameter) {
  size_t pos = 0;
  size_t old_pos = 0;
  uint8_t dummy = 0;
  size_t total_received = 0;
  
  while (1) {
    xQueueReceive(inst->queue, &dummy,
                  portMAX_DELAY);

    if (pos != old_pos) {
      if (pos > old_pos) {
        size_t len = pos - old_pos;
        total_received = len;
        LOG_DEBUG("[%d] %d received", id, (int)len);
        LOG_DEBUG_RAW("RAW: ", &gps_recv[old_pos], len);
        gps_parse_process(&inst->gps, &gps_recv[old_pos], pos - old_pos);
      } else {
        size_t len1 = GPS_UART_MAX_RECV_SIZE - old_pos;
        size_t len2 = pos;
        total_received = len1 + len2;
        LOG_DEBUG("[%d] %d received (wrap around)", id, (int)(len1 + len2));
        LOG_DEBUG_RAW("RAW: ", &gps_recv[old_pos], len1);
        gps_parse_process(&inst->gps, &gps_recv[old_pos],
                          GPS_UART_MAX_RECV_SIZE - old_pos);
        if (pos > 0) {
          LOG_DEBUG_RAW("RAW: ", gps_recv, len2);
          gps_parse_process(&inst->gps, gps_recv, pos);
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

void gps_init_all(void) {
  const board_config_t *config = board_get_config();

  for (uint8_t i = 0; i < config->gps_cnt && i < GPS_ID_MAX; i++) {
    gps_type_t type = config->gps[i];

    LOG_DEBUG("GPS[%d] 초기화 - 타입: %s", i,
             type == GPS_TYPE_F9P     ? "F9P"
             : type == GPS_TYPE_UM982 ? "UM982"
                                      : "UNKNOWN");

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

    gps_instances[i].queue = xQueueCreate(10, sizeof(uint8_t));
    if (gps_instances[i].queue == NULL) {
      LOG_ERR("GPS[%d] 큐 생성 실패", i);
      gps_instances[i].enabled = false;
      continue;
    }

    gps_port_set_queue((gps_id_t)i, gps_instances[i].queue);

    gps_instances[i].cmd_queue = xQueueCreate(5, sizeof(gps_cmd_request_t));
    if (gps_instances[i].cmd_queue == NULL) {
      LOG_ERR("GPS[%d] TX 큐 생성 실패", i);
      gps_instances[i].enabled = false;
      continue;
    }

    gps_port_start(&gps_instances[i].gps);

    char task_name[16];
    snprintf(task_name, sizeof(task_name), "gps_rx_%d", i);

    BaseType_t ret =
        xTaskCreate(gps_process_task, task_name, 1024,
                    (void *)(uintptr_t)i, // GPS ID
                    tskIDLE_PRIORITY + 1, &gps_instances[i].task);

    if (ret != pdPASS) {
      LOG_ERR("GPS[%d] RX 태스크 생성 실패", i);
      gps_instances[i].enabled = false;
      continue;
    }

    snprintf(task_name, sizeof(task_name), "gps_tx_%d", i);
    ret = xTaskCreate(gps_tx_task, task_name, 512,
                      (void *)(uintptr_t)i, // GPS ID를 파라미터로 전달
                      tskIDLE_PRIORITY + 1, &gps_instances[i].tx_task);

    if (ret != pdPASS) {
      LOG_ERR("GPS[%d] TX 태스크 생성 실패", i);
      gps_instances[i].enabled = false;
      continue;
    }

    LOG_INFO("GPS[%d] 인스턴스 초기화 완료", i);

  }

  LOG_INFO("GPS 전체 인스턴스 초기화 완료");
  if (config->board == BOARD_TYPE_BASE_F9P || config->board == BOARD_TYPE_BASE_UM982) {
    user_params_t *params = flash_params_get_current();
    if (params->base_auto_fix_enabled) {
      LOG_INFO("Base Auto-Fix 모드 활성화");

      // 첫 번째 GPS 인스턴스로 초기화
      if (base_auto_fix_init(GPS_ID_BASE)) {
        // 초기화 성공 시 시작
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

bool gps_send_cmd_sync(const char *cmd, uint32_t timeout_ms) {
  if (!cmd || strlen(cmd) == 0) {
    LOG_ERR("GPS[%d] empty command", id);
    return false;
  }

  SemaphoreHandle_t resp_sem = xSemaphoreCreateBinary();
  if (resp_sem == NULL) {
    LOG_ERR("GPS[%d] failed to create semaphore", id);
    return false;
  }

  // 명령어 요청 구조체 생성
  bool result = false;
 gps_cmd_request_t cmd_req = {
      .timeout_ms = timeout_ms,
      .is_async = false,
      .response_sem = response_sem,
      .result = &result,
      .callback = NULL,
      .user_data = NULL,
  };

  if (xSemaphoreTake(response_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
    // 처리 완료
    vSemaphoreDelete(response_sem);
    return result;
  } else {
    LOG_ERR("GPS[%d] 전송 태스크 응답 없음", id);
    vSemaphoreDelete(response_sem);
    return false;
  }
}

// bool gps_send_raw_data(gps_id_t id, const uint8_t *data, size_t len) {
//   if (id >= GPS_ID_MAX || !gps_instances[id].enabled) {
//     LOG_ERR("GPS[%d] invalid", id);
//     return false;
//   }

//   if (!data || len == 0) {
//     LOG_ERR("GPS[%d] invalid data", id);
//     return false;
//   }

//   gps_instance_t *inst = &gps_instances[id];

//   if (!inst->gps.ops || !inst->gps.ops->send) {
//     LOG_ERR("GPS[%d] ops invalid", id);
//     return false;
//   }

//   if (xSemaphoreTake(inst->gps.mutex, pdMS_TO_TICKS(2000)) == pdTRUE) {
//     inst->gps.ops->send((const char *)data, len);
//     xSemaphoreGive(inst->gps.mutex);
//     LOG_DEBUG("GPS[%d] 송신 %d 바이트", id, len);
//     return true;
//   } else {
//     LOG_ERR("GPS[%d] 뮤텍스 타임아웃", id);
//     return false;
//   }
// }



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
 * - Ublox F9P: HPPOSLLH (lat, lon, height, msl) + RELPOSNED (heading)
 */
bool gps_format_position_data(char *buffer)
{
  gps_instance_t *inst = &gps_instances[0];
  gps_instance_t *heading_inst = &gps_instances[1];
  const board_config_t *config = board_get_config();
  double lat, lon, geoid, msl_alt, ellipsoid_alt, heading;
  char ns = 'N', ew = 'E';
  int fix;
  int sat_num;

  char ns_str[2] = {ns, '\0'};
  char ew_str[2] = {ew, '\0'};

  taskENTER_CRITICAL();
  if(config->board == BOARD_TYPE_ROVER_F9P)
  {
    lat = convert_dm(inst->gps.ubx_data.hpposllh.lat * 1e-7 + inst->gps.ubx_data.hpposllh.lat_hp * 1e-9);
    lon = convert_dm(inst->gps.ubx_data.hpposllh.lon * 1e-7 + inst->gps.ubx_data.hpposllh.lon_hp * 1e-9);
    ellipsoid_alt = (inst->gps.ubx_data.hpposllh.height + inst->gps.ubx_data.hpposllh.height_hp * (double)0.1) / (double)1000.0;
    msl_alt = (inst->gps.ubx_data.hpposllh.msl + inst->gps.ubx_data.hpposllh.msl_hp * (double)0.1) / (double)1000.0;
    geoid = ellipsoid_alt - msl_alt;
    heading = heading_inst->gps.ubx_data.relposned.rel_pos_heading * 1e-5;
    ns = inst->gps.nmea_data.gga.ns;
    ew = inst->gps.nmea_data.gga.ew;
    fix = inst->gps.nmea_data.gga.fix;
    sat_num = inst->gps.nmea_data.gga.sat_num;
  }
  else if (config->board == BOARD_TYPE_ROVER_UM982) 
  {
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
  }
  taskEXIT_CRITICAL();

  int written = sprintf(buffer,
                         "+GPS,%.9lf,%s,%.9lf,%s,%.4lf,%.4lf,%.5lf,%d,%d\n\r",
                        lat, ns_str, lon, ew_str,
                        ellipsoid_alt,
                        geoid,
                        heading,
                        fix,
                        sat_num);

  return true;
}


 

/**

 * @brief 특정 GPS 인스턴스 정리 및 태스크 삭제

 */

bool gps_cleanup_instance(gps_id_t id)
{

  return true;

}


/**

 * @brief 모든 GPS 인스턴스 정리 및 태스크 삭제

 */

void gps_cleanup_all(void)
{

  LOG_INFO("모든 GPS 인스턴스 정리 완료 (IDLE 태스크 정리 대기 완료)");

}
