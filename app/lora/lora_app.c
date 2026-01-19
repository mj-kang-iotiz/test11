#include "lora.h"
#include "lora_app.h"
#include "lora_port.h"
#include "board_config.h"
#include "event_bus.h"
#include "gps.h"
#include "gps_app.h"
#include "rtcm.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "led.h"

#ifndef TAG
#define TAG "LORA_APP"
#endif

#include "log.h"

/*===========================================================================
 * 이벤트 핸들러 선언
 *===========================================================================*/
static void on_rtcm_for_lora(const event_t *event, void *user_data);

/*===========================================================================
 * 태스크 선언
 *===========================================================================*/
static void lora_process_task(void *pvParameter);
static void lora_tx_task(void *pvParameter);

/**
 * @brief LoRa P2P BASE 모드 초기화 명령어
 */
static const char *lora_p2p_base_cmds[] = {
    "at+set_config=lora:work_mode:1\r\n",             // P2P 모드 (1=P2P, 0=LoRaWAN)
    "at+set_config=lorap2p:922500000:7:2:1:8:14\r\n", // 922.5MHz, SF7, BW500kHz, CR4/5, Preamble8, 14dBm
    "at+set_config=lorap2p:transfer_mode:2\r\n",      // Transfer mode 2 (BASE)
};

/**
 * @brief LoRa P2P ROVER 모드 초기화 명령어
 */
static const char *lora_p2p_rover_cmds[] = {
    "at+set_config=lora:work_mode:1\r\n",             // P2P 모드 (1=P2P, 0=LoRaWAN)
    "at+set_config=lorap2p:922500000:7:2:1:8:14\r\n", // 922.5MHz, SF7, BW500kHz, CR4/5, Preamble8, 14dBm
    "at+set_config=lorap2p:transfer_mode:1\r\n",      // Transfer mode 1 (ROVER)
};

#define LORA_P2P_BASE_CMD_COUNT (sizeof(lora_p2p_base_cmds) / sizeof(lora_p2p_base_cmds[0]))
#define LORA_P2P_ROVER_CMD_COUNT (sizeof(lora_p2p_rover_cmds) / sizeof(lora_p2p_rover_cmds[0]))

typedef void (*lora_init_callback_t)(bool success, void *user_data);

typedef struct
{
  uint8_t current_step;          // 현재 단계 (0 ~ cmd_count-1)
  uint8_t retry_count;           // 현재 단계 재시도 횟수
  const char **cmd_list;         // 명령어 리스트
  uint8_t cmd_count;             // 명령어 개수
  lora_init_callback_t callback; // 완료 콜백
} lora_init_context_t;

/*===========================================================================
 * 앱 인스턴스 (BLE 스타일 - lora_app_t 사용)
 *===========================================================================*/
static lora_app_t instance;

/**
 * @brief LoRa 초기화 완료 콜백
 */
static void lora_overall_init_complete(bool success, void *user_data)
{
  const board_config_t *config = board_get_config();
  if (config->lora_mode == LORA_MODE_BASE)
  {
    LOG_INFO("LoRa BASE init %s", success ? "succeeded" : "failed");
  }
  else if (config->lora_mode == LORA_MODE_ROVER)
  {
    LOG_INFO("LoRa ROVER init %s", success ? "succeeded" : "failed");
  }

  if (success)
  {
    instance.lora.init_complete = true;
    LOG_INFO("LoRa init complete - now accepting P2P data");

    led_set_color(3, LED_COLOR_GREEN);
    led_set_state(3, true);
  }
  else
  {
    LOG_ERR("LoRa init failed");
    led_set_color(3, LED_COLOR_RED);
    led_set_state(3, true);
  }
}

/**
 * @brief LoRa 초기화 명령어 콜백 (재귀적 호출)
 */
static void lora_init_command_callback(bool success, void *user_data)
{
  lora_init_context_t *ctx = (lora_init_context_t *)user_data;

  if (!ctx)
  {
    LOG_ERR("LoRa init context is NULL");
    return;
  }

  if (success)
  {
    // 명령어 성공
    LOG_INFO("LoRa init step %d/%d OK: %s",
             ctx->current_step + 1, ctx->cmd_count,
             ctx->cmd_list[ctx->current_step]);

    // 다음 단계로
    ctx->current_step++;
    ctx->retry_count = 0;

    // 모든 단계 완료?
    if (ctx->current_step >= ctx->cmd_count)
    {
      LOG_INFO("LoRa init sequence complete!");
      if (ctx->callback)
      {
        ctx->callback(true, NULL);
      }
      // 컨텍스트 메모리 해제
      vPortFree(ctx);
      return;
    }

    // 다음 명령어 전송
    // work_mode 명령어는 응답 파싱 건너뛰기
    bool skip = (strstr(ctx->cmd_list[ctx->current_step], "work_mode") != NULL);
    lora_send_command_async(ctx->cmd_list[ctx->current_step],
                            LORA_INIT_TIMEOUT_MS, 0, lora_init_command_callback, ctx, skip);
  }
  else
  {
    // 명령어 실패
    ctx->retry_count++;

    if (ctx->retry_count < LORA_INIT_MAX_RETRY)
    {
      // 재시도
      LOG_WARN("LoRa init step %d/%d failed, retrying (%d/%d): %s",
               ctx->current_step + 1, ctx->cmd_count,
               ctx->retry_count, LORA_INIT_MAX_RETRY,
               ctx->cmd_list[ctx->current_step]);

      // 같은 명령어 재전송
      // work_mode 명령어는 응답 파싱 건너뛰기
      bool skip = (strstr(ctx->cmd_list[ctx->current_step], "work_mode") != NULL);
      lora_send_command_async(ctx->cmd_list[ctx->current_step],
                              LORA_INIT_TIMEOUT_MS, 0, lora_init_command_callback, ctx, skip);
    }
    else
    {
      // 최대 재시도 초과
      LOG_ERR("LoRa init failed at step %d/%d after %d retries: %s",
              ctx->current_step + 1, ctx->cmd_count,
              LORA_INIT_MAX_RETRY, ctx->cmd_list[ctx->current_step]);

      if (ctx->callback)
      {
        ctx->callback(false, NULL);
      }
      // 컨텍스트 메모리 해제
      vPortFree(ctx);
    }
  }
}

/**
 * @brief LoRa P2P BASE 모드 초기화 (비동기)
 */
static bool lora_init_p2p_base_async(lora_init_callback_t callback)
{
  // 초기화 컨텍스트 생성 (동적 할당)
  lora_init_context_t *ctx = (lora_init_context_t *)pvPortMalloc(sizeof(lora_init_context_t));
  if (!ctx)
  {
    LOG_ERR("Failed to allocate LoRa init context");
    return false;
  }

  // 컨텍스트 초기화
  ctx->current_step = 0;
  ctx->retry_count = 0;
  ctx->cmd_list = lora_p2p_base_cmds;
  ctx->cmd_count = LORA_P2P_BASE_CMD_COUNT;
  ctx->callback = callback;

  LOG_INFO("Starting LoRa P2P BASE init sequence (%d commands)", ctx->cmd_count);

  // 첫 번째 명령어 전송
  // work_mode 명령어는 응답 파싱 건너뛰기
  bool skip = (strstr(ctx->cmd_list[0], "work_mode") != NULL);
  if (!lora_send_command_async(ctx->cmd_list[0], LORA_INIT_TIMEOUT_MS, 0,
                               lora_init_command_callback, ctx, skip))
  {
    LOG_ERR("Failed to start LoRa init sequence");
    vPortFree(ctx);
    return false;
  }

  return true;
}

/**
 * @brief LoRa P2P ROVER 모드 초기화 (비동기)
 */
static bool lora_init_p2p_rover_async(lora_init_callback_t callback)
{
  // 초기화 컨텍스트 생성 (동적 할당)
  lora_init_context_t *ctx = (lora_init_context_t *)pvPortMalloc(sizeof(lora_init_context_t));
  if (!ctx)
  {
    LOG_ERR("Failed to allocate LoRa init context");
    return false;
  }

  // 컨텍스트 초기화
  ctx->current_step = 0;
  ctx->retry_count = 0;
  ctx->cmd_list = lora_p2p_rover_cmds;
  ctx->cmd_count = LORA_P2P_ROVER_CMD_COUNT;
  ctx->callback = callback;

  LOG_INFO("Starting LoRa P2P ROVER init sequence (%d commands)", ctx->cmd_count);

  // 첫 번째 명령어 전송
  // work_mode 명령어는 응답 파싱 건너뛰기
  bool skip = (strstr(ctx->cmd_list[0], "work_mode") != NULL);
  if (!lora_send_command_async(ctx->cmd_list[0], LORA_INIT_TIMEOUT_MS, 0,
                               lora_init_command_callback, ctx, skip))
  {
    LOG_ERR("Failed to start LoRa init sequence");
    vPortFree(ctx);
    return false;
  }

  return true;
}

/**
 * @brief AT 명령어 응답 파싱 (OK/ERROR 감지)
 *
 * @param data 응답 데이터
 * @param len 데이터 길이
 * @return true: OK, false: ERROR or 미감지
 */
static bool lora_parse_at_response(const char *data, size_t len)
{
  // OK 응답 감지 (다양한 형식 지원)
  // - "OK\r\n", "OK\n" (일반 응답)
  // - "Initialization OK" (work_mode 변경 시)
  // - 대소문자 무시
  if (strstr(data, "OK") != NULL || strstr(data, "ok") != NULL)
  {
    return true;
  }

  // ERROR 응답 감지
  if (strstr(data, "ERROR") != NULL || strstr(data, "error") != NULL)
  {
    return false;
  }

  return false;
}

/**
 * @brief RTCM fragment 재조립 버퍼 초기화
 */
static void rtcm_reassembly_reset(lora_rtcm_reassembly_t *reasm)
{
  reasm->buffer_pos = 0;
  reasm->expected_len = 0;
  reasm->has_header = false;
  reasm->last_recv_tick = 0;
}

/**
 * @brief RTCM fragment 처리 및 재조립 (개선된 버전)
 *
 * - Preamble(0xD3) 자동 탐색
 * - 완료 후 남은 데이터 자동 보존
 *
 * @param reasm 재조립 버퍼
 * @param data 수신된 fragment 데이터
 * @param len fragment 길이
 * @return true: 완전한 RTCM 패킷 재조립 완료, false: 더 많은 fragment 필요
 */
static bool rtcm_reassembly_process(lora_rtcm_reassembly_t *reasm, const uint8_t *data, size_t len)
{
  TickType_t current_tick = xTaskGetTickCount();

  // 타임아웃 체크 (마지막 수신 후 5초 경과 시 초기화)
  if (reasm->buffer_pos > 0 && reasm->last_recv_tick > 0)
  {
    TickType_t elapsed_ms = (current_tick - reasm->last_recv_tick) * 1000 / configTICK_RATE_HZ;
    if (elapsed_ms > RTCM_REASSEMBLY_TIMEOUT_MS)
    {
      LOG_WARN("RTCM reassembly timeout - resetting buffer");
      rtcm_reassembly_reset(reasm);
    }
  }

  // 버퍼 오버플로우 체크
  if (reasm->buffer_pos + len > RTCM_REASSEMBLY_BUF_SIZE)
  {
    LOG_ERR("RTCM reassembly buffer overflow - resetting");
    rtcm_reassembly_reset(reasm);
    return false;
  }

  // Fragment 데이터 추가
  memcpy(&reasm->buffer[reasm->buffer_pos], data, len);
  reasm->buffer_pos += len;
  reasm->last_recv_tick = current_tick;

  LOG_INFO("RTCM fragment received: %d bytes, total: %d bytes", len, reasm->buffer_pos);

  // 헤더가 없으면 Preamble 스캔
  if (!reasm->has_header)
  {
    // 0xD3(Preamble)를 찾을 때까지 스캔
    size_t preamble_pos = 0;
    bool found = false;

    for (size_t i = 0; i < reasm->buffer_pos; i++)
    {
      if (reasm->buffer[i] == 0xD3)
      {
        preamble_pos = i;
        found = true;
        LOG_INFO("RTCM preamble found at offset %d", i);
        break;
      }
    }

    if (!found)
    {
      // Preamble을 못 찾으면 마지막 바이트만 남기고 버림
      // (다음 fragment에서 0xD3가 올 수 있음)
      if (reasm->buffer_pos > 1)
      {
        LOG_WARN("No preamble found, keeping last byte");
        reasm->buffer[0] = reasm->buffer[reasm->buffer_pos - 1];
        reasm->buffer_pos = 1;
      }
      return false;
    }

    // Preamble이 중간에 있으면 앞으로 이동
    if (preamble_pos > 0)
    {
      LOG_INFO("Moving preamble from offset %d to start", preamble_pos);
      memmove(reasm->buffer, &reasm->buffer[preamble_pos], reasm->buffer_pos - preamble_pos);
      reasm->buffer_pos -= preamble_pos;
    }

    // 헤더 파싱 시도 (최소 3바이트 필요: preamble + reserved+len)
    if (reasm->buffer_pos >= 3)
    {
      // Payload 길이 파싱 (10 bits)
      uint16_t payload_len = ((reasm->buffer[1] & 0x03) << 8) | reasm->buffer[2];
      reasm->expected_len = 3 + payload_len + 3; // header(3) + payload + CRC(3)
      reasm->has_header = true;

      LOG_INFO("RTCM header parsed: payload=%d bytes, expected total=%d bytes",
               payload_len, reasm->expected_len);

      // 예상 길이가 비정상적으로 크면 리셋
      if (reasm->expected_len > RTCM_REASSEMBLY_BUF_SIZE)
      {
        LOG_ERR("Invalid RTCM length: %d > %d - resetting",
                reasm->expected_len, RTCM_REASSEMBLY_BUF_SIZE);
        rtcm_reassembly_reset(reasm);
        return false;
      }
    }
  }

  // 완전한 패킷 수신 체크
  if (reasm->has_header && reasm->buffer_pos >= reasm->expected_len)
  {
    LOG_INFO("RTCM packet reassembly complete: %d bytes (buffer has %d bytes)",
             reasm->expected_len, reasm->buffer_pos);
    return true;
  }

  // 더 많은 fragment 필요
  return false;
}

/**
 * @brief AT+RECV 응답 파싱
 *
 * Format: at+recv=<RSSI>,<SNR>,<Data Length>:<Data>
 * Example: at+recv=-50,10,5:48656C6C6F
 *
 * @param data 응답 데이터
 * @param recv_data 출력 구조체
 * @return true: 파싱 성공, false: 파싱 실패
 */
static bool lora_parse_p2p_recv(const char *data, lora_p2p_recv_data_t *recv_data)
{
  // "at+recv=" 또는 "AT+RECV=" 찾기
  const char *start = strstr(data, "at+recv=");
  if (!start)
  {
    start = strstr(data, "AT+RECV=");
  }
  if (!start)
  {
    return false;
  }

  start += 8; // "at+recv=" 건너뛰기

  // RSSI 파싱
  char *end = NULL;
  recv_data->rssi = (int16_t)strtol(start, &end, 10);
  if (!end || *end != ',')
  {
    return false;
  }

  // SNR 파싱
  start = end + 1;
  recv_data->snr = (int16_t)strtol(start, &end, 10);
  if (!end || *end != ',')
  {
    return false;
  }

  // Data Length 파싱
  start = end + 1;
  recv_data->data_len = (uint16_t)strtol(start, &end, 10);
  if (!end || *end != ':')
  {
    return false;
  }

  // Data 파싱 (HEX string)
  start = end + 1;

  // 데이터 길이 확인
  if (recv_data->data_len > sizeof(recv_data->data) - 1)
  {
    LOG_ERR("P2P recv data too long: %d bytes", recv_data->data_len);
    return false;
  }

  // HEX string을 바이너리로 변환
  for (uint16_t i = 0; i < recv_data->data_len; i++)
  {
    char hex[3] = {start[i * 2], start[i * 2 + 1], '\0'};
    recv_data->data[i] = (char)strtol(hex, NULL, 16);
  }
  recv_data->data[recv_data->data_len] = '\0';

  LOG_INFO("P2P recv: RSSI=%d, SNR=%d, Len=%d",
           recv_data->rssi, recv_data->snr, recv_data->data_len);

  return true;
}

/**
 * @brief LoRa TX Task (명령어 송신 및 응답 대기)
 */
static void lora_tx_task(void *pvParameter)
{
  lora_cmd_request_t cmd_req;

  LOG_INFO("LoRa TX Task started");

  // TX Task 준비 완료 플래그 설정
  instance.lora.tx_task_ready = true;
  LOG_INFO("LoRa TX Task ready");

  const board_config_t *config = board_get_config();

  while (1)
  {
    if (xQueueReceive(instance.lora.cmd_queue, &cmd_req, portMAX_DELAY) == pdTRUE)
    {
      if(config->lora_mode == LORA_MODE_BASE && instance.lora.init_complete)
      {
        led_set_toggle(3);
      }
      LOG_INFO("LoRa sending command: %s", cmd_req.cmd);

      // 현재 명령어 요청 저장 (RX Task에서 응답 처리용)
      instance.lora.current_cmd_req = &cmd_req;
      if (cmd_req.is_async)
      {
        cmd_req.async_result = false;
      }
      else
      {
        *(cmd_req.result) = false;
      }

      // 시작 시간 기록 (ToA 계산용)
      TickType_t start_tick = xTaskGetTickCount();

      // 명령어 전송 (UART 충돌 방지를 위해 mutex 사용)
      if (instance.lora.ops && instance.lora.ops->send)
      {
        xSemaphoreTake(instance.lora.mutex, portMAX_DELAY);
        instance.lora.ops->send(cmd_req.cmd, strlen(cmd_req.cmd));
        xSemaphoreGive(instance.lora.mutex);
      }
      else
      {
        LOG_ERR("LoRa send ops not available");
        instance.lora.current_cmd_req = NULL;
        if (cmd_req.is_async)
        {
          cmd_req.async_result = false;
          if (cmd_req.callback)
          {
            cmd_req.callback(false, cmd_req.user_data);
          }
          vSemaphoreDelete(cmd_req.response_sem);
        }
        else
        {
          *(cmd_req.result) = false;
          xSemaphoreGive(cmd_req.response_sem);
        }
        continue;
      }

      // skip_response이면 응답 파싱 건너뛰고 delay 후 성공 처리
      if (cmd_req.skip_response)
      {
        LOG_INFO("Skipping response check, waiting %d ms", cmd_req.timeout_ms);
        vTaskDelay(pdMS_TO_TICKS(cmd_req.timeout_ms));

        // 성공으로 처리
        if (cmd_req.is_async)
        {
          cmd_req.async_result = true;
        }
        else
        {
          *(cmd_req.result) = true;
        }
      }
      else
      {
        // 응답 대기 (타임아웃 적용)
        if (xSemaphoreTake(cmd_req.response_sem, pdMS_TO_TICKS(cmd_req.timeout_ms)) == pdTRUE)
        {
          // 응답 수신 완료 (RX Task가 세마포어를 줌)
          if (cmd_req.is_async)
          {
            LOG_INFO("LoRa response received: %s",
                     cmd_req.async_result ? "OK" : "ERROR");
          }
          else
          {
            LOG_INFO("LoRa response received: %s",
                     *(cmd_req.result) ? "OK" : "ERROR");
          }

          // ToA 대기: AT 명령어 전송 시작 시점부터 ToA 경과 보장
          if (cmd_req.toa_ms > 0)
          {
            TickType_t elapsed_tick = xTaskGetTickCount() - start_tick;
            uint32_t elapsed_ms = elapsed_tick * 1000 / configTICK_RATE_HZ;

            if (elapsed_ms < cmd_req.toa_ms)
            {
              uint32_t remaining_ms = cmd_req.toa_ms - elapsed_ms;
              LOG_INFO("Waiting remaining ToA %dms (elapsed=%dms, total=%dms)",
                       remaining_ms, elapsed_ms, cmd_req.toa_ms);
              vTaskDelay(pdMS_TO_TICKS(remaining_ms));
            }
            else
            {
              LOG_INFO("ToA already satisfied: elapsed=%dms >= ToA=%dms",
                       elapsed_ms, cmd_req.toa_ms);
            }
          }
        }
        else
        {
          // 타임아웃
          LOG_WARN("LoRa command timeout");
          if (cmd_req.is_async)
          {
            cmd_req.async_result = false;
          }
          else
          {
            *(cmd_req.result) = false;
          }
        }
      }

      // 현재 명령어 요청 초기화
      instance.lora.current_cmd_req = NULL;

      // 비동기: 콜백 호출
      if (cmd_req.is_async)
      {
        if (cmd_req.callback)
        {
          cmd_req.callback(cmd_req.async_result, cmd_req.user_data);
        }

        // 비동기는 세마포어를 TX Task에서 삭제
        vSemaphoreDelete(cmd_req.response_sem);
      }
      else
      {
        // 동기: 외부 호출자에게 처리 완료 알림 (세마포어 반환)
        xSemaphoreGive(cmd_req.response_sem);
      }
    }
  }

  vTaskDelete(NULL);
}

/**
 * @brief LoRa RX Task (수신 데이터 처리)
 */
static void lora_process_task(void *pvParameter)
{
  size_t pos = 0;
  size_t old_pos = 0;
  uint8_t dummy = 0;

  __attribute__((section(".ccmram"))) static char temp_buf[1024];
  LOG_INFO("LoRa RX Task started");

  // RX Task 준비 완료 플래그 설정
  instance.lora.rx_task_ready = true;
  LOG_INFO("LoRa RX Task ready");

  // TX/RX Task 모두 준비될 때까지 대기
  while (!instance.lora.tx_task_ready || !instance.lora.rx_task_ready)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  LOG_INFO("Both TX and RX tasks ready, starting LoRa initialization");

  const board_config_t *config = board_get_config();

  if (config->lora_mode == LORA_MODE_BASE)
  {
    lora_init_p2p_base_async(lora_overall_init_complete);
  }
  else if (config->lora_mode == LORA_MODE_ROVER)
  {
    lora_init_p2p_rover_async(lora_overall_init_complete);
    led_set_color(3, LED_COLOR_GREEN);
    led_set_state(3, true);
  }

  while (1)
  {
    xQueueReceive(instance.lora.rx_queue, &dummy, portMAX_DELAY);

    pos = lora_port_get_rx_pos();
    char *lora_recv = lora_port_get_recv_buf();

    if (pos != old_pos)
    {
      size_t len = 0;

      if (pos > old_pos)
      {
        len = pos - old_pos;

        // 데이터 처리
        memcpy(temp_buf, &lora_recv[old_pos], len);
        temp_buf[len] = '\0';

        // AT 명령어 응답 처리
        if (instance.lora.current_cmd_req != NULL)
        {
          bool result = lora_parse_at_response(temp_buf, len);
          LOG_INFO("Parse result: %s", result ? "OK" : "ERROR/NONE");

          if (strstr(temp_buf, "OK") || strstr(temp_buf, "ERROR"))
          {
            // 응답 결과 저장
            if (instance.lora.current_cmd_req->is_async)
            {
              instance.lora.current_cmd_req->async_result = result;
            }
            else
            {
              *(instance.lora.current_cmd_req->result) = result;
            }

            // 세마포어 해제 (TX Task로 응답 완료 알림)
            xSemaphoreGive(instance.lora.current_cmd_req->response_sem);
          }
          else
          {
            LOG_WARN("No OK/ERROR in response");
          }
        }
        else
        {
          LOG_WARN("current_cmd_req is NULL, skipping response handling");
        }

        // P2P 수신 데이터 처리 (at+recv=...)
        // 초기화 완료 후에만 처리 (초기화 중 데이터는 무시)

        if ((strstr(temp_buf, "at+recv=") || strstr(temp_buf, "AT+RECV=")) && instance.lora.init_complete)
        {
          if(config->lora_mode == LORA_MODE_ROVER)
          {
            led_set_toggle(3);
          }

          lora_p2p_recv_data_t recv_data;

          if (lora_parse_p2p_recv(temp_buf, &recv_data))
          {
            // 콜백이 등록되어 있으면 콜백 호출
            if (instance.lora.p2p_recv_callback)
            {
              instance.lora.p2p_recv_callback(&recv_data, instance.lora.p2p_recv_user_data);
            }
            else
            {
              // 콜백이 없으면 RTCM fragment 재조립 및 GPS로 전송
              LOG_INFO("P2P data received: %d bytes, RSSI=%d, SNR=%d",
                       recv_data.data_len, recv_data.rssi, recv_data.snr);

              // RTCM fragment 재조립
              bool complete = rtcm_reassembly_process(&instance.lora.rtcm_reassembly,
                                                      (uint8_t *)recv_data.data,
                                                      recv_data.data_len);

              if (complete)
              {
                // 완전한 RTCM 패킷 수신 - 검증 후 GPS로 전송
                if (rtcm_validate_packet(instance.lora.rtcm_reassembly.buffer,
                                        instance.lora.rtcm_reassembly.expected_len))
                {
                  LOG_INFO("Valid RTCM packet - sending to GPS via UART");

                  // GPS UART로 직접 전송
                  if (!gps_send_raw_data(GPS_ID_BASE,
                                         instance.lora.rtcm_reassembly.buffer,
                                         instance.lora.rtcm_reassembly.expected_len))
                  {
                    LOG_ERR("Failed to send RTCM data to GPS");
                  }
                }
                else
                {
                  LOG_ERR("Invalid RTCM packet - discarding");
                }

                // 남은 데이터 처리 (다음 RTCM 패킷의 시작일 수 있음)
                if (instance.lora.rtcm_reassembly.buffer_pos > instance.lora.rtcm_reassembly.expected_len)
                {
                  size_t remaining = instance.lora.rtcm_reassembly.buffer_pos - instance.lora.rtcm_reassembly.expected_len;
                  LOG_INFO("Remaining %d bytes in buffer - moving to front", remaining);

                  // 남은 데이터를 버퍼 앞으로 이동
                  memmove(instance.lora.rtcm_reassembly.buffer,
                          &instance.lora.rtcm_reassembly.buffer[instance.lora.rtcm_reassembly.expected_len],
                          remaining);
                  instance.lora.rtcm_reassembly.buffer_pos = remaining;
                  instance.lora.rtcm_reassembly.has_header = false;
                  instance.lora.rtcm_reassembly.expected_len = 0;

                  // 남은 데이터로 다음 패킷 시작 시도
                  // (재귀 호출 대신 다음 수신에서 처리됨)
                }
                else
                {
                  // 남은 데이터가 없으면 완전히 초기화
                  rtcm_reassembly_reset(&instance.lora.rtcm_reassembly);
                }
              }
            }
          }
        }
        else if ((strstr(temp_buf, "at+recv=") || strstr(temp_buf, "AT+RECV=")) && !instance.lora.init_complete)
        {
          LOG_WARN("Ignoring P2P data during initialization");
        }

        old_pos = pos;
      }
      else
      {
        // Circular buffer wrap-around
        len = LORA_RECV_BUF_SIZE - old_pos + pos;

        size_t first_part = LORA_RECV_BUF_SIZE - old_pos;
        memcpy(temp_buf, &lora_recv[old_pos], first_part);
        memcpy(&temp_buf[first_part], &lora_recv[0], pos);
        temp_buf[len] = '\0';

        LOG_INFO("LoRa recv (wrap): %s", temp_buf);

        // 동일한 처리 로직
        if (instance.lora.current_cmd_req != NULL)
        {
          bool result = lora_parse_at_response(temp_buf, len);

          if (strstr(temp_buf, "OK") || strstr(temp_buf, "ERROR"))
          {
            if (instance.lora.current_cmd_req->is_async)
            {
              instance.lora.current_cmd_req->async_result = result;
            }
            else
            {
              *(instance.lora.current_cmd_req->result) = result;
            }

            xSemaphoreGive(instance.lora.current_cmd_req->response_sem);
          }
        }

        if ((strstr(temp_buf, "at+recv=") || strstr(temp_buf, "AT+RECV=")) && instance.lora.init_complete)
        {
          if(config->lora_mode == LORA_MODE_ROVER)
          {
            led_set_toggle(3);
          }

          lora_p2p_recv_data_t recv_data;

          if (lora_parse_p2p_recv(temp_buf, &recv_data))
          {
            // 콜백이 등록되어 있으면 콜백 호출
            if (instance.lora.p2p_recv_callback)
            {
              instance.lora.p2p_recv_callback(&recv_data, instance.lora.p2p_recv_user_data);
            }
            else
            {
              // 콜백이 없으면 RTCM fragment 재조립 및 GPS로 전송
              LOG_INFO("P2P data received (wrap): %d bytes, RSSI=%d, SNR=%d",
                       recv_data.data_len, recv_data.rssi, recv_data.snr);

              // RTCM fragment 재조립
              bool complete = rtcm_reassembly_process(&instance.lora.rtcm_reassembly,
                                                      (uint8_t *)recv_data.data,
                                                      recv_data.data_len);

              if (complete)
              {
                // 완전한 RTCM 패킷 수신 - 검증 후 GPS로 전송
                if (rtcm_validate_packet(instance.lora.rtcm_reassembly.buffer,
                                        instance.lora.rtcm_reassembly.expected_len))
                {
                  LOG_INFO("Valid RTCM packet - sending to GPS via UART (wrap)");

                  // GPS UART로 직접 전송
                  if (!gps_send_raw_data(GPS_ID_BASE,
                                         instance.lora.rtcm_reassembly.buffer,
                                         instance.lora.rtcm_reassembly.expected_len))
                  {
                    LOG_ERR("Failed to send RTCM data to GPS (wrap)");
                  }
                }
                else
                {
                  LOG_ERR("Invalid RTCM packet - discarding (wrap)");
                }

                // 남은 데이터 처리 (다음 RTCM 패킷의 시작일 수 있음)
                if (instance.lora.rtcm_reassembly.buffer_pos > instance.lora.rtcm_reassembly.expected_len)
                {
                  size_t remaining = instance.lora.rtcm_reassembly.buffer_pos - instance.lora.rtcm_reassembly.expected_len;
                  LOG_INFO("Remaining %d bytes in buffer - moving to front (wrap)", remaining);

                  // 남은 데이터를 버퍼 앞으로 이동
                  memmove(instance.lora.rtcm_reassembly.buffer,
                          &instance.lora.rtcm_reassembly.buffer[instance.lora.rtcm_reassembly.expected_len],
                          remaining);
                  instance.lora.rtcm_reassembly.buffer_pos = remaining;
                  instance.lora.rtcm_reassembly.has_header = false;
                  instance.lora.rtcm_reassembly.expected_len = 0;

                  // 남은 데이터로 다음 패킷 시작 시도
                  // (재귀 호출 대신 다음 수신에서 처리됨)
                }
                else
                {
                  // 남은 데이터가 없으면 완전히 초기화
                  rtcm_reassembly_reset(&instance.lora.rtcm_reassembly);
                }
              }
            }
          }
        }
        else if ((strstr(temp_buf, "at+recv=") || strstr(temp_buf, "AT+RECV=")) && !instance.lora.init_complete)
        {
          LOG_WARN("Ignoring P2P data during initialization (wrap)");
        }

        old_pos = pos;
      }
    }
  }

  vTaskDelete(NULL);
}

/*===========================================================================
 * 인스턴스 초기화/해제 (내부용)
 *===========================================================================*/

void lora_instance_init(void)
{
  memset(&instance, 0, sizeof(lora_app_t));
  lora_init(&instance.lora, NULL);

  // RTCM 재조립 버퍼 초기화
  rtcm_reassembly_reset(&instance.lora.rtcm_reassembly);

  if (lora_port_init_instance(&instance.lora) != 0)
  {
    LOG_ERR("LORA 포트 초기화 실패");
    return;
  }

  // RX 이벤트 큐 생성
  instance.lora.rx_queue = xQueueCreate(LORA_RX_QUEUE_SIZE, sizeof(uint8_t));
  if (instance.lora.rx_queue == NULL)
  {
    LOG_ERR("LORA RX 큐 생성 실패");
    return;
  }

  // TX 명령어 큐 생성
  instance.lora.cmd_queue = xQueueCreate(LORA_CMD_QUEUE_SIZE, sizeof(lora_cmd_request_t));
  if (instance.lora.cmd_queue == NULL)
  {
    LOG_ERR("LORA TX 큐 생성 실패");
    return;
  }

  lora_port_set_queue(instance.lora.rx_queue);
  instance.lora.mutex = xSemaphoreCreateMutex();
  lora_port_start(&instance.lora);

  // RX Task 생성
  BaseType_t ret = xTaskCreate(lora_process_task, "lora_rx", 1024,
                               NULL, tskIDLE_PRIORITY + 3, &instance.lora.rx_task);
  if (ret != pdPASS)
  {
    LOG_ERR("LORA RX Task 생성 실패");
    return;
  }

  // TX Task 생성
  ret = xTaskCreate(lora_tx_task, "lora_tx", 1024,
                    NULL, tskIDLE_PRIORITY + 3, &instance.lora.tx_task);
  if (ret != pdPASS)
  {
    LOG_ERR("LORA TX Task 생성 실패");
    return;
  }

  instance.lora.initialized = true;
  instance.enabled = true;

  LOG_INFO("LORA 인스턴스 초기화 완료");
}

void lora_instance_deinit(void)
{
  LOG_INFO("LoRa 인스턴스 중지 시작...");

  // 초기화되지 않은 상태면 무시
  if (!instance.lora.initialized)
  {
    LOG_WARN("LoRa 인스턴스가 초기화되지 않았습니다");
    return;
  }

  // lora_deinit 호출 (태스크, 큐, 뮤텍스 정리)
  lora_deinit(&instance.lora);

  // UART 비활성화
  lora_port_stop(&instance.lora);

  // 앱 레벨 상태 초기화
  instance.enabled = false;

  led_set_color(3, LED_COLOR_NONE);
  led_set_state(3, false);

  LOG_INFO("LoRa 인스턴스 중지 완료");
}

/*===========================================================================
 * 명령어 전송 API
 *===========================================================================*/

bool lora_send_command_sync(const char *cmd, uint32_t timeout_ms)
{
  if (!instance.lora.initialized)
  {
    LOG_ERR("LoRa not initialized");
    return false;
  }

  if (!cmd || strlen(cmd) == 0)
  {
    LOG_ERR("Empty command");
    return false;
  }

  // 세마포어 생성
  SemaphoreHandle_t response_sem = xSemaphoreCreateBinary();
  if (response_sem == NULL)
  {
    LOG_ERR("Failed to create semaphore");
    return false;
  }

  bool result = false;

  // 명령어 요청 구조체 생성 (동기 방식)
  lora_cmd_request_t cmd_req = {
      .timeout_ms = timeout_ms,
      .is_async = false,
      .skip_response = false,
      .response_sem = response_sem,
      .result = &result,
      .callback = NULL,
      .user_data = NULL,
  };

  strncpy(cmd_req.cmd, cmd, sizeof(cmd_req.cmd) - 1);
  cmd_req.cmd[sizeof(cmd_req.cmd) - 1] = '\0';

  // TX 태스크로 명령어 전송 요청
  if (xQueueSend(instance.lora.cmd_queue, &cmd_req, pdMS_TO_TICKS(1000)) != pdTRUE)
  {
    LOG_ERR("Failed to send command to TX task");
    vSemaphoreDelete(response_sem);
    return false;
  }

  // TX 태스크에서 처리 완료 대기
  if (xSemaphoreTake(response_sem, pdMS_TO_TICKS(timeout_ms + 1000)) == pdTRUE)
  {
    // 처리 완료
    vSemaphoreDelete(response_sem);
    return result;
  }
  else
  {
    // 외부 타임아웃 (TX 태스크 응답 없음)
    LOG_ERR("TX task did not respond");
    vSemaphoreDelete(response_sem);
    return false;
  }
}

bool lora_send_command_async(const char *cmd, uint32_t timeout_ms, uint32_t toa_ms,
                             lora_cmd_callback_t callback, void *user_data,
                             bool skip_response)
{
  if (!instance.lora.initialized)
  {
    LOG_ERR("LoRa not initialized");
    return false;
  }

  if (!cmd || strlen(cmd) == 0)
  {
    LOG_ERR("Empty command");
    return false;
  }

  // 세마포어 생성 (TX Task 내부에서 응답 대기용)
  SemaphoreHandle_t response_sem = xSemaphoreCreateBinary();
  if (response_sem == NULL)
  {
    LOG_ERR("Failed to create semaphore");
    return false;
  }

  // 명령어 요청 구조체 생성 (비동기 방식)
  lora_cmd_request_t cmd_req = {
      .timeout_ms = timeout_ms,
      .toa_ms = toa_ms,
      .is_async = true,
      .skip_response = skip_response,
      .response_sem = response_sem,
      .result = NULL,
      .callback = callback,
      .user_data = user_data,
      .async_result = false,
  };

  strncpy(cmd_req.cmd, cmd, sizeof(cmd_req.cmd) - 1);
  cmd_req.cmd[sizeof(cmd_req.cmd) - 1] = '\0';

  // TX 태스크로 명령어 전송 요청
  if (xQueueSend(instance.lora.cmd_queue, &cmd_req, pdMS_TO_TICKS(1000)) != pdTRUE)
  {
    LOG_ERR("Failed to send command to TX task");
    vSemaphoreDelete(response_sem);
    return false;
  }

  // 즉시 반환 (non-blocking)
  LOG_INFO("Async command queued");
  return true;
}

/*===========================================================================
 * P2P 설정 API
 *===========================================================================*/

bool lora_set_work_mode(lora_work_mode_t mode, uint32_t timeout_ms)
{
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "at+set_config=lora:work_mode:%d\r\n", mode);
  return lora_send_command_sync(cmd, timeout_ms);
}

bool lora_set_p2p_config(uint32_t freq, uint8_t sf, uint8_t bw, uint8_t cr,
                         uint16_t preamlen, uint8_t pwr, uint32_t timeout_ms)
{
  char cmd[128];
  snprintf(cmd, sizeof(cmd),
           "at+set_config=lorap2p:%lu:%d:%d:%d:%d:%d\r\n",
           freq, sf, bw, cr, preamlen, pwr);
  return lora_send_command_sync(cmd, timeout_ms);
}

bool lora_set_p2p_transfer_mode(lora_p2p_transfer_mode_t mode, uint32_t timeout_ms)
{
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "at+set_config=lorap2p:transfer_mode:%d\r\n", mode);
  return lora_send_command_sync(cmd, timeout_ms);
}

/*===========================================================================
 * P2P 데이터 전송 API
 *===========================================================================*/

bool lora_send_p2p_data(const char *data, uint32_t timeout_ms)
{
  if (!data)
  {
    LOG_ERR("NULL data");
    return false;
  }

  char cmd[512];
  snprintf(cmd, sizeof(cmd), "at+send=lorap2p:%s\r\n", data);
  return lora_send_command_sync(cmd, timeout_ms);
}

bool lora_send_p2p_raw(const uint8_t *data, size_t len, uint32_t timeout_ms)
{
  if (!instance.lora.initialized)
  {
    LOG_ERR("LoRa not initialized");
    return false;
  }

  if (!data || len == 0)
  {
    LOG_ERR("NULL data or zero length");
    return false;
  }

  // HEX conversion doubles the size, so max binary is 118 bytes (-> 236 HEX chars)
  if (len > 118)
  {
    LOG_ERR("Data too large: %d > 118 (max binary for HEX ASCII)", len);
    return false;
  }

  // Convert binary data to HEX ASCII string
  // Each byte becomes 2 HEX characters, plus null terminator
  char hex_string[512];  // 236 * 2 + 1 = 473 bytes max
  if (len * 2 >= sizeof(hex_string))
  {
    LOG_ERR("HEX string buffer too small");
    return false;
  }

  for (size_t i = 0; i < len; i++)
  {
    snprintf(&hex_string[i * 2], 3, "%02X", data[i]);
  }
  hex_string[len * 2] = '\0';

  LOG_INFO("Sending raw P2P data: %d bytes -> %d HEX chars", len, len * 2);
  if (len >= 4) {
    LOG_INFO("First 4 bytes (binary): %02X %02X %02X %02X", data[0], data[1], data[2], data[3]);
    LOG_INFO("First 8 HEX chars: %.8s", hex_string);
  }

  // Create AT command: at+send=lorap2p:<HEX_STRING>\r\n
  char cmd[600];
  snprintf(cmd, sizeof(cmd), "at+send=lorap2p:%s\r\n", hex_string);

  // Use the standard command sending mechanism
  return lora_send_command_sync(cmd, timeout_ms);
}

bool lora_send_p2p_raw_async(const uint8_t *data, size_t len, uint32_t timeout_ms,
                              lora_cmd_callback_t callback, void *user_data)
{
  if (!instance.lora.initialized)
  {
    LOG_ERR("LoRa not initialized");
    return false;
  }

  if (!data || len == 0)
  {
    LOG_ERR("NULL data or zero length");
    return false;
  }

  // HEX conversion doubles the size, so max binary is 118 bytes (-> 236 HEX chars)
  if (len > 118)
  {
    LOG_ERR("Data too large: %d > 118 (max binary for HEX ASCII)", len);
    return false;
  }

  // Convert binary data to HEX ASCII string
  // Each byte becomes 2 HEX characters, plus null terminator
  char hex_string[300];  // 118 * 2 + 1 = 237 bytes max
  if (len * 2 >= sizeof(hex_string))
  {
    LOG_ERR("HEX string buffer too small");
    return false;
  }

  for (size_t i = 0; i < len; i++)
  {
    snprintf(&hex_string[i * 2], 3, "%02X", data[i]);
  }
  hex_string[len * 2] = '\0';

  LOG_INFO("Sending raw P2P data (async): %d bytes -> %d HEX chars", len, len * 2);
  if (len >= 4) {
    LOG_INFO("First 4 bytes (binary): %02X %02X %02X %02X", data[0], data[1], data[2], data[3]);
  }

  uint32_t toa_ms = 0;

  // Create AT command: at+send=lorap2p:<HEX_STRING>\r\n
  char cmd[300];
  snprintf(cmd, sizeof(cmd), "at+send=lorap2p:%s\r\n", hex_string);

  // Use async command sending mechanism
  return lora_send_command_async(cmd, timeout_ms, toa_ms, callback, user_data, false);
}

void lora_set_p2p_recv_callback(lora_p2p_recv_callback_t callback, void *user_data)
{
  instance.lora.p2p_recv_callback = callback;
  instance.lora.p2p_recv_user_data = user_data;
}

/*===========================================================================
 * 핸들/인스턴스 API
 *===========================================================================*/

lora_t *lora_get_handle(void)
{
  return &instance.lora;
}

lora_t *lora_app_get_handle(void)
{
  if (!instance.enabled) {
    return NULL;
  }
  return &instance.lora;
}

lora_app_t *lora_app_get_instance(void)
{
  return &instance;
}

/*===========================================================================
 * 하위 호환성 API
 *===========================================================================*/

void lora_start_rover(void)
{
  LOG_INFO("Rover 모드 LoRa 시작");
  if (!instance.lora.initialized)
  {
    lora_instance_init();
    LOG_INFO("LoRa 초기화 완료");
  }
  else
  {
    LOG_WARN("LORA가 이미 초기화 되어 있습니다");
  }
}

/*===========================================================================
 * LoRa 앱 시작/종료 API (BLE 스타일)
 *===========================================================================*/

void lora_app_start(void)
{
  const board_config_t *config = board_get_config();

  LOG_INFO("LoRa 앱 시작 - 모드: %s",
           config->lora_mode == LORA_MODE_BASE ? "BASE" : "ROVER");

  /* 인스턴스 초기화 */
  if (!instance.lora.initialized)
  {
    lora_instance_init();
  }

  /* Base 모드일 때만 RTCM 이벤트 구독 */
  if (config->lora_mode == LORA_MODE_BASE)
  {
    event_bus_subscribe(EVENT_RTCM_FOR_LORA, on_rtcm_for_lora, NULL);
    LOG_INFO("RTCM 이벤트 구독 완료");
  }

  LOG_INFO("LoRa 앱 시작 완료");
}

void lora_app_stop(void)
{
  const board_config_t *config = board_get_config();

  LOG_INFO("LoRa 앱 종료 시작");

  /* 이벤트 구독 해제 */
  if (config->lora_mode == LORA_MODE_BASE)
  {
    event_bus_unsubscribe(EVENT_RTCM_FOR_LORA, on_rtcm_for_lora);
  }

  /* 인스턴스 정리 */
  lora_instance_deinit();

  LOG_INFO("LoRa 앱 종료 완료");
}

/*===========================================================================
 * 앱 레벨 래퍼 API
 *===========================================================================*/

bool lora_app_is_ready(void)
{
  return lora_is_ready(&instance.lora);
}

bool lora_app_send_p2p_raw(const uint8_t *data, size_t len, uint32_t timeout_ms,
                           lora_cmd_callback_t callback, void *user_data)
{
  return lora_send_p2p_raw_async(data, len, timeout_ms, callback, user_data);
}

bool lora_app_send_cmd_sync(const char *cmd, uint32_t timeout_ms)
{
  return lora_send_command_sync(cmd, timeout_ms);
}

bool lora_app_send_cmd_async(const char *cmd, uint32_t timeout_ms, uint32_t toa_ms,
                             lora_cmd_callback_t callback, void *user_data,
                             bool skip_response)
{
  return lora_send_command_async(cmd, timeout_ms, toa_ms, callback, user_data, skip_response);
}

bool lora_app_set_work_mode(lora_work_mode_t mode, uint32_t timeout_ms)
{
  return lora_set_work_mode(mode, timeout_ms);
}

bool lora_app_set_p2p_config(uint32_t freq, uint8_t sf, uint8_t bw, uint8_t cr,
                             uint16_t preamlen, uint8_t pwr, uint32_t timeout_ms)
{
  return lora_set_p2p_config(freq, sf, bw, cr, preamlen, pwr, timeout_ms);
}

bool lora_app_set_p2p_transfer_mode(lora_p2p_transfer_mode_t mode, uint32_t timeout_ms)
{
  return lora_set_p2p_transfer_mode(mode, timeout_ms);
}

void lora_app_set_p2p_recv_callback(lora_p2p_recv_callback_t callback, void *user_data)
{
  lora_set_p2p_recv_callback(callback, user_data);
}

/*===========================================================================
 * 이벤트 핸들러 구현
 *===========================================================================*/

/**
 * @brief RTCM 전송 이벤트 핸들러
 *
 * GPS에서 RTCM 수신 시 이벤트 버스를 통해 호출됨
 */
static void on_rtcm_for_lora(const event_t *event, void *user_data)
{
  (void)user_data;

  if (!instance.lora.initialized || !instance.lora.init_complete)
  {
    LOG_WARN("LoRa not ready, skipping RTCM");
    return;
  }

  uint8_t gps_id = event->data.rtcm.gps_id;
  gps_t *gps = gps_get_instance_handle(gps_id);

  if (gps == NULL)
  {
    LOG_ERR("GPS 인스턴스 없음 (id=%d)", gps_id);
    return;
  }

  /* RTCM 데이터를 LoRa로 전송 */
  rtcm_send_to_lora(gps);
}
