#include "rtcm.h"
#include "gps.h"
#include "gps_parser.h"
#include "gps_proto_def.h"
#include "lora_app.h"
#include "dev_assert.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

#ifndef TAG
  #define TAG "RTCM"
#endif

#include "log.h"

/*===========================================================================
 * X-Macro 기반 문자열 변환 함수
 *===========================================================================*/
static const char* rtcm_msg_to_str(uint16_t msg_type) {
    if (msg_type == 0) return "NONE";

    switch (msg_type) {
#define X(name, type, description) case type: return description;
        RTCM_MSG_TABLE(X)
#undef X
        default: return "UNKNOWN";
    }
}

/*===========================================================================
 * RTCM 상수
 *===========================================================================*/
#define RTCM_PREAMBLE       0xD3
#define RTCM_HEADER_SIZE    3     /* preamble(1) + length(2) */
#define RTCM_CRC_SIZE       3     /* CRC24Q */
#define RTCM_MIN_PACKET     6     /* header(3) + CRC(3) */
#define RTCM_MAX_PAYLOAD    1023  /* 10-bit length field max */

// HEX ASCII로 변환하면 데이터가 2배 증가:
// LoRa 최대 236 HEX 문자 = 118 바이트 binary
#define RTCM_MAX_FRAGMENT_SIZE 118  // Max binary size per fragment

// LoRa Time on Air calculation (SF7, BW125, CR4/5, Preamble 8)
// HEX 변환: 1 byte -> 2 HEX chars
// 최대 118바이트 = 236 HEX chars = 350ms (측정값)
// 20% 여유: 350ms * 1.2 = 420ms
#define RTCM_MAX_BINARY_BYTES 118   // 최대 binary 크기
#define LORA_TOA_MAX_MS 350         // 118바이트(236 HEX) 전송 시간
#define LORA_TOA_MARGIN_PERCENT 20  // 20% margin

/**
 * @brief Calculate LoRa Time on Air (ToA) with 20% margin
 *
 * Formula: ToA = (bytes / 118) * 350ms * 1.2
 *
 * @param binary_bytes Binary payload size (before HEX conversion)
 * @return Time on Air in milliseconds (with 20% margin)
 */
static uint32_t calculate_lora_toa(size_t binary_bytes) {
  // ToA(ms) = (bytes / 118) * 350 * 1.2
  uint32_t toa_ms = (binary_bytes * LORA_TOA_MAX_MS / RTCM_MAX_BINARY_BYTES);
  toa_ms = toa_ms * (100 + LORA_TOA_MARGIN_PERCENT) / 100;

  // Minimum ToA
  if (toa_ms < 60) {
    toa_ms = 60;  // 최소 60ms (20% margin 포함)
  }

  return toa_ms;
}

/**
 * @brief Callback for last fragment transmission completion
 */
static void rtcm_last_fragment_callback(bool success, void *user_data) {
  uint16_t msg_type = (uint16_t)(uintptr_t)user_data;

  if (success) {
    LOG_INFO("RTCM transmission complete (type=%d)", msg_type);
  } else {
    LOG_ERR("RTCM last fragment transmission failed (type=%d)", msg_type);
  }
}

void rtcm_tx_task_init(void) {
  // No task needed anymore - direct async transmission
  LOG_INFO("RTCM async transmission initialized (no task)");
}

/**
 * @brief RTCM 데이터를 LoRa로 전송 (링버퍼에서 읽기)
 *
 * 링버퍼에서 RTCM 패킷을 하나씩 읽어서 LoRa로 전송합니다.
 * 여러 RTCM 메시지가 큐잉되어 있으면 하나씩 순차 전송됩니다.
 *
 * @param gps GPS 핸들
 * @return true: 성공, false: 실패
 */
bool rtcm_send_to_lora(gps_t *gps) {
    if (!gps) {
        LOG_ERR("GPS handle is NULL");
        return false;
    }

    /* RTCM 링버퍼에서 패킷 읽기 */
    uint8_t packet[GPS_MAX_PACKET_LEN];
    size_t rtcm_len = 0;

    if (xSemaphoreTake(gps->rtcm_data.mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        LOG_ERR("Failed to acquire RTCM mutex");
        return false;
    }

    /* 링버퍼 크기 확인 */
    if (ringbuffer_size(&gps->rtcm_data.rb) < RTCM_MIN_PACKET) {
        xSemaphoreGive(gps->rtcm_data.mutex);
        return false;  /* 데이터 없음 */
    }

    /* 헤더 peek (preamble + length) */
    uint8_t header[RTCM_HEADER_SIZE];
    if (!ringbuffer_peek(&gps->rtcm_data.rb, (char*)header, RTCM_HEADER_SIZE, 0)) {
        xSemaphoreGive(gps->rtcm_data.mutex);
        return false;
    }

    /* 페이로드 길이 추출 */
    uint16_t payload_len = ((header[1] & 0x03) << 8) | header[2];
    rtcm_len = RTCM_HEADER_SIZE + payload_len + RTCM_CRC_SIZE;

    /* 전체 패킷 읽기 */
    if (ringbuffer_size(&gps->rtcm_data.rb) < rtcm_len || rtcm_len > GPS_MAX_PACKET_LEN) {
        xSemaphoreGive(gps->rtcm_data.mutex);
        return false;
    }

    ringbuffer_read(&gps->rtcm_data.rb, (char*)packet, rtcm_len);
    xSemaphoreGive(gps->rtcm_data.mutex);

    /* 메시지 타입 추출 */
    uint16_t msg_type = 0;
    if (payload_len >= 2) {
        msg_type = (packet[3] << 4) | ((packet[4] >> 4) & 0x0F);
    }

    /* Fragment 계산 및 전송 */
    uint8_t total_fragments = (rtcm_len + RTCM_MAX_FRAGMENT_SIZE - 1) / RTCM_MAX_FRAGMENT_SIZE;

    LOG_INFO("RTCM TX: type=%d, len=%d, fragments=%d", msg_type, rtcm_len, total_fragments);

    for (uint8_t i = 0; i < total_fragments; i++) {
        size_t offset = i * RTCM_MAX_FRAGMENT_SIZE;
        size_t fragment_len = (rtcm_len - offset > RTCM_MAX_FRAGMENT_SIZE)
                              ? RTCM_MAX_FRAGMENT_SIZE
                              : (rtcm_len - offset);

        uint32_t toa_ms = calculate_lora_toa(fragment_len);

        LOG_DEBUG("Queueing fragment %d/%d: %d bytes", i + 1, total_fragments, fragment_len);

        bool is_last = (i == total_fragments - 1);
        lora_command_callback_t callback = is_last ? rtcm_last_fragment_callback : NULL;
        void *user_data = is_last ? (void*)(uintptr_t)msg_type : NULL;

        if (!lora_send_p2p_raw_async(&packet[offset], fragment_len, toa_ms, callback, user_data)) {
            LOG_ERR("Failed to queue fragment %d/%d - LoRa TX queue full?", i + 1, total_fragments);
            return false;
        }
    }

    LOG_INFO("All %d fragments queued to LoRa TX task", total_fragments);
    return true;
}

/**
 * @brief RTCM3 CRC24Q 계산
 *
 * @param buffer 데이터 버퍼
 * @param len 데이터 길이 (CRC 제외)
 * @return 24-bit CRC 값
 */
uint32_t rtcm_calc_crc(const uint8_t *buffer, size_t len)
{
  // CRC24Q polynomial: 0x1864CFB
  uint32_t crc = 0;

  for (size_t i = 0; i < len; i++)
  {
    crc ^= ((uint32_t)buffer[i]) << 16;

    for (int j = 0; j < 8; j++)
    {
      crc <<= 1;
      if (crc & 0x1000000)
      {
        crc ^= 0x1864CFB;
      }
    }
  }

  return crc & 0xFFFFFF;
}

/**
 * @brief RTCM 패킷 검증 (헤더 및 CRC)
 *
 * @param buffer RTCM 패킷 버퍼
 * @param len 패킷 전체 길이
 * @return true: 유효, false: 무효
 */
bool rtcm_validate_packet(const uint8_t *buffer, size_t len)
{
  // 최소 길이 확인 (preamble 1 + reserved+len 2 + CRC 3 = 6 bytes)
  if (len < 6)
  {
    LOG_ERR("RTCM packet too short: %d bytes", len);
    return false;
  }

  // Preamble 확인
  if (buffer[0] != 0xD3)
  {
    LOG_ERR("Invalid RTCM preamble: 0x%02X (expected 0xD3)", buffer[0]);
    return false;
  }

  // Length 파싱 (10 bits, bytes 1-2)
  uint16_t payload_len = ((buffer[1] & 0x03) << 8) | buffer[2];
  uint16_t expected_total_len = 3 + payload_len + 3; // header(3) + payload + CRC(3)

  if (len != expected_total_len)
  {
    LOG_ERR("RTCM length mismatch: got %d, expected %d (payload=%d)",
            len, expected_total_len, payload_len);
    return false;
  }

  // CRC24Q 검증
  uint32_t calculated_crc = rtcm_calc_crc(buffer, len - 3);
  uint32_t received_crc = ((uint32_t)buffer[len - 3] << 16) |
                          ((uint32_t)buffer[len - 2] << 8) |
                          buffer[len - 1];

  if (calculated_crc != received_crc)
  {
    LOG_ERR("RTCM CRC mismatch: calculated 0x%06X, received 0x%06X",
            calculated_crc, received_crc);
    return false;
  }

  LOG_INFO("RTCM packet valid: len=%d, payload=%d, CRC=0x%06X",
           len, payload_len, received_crc);
  return true;
}

/*===========================================================================
 * RTCM 패킷 파싱 (Chain 방식)
 *===========================================================================*/

parse_result_t rtcm_try_parse(gps_t *gps, ringbuffer_t *rb) {
    /* 1. 첫 바이트 확인 - 0xD3 아니면 NOT_MINE */
    uint8_t first;
    if (!ringbuffer_peek(rb, (char*)&first, 1, 0)) {
        return PARSE_NEED_MORE;
    }
    if (first != RTCM_PREAMBLE) {
        return PARSE_NOT_MINE;
    }

    /* 2. 헤더 (3바이트) peek */
    if (ringbuffer_size(rb) < RTCM_HEADER_SIZE) {
        return PARSE_NEED_MORE;
    }

    uint8_t header[RTCM_HEADER_SIZE];
    if (!ringbuffer_peek(rb, (char*)header, RTCM_HEADER_SIZE, 0)) {
        return PARSE_NEED_MORE;
    }

    /* 3. 페이로드 길이 추출 (10-bit) */
    uint16_t payload_len = ((header[1] & 0x03) << 8) | header[2];

    if (payload_len > RTCM_MAX_PAYLOAD) {
        /* 비정상적인 길이 */
        return PARSE_INVALID;
    }

    /* 4. 전체 패킷 길이 = 헤더(3) + 페이로드 + CRC(3) */
    size_t total_len = RTCM_HEADER_SIZE + payload_len + RTCM_CRC_SIZE;

    if (ringbuffer_size(rb) < total_len) {
        return PARSE_NEED_MORE;
    }

    /* 5. 전체 패킷 peek */
    uint8_t packet[GPS_MAX_PACKET_LEN];
    if (total_len > GPS_MAX_PACKET_LEN) {
        return PARSE_INVALID;
    }

    if (!ringbuffer_peek(rb, (char*)packet, total_len, 0)) {
        return PARSE_NEED_MORE;
    }

    /* 6. CRC24Q 검증 */
    uint32_t calc_crc = rtcm_calc_crc(packet, total_len - RTCM_CRC_SIZE);
    uint32_t recv_crc = ((uint32_t)packet[total_len - 3] << 16) |
                        ((uint32_t)packet[total_len - 2] << 8) |
                        packet[total_len - 1];

    if (calc_crc != recv_crc) {
        gps->parser_ctx.stats.crc_errors++;
        ringbuffer_advance(rb, total_len);
        return PARSE_INVALID;
    }

    /* 7. 메시지 타입 추출 (12-bit, 페이로드 첫 12비트) */
    uint16_t msg_type = 0;
    if (payload_len >= 2) {
        msg_type = (packet[3] << 4) | ((packet[4] >> 4) & 0x0F);
    }

    /* 8. RTCM 데이터를 링버퍼에 저장 (LoRa 전송용) */
    if (xSemaphoreTake(gps->rtcm_data.mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        /* 버퍼 공간 확인 후 쓰기 */
        if (ringbuffer_free_size(&gps->rtcm_data.rb) >= total_len) {
            ringbuffer_write(&gps->rtcm_data.rb, (char*)packet, total_len);
            gps->rtcm_data.last_msg_type = msg_type;
        } else {
            LOG_WARN("RTCM buffer full, dropped msg_type=%d", msg_type);
        }
        xSemaphoreGive(gps->rtcm_data.mutex);
    } else {
        LOG_ERR("Failed to acquire RTCM mutex");
    }

    /* 9. advance */
    ringbuffer_advance(rb, total_len);
    gps->parser_ctx.stats.rtcm_packets++;
    gps->parser_ctx.stats.last_rtcm_tick = xTaskGetTickCount();

    /* 10. RTCM 수신 이벤트 핸들러 호출 */
    if (gps->handler) {
        gps_event_t event = {
            .type = GPS_EVENT_RTCM_RECEIVED,
            .protocol = GPS_PROTOCOL_RTCM,
            .timestamp_ms = xTaskGetTickCount(),
            .data.rtcm.msg_type = msg_type,
            .data.rtcm.length = total_len,
            .source.rtcm_msg_type = msg_type
        };
        gps->handler(gps, &event);
    }

    return PARSE_OK;
}
