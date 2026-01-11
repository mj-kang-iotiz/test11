#ifndef BLE_PARSER_H
#define BLE_PARSER_H

/**
 * @file ble_parser.h
 * @brief BLE AT 응답 파서
 *
 * BLE 모듈로부터 수신된 데이터 파싱
 * - AT 응답: +OK, +ERROR, +READY, +CONNECTED, +DISCONNECTED 등
 * - 앱 명령어: SD+, SC+, SM+ 등 (외부 PC로부터)
 */

#include "ble_types.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*===========================================================================
 * 파서 컨텍스트
 *===========================================================================*/
typedef struct {
    ble_parse_state_t state;        /**< 현재 파싱 상태 */
    char buf[BLE_PARSER_BUF_SIZE];  /**< 파싱 버퍼 */
    size_t pos;                     /**< 현재 버퍼 위치 */
} ble_parser_ctx_t;

/*===========================================================================
 * 파싱 결과
 *===========================================================================*/
typedef enum {
    BLE_PARSE_RESULT_NONE,          /**< 파싱 중 (더 많은 데이터 필요) */
    BLE_PARSE_RESULT_AT_OK,         /**< +OK */
    BLE_PARSE_RESULT_AT_ERROR,      /**< +ERROR */
    BLE_PARSE_RESULT_AT_READY,      /**< +READY */
    BLE_PARSE_RESULT_AT_CONNECTED,  /**< +CONNECTED */
    BLE_PARSE_RESULT_AT_DISCONNECTED, /**< +DISCONNECTED */
    BLE_PARSE_RESULT_AT_ADVERTISING,  /**< +ADVERTISING */
    BLE_PARSE_RESULT_AT_OTHER,      /**< 기타 AT 응답 (+xxx) */
    BLE_PARSE_RESULT_APP_CMD,       /**< 앱 명령어 (SD+, SC+ 등) */
    BLE_PARSE_RESULT_OVERFLOW,      /**< 버퍼 오버플로우 */
} ble_parse_result_t;

/*===========================================================================
 * API
 *===========================================================================*/

/**
 * @brief 파서 초기화
 * @param ctx 파서 컨텍스트
 */
void ble_parser_init(ble_parser_ctx_t *ctx);

/**
 * @brief 파서 리셋
 * @param ctx 파서 컨텍스트
 */
void ble_parser_reset(ble_parser_ctx_t *ctx);

/**
 * @brief 데이터 파싱 (바이트 단위)
 * @param ctx 파서 컨텍스트
 * @param byte 수신 바이트
 * @return 파싱 결과
 */
ble_parse_result_t ble_parser_process_byte(ble_parser_ctx_t *ctx, uint8_t byte);

/**
 * @brief 데이터 파싱 (버퍼 단위)
 * @param ctx 파서 컨텍스트
 * @param data 수신 데이터
 * @param len 데이터 길이
 * @param result 파싱 결과 (OUT, NULL 가능)
 * @return 파싱된 바이트 수
 */
size_t ble_parser_process(ble_parser_ctx_t *ctx, const uint8_t *data, size_t len,
                          ble_parse_result_t *result);

/**
 * @brief 파싱된 라인 가져오기
 * @param ctx 파서 컨텍스트
 * @return 파싱된 문자열 (널 종료)
 */
const char *ble_parser_get_line(const ble_parser_ctx_t *ctx);

/**
 * @brief 파싱 결과가 AT 응답인지 확인
 * @param result 파싱 결과
 * @return true: AT 응답
 */
static inline bool ble_parse_result_is_at(ble_parse_result_t result)
{
    return (result >= BLE_PARSE_RESULT_AT_OK && result <= BLE_PARSE_RESULT_AT_OTHER);
}

/**
 * @brief AT 응답에서 파라미터 추출
 *
 * 예: "+MANUF:TEST_DEVICE" → "TEST_DEVICE"
 *
 * @param line 파싱된 라인
 * @return 파라미터 시작 위치 (없으면 NULL)
 */
const char *ble_parser_get_at_param(const char *line);

#endif /* BLE_PARSER_H */
