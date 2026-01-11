/**
 * @file ble_parser.c
 * @brief BLE AT 응답 파서 구현
 */

#include "ble_parser.h"
#include <string.h>

/*===========================================================================
 * 내부 함수 선언
 *===========================================================================*/
static ble_parse_result_t ble_parser_classify_line(const char *line);

/*===========================================================================
 * API 구현
 *===========================================================================*/

void ble_parser_init(ble_parser_ctx_t *ctx)
{
    if (!ctx) return;

    ctx->state = BLE_PARSE_STATE_IDLE;
    ctx->pos = 0;
    ctx->buf[0] = '\0';
}

void ble_parser_reset(ble_parser_ctx_t *ctx)
{
    if (!ctx) return;

    ctx->state = BLE_PARSE_STATE_IDLE;
    ctx->pos = 0;
    ctx->buf[0] = '\0';
}

ble_parse_result_t ble_parser_process_byte(ble_parser_ctx_t *ctx, uint8_t byte)
{
    if (!ctx) return BLE_PARSE_RESULT_NONE;

    /* CR/LF 처리 - 라인 종료 */
    if (byte == '\r' || byte == '\n') {
        if (ctx->pos > 0) {
            /* 라인 완성 - 분류 */
            ctx->buf[ctx->pos] = '\0';
            ble_parse_result_t result = ble_parser_classify_line(ctx->buf);

            /* 상태 리셋 (다음 라인 준비) */
            ctx->state = BLE_PARSE_STATE_IDLE;
            ctx->pos = 0;

            return result;
        }
        /* 빈 라인 무시 */
        return BLE_PARSE_RESULT_NONE;
    }

    /* 버퍼 오버플로우 체크 */
    if (ctx->pos >= BLE_PARSER_BUF_SIZE - 1) {
        ctx->state = BLE_PARSE_STATE_IDLE;
        ctx->pos = 0;
        ctx->buf[0] = '\0';
        return BLE_PARSE_RESULT_OVERFLOW;
    }

    /* 첫 바이트에서 상태 결정 */
    if (ctx->state == BLE_PARSE_STATE_IDLE) {
        if (byte == '+') {
            ctx->state = BLE_PARSE_STATE_AT_RESP;
        } else if (byte == 'A') {
            ctx->state = BLE_PARSE_STATE_AT_CMD;
        } else {
            ctx->state = BLE_PARSE_STATE_APP_CMD;
        }
    }

    /* AT 명령어 상태에서 'T' 확인 */
    if (ctx->state == BLE_PARSE_STATE_AT_CMD && ctx->pos == 1) {
        if (byte != 'T') {
            /* AT가 아니면 앱 명령어로 전환 */
            ctx->state = BLE_PARSE_STATE_APP_CMD;
        }
    }

    /* 버퍼에 추가 */
    ctx->buf[ctx->pos++] = (char)byte;
    ctx->buf[ctx->pos] = '\0';

    return BLE_PARSE_RESULT_NONE;
}

size_t ble_parser_process(ble_parser_ctx_t *ctx, const uint8_t *data, size_t len,
                          ble_parse_result_t *result)
{
    if (!ctx || !data || len == 0) {
        if (result) *result = BLE_PARSE_RESULT_NONE;
        return 0;
    }

    size_t processed = 0;

    for (size_t i = 0; i < len; i++) {
        ble_parse_result_t r = ble_parser_process_byte(ctx, data[i]);
        processed++;

        if (r != BLE_PARSE_RESULT_NONE) {
            if (result) *result = r;
            return processed;
        }
    }

    if (result) *result = BLE_PARSE_RESULT_NONE;
    return processed;
}

const char *ble_parser_get_line(const ble_parser_ctx_t *ctx)
{
    if (!ctx) return "";
    return ctx->buf;
}

const char *ble_parser_get_at_param(const char *line)
{
    if (!line) return NULL;

    /* ':' 또는 '=' 찾기 */
    const char *sep = strchr(line, ':');
    if (!sep) {
        sep = strchr(line, '=');
    }

    if (sep) {
        return sep + 1;
    }

    return NULL;
}

/*===========================================================================
 * 내부 함수 구현
 *===========================================================================*/

/**
 * @brief 파싱된 라인을 분류
 */
static ble_parse_result_t ble_parser_classify_line(const char *line)
{
    if (!line || line[0] == '\0') {
        return BLE_PARSE_RESULT_NONE;
    }

    /* AT 응답 (+로 시작) */
    if (line[0] == '+') {
        if (strncmp(line, "+OK", 3) == 0) {
            return BLE_PARSE_RESULT_AT_OK;
        }
        if (strncmp(line, "+ERROR", 6) == 0 || strncmp(line, "+ERR", 4) == 0) {
            return BLE_PARSE_RESULT_AT_ERROR;
        }
        if (strncmp(line, "+READY", 6) == 0) {
            return BLE_PARSE_RESULT_AT_READY;
        }
        if (strncmp(line, "+CONNECTED", 10) == 0) {
            return BLE_PARSE_RESULT_AT_CONNECTED;
        }
        if (strncmp(line, "+DISCONNECTED", 13) == 0) {
            return BLE_PARSE_RESULT_AT_DISCONNECTED;
        }
        if (strncmp(line, "+ADVERTISING", 12) == 0) {
            return BLE_PARSE_RESULT_AT_ADVERTISING;
        }
        /* 기타 AT 응답 (+MANUF:xxx 등) */
        return BLE_PARSE_RESULT_AT_OTHER;
    }

    /* 앱 명령어 (SD+, SC+, SM+ 등) */
    return BLE_PARSE_RESULT_APP_CMD;
}
