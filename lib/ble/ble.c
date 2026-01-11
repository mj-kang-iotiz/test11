/**
 * @file ble.c
 * @brief BLE 핵심 로직 구현
 */

#include "ble.h"
#include <string.h>

/*===========================================================================
 * 내부 함수 선언
 *===========================================================================*/
static void ble_process_parsed_result(ble_t *ble, ble_parse_result_t result);

/*===========================================================================
 * 초기화 API
 *===========================================================================*/

bool ble_init(ble_t *ble, const ble_hal_ops_t *ops)
{
    if (!ble || !ops) {
        return false;
    }

    memset(ble, 0, sizeof(ble_t));

    ble->ops = ops;
    ble->mode = BLE_MODE_BYPASS;
    ble->conn_state = BLE_CONN_DISCONNECTED;
    ble->ready = false;
    ble->handler = NULL;
    ble->user_data = NULL;

    /* 링버퍼 초기화 (GPS처럼) */
    ringbuffer_init(&ble->rx_buf, ble->rx_buf_mem, sizeof(ble->rx_buf_mem));

    /* 파서 초기화 */
    ble_parser_init(&ble->parser_ctx);

    /* AT 요청 초기화 */
    ble->at_request.active = false;
    ble->at_request.sem = NULL;
    ble->at_request.status = BLE_AT_STATUS_IDLE;

    /* 뮤텍스 초기화 */
    ble->mutex = xSemaphoreCreateMutex();
    if (!ble->mutex) {
        return false;
    }

    /* AT 요청용 세마포어 생성 */
    ble->at_request.sem = xSemaphoreCreateBinary();
    if (!ble->at_request.sem) {
        vSemaphoreDelete(ble->mutex);
        return false;
    }

    return true;
}

void ble_set_evt_handler(ble_t *ble, ble_evt_handler_t handler, void *user_data)
{
    if (!ble) return;

    ble->handler = handler;
    ble->user_data = user_data;
}

/*===========================================================================
 * 링버퍼 API
 *===========================================================================*/

void ble_rx_write(ble_t *ble, const uint8_t *data, size_t len)
{
    if (!ble || !data || len == 0) {
        return;
    }

    ringbuffer_write(&ble->rx_buf, (const char *)data, len);
}

ringbuffer_t *ble_get_rx_buf(ble_t *ble)
{
    if (!ble) return NULL;
    return &ble->rx_buf;
}

/*===========================================================================
 * 데이터 송수신 API
 *===========================================================================*/

bool ble_send(ble_t *ble, const char *data, size_t len)
{
    if (!ble || !data || len == 0) {
        return false;
    }

    if (!ble->ops || !ble->ops->send) {
        return false;
    }

    return (ble->ops->send(data, len) == 0);
}

void ble_process_rx(ble_t *ble)
{
    if (!ble) {
        return;
    }

    /* 링버퍼에서 1바이트씩 읽어서 파싱 */
    char byte;
    while (ringbuffer_read_byte(&ble->rx_buf, &byte)) {
        ble_parse_result_t result = ble_parser_process_byte(&ble->parser_ctx, (uint8_t)byte);

        if (result != BLE_PARSE_RESULT_NONE) {
            ble_process_parsed_result(ble, result);
        }
    }
}

/*===========================================================================
 * AT 명령어 API
 *===========================================================================*/

bool ble_enter_at_mode(ble_t *ble)
{
    if (!ble || !ble->ops || !ble->ops->at_mode) {
        return false;
    }

    ble->ops->at_mode();
    ble->mode = BLE_MODE_AT;
    return true;
}

bool ble_enter_bypass_mode(ble_t *ble)
{
    if (!ble || !ble->ops || !ble->ops->bypass_mode) {
        return false;
    }

    ble->ops->bypass_mode();
    ble->mode = BLE_MODE_BYPASS;
    return true;
}

ble_at_status_t ble_send_at_cmd_sync(ble_t *ble, const char *cmd,
                                      char *response, size_t response_size,
                                      uint32_t timeout_ms)
{
    if (!ble || !cmd) {
        return BLE_AT_STATUS_ERROR;
    }

    /* 뮤텍스 획득 */
    if (xSemaphoreTake(ble->mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return BLE_AT_STATUS_TIMEOUT;
    }

    /* 이미 활성 요청이 있으면 에러 */
    if (ble->at_request.active) {
        xSemaphoreGive(ble->mutex);
        return BLE_AT_STATUS_ERROR;
    }

    /* AT 요청 설정 */
    strncpy(ble->at_request.expected, "+OK", sizeof(ble->at_request.expected) - 1);
    ble->at_request.response[0] = '\0';
    ble->at_request.response_len = 0;
    ble->at_request.status = BLE_AT_STATUS_PENDING;
    ble->at_request.active = true;

    /* 세마포어 초기화 (이전 신호 제거) */
    xSemaphoreTake(ble->at_request.sem, 0);

    /* AT 모드 전환 */
    ble_enter_at_mode(ble);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 명령 전송 */
    if (!ble_send(ble, cmd, strlen(cmd))) {
        ble->at_request.active = false;
        ble_enter_bypass_mode(ble);
        xSemaphoreGive(ble->mutex);
        return BLE_AT_STATUS_ERROR;
    }

    /* 뮤텍스 해제 (응답 수신 허용) */
    xSemaphoreGive(ble->mutex);

    /* 응답 대기 */
    BaseType_t result = xSemaphoreTake(ble->at_request.sem, pdMS_TO_TICKS(timeout_ms));

    /* 결과 처리 */
    xSemaphoreTake(ble->mutex, portMAX_DELAY);

    ble_at_status_t status;
    if (result != pdTRUE) {
        status = BLE_AT_STATUS_TIMEOUT;
    } else {
        status = ble->at_request.status;
    }

    /* 응답 복사 */
    if (response && response_size > 0) {
        strncpy(response, ble->at_request.response, response_size - 1);
        response[response_size - 1] = '\0';
    }

    ble->at_request.active = false;

    /* Bypass 모드 복귀 */
    ble_enter_bypass_mode(ble);

    xSemaphoreGive(ble->mutex);

    return status;
}

/*===========================================================================
 * 상태 조회 API
 *===========================================================================*/

ble_mode_t ble_get_mode(const ble_t *ble)
{
    if (!ble) return BLE_MODE_BYPASS;
    return ble->mode;
}

ble_conn_state_t ble_get_conn_state(const ble_t *ble)
{
    if (!ble) return BLE_CONN_DISCONNECTED;
    return ble->conn_state;
}

void ble_set_conn_state(ble_t *ble, ble_conn_state_t state)
{
    if (!ble) return;
    ble->conn_state = state;
}

bool ble_is_ready(const ble_t *ble)
{
    if (!ble) return false;
    return ble->ready;
}

/*===========================================================================
 * 내부 API
 *===========================================================================*/

void ble_handle_at_response(ble_t *ble, ble_parse_result_t result, const char *line)
{
    if (!ble || !line) return;

    /* 동기 AT 명령 대기 중인지 확인 */
    if (!ble->at_request.active) {
        return;
    }

    if (ble->at_request.status != BLE_AT_STATUS_PENDING) {
        return;
    }

    /* 응답 저장 */
    strncpy(ble->at_request.response, line, sizeof(ble->at_request.response) - 1);
    ble->at_request.response[sizeof(ble->at_request.response) - 1] = '\0';
    ble->at_request.response_len = strlen(ble->at_request.response);

    /* 상태 업데이트 */
    switch (result) {
        case BLE_PARSE_RESULT_AT_OK:
        case BLE_PARSE_RESULT_AT_OTHER:  /* 기타 응답도 성공으로 처리 */
            ble->at_request.status = BLE_AT_STATUS_COMPLETED;
            break;

        case BLE_PARSE_RESULT_AT_ERROR:
            ble->at_request.status = BLE_AT_STATUS_ERROR;
            break;

        default:
            /* 연결/연결해제 등은 응답으로 처리하지 않음 */
            return;
    }

    /* 세마포어 해제 (대기 중인 태스크 깨우기) */
    xSemaphoreGive(ble->at_request.sem);
}

/*===========================================================================
 * 내부 함수 구현
 *===========================================================================*/

/**
 * @brief 파싱 결과 처리
 *
 * 파싱 완료 시 호출, AT 응답 처리 및 이벤트 발생
 */
static void ble_process_parsed_result(ble_t *ble, ble_parse_result_t result)
{
    if (!ble) return;

    /* 파싱된 라인 복사 */
    strncpy(ble->line_buf, ble_parser_get_line(&ble->parser_ctx),
            sizeof(ble->line_buf) - 1);
    ble->line_buf[sizeof(ble->line_buf) - 1] = '\0';

    /* AT 응답이면서 AT 모드일 때만 AT 응답 처리 */
    if (ble_parse_result_is_at(result)) {
        ble_handle_at_response(ble, result, ble->line_buf);
    }

    /* 이벤트 발생 */
    if (ble->handler) {
        ble_event_t event = {0};
        event.timestamp_ms = xTaskGetTickCount();

        switch (result) {
            case BLE_PARSE_RESULT_AT_OK:
                event.type = BLE_EVENT_AT_OK;
                strncpy(event.data.at_response.response, ble->line_buf,
                        sizeof(event.data.at_response.response) - 1);
                event.data.at_response.len = strlen(ble->line_buf);
                break;

            case BLE_PARSE_RESULT_AT_ERROR:
                event.type = BLE_EVENT_AT_ERROR;
                strncpy(event.data.at_response.response, ble->line_buf,
                        sizeof(event.data.at_response.response) - 1);
                event.data.at_response.len = strlen(ble->line_buf);
                break;

            case BLE_PARSE_RESULT_AT_READY:
                event.type = BLE_EVENT_AT_READY;
                ble->ready = true;
                break;

            case BLE_PARSE_RESULT_AT_CONNECTED:
                event.type = BLE_EVENT_CONNECTED;
                ble->conn_state = BLE_CONN_CONNECTED;
                break;

            case BLE_PARSE_RESULT_AT_DISCONNECTED:
                event.type = BLE_EVENT_DISCONNECTED;
                ble->conn_state = BLE_CONN_DISCONNECTED;
                break;

            case BLE_PARSE_RESULT_AT_ADVERTISING:
                event.type = BLE_EVENT_ADVERTISING;
                break;

            case BLE_PARSE_RESULT_AT_OTHER:
                event.type = BLE_EVENT_AT_OK;
                strncpy(event.data.at_response.response, ble->line_buf,
                        sizeof(event.data.at_response.response) - 1);
                event.data.at_response.len = strlen(ble->line_buf);
                break;

            case BLE_PARSE_RESULT_APP_CMD:
                /* Bypass 모드에서만 앱 명령어 처리 */
                if (ble->mode == BLE_MODE_BYPASS) {
                    event.type = BLE_EVENT_APP_CMD;
                    event.data.app_cmd.cmd = ble->line_buf;
                    event.data.app_cmd.param = NULL;
                } else {
                    return;  /* AT 모드에서는 무시 */
                }
                break;

            default:
                return;
        }

        ble->handler(ble, &event);
    }
}
