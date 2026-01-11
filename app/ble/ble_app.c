/**
 * @file ble_app.c
 * @brief BLE 애플리케이션 구현
 */

#include "ble_app.h"
#include "ble_port.h"
#include "ble_cmd.h"
#include "board_config.h"
#include "flash_params.h"

#include <string.h>
#include <stdio.h>

#ifndef TAG
#define TAG "BLE_APP"
#endif

#include "log.h"

/*===========================================================================
 * 정적 변수
 *===========================================================================*/
static ble_instance_t ble_instance = {0};

/*===========================================================================
 * 내부 함수 선언
 *===========================================================================*/
static void ble_rx_task(void *pvParameter);
static void ble_evt_handler(ble_t *ble, const ble_event_t *event);

/*===========================================================================
 * 앱 시작/종료 API
 *===========================================================================*/

void ble_app_start(void)
{
    const board_config_t *config = board_get_config();

    LOG_INFO("BLE 앱 시작 - 보드: %d", config->board);

    if (!config->use_ble) {
        LOG_INFO("BLE 사용 안함");
        return;
    }

    /* 포트 초기화 (HAL ops 설정, 링버퍼 초기화) */
    if (ble_port_init_instance(&ble_instance.ble) != 0) {
        LOG_ERR("BLE 포트 초기화 실패");
        return;
    }

    /* RX 큐 생성 (IDLE 인터럽트 신호용) */
    ble_instance.rx_queue = xQueueCreate(10, sizeof(uint8_t));
    if (!ble_instance.rx_queue) {
        LOG_ERR("BLE RX 큐 생성 실패");
        return;
    }

    ble_port_set_queue(ble_instance.rx_queue);

    /* BLE 핸들 포인터 전달 (ble_port에서 링버퍼 접근용) */
    ble_port_set_ble_handle(&ble_instance.ble);

    /* 이벤트 핸들러 설정 */
    ble_set_evt_handler(&ble_instance.ble, ble_evt_handler, &ble_instance);

    /* 포트 시작 (UART DMA 활성화) */
    ble_port_start(&ble_instance.ble);

    /* RX 태스크 생성 */
    BaseType_t ret = xTaskCreate(ble_rx_task, "ble_rx", 512,
                                  &ble_instance, tskIDLE_PRIORITY + 1,
                                  &ble_instance.rx_task);
    if (ret != pdPASS) {
        LOG_ERR("BLE RX 태스크 생성 실패");
        vQueueDelete(ble_instance.rx_queue);
        return;
    }

    ble_instance.enabled = true;
    ble_instance.running = true;

    LOG_INFO("BLE 앱 시작 완료");
}

void ble_app_stop(void)
{
    if (!ble_instance.enabled) {
        return;
    }

    LOG_INFO("BLE 앱 종료 시작");

    ble_instance.running = false;

    /* 포트 종료 */
    ble_port_stop(&ble_instance.ble);

    /* RX 태스크 종료 대기 */
    if (ble_instance.rx_task) {
        vTaskDelay(pdMS_TO_TICKS(100));
        vTaskDelete(ble_instance.rx_task);
        ble_instance.rx_task = NULL;
    }

    /* 큐 삭제 */
    if (ble_instance.rx_queue) {
        vQueueDelete(ble_instance.rx_queue);
        ble_instance.rx_queue = NULL;
    }

    ble_instance.enabled = false;

    LOG_INFO("BLE 앱 종료 완료");
}

/*===========================================================================
 * BLE 핸들/인스턴스 API
 *===========================================================================*/

ble_t *ble_app_get_handle(void)
{
    if (!ble_instance.enabled) {
        return NULL;
    }
    return &ble_instance.ble;
}

ble_instance_t *ble_app_get_instance(void)
{
    return &ble_instance;
}

/*===========================================================================
 * 데이터 송수신 API
 *===========================================================================*/

bool ble_app_send(const char *data, size_t len)
{
    if (!ble_instance.enabled) {
        LOG_ERR("BLE not enabled");
        return false;
    }

    return ble_send(&ble_instance.ble, data, len);
}

/*===========================================================================
 * AT 명령어 API (동기)
 *===========================================================================*/

bool ble_app_set_device_name(const char *name, uint32_t timeout_ms)
{
    if (!ble_instance.enabled || !name) {
        return false;
    }

    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+MANUF=%s\r", name);

    ble_at_status_t status = ble_send_at_cmd_sync(&ble_instance.ble, cmd,
                                                   NULL, 0, timeout_ms);

    if (status == BLE_AT_STATUS_COMPLETED) {
        LOG_INFO("디바이스 이름 설정 완료: %s", name);
        return true;
    }

    LOG_ERR("디바이스 이름 설정 실패: %d", status);
    return false;
}

bool ble_app_get_device_name(char *name_buf, size_t buf_size, uint32_t timeout_ms)
{
    if (!ble_instance.enabled || !name_buf || buf_size == 0) {
        return false;
    }

    char response[BLE_AT_RESPONSE_MAX];
    ble_at_status_t status = ble_send_at_cmd_sync(&ble_instance.ble,
                                                   "AT+MANUF?\r",
                                                   response, sizeof(response),
                                                   timeout_ms);

    if (status == BLE_AT_STATUS_COMPLETED) {
        /* 응답에서 이름 추출: +MANUF:NAME 또는 NAME */
        const char *param = ble_parser_get_at_param(response);
        if (param) {
            strncpy(name_buf, param, buf_size - 1);
            name_buf[buf_size - 1] = '\0';
        } else {
            strncpy(name_buf, response, buf_size - 1);
            name_buf[buf_size - 1] = '\0';
        }
        return true;
    }

    return false;
}

bool ble_app_set_uart_baudrate(uint32_t baudrate, uint32_t timeout_ms)
{
    if (!ble_instance.enabled) {
        return false;
    }

    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+UART=%lu\r", baudrate);

    ble_at_status_t status = ble_send_at_cmd_sync(&ble_instance.ble, cmd,
                                                   NULL, 0, timeout_ms);

    if (status == BLE_AT_STATUS_COMPLETED) {
        LOG_INFO("UART 속도 설정 완료: %lu", baudrate);
        return true;
    }

    LOG_ERR("UART 속도 설정 실패: %d", status);
    return false;
}

bool ble_app_start_advertising(uint32_t timeout_ms)
{
    if (!ble_instance.enabled) {
        return false;
    }

    ble_at_status_t status = ble_send_at_cmd_sync(&ble_instance.ble,
                                                   "AT+ADVON\r",
                                                   NULL, 0, timeout_ms);

    if (status == BLE_AT_STATUS_COMPLETED) {
        LOG_INFO("Advertising 시작 완료");
        return true;
    }

    LOG_ERR("Advertising 시작 실패: %d", status);
    return false;
}

bool ble_app_disconnect(uint32_t timeout_ms)
{
    if (!ble_instance.enabled) {
        return false;
    }

    ble_at_status_t status = ble_send_at_cmd_sync(&ble_instance.ble,
                                                   "AT+DISCONNECT\r",
                                                   NULL, 0, timeout_ms);

    if (status == BLE_AT_STATUS_COMPLETED) {
        LOG_INFO("연결 해제 완료");
        return true;
    }

    LOG_ERR("연결 해제 실패: %d", status);
    return false;
}

/*===========================================================================
 * 상태 조회 API
 *===========================================================================*/

ble_conn_state_t ble_app_get_conn_state(void)
{
    if (!ble_instance.enabled) {
        return BLE_CONN_DISCONNECTED;
    }
    return ble_get_conn_state(&ble_instance.ble);
}

void ble_app_set_conn_state(ble_conn_state_t state)
{
    if (!ble_instance.enabled) {
        return;
    }

    ble_set_conn_state(&ble_instance.ble, state);

    if (state == BLE_CONN_CONNECTED) {
        LOG_INFO("BLE Connected");
    } else {
        LOG_INFO("BLE Disconnected");
    }
}

ble_mode_t ble_app_get_mode(void)
{
    if (!ble_instance.enabled) {
        return BLE_MODE_BYPASS;
    }
    return ble_get_mode(&ble_instance.ble);
}

/*===========================================================================
 * 내부 함수 구현
 *===========================================================================*/

/**
 * @brief RX 태스크
 *
 * DMA 버퍼에서 데이터를 읽어 링버퍼에 쓰고,
 * 링버퍼에서 읽어 파싱 (GPS 구조와 동일)
 */
static void ble_rx_task(void *pvParameter)
{
    ble_instance_t *inst = (ble_instance_t *)pvParameter;
    uint8_t dummy;
    size_t old_pos = 0;
    size_t pos;

    LOG_INFO("BLE RX 태스크 시작");

    while (inst->running) {
        /* RX 신호 대기 (타임아웃 100ms) */
        xQueueReceive(inst->rx_queue, &dummy, pdMS_TO_TICKS(100));

        /* DMA 버퍼에서 새 데이터 확인 */
        pos = ble_port_get_rx_pos();
        char *dma_buf = ble_port_get_recv_buf();

        if (pos != old_pos) {
            if (pos > old_pos) {
                /* 선형 데이터 → 링버퍼에 쓰기 */
                size_t len = pos - old_pos;
                ble_rx_write(&inst->ble, (const uint8_t *)&dma_buf[old_pos], len);
            } else {
                /* 래핑된 데이터 → 링버퍼에 쓰기 */
                size_t len1 = BLE_RX_BUF_SIZE - old_pos;
                ble_rx_write(&inst->ble, (const uint8_t *)&dma_buf[old_pos], len1);

                if (pos > 0) {
                    ble_rx_write(&inst->ble, (const uint8_t *)dma_buf, pos);
                }
            }

            old_pos = pos;
            if (old_pos >= BLE_RX_BUF_SIZE) {
                old_pos = 0;
            }
        }

        /* 링버퍼에서 읽어서 파싱 */
        ble_process_rx(&inst->ble);
    }

    LOG_INFO("BLE RX 태스크 종료");
    vTaskDelete(NULL);
}

/**
 * @brief BLE 이벤트 핸들러
 *
 * lib/ble에서 파싱된 이벤트 처리
 */
static void ble_evt_handler(ble_t *ble, const ble_event_t *event)
{
    if (!ble || !event) {
        return;
    }

    switch (event->type) {
        case BLE_EVENT_AT_OK:
            LOG_DEBUG("AT OK: %s", event->data.at_response.response);
            break;

        case BLE_EVENT_AT_ERROR:
            LOG_WARN("AT ERROR: %s", event->data.at_response.response);
            break;

        case BLE_EVENT_AT_READY:
            LOG_INFO("BLE 모듈 준비 완료");
            break;

        case BLE_EVENT_CONNECTED:
            LOG_INFO("BLE 연결됨 (이벤트)");
            break;

        case BLE_EVENT_DISCONNECTED:
            LOG_INFO("BLE 연결 해제됨 (이벤트)");
            break;

        case BLE_EVENT_ADVERTISING:
            LOG_DEBUG("BLE Advertising 시작됨");
            break;

        case BLE_EVENT_APP_CMD:
            /* 앱 명령어 처리 (ble_cmd.c) */
            LOG_INFO("앱 명령어 수신: %s", event->data.app_cmd.cmd);
            ble_app_cmd_handler(&ble_instance, event->data.app_cmd.cmd);
            break;

        default:
            break;
    }
}
