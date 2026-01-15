/**
 * @file ble_task.c
 * @brief BLE RX 태스크 구현
 *
 * 인터럽트에서 DMA→링버퍼 쓰기 완료 후 신호 수신
 * 태스크는 링버퍼에서 읽어 파싱 수행
 */

#include "ble.h"
#include <string.h>

#ifndef TAG
#define TAG "BLE_TASK"
#endif

#include "log.h"

/*===========================================================================
 * 설정
 *===========================================================================*/
#define BLE_RX_TASK_STACK_SIZE  512
#define BLE_RX_TASK_PRIORITY    3
#define BLE_RX_QUEUE_SIZE       16

/*===========================================================================
 * 내부 함수
 *===========================================================================*/

/**
 * @brief RX 태스크 함수
 *
 * 링버퍼에서 데이터 읽어 파싱
 */
static void ble_rx_task_func(void *pvParameter)
{
    ble_t *ble = (ble_t *)pvParameter;
    uint8_t dummy;

    LOG_INFO("BLE RX 태스크 시작");

    while (ble->running) {
        /* RX 신호 대기 (타임아웃 100ms) */
        if (xQueueReceive(ble->rx_queue, &dummy, pdMS_TO_TICKS(100)) == pdTRUE) {
            /* 링버퍼에서 읽어서 파싱 */
            ble_process_rx(ble);
        }
    }

    LOG_INFO("BLE RX 태스크 종료");
    vTaskDelete(NULL);
}

/*===========================================================================
 * API 구현
 *===========================================================================*/

bool ble_rx_task_start(ble_t *ble)
{
    if (!ble) {
        return false;
    }

    /* 이미 실행 중이면 스킵 */
    if (ble->running) {
        return true;
    }

    /* RX 큐 생성 */
    ble->rx_queue = xQueueCreate(BLE_RX_QUEUE_SIZE, sizeof(uint8_t));
    if (!ble->rx_queue) {
        LOG_ERR("BLE RX 큐 생성 실패");
        return false;
    }

    ble->running = true;

    /* RX 태스크 생성 */
    BaseType_t ret = xTaskCreate(
        ble_rx_task_func,
        "ble_rx",
        BLE_RX_TASK_STACK_SIZE,
        ble,
        BLE_RX_TASK_PRIORITY,
        &ble->rx_task
    );

    if (ret != pdPASS) {
        LOG_ERR("BLE RX 태스크 생성 실패");
        ble->running = false;
        vQueueDelete(ble->rx_queue);
        ble->rx_queue = NULL;
        return false;
    }

    LOG_INFO("BLE RX 태스크 생성 완료");
    return true;
}

void ble_rx_task_stop(ble_t *ble)
{
    if (!ble || !ble->running) {
        return;
    }

    ble->running = false;

    /* 태스크 종료 대기 */
    vTaskDelay(pdMS_TO_TICKS(200));

    /* 큐 삭제 */
    if (ble->rx_queue) {
        vQueueDelete(ble->rx_queue);
        ble->rx_queue = NULL;
    }

    ble->rx_task = NULL;

    LOG_INFO("BLE RX 태스크 정지 완료");
}

QueueHandle_t ble_get_rx_queue(ble_t *ble)
{
    if (!ble) {
        return NULL;
    }
    return ble->rx_queue;
}
