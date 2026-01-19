#include "lora.h"

#ifndef TAG
    #define TAG "LORA"
#endif

#include "log.h"
#include <string.h>

/*===========================================================================
 * 초기화 API
 *===========================================================================*/

bool lora_init(lora_t *lora, const lora_hal_ops_t *ops)
{
    if (!lora) {
        LOG_ERR("NULL lora handle");
        return false;
    }

    /* 구조체 초기화 */
    memset(lora, 0, sizeof(lora_t));
    lora->ops = ops;

    /* 상태 초기화 */
    lora->initialized = false;
    lora->init_complete = false;
    lora->tx_task_ready = false;
    lora->rx_task_ready = false;

    LOG_INFO("LoRa 구조체 초기화 완료");
    return true;
}

void lora_deinit(lora_t *lora)
{
    if (!lora) {
        return;
    }

    LOG_INFO("LoRa 리소스 해제 시작");

    /* 태스크 정지 */
    lora_task_stop(lora);

    /* 큐 삭제 */
    if (lora->rx_queue != NULL) {
        vQueueDelete(lora->rx_queue);
        lora->rx_queue = NULL;
    }

    if (lora->cmd_queue != NULL) {
        vQueueDelete(lora->cmd_queue);
        lora->cmd_queue = NULL;
    }

    /* 뮤텍스 삭제 */
    if (lora->mutex != NULL) {
        vSemaphoreDelete(lora->mutex);
        lora->mutex = NULL;
    }

    /* 상태 초기화 */
    lora->initialized = false;
    lora->init_complete = false;
    lora->tx_task_ready = false;
    lora->rx_task_ready = false;
    lora->current_cmd_req = NULL;
    lora->p2p_recv_callback = NULL;
    lora->p2p_recv_user_data = NULL;

    /* RTCM 재조립 버퍼 초기화 */
    memset(&lora->rtcm_reassembly, 0, sizeof(lora->rtcm_reassembly));

    LOG_INFO("LoRa 리소스 해제 완료");
}

/*===========================================================================
 * 태스크 API (실제 구현은 lora_app.c에서)
 *===========================================================================*/

void lora_task_stop(lora_t *lora)
{
    if (!lora) {
        return;
    }

    /* RX 태스크 삭제 */
    if (lora->rx_task != NULL) {
        vTaskDelete(lora->rx_task);
        lora->rx_task = NULL;
        LOG_INFO("LoRa RX Task 삭제");
    }

    /* TX 태스크 삭제 */
    if (lora->tx_task != NULL) {
        vTaskDelete(lora->tx_task);
        lora->tx_task = NULL;
        LOG_INFO("LoRa TX Task 삭제");
    }

    lora->tx_task_ready = false;
    lora->rx_task_ready = false;
}

QueueHandle_t lora_get_rx_queue(lora_t *lora)
{
    if (!lora) {
        return NULL;
    }
    return lora->rx_queue;
}

/*===========================================================================
 * P2P 수신 콜백 API
 *===========================================================================*/

void lora_set_p2p_recv_callback(lora_t *lora, lora_p2p_recv_callback_t callback,
                                void *user_data)
{
    if (!lora) {
        return;
    }
    lora->p2p_recv_callback = callback;
    lora->p2p_recv_user_data = user_data;
}

/*===========================================================================
 * 상태 조회 API
 *===========================================================================*/

bool lora_is_ready(const lora_t *lora)
{
    if (!lora) {
        return false;
    }
    return lora->initialized && lora->init_complete;
}
