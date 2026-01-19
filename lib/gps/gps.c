/**
 * @file gps.c
 * @brief GPS 초기화 및 태스크
 */

#include "gps.h"
#include "gps_config.h"
#include "gps_parser.h"
#include <string.h>

#ifndef TAG
#define TAG "GPS"
#endif

#include "log.h"

/*===========================================================================
 * 내부 함수 선언
 *===========================================================================*/
static void gps_process_task(void *pvParameter);

/*===========================================================================
 * GPS 초기화
 *===========================================================================*/

bool gps_init(gps_t *gps) {
    if (!gps) {
        LOG_ERR("GPS handle is NULL");
        return false;
    }

    memset(gps, 0, sizeof(gps_t));

    /* RX 링버퍼 초기화 */
    ringbuffer_init(&gps->rx_buf, gps->rx_buf_mem, sizeof(gps->rx_buf_mem));

    /* RTCM 링버퍼 초기화 */
    ringbuffer_init(&gps->rtcm_data.rb, gps->rtcm_data.rb_mem, sizeof(gps->rtcm_data.rb_mem));

    /* 파서 초기화 */
    gps_parser_init(gps);

    /* OS 객체 생성 */
    gps->pkt_queue = xQueueCreate(10, sizeof(uint8_t));
    if (!gps->pkt_queue) {
        LOG_ERR("Failed to create pkt_queue");
        return false;
    }

    gps->mutex = xSemaphoreCreateMutex();
    if (!gps->mutex) {
        LOG_ERR("Failed to create mutex");
        return false;
    }

    gps->cmd_sem = xSemaphoreCreateBinary();
    if (!gps->cmd_sem) {
        LOG_ERR("Failed to create cmd_sem");
        return false;
    }

    /* RTCM 버퍼 mutex 생성 */
    gps->rtcm_data.mutex = xSemaphoreCreateMutex();
    if (!gps->rtcm_data.mutex) {
        LOG_ERR("Failed to create rtcm_mutex");
        return false;
    }

    /* RX 태스크 생성 */
    BaseType_t ret = xTaskCreate(
        gps_process_task,
        "gps_pkt",
        1024,
        (void*)gps,
        tskIDLE_PRIORITY + 1,
        &gps->pkt_task
    );

    if (ret != pdPASS) {
        LOG_ERR("Failed to create gps_process_task");
        return false;
    }

    gps->is_running = true;
    LOG_INFO("GPS initialized");

    return true;
}

/*===========================================================================
 * GPS 리소스 해제
 *===========================================================================*/

void gps_deinit(gps_t *gps) {
    if (!gps) {
        LOG_ERR("GPS handle is NULL");
        return;
    }

    LOG_INFO("GPS deinit start");

    /* 1. 프로세스 태스크 종료 */
    if (gps->is_running) {
        gps_stop(gps);
    }

    /* 2. OS 객체 삭제 */
    if (gps->pkt_queue) {
        vQueueDelete(gps->pkt_queue);
        gps->pkt_queue = NULL;
    }

    if (gps->mutex) {
        vSemaphoreDelete(gps->mutex);
        gps->mutex = NULL;
    }

    if (gps->cmd_sem) {
        vSemaphoreDelete(gps->cmd_sem);
        gps->cmd_sem = NULL;
    }

    if (gps->rtcm_data.mutex) {
        vSemaphoreDelete(gps->rtcm_data.mutex);
        gps->rtcm_data.mutex = NULL;
    }

    /* 3. 태스크 핸들 초기화 */
    gps->pkt_task = NULL;

    /* 4. 상태 초기화 */
    gps->is_alive = false;
    gps->is_running = false;
    gps->handler = NULL;
    gps->ops = NULL;

    LOG_INFO("GPS deinit complete");
}

/*===========================================================================
 * 이벤트 핸들러 설정
 *===========================================================================*/

void gps_set_evt_handler(gps_t *gps, gps_evt_handler handler) {
    if (gps && handler) {
        gps->handler = handler;
    }
}

/*===========================================================================
 * 동기 명령어 전송
 *
 * 멀티태스크 지원:
 * - 여러 태스크가 동시에 호출 가능
 * - mutex로 한 번에 하나씩만 송신하도록 보호
 * - RX Task는 mutex 사용 안함 (데드락 방지)
 *===========================================================================*/

/**
 * @brief GPS 종료 요청
 *
 * 프로세스 태스크를 안전하게 종료시킵니다.
 * 종료 신호를 큐에 보내 portMAX_DELAY 대기를 깨웁니다.
 */
void gps_stop(gps_t *gps) {
    if (!gps) {
        return;
    }

    gps->is_running = false;

    /* 큐에 더미 데이터를 보내 portMAX_DELAY 대기를 깨움 */
    if (gps->pkt_queue) {
        uint8_t dummy = 0xFF;  /* 종료 신호 */
        xQueueSend(gps->pkt_queue, &dummy, 0);
    }

    /* 태스크가 종료될 때까지 대기 (최대 500ms) */
    uint32_t wait_count = 0;
    while (gps->is_alive && wait_count < 50) {
        vTaskDelay(pdMS_TO_TICKS(10));
        wait_count++;
    }

    if (gps->is_alive) {
        LOG_WARN("GPS process task did not stop gracefully");
    }
}

bool gps_send_cmd_sync(gps_t *gps, const char *cmd, uint32_t timeout_ms) {
    if (!gps || !cmd || !gps->ops || !gps->ops->send) {
        LOG_ERR("Invalid parameters");
        return false;
    }

    /* 전체를 mutex로 보호 (한 번에 하나씩만 송신) */
    if (xSemaphoreTake(gps->mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        LOG_ERR("Failed to acquire mutex for cmd: %s", cmd);
        return false;
    }

    /* 세마포어 비우기 (이전 잔여 시그널 제거) */
    xSemaphoreTake(gps->cmd_sem, 0);

    /* 응답 대기 상태 설정 */
    gps->parser_ctx.cmd_ctx.waiting = true;
    gps->parser_ctx.cmd_ctx.result_ok = false;

    /* 명령어 전송 */
    size_t cmd_len = strlen(cmd);
    gps->ops->send(cmd, cmd_len);
    gps->ops->send("\r\n", 2);

    LOG_DEBUG("CMD TX: %s", cmd);

    /* 응답 대기 (RX Task가 응답 받으면 세마포어 시그널) */
    BaseType_t got = xSemaphoreTake(gps->cmd_sem, pdMS_TO_TICKS(timeout_ms));

    /* 정리 */
    gps->parser_ctx.cmd_ctx.waiting = false;

    bool result = (got == pdTRUE) && gps->parser_ctx.cmd_ctx.result_ok;

    /* Mutex 해제 */
    xSemaphoreGive(gps->mutex);

    if (!got) {
        LOG_WARN("CMD timeout: %s", cmd);
        return false;
    }

    LOG_DEBUG("CMD response: %s", result ? "OK" : "ERROR");
    return result;
}

/*===========================================================================
 * GPS 패킷 처리 태스크
 *===========================================================================*/

static void gps_process_task(void *pvParameter) {
    gps_t *gps = (gps_t *)pvParameter;
    uint8_t dummy;

    gps->is_alive = true;
    LOG_INFO("GPS process task started");

    while (gps->is_running) {
        /* RX 신호 대기 (UART ISR에서 queue send) */
        if (xQueueReceive(gps->pkt_queue, &dummy, portMAX_DELAY) == pdTRUE) {
            /* 새 파서로 패킷 파싱 */
            static char buf[2048];
            size_t len = ringbuffer_size(&gps->rx_buf);
            if (len > 0) {
                len = (len > sizeof(buf)) ? sizeof(buf) : len;
                ringbuffer_peek(&gps->rx_buf, buf, len, 0);
                LOG_DEBUG_RAW("", buf, len);
            }

            gps_parser_process(gps);
        }

        /* 주기적 타임아웃 체크 (optional) */
        /* 여기서 불완전 패킷 타임아웃 처리 등 추가 가능 */
    }

    gps->is_alive = false;
    LOG_INFO("GPS process task stopped");
    vTaskDelete(NULL);
}

/*===========================================================================
 * 레거시 API (deprecated - 기존 코드 호환용)
 *
 * 기존 바이트 단위 파싱은 더 이상 사용하지 않음.
 * 새 파서(gps_parser_process)는 ringbuffer 기반으로 동작.
 *===========================================================================*/

void gps_parse_process(gps_t *gps, const void *data, size_t len) {
    /* deprecated: 이 함수는 더 이상 사용하지 않음
     * 대신 UART ISR에서 ringbuffer_write() 후 queue send
     * 그러면 gps_process_task에서 gps_parser_process() 호출
     */
    (void)gps;
    (void)data;
    (void)len;
    LOG_WARN("gps_parse_process is deprecated, use ringbuffer + gps_parser_process");
}

/*===========================================================================
 * GGA Raw 저장 (레거시 호환용)
 *===========================================================================*/
#if defined(USE_STORE_RAW_GGA)
void _gps_gga_raw_add(gps_t *gps, char ch) {
    if (gps->nmea_data.gga_raw_pos < sizeof(gps->nmea_data.gga_raw) - 1) {
        gps->nmea_data.gga_raw[gps->nmea_data.gga_raw_pos] = ch;
        gps->nmea_data.gga_raw[++gps->nmea_data.gga_raw_pos] = '\0';
    }
}
#endif
