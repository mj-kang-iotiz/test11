#ifndef GPS_H
#define GPS_H

/**
 * @file gps.h
 * @brief GPS 메인 헤더
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "gps_parser.h"
#include "gps_types.h"
#include "gps_nmea.h"
#include "gps_unicore.h"
#include "rtcm.h"
#include "ringbuffer.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/*===========================================================================
 * 이벤트 핸들러 타입
 *===========================================================================*/
typedef void (*gps_evt_handler)(gps_t *gps, gps_event_t event,
                                gps_protocol_t protocol, gps_msg_t msg);

/*===========================================================================
 * RTCM 데이터 저장 구조체 (LoRa 전송용)
 *===========================================================================*/
typedef struct {
    ringbuffer_t rb;                     /**< RTCM 링버퍼 (여러 메시지 큐잉) */
    char rb_mem[4096];                   /**< 링버퍼 메모리 (RTCM은 큰 편) */
    SemaphoreHandle_t mutex;             /**< RTCM 버퍼 접근 보호 */
    uint16_t last_msg_type;              /**< 마지막 수신 메시지 타입 */
} gps_rtcm_data_t;

/*===========================================================================
 * Unicore Binary 데이터 저장 구조체
 *===========================================================================*/
typedef struct {
    gps_unicore_bin_header_t header;
    hpd_unicore_bestnavb_t bestnav;
} gps_unicore_bin_data_t;

/*===========================================================================
 * GPS 메인 구조체
 *===========================================================================*/
typedef struct gps_s {
    /*--- OS 변수 ---*/
    TaskHandle_t pkt_task;          /**< 패킷 처리 태스크 핸들 */
    SemaphoreHandle_t mutex;        /**< 송신 보호 뮤텍스 (gps_send_cmd_sync 전용) */
    QueueHandle_t pkt_queue;        /**< RX 신호 큐 */

    /*--- HAL ---*/
    const gps_hal_ops_t *ops;       /**< HAL 연산 함수 포인터 */

    /*--- RX 버퍼 ---*/
    ringbuffer_t rx_buf;            /**< RX 링버퍼 */
    char rx_buf_mem[2048];          /**< RX 버퍼 메모리 */

    /*--- 파서 ---*/
    gps_parser_ctx_t parser_ctx;    /**< 파서 컨텍스트 */

    /*--- 파싱된 데이터 ---*/
    gps_nmea_data_t nmea_data;      /**< NMEA 파싱 데이터 (GGA, THS 등) */
    gps_unicore_bin_data_t unicore_bin_data; /**< Unicore Binary 데이터 */
    gps_rtcm_data_t rtcm_data;      /**< RTCM 데이터 (LoRa 전송용) */

    /*--- 명령어 처리 ---*/
    SemaphoreHandle_t cmd_sem;      /**< 명령어 응답 대기 세마포어 */

    /*--- 상태 ---*/
    bool is_alive;                  /**< RX 태스크 동작 여부 */
    bool is_running;                /**< 실행 상태 */

    /*--- 이벤트 핸들러 ---*/
    gps_evt_handler handler;        /**< 이벤트 콜백 */
} gps_t;

/*===========================================================================
 * API
 *===========================================================================*/

/**
 * @brief GPS 초기화
 * @param gps GPS 핸들
 * @return true: 성공
 */
bool gps_init(gps_t *gps);

/**
 * @brief GPS 이벤트 핸들러 설정
 * @param gps GPS 핸들
 * @param handler 이벤트 핸들러
 */
void gps_set_evt_handler(gps_t *gps, gps_evt_handler handler);

/**
 * @brief 동기 명령어 전송
 * @param gps GPS 핸들
 * @param cmd 명령어 문자열
 * @param timeout_ms 타임아웃 (ms)
 * @return true: 성공 (OK 응답), false: 실패
 */
bool gps_send_cmd_sync(gps_t *gps, const char *cmd, uint32_t timeout_ms);

/*===========================================================================
 * 레거시 API (deprecated - 호환용)
 *===========================================================================*/

/** @deprecated 새 파서에서는 사용하지 않음 */
void gps_parse_process(gps_t *gps, const void *data, size_t len);

#endif /* GPS_H */
