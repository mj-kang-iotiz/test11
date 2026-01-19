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
typedef void (*gps_evt_handler)(gps_t *gps, const gps_event_t *event);

/*===========================================================================
 * 공용 GPS 데이터 구조체
 * 여러 소스(GGA, BESTNAV, THS 등)에서 업데이트되는 통합 데이터
 *===========================================================================*/
typedef struct {
    /* === 위치 (BESTNAV가 업데이트) === */
    struct {
        double latitude;            /**< 위도 (degree) */
        double longitude;           /**< 경도 (degree) */
        double altitude;            /**< 고도 (meter) */
        float lat_std;              /**< 위도 표준편차 (meter) */
        float lon_std;              /**< 경도 표준편차 (meter) */
        float alt_std;              /**< 고도 표준편차 (meter) */
        uint32_t timestamp_ms;      /**< 업데이트 시각 */
    } position;

    /* === 속도 (BESTNAV가 업데이트) === */
    struct {
        double hor_speed;           /**< 수평 속도 (m/s) */
        double ver_speed;           /**< 수직 속도 (m/s) */
        double track;               /**< 진행 방향 (degree, 0-360) */
        uint32_t timestamp_ms;      /**< 업데이트 시각 */
    } velocity;

    /* === 헤딩 (THS가 업데이트) === */
    struct {
        double heading;             /**< 헤딩 (degree, 0-360) */
        uint8_t mode;               /**< 헤딩 모드 (gps_ths_mode_t) */
        uint32_t timestamp_ms;      /**< 업데이트 시각 */
    } heading;

    /* === 상태 정보 === */
    struct {
        gps_fix_t fix_type;         /**< Fix 타입 (GGA가 업데이트) */
        uint8_t sat_count;          /**< 위성 수 (BESTNAV.sv가 업데이트) */
        uint8_t used_sat_count;     /**< 사용 위성 수 (BESTNAV.used_sv) */
        float hdop;                 /**< HDOP (GGA가 업데이트) */
        uint32_t fix_timestamp_ms;  /**< Fix 업데이트 시각 */
        uint32_t sat_timestamp_ms;  /**< 위성수 업데이트 시각 */
        bool fix_changed;           /**< Fix 상태 변경됨 (이벤트 발생용) */
    } status;

} gps_common_data_t;

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
 * 여러 메시지가 공용 필드를 업데이트 (BESTNAV, HEADING2, BESTPOS, BESTVEL 등)
 *===========================================================================*/
typedef struct {
    /* === 공용 헤더 정보 === */
    uint16_t last_msg_id;           /**< 마지막 수신 메시지 ID */
    uint32_t gps_week;              /**< GPS 주 */
    uint32_t gps_ms;                /**< GPS 밀리초 */
    uint32_t timestamp_ms;          /**< 수신 시각 (xTaskGetTickCount) */

    /* === 공용 위치 데이터 (BESTNAV, BESTPOS가 업데이트) === */
    struct {
        bool valid;                 /**< 유효한 데이터 있음 */
        double latitude;            /**< 위도 (degree) */
        double longitude;           /**< 경도 (degree) */
        double altitude;            /**< 고도 (meter) */
        uint8_t pos_type;           /**< 위치 타입 (0=NONE, 16=RTK_FIXED, 17=RTK_FLOAT) */
        float lat_std;              /**< 위도 표준편차 (meter) */
        float lon_std;              /**< 경도 표준편차 (meter) */
        float alt_std;              /**< 고도 표준편차 (meter) */
        uint16_t source_msg;        /**< 출처 메시지 ID (2118=BESTNAV, 42=BESTPOS) */
    } position;

    /* === 공용 헤딩 데이터 (HEADING2가 업데이트) === */
    struct {
        bool valid;                 /**< 유효한 데이터 있음 */
        double heading;             /**< 헤딩 (degree, 0-360) */
        double pitch;               /**< 피치 (degree) */
        float heading_std;          /**< 헤딩 표준편차 (degree) */
        float pitch_std;            /**< 피치 표준편차 (degree) */
        uint16_t source_msg;        /**< 출처 메시지 ID (2120=HEADING2) */
    } heading;

    /* === 공용 속도 데이터 (BESTNAV, BESTVEL이 업데이트) === */
    struct {
        bool valid;                 /**< 유효한 데이터 있음 */
        double hor_speed;           /**< 수평 속도 (m/s) */
        double trk_gnd;             /**< 진행 방향 (degree, 0-360) */
        double ver_speed;           /**< 수직 속도 (m/s) */
        uint16_t source_msg;        /**< 출처 메시지 ID (2118=BESTNAV, 99=BESTVEL) */
    } velocity;

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

    /*--- 파싱된 데이터 (프로토콜별 원본) ---*/
    gps_nmea_data_t nmea_data;      /**< NMEA 파싱 데이터 (GGA, THS 등) */
    gps_unicore_bin_data_t unicore_bin_data; /**< Unicore Binary 데이터 */
    gps_rtcm_data_t rtcm_data;      /**< RTCM 데이터 (LoRa 전송용) */

    /*--- 공용 데이터 (통합) ---*/
    gps_common_data_t data;         /**< 통합 GPS 데이터 (BESTNAV→위치, GGA→fix, THS→헤딩) */

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
 * @brief GPS 리소스 해제
 *
 * 프로세스 태스크를 종료하고 모든 OS 객체(큐, 세마포어, 뮤텍스)를 해제합니다.
 *
 * @param gps GPS 핸들
 */
void gps_deinit(gps_t *gps);

/**
 * @brief GPS 종료 요청
 *
 * 프로세스 태스크를 안전하게 종료시킵니다.
 * 종료 신호를 큐에 보내 portMAX_DELAY 대기를 깨웁니다.
 *
 * @param gps GPS 핸들
 */
void gps_stop(gps_t *gps);

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
