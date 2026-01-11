#ifndef BLE_TYPES_H
#define BLE_TYPES_H

/**
 * @file ble_types.h
 * @brief BLE 타입 정의
 *
 * BLE 모듈 이벤트, 상태, HAL 타입 정의
 * - 외부 PC 명령어 송수신
 * - AT 명령어로 BLE 설정
 * - FOTA 지원 (예정)
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Forward declaration */
typedef struct ble_s ble_t;

/*===========================================================================
 * BLE 모드
 *===========================================================================*/
typedef enum {
    BLE_MODE_BYPASS,    /**< 일반 UART 통신 모드 (데이터 수신) */
    BLE_MODE_AT,        /**< AT 커맨드 모드 (설정) */
} ble_mode_t;

/*===========================================================================
 * BLE 연결 상태
 *===========================================================================*/
typedef enum {
    BLE_CONN_DISCONNECTED,  /**< 연결 안됨 */
    BLE_CONN_CONNECTED,     /**< 연결됨 */
} ble_conn_state_t;

/*===========================================================================
 * AT 명령 상태
 *===========================================================================*/
typedef enum {
    BLE_AT_STATUS_IDLE,         /**< 대기 중 */
    BLE_AT_STATUS_PENDING,      /**< 응답 대기 중 */
    BLE_AT_STATUS_COMPLETED,    /**< 성공 (+OK) */
    BLE_AT_STATUS_ERROR,        /**< 실패 (+ERROR) */
    BLE_AT_STATUS_TIMEOUT,      /**< 타임아웃 */
} ble_at_status_t;

/*===========================================================================
 * BLE 이벤트 타입 (GPS의 gps_event_type_t 참고)
 *===========================================================================*/
typedef enum {
    BLE_EVENT_NONE = 0,

    /* AT 응답 이벤트 */
    BLE_EVENT_AT_OK,            /**< +OK 응답 */
    BLE_EVENT_AT_ERROR,         /**< +ERROR 응답 */
    BLE_EVENT_AT_READY,         /**< +READY (모듈 부팅 완료) */

    /* 연결 상태 이벤트 */
    BLE_EVENT_CONNECTED,        /**< BLE 연결됨 */
    BLE_EVENT_DISCONNECTED,     /**< BLE 연결 해제됨 */
    BLE_EVENT_ADVERTISING,      /**< Advertising 시작 */

    /* 데이터 수신 이벤트 (Bypass 모드) */
    BLE_EVENT_DATA_RECEIVED,    /**< 외부 PC로부터 데이터 수신 */

    /* 앱 명령어 이벤트 */
    BLE_EVENT_APP_CMD,          /**< 앱 명령어 수신 (SD+, SC+ 등) */

    /* FOTA 이벤트 (예정) */
    BLE_EVENT_FOTA_START,       /**< FOTA 시작 */
    BLE_EVENT_FOTA_DATA,        /**< FOTA 데이터 수신 */
    BLE_EVENT_FOTA_COMPLETE,    /**< FOTA 완료 */
    BLE_EVENT_FOTA_ERROR,       /**< FOTA 에러 */

} ble_event_type_t;

/*===========================================================================
 * BLE 이벤트 구조체 (GPS의 gps_event_t 참고)
 *===========================================================================*/
typedef struct {
    ble_event_type_t type;      /**< 이벤트 타입 */
    uint32_t timestamp_ms;      /**< 수신 시간 (xTaskGetTickCount) */

    /* 이벤트별 데이터 */
    union {
        /* AT 응답 */
        struct {
            char response[64];  /**< 응답 문자열 */
            size_t len;         /**< 응답 길이 */
        } at_response;

        /* 데이터 수신 (Bypass 모드) */
        struct {
            const uint8_t *data;    /**< 수신 데이터 포인터 */
            size_t len;             /**< 데이터 길이 */
        } rx_data;

        /* 앱 명령어 */
        struct {
            const char *cmd;        /**< 명령어 문자열 */
            const char *param;      /**< 파라미터 */
        } app_cmd;

        /* FOTA 데이터 (예정) */
        struct {
            uint32_t offset;        /**< 데이터 오프셋 */
            const uint8_t *data;    /**< 데이터 */
            size_t len;             /**< 길이 */
        } fota_data;

    } data;

} ble_event_t;

/*===========================================================================
 * HAL 연산 (GPS의 gps_hal_ops_t 참고)
 *===========================================================================*/
typedef struct {
    int (*init)(void);
    int (*start)(void);
    int (*stop)(void);
    int (*reset)(void);
    int (*send)(const char *data, size_t len);
    int (*recv)(char *buf, size_t len);
    int (*at_mode)(void);       /**< AT 모드 전환 (GPIO 제어) */
    int (*bypass_mode)(void);   /**< Bypass 모드 전환 (GPIO 제어) */
} ble_hal_ops_t;

/*===========================================================================
 * 파서 상태 (내부용)
 *===========================================================================*/
typedef enum {
    BLE_PARSE_STATE_IDLE,       /**< 대기 중 */
    BLE_PARSE_STATE_AT_RESP,    /**< AT 응답 파싱 중 (+로 시작) */
    BLE_PARSE_STATE_AT_CMD,     /**< AT 명령어 파싱 중 (AT로 시작) */
    BLE_PARSE_STATE_APP_CMD,    /**< 앱 명령어 파싱 중 */
} ble_parse_state_t;

/*===========================================================================
 * 버퍼 크기 상수
 *===========================================================================*/
#define BLE_RX_BUF_SIZE         1024    /**< RX 버퍼 크기 */
#define BLE_AT_RESPONSE_MAX     256     /**< AT 응답 최대 길이 */
#define BLE_PARSER_BUF_SIZE     256     /**< 파서 버퍼 크기 */

#endif /* BLE_TYPES_H */
