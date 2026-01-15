#ifndef BLE_H
#define BLE_H

/**
 * @file ble.h
 * @brief BLE 메인 헤더
 *
 * BLE 모듈 핵심 기능:
 * - AT 명령어로 모듈 설정 (AT 모드)
 * - 외부 PC 명령어 송수신 (Bypass 모드)
 * - 이벤트 기반 데이터 처리
 * - FOTA 지원 (예정)
 *
 * GPS 구조 참고하여 설계:
 * - lib/ble: 핵심 라이브러리 (ble.h, ble_types.h, ble_parser.h)
 * - app/ble: 앱 레벨 (ble_app.h, ble_port.h, ble_cmd.h)
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "ble_types.h"
#include "ble_parser.h"
#include "ringbuffer.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/*===========================================================================
 * 이벤트 핸들러 타입
 *===========================================================================*/
typedef void (*ble_evt_handler_t)(ble_t *ble, const ble_event_t *event);

/*===========================================================================
 * AT 명령 응답 컨텍스트 (동기 명령용)
 *===========================================================================*/
typedef struct {
    char expected[32];              /**< 기대 응답 (+OK 등) */
    char response[BLE_AT_RESPONSE_MAX]; /**< 실제 응답 버퍼 */
    size_t response_len;            /**< 응답 길이 */
    SemaphoreHandle_t sem;          /**< 응답 대기 세마포어 */
    ble_at_status_t status;         /**< 명령 상태 */
    bool active;                    /**< 요청 활성 상태 */
} ble_at_request_t;

/*===========================================================================
 * BLE 메인 구조체 (GPS의 gps_t 참고)
 *===========================================================================*/
struct ble_s {
    /*--- HAL ---*/
    const ble_hal_ops_t *ops;       /**< HAL 연산 함수 포인터 */

    /*--- RX 버퍼 (GPS처럼 링버퍼 사용) ---*/
    ringbuffer_t rx_buf;            /**< RX 링버퍼 */
    char rx_buf_mem[BLE_RX_BUF_SIZE]; /**< RX 버퍼 메모리 */

    /*--- 파서 ---*/
    ble_parser_ctx_t parser_ctx;    /**< 파서 컨텍스트 */
    char line_buf[BLE_PARSER_BUF_SIZE]; /**< 파싱된 라인 저장 */

    /*--- 상태 ---*/
    ble_mode_t mode;                /**< 현재 모드 (AT/Bypass) */
    ble_conn_state_t conn_state;    /**< 연결 상태 */
    bool ready;                     /**< 모듈 준비 완료 */

    /*--- AT 명령 동기화 ---*/
    ble_at_request_t at_request;    /**< 동기 AT 명령 요청 */
    SemaphoreHandle_t mutex;        /**< 송신 보호 뮤텍스 */

    /*--- 이벤트 핸들러 ---*/
    ble_evt_handler_t handler;      /**< 이벤트 콜백 */
    void *user_data;                /**< 사용자 데이터 */

    /*--- RX 태스크 (lib에서 관리) ---*/
    QueueHandle_t rx_queue;         /**< RX 신호 큐 */
    TaskHandle_t rx_task;           /**< RX 태스크 핸들 */
    volatile bool running;          /**< 태스크 실행 상태 */
};

/*===========================================================================
 * 초기화 API
 *===========================================================================*/

/**
 * @brief BLE 구조체 초기화
 *
 * HAL ops 설정, 링버퍼 초기화, 파서 초기화
 * OS 리소스(세마포어, 뮤텍스) 생성
 *
 * @param ble BLE 핸들
 * @param ops HAL 연산
 * @return true: 성공
 */
bool ble_init(ble_t *ble, const ble_hal_ops_t *ops);

/**
 * @brief BLE 이벤트 핸들러 설정
 * @param ble BLE 핸들
 * @param handler 이벤트 핸들러
 * @param user_data 사용자 데이터 (이벤트 핸들러로 전달)
 */
void ble_set_evt_handler(ble_t *ble, ble_evt_handler_t handler, void *user_data);

/*===========================================================================
 * 링버퍼 API (DMA에서 데이터 기록용)
 *===========================================================================*/

/**
 * @brief RX 링버퍼에 데이터 쓰기
 *
 * DMA 인터럽트 또는 IDLE 인터럽트에서 호출
 *
 * @param ble BLE 핸들
 * @param data 수신 데이터
 * @param len 데이터 길이
 */
void ble_rx_write(ble_t *ble, const uint8_t *data, size_t len);

/**
 * @brief RX 링버퍼 포인터 가져오기
 * @param ble BLE 핸들
 * @return 링버퍼 포인터
 */
ringbuffer_t *ble_get_rx_buf(ble_t *ble);

/*===========================================================================
 * 데이터 송수신 API
 *===========================================================================*/

/**
 * @brief 데이터 전송 (Bypass 모드)
 * @param ble BLE 핸들
 * @param data 전송 데이터
 * @param len 데이터 길이
 * @return true: 성공
 */
bool ble_send(ble_t *ble, const char *data, size_t len);

/**
 * @brief RX 링버퍼에서 데이터 읽어서 파싱
 *
 * RX 태스크에서 주기적으로 호출
 * 파싱 완료 시 이벤트 핸들러 호출
 *
 * @param ble BLE 핸들
 */
void ble_process_rx(ble_t *ble);

/*===========================================================================
 * AT 명령어 API (동기)
 *===========================================================================*/

/**
 * @brief AT 모드 전환
 * @param ble BLE 핸들
 * @return true: 성공
 */
bool ble_enter_at_mode(ble_t *ble);

/**
 * @brief Bypass 모드 전환
 * @param ble BLE 핸들
 * @return true: 성공
 */
bool ble_enter_bypass_mode(ble_t *ble);

/**
 * @brief AT 명령 동기 전송
 *
 * AT 모드로 전환 → 명령 전송 → 응답 대기 → Bypass 복귀
 *
 * @param ble BLE 핸들
 * @param cmd AT 명령어 (예: "AT+MANUF=TEST")
 * @param response 응답 버퍼 (NULL 가능)
 * @param response_size 응답 버퍼 크기
 * @param timeout_ms 타임아웃 (ms)
 * @return AT 명령 상태
 */
ble_at_status_t ble_send_at_cmd_sync(ble_t *ble, const char *cmd,
                                      char *response, size_t response_size,
                                      uint32_t timeout_ms);

/*===========================================================================
 * 상태 조회 API
 *===========================================================================*/

/**
 * @brief 현재 모드 조회
 * @param ble BLE 핸들
 * @return 현재 모드
 */
ble_mode_t ble_get_mode(const ble_t *ble);

/**
 * @brief 연결 상태 조회
 * @param ble BLE 핸들
 * @return 연결 상태
 */
ble_conn_state_t ble_get_conn_state(const ble_t *ble);

/**
 * @brief 연결 상태 설정 (GPIO 인터럽트에서 호출)
 * @param ble BLE 핸들
 * @param state 연결 상태
 */
void ble_set_conn_state(ble_t *ble, ble_conn_state_t state);

/**
 * @brief 모듈 준비 상태 조회
 * @param ble BLE 핸들
 * @return true: 준비 완료
 */
bool ble_is_ready(const ble_t *ble);

/*===========================================================================
 * RX 태스크 API
 *===========================================================================*/

/**
 * @brief RX 태스크 시작
 *
 * 링버퍼에서 데이터 읽어 파싱하는 태스크 생성
 *
 * @param ble BLE 핸들
 * @return true: 성공
 */
bool ble_rx_task_start(ble_t *ble);

/**
 * @brief RX 태스크 정지
 * @param ble BLE 핸들
 */
void ble_rx_task_stop(ble_t *ble);

/**
 * @brief RX 큐 가져오기 (인터럽트에서 신호 전송용)
 * @param ble BLE 핸들
 * @return RX 큐 핸들
 */
QueueHandle_t ble_get_rx_queue(ble_t *ble);

/*===========================================================================
 * 내부 API (app 레벨에서 사용)
 *===========================================================================*/

/**
 * @brief AT 응답 매칭 처리 (내부용)
 *
 * 파서에서 AT 응답을 받으면 호출
 * 동기 AT 명령 대기 중이면 세마포어 해제
 *
 * @param ble BLE 핸들
 * @param result 파싱 결과
 * @param line 파싱된 라인
 */
void ble_handle_at_response(ble_t *ble, ble_parse_result_t result, const char *line);

#endif /* BLE_H */
