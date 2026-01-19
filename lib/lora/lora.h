#ifndef LORA_H
#define LORA_H

/**
 * @file lora.h
 * @brief LoRa 메인 헤더
 *
 * LoRa 모듈 핵심 기능:
 * - AT 명령어로 모듈 설정
 * - P2P 데이터 송수신
 * - 이벤트 기반 데이터 처리
 *
 * BLE 구조 참고하여 설계:
 * - lib/lora: 핵심 라이브러리 (lora.h, AT 파서, RX/TX 태스크)
 * - app/lora: 앱 레벨 (lora_app.h, lora_port.h)
 */

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/*===========================================================================
 * 설정
 *===========================================================================*/
#define LORA_CMD_QUEUE_SIZE     25
#define LORA_RX_QUEUE_SIZE      10
#define LORA_AT_CMD_TIMEOUT_MS  2000
#define LORA_INIT_MAX_RETRY     3
#define LORA_INIT_TIMEOUT_MS    2000
#define LORA_RECV_BUF_SIZE      1024

/*===========================================================================
 * RTCM 재조립 버퍼
 *===========================================================================*/
#define RTCM_REASSEMBLY_BUF_SIZE    512
#define RTCM_REASSEMBLY_TIMEOUT_MS  5000

typedef struct {
  uint8_t buffer[RTCM_REASSEMBLY_BUF_SIZE];
  uint16_t buffer_pos;
  uint16_t expected_len;
  bool has_header;
  TickType_t last_recv_tick;
} lora_rtcm_reassembly_t;

/*===========================================================================
 * P2P 수신 데이터
 *===========================================================================*/
typedef struct {
  int16_t rssi;
  int16_t snr;
  uint16_t data_len;
  char data[256];
} lora_p2p_recv_data_t;

/*===========================================================================
 * 명령어 콜백
 *===========================================================================*/
typedef void (*lora_cmd_callback_t)(bool success, void *user_data);

/*===========================================================================
 * 명령어 요청 구조체
 *===========================================================================*/
typedef struct {
  char cmd[256];
  uint32_t timeout_ms;
  uint32_t toa_ms;
  bool is_async;
  bool skip_response;

  SemaphoreHandle_t response_sem;
  bool *result;

  lora_cmd_callback_t callback;
  void *user_data;
  bool async_result;
} lora_cmd_request_t;

/*===========================================================================
 * HAL 연산 함수 포인터
 *===========================================================================*/
typedef struct {
  int (*init)(void);
  int (*start)(void);
  int (*stop)(void);
  int (*reset)(void);
  int (*send)(const char *data, size_t len);
  int (*recv)(char *buf, size_t len);
} lora_hal_ops_t;

/*===========================================================================
 * P2P 수신 콜백
 *===========================================================================*/
typedef void (*lora_p2p_recv_callback_t)(lora_p2p_recv_data_t *recv_data, void *user_data);

/*===========================================================================
 * LoRa 이벤트 핸들러
 *===========================================================================*/
struct lora_s;
typedef struct lora_s lora_t;

typedef void (*lora_evt_handler_t)(lora_t *lora, bool success);

/*===========================================================================
 * LoRa 메인 구조체 (BLE의 ble_t 참고)
 *===========================================================================*/
struct lora_s {
  /*--- HAL ---*/
  const lora_hal_ops_t *ops;

  /*--- RX/TX 태스크 ---*/
  QueueHandle_t rx_queue;
  QueueHandle_t cmd_queue;
  TaskHandle_t rx_task;
  TaskHandle_t tx_task;
  SemaphoreHandle_t mutex;

  /*--- 상태 ---*/
  bool initialized;
  bool init_complete;
  volatile bool tx_task_ready;
  volatile bool rx_task_ready;

  /*--- 현재 명령어 ---*/
  lora_cmd_request_t *current_cmd_req;

  /*--- P2P 수신 콜백 ---*/
  lora_p2p_recv_callback_t p2p_recv_callback;
  void *p2p_recv_user_data;

  /*--- RTCM 재조립 ---*/
  lora_rtcm_reassembly_t rtcm_reassembly;

  /*--- 초기화 완료 콜백 ---*/
  lora_evt_handler_t init_complete_handler;
  void *init_handler_user_data;
};

/*===========================================================================
 * 초기화 API
 *===========================================================================*/

/**
 * @brief LoRa 구조체 초기화
 * @param lora LoRa 핸들
 * @param ops HAL 연산
 * @return true: 성공
 */
bool lora_init(lora_t *lora, const lora_hal_ops_t *ops);

/**
 * @brief LoRa 리소스 해제
 * @param lora LoRa 핸들
 */
void lora_deinit(lora_t *lora);

/*===========================================================================
 * 태스크 API
 *===========================================================================*/

/**
 * @brief RX/TX 태스크 시작
 * @param lora LoRa 핸들
 * @return true: 성공
 */
bool lora_task_start(lora_t *lora);

/**
 * @brief 태스크 정지
 * @param lora LoRa 핸들
 */
void lora_task_stop(lora_t *lora);

/**
 * @brief RX 큐 가져오기 (인터럽트용)
 * @param lora LoRa 핸들
 * @return RX 큐 핸들
 */
QueueHandle_t lora_get_rx_queue(lora_t *lora);

/*===========================================================================
 * 명령어 API
 *===========================================================================*/

/**
 * @brief AT 명령어 전송 (동기)
 * @param lora LoRa 핸들
 * @param cmd AT 명령어
 * @param timeout_ms 타임아웃
 * @return true: OK, false: ERROR/TIMEOUT
 */
bool lora_send_cmd_sync(lora_t *lora, const char *cmd, uint32_t timeout_ms);

/**
 * @brief AT 명령어 전송 (비동기)
 * @param lora LoRa 핸들
 * @param cmd AT 명령어
 * @param timeout_ms 타임아웃
 * @param toa_ms Time on Air
 * @param callback 완료 콜백
 * @param user_data 사용자 데이터
 * @param skip_response 응답 파싱 건너뛰기
 * @return true: 큐 추가 성공
 */
bool lora_send_cmd_async(lora_t *lora, const char *cmd, uint32_t timeout_ms,
                         uint32_t toa_ms, lora_cmd_callback_t callback,
                         void *user_data, bool skip_response);

/*===========================================================================
 * P2P 데이터 API
 *===========================================================================*/

/**
 * @brief P2P Raw 데이터 전송 (비동기)
 * @param lora LoRa 핸들
 * @param data Raw binary 데이터
 * @param len 데이터 길이 (최대 118)
 * @param timeout_ms 타임아웃
 * @param callback 완료 콜백
 * @param user_data 사용자 데이터
 * @return true: 큐 추가 성공
 */
bool lora_send_p2p_raw(lora_t *lora, const uint8_t *data, size_t len,
                       uint32_t timeout_ms, lora_cmd_callback_t callback,
                       void *user_data);

/**
 * @brief P2P 수신 콜백 등록
 * @param lora LoRa 핸들
 * @param callback 수신 콜백
 * @param user_data 사용자 데이터
 */
void lora_set_p2p_recv_callback(lora_t *lora, lora_p2p_recv_callback_t callback,
                                void *user_data);

/*===========================================================================
 * 초기화 시퀀스 API
 *===========================================================================*/

/**
 * @brief P2P Base 모드 초기화 (비동기)
 * @param lora LoRa 핸들
 * @param callback 완료 콜백
 * @return true: 시작 성공
 */
bool lora_init_p2p_base(lora_t *lora, lora_evt_handler_t callback);

/**
 * @brief P2P Rover 모드 초기화 (비동기)
 * @param lora LoRa 핸들
 * @param callback 완료 콜백
 * @return true: 시작 성공
 */
bool lora_init_p2p_rover(lora_t *lora, lora_evt_handler_t callback);

/*===========================================================================
 * 상태 조회 API
 *===========================================================================*/

/**
 * @brief 초기화 완료 여부
 * @param lora LoRa 핸들
 * @return true: 초기화 완료
 */
bool lora_is_ready(const lora_t *lora);

#endif /* LORA_H */
