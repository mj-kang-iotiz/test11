#ifndef LORA_APP_H
#define LORA_APP_H

/**
 * @file lora_app.h
 * @brief LoRa 애플리케이션 헤더
 *
 * LoRa 앱 레벨 기능:
 * - LoRa 앱 시작/종료 (ble_app_start/stop 참고)
 * - 이벤트 버스 연동
 * - 외부 모듈과 디커플링
 */

#include "lora.h"
#include <stdbool.h>
#include <stdint.h>

/*===========================================================================
 * LoRa 앱 인스턴스 (ble_instance_t 참고)
 *===========================================================================*/
typedef struct {
    lora_t lora;                    /**< LoRa 핸들 (lib/lora) - 태스크/큐 포함 */
    bool enabled;                   /**< 활성화 상태 */
} lora_app_t;

/*===========================================================================
 * LoRa 작동 모드 (앱 레벨)
 *===========================================================================*/
typedef enum {
  LORA_WORK_MODE_LORAWAN = 0,
  LORA_WORK_MODE_P2P = 1
} lora_work_mode_t;

typedef enum {
  LORA_P2P_TRANSFER_MODE_RECEIVER = 1,
  LORA_P2P_TRANSFER_MODE_SENDER = 2
} lora_p2p_transfer_mode_t;

/*===========================================================================
 * 앱 시작/종료 API (ble_app_start/stop 참고)
 *===========================================================================*/

/**
 * @brief LoRa 앱 시작
 *
 * board_config.h 설정을 읽어서 LoRa 앱 시작
 * - HAL 초기화
 * - RX/TX 태스크 생성
 * - 이벤트 버스 구독 (Base 모드)
 */
void lora_app_start(void);

/**
 * @brief LoRa 앱 종료
 *
 * LoRa 앱을 안전하게 종료하고 리소스 정리
 */
void lora_app_stop(void);

/*===========================================================================
 * 핸들/인스턴스 API
 *===========================================================================*/

/**
 * @brief LoRa 핸들 가져오기
 * @return lora_t* LoRa 핸들 (NULL: 비활성화)
 */
lora_t *lora_app_get_handle(void);

/**
 * @brief LoRa 앱 인스턴스 가져오기
 * @return lora_app_t* 앱 인스턴스
 */
lora_app_t *lora_app_get_instance(void);

/*===========================================================================
 * 데이터 송신 API (래퍼)
 *===========================================================================*/

/**
 * @brief P2P Raw 데이터 전송 (비동기)
 *
 * lora_send_p2p_raw() 래퍼 - 앱 인스턴스 사용
 *
 * @param data Raw binary 데이터
 * @param len 데이터 길이 (최대 118)
 * @param timeout_ms 타임아웃
 * @param callback 완료 콜백
 * @param user_data 사용자 데이터
 * @return true: 큐 추가 성공
 */
bool lora_app_send_p2p_raw(const uint8_t *data, size_t len, uint32_t timeout_ms,
                           lora_cmd_callback_t callback, void *user_data);

/**
 * @brief AT 명령어 전송 (동기) - 래퍼
 * @param cmd AT 명령어
 * @param timeout_ms 타임아웃
 * @return true: OK
 */
bool lora_app_send_cmd_sync(const char *cmd, uint32_t timeout_ms);

/**
 * @brief AT 명령어 전송 (비동기) - 래퍼
 */
bool lora_app_send_cmd_async(const char *cmd, uint32_t timeout_ms, uint32_t toa_ms,
                             lora_cmd_callback_t callback, void *user_data,
                             bool skip_response);

/*===========================================================================
 * P2P 설정 API
 *===========================================================================*/

/**
 * @brief LoRa 작동 모드 설정
 */
bool lora_app_set_work_mode(lora_work_mode_t mode, uint32_t timeout_ms);

/**
 * @brief LoRa P2P 설정
 */
bool lora_app_set_p2p_config(uint32_t freq, uint8_t sf, uint8_t bw, uint8_t cr,
                             uint16_t preamlen, uint8_t pwr, uint32_t timeout_ms);

/**
 * @brief LoRa P2P 전송 모드 설정
 */
bool lora_app_set_p2p_transfer_mode(lora_p2p_transfer_mode_t mode, uint32_t timeout_ms);

/**
 * @brief P2P 수신 콜백 등록
 */
void lora_app_set_p2p_recv_callback(lora_p2p_recv_callback_t callback, void *user_data);

/*===========================================================================
 * 상태 조회 API
 *===========================================================================*/

/**
 * @brief LoRa 준비 상태
 * @return true: 초기화 완료
 */
bool lora_app_is_ready(void);

/*===========================================================================
 * 하위 호환성 API (기존 코드 지원)
 *===========================================================================*/

/* 기존 함수명 유지 - deprecated, lora_app_* 사용 권장 */
void lora_instance_init(void);
void lora_instance_deinit(void);
void lora_start_rover(void);
lora_t *lora_get_handle(void);

/* 기존 전역 함수 - deprecated */
bool lora_send_command_sync(const char *cmd, uint32_t timeout_ms);
bool lora_send_command_async(const char *cmd, uint32_t timeout_ms, uint32_t toa_ms,
                             lora_cmd_callback_t callback, void *user_data,
                             bool skip_response);
bool lora_send_p2p_raw_async(const uint8_t *data, size_t len, uint32_t timeout_ms,
                             lora_cmd_callback_t callback, void *user_data);
bool lora_set_work_mode(lora_work_mode_t mode, uint32_t timeout_ms);
bool lora_set_p2p_config(uint32_t freq, uint8_t sf, uint8_t bw, uint8_t cr,
                         uint16_t preamlen, uint8_t pwr, uint32_t timeout_ms);
bool lora_set_p2p_transfer_mode(lora_p2p_transfer_mode_t mode, uint32_t timeout_ms);

#endif /* LORA_APP_H */
