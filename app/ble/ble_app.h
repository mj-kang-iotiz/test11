#ifndef BLE_APP_H
#define BLE_APP_H

/**
 * @file ble_app.h
 * @brief BLE 애플리케이션 헤더
 *
 * BLE 앱 레벨 기능:
 * - BLE 앱 시작/종료 (gps_app_start/stop 참고)
 * - RX 태스크 관리
 * - 모듈 초기화 시퀀스
 * - 외부 명령어 처리 (ble_cmd.h/c)
 */

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "ble.h"

#include <stdbool.h>
#include <stdint.h>

/*===========================================================================
 * BLE 앱 인스턴스 (gps_instance_t 참고)
 *===========================================================================*/
typedef struct {
    ble_t ble;                      /**< BLE 핸들 (lib/ble) - 태스크/큐 포함 */
    bool enabled;                   /**< 활성화 상태 */
} ble_instance_t;

/*===========================================================================
 * 앱 시작/종료 API (gps_app_start/stop 참고)
 *===========================================================================*/

/**
 * @brief BLE 앱 시작
 *
 * board_config.h 설정을 읽어서 BLE 앱 시작
 * - HAL 초기화
 * - RX 태스크 생성
 * - 모듈 초기 설정 (디바이스 이름 등)
 */
void ble_app_start(void);

/**
 * @brief BLE 앱 종료
 *
 * BLE 앱을 안전하게 종료하고 리소스 정리
 */
void ble_app_stop(void);

/*===========================================================================
 * BLE 핸들/인스턴스 API
 *===========================================================================*/

/**
 * @brief BLE 핸들 가져오기
 * @return BLE 핸들 포인터 (NULL: 비활성화)
 */
ble_t *ble_app_get_handle(void);

/**
 * @brief BLE 인스턴스 가져오기
 * @return BLE 인스턴스 포인터
 */
ble_instance_t *ble_app_get_instance(void);

/*===========================================================================
 * 데이터 송수신 API
 *===========================================================================*/

/**
 * @brief 데이터 전송 (Bypass 모드)
 *
 * 외부 PC로 데이터 전송
 *
 * @param data 전송 데이터
 * @param len 데이터 길이
 * @return true: 성공
 */
bool ble_app_send(const char *data, size_t len);

/*===========================================================================
 * AT 명령어 API (동기)
 *===========================================================================*/

/**
 * @brief 디바이스 이름 설정
 * @param name 디바이스 이름
 * @param timeout_ms 타임아웃 (ms)
 * @return true: 성공
 */
bool ble_app_set_device_name(const char *name, uint32_t timeout_ms);

/**
 * @brief 디바이스 이름 조회
 * @param name_buf 이름 버퍼
 * @param buf_size 버퍼 크기
 * @param timeout_ms 타임아웃 (ms)
 * @return true: 성공
 */
bool ble_app_get_device_name(char *name_buf, size_t buf_size, uint32_t timeout_ms);

/**
 * @brief UART 속도 설정
 * @param baudrate 속도 (예: 115200)
 * @param timeout_ms 타임아웃 (ms)
 * @return true: 성공
 */
bool ble_app_set_uart_baudrate(uint32_t baudrate, uint32_t timeout_ms);

/**
 * @brief Advertising 시작
 * @param timeout_ms 타임아웃 (ms)
 * @return true: 성공
 */
bool ble_app_start_advertising(uint32_t timeout_ms);

/**
 * @brief 연결 해제
 * @param timeout_ms 타임아웃 (ms)
 * @return true: 성공
 */
bool ble_app_disconnect(uint32_t timeout_ms);

/*===========================================================================
 * 상태 조회 API
 *===========================================================================*/

/**
 * @brief 연결 상태 조회
 * @return 연결 상태
 */
ble_conn_state_t ble_app_get_conn_state(void);

/**
 * @brief 연결 상태 설정 (GPIO 인터럽트에서 호출)
 * @param state 연결 상태
 */
void ble_app_set_conn_state(ble_conn_state_t state);

/**
 * @brief 현재 모드 조회
 * @return 현재 모드
 */
ble_mode_t ble_app_get_mode(void);

#endif /* BLE_APP_H */
