#ifndef BLE_PORT_H
#define BLE_PORT_H

/**
 * @file ble_port.h
 * @brief BLE UART 포트 (STM32 HAL)
 *
 * UART5 (PC12 TX, PD2 RX) + DMA1_Stream0
 */

#include <stdbool.h>
#include <stdint.h>
#include "ble.h"
#include "FreeRTOS.h"
#include "queue.h"

/**
 * @brief BLE 포트 초기화
 *
 * HAL ops 설정 및 UART/DMA 초기화
 *
 * @param ble BLE 핸들
 * @return 0: 성공, -1: 실패
 */
int ble_port_init_instance(ble_t *ble);

/**
 * @brief BLE 포트 시작
 *
 * UART DMA 활성화 및 모듈 설정
 *
 * @param ble BLE 핸들
 */
void ble_port_start(ble_t *ble);

/**
 * @brief BLE 포트 종료
 * @param ble BLE 핸들
 */
void ble_port_stop(ble_t *ble);

/**
 * @brief DMA RX 현재 위치 조회
 * @return 현재 위치
 */
uint32_t ble_port_get_rx_pos(void);

/**
 * @brief DMA RX 버퍼 포인터 조회
 * @return 버퍼 포인터
 */
char *ble_port_get_recv_buf(void);

/**
 * @brief RX 신호 큐 설정
 * @param queue RX 큐
 */
void ble_port_set_queue(QueueHandle_t queue);

/**
 * @brief BLE 핸들 설정 (링버퍼 접근용)
 * @param ble BLE 핸들
 */
void ble_port_set_ble_handle(ble_t *ble);

/**
 * @brief Polling 방식 라인 수신 (초기화용)
 * @param buf 버퍼
 * @param buf_size 버퍼 크기
 * @param timeout_ms 타임아웃 (ms)
 * @return 수신 바이트 수, -1: 타임아웃
 */
int ble_uart5_recv_line_poll(char *buf, size_t buf_size, uint32_t timeout_ms);

#endif /* BLE_PORT_H */
