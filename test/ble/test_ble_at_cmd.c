/**
 * @file test_ble_at_cmd.c
 * @brief BLE AT 명령어 처리 테스트 (NLE521)
 *
 * 이 테스트는 FreeRTOS 의존성 때문에 PC에서 직접 실행 불가.
 * 타겟 보드에서 테스트하거나, FreeRTOS 모의 환경 필요.
 *
 * 테스트 시나리오 (타겟 보드):
 * 1. ble_app_start() 호출
 * 2. ble_send_at_cmd_sync() 호출하여 AT 명령 전송
 * 3. 응답 확인
 */

#include <stdio.h>
#include <string.h>

#ifdef USE_FREERTOS
#include "FreeRTOS.h"
#include "task.h"
#include "ble_app.h"
#include "ble.h"
#include "log.h"

#ifndef TAG
#define TAG "BLE_TEST"
#endif

/**
 * @brief AT 명령어 테스트 태스크
 */
static void ble_test_task(void *pvParameter)
{
    char response[128];
    ble_at_status_t status;
    ble_t *ble = ble_app_get_handle();

    LOG_INFO("=== BLE AT 명령어 테스트 시작 ===");

    vTaskDelay(pdMS_TO_TICKS(2000)); /* BLE 모듈 초기화 대기 */

    /* 테스트 1: AT 명령 (기본 확인) */
    LOG_INFO("테스트 1: AT 명령");
    status = ble_send_at_cmd_sync(ble, "AT\r", response, sizeof(response), 1000);
    if (status == BLE_AT_STATUS_COMPLETED) {
        LOG_INFO("  [PASS] AT 응답: %s", response);
    } else {
        LOG_ERR("  [FAIL] AT 명령 실패 (status=%d)", status);
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    /* 테스트 2: AT+MANUF? (디바이스 이름 조회) */
    LOG_INFO("테스트 2: AT+MANUF?");
    status = ble_send_at_cmd_sync(ble, "AT+MANUF?\r", response, sizeof(response), 1000);
    if (status == BLE_AT_STATUS_COMPLETED) {
        LOG_INFO("  [PASS] 디바이스 이름: %s", response);
    } else {
        LOG_ERR("  [FAIL] AT+MANUF? 실패 (status=%d)", status);
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    /* 테스트 3: AT+UART? (UART 설정 조회) */
    LOG_INFO("테스트 3: AT+UART?");
    status = ble_send_at_cmd_sync(ble, "AT+UART?\r", response, sizeof(response), 1000);
    if (status == BLE_AT_STATUS_COMPLETED) {
        LOG_INFO("  [PASS] UART 설정: %s", response);
    } else {
        LOG_ERR("  [FAIL] AT+UART? 실패 (status=%d)", status);
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    /* 테스트 4: AT+VERSION? (펌웨어 버전 조회) */
    LOG_INFO("테스트 4: AT+VERSION?");
    status = ble_send_at_cmd_sync(ble, "AT+VERSION?\r", response, sizeof(response), 1000);
    if (status == BLE_AT_STATUS_COMPLETED) {
        LOG_INFO("  [PASS] 펌웨어 버전: %s", response);
    } else {
        LOG_ERR("  [FAIL] AT+VERSION? 실패 (status=%d)", status);
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    /* 테스트 5: AT+MAC? (MAC 주소 조회) */
    LOG_INFO("테스트 5: AT+MAC?");
    status = ble_send_at_cmd_sync(ble, "AT+MAC?\r", response, sizeof(response), 1000);
    if (status == BLE_AT_STATUS_COMPLETED) {
        LOG_INFO("  [PASS] MAC 주소: %s", response);
    } else {
        LOG_ERR("  [FAIL] AT+MAC? 실패 (status=%d)", status);
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    /* 테스트 6: 잘못된 명령어 */
    LOG_INFO("테스트 6: 잘못된 명령어");
    status = ble_send_at_cmd_sync(ble, "AT+INVALID\r", response, sizeof(response), 1000);
    if (status == BLE_AT_STATUS_ERROR) {
        LOG_INFO("  [PASS] 에러 응답 수신: %s", response);
    } else if (status == BLE_AT_STATUS_TIMEOUT) {
        LOG_INFO("  [PASS] 타임아웃 (예상된 동작)");
    } else {
        LOG_WARN("  [WARN] 예상치 못한 응답 (status=%d)", status);
    }

    LOG_INFO("=== BLE AT 명령어 테스트 완료 ===");

    vTaskDelete(NULL);
}

/**
 * @brief BLE 테스트 시작
 */
void ble_test_start(void)
{
    xTaskCreate(ble_test_task, "ble_test", 2048, NULL, 1, NULL);
}

#else /* USE_FREERTOS not defined */

/*
 * PC에서 실행 시 안내 메시지
 */
int main(void)
{
    printf("========================================\n");
    printf(" BLE AT Command Test (NLE521)\n");
    printf("========================================\n");
    printf("\n");
    printf("이 테스트는 타겟 보드에서 실행해야 합니다.\n");
    printf("\n");
    printf("사용법:\n");
    printf("  1. 프로젝트에 이 파일 추가\n");
    printf("  2. USE_FREERTOS 정의\n");
    printf("  3. main.c에서 ble_test_start() 호출\n");
    printf("\n");
    printf("테스트 항목:\n");
    printf("  - AT (기본 확인)\n");
    printf("  - AT+MANUF? (디바이스 이름)\n");
    printf("  - AT+UART? (UART 설정)\n");
    printf("  - AT+VERSION? (펌웨어 버전)\n");
    printf("  - AT+MAC? (MAC 주소)\n");
    printf("  - 잘못된 명령어 에러 처리\n");
    printf("\n");
    printf("========================================\n");

    return 0;
}

#endif /* USE_FREERTOS */
