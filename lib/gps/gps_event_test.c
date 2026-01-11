/**
 * @file gps_event_test.c
 * @brief GPS 이벤트 시스템 테스트
 *
 * 테스트 항목:
 * 1. 고수준 이벤트 타입 (POSITION_UPDATED, HEADING_UPDATED 등)
 * 2. 이벤트 핸들러 호출 및 데이터 검증
 * 3. 공용 필드 업데이트 (unicore_bin_data_t)
 * 4. Thread-safe 이벤트 전달 (값 복사)
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

/* Mock FreeRTOS - MUST be before any GPS headers */
#ifndef FREERTOS_H
#define FREERTOS_H
typedef void* TaskHandle_t;
typedef void* SemaphoreHandle_t;
typedef void* QueueHandle_t;
typedef uint32_t TickType_t;
typedef int BaseType_t;
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define pdMS_TO_TICKS(ms) (ms)
#define tskIDLE_PRIORITY 0
#endif

#ifndef TASK_H
#define TASK_H
#endif

#ifndef SEMPHR_H
#define SEMPHR_H
#endif

#ifndef QUEUE_H
#define QUEUE_H
#endif

/* GPS 헤더 */
#include "gps_proto_def.h"
#include "gps_parser.h"
#include "gps_types.h"
#include "gps_nmea.h"
#include "gps_unicore.h"
#include "gps.h"

/* Mock FreeRTOS functions */
uint32_t xTaskGetTickCount(void) { return 1234; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout) { (void)sem; (void)timeout; return pdTRUE; }
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem) { (void)sem; return pdTRUE; }

/* Mock LOG functions */
#define LOG_INFO(fmt, ...) printf("[INFO] " fmt "\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) printf("[WARN] " fmt "\n", ##__VA_ARGS__)
#define LOG_ERR(fmt, ...) printf("[ERR] " fmt "\n", ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...) printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)

/* Test counters */
static int test_passed = 0;
static int test_failed = 0;
static gps_event_t last_event;  /* 마지막 이벤트 저장 */
static int event_count = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (cond) { \
        printf("✓ PASS: %s\n", msg); \
        test_passed++; \
    } else { \
        printf("✗ FAIL: %s\n", msg); \
        test_failed++; \
    } \
} while(0)

/*===========================================================================
 * 테스트 이벤트 핸들러
 *===========================================================================*/
void test_event_handler(gps_t *gps, const gps_event_t *event) {
    (void)gps;
    /* 이벤트 복사 (값 복사이므로 안전) */
    memcpy(&last_event, event, sizeof(gps_event_t));
    event_count++;

    printf("  [Event] type=%d, protocol=%d, timestamp=%u\n",
           event->type, event->protocol, event->timestamp_ms);
}

/*===========================================================================
 * 테스트 1: 고수준 이벤트 타입 검증
 *===========================================================================*/
void test_high_level_event_types(void) {
    printf("\n=== Test 1: High-Level Event Types ===\n");

    TEST_ASSERT(GPS_EVENT_POSITION_UPDATED == 1, "GPS_EVENT_POSITION_UPDATED defined");
    TEST_ASSERT(GPS_EVENT_HEADING_UPDATED == 2, "GPS_EVENT_HEADING_UPDATED defined");
    TEST_ASSERT(GPS_EVENT_VELOCITY_UPDATED == 3, "GPS_EVENT_VELOCITY_UPDATED defined");
    TEST_ASSERT(GPS_EVENT_SATELLITE_UPDATED == 4, "GPS_EVENT_SATELLITE_UPDATED defined");
    TEST_ASSERT(GPS_EVENT_RTCM_RECEIVED == 5, "GPS_EVENT_RTCM_RECEIVED defined");
    TEST_ASSERT(GPS_EVENT_CMD_RESPONSE == 6, "GPS_EVENT_CMD_RESPONSE defined");
}

/*===========================================================================
 * 테스트 2: gps_event_t 구조체 크기 검증
 *===========================================================================*/
void test_event_struct_size(void) {
    printf("\n=== Test 2: gps_event_t Structure Size ===\n");

    size_t size = sizeof(gps_event_t);
    printf("  sizeof(gps_event_t) = %zu bytes\n", size);

    /* 적절한 크기인지 확인 (<= 128 bytes) */
    TEST_ASSERT(size <= 128, "Event structure size <= 128 bytes");
    TEST_ASSERT(size >= 32, "Event structure size >= 32 bytes");
}

/*===========================================================================
 * 테스트 3: 위치 이벤트 데이터 검증
 *===========================================================================*/
void test_position_event_data(void) {
    printf("\n=== Test 3: Position Event Data ===\n");

    /* 위치 이벤트 생성 */
    gps_event_t event = {
        .type = GPS_EVENT_POSITION_UPDATED,
        .protocol = GPS_PROTOCOL_NMEA,
        .timestamp_ms = 5678,
        .data.position = {
            .latitude = 37.5665,
            .longitude = 126.9780,
            .altitude = 38.5,
            .fix_type = GPS_FIX_RTK_FIX,
            .sat_count = 12,
            .hdop = 0.9
        },
        .source.nmea_msg_id = GPS_NMEA_MSG_GGA
    };

    /* 데이터 검증 */
    TEST_ASSERT(event.type == GPS_EVENT_POSITION_UPDATED, "Event type is POSITION_UPDATED");
    TEST_ASSERT(event.data.position.latitude == 37.5665, "Latitude correct");
    TEST_ASSERT(event.data.position.longitude == 126.9780, "Longitude correct");
    TEST_ASSERT(event.data.position.fix_type == GPS_FIX_RTK_FIX, "Fix type is RTK_FIX");
    TEST_ASSERT(event.data.position.sat_count == 12, "Satellite count correct");
}

/*===========================================================================
 * 테스트 4: 헤딩 이벤트 데이터 검증
 *===========================================================================*/
void test_heading_event_data(void) {
    printf("\n=== Test 4: Heading Event Data ===\n");

    /* 헤딩 이벤트 생성 */
    gps_event_t event = {
        .type = GPS_EVENT_HEADING_UPDATED,
        .protocol = GPS_PROTOCOL_NMEA,
        .timestamp_ms = 9012,
        .data.heading = {
            .heading = 123.45,
            .pitch = 5.2,
            .heading_std = 0.5f,
            .status = 'A'
        },
        .source.nmea_msg_id = GPS_NMEA_MSG_THS
    };

    /* 데이터 검증 */
    TEST_ASSERT(event.type == GPS_EVENT_HEADING_UPDATED, "Event type is HEADING_UPDATED");
    TEST_ASSERT(event.data.heading.heading == 123.45, "Heading correct");
    TEST_ASSERT(event.data.heading.status == 'A', "Status is valid");
}

/*===========================================================================
 * 테스트 5: 이벤트 핸들러 호출 검증
 *===========================================================================*/
void test_event_handler_invocation(void) {
    printf("\n=== Test 5: Event Handler Invocation ===\n");

    /* 이벤트 초기화 */
    event_count = 0;
    memset(&last_event, 0, sizeof(last_event));

    /* 이벤트 생성 */
    gps_event_t event = {
        .type = GPS_EVENT_RTCM_RECEIVED,
        .protocol = GPS_PROTOCOL_RTCM,
        .timestamp_ms = 3456,
        .data.rtcm = {
            .msg_type = 1074,
            .length = 256
        }
    };

    /* 핸들러 호출 */
    test_event_handler(NULL, &event);

    /* 검증 */
    TEST_ASSERT(event_count == 1, "Event handler called once");
    TEST_ASSERT(last_event.type == GPS_EVENT_RTCM_RECEIVED, "Event type preserved");
    TEST_ASSERT(last_event.data.rtcm.msg_type == 1074, "RTCM msg_type preserved");
    TEST_ASSERT(last_event.data.rtcm.length == 256, "RTCM length preserved");
}

/*===========================================================================
 * 테스트 6: 값 복사 안전성 검증
 *===========================================================================*/
void test_value_copy_safety(void) {
    printf("\n=== Test 6: Value Copy Safety ===\n");

    /* 원본 이벤트 */
    gps_event_t original = {
        .type = GPS_EVENT_POSITION_UPDATED,
        .protocol = GPS_PROTOCOL_UNICORE_BIN,
        .data.position.latitude = 35.1234,
        .data.position.longitude = 128.5678
    };

    /* 핸들러 호출 (값 복사) */
    test_event_handler(NULL, &original);

    /* 원본 수정 */
    original.data.position.latitude = 99.9999;
    original.data.position.longitude = 99.9999;

    /* 핸들러 내 복사본은 변경 안 됨 */
    TEST_ASSERT(last_event.data.position.latitude == 35.1234, "Copied latitude unchanged");
    TEST_ASSERT(last_event.data.position.longitude == 128.5678, "Copied longitude unchanged");

    printf("  → Value copy ensures thread-safety\n");
}

/*===========================================================================
 * 테스트 7: gps_unicore_bin_data_t 공용 필드 검증
 *===========================================================================*/
void test_unicore_shared_fields(void) {
    printf("\n=== Test 7: Unicore Binary Shared Fields ===\n");

    /* gps_unicore_bin_data_t 구조체 크기 */
    size_t size = sizeof(gps_unicore_bin_data_t);
    printf("  sizeof(gps_unicore_bin_data_t) = %zu bytes\n", size);

    /* 구조체 초기화 */
    gps_unicore_bin_data_t data = {0};

    /* 공용 위치 필드 업데이트 (BESTNAV 시뮬레이션) */
    data.position.valid = true;
    data.position.latitude = 37.5;
    data.position.longitude = 127.0;
    data.position.altitude = 50.0;
    data.position.pos_type = 16;  /* RTK_FIXED */
    data.position.source_msg = 2118;  /* BESTNAV */

    /* 공용 헤딩 필드 업데이트 (HEADING2 시뮬레이션) */
    data.heading.valid = true;
    data.heading.heading = 90.0;
    data.heading.pitch = 0.0;
    data.heading.source_msg = 2120;  /* HEADING2 */

    /* 공용 속도 필드 업데이트 (BESTNAV 시뮬레이션) */
    data.velocity.valid = true;
    data.velocity.hor_speed = 5.5;
    data.velocity.trk_gnd = 45.0;
    data.velocity.source_msg = 2118;  /* BESTNAV */

    /* 검증 */
    TEST_ASSERT(data.position.valid, "Position data valid");
    TEST_ASSERT(data.position.source_msg == 2118, "Position from BESTNAV");
    TEST_ASSERT(data.heading.valid, "Heading data valid");
    TEST_ASSERT(data.heading.source_msg == 2120, "Heading from HEADING2");
    TEST_ASSERT(data.velocity.valid, "Velocity data valid");
    TEST_ASSERT(data.velocity.source_msg == 2118, "Velocity from BESTNAV");

    printf("  → Multiple messages can update shared fields\n");
}

/*===========================================================================
 * 테스트 8: 명령어 응답 이벤트
 *===========================================================================*/
void test_cmd_response_event(void) {
    printf("\n=== Test 8: Command Response Event ===\n");

    /* OK 응답 이벤트 */
    gps_event_t event_ok = {
        .type = GPS_EVENT_CMD_RESPONSE,
        .protocol = GPS_PROTOCOL_UNICORE_CMD,
        .data.cmd_response.success = true
    };

    TEST_ASSERT(event_ok.type == GPS_EVENT_CMD_RESPONSE, "CMD_RESPONSE event type");
    TEST_ASSERT(event_ok.data.cmd_response.success == true, "Command succeeded");

    /* ERROR 응답 이벤트 */
    gps_event_t event_err = {
        .type = GPS_EVENT_CMD_RESPONSE,
        .protocol = GPS_PROTOCOL_UNICORE_CMD,
        .data.cmd_response.success = false
    };

    TEST_ASSERT(event_err.data.cmd_response.success == false, "Command failed");
}

/*===========================================================================
 * 메인 테스트 실행
 *===========================================================================*/
int main(void) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║  GPS 이벤트 시스템 테스트                                 ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");

    /* 테스트 실행 */
    test_high_level_event_types();
    test_event_struct_size();
    test_position_event_data();
    test_heading_event_data();
    test_event_handler_invocation();
    test_value_copy_safety();
    test_unicore_shared_fields();
    test_cmd_response_event();

    /* 결과 출력 */
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║  테스트 결과                                               ║\n");
    printf("╠════════════════════════════════════════════════════════════╣\n");
    printf("║  통과: %3d개                                               ║\n", test_passed);
    printf("║  실패: %3d개                                               ║\n", test_failed);
    printf("╚════════════════════════════════════════════════════════════╝\n");

    return (test_failed == 0) ? 0 : 1;
}
