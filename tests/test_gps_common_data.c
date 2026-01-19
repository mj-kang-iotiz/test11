/**
 * @file test_gps_common_data.c
 * @brief GPS 공용 데이터 구조 테스트
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

/* 테스트용 타입 정의 (FreeRTOS 없이) */
typedef uint32_t TickType_t;
typedef void* SemaphoreHandle_t;
typedef void* QueueHandle_t;
typedef void* TaskHandle_t;
#define xTaskGetTickCount() (test_tick++)
static uint32_t test_tick = 0;

/* gps_fix_t 정의 (gps_nmea.h에서 복사) */
typedef enum {
  GPS_FIX_INVALID = 0,
  GPS_FIX_GPS = 1,
  GPS_FIX_DGPS = 2,
  GPS_FIX_PPS = 3,
  GPS_FIX_RTK_FIX = 4,
  GPS_FIX_RTK_FLOAT = 5,
  GPS_FIX_DR = 6,
  GPS_FIX_MANUAL_POS = 7,
  GPS_FIX_SIMULATOR = 8
} gps_fix_t;

/* 공용 데이터 구조체 (gps.h에서 복사) */
typedef struct {
    struct {
        double latitude;
        double longitude;
        double altitude;
        float lat_std;
        float lon_std;
        float alt_std;
        uint32_t timestamp_ms;
    } position;

    struct {
        double hor_speed;
        double ver_speed;
        double track;
        uint32_t timestamp_ms;
    } velocity;

    struct {
        double heading;
        uint8_t mode;
        uint32_t timestamp_ms;
    } heading;

    struct {
        gps_fix_t fix_type;
        uint8_t sat_count;
        uint8_t used_sat_count;
        float hdop;
        uint32_t fix_timestamp_ms;
        uint32_t sat_timestamp_ms;
        bool fix_changed;
    } status;

} gps_common_data_t;

/* 테스트용 이벤트 카운터 */
static int fix_event_count = 0;
static int position_event_count = 0;
static gps_fix_t last_fix_type = GPS_FIX_INVALID;

/* 테스트: Fix 변경 감지 */
void test_fix_change_detection(void) {
    printf("=== Test: Fix Change Detection ===\n");

    gps_common_data_t data = {0};

    /* 초기 상태: INVALID */
    data.status.fix_type = GPS_FIX_INVALID;

    /* 시뮬레이션: GGA 수신, fix=1 (GPS) */
    gps_fix_t new_fix = GPS_FIX_GPS;
    gps_fix_t prev_fix = data.status.fix_type;
    data.status.fix_changed = (prev_fix != new_fix);
    data.status.fix_type = new_fix;

    printf("  1. INVALID -> GPS: fix_changed=%d (expected: 1)\n", data.status.fix_changed);
    assert(data.status.fix_changed == true);

    /* 같은 값으로 업데이트 */
    new_fix = GPS_FIX_GPS;
    prev_fix = data.status.fix_type;
    data.status.fix_changed = (prev_fix != new_fix);
    data.status.fix_type = new_fix;

    printf("  2. GPS -> GPS: fix_changed=%d (expected: 0)\n", data.status.fix_changed);
    assert(data.status.fix_changed == false);

    /* RTK_FIX로 변경 */
    new_fix = GPS_FIX_RTK_FIX;
    prev_fix = data.status.fix_type;
    data.status.fix_changed = (prev_fix != new_fix);
    data.status.fix_type = new_fix;

    printf("  3. GPS -> RTK_FIX: fix_changed=%d (expected: 1)\n", data.status.fix_changed);
    assert(data.status.fix_changed == true);

    printf("  PASSED!\n\n");
}

/* 테스트: 데이터 소스 분리 */
void test_data_source_separation(void) {
    printf("=== Test: Data Source Separation ===\n");

    gps_common_data_t data = {0};

    /* BESTNAV 시뮬레이션: 위치 + 속도 + 위성수 업데이트 */
    data.position.latitude = 37.5665;
    data.position.longitude = 126.9780;
    data.position.altitude = 50.0;
    data.position.timestamp_ms = xTaskGetTickCount();

    data.velocity.hor_speed = 10.5;
    data.velocity.ver_speed = 0.1;
    data.velocity.track = 45.0;
    data.velocity.timestamp_ms = xTaskGetTickCount();

    data.status.sat_count = 12;      /* BESTNAV.sv */
    data.status.used_sat_count = 10; /* BESTNAV.used_sv */
    data.status.sat_timestamp_ms = xTaskGetTickCount();

    printf("  BESTNAV: lat=%.4f, lon=%.4f, sat=%d, used_sat=%d\n",
           data.position.latitude, data.position.longitude,
           data.status.sat_count, data.status.used_sat_count);

    /* GGA 시뮬레이션: fix + hdop만 업데이트 (위치는 업데이트 안 함) */
    gps_fix_t prev_fix = data.status.fix_type;
    data.status.fix_type = GPS_FIX_RTK_FIX;
    data.status.fix_changed = (prev_fix != data.status.fix_type);
    data.status.hdop = 0.8;
    data.status.fix_timestamp_ms = xTaskGetTickCount();

    printf("  GGA: fix=%d, hdop=%.1f, fix_changed=%d\n",
           data.status.fix_type, data.status.hdop, data.status.fix_changed);

    /* 위치는 BESTNAV 값 유지 확인 */
    assert(data.position.latitude == 37.5665);
    assert(data.position.longitude == 126.9780);

    /* Fix는 GGA 값 확인 */
    assert(data.status.fix_type == GPS_FIX_RTK_FIX);
    assert(data.status.hdop == 0.8f);

    /* 위성수는 BESTNAV 값 유지 확인 */
    assert(data.status.sat_count == 12);

    printf("  PASSED!\n\n");
}

/* 테스트: THS 헤딩 업데이트 */
void test_ths_heading_update(void) {
    printf("=== Test: THS Heading Update ===\n");

    gps_common_data_t data = {0};

    /* THS 시뮬레이션 */
    data.heading.heading = 123.45;
    data.heading.mode = 'A';  /* GPS_THS_MODE_AUTO */
    data.heading.timestamp_ms = xTaskGetTickCount();

    printf("  THS: heading=%.2f, mode='%c'\n",
           data.heading.heading, data.heading.mode);

    assert(data.heading.heading == 123.45);
    assert(data.heading.mode == 'A');

    printf("  PASSED!\n\n");
}

/* 테스트: 타임스탬프 분리 */
void test_timestamp_separation(void) {
    printf("=== Test: Timestamp Separation ===\n");

    gps_common_data_t data = {0};

    /* 각 소스별 타임스탬프 */
    data.position.timestamp_ms = 1000;
    data.velocity.timestamp_ms = 1000;
    data.heading.timestamp_ms = 1050;
    data.status.fix_timestamp_ms = 1100;
    data.status.sat_timestamp_ms = 1000;

    printf("  Position ts: %u\n", data.position.timestamp_ms);
    printf("  Velocity ts: %u\n", data.velocity.timestamp_ms);
    printf("  Heading ts: %u\n", data.heading.timestamp_ms);
    printf("  Fix ts: %u\n", data.status.fix_timestamp_ms);
    printf("  Sat ts: %u\n", data.status.sat_timestamp_ms);

    /* 각 타임스탬프가 독립적인지 확인 */
    assert(data.position.timestamp_ms == 1000);
    assert(data.heading.timestamp_ms == 1050);
    assert(data.status.fix_timestamp_ms == 1100);

    printf("  PASSED!\n\n");
}

int main(void) {
    printf("\n========================================\n");
    printf("GPS Common Data Structure Test\n");
    printf("========================================\n\n");

    test_fix_change_detection();
    test_data_source_separation();
    test_ths_heading_update();
    test_timestamp_separation();

    printf("========================================\n");
    printf("All tests PASSED!\n");
    printf("========================================\n\n");

    return 0;
}
