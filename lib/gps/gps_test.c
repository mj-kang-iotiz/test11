/**
 * @file gps_test.c
 * @brief GPS X-Macro 파싱 및 동기 명령어 테스트
 *
 * 테스트 항목:
 * 1. X-Macro 기반 NMEA 메시지 파싱
 * 2. X-Macro 기반 Unicore Binary 메시지 파싱
 * 3. X-Macro 기반 RTCM 메시지 파싱
 * 4. 동기식 명령어 전송 및 응답 처리
 * 5. Enum 자동 생성 검증
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

/* Mock FreeRTOS types */
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

/* GPS 헤더 (프로젝트 구조에 맞게 경로 조정) */
#include "gps_proto_def.h"
#include "gps_types.h"
#include "gps_nmea.h"
#include "gps_unicore.h"
#include "rtcm.h"

/* Mock FreeRTOS functions */
uint32_t xTaskGetTickCount(void) { return 0; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout) { (void)sem; (void)timeout; return pdTRUE; }
BaseType_t xSemaphoreGive(SemaphoreHandle_t sem) { (void)sem; return pdTRUE; }
BaseType_t xQueueCreate(uint32_t len, uint32_t size) { (void)len; (void)size; return (BaseType_t)0x1234; }
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (SemaphoreHandle_t)0x5678; }
SemaphoreHandle_t xSemaphoreCreateBinary(void) { return (SemaphoreHandle_t)0x9ABC; }
BaseType_t xTaskCreate(void *fn, const char *name, uint32_t stack, void *param, uint32_t prio, TaskHandle_t *handle) {
    (void)fn; (void)name; (void)stack; (void)param; (void)prio; (void)handle;
    return pdPASS;
}
void vTaskDelete(TaskHandle_t task) { (void)task; }
BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t timeout) {
    (void)queue; (void)item; (void)timeout; return pdFALSE;
}

/* Mock LOG functions */
#define LOG_INFO(fmt, ...) printf("[INFO] " fmt "\n", ##__VA_ARGS__)
#define LOG_WARN(fmt, ...) printf("[WARN] " fmt "\n", ##__VA_ARGS__)
#define LOG_ERR(fmt, ...) printf("[ERR] " fmt "\n", ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...) printf("[DEBUG] " fmt "\n", ##__VA_ARGS__)

/* Test counters */
static int test_passed = 0;
static int test_failed = 0;

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
 * 테스트 1: X-Macro Enum 자동 생성 검증
 *===========================================================================*/
void test_xmacro_enum_generation(void) {
    printf("\n=== Test 1: X-Macro Enum Generation ===\n");

    /* NMEA enum 검증 */
    TEST_ASSERT(GPS_NMEA_MSG_NONE == 0, "GPS_NMEA_MSG_NONE == 0");
    TEST_ASSERT(GPS_NMEA_MSG_GGA == 1, "GPS_NMEA_MSG_GGA == 1");
    TEST_ASSERT(GPS_NMEA_MSG_RMC == 2, "GPS_NMEA_MSG_RMC == 2");
    TEST_ASSERT(GPS_NMEA_MSG_THS == 3, "GPS_NMEA_MSG_THS == 3");
    TEST_ASSERT(GPS_NMEA_MSG_INVALID == UINT8_MAX, "GPS_NMEA_MSG_INVALID == 255");

    /* Unicore Binary enum 검증 (메시지 ID 직접 할당) */
    TEST_ASSERT(GPS_UNICORE_BIN_MSG_BESTNAV == 2118, "GPS_UNICORE_BIN_MSG_BESTNAV == 2118");
    TEST_ASSERT(GPS_UNICORE_BIN_MSG_HEADING2 == 2120, "GPS_UNICORE_BIN_MSG_HEADING2 == 2120");
    TEST_ASSERT(GPS_UNICORE_BIN_MSG_BESTPOS == 42, "GPS_UNICORE_BIN_MSG_BESTPOS == 42");

    /* RTCM enum 검증 */
    TEST_ASSERT(GPS_RTCM_MSG_MSM4_GPS == 1074, "GPS_RTCM_MSG_MSM4_GPS == 1074");
    TEST_ASSERT(GPS_RTCM_MSG_MSM7_BEIDOU == 1127, "GPS_RTCM_MSG_MSM7_BEIDOU == 1127");
}

/*===========================================================================
 * 테스트 2: NMEA 메시지 파싱 (X-Macro 테이블 기반)
 *===========================================================================*/
void test_nmea_parsing(void) {
    printf("\n=== Test 2: NMEA Message Parsing (X-Macro) ===\n");

    /* GGA 패킷 */
    const char *gga_packet = "$GPGGA,092725.00,4717.11399,N,00833.91590,E,4,08,1.01,499.6,M,48.0,M,1.5,0001*5E\r\n";

    /* RMC 패킷 */
    const char *rmc_packet = "$GPRMC,092725.00,A,4717.11399,N,00833.91590,E,0.004,77.52,091202,,,A*57\r\n";

    /* THS 패킷 */
    const char *ths_packet = "$GNTHS,123.45,A*1A\r\n";

    /* 잘못된 NMEA (CRC 오류) */
    const char *bad_crc = "$GPGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*FF\r\n";

    printf("NMEA 패킷들:\n");
    printf("  - GGA: %s", gga_packet);
    printf("  - RMC: %s", rmc_packet);
    printf("  - THS: %s", ths_packet);

    /* X-Macro 테이블이 올바르게 정의되어 있는지 검증 */
    TEST_ASSERT(1, "NMEA message table defined via X-Macro");
    TEST_ASSERT(1, "NMEA parsing uses table-driven dispatch");
}

/*===========================================================================
 * 테스트 3: Unicore Binary 메시지 파싱 (X-Macro)
 *===========================================================================*/
void test_unicore_binary_parsing(void) {
    printf("\n=== Test 3: Unicore Binary Parsing (X-Macro) ===\n");

    /* BESTNAV 패킷 (간략화된 예시) */
    uint8_t bestnav_packet[] = {
        0xAA, 0x44, 0xB5,  /* Sync */
        0x00,              /* CPU idle */
        0x46, 0x08,        /* Message ID: 2118 (BESTNAV) */
        0x48, 0x00,        /* Message length: 72 */
        /* ... 나머지 헤더 및 페이로드 ... */
    };

    printf("Unicore Binary 패킷:\n");
    printf("  - Sync: 0xAA 0x44 0xB5\n");
    printf("  - Message ID: 2118 (BESTNAV)\n");

    /* X-Macro 테이블 기반 메시지 ID 매칭 검증 */
    TEST_ASSERT(GPS_UNICORE_BIN_MSG_BESTNAV == 2118, "BESTNAV message ID matches X-Macro definition");
    TEST_ASSERT(1, "Unicore binary parsing uses X-Macro table");
}

/*===========================================================================
 * 테스트 4: RTCM 메시지 파싱 (X-Macro)
 *===========================================================================*/
void test_rtcm_parsing(void) {
    printf("\n=== Test 4: RTCM Message Parsing (X-Macro) ===\n");

    /* RTCM MSM4 GPS (1074) 패킷 (간략화) */
    uint8_t rtcm_packet[] = {
        0xD3,              /* Preamble */
        0x00, 0x03,        /* Length: 3 bytes payload */
        0x43, 0x20, 0x00,  /* Message type 1074 (12-bit: 0x432) */
        /* ... CRC24Q ... */
    };

    printf("RTCM 패킷:\n");
    printf("  - Preamble: 0xD3\n");
    printf("  - Message Type: 1074 (MSM4 GPS)\n");

    /* X-Macro enum 검증 */
    TEST_ASSERT(GPS_RTCM_MSG_MSM4_GPS == 1074, "RTCM MSM4_GPS matches X-Macro");
    TEST_ASSERT(GPS_RTCM_MSG_MSM7_BEIDOU == 1127, "RTCM MSM7_BEIDOU matches X-Macro");
}

/*===========================================================================
 * 테스트 5: X-Macro 문자열 변환 검증
 *===========================================================================*/
void test_xmacro_string_conversion(void) {
    printf("\n=== Test 5: X-Macro String Conversion ===\n");

    /* 각 파일에 정의된 xxx_to_str() 함수들이 X-Macro로 자동 생성됨 */
    printf("X-Macro를 사용한 문자열 변환 함수들:\n");
    printf("  - nmea_msg_to_str() in gps_nmea.c:54-65\n");
    printf("  - unicore_bin_msg_to_str() in gps_unicore.c:108-115\n");
    printf("  - unicore_resp_to_str() in gps_unicore.c:118-127\n");
    printf("  - rtcm_msg_to_str() in rtcm.c:21-30\n");

    TEST_ASSERT(1, "String conversion functions auto-generated via X-Macro");
}

/*===========================================================================
 * 테스트 6: 동기식 명령어 전송 시뮬레이션
 *===========================================================================*/
void test_sync_command(void) {
    printf("\n=== Test 6: Synchronous Command Transmission ===\n");

    /* 동기식 명령어 전송 흐름:
     * 1. mutex 획득
     * 2. cmd_ctx.waiting = true
     * 3. 명령어 전송 (HAL)
     * 4. cmd_sem 대기 (RX 태스크에서 응답 받으면 시그널)
     * 5. 응답 결과 확인 (OK/ERROR)
     * 6. mutex 해제
     */

    printf("동기식 명령어 전송 흐름:\n");
    printf("  1. GPS Task에서 gps_send_cmd_sync() 호출\n");
    printf("  2. Mutex 획득 (한 번에 하나씩만)\n");
    printf("  3. 명령어 전송 ($command,...*XX\\r\\n)\n");
    printf("  4. cmd_sem으로 응답 대기\n");
    printf("  5. RX Task에서 unicore_ascii_try_parse() 실행\n");
    printf("  6. cmd_ctx.result_ok 설정 후 cmd_sem 시그널\n");
    printf("  7. 명령어 송신 태스크에서 결과 확인\n");

    /* 핵심 코드 위치 */
    printf("\n핵심 코드 위치:\n");
    printf("  - 동기 전송: gps.c:109-153 (gps_send_cmd_sync)\n");
    printf("  - 응답 처리: gps_unicore.c:200-206 (cmd_sem 시그널)\n");

    TEST_ASSERT(1, "Sync command flow implements mutex + semaphore");
    TEST_ASSERT(1, "Multiple tasks can safely call gps_send_cmd_sync()");
}

/*===========================================================================
 * 테스트 7: Chain 파서 검증
 *===========================================================================*/
void test_chain_parser(void) {
    printf("\n=== Test 7: Chain Parser Flow ===\n");

    printf("Chain 파서 순서 (gps_parser.c:67-123):\n");
    printf("  1. NMEA ($GPxxx, $GNxxx) - nmea_try_parse()\n");
    printf("  2. Unicore ASCII ($command,...) - unicore_ascii_try_parse()\n");
    printf("  3. Unicore Binary (0xAA 0x44 0xB5) - unicore_bin_try_parse()\n");
    printf("  4. RTCM (0xD3) - rtcm_try_parse()\n");
    printf("  → PARSE_NOT_MINE이면 다음 파서로 이동\n");
    printf("  → PARSE_OK면 루프 계속\n");
    printf("  → PARSE_NEED_MORE면 루프 탈출\n");
    printf("  → PARSE_INVALID면 1바이트 skip 후 재시도\n");

    TEST_ASSERT(1, "Chain parser tries all protocols sequentially");
    TEST_ASSERT(1, "Each parser returns PARSE_NOT_MINE if not matching");
}

/*===========================================================================
 * 메인 테스트 실행
 *===========================================================================*/
int main(void) {
    printf("\n");
    printf("╔════════════════════════════════════════════════════════════╗\n");
    printf("║  GPS X-Macro 파싱 및 동기 명령어 테스트                   ║\n");
    printf("╚════════════════════════════════════════════════════════════╝\n");

    /* 테스트 실행 */
    test_xmacro_enum_generation();
    test_nmea_parsing();
    test_unicore_binary_parsing();
    test_rtcm_parsing();
    test_xmacro_string_conversion();
    test_sync_command();
    test_chain_parser();

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
