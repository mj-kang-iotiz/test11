/**
 * @file test_unicore_ascii_parser.c
 * @brief Unicore ASCII 파서 테스트 ($command,response:OK)
 *
 * PC에서 실행:
 *   gcc test_unicore_ascii_parser.c -o test_unicore_ascii_parser && ./test_unicore_ascii_parser
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/*===========================================================================
 * FreeRTOS 모킹
 *===========================================================================*/
#define pdTRUE 1
#define pdFALSE 0

typedef int BaseType_t;
typedef void* TaskHandle_t;
typedef void* SemaphoreHandle_t;
typedef void* QueueHandle_t;

static uint32_t mock_tick_count = 1000;
static bool mock_semaphore_given = false;

uint32_t xTaskGetTickCount(void) {
    return mock_tick_count;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t sem) {
    (void)sem;
    mock_semaphore_given = true;
    return pdTRUE;
}

/* LOG 모킹 */
#define LOG_INFO(fmt, ...) printf("[INFO] " fmt "\n", ##__VA_ARGS__)

/*===========================================================================
 * Ringbuffer 구현 (테스트용 최소 구현)
 *===========================================================================*/
typedef struct {
    char *buffer;
    volatile size_t head;
    volatile size_t tail;
    size_t size;
} ringbuffer_t;

static void ringbuffer_init(ringbuffer_t *rb, char *buffer, size_t size) {
    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
}

static size_t ringbuffer_size(ringbuffer_t *rb) {
    if (rb->head >= rb->tail) {
        return rb->head - rb->tail;
    }
    return rb->size + rb->head - rb->tail;
}

static void ringbuffer_write(ringbuffer_t *rb, const char *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        rb->buffer[rb->head] = data[i];
        rb->head = (rb->head + 1) % rb->size;
    }
}

static bool ringbuffer_peek(ringbuffer_t *rb, char *data, size_t len, size_t offset) {
    size_t current_size = ringbuffer_size(rb);
    if (offset >= current_size || len > (current_size - offset)) {
        return false;
    }

    for (size_t i = 0; i < len; i++) {
        size_t idx = (rb->tail + offset + i) % rb->size;
        data[i] = rb->buffer[idx];
    }
    return true;
}

static bool ringbuffer_advance(ringbuffer_t *rb, size_t len) {
    rb->tail = (rb->tail + len) % rb->size;
    return true;
}

static bool ringbuffer_find_char(ringbuffer_t *rb, char ch, size_t max_search, size_t *pos) {
    size_t avail = ringbuffer_size(rb);
    size_t search_len = (avail < max_search) ? avail : max_search;

    for (size_t i = 0; i < search_len; i++) {
        char c;
        if (ringbuffer_peek(rb, &c, 1, i)) {
            if (c == ch) {
                *pos = i;
                return true;
            }
        }
    }
    return false;
}

/*===========================================================================
 * GPS 타입 최소 정의 (테스트용)
 *===========================================================================*/

#define GPS_UNICORE_ASCII_MAX   128

typedef enum {
    GPS_UNICORE_RESP_NONE = 0,
    GPS_UNICORE_RESP_OK,
    GPS_UNICORE_RESP_ERROR,
    GPS_UNICORE_RESP_UNKNOWN
} gps_unicore_resp_t;

typedef enum {
    GPS_EVENT_CMD_RESPONSE,
} gps_event_type_t;

typedef enum {
    GPS_PROTOCOL_UNICORE_CMD,
} gps_protocol_t;

typedef struct {
    gps_event_type_t type;
    gps_protocol_t protocol;
    uint32_t timestamp_ms;
    union {
        struct {
            bool success;
        } cmd_response;
    } data;
} gps_event_t;

typedef struct {
    bool waiting;
    bool result_ok;
} gps_cmd_ctx_t;

typedef struct {
    uint32_t crc_errors;
    uint32_t unicore_cmd_packets;
} gps_parser_stats_t;

typedef struct {
    gps_cmd_ctx_t cmd_ctx;
    gps_parser_stats_t stats;
} gps_parser_ctx_t;

typedef enum {
    PARSE_NOT_MINE = 0,
    PARSE_NEED_MORE,
    PARSE_OK,
    PARSE_INVALID,
} parse_result_t;

struct gps_s;
typedef struct gps_s gps_t;
typedef void (*gps_evt_handler)(gps_t *gps, const gps_event_t *event);

struct gps_s {
    gps_parser_ctx_t parser_ctx;
    SemaphoreHandle_t cmd_sem;
    gps_evt_handler handler;
    ringbuffer_t rx_buf;
    char rx_buf_mem[2048];
};

/* 이벤트 핸들러 결과 저장용 */
static bool last_event_success = false;
static bool event_handler_called = false;

static void test_event_handler(gps_t *gps, const gps_event_t *event) {
    (void)gps;
    event_handler_called = true;
    if (event->type == GPS_EVENT_CMD_RESPONSE) {
        last_event_success = event->data.cmd_response.success;
    }
}

/*===========================================================================
 * 유틸리티 함수
 *===========================================================================*/

static uint8_t hex_char_to_num(char ch) {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return 0;
}

static uint8_t hex_to_byte(const char *hex) {
    return (hex_char_to_num(hex[0]) << 4) | hex_char_to_num(hex[1]);
}

/* CRC 계산 헬퍼 (콜론까지 포함) */
static uint8_t calc_unicore_crc(const char *cmd) {
    uint8_t crc = 0;
    /* $ 다음부터 계산 */
    const char *p = cmd + 1;
    while (*p && *p != '*') {
        crc ^= (uint8_t)*p;
        /* 콜론까지만 포함 */
        if (*p == ':') break;
        p++;
    }
    return crc;
}

/* CRC 포함 문자열 생성 */
static char test_buf[256];
static const char* make_cmd_response(const char *cmd, const char *resp) {
    /* $command,<cmd>,response: <resp>*XX 형식 생성 */
    snprintf(test_buf, sizeof(test_buf), "$command,%s,response: %s", cmd, resp);
    /* CRC 계산 (콜론까지) */
    uint8_t crc = calc_unicore_crc(test_buf);
    size_t len = strlen(test_buf);
    snprintf(test_buf + len, sizeof(test_buf) - len, "*%02X\r\n", crc);
    return test_buf;
}

/*===========================================================================
 * Unicore ASCII 파서 (테스트 대상 - 수정된 코드)
 *===========================================================================*/

static parse_result_t unicore_ascii_try_parse(gps_t *gps, ringbuffer_t *rb) {
    /* 1. 첫 바이트 확인 - '$' 아니면 NOT_MINE */
    char first;
    if (!ringbuffer_peek(rb, &first, 1, 0)) {
        return PARSE_NEED_MORE;
    }
    if (first != '$') {
        return PARSE_NOT_MINE;
    }

    /* 2. "$command," 패턴 확인 (9바이트) */
    char prefix[10];
    if (!ringbuffer_peek(rb, prefix, 9, 0)) {
        return PARSE_NEED_MORE;
    }
    prefix[9] = '\0';

    if (strncmp(prefix + 1, "command,", 8) != 0) {
        return PARSE_NOT_MINE;  /* NMEA일 수 있음 */
    }

    /* 3. '\r' 찾기 (패킷 끝) */
    size_t cr_pos;
    if (!ringbuffer_find_char(rb, '\r', GPS_UNICORE_ASCII_MAX, &cr_pos)) {
        if (ringbuffer_size(rb) >= GPS_UNICORE_ASCII_MAX) {
            return PARSE_INVALID;
        }
        return PARSE_NEED_MORE;
    }

    /* 4. 전체 패킷 peek */
    size_t pkt_len = cr_pos + 1;
    char next_char;
    if (ringbuffer_peek(rb, &next_char, 1, cr_pos + 1) && next_char == '\n') {
        pkt_len++;
    }

    char buf[GPS_UNICORE_ASCII_MAX + 1];
    if (!ringbuffer_peek(rb, buf, cr_pos, 0)) {
        return PARSE_NEED_MORE;
    }
    buf[cr_pos] = '\0';

    /* 5. CRC 검증 */
    const char *star = memchr(buf, '*', cr_pos);
    if (!star || (size_t)(star - buf + 3) > cr_pos) {
        gps->parser_ctx.stats.crc_errors++;
        ringbuffer_advance(rb, pkt_len);
        return PARSE_INVALID;
    }

    /* ':' 위치 찾기 */
    const char *colon = memchr(buf, ':', cr_pos);
    /* CRC 계산 끝점: 콜론 포함 (colon + 1), 없으면 * 전까지 */
    const char *crc_end = colon ? (colon + 1) : star;

    /* CRC 계산 ($ 다음부터 : 포함까지) */
    uint8_t calc_crc = 0;
    for (const char *p = buf + 1; p < crc_end; p++) {
        calc_crc ^= (uint8_t)*p;
    }

    uint8_t recv_crc = hex_to_byte(star + 1);
    if (calc_crc != recv_crc) {
        gps->parser_ctx.stats.crc_errors++;
        ringbuffer_advance(rb, pkt_len);
        return PARSE_INVALID;
    }

    /* 6. Response 파싱 ("response:OK" 또는 "response: OK") */
    gps_unicore_resp_t resp = GPS_UNICORE_RESP_UNKNOWN;
    const char *resp_str = strstr(buf, "response:");
    if (resp_str) {
        resp_str += 9;  /* "response:" 길이 */

        /* 콜론 뒤 공백 건너뛰기 (UM982는 "response: OK" 형식) */
        while (*resp_str == ' ' || *resp_str == '\t') {
            resp_str++;
        }

        /* 응답 전체 메시지 로그 출력 */
        LOG_INFO("UM982 <- %s", resp_str);

        if (strncmp(resp_str, "OK", 2) == 0) {
            resp = GPS_UNICORE_RESP_OK;
        } else if (strncmp(resp_str, "ERROR", 5) == 0) {
            resp = GPS_UNICORE_RESP_ERROR;
        }
    }

    /* 7. advance */
    ringbuffer_advance(rb, pkt_len);
    gps->parser_ctx.stats.unicore_cmd_packets++;

    /* 8. 명령어 응답 대기 중이면 세마포어 시그널 */
    if (gps->parser_ctx.cmd_ctx.waiting) {
        gps->parser_ctx.cmd_ctx.result_ok = (resp == GPS_UNICORE_RESP_OK);
        if (gps->cmd_sem) {
            xSemaphoreGive(gps->cmd_sem);
        }
    }

    /* 9. 명령어 응답 이벤트 핸들러 호출 */
    if (gps->handler) {
        gps_event_t event = {
            .type = GPS_EVENT_CMD_RESPONSE,
            .protocol = GPS_PROTOCOL_UNICORE_CMD,
            .timestamp_ms = xTaskGetTickCount(),
            .data.cmd_response.success = (resp == GPS_UNICORE_RESP_OK)
        };
        gps->handler(gps, &event);
    }

    return PARSE_OK;
}

/*===========================================================================
 * 테스트 유틸리티
 *===========================================================================*/

static int test_passed = 0;
static int test_failed = 0;

#define TEST_ASSERT(cond, msg) do { \
    if (cond) { \
        printf("  [PASS] %s\n", msg); \
        test_passed++; \
    } else { \
        printf("  [FAIL] %s\n", msg); \
        test_failed++; \
    } \
} while(0)

static void reset_test_state(gps_t *gps) {
    memset(gps, 0, sizeof(gps_t));
    ringbuffer_init(&gps->rx_buf, gps->rx_buf_mem, sizeof(gps->rx_buf_mem));
    gps->cmd_sem = (SemaphoreHandle_t)1;
    gps->handler = test_event_handler;
    gps->parser_ctx.cmd_ctx.waiting = true;

    mock_semaphore_given = false;
    event_handler_called = false;
    last_event_success = false;
}

static void feed_data(gps_t *gps, const char *data) {
    ringbuffer_write(&gps->rx_buf, data, strlen(data));
}

/*===========================================================================
 * 테스트 케이스
 *===========================================================================*/

static void test_response_without_space(void) {
    printf("\n=== 공백 없는 응답 파싱 테스트 ===\n");

    gps_t gps;
    reset_test_state(&gps);

    /* 공백 없는 형식도 지원 확인 - CRC 자동 계산 */
    feed_data(&gps, make_cmd_response("unmask BDS", "OK"));

    parse_result_t result = unicore_ascii_try_parse(&gps, &gps.rx_buf);

    TEST_ASSERT(result == PARSE_OK, "파싱 결과 PARSE_OK");
    TEST_ASSERT(gps.parser_ctx.cmd_ctx.result_ok == true, "result_ok == true");
    TEST_ASSERT(mock_semaphore_given == true, "세마포어 시그널됨");
    TEST_ASSERT(event_handler_called == true, "이벤트 핸들러 호출됨");
    TEST_ASSERT(last_event_success == true, "이벤트 success == true");
}

static void test_response_with_space(void) {
    printf("\n=== 공백 있는 응답 파싱 테스트 (UM982 실제 형식) ===\n");

    gps_t gps;
    reset_test_state(&gps);

    /* UM982 실제 응답 형식 - CRC 자동 계산 */
    feed_data(&gps, make_cmd_response("unmask GPS", "OK"));

    parse_result_t result = unicore_ascii_try_parse(&gps, &gps.rx_buf);

    TEST_ASSERT(result == PARSE_OK, "파싱 결과 PARSE_OK");
    TEST_ASSERT(gps.parser_ctx.cmd_ctx.result_ok == true, "result_ok == true (핵심 버그 수정)");
    TEST_ASSERT(mock_semaphore_given == true, "세마포어 시그널됨");
    TEST_ASSERT(event_handler_called == true, "이벤트 핸들러 호출됨");
    TEST_ASSERT(last_event_success == true, "이벤트 success == true");
}

static void test_response_with_multiple_spaces(void) {
    printf("\n=== 여러 공백 있는 응답 파싱 테스트 ===\n");

    gps_t gps;
    reset_test_state(&gps);

    /* 공백 2개 테스트 - CRC 수동 계산 (콜론까지) */
    /* "$command,test,response:" -> CRC 계산 */
    const char *base = "$command,test,response:";
    uint8_t crc = 0;
    for (const char *p = base + 1; *p; p++) {
        crc ^= (uint8_t)*p;
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "%s  OK*%02X\r\n", base, crc);
    feed_data(&gps, buf);

    parse_result_t result = unicore_ascii_try_parse(&gps, &gps.rx_buf);

    TEST_ASSERT(result == PARSE_OK, "파싱 결과 PARSE_OK");
    TEST_ASSERT(gps.parser_ctx.cmd_ctx.result_ok == true, "result_ok == true (다중 공백도 처리)");
}

static void test_error_response_with_space(void) {
    printf("\n=== ERROR 응답 파싱 테스트 ===\n");

    gps_t gps;
    reset_test_state(&gps);

    /* ERROR 응답 - CRC 자동 계산 */
    feed_data(&gps, make_cmd_response("bad cmd", "ERROR"));

    parse_result_t result = unicore_ascii_try_parse(&gps, &gps.rx_buf);

    TEST_ASSERT(result == PARSE_OK, "파싱 결과 PARSE_OK");
    TEST_ASSERT(gps.parser_ctx.cmd_ctx.result_ok == false, "result_ok == false (ERROR 응답)");
    TEST_ASSERT(event_handler_called == true, "이벤트 핸들러 호출됨");
    TEST_ASSERT(last_event_success == false, "이벤트 success == false");
}

static void test_invalid_crc(void) {
    printf("\n=== 잘못된 CRC 테스트 ===\n");

    gps_t gps;
    reset_test_state(&gps);

    /* 의도적으로 틀린 CRC (FF) 사용 */
    feed_data(&gps, "$command,test,response: OK*FF\r\n");

    parse_result_t result = unicore_ascii_try_parse(&gps, &gps.rx_buf);

    TEST_ASSERT(result == PARSE_INVALID, "파싱 결과 PARSE_INVALID (CRC 오류)");
    TEST_ASSERT(gps.parser_ctx.stats.crc_errors == 1, "CRC 오류 카운트 증가");
}

static void test_nmea_returns_not_mine(void) {
    printf("\n=== NMEA 패킷 NOT_MINE 테스트 ===\n");

    gps_t gps;
    reset_test_state(&gps);

    feed_data(&gps, "$GPGGA,123456.00,3723.2475,N,12158.3416,W,1,08,0.9,545.4,M,46.9,M,,*47\r\n");

    parse_result_t result = unicore_ascii_try_parse(&gps, &gps.rx_buf);

    TEST_ASSERT(result == PARSE_NOT_MINE, "NMEA 패킷은 PARSE_NOT_MINE 반환");
}

static void test_incomplete_packet(void) {
    printf("\n=== 불완전한 패킷 테스트 ===\n");

    gps_t gps;
    reset_test_state(&gps);

    /* CR/LF 없이 불완전한 패킷 */
    const char *base = "$command,test,response:";
    uint8_t crc = 0;
    for (const char *p = base + 1; *p; p++) {
        crc ^= (uint8_t)*p;
    }
    char buf[128];
    snprintf(buf, sizeof(buf), "%s OK*%02X", base, crc);  /* CR/LF 없음 */
    feed_data(&gps, buf);

    parse_result_t result = unicore_ascii_try_parse(&gps, &gps.rx_buf);

    TEST_ASSERT(result == PARSE_NEED_MORE, "불완전한 패킷은 PARSE_NEED_MORE 반환");
}

static void test_real_um982_sequence(void) {
    printf("\n=== 실제 UM982 응답 시퀀스 테스트 ===\n");

    gps_t gps;

    /* 실제 UM982 명령어들 - CRC 자동 계산 */
    const char *commands[] = {
        "unmask GPS",
        "unmask GLO",
        "unmask GAL",
        "unmask QZSS",
    };

    int success_count = 0;
    for (size_t i = 0; i < sizeof(commands)/sizeof(commands[0]); i++) {
        reset_test_state(&gps);
        feed_data(&gps, make_cmd_response(commands[i], "OK"));

        parse_result_t result = unicore_ascii_try_parse(&gps, &gps.rx_buf);

        if (result == PARSE_OK && gps.parser_ctx.cmd_ctx.result_ok) {
            success_count++;
            printf("  [PASS] 응답 %zu 파싱 성공\n", i + 1);
        } else {
            printf("  [FAIL] 응답 %zu 파싱 실패 (result=%d, result_ok=%d)\n",
                   i + 1, result, gps.parser_ctx.cmd_ctx.result_ok);
            test_failed++;
        }
    }
    test_passed += success_count;
}

/*===========================================================================
 * 메인
 *===========================================================================*/

int main(void) {
    printf("========================================\n");
    printf(" Unicore ASCII Parser Test (UM982)\n");
    printf("========================================\n");

    test_response_without_space();
    test_response_with_space();
    test_response_with_multiple_spaces();
    test_error_response_with_space();
    test_invalid_crc();
    test_nmea_returns_not_mine();
    test_incomplete_packet();
    test_real_um982_sequence();

    printf("\n========================================\n");
    printf(" 결과: %d passed, %d failed\n", test_passed, test_failed);
    printf("========================================\n");

    return test_failed > 0 ? 1 : 0;
}
