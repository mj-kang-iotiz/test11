/**
 * @file test_ble_parser.c
 * @brief BLE 파서 테스트 (NLE521 AT 응답)
 *
 * PC에서 실행: gcc -I../../lib/ble test_ble_parser.c ../../lib/ble/ble_parser.c -o test_ble_parser && ./test_ble_parser
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "ble_parser.h"

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

/**
 * @brief 문자열을 파서에 입력하고 결과 반환
 */
static ble_parse_result_t parse_string(ble_parser_ctx_t *ctx, const char *str)
{
    ble_parse_result_t result = BLE_PARSE_RESULT_NONE;

    for (size_t i = 0; str[i] != '\0'; i++) {
        ble_parse_result_t r = ble_parser_process_byte(ctx, (uint8_t)str[i]);
        if (r != BLE_PARSE_RESULT_NONE) {
            result = r;
        }
    }

    return result;
}

/*===========================================================================
 * 테스트 케이스
 *===========================================================================*/

/**
 * @brief AT 응답 파싱 테스트
 */
static void test_at_responses(void)
{
    printf("\n=== AT 응답 파싱 테스트 ===\n");

    ble_parser_ctx_t ctx;

    /* +OK 응답 */
    ble_parser_init(&ctx);
    TEST_ASSERT(parse_string(&ctx, "+OK\r") == BLE_PARSE_RESULT_AT_OK,
                "+OK 응답 파싱");

    /* +OK=value 응답 */
    ble_parser_init(&ctx);
    TEST_ASSERT(parse_string(&ctx, "+OK=115200\r") == BLE_PARSE_RESULT_AT_OK,
                "+OK=value 응답 파싱");

    /* +ERROR 응답 */
    ble_parser_init(&ctx);
    TEST_ASSERT(parse_string(&ctx, "+ERROR\r") == BLE_PARSE_RESULT_AT_ERROR,
                "+ERROR 응답 파싱");

    /* +ERR 응답 (NLE521 사용) */
    ble_parser_init(&ctx);
    TEST_ASSERT(parse_string(&ctx, "+ERR\r") == BLE_PARSE_RESULT_AT_ERROR,
                "+ERR 응답 파싱");

    /* +ERR:01 응답 */
    ble_parser_init(&ctx);
    TEST_ASSERT(parse_string(&ctx, "+ERR:01\r") == BLE_PARSE_RESULT_AT_ERROR,
                "+ERR:01 에러 코드 파싱");

    /* +READY 응답 */
    ble_parser_init(&ctx);
    TEST_ASSERT(parse_string(&ctx, "+READY\r") == BLE_PARSE_RESULT_AT_READY,
                "+READY 응답 파싱");

    /* +CONNECTED 응답 */
    ble_parser_init(&ctx);
    TEST_ASSERT(parse_string(&ctx, "+CONNECTED\r") == BLE_PARSE_RESULT_AT_CONNECTED,
                "+CONNECTED 응답 파싱");

    /* +DISCONNECTED 응답 */
    ble_parser_init(&ctx);
    TEST_ASSERT(parse_string(&ctx, "+DISCONNECTED\r") == BLE_PARSE_RESULT_AT_DISCONNECTED,
                "+DISCONNECTED 응답 파싱");

    /* +ADVERTISING 응답 */
    ble_parser_init(&ctx);
    TEST_ASSERT(parse_string(&ctx, "+ADVERTISING\r") == BLE_PARSE_RESULT_AT_ADVERTISING,
                "+ADVERTISING 응답 파싱");

    /* +MANUF:DeviceName 응답 */
    ble_parser_init(&ctx);
    ble_parse_result_t r = parse_string(&ctx, "+MANUF:MyDevice\r");
    TEST_ASSERT(r == BLE_PARSE_RESULT_AT_OTHER, "+MANUF:xxx 기타 응답 파싱");
    TEST_ASSERT(strcmp(ble_parser_get_line(&ctx), "+MANUF:MyDevice") == 0,
                "+MANUF:xxx 라인 내용 확인");

    /* +UART:115200 응답 */
    ble_parser_init(&ctx);
    r = parse_string(&ctx, "+UART:115200\r");
    TEST_ASSERT(r == BLE_PARSE_RESULT_AT_OTHER, "+UART:xxx 기타 응답 파싱");
}

/**
 * @brief 앱 명령어 파싱 테스트 (Bypass 모드)
 */
static void test_app_commands(void)
{
    printf("\n=== 앱 명령어 파싱 테스트 ===\n");

    ble_parser_ctx_t ctx;

    /* SD+ 명령어 */
    ble_parser_init(&ctx);
    ble_parse_result_t r = parse_string(&ctx, "SD+GET_STATUS\r");
    TEST_ASSERT(r == BLE_PARSE_RESULT_APP_CMD, "SD+ 명령어 파싱");
    TEST_ASSERT(strcmp(ble_parser_get_line(&ctx), "SD+GET_STATUS") == 0,
                "SD+ 명령어 내용 확인");

    /* SC+ 명령어 */
    ble_parser_init(&ctx);
    r = parse_string(&ctx, "SC+SET_MODE=1\r");
    TEST_ASSERT(r == BLE_PARSE_RESULT_APP_CMD, "SC+ 명령어 파싱");

    /* SM+ 명령어 */
    ble_parser_init(&ctx);
    r = parse_string(&ctx, "SM+VERSION\r");
    TEST_ASSERT(r == BLE_PARSE_RESULT_APP_CMD, "SM+ 명령어 파싱");

    /* 일반 문자열 */
    ble_parser_init(&ctx);
    r = parse_string(&ctx, "HELLO\r");
    TEST_ASSERT(r == BLE_PARSE_RESULT_APP_CMD, "일반 문자열 파싱");
}

/**
 * @brief 연속 응답 파싱 테스트
 */
static void test_sequential_responses(void)
{
    printf("\n=== 연속 응답 파싱 테스트 ===\n");

    ble_parser_ctx_t ctx;
    ble_parser_init(&ctx);

    /* 여러 응답 연속 수신 시뮬레이션 */
    const char *responses = "+READY\r\n+OK\r\n+CONNECTED\r\n";
    ble_parse_result_t results[10];
    int result_count = 0;

    for (size_t i = 0; responses[i] != '\0'; i++) {
        ble_parse_result_t r = ble_parser_process_byte(&ctx, (uint8_t)responses[i]);
        if (r != BLE_PARSE_RESULT_NONE) {
            results[result_count++] = r;
        }
    }

    TEST_ASSERT(result_count == 3, "3개 응답 파싱됨");
    TEST_ASSERT(results[0] == BLE_PARSE_RESULT_AT_READY, "첫번째: +READY");
    TEST_ASSERT(results[1] == BLE_PARSE_RESULT_AT_OK, "두번째: +OK");
    TEST_ASSERT(results[2] == BLE_PARSE_RESULT_AT_CONNECTED, "세번째: +CONNECTED");
}

/**
 * @brief CR/LF 처리 테스트
 */
static void test_line_endings(void)
{
    printf("\n=== 라인 종료 문자 테스트 ===\n");

    ble_parser_ctx_t ctx;

    /* CR만 */
    ble_parser_init(&ctx);
    TEST_ASSERT(parse_string(&ctx, "+OK\r") == BLE_PARSE_RESULT_AT_OK,
                "CR만 있는 경우");

    /* LF만 */
    ble_parser_init(&ctx);
    TEST_ASSERT(parse_string(&ctx, "+OK\n") == BLE_PARSE_RESULT_AT_OK,
                "LF만 있는 경우");

    /* CR+LF */
    ble_parser_init(&ctx);
    TEST_ASSERT(parse_string(&ctx, "+OK\r\n") == BLE_PARSE_RESULT_AT_OK,
                "CR+LF 있는 경우");

    /* 빈 라인 무시 */
    ble_parser_init(&ctx);
    ble_parse_result_t r = parse_string(&ctx, "\r\n\r\n+OK\r\n");
    TEST_ASSERT(r == BLE_PARSE_RESULT_AT_OK, "빈 라인 무시하고 파싱");
}

/**
 * @brief AT 파라미터 추출 테스트
 */
static void test_at_param_extraction(void)
{
    printf("\n=== AT 파라미터 추출 테스트 ===\n");

    const char *param;

    /* 콜론 구분자 */
    param = ble_parser_get_at_param("+MANUF:MyDevice");
    TEST_ASSERT(param != NULL && strcmp(param, "MyDevice") == 0,
                "+MANUF:MyDevice 파라미터 추출");

    /* 등호 구분자 */
    param = ble_parser_get_at_param("+UART=115200");
    TEST_ASSERT(param != NULL && strcmp(param, "115200") == 0,
                "+UART=115200 파라미터 추출");

    /* 파라미터 없음 */
    param = ble_parser_get_at_param("+OK");
    TEST_ASSERT(param == NULL, "+OK 파라미터 없음");
}

/**
 * @brief 버퍼 오버플로우 테스트
 */
static void test_buffer_overflow(void)
{
    printf("\n=== 버퍼 오버플로우 테스트 ===\n");

    ble_parser_ctx_t ctx;
    ble_parser_init(&ctx);

    /* BLE_PARSER_BUF_SIZE(256) 초과 데이터 - 중간에 오버플로우 감지 확인 */
    char long_str[300];
    memset(long_str, 'A', sizeof(long_str));

    bool overflow_detected = false;
    for (int i = 0; i < 280; i++) {
        ble_parse_result_t r = ble_parser_process_byte(&ctx, long_str[i]);
        if (r == BLE_PARSE_RESULT_OVERFLOW) {
            overflow_detected = true;
            break;
        }
    }

    TEST_ASSERT(overflow_detected, "버퍼 오버플로우 감지");
}

/*===========================================================================
 * 메인
 *===========================================================================*/

int main(void)
{
    printf("========================================\n");
    printf(" BLE Parser Test (NLE521)\n");
    printf("========================================\n");

    test_at_responses();
    test_app_commands();
    test_sequential_responses();
    test_line_endings();
    test_at_param_extraction();
    test_buffer_overflow();

    printf("\n========================================\n");
    printf(" 결과: %d passed, %d failed\n", test_passed, test_failed);
    printf("========================================\n");

    return test_failed > 0 ? 1 : 0;
}
