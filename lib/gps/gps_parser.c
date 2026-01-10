/**
 * @file gps_parser.c
 * @brief GPS 메인 파서 (Chain 방식)
 *
 * 각 프로토콜 파서를 순차적으로 호출하여 패킷 파싱
 * NMEA -> Unicore ASCII -> Unicore Binary -> RTCM
 */

#include "gps_parser.h"
#include "gps.h"
#include "gps_proto_def.h"
#include <string.h>

#ifndef TAG
#define TAG "GPS_PARSER"
#endif

#include "log.h"

/*===========================================================================
 * 유틸리티 함수 구현
 *===========================================================================*/

bool ringbuffer_find_char(ringbuffer_t *rb, char ch, size_t max_search, size_t *pos) {
    size_t available = ringbuffer_size(rb);
    size_t search_len = (max_search < available) ? max_search : available;

    for (size_t i = 0; i < search_len; i++) {
        char c;
        if (!ringbuffer_peek(rb, &c, 1, i)) {
            return false;
        }
        if (c == ch) {
            *pos = i;
            return true;
        }
    }
    return false;
}

uint8_t hex_char_to_num(char ch) {
    if (ch >= '0' && ch <= '9') return ch - '0';
    if (ch >= 'A' && ch <= 'F') return ch - 'A' + 10;
    if (ch >= 'a' && ch <= 'f') return ch - 'a' + 10;
    return 0;
}

uint8_t hex_to_byte(const char *hex) {
    return (hex_char_to_num(hex[0]) << 4) | hex_char_to_num(hex[1]);
}

/*===========================================================================
 * 파서 초기화
 *===========================================================================*/

void gps_parser_init(gps_t *gps) {
    if (!gps) return;

    memset(&gps->parser_ctx, 0, sizeof(gps_parser_ctx_t));
    LOG_INFO("GPS parser initialized");
}

/*===========================================================================
 * 메인 파서 루프 (Chain 방식)
 *===========================================================================*/

parse_result_t gps_parser_process(gps_t *gps) {
    if (!gps) return PARSE_INVALID;

    ringbuffer_t *rb = &gps->rx_buf;
    parse_result_t result = PARSE_NEED_MORE;

    while (ringbuffer_size(rb) > 0) {
        /*
         * Chain 방식: 각 파서가 "내 패킷인지" 판단
         * NOT_MINE 리턴하면 다음 파서로 넘어감
         */

        /* 1. NMEA 시도 ($GPxxx, $GNxxx 등) */
        result = nmea_try_parse(gps, rb);

        /* 2. NMEA가 아니면 Unicore ASCII 시도 ($command,response:...) */
        if (result == PARSE_NOT_MINE) {
            result = unicore_ascii_try_parse(gps, rb);
        }

        /* 3. Unicore ASCII도 아니면 Unicore Binary 시도 (0xAA 0x44 0xB5) */
        if (result == PARSE_NOT_MINE) {
            result = unicore_bin_try_parse(gps, rb);
        }

        /* 4. RTCM 시도 (0xD3) */
        if (result == PARSE_NOT_MINE) {
            result = rtcm_try_parse(gps, rb);
        }

        /* 결과 처리 */
        switch (result) {
            case PARSE_OK:
                /* 파싱 성공, advance는 각 파서에서 완료 */
                gps->parser_ctx.stats.rx_packets++;
                continue;  /* 다음 패킷 처리 */

            case PARSE_NEED_MORE:
                /* 데이터 부족, 루프 탈출 */
                return result;

            case PARSE_INVALID:
                /* CRC 오류 등, 1바이트 skip 후 재시도 */
                gps->parser_ctx.stats.invalid_packets++;
                ringbuffer_advance(rb, 1);
                continue;

            case PARSE_NOT_MINE:
                /* 모든 파서가 거부, 알 수 없는 바이트 skip */
                gps->parser_ctx.stats.unknown_packets++;
                ringbuffer_advance(rb, 1);
                continue;
        }
    }

    return PARSE_NEED_MORE;
}

/*===========================================================================
 * 파서 통계 API
 *===========================================================================*/

const gps_parser_stats_t* gps_parser_get_stats(gps_t *gps) {
    if (!gps) return NULL;
    return &gps->parser_ctx.stats;
}

void gps_parser_reset_stats(gps_t *gps) {
    if (!gps) return;
    memset(&gps->parser_ctx.stats, 0, sizeof(gps_parser_stats_t));
}
