/**
 * @file gps_nmea.c
 * @brief NMEA 183 프로토콜 파서
 */

#include "gps_nmea.h"
#include "gps.h"
#include "gps_parser.h"
#include "gps_proto_def.h"
#include "dev_assert.h"
#include <string.h>
#include <stdlib.h>

#ifndef TAG
#define TAG "GPS_NMEA"
#endif

#include "log.h"

/*===========================================================================
 * 내부 함수 선언
 *===========================================================================*/
static double parse_lat_lon_str(const char *str);
static void nmea_parse_gga(gps_t *gps, const char *buf, size_t len);
static void nmea_parse_rmc(gps_t *gps, const char *buf, size_t len);
static void nmea_parse_ths(gps_t *gps, const char *buf, size_t len);
static bool nmea_verify_crc(const char *buf, size_t len, size_t *star_pos);
static size_t nmea_count_fields(const char *buf, size_t len);

/*===========================================================================
 * X-Macro 기반 핸들러 테이블
 *===========================================================================*/
typedef void (*nmea_handler_t)(gps_t *gps, const char *buf, size_t len);

static const struct {
    gps_nmea_msg_t msg_id;  /* enum 값 명시적으로 저장 */
    const char *str;
    nmea_handler_t handler;
    uint8_t field_count;
    bool is_urc;
} nmea_msg_table[] = {
#define X(name, str, handler, field_count, is_urc) \
    { GPS_NMEA_MSG_##name, str, handler, field_count, is_urc },
    NMEA_MSG_TABLE(X)
#undef X
};

#define NMEA_MSG_TABLE_SIZE (sizeof(nmea_msg_table) / sizeof(nmea_msg_table[0]))

/*===========================================================================
 * X-Macro 기반 문자열 변환 함수
 *===========================================================================*/
static const char* nmea_msg_to_str(gps_nmea_msg_t msg_id) {
    if (msg_id == GPS_NMEA_MSG_NONE) return "NONE";
    if (msg_id == GPS_NMEA_MSG_INVALID) return "INVALID";

    switch (msg_id) {
#define X(name, str, handler, field_count, is_urc) \
        case GPS_NMEA_MSG_##name: return str;
        NMEA_MSG_TABLE(X)
#undef X
        default: return "UNKNOWN";
    }
}

/*===========================================================================
 * NMEA 패킷 파싱 (Chain 방식)
 *===========================================================================*/

parse_result_t nmea_try_parse(gps_t *gps, ringbuffer_t *rb) {
    /* 1. 첫 바이트 확인 - '$' 아니면 NOT_MINE */
    char first;
    if (!ringbuffer_peek(rb, &first, 1, 0)) {
        return PARSE_NEED_MORE;
    }
    if (first != '$') {
        return PARSE_NOT_MINE;
    }

    /* 2. 메시지 타입 확인 (6바이트: $GPGGA 형태) */
    char prefix[7];
    if (!ringbuffer_peek(rb, prefix, 6, 0)) {
        return PARSE_NEED_MORE;
    }
    prefix[6] = '\0';

    /* 3. NMEA 메시지인지 확인 (GP, GN, GL, GA, GB로 시작) */
    const char *talker = &prefix[1];  /* $ 다음 */
    if (!(talker[0] == 'G' && (talker[1] == 'P' || talker[1] == 'N' ||
                               talker[1] == 'L' || talker[1] == 'A' ||
                               talker[1] == 'B'))) {
        return PARSE_NOT_MINE;  /* Unicore ASCII일 수 있음 ($command,...) */
    }

    /* 4. 메시지 타입 확인 (GGA, RMC, THS 등) */
    const char *msg_type = &prefix[3];  /* 탈커 ID 다음 */
    gps_nmea_msg_t msg_id = GPS_NMEA_MSG_NONE;
    int msg_idx = -1;

    for (size_t i = 0; i < NMEA_MSG_TABLE_SIZE; i++) {
        DEV_ASSERT(nmea_msg_table[i].str != NULL);
        if (strncmp(msg_type, nmea_msg_table[i].str, 3) == 0) {
            msg_id = nmea_msg_table[i].msg_id;  /* 테이블에서 직접 읽기 */
            msg_idx = i;
            break;
        }
    }

    if (msg_id == GPS_NMEA_MSG_NONE) {
        return PARSE_NOT_MINE;  /* 알 수 없는 NMEA 메시지 */
    }

    /* 테이블 인덱스와 enum 값이 일치하는지 검증 */
    DEV_ASSERT(msg_idx >= 0 && msg_idx < (int)NMEA_MSG_TABLE_SIZE);
    DEV_ASSERT(nmea_msg_table[msg_idx].msg_id == msg_id);

    /* 5. '\r' 찾기 (패킷 끝) */
    size_t cr_pos;
    if (!ringbuffer_find_char(rb, '\r', GPS_NMEA_MAX_LEN, &cr_pos)) {
        /* '\r' 없음 - 최대 길이 초과하면 INVALID */
        if (ringbuffer_size(rb) >= GPS_NMEA_MAX_LEN) {
            LOG_WARN("NMEA packet too long without \\r, dropping");
            return PARSE_INVALID;
        }
        /* 데이터 부족, 더 기다림 */
        return PARSE_NEED_MORE;
    }

    /* 6. 전체 패킷 peek */
    size_t pkt_len = cr_pos + 1;  /* '\r' 포함 */

    /* '\n'도 있으면 포함 */
    char next_char;
    if (ringbuffer_peek(rb, &next_char, 1, cr_pos + 1) && next_char == '\n') {
        pkt_len++;
    }

    char buf[GPS_NMEA_MAX_LEN + 1];
    if (!ringbuffer_peek(rb, buf, cr_pos, 0)) {
        return PARSE_NEED_MORE;
    }
    buf[cr_pos] = '\0';

    /* 7. Field count 검증 */
    size_t field_count = nmea_count_fields(buf, cr_pos);
    DEV_ASSERT(nmea_msg_table[msg_idx].field_count > 0);
    if (field_count < nmea_msg_table[msg_idx].field_count) {
        /* 필드 수 부족 - 잘못된 패킷 */
        LOG_WARN("NMEA %s field count mismatch: got %zu, expected %d",
                 nmea_msg_to_str(msg_id), field_count,
                 nmea_msg_table[msg_idx].field_count);
        ringbuffer_advance(rb, pkt_len);
        return PARSE_INVALID;
    }

    /* 8. CRC 검증 */
    size_t star_pos;
    if (!nmea_verify_crc(buf, cr_pos, &star_pos)) {
        gps->parser_ctx.stats.crc_errors++;
        ringbuffer_advance(rb, pkt_len);
        return PARSE_INVALID;
    }

    /* 9. 데이터 파싱 */
    if (nmea_msg_table[msg_idx].handler) {
        nmea_msg_table[msg_idx].handler(gps, buf, star_pos);
    }

    /* 10. advance 및 이벤트 */
    ringbuffer_advance(rb, pkt_len);
    gps->parser_ctx.stats.nmea_packets++;
    gps->parser_ctx.stats.last_nmea_tick = xTaskGetTickCount();

    /* URC이면 고수준 이벤트 핸들러 호출 */
    if (nmea_msg_table[msg_idx].is_urc && gps->handler) {
        gps_event_t event = {
            .protocol = GPS_PROTOCOL_NMEA,
            .timestamp_ms = xTaskGetTickCount(),
            .source.nmea_msg_id = msg_id
        };

        /* 메시지 타입별로 적절한 이벤트 생성 */
        if (msg_id == GPS_NMEA_MSG_GGA) {
            /* Fix 상태가 변경된 경우에만 이벤트 발생 */
            if (gps->data.status.fix_changed) {
                event.type = GPS_EVENT_FIX_UPDATED;
                event.data.fix.fix_type = gps->data.status.fix_type;
                event.data.fix.hdop = gps->data.status.hdop;
                gps->handler(gps, &event);
            }
        } else if (msg_id == GPS_NMEA_MSG_THS) {
            /* 헤딩 업데이트 이벤트 */
            event.type = GPS_EVENT_HEADING_UPDATED;
            event.data.heading.heading = gps->nmea_data.ths.heading;
            event.data.heading.pitch = 0.0;  /* THS에는 pitch 없음 */
            event.data.heading.heading_std = 0.0f;
            event.data.heading.status = gps->nmea_data.ths.mode;
            gps->handler(gps, &event);
        } else if (msg_id == GPS_NMEA_MSG_RMC) {
            /* 속도 업데이트 이벤트 (RMC 구현 시) */
            event.type = GPS_EVENT_VELOCITY_UPDATED;
            /* TODO: RMC 파싱 후 속도 데이터 추가 */
            gps->handler(gps, &event);
        }
    }

#if defined(USE_STORE_RAW_GGA)
    /* GGA raw 데이터 저장 */
    if (msg_id == GPS_NMEA_MSG_GGA && cr_pos < sizeof(gps->nmea_data.gga_raw) - 2) {
        memcpy(gps->nmea_data.gga_raw, buf, cr_pos);
        gps->nmea_data.gga_raw[cr_pos] = '\r';
        gps->nmea_data.gga_raw[cr_pos + 1] = '\n';
        gps->nmea_data.gga_raw[cr_pos + 2] = '\0';
        gps->nmea_data.gga_raw_pos = cr_pos + 2;
        gps->nmea_data.gga_is_rdy = true;
    }
#endif

    return PARSE_OK;
}

/*===========================================================================
 * 내부 함수 구현
 *===========================================================================*/

/**
 * @brief NMEA CRC 검증
 */
static bool nmea_verify_crc(const char *buf, size_t len, size_t *star_pos) {
    /* '*' 찾기 */
    const char *star = memchr(buf, '*', len);
    if (!star || (size_t)(star - buf + 3) > len) {
        return false;  /* checksum 형식 오류 */
    }

    *star_pos = star - buf;

    /* CRC 계산 ($ 다음부터 * 전까지) */
    uint8_t calc_crc = 0;
    for (const char *p = buf + 1; p < star; p++) {
        calc_crc ^= (uint8_t)*p;
    }

    /* 수신된 CRC */
    uint8_t recv_crc = hex_to_byte(star + 1);

    return (calc_crc == recv_crc);
}

/**
 * @brief NMEA 필드 수 카운트
 */
static size_t nmea_count_fields(const char *buf, size_t len) {
    size_t count = 1;  /* 최소 1개 필드 */
    for (size_t i = 0; i < len; i++) {
        if (buf[i] == ',') count++;
        if (buf[i] == '*') break;
    }
    return count;
}

/**
 * @brief 위도/경도 파싱 (DDMM.MMMM 형식)
 */
static double parse_lat_lon_str(const char *str) {
    if (!str || *str == ',' || *str == '*') return 0.0;
    double val = atof(str);
    double deg = (double)((int)(val / 100));
    double min = val - (deg * 100);
    return deg + (min / 60.0);
}

/**
 * @brief 문자열에서 다음 필드 시작 위치 찾기
 */
static const char* get_field(const char *buf, size_t len, int field_num) {
    int current = 0;
    const char *p = buf;
    const char *end = buf + len;

    while (p < end && current < field_num) {
        if (*p == ',') current++;
        p++;
    }

    return (current == field_num) ? p : NULL;
}

/**
 * @brief 필드 값을 double로 파싱
 */
static double parse_field_double(const char *field) {
    if (!field || *field == ',' || *field == '*') return 0.0;
    return atof(field);
}

/**
 * @brief 필드 값을 int로 파싱
 */
static int parse_field_int(const char *field) {
    if (!field || *field == ',' || *field == '*') return 0;
    return atoi(field);
}

/**
 * @brief 필드 값을 char로 파싱
 */
static char parse_field_char(const char *field) {
    if (!field || *field == ',' || *field == '*') return '\0';
    return *field;
}

/*===========================================================================
 * GGA 파싱
 *===========================================================================*/
static void nmea_parse_gga(gps_t *gps, const char *buf, size_t len) {
    const char *field;

    /* Field 1: Time (HHMMSS.ss) */
    field = get_field(buf, len, 1);
    if (field && *field != ',') {
        gps->nmea_data.gga.hour = (field[0] - '0') * 10 + (field[1] - '0');
        gps->nmea_data.gga.min = (field[2] - '0') * 10 + (field[3] - '0');
        gps->nmea_data.gga.sec = (field[4] - '0') * 10 + (field[5] - '0');
    }

    /* Field 2: Latitude */
    field = get_field(buf, len, 2);
    if (field) gps->nmea_data.gga.lat = parse_lat_lon_str(field);

    /* Field 3: N/S */
    field = get_field(buf, len, 3);
    gps->nmea_data.gga.ns = parse_field_char(field);

    /* Field 4: Longitude */
    field = get_field(buf, len, 4);
    if (field) gps->nmea_data.gga.lon = parse_lat_lon_str(field);

    /* Field 5: E/W */
    field = get_field(buf, len, 5);
    gps->nmea_data.gga.ew = parse_field_char(field);

    /* Field 6: Fix quality */
    field = get_field(buf, len, 6);
    gps->nmea_data.gga.fix = (gps_fix_t)parse_field_int(field);

    /* Field 7: Number of satellites */
    field = get_field(buf, len, 7);
    gps->nmea_data.gga.sat_num = parse_field_int(field);

    /* Field 8: HDOP */
    field = get_field(buf, len, 8);
    gps->nmea_data.gga.hdop = parse_field_double(field);

    /* Field 9: Altitude */
    field = get_field(buf, len, 9);
    gps->nmea_data.gga.alt = parse_field_double(field);

    /* Field 11: Geoid separation */
    field = get_field(buf, len, 11);
    gps->nmea_data.gga.geo_sep = parse_field_double(field);

    /* === 공용 데이터 업데이트 (fix_type, hdop만) === */
    gps_fix_t new_fix = gps->nmea_data.gga.fix;

    /* fix_type이 변경된 경우에만 업데이트 */
    if (gps->data.status.fix_type != new_fix) {
        gps->data.status.fix_type = new_fix;
        gps->data.status.fix_changed = true;
        gps->data.status.fix_timestamp_ms = xTaskGetTickCount();
    } else {
        gps->data.status.fix_changed = false;
    }

    /* hdop은 항상 업데이트 */
    gps->data.status.hdop = gps->nmea_data.gga.hdop;
}

/*===========================================================================
 * RMC 파싱 (간략화 - 필요시 확장)
 *===========================================================================*/
static void nmea_parse_rmc(gps_t *gps, const char *buf, size_t len) {
    /* RMC 파싱 구현 필요시 추가 */
    (void)gps;
    (void)buf;
    (void)len;
}

/*===========================================================================
 * THS 파싱 (Heading)
 *===========================================================================*/
static void nmea_parse_ths(gps_t *gps, const char *buf, size_t len) {
    const char *field;

    /* Field 1: Heading */
    field = get_field(buf, len, 1);
    gps->nmea_data.ths.heading = parse_field_double(field);

    /* Field 2: Mode */
    field = get_field(buf, len, 2);
    gps->nmea_data.ths.mode = (gps_ths_mode_t)parse_field_char(field);

    /* === 공용 데이터 업데이트 === */
    gps->data.heading.heading = gps->nmea_data.ths.heading;
    gps->data.heading.mode = (uint8_t)gps->nmea_data.ths.mode;
    gps->data.heading.timestamp_ms = xTaskGetTickCount();
}

