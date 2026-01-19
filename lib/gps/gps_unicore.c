/**
 * @file gps_unicore.c
 * @brief Unicore GPS 프로토콜 파서 (ASCII + Binary)
 */

#include "gps_unicore.h"
#include "gps.h"
#include "gps_parser.h"
#include "gps_proto_def.h"
#include "dev_assert.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifndef TAG
#define TAG "GPS_UNICORE"
#endif

#include "log.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/*===========================================================================
 * CRC32 테이블 (Unicore Binary용)
 *===========================================================================*/
static const uint32_t crc_table[256] = {
    0x00000000UL, 0x77073096UL, 0xee0e612cUL, 0x990951baUL, 0x076dc419UL,
    0x706af48fUL, 0xe963a535UL, 0x9e6495a3UL, 0x0edb8832UL, 0x79dcb8a4UL,
    0xe0d5e91eUL, 0x97d2d988UL, 0x09b64c2bUL, 0x7eb17cbdUL, 0xe7b82d07UL,
    0x90bf1d91UL, 0x1db71064UL, 0x6ab020f2UL, 0xf3b97148UL, 0x84be41deUL,
    0x1adad47dUL, 0x6ddde4ebUL, 0xf4d4b551UL, 0x83d385c7UL, 0x136c9856UL,
    0x646ba8c0UL, 0xfd62f97aUL, 0x8a65c9ecUL, 0x14015c4fUL, 0x63066cd9UL,
    0xfa0f3d63UL, 0x8d080df5UL, 0x3b6e20c8UL, 0x4c69105eUL, 0xd56041e4UL,
    0xa2677172UL, 0x3c03e4d1UL, 0x4b04d447UL, 0xd20d85fdUL, 0xa50ab56bUL,
    0x35b5a8faUL, 0x42b2986cUL, 0xdbbbc9d6UL, 0xacbcf940UL, 0x32d86ce3UL,
    0x45df5c75UL, 0xdcd60dcfUL, 0xabd13d59UL, 0x26d930acUL, 0x51de003aUL,
    0xc8d75180UL, 0xbfd06116UL, 0x21b4f4b5UL, 0x56b3c423UL, 0xcfba9599UL,
    0xb8bda50fUL, 0x2802b89eUL, 0x5f058808UL, 0xc60cd9b2UL, 0xb10be924UL,
    0x2f6f7c87UL, 0x58684c11UL, 0xc1611dabUL, 0xb6662d3dUL, 0x76dc4190UL,
    0x01db7106UL, 0x98d220bcUL, 0xefd5102aUL, 0x71b18589UL, 0x06b6b51fUL,
    0x9fbfe4a5UL, 0xe8b8d433UL, 0x7807c9a2UL, 0x0f00f934UL, 0x9609a88eUL,
    0xe10e9818UL, 0x7f6a0dbbUL, 0x086d3d2dUL, 0x91646c97UL, 0xe6635c01UL,
    0x6b6b51f4UL, 0x1c6c6162UL, 0x856530d8UL, 0xf262004eUL, 0x6c0695edUL,
    0x1b01a57bUL, 0x8208f4c1UL, 0xf50fc457UL, 0x65b0d9c6UL, 0x12b7e950UL,
    0x8bbeb8eaUL, 0xfcb9887cUL, 0x62dd1ddfUL, 0x15da2d49UL, 0x8cd37cf3UL,
    0xfbd44c65UL, 0x4db26158UL, 0x3ab551ceUL, 0xa3bc0074UL, 0xd4bb30e2UL,
    0x4adfa541UL, 0x3dd895d7UL, 0xa4d1c46dUL, 0xd3d6f4fbUL, 0x4369e96aUL,
    0x346ed9fcUL, 0xad678846UL, 0xda60b8d0UL, 0x44042d73UL, 0x33031de5UL,
    0xaa0a4c5fUL, 0xdd0d7cc9UL, 0x5005713cUL, 0x270241aaUL, 0xbe0b1010UL,
    0xc90c2086UL, 0x5768b525UL, 0x206f85b3UL, 0xb966d409UL, 0xce61e49fUL,
    0x5edef90eUL, 0x29d9c998UL, 0xb0d09822UL, 0xc7d7a8b4UL, 0x59b33d17UL,
    0x2eb40d81UL, 0xb7bd5c3bUL, 0xc0ba6cadUL, 0xedb88320UL, 0x9abfb3b6UL,
    0x03b6e20cUL, 0x74b1d29aUL, 0xead54739UL, 0x9dd277afUL, 0x04db2615UL,
    0x73dc1683UL, 0xe3630b12UL, 0x94643b84UL, 0x0d6d6a3eUL, 0x7a6a5aa8UL,
    0xe40ecf0bUL, 0x9309ff9dUL, 0x0a00ae27UL, 0x7d079eb1UL, 0xf00f9344UL,
    0x8708a3d2UL, 0x1e01f268UL, 0x6906c2feUL, 0xf762575dUL, 0x806567cbUL,
    0x196c3671UL, 0x6e6b06e7UL, 0xfed41b76UL, 0x89d32be0UL, 0x10da7a5aUL,
    0x67dd4accUL, 0xf9b9df6fUL, 0x8ebeeff9UL, 0x17b7be43UL, 0x60b08ed5UL,
    0xd6d6a3e8UL, 0xa1d1937eUL, 0x38d8c2c4UL, 0x4fdff252UL, 0xd1bb67f1UL,
    0xa6bc5767UL, 0x3fb506ddUL, 0x48b2364bUL, 0xd80d2bdaUL, 0xaf0a1b4cUL,
    0x36034af6UL, 0x41047a60UL, 0xdf60efc3UL, 0xa867df55UL, 0x316e8eefUL,
    0x4669be79UL, 0xcb61b38cUL, 0xbc66831aUL, 0x256fd2a0UL, 0x5268e236UL,
    0xcc0c7795UL, 0xbb0b4703UL, 0x220216b9UL, 0x5505262fUL, 0xc5ba3bbeUL,
    0xb2bd0b28UL, 0x2bb45a92UL, 0x5cb36a04UL, 0xc2d7ffa7UL, 0xb5d0cf31UL,
    0x2cd99e8bUL, 0x5bdeae1dUL, 0x9b64c2b0UL, 0xec63f226UL, 0x756aa39cUL,
    0x026d930aUL, 0x9c0906a9UL, 0xeb0e363fUL, 0x72076785UL, 0x05005713UL,
    0x95bf4a82UL, 0xe2b87a14UL, 0x7bb12baeUL, 0x0cb61b38UL, 0x92d28e9bUL,
    0xe5d5be0dUL, 0x7cdcefb7UL, 0x0bdbdf21UL, 0x86d3d2d4UL, 0xf1d4e242UL,
    0x68ddb3f8UL, 0x1fda836eUL, 0x81be16cdUL, 0xf6b9265bUL, 0x6fb077e1UL,
    0x18b74777UL, 0x88085ae6UL, 0xff0f6a70UL, 0x66063bcaUL, 0x11010b5cUL,
    0x8f659effUL, 0xf862ae69UL, 0x616bffd3UL, 0x166ccf45UL, 0xa00ae278UL,
    0xd70dd2eeUL, 0x4e048354UL, 0x3903b3c2UL, 0xa7672661UL, 0xd06016f7UL,
    0x4969474dUL, 0x3e6e77dbUL, 0xaed16a4aUL, 0xd9d65adcUL, 0x40df0b66UL,
    0x37d83bf0UL, 0xa9bcae53UL, 0xdebb9ec5UL, 0x47b2cf7fUL, 0x30b5ffe9UL,
    0xbdbdf21cUL, 0xcabac28aUL, 0x53b39330UL, 0x24b4a3a6UL, 0xbad03605UL,
    0xcdd70693UL, 0x54de5729UL, 0x23d967bfUL, 0xb3667a2eUL, 0xc4614ab8UL,
    0x5d681b02UL, 0x2a6f2b94UL, 0xb40bbe37UL, 0xc30c8ea1UL, 0x5a05df1bUL,
    0x2d02ef8dUL
};

/*===========================================================================
 * 내부 함수 선언
 *===========================================================================*/
static uint32_t calc_crc32(const uint8_t *buf, size_t len);
static bool unicore_ascii_verify_crc(const char *buf, size_t len, size_t *star_pos);
static void unicore_bin_parse_bestnav(gps_t *gps, const uint8_t *payload, size_t len);

/*===========================================================================
 * X-Macro 기반 Unicore Binary 핸들러 테이블
 *===========================================================================*/
typedef void (*unicore_bin_handler_t)(gps_t *gps, const uint8_t *payload, size_t len);

static const struct {
    uint16_t msg_id;
    unicore_bin_handler_t handler;
    bool is_urc;
} unicore_bin_msg_table[] = {
#define X(name, msg_id, handler, is_urc) { msg_id, handler, is_urc },
    UNICORE_BIN_MSG_TABLE(X)
#undef X
};

#define UNICORE_BIN_MSG_TABLE_SIZE (sizeof(unicore_bin_msg_table) / sizeof(unicore_bin_msg_table[0]))

/*===========================================================================
 * X-Macro 기반 문자열 변환 함수
 *===========================================================================*/

/* Unicore Binary 메시지 ID → 문자열 */
static const char* unicore_bin_msg_to_str(uint16_t msg_id) {
    switch (msg_id) {
#define X(name, id, handler, is_urc) case id: return #name;
        UNICORE_BIN_MSG_TABLE(X)
#undef X
        default: return "UNKNOWN";
    }
}

/* Unicore Response → 문자열 */
static const char* unicore_resp_to_str(gps_unicore_resp_t resp) {
    if (resp == GPS_UNICORE_RESP_NONE) return "NONE";

    switch (resp) {
#define X(name, str) case GPS_UNICORE_RESP_##name: return str;
        UNICORE_RESP_TABLE(X)
#undef X
        default: return "UNKNOWN";
    }
}

/*===========================================================================
 * Unicore ASCII 파서 ($command,response:OK*XX)
 *===========================================================================*/

parse_result_t unicore_ascii_try_parse(gps_t *gps, ringbuffer_t *rb) {
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
    size_t star_pos;
    if (!unicore_ascii_verify_crc(buf, cr_pos, &star_pos)) {
        gps->parser_ctx.stats.crc_errors++;
        ringbuffer_advance(rb, pkt_len);
        return PARSE_INVALID;
    }

    /* 6. Response 파싱 ("response:OK" 또는 "response:ERROR") */
    gps_unicore_resp_t resp = GPS_UNICORE_RESP_UNKNOWN;
    const char *resp_str = strstr(buf, "response:");
    if (resp_str) {
        resp_str += 9;  /* "response:" 길이 */

        while(*resp_str == ' ' || *resp_str == '\t')
        {
        	resp_str++;
        }
        /* 응답 전체 메시지 로그 출력 (명령어 초기화 시 확인용) */
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
 * Unicore Binary 파서 (0xAA 0x44 0xB5)
 *===========================================================================*/

parse_result_t unicore_bin_try_parse(gps_t *gps, ringbuffer_t *rb) {
    /* 1. 첫 바이트 확인 */
    uint8_t first;
    if (!ringbuffer_peek(rb, (char*)&first, 1, 0)) {
        return PARSE_NEED_MORE;
    }
    if (first != GPS_UNICORE_BIN_SYNC_1) {  /* 0xAA */
        return PARSE_NOT_MINE;
    }

    /* 2. Sync 패턴 확인 (3바이트: 0xAA 0x44 0xB5) */
    uint8_t sync[3];
    if (!ringbuffer_peek(rb, (char*)sync, 3, 0)) {
        return PARSE_NEED_MORE;
    }
    if (sync[1] != GPS_UNICORE_BIN_SYNC_2 || sync[2] != GPS_UNICORE_BIN_SYNC_3) {
        return PARSE_NOT_MINE;  /* 0xAA로 시작하지만 Unicore binary 아님 */
    }

    /* 3. 헤더 길이만큼 peek (24바이트) */
    if (ringbuffer_size(rb) < GPS_UNICORE_BIN_HEADER_SIZE) {
        return PARSE_NEED_MORE;
    }

    uint8_t header[GPS_UNICORE_BIN_HEADER_SIZE];
    if (!ringbuffer_peek(rb, (char*)header, GPS_UNICORE_BIN_HEADER_SIZE, 0)) {
        return PARSE_NEED_MORE;
    }

    /* 4. 메시지 길이 추출 (offset 6-7, little endian) */
    uint16_t msg_len = header[6] | (header[7] << 8);
    uint16_t msg_id = header[4] | (header[5] << 8);

    /* 5. 전체 패킷 길이 = 헤더(24) + 페이로드 + CRC(4) */
    size_t total_len = GPS_UNICORE_BIN_HEADER_SIZE + msg_len + 4;

    if (total_len > GPS_MAX_PACKET_LEN) {
        /* 비정상적으로 큰 패킷 */
        return PARSE_INVALID;
    }

    if (ringbuffer_size(rb) < total_len) {
        return PARSE_NEED_MORE;
    }

    /* 6. 전체 패킷 peek */
    uint8_t packet[GPS_MAX_PACKET_LEN];
    if (!ringbuffer_peek(rb, (char*)packet, total_len, 0)) {
        return PARSE_NEED_MORE;
    }

    /* 7. CRC32 검증 */
    uint32_t calc_crc = calc_crc32(packet, GPS_UNICORE_BIN_HEADER_SIZE + msg_len);
    uint32_t recv_crc;
    memcpy(&recv_crc, &packet[GPS_UNICORE_BIN_HEADER_SIZE + msg_len], 4);

    if (calc_crc != recv_crc) {
        gps->parser_ctx.stats.crc_errors++;
        ringbuffer_advance(rb, total_len);
        return PARSE_INVALID;
    }

    /* 8. 메시지별 데이터 파싱 (테이블 기반) */
    const uint8_t *payload = &packet[GPS_UNICORE_BIN_HEADER_SIZE];

    for (size_t i = 0; i < UNICORE_BIN_MSG_TABLE_SIZE; i++) {
        if (unicore_bin_msg_table[i].msg_id == msg_id) {
            if (unicore_bin_msg_table[i].handler) {
                unicore_bin_msg_table[i].handler(gps, payload, msg_len);
            }
            break;
        }
    }

    /* 9. 헤더 정보 저장 */
    const gps_unicore_bin_header_t *hdr = (const gps_unicore_bin_header_t *)header;
    gps->unicore_bin_data.last_msg_id = msg_id;
    gps->unicore_bin_data.gps_week = hdr->wm;
    gps->unicore_bin_data.gps_ms = hdr->ms;
    gps->unicore_bin_data.timestamp_ms = xTaskGetTickCount();

    /* 10. advance */
    ringbuffer_advance(rb, total_len);
    gps->parser_ctx.stats.unicore_bin_packets++;

    LOG_DEBUG("Unicore Binary: %s (ID=%d, len=%d)",
              unicore_bin_msg_to_str(msg_id), msg_id, total_len);

    /* 11. 고수준 이벤트 핸들러 호출 (URC) */
    if (gps->handler) {
        gps_event_t event = {
            .protocol = GPS_PROTOCOL_UNICORE_BIN,
            .timestamp_ms = xTaskGetTickCount(),
            .source.unicore_bin_msg_id = msg_id
        };

        /* 메시지 타입별로 적절한 이벤트 생성 */
        if (msg_id == GPS_UNICORE_BIN_MSG_BESTNAV) {
            /* 위치 업데이트 이벤트 (공용 데이터 기준) */
            event.type = GPS_EVENT_POSITION_UPDATED;
            event.data.position.latitude = gps->data.position.latitude;
            event.data.position.longitude = gps->data.position.longitude;
            event.data.position.altitude = gps->data.position.altitude;
            event.data.position.fix_type = gps->data.status.fix_type;  /* GGA에서 업데이트 */
            event.data.position.sat_count = gps->data.status.sat_count;  /* BESTNAV.sv */
            event.data.position.hdop = gps->data.status.hdop;  /* GGA에서 업데이트 */
            gps->handler(gps, &event);

            /* 속도 업데이트 이벤트 */
            event.type = GPS_EVENT_VELOCITY_UPDATED;
            event.data.velocity.speed = gps->data.velocity.hor_speed;
            event.data.velocity.track = gps->data.velocity.track;
            event.data.velocity.mode = 0;
            gps->handler(gps, &event);
        }
        /* TODO: HEADING2, BESTPOS, BESTVEL 등 다른 메시지 처리 */
    }

    return PARSE_OK;
}

/*===========================================================================
 * 내부 함수 구현
 *===========================================================================*/

/**
 * @brief CRC32 계산 (Unicore Binary용)
 */
static uint32_t calc_crc32(const uint8_t *buf, size_t len) {
    uint32_t crc32 = 0;
    for (size_t i = 0; i < len; i++) {
        crc32 = crc_table[(crc32 ^ buf[i]) & 0xFF] ^ (crc32 >> 8);
    }
    return crc32;
}

/**
 * @brief Unicore ASCII CRC 검증
 * 형식: $command,response:OK*XX (XOR checksum, $ 다음부터 * 전까지)
 * 단, ':' 이후는 CRC에 포함하지 않음
 */
static bool unicore_ascii_verify_crc(const char *buf, size_t len, size_t *star_pos) {
    const char *star = memchr(buf, '*', len);
    if (!star || (size_t)(star - buf + 3) > len) {
        return false;
    }

    *star_pos = star - buf;

    /* ':' 위치 찾기 */
    const char *colon = memchr(buf, ':', len);
    const char *crc_end = colon ? (colon + 1) : star;

    /* CRC 계산 ($ 다음부터 : 또는 * 전까지) */
    uint8_t calc_crc = 0;
    for (const char *p = buf + 1; p < crc_end; p++) {
        calc_crc ^= (uint8_t)*p;
    }

    uint8_t recv_crc = hex_to_byte(star + 1);
    return (calc_crc == recv_crc);
}

/**
 * @brief BESTNAV 메시지 파싱
 */
static void unicore_bin_parse_bestnav(gps_t *gps, const uint8_t *payload, size_t len) {
    if (len < sizeof(hpd_unicore_bestnavb_t)) {
        return;
    }

    /* 임시로 구조체에 복사 */
    hpd_unicore_bestnavb_t nav;
    memcpy(&nav, payload, sizeof(hpd_unicore_bestnavb_t));

    uint32_t now = xTaskGetTickCount();

    /* === 프로토콜별 원본 데이터 업데이트 (unicore_bin_data) === */
    gps->unicore_bin_data.position.valid = true;
    gps->unicore_bin_data.position.latitude = nav.lat;
    gps->unicore_bin_data.position.longitude = nav.lon;
    gps->unicore_bin_data.position.altitude = nav.height;
    gps->unicore_bin_data.position.pos_type = nav.pos_type;
    gps->unicore_bin_data.position.lat_std = nav.lat_dev;
    gps->unicore_bin_data.position.lon_std = nav.lon_dev;
    gps->unicore_bin_data.position.alt_std = nav.height_dev;
    gps->unicore_bin_data.position.source_msg = 2118;  /* BESTNAV */

    gps->unicore_bin_data.velocity.valid = true;
    gps->unicore_bin_data.velocity.hor_speed = nav.hor_speed;
    gps->unicore_bin_data.velocity.trk_gnd = nav.trk_gnd;
    gps->unicore_bin_data.velocity.ver_speed = nav.vert_speed;
    gps->unicore_bin_data.velocity.source_msg = 2118;  /* BESTNAV */

    /* === 공용 데이터 업데이트 (gps->data) === */
    /* 위치 */
    gps->data.position.latitude = nav.lat;
    gps->data.position.longitude = nav.lon;
    gps->data.position.altitude = nav.height;
    gps->data.position.lat_std = nav.lat_dev;
    gps->data.position.lon_std = nav.lon_dev;
    gps->data.position.alt_std = nav.height_dev;
    gps->data.position.timestamp_ms = now;

    /* 속도 */
    gps->data.velocity.hor_speed = nav.hor_speed;
    gps->data.velocity.ver_speed = nav.vert_speed;
    gps->data.velocity.track = nav.trk_gnd;
    gps->data.velocity.timestamp_ms = now;

    /* 위성수 */
    gps->data.status.sat_count = nav.sv;
    gps->data.status.used_sat_count = nav.used_sv;
    gps->data.status.sat_timestamp_ms = now;
}

/*===========================================================================
 * 레거시 API (기존 코드 호환용)
 *===========================================================================*/

gps_unicore_resp_t gps_get_unicore_response(gps_t *gps) {
    /* deprecated */
    (void)gps;
    return GPS_UNICORE_RESP_UNKNOWN;
}

uint8_t gps_parse_unicore_term(gps_t *gps) {
    /* deprecated */
    (void)gps;
    return 0;
}

uint8_t gps_parse_unicore_bin(gps_t *gps) {
    /* deprecated */
    (void)gps;
    return 0;
}
