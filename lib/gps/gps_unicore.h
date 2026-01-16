#ifndef GPS_UNICORE_H
#define GPS_UNICORE_H

#include "gps_types.h"
#include "gps_proto_def.h"
#include <stdint.h>
#include <stdbool.h>

#define GPS_UNICORE_TERM_SIZE 32
#define GPS_UNICORE_BIN_HEADER_SIZE 24
#define GPS_UNICORE_BIN_SYNC_1 0xAA
#define GPS_UNICORE_BIN_SYNC_2 0x44
#define GPS_UNICORE_BIN_SYNC_3 0xB5

/**
 * @brief BESTNAV pos_type 값 (위치 솔루션 타입)
 * GGA의 gps_fix_t와 다른 값 체계이므로 주의!
 */
typedef enum {
    GPS_POS_TYPE_NONE        = 0,   /**< 솔루션 없음 */
    GPS_POS_TYPE_RTK_FIXED   = 16,  /**< RTK Fixed (cm급 정밀도) */
    GPS_POS_TYPE_RTK_FLOAT   = 17,  /**< RTK Float (dm급 정밀도) */
    GPS_POS_TYPE_SBAS        = 18,  /**< SBAS */
    GPS_POS_TYPE_PSRDIFF     = 32,  /**< PSRDIFF */
    GPS_POS_TYPE_WAAS        = 33,  /**< WAAS */
    GPS_POS_TYPE_PPP         = 34,  /**< PPP (Precise Point Positioning) */
    GPS_POS_TYPE_SINGLE      = 50,  /**< 단독 측위 (m급 정밀도) */
    GPS_POS_TYPE_INS_PSRSP   = 52,  /**< INS + Single Point */
    GPS_POS_TYPE_INS_PSRDIFF = 53,  /**< INS + PSRDIFF */
    GPS_POS_TYPE_INS_RTK_FLOAT = 54, /**< INS + RTK Float */
    GPS_POS_TYPE_INS_RTK_FIXED = 55, /**< INS + RTK Fixed */
} gps_pos_type_t;

/** @brief RTK Fixed 상태인지 확인하는 매크로 */
#define GPS_IS_RTK_FIXED(pos_type) ((pos_type) == GPS_POS_TYPE_RTK_FIXED || (pos_type) == GPS_POS_TYPE_INS_RTK_FIXED)

typedef enum {
  GPS_UNICORE_MSG_NONE = 0,
  GPS_UNICORE_MSG_COMMAND = 1
} gps_unicore_msg_t;

/**
 * @brief Unicore 명령어 응답 타입 (X-Macro로 자동 생성)
 */
typedef enum {
  GPS_UNICORE_RESP_NONE = 0,
#define X(name, str) GPS_UNICORE_RESP_##name,
  UNICORE_RESP_TABLE(X)
#undef X
} gps_unicore_resp_t;

typedef struct {
  char term_str[GPS_UNICORE_TERM_SIZE];
  uint8_t term_pos;
  uint8_t term_num;

  gps_unicore_msg_t msg_type;
  uint8_t crc;
  uint8_t star;
  uint8_t colon; ///< '$이후부터 : 받을때 까지만 crc 검사 해야함! response: OK에서 스페이스바 부터 OK까지 포함하면 안됨
  gps_unicore_resp_t response;
} gps_unicore_parser_t;

/**
 * @brief Unicore Binary 메시지 타입 (X-Macro로 자동 생성)
 */
typedef enum {
#define X(name, msg_id, handler, is_urc) GPS_UNICORE_BIN_MSG_##name = msg_id,
    UNICORE_BIN_MSG_TABLE(X)
#undef X
} gps_unicore_bin_msg_t;

typedef struct __attribute__((packed)) {
    uint8_t sync[3]; ///< 0xAA 0x44 0xB5
    uint8_t cpu_idle; ///< CPU idle 0-100
    uint16_t message_id;
    uint16_t message_len;
    uint8_t time_ref; ///< reference time (GPST or BDST)
    uint8_t time_status; ///< time status
    uint16_t wm; ///< week number
    uint32_t ms; ///< seconds of week (ms)
    uint32_t version; ///< release version
    uint8_t reserved;
    uint8_t leap_sec; ///< leap second
    uint16_t delay_ms; ///< output delay
} gps_unicore_bin_header_t;

typedef struct {
  gps_unicore_bin_header_t header;
  uint32_t crc32;
} gps_unicore_bin_parser_t;

typedef struct __attribute__((packed))
{
    uint32_t psol_status;
    uint32_t pos_type;
    double lat;
    double lon;
    double height;
    float geoid;
    uint32_t datum_id;
    float lat_dev; ///< latitude deviation
    float lon_dev;
    float height_dev;
    char base_station_id[4];
    float diff_age;
    float sol_age;
    uint8_t sv;
    uint8_t used_sv;
    uint8_t reserved[3];
    uint8_t ext_sol_stat;
    uint8_t galileo_bds3_sig_mask;
    uint8_t gps_glonass_bds2_sig_mask;
    uint32_t v_sol_status;
    uint32_t vel_type;
    float latency;
    float age;
    double hor_speed;
    double trk_gnd;
    double vert_speed;
    float verspd_std;
    float horspd_std;
}hpd_unicore_bestnavb_t;

/* gps_unicore_bin_data_t는 gps.h에서 정의됨 (header 필드 포함) */

gps_unicore_resp_t gps_get_unicore_response(gps_t *gps);
uint8_t gps_parse_unicore_term(gps_t *gps);
uint8_t gps_parse_unicore_bin(gps_t *gps);

#endif