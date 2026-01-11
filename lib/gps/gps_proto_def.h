#ifndef GPS_PROTO_DEF_H
#define GPS_PROTO_DEF_H

/**
 * @file gps_proto_def.h
 * @brief GPS 프로토콜 정의 (X-Macro 테이블)
 *
 * X-Macro 패턴으로 enum, 테이블, 문자열 변환을 한 곳에서 관리
 * UM982 GPS 기준
 */

/*===========================================================================
 * NMEA 183 메시지 정의 (UM982 지원)
 * X(name, str, handler, field_count, is_urc)
 *   - name: enum 이름 suffix (GGA, RMC 등) → GPS_NMEA_MSG_GGA로 자동 생성
 *   - str: 메시지 문자열 ("GGA", "RMC" 등)
 *   - handler: 파싱 핸들러 함수 (NULL이면 무시)
 *   - field_count: 최소 필드 수 (delimiter ',' 기준, 0이면 체크 안함)
 *   - is_urc: URC(비동기 데이터) 여부
 *
 * NOTE: enum 값은 GPS_NMEA_MSG_NONE(0) 다음부터 자동으로 1, 2, 3... 할당됨
 *       테이블에서는 생성된 enum 값을 명시적으로 저장하여 안전하게 매핑
 *===========================================================================*/
#define NMEA_MSG_TABLE(X) \
    X(GGA, "GGA", nmea_parse_gga, 14, true)  /* Global Positioning System Fix Data */ \
    X(RMC, "RMC", nmea_parse_rmc, 12, true)  /* Recommended Minimum */ \
    X(THS, "THS", nmea_parse_ths,  2, true)  /* True Heading and Status */ \
    X(VTG, "VTG", NULL,            8, false) /* Track made good and Ground speed */ \
    X(GSA, "GSA", NULL,           17, false) /* GPS DOP and active satellites */ \
    X(GSV, "GSV", NULL,            4, false) /* Satellites in view */ \
    X(ZDA, "ZDA", NULL,            6, false) /* Time & Date */

/*===========================================================================
 * Unicore ASCII 응답 정의 (Command Response)
 * X(name, str)
 *   - name: enum 이름 (GPS_UNICORE_RESP_xxx)
 *   - str: 응답 문자열
 *===========================================================================*/
#define UNICORE_RESP_TABLE(X) \
    X(OK,      "OK")    \
    X(ERROR,   "ERROR") \
    X(UNKNOWN, "")

/*===========================================================================
 * Unicore Binary 메시지 정의 (UM982 지원)
 * X(name, msg_id, handler, is_urc)
 *   - name: enum 이름
 *   - msg_id: 메시지 ID (16비트)
 *   - handler: 파싱 핸들러 함수 (NULL이면 무시)
 *   - is_urc: URC 여부
 *===========================================================================*/
#define UNICORE_BIN_MSG_TABLE(X) \
    X(BESTNAV,  2118, unicore_bin_parse_bestnav,  true)  /* Best GNSS position & velocity */ \
    X(HEADING2, 2120, NULL,                       true)  /* Dual-antenna heading (TODO) */ \
    X(INSPVA,   2122, NULL,                       true)  /* INS position, velocity, attitude */ \
    X(BESTPOS,  42,   NULL,                       true)  /* Best GNSS position */ \
    X(BESTVEL,  99,   NULL,                       true)  /* Best GNSS velocity */

/*===========================================================================
 * RTCM 메시지 정의 (UM982 지원)
 * X(name, msg_type, description)
 *   - name: enum 이름
 *   - msg_type: RTCM 메시지 타입 (12비트)
 *   - description: 설명
 *
 * 참고: RTCM은 raw 전송만 하므로 핸들러 불필요
 *===========================================================================*/
#define RTCM_MSG_TABLE(X) \
    X(MSM4_GPS,     1074, "GPS MSM4")        \
    X(MSM7_GPS,     1077, "GPS MSM7")        \
    X(MSM4_GLONASS, 1084, "GLONASS MSM4")    \
    X(MSM7_GLONASS, 1087, "GLONASS MSM7")    \
    X(MSM4_GALILEO, 1094, "Galileo MSM4")    \
    X(MSM7_GALILEO, 1097, "Galileo MSM7")    \
    X(MSM4_BEIDOU,  1124, "BeiDou MSM4")     \
    X(MSM7_BEIDOU,  1127, "BeiDou MSM7")     \
    X(STATION_INFO, 1005, "Station Position")

/*===========================================================================
 * 프로토콜 상수
 *===========================================================================*/
#define GPS_MAX_PACKET_LEN      512   // 최대 패킷 길이 (Unicore Binary)
#define GPS_NMEA_MAX_LEN        120   // NMEA 최대 길이
#define GPS_UNICORE_ASCII_MAX   128   // Unicore ASCII 최대 길이

#endif /* GPS_PROTO_DEF_H */
