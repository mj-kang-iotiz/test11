#ifndef GPS_PROTO_DEF_H
#define GPS_PROTO_DEF_H

/**
 * @file gps_proto_def.h
 * @brief GPS 프로토콜 정의 (X-Macro 테이블)
 *
 * X-Macro 패턴을 사용하여 메시지 타입, 핸들러, 필드 수 등을 한 곳에서 관리
 */

/*===========================================================================
 * NMEA 183 메시지 정의
 * X(name, str, handler, field_count, is_urc)
 *   - name: enum 이름 (GPS_NMEA_MSG_xxx)
 *   - str: 메시지 문자열 (GGA, RMC 등)
 *   - handler: 파싱 핸들러 함수
 *   - field_count: 기대 필드 수 (delimiter ',' 기준)
 *   - is_urc: URC(비동기 데이터) 여부
 *===========================================================================*/
#define NMEA_MSG_TABLE(X) \
    X(GGA, "GGA", nmea_parse_gga, 14, true)  \
    X(RMC, "RMC", nmea_parse_rmc, 12, true)  \
    X(THS, "THS", nmea_parse_ths,  2, true)

/*===========================================================================
 * Unicore ASCII 응답 정의 (Command Response)
 * X(name, str)
 *   - name: enum 이름 (GPS_UNICORE_RESP_xxx)
 *   - str: 응답 문자열
 *===========================================================================*/
#define UNICORE_RESP_TABLE(X) \
    X(OK,    "OK")    \
    X(ERROR, "ERROR")

/*===========================================================================
 * Unicore Binary 메시지 정의
 * X(name, msg_id, handler, is_urc)
 *   - name: enum 이름
 *   - msg_id: 메시지 ID (16비트)
 *   - handler: 파싱 핸들러 함수
 *   - is_urc: URC 여부
 *===========================================================================*/
#define UNICORE_BIN_MSG_TABLE(X) \
    X(BESTNAV, 2118, unicore_bin_parse_bestnav, true)

/*===========================================================================
 * RTCM 메시지 정의
 * X(name, msg_type, is_urc)
 *   - name: enum 이름
 *   - msg_type: RTCM 메시지 타입 (12비트)
 *   - is_urc: URC 여부 (RTCM은 항상 true)
 *===========================================================================*/
#define RTCM_MSG_TABLE(X) \
    X(1005, 1005, true) \
    X(1074, 1074, true) \
    X(1077, 1077, true) \
    X(1084, 1084, true) \
    X(1087, 1087, true) \
    X(1094, 1094, true) \
    X(1097, 1097, true) \
    X(1124, 1124, true) \
    X(1127, 1127, true)

/*===========================================================================
 * 프로토콜 상수
 *===========================================================================*/
#define GPS_MAX_PACKET_LEN      512   // 최대 패킷 길이
#define GPS_NMEA_MAX_LEN        120   // NMEA 최대 길이
#define GPS_UNICORE_ASCII_MAX   128   // Unicore ASCII 최대 길이

#endif /* GPS_PROTO_DEF_H */
