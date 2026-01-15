#ifndef GPS_PARSER_H
#define GPS_PARSER_H

/**
 * @file gps_parser.h
 * @brief GPS 파서 인터페이스
 *
 * Chain 방식의 프로토콜 파서
 * - 각 프로토콜 파서가 "내 패킷인지" 판단
 * - 아니면 다음 파서로 넘김
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ringbuffer.h"
#include "gps_event.h"

/*===========================================================================
 * 파서 결과 타입
 *===========================================================================*/
typedef enum {
    PARSE_NOT_MINE = 0,   /**< 이 프로토콜 아님 -> 다음 파서 시도 */
    PARSE_NEED_MORE,      /**< 내 패킷 맞지만 데이터 부족 -> 루프 탈출, 대기 */
    PARSE_OK,             /**< 파싱 완료, advance 됨 -> 계속 루프 */
    PARSE_INVALID,        /**< 내 패킷인데 잘못됨 (CRC 등) -> 1 byte skip */
} parse_result_t;

/*===========================================================================
 * 명령어 응답 대기 컨텍스트
 *===========================================================================*/
typedef struct {
    bool waiting;                 /**< 응답 대기 중 여부 */
    bool result_ok;               /**< 응답 결과 (OK/ERROR) */
} gps_cmd_ctx_t;

/*===========================================================================
 * 파서 통계 (디버깅용)
 *===========================================================================*/
typedef struct {
    uint32_t rx_packets;          /**< 수신 패킷 수 */
    uint32_t nmea_packets;        /**< NMEA 패킷 수 */
    uint32_t unicore_cmd_packets; /**< Unicore 명령어 응답 수 */
    uint32_t unicore_bin_packets; /**< Unicore Binary 패킷 수 */
    uint32_t rtcm_packets;        /**< RTCM 패킷 수 */
    uint32_t crc_errors;          /**< CRC 오류 수 */
    uint32_t invalid_packets;     /**< 잘못된 패킷 수 */
    uint32_t unknown_packets;     /**< 알 수 없는 패킷 (skip) */

    /* 수신 시간 추적 */
    uint32_t last_rx_tick;        /**< 마지막 수신 tick (xTaskGetTickCount) */
    uint32_t last_nmea_tick;      /**< 마지막 NMEA 수신 tick */
    uint32_t last_rtcm_tick;      /**< 마지막 RTCM 수신 tick */
} gps_parser_stats_t;

/*===========================================================================
 * 파서 컨텍스트
 *===========================================================================*/
typedef struct {
    gps_cmd_ctx_t cmd_ctx;        /**< 명령어 응답 컨텍스트 */
    gps_parser_stats_t stats;     /**< 파서 통계 */
} gps_parser_ctx_t;

/*===========================================================================
 * 메인 파서 API
 *===========================================================================*/

/**
 * @brief GPS 파서 초기화
 * @param gps GPS 핸들
 */
void gps_parser_init(gps_t *gps);

/**
 * @brief GPS 패킷 파싱 (메인 루프)
 *
 * ringbuffer에서 데이터를 읽어 파싱
 * Chain 방식: NMEA -> Unicore ASCII -> Unicore Binary -> RTCM
 *
 * @param gps GPS 핸들
 * @return 마지막 파싱 결과
 */
parse_result_t gps_parser_process(gps_t *gps);

/**
 * @brief 파서 통계 가져오기
 * @param gps GPS 핸들
 * @return 통계 구조체 포인터
 */
const gps_parser_stats_t* gps_parser_get_stats(gps_t *gps);

/**
 * @brief 파서 통계 초기화
 * @param gps GPS 핸들
 */
void gps_parser_reset_stats(gps_t *gps);

/*===========================================================================
 * 개별 프로토콜 파서 (내부용, 각 파일에서 구현)
 *===========================================================================*/

/**
 * @brief NMEA 패킷 파싱 시도
 * @param gps GPS 핸들
 * @param rb ringbuffer
 * @return 파싱 결과
 */
parse_result_t nmea_try_parse(gps_t *gps, ringbuffer_t *rb);

/**
 * @brief Unicore ASCII 패킷 파싱 시도 ($command,response:...)
 * @param gps GPS 핸들
 * @param rb ringbuffer
 * @return 파싱 결과
 */
parse_result_t unicore_ascii_try_parse(gps_t *gps, ringbuffer_t *rb);

/**
 * @brief Unicore Binary 패킷 파싱 시도 (0xAA 0x44 0xB5)
 * @param gps GPS 핸들
 * @param rb ringbuffer
 * @return 파싱 결과
 */
parse_result_t unicore_bin_try_parse(gps_t *gps, ringbuffer_t *rb);

/**
 * @brief RTCM 패킷 파싱 시도 (0xD3)
 * @param gps GPS 핸들
 * @param rb ringbuffer
 * @return 파싱 결과
 */
parse_result_t rtcm_try_parse(gps_t *gps, ringbuffer_t *rb);

/*===========================================================================
 * 유틸리티 함수
 *===========================================================================*/

/**
 * @brief ringbuffer에서 특정 문자 위치 찾기
 * @param rb ringbuffer
 * @param ch 찾을 문자
 * @param max_search 최대 검색 범위
 * @param[out] pos 찾은 위치
 * @return true: 찾음, false: 못 찾음
 */
bool ringbuffer_find_char(ringbuffer_t *rb, char ch, size_t max_search, size_t *pos);

/**
 * @brief HEX 문자를 숫자로 변환
 * @param ch HEX 문자 ('0'-'9', 'A'-'F', 'a'-'f')
 * @return 숫자 값 (0-15), 실패 시 0
 */
uint8_t hex_char_to_num(char ch);

/**
 * @brief 2자리 HEX 문자열을 바이트로 변환
 * @param hex HEX 문자열 (2자리)
 * @return 바이트 값
 */
uint8_t hex_to_byte(const char *hex);

#endif /* GPS_PARSER_H */
