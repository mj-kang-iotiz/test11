#ifndef GPS_TYPES_H
#define GPS_TYPES_H

/**
 * @file gps_types.h
 * @brief GPS 기본 타입 정의
 */

#include "gps_config.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct gps_s gps_t;

/**
 * @brief GPS 파싱 상태
 */
typedef enum {
  GPS_PARSE_STATE_NONE = 0,
  GPS_PARSE_STATE_INVALID = UINT8_MAX
} gps_parse_state_t;

/**
 * @brief NMEA183 프로토콜 sentence (X-Macro로 자동 생성)
 */
#include "gps_proto_def.h"

typedef enum {
  GPS_NMEA_MSG_NONE = 0,
#define X(name, str, handler, field_count, is_urc) GPS_NMEA_MSG_##name,
  NMEA_MSG_TABLE(X)
#undef X
  GPS_NMEA_MSG_INVALID = UINT8_MAX
} gps_nmea_msg_t;

/**
 * @brief RTCM 메시지 타입 (X-Macro로 자동 생성)
 */
typedef enum {
  GPS_RTCM_MSG_NONE = 0,
#define X(name, msg_type, description) GPS_RTCM_MSG_##name = msg_type,
  RTCM_MSG_TABLE(X)
#undef X
  GPS_RTCM_MSG_UNKNOWN = 0xFFFF
} gps_rtcm_msg_t;

/**
 * @brief GPS HAL 인터페이스
 */
typedef struct {
  int (*init)(void);
  int (*start)(void);
  int (*stop)(void);
  int (*reset)(void);
  int (*send)(const char *data, size_t len);
  int (*recv)(char *buf, size_t len);
} gps_hal_ops_t;

#endif /* GPS_TYPES_H */
