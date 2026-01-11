#ifndef GPS_TYPES_H
#define GPS_TYPES_H

#include "gps_config.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct gps_s gps_t;

/* Forward declarations */
typedef enum gps_event_type_e gps_event_type_t;
typedef enum gps_protocol_e gps_protocol_t;

/**
 * @brief GPS 파싱 상태
 *
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
 * @brief GPS 이벤트 정보 (이벤트 핸들러용)
 *
 * 고수준 이벤트 + 핵심 데이터를 값 복사 방식으로 전달
 * Thread-safe: 이벤트 핸들러는 GPS Task 컨텍스트에서 실행되므로 안전
 */
typedef struct {
  gps_event_type_t type;      /**< 이벤트 타입 */
  gps_protocol_t protocol;    /**< 프로토콜 (어디서 왔는지) */
  uint32_t timestamp_ms;      /**< 수신 시간 (xTaskGetTickCount) */

  /* 이벤트별 핵심 데이터 (값 복사) */
  union {
    /* 위치 업데이트 */
    struct {
      double latitude;        /**< 위도 (degree) */
      double longitude;       /**< 경도 (degree) */
      double altitude;        /**< 고도 (meter) */
      uint8_t fix_type;       /**< Fix 타입 (GPS_FIX_xxx) */
      uint8_t sat_count;      /**< 위성 수 */
      double hdop;            /**< 수평 정밀도 */
    } position;

    /* 헤딩 업데이트 */
    struct {
      double heading;         /**< 헤딩 (degree, 0-360) */
      double pitch;           /**< 피치 (degree, 옵션) */
      float heading_std;      /**< 헤딩 표준편차 (degree, 옵션) */
      uint8_t status;         /**< 상태 ('A'=valid, 'V'=invalid) */
    } heading;

    /* 속도 업데이트 */
    struct {
      double speed;           /**< 속도 (m/s) */
      double track;           /**< 진행 방향 (degree, 0-360) */
      uint8_t mode;           /**< 모드 */
    } velocity;

    /* 위성 정보 업데이트 */
    struct {
      uint8_t sat_count;      /**< 위성 수 */
      double hdop;            /**< 수평 정밀도 */
      double vdop;            /**< 수직 정밀도 */
    } satellite;

    /* RTCM 수신 */
    struct {
      uint16_t msg_type;      /**< RTCM 메시지 타입 (1074, 1127 등) */
      uint16_t length;        /**< 메시지 길이 */
    } rtcm;

    /* 명령어 응답 */
    struct {
      bool success;           /**< OK=true, ERROR=false */
    } cmd_response;
  } data;

  /* 디버깅/로깅용 저수준 정보 */
  union {
    gps_nmea_msg_t nmea_msg_id;         /**< NMEA 메시지 ID */
    uint16_t unicore_bin_msg_id;        /**< Unicore Binary 메시지 ID */
    uint16_t rtcm_msg_type;             /**< RTCM 메시지 타입 */
  } source;

} gps_event_t;

typedef struct {
  int (*init)(void);
  int (*start)(void);
  int (*stop)(void);
  int (*reset)(void);
  int (*send)(const char *data, size_t len);
  int (*recv)(char *buf, size_t len);
} gps_hal_ops_t;

#endif
