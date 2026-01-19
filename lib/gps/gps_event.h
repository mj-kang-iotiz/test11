#ifndef GPS_EVENT_H
#define GPS_EVENT_H

/**
 * @file gps_event.h
 * @brief GPS 이벤트 타입 정의
 */

#include "gps_types.h"

/**
 * @brief 프로토콜 타입
 */
typedef enum gps_protocol_e {
    GPS_PROTOCOL_NONE = 0,
    GPS_PROTOCOL_NMEA,            /**< $GPGGA, $GNGGA 등 NMEA-0183 */
    GPS_PROTOCOL_UNICORE_CMD,     /**< $command,response:OK*XX (설정 명령어) */
    GPS_PROTOCOL_UNICORE_BIN,     /**< 0xAA 0x44 0xB5 ... (Binary 메시지) */
    GPS_PROTOCOL_RTCM,            /**< 0xD3 ... (RTCM3) */
} gps_protocol_t;

/**
 * @brief 고수준 이벤트 타입 (애플리케이션 중심)
 */
typedef enum gps_event_type_e {
    GPS_EVENT_NONE = 0,
    GPS_EVENT_POSITION_UPDATED,   /**< 위치 업데이트 (BESTNAV, BESTPOS) */
    GPS_EVENT_HEADING_UPDATED,    /**< 헤딩 업데이트 (THS, HEADING2) */
    GPS_EVENT_VELOCITY_UPDATED,   /**< 속도 업데이트 (RMC, BESTVEL) */
    GPS_EVENT_FIX_UPDATED,        /**< Fix 상태 업데이트 (GGA) */
    GPS_EVENT_SATELLITE_UPDATED,  /**< 위성 정보 업데이트 (GSA, GSV) */
    GPS_EVENT_RTCM_RECEIVED,      /**< RTCM 데이터 수신 (LoRa 전송용) */
    GPS_EVENT_CMD_RESPONSE,       /**< 명령어 응답 수신 (OK/ERROR) */
} gps_event_type_t;

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

    /* Fix 상태 업데이트 (GGA) */
    struct {
      uint8_t fix_type;       /**< Fix 타입 (gps_fix_t) */
      double hdop;            /**< 수평 정밀도 */
    } fix;

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

#endif /* GPS_EVENT_H */
