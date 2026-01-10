#ifndef GPS_TYPES_H
#define GPS_TYPES_H

#include "gps_config.h"
#include <stdint.h>

typedef struct gps_s gps_t;

/**
 * @brief GPS 파싱 상태
 *
 */
typedef enum {
  GPS_PARSE_STATE_NONE = 0,

  GPS_PARSE_STATE_INVALID = UINT8_MAX
} gps_parse_state_t;

/**
 * @brief NMEA183 프로토콜 sentence
 *
 */
typedef enum {
  GPS_NMEA_MSG_NONE = 0,
  GPS_NMEA_MSG_GGA = 1,
  GPS_NMEA_MSG_RMC = 2,
  GPS_NMEA_MSG_THS = 3,
  GPS_NMEA_MSG_INVALID = UINT8_MAX
} gps_nmea_msg_t;

typedef union {
  gps_nmea_msg_t nmea;
  struct {
    uint8_t class;
    uint8_t id;
  } ubx;
  struct {
    uint16_t msg;
  } unicore_bin;
  struct {
    uint8_t response;
  } unicore;
  struct {
    uint16_t msg_type;
  } rtcm;
} gps_msg_t;

typedef struct {
  int (*init)(void);
  int (*start)(void);
  int (*stop)(void);
  int (*reset)(void);
  int (*send)(const char *data, size_t len);
  int (*recv)(char *buf, size_t len);
} gps_hal_ops_t;

#endif
