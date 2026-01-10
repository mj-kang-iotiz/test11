#ifndef GPS_H
#define GPS_H

#include "FreeRTOS.h"
#include "gps_types.h"
#include "gps_nmea.h"
#include "gps_unicore.h"
#include "rtcm.h"
#include "semphr.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define GPS_PAYLOAD_SIZE 1029

typedef struct {
  int (*init)(void);
  int (*start)(void);
  int (*stop)(void);
  int (*reset)(void);
  int (*send)(const char *data, size_t len);
  int (*recv)(char *buf, size_t len);
} gps_hal_ops_t;

typedef void (*gps_evt_handler)(gps_t *gps, gps_event_t event,
                            gps_protocol_t protocol, gps_msg_t msg);

/**
 * @brief GPS 구조체
 *
 */
typedef struct gps_s {
  /* state */
  gps_protocol_t protocol;

  /* os variable */
  TaskHandle_t pkt_task;
  SemaphoreHandle_t mutex;

  /* hal */
  const gps_hal_ops_t *ops;

  /* parse */
  gps_parse_state_t state;
  char payload[GPS_PAYLOAD_SIZE];
  uint32_t pos;

  /* protocol header */
  gps_nmea_parser_t nmea;
  gps_unicore_parser_t unicore;
  gps_unicore_bin_parser_t unicore_bin;
  gps_rtcm_parser_t rtcm;

  /* info */
  gps_nmea_data_t nmea_data;
  gps_unicore_bin_data_t unicore_bin_data;

  /* evt handler */
  gps_evt_handler handler;
} gps_t;

void gps_init(gps_t *gps);
void gps_parse_process(gps_t *gps, const void *data, size_t len);
void gps_set_evt_handler(gps_t *gps, gps_evt_handler handler);

/* internal */
void _gps_gga_raw_add(gps_t *gps, char ch);

#endif
