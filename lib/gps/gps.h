#ifndef GPS_H
#define GPS_H

#include "FreeRTOS.h"
#include "gps_types.h"
#include "gps_nmea.h"
#include "gps_unicore.h"
#include "rtcm.h"
#include "ringbuffer.h"
#include "semphr.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef void (*gps_evt_handler)(gps_t *gps, gps_event_t event,
                            gps_protocol_t protocol, gps_msg_t msg);

/**
 * @brief GPS 구조체
 *
 */
typedef struct gps_s {
  /* os variable */
  TaskHandle_t pkt_task; // 패킷 처리 태스크
  SemaphoreHandle_t mutex; // 공유자원 보호
  QueueHandle_t pkt_queue; // 명령어 수신 용

  /* hal */
  const gps_hal_ops_t *ops;

  /* packet data */
  ringbuffer_t rx_buf;
  gps_pasrser_t parser;

  /* cmd */
  SemaphoreHandle_t cmd_sem; // 명령어 처리용 세마포어

  /* status*/
  bool is_alive; // RX 태스크 동작 여부
  bool is_running;

  /* evt handler */
  gps_evt_handler handler;
} gps_t;

void gps_init(gps_t *gps);
void gps_parse_process(gps_t *gps, const void *data, size_t len);
void gps_set_evt_handler(gps_t *gps, gps_evt_handler handler);

#endif
