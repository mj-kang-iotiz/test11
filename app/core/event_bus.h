#ifndef EVENT_BUS_H
#define EVENT_BUS_H

#include <stdbool.h>
#include <stdint.h>
#include "gps_nmea.h"

/*===========================================================================
 * Event Types (Central Definition)
 *===========================================================================*/
typedef enum {
    /* GPS Events (0x0100 ~ 0x01FF) */
    EVENT_GPS_FIX_CHANGED = 0x0100,
    EVENT_GPS_GGA_UPDATE,

    /* NTRIP Events (0x0200 ~ 0x02FF) */
    EVENT_NTRIP_CONNECTED = 0x0200,
    EVENT_NTRIP_DISCONNECTED,

    /* LoRa Events (0x0300 ~ 0x03FF) */
    EVENT_RTCM_FOR_LORA = 0x0300,

    /* System Events (0x0400 ~ 0x04FF) */
    EVENT_SYSTEM_SHUTDOWN = 0x0400,

    EVENT_TYPE_MAX
} event_type_t;

/*===========================================================================
 * Event Data Structures
 *===========================================================================*/
typedef struct {
    gps_fix_t fix;
    uint8_t gps_id;
} event_gps_fix_data_t;

typedef struct {
    double lat;
    double lon;
    double alt;
    uint8_t gps_id;
} event_gps_gga_data_t;

typedef struct {
    bool connected;
} event_ntrip_data_t;

typedef struct {
    uint8_t gps_id;
} event_rtcm_data_t;

/**
 * @brief Event structure
 */
typedef struct {
    event_type_t type;
    union {
        event_gps_fix_data_t gps_fix;
        event_gps_gga_data_t gps_gga;
        event_ntrip_data_t ntrip;
        event_rtcm_data_t rtcm;
    } data;
} event_t;

/*===========================================================================
 * Configuration
 *===========================================================================*/
#define EVENT_BUS_MAX_SUBSCRIBERS 16

/*===========================================================================
 * Callback Type
 *===========================================================================*/
typedef void (*event_handler_t)(const event_t *event, void *user_data);

/*===========================================================================
 * API
 *===========================================================================*/

/**
 * @brief Initialize event bus
 */
void event_bus_init(void);

/**
 * @brief Subscribe to an event type
 *
 * @param type Event type to subscribe
 * @param handler Callback function
 * @param user_data User data passed to callback (can be NULL)
 * @return true Success
 * @return false Failed (max subscribers reached)
 */
bool event_bus_subscribe(event_type_t type, event_handler_t handler, void *user_data);

/**
 * @brief Unsubscribe from an event type
 *
 * @param type Event type
 * @param handler Callback function to remove
 */
void event_bus_unsubscribe(event_type_t type, event_handler_t handler);

/**
 * @brief Publish an event (async - enqueues and returns immediately)
 *
 * Event is placed in queue and processed by dispatcher task.
 * Publisher is NOT blocked by handler execution.
 *
 * @param event Event to publish
 */
void event_bus_publish(const event_t *event);

#endif /* EVENT_BUS_H */
