#include "event_bus.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>

#ifndef TAG
#define TAG "EVENT_BUS"
#endif

#include "log.h"

/*===========================================================================
 * Internal Types
 *===========================================================================*/
typedef struct {
    event_type_t type;
    event_handler_t handler;
    void *user_data;
    bool active;
} subscriber_t;

/*===========================================================================
 * Internal Variables
 *===========================================================================*/
static subscriber_t subscribers[EVENT_BUS_MAX_SUBSCRIBERS];
static uint8_t subscriber_count = 0;
static SemaphoreHandle_t bus_mutex = NULL;

/* ISR deferred event queue */
#define EVENT_QUEUE_SIZE 10
static QueueHandle_t deferred_queue = NULL;

/*===========================================================================
 * Implementation
 *===========================================================================*/

void event_bus_init(void) {
    memset(subscribers, 0, sizeof(subscribers));
    subscriber_count = 0;

    if (bus_mutex == NULL) {
        bus_mutex = xSemaphoreCreateMutex();
        if (bus_mutex == NULL) {
            LOG_ERR("Failed to create mutex");
        }
    }

    if (deferred_queue == NULL) {
        deferred_queue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(event_t));
        if (deferred_queue == NULL) {
            LOG_ERR("Failed to create deferred queue");
        }
    }

    LOG_INFO("Event bus initialized");
}

bool event_bus_subscribe(event_type_t type, event_handler_t handler, void *user_data) {
    if (handler == NULL) {
        return false;
    }

    if (bus_mutex != NULL) {
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
    }

    /* Check for duplicate */
    for (uint8_t i = 0; i < subscriber_count; i++) {
        if (subscribers[i].active &&
            subscribers[i].type == type &&
            subscribers[i].handler == handler) {
            if (bus_mutex != NULL) {
                xSemaphoreGive(bus_mutex);
            }
            LOG_WARN("Already subscribed");
            return false;
        }
    }

    /* Find empty slot or add new */
    int slot = -1;
    for (uint8_t i = 0; i < subscriber_count; i++) {
        if (!subscribers[i].active) {
            slot = i;
            break;
        }
    }

    if (slot < 0) {
        if (subscriber_count >= EVENT_BUS_MAX_SUBSCRIBERS) {
            if (bus_mutex != NULL) {
                xSemaphoreGive(bus_mutex);
            }
            LOG_ERR("Max subscribers reached");
            return false;
        }
        slot = subscriber_count++;
    }

    subscribers[slot].type = type;
    subscribers[slot].handler = handler;
    subscribers[slot].user_data = user_data;
    subscribers[slot].active = true;

    if (bus_mutex != NULL) {
        xSemaphoreGive(bus_mutex);
    }

    LOG_DEBUG("Subscribed to event 0x%04X", type);
    return true;
}

void event_bus_unsubscribe(event_type_t type, event_handler_t handler) {
    if (bus_mutex != NULL) {
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
    }

    for (uint8_t i = 0; i < subscriber_count; i++) {
        if (subscribers[i].active &&
            subscribers[i].type == type &&
            subscribers[i].handler == handler) {
            subscribers[i].active = false;
            LOG_DEBUG("Unsubscribed from event 0x%04X", type);
            break;
        }
    }

    if (bus_mutex != NULL) {
        xSemaphoreGive(bus_mutex);
    }
}

void event_bus_publish(const event_t *event) {
    if (event == NULL) {
        return;
    }

    if (bus_mutex != NULL) {
        xSemaphoreTake(bus_mutex, portMAX_DELAY);
    }

    for (uint8_t i = 0; i < subscriber_count; i++) {
        if (subscribers[i].active && subscribers[i].type == event->type) {
            /* Release mutex before calling handler to prevent deadlock */
            if (bus_mutex != NULL) {
                xSemaphoreGive(bus_mutex);
            }

            subscribers[i].handler(event, subscribers[i].user_data);

            if (bus_mutex != NULL) {
                xSemaphoreTake(bus_mutex, portMAX_DELAY);
            }
        }
    }

    if (bus_mutex != NULL) {
        xSemaphoreGive(bus_mutex);
    }
}

bool event_bus_publish_from_isr(const event_t *event) {
    if (event == NULL || deferred_queue == NULL) {
        return false;
    }

    BaseType_t higher_priority_woken = pdFALSE;
    BaseType_t result = xQueueSendFromISR(deferred_queue, event, &higher_priority_woken);

    portYIELD_FROM_ISR(higher_priority_woken);

    return (result == pdTRUE);
}

void event_bus_process(void) {
    if (deferred_queue == NULL) {
        return;
    }

    event_t event;
    while (xQueueReceive(deferred_queue, &event, 0) == pdTRUE) {
        event_bus_publish(&event);
    }
}
