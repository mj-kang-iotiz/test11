/**
 * @file gps_role.c
 * @brief GPS 역할 감지 (Base/Rover)
 */

#include "gps_role.h"
#include "stm32f4xx_ll_gpio.h"

#ifndef TAG
#define TAG "GPS_ROLE"
#endif

#include "log.h"

/*===========================================================================
 * 설정 (TODO: 실제 GPIO 핀에 맞게 수정 필요)
 *===========================================================================*/

/* 역할 감지 GPIO 설정 */
#define GPS_ROLE_GPIO_PORT    GPIOB
#define GPS_ROLE_GPIO_PIN     LL_GPIO_PIN_12

/* 핀 상태에 따른 역할 매핑 */
#define GPS_ROLE_PIN_BASE     1   /* HIGH = Base */
#define GPS_ROLE_PIN_ROVER    0   /* LOW = Rover */

/*===========================================================================
 * 내부 변수
 *===========================================================================*/

static gps_role_t g_current_role = GPS_ROLE_UNKNOWN;

/*===========================================================================
 * 공개 API
 *===========================================================================*/

gps_role_t gps_role_detect(void)
{
  /* GPIO 핀 읽기 */
  uint32_t pin_state = LL_GPIO_IsInputPinSet(GPS_ROLE_GPIO_PORT, GPS_ROLE_GPIO_PIN);

  if (pin_state == GPS_ROLE_PIN_BASE) {
    g_current_role = GPS_ROLE_BASE;
  } else {
    g_current_role = GPS_ROLE_ROVER;
  }

  LOG_INFO("GPS 역할 감지: %s (GPIO=%lu)",
           gps_role_to_string(g_current_role), pin_state);

  return g_current_role;
}

gps_role_t gps_role_get(void)
{
  return g_current_role;
}

bool gps_role_is_base(void)
{
  return (g_current_role == GPS_ROLE_BASE);
}

bool gps_role_is_rover(void)
{
  return (g_current_role == GPS_ROLE_ROVER);
}

const char* gps_role_to_string(gps_role_t role)
{
  switch (role) {
    case GPS_ROLE_BASE:   return "Base";
    case GPS_ROLE_ROVER:  return "Rover";
    default:              return "Unknown";
  }
}
