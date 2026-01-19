/**
 * @file gps_role.c
 * @brief GPS 역할 감지 (Base/Rover)
 *
 * 현재는 빌드 타임에 역할이 결정됩니다.
 * 나중에 GPIO 핀을 읽어서 런타임에 판단하도록 변경 예정입니다.
 */

#include "gps_role.h"
#include "board_config.h"

#ifndef TAG
#define TAG "GPS_ROLE"
#endif

#include "log.h"

/*===========================================================================
 * 설정
 *
 * 역할 감지 방법:
 * 1. 빌드 타임: board_config.h의 LORA_MODE 매크로 사용 (현재)
 * 2. 런타임 GPIO: 추후 구현 예정 (GPS_ROLE_USE_GPIO 정의 시)
 *===========================================================================*/

/* GPIO 기반 역할 감지 활성화 (추후 사용) */
// #define GPS_ROLE_USE_GPIO

#ifdef GPS_ROLE_USE_GPIO
#include "stm32f4xx_ll_gpio.h"
#define GPS_ROLE_GPIO_PORT    GPIOB
#define GPS_ROLE_GPIO_PIN     LL_GPIO_PIN_12
#define GPS_ROLE_PIN_BASE     1   /* HIGH = Base */
#define GPS_ROLE_PIN_ROVER    0   /* LOW = Rover */
#endif

/*===========================================================================
 * 내부 변수
 *===========================================================================*/

static gps_role_t g_current_role = GPS_ROLE_UNKNOWN;

/*===========================================================================
 * 공개 API
 *===========================================================================*/

gps_role_t gps_role_detect(void)
{
#ifdef GPS_ROLE_USE_GPIO
  /* GPIO 핀 읽기 (추후 사용) */
  uint32_t pin_state = LL_GPIO_IsInputPinSet(GPS_ROLE_GPIO_PORT, GPS_ROLE_GPIO_PIN);

  if (pin_state == GPS_ROLE_PIN_BASE) {
    g_current_role = GPS_ROLE_BASE;
  } else {
    g_current_role = GPS_ROLE_ROVER;
  }

  LOG_INFO("GPS 역할 감지 (GPIO): %s (pin=%lu)",
           gps_role_to_string(g_current_role), pin_state);
#else
  /* 빌드 타임 결정 (board_config.h 매크로 사용) */
  #if (LORA_MODE == LORA_MODE_BASE)
    g_current_role = GPS_ROLE_BASE;
  #elif (LORA_MODE == LORA_MODE_ROVER)
    g_current_role = GPS_ROLE_ROVER;
  #else
    g_current_role = GPS_ROLE_UNKNOWN;
  #endif

  LOG_INFO("GPS 역할 (빌드 타임): %s", gps_role_to_string(g_current_role));
#endif

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
