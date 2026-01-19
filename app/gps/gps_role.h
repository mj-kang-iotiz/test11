#ifndef GPS_ROLE_H
#define GPS_ROLE_H

/**
 * @file gps_role.h
 * @brief GPS 역할 감지 (Base/Rover)
 *
 * GPIO 핀을 읽어서 런타임에 Base/Rover 역할을 판단합니다.
 */

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief GPS 역할 타입
 */
typedef enum {
  GPS_ROLE_UNKNOWN = 0,   /**< 미확인 */
  GPS_ROLE_BASE,          /**< Base Station */
  GPS_ROLE_ROVER          /**< Rover */
} gps_role_t;

/**
 * @brief GPIO로 역할 감지
 *
 * 지정된 GPIO 핀을 읽어서 Base/Rover 역할을 판단합니다.
 * - HIGH: Base
 * - LOW: Rover
 *
 * @return gps_role_t 감지된 역할
 */
gps_role_t gps_role_detect(void);

/**
 * @brief 현재 역할 조회
 *
 * 캐시된 역할을 반환합니다. gps_role_detect()가 한 번도
 * 호출되지 않았으면 GPS_ROLE_UNKNOWN을 반환합니다.
 *
 * @return gps_role_t 현재 역할
 */
gps_role_t gps_role_get(void);

/**
 * @brief 역할이 Base인지 확인
 * @return true: Base, false: Rover 또는 Unknown
 */
bool gps_role_is_base(void);

/**
 * @brief 역할이 Rover인지 확인
 * @return true: Rover, false: Base 또는 Unknown
 */
bool gps_role_is_rover(void);

/**
 * @brief 역할 이름 문자열 반환
 * @param role 역할
 * @return 역할 이름 ("Base", "Rover", "Unknown")
 */
const char* gps_role_to_string(gps_role_t role);

#endif /* GPS_ROLE_H */
