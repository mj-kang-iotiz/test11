#ifndef BASE_AUTO_FIX_H
#define BASE_AUTO_FIX_H

#include <stdbool.h>
#include <stdint.h>
#include "gps_nmea.h"

  typedef enum
  {
    BASE_AUTO_FIX_DISABLED,     // 비활성화 (일반 Base 모드)
    BASE_AUTO_FIX_INIT,         // 초기화
    BASE_AUTO_FIX_NTRIP_WAIT,   // NTRIP 연결 대기
    BASE_AUTO_FIX_WAIT_RTK_FIX, // RTK Fix 대기
    BASE_AUTO_FIX_AVERAGING,    // RTK Fix 후 1분간 평균 계산
    BASE_AUTO_FIX_SWITCHING,    // Base Fixed 모드 전환 중
    BASE_AUTO_FIX_COMPLETED,    // 완료 (NTRIP/LTE 종료됨)
    BASE_AUTO_FIX_FAILED        // 실패
  } base_auto_fix_state_t;

  /**
   * @brief 좌표 샘플
   */
  typedef struct
  {
    double lat; // 위도 (도)
    double lon; // 경도 (도)
    double alt; // 고도 (m)
  } coord_sample_t;

  /**

   * @brief 평균 좌표 결과

   */

  typedef struct
  {
    double lat; // 평균 위도
    double lon; // 평균 경도
    double alt; // 평균 고도
    uint32_t count; // 유효 샘플 수
    uint32_t rejected; // 이상치 제거 샘플 수
  } coord_average_t;

  /**

   * @brief Base Auto-Fix 모듈 초기화

   * @param gps_id GPS ID (GPS_ID_BASE 또는 GPS_ID_ROVER)

   * @return true 성공, false 실패

   */

  bool base_auto_fix_init(uint8_t gps_id);

  /**

   * @brief Base Auto-Fix 모드 시작

   * @return true 성공, false 실패

   */

  bool base_auto_fix_start(void);

  /**
   * @brief Base Auto-Fix 모드 중지
   */
  void base_auto_fix_stop(void);

  /**
   * @brief Base Auto-Fix 모듈 해제
   *
   * 모든 리소스(타이머, 큐, 태스크)를 해제합니다.
   */
  void base_auto_fix_deinit(void);

  /**

   * @brief 현재 상태 조회

   * @return base_auto_fix_state_t

   */

  base_auto_fix_state_t base_auto_fix_get_state(void);

  /**
   * @brief 평균 좌표 조회
   * @param result 결과 저장 포인터
   * @return true 성공, false 실패 (아직 계산 안 됨)
   */

  bool base_auto_fix_get_average_coord(coord_average_t *result);


#endif // BASE_AUTO_FIX_H
