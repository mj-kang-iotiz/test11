#ifndef BLE_CMD_H
#define BLE_CMD_H

/**
 * @file ble_cmd.h
 * @brief BLE 외부 명령어 핸들러
 *
 * 외부 PC에서 BLE를 통해 수신되는 명령어 처리
 * - SD+: 디바이스 이름 설정
 * - SC+: Caster 설정
 * - SM+: Mountpoint 설정
 * - SI+: ID 설정
 * - SP+: Password 설정
 * - SG+: GPS 위치 설정
 * - SS: 설정 저장
 * - GD: 디바이스 이름 조회
 * - GI: ID 조회
 * - GP: Password 조회
 * - GG: GPS 위치 조회
 * - RS: 리셋
 */

#include "ble_app.h"

/**
 * @brief 앱 명령어 핸들러
 *
 * 파싱된 앱 명령어를 처리
 *
 * @param inst BLE 인스턴스
 * @param cmd 명령어 문자열 (예: "SD+MyDevice")
 */
void ble_app_cmd_handler(ble_instance_t *inst, const char *cmd);

#endif /* BLE_CMD_H */
