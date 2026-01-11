/**
 * @file ble_cmd.c
 * @brief BLE 외부 명령어 핸들러 구현
 */

#include "ble_cmd.h"
#include "board_config.h"
#include "flash_params.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef TAG
#define TAG "BLE_CMD"
#endif

#include "log.h"

/*===========================================================================
 * 응답 문자열
 *===========================================================================*/
static const char *ble_resp_str_ok = "OK\n";
static const char *ble_resp_str_err = "+ERROR\n";

#define BLE_RESP_SEND(data) ble_app_send(data, strlen(data))
#define BLE_RESP_SEND_OK() BLE_RESP_SEND(ble_resp_str_ok)
#define BLE_RESP_SEND_ERR() BLE_RESP_SEND(ble_resp_str_err)

/*===========================================================================
 * 명령어 핸들러 타입
 *===========================================================================*/
typedef void (*ble_cmd_handler_t)(ble_instance_t *inst, const char *param);

typedef struct {
    const char *name;
    ble_cmd_handler_t handler;
} ble_cmd_entry_t;

/*===========================================================================
 * 명령어 핸들러 선언
 *===========================================================================*/
static void sd_handler(ble_instance_t *inst, const char *param);
static void sc_handler(ble_instance_t *inst, const char *param);
static void sm_handler(ble_instance_t *inst, const char *param);
static void si_handler(ble_instance_t *inst, const char *param);
static void sp_handler(ble_instance_t *inst, const char *param);
static void sg_handler(ble_instance_t *inst, const char *param);
static void ss_handler(ble_instance_t *inst, const char *param);
static void gd_handler(ble_instance_t *inst, const char *param);
static void gi_handler(ble_instance_t *inst, const char *param);
static void gp_handler(ble_instance_t *inst, const char *param);
static void gg_handler(ble_instance_t *inst, const char *param);
static void rs_handler(ble_instance_t *inst, const char *param);

/*===========================================================================
 * 명령어 테이블
 *===========================================================================*/
static const ble_cmd_entry_t cmd_table[] = {
    {"SD+", sd_handler},    /* 디바이스 이름 설정 */
    {"SC+", sc_handler},    /* Caster 설정 */
    {"SM+", sm_handler},    /* Mountpoint 설정 */
    {"SI+", si_handler},    /* ID 설정 */
    {"SP+", sp_handler},    /* Password 설정 */
    {"SG+", sg_handler},    /* GPS 위치 설정 */
    {"SS", ss_handler},     /* 설정 저장 */
    {"GD", gd_handler},     /* 디바이스 이름 조회 */
    {"GI", gi_handler},     /* ID 조회 */
    {"GP", gp_handler},     /* Password 조회 */
    {"GG", gg_handler},     /* GPS 위치 조회 */
    {"RS", rs_handler},     /* 리셋 */
    {NULL, NULL}
};

/*===========================================================================
 * 앱 명령어 핸들러
 *===========================================================================*/

void ble_app_cmd_handler(ble_instance_t *inst, const char *cmd)
{
    if (!inst || !cmd) {
        return;
    }

    LOG_DEBUG("앱 명령어 처리: %s", cmd);

    for (int i = 0; cmd_table[i].name != NULL; i++) {
        size_t name_len = strlen(cmd_table[i].name);

        if (strncmp(cmd, cmd_table[i].name, name_len) == 0) {
            cmd_table[i].handler(inst, cmd + name_len);
            return;
        }
    }

    LOG_WARN("알 수 없는 명령어: %s", cmd);
}

/*===========================================================================
 * 설정 명령어 핸들러
 *===========================================================================*/

/**
 * @brief 디바이스 이름 설정 (SD+<name>)
 */
static void sd_handler(ble_instance_t *inst, const char *param)
{
    user_params_t *params = flash_params_get_current();
    char device_name[32];
    char buf[100];

    sscanf(param, "%[^\n\r]", device_name);

    /* BLE 모듈 디바이스 이름 설정 */
    if (ble_app_set_device_name(device_name, 2000)) {
        /* Flash에 저장 */
        flash_params_set_ble_device_name(device_name);
        flash_params_erase();
        flash_params_write(params);

        snprintf(buf, sizeof(buf), "Set %s Complete\n\r", device_name);
        ble_app_send(buf, strlen(buf));
    } else {
        BLE_RESP_SEND_ERR();
    }
}

/**
 * @brief Caster 설정 (SC+<host>:<port>)
 */
static void sc_handler(ble_instance_t *inst, const char *param)
{
    user_params_t *params = flash_params_get_current();
    char buf[100];
    char caster_host[64] = {0};
    char caster_port[8] = {0};

    /* host:port 분리 */
    char temp[128];
    strncpy(temp, param, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';

    char *port_sep = strrchr(temp, ':');
    if (port_sep) {
        *port_sep = '\0';
        strncpy(caster_host, temp, sizeof(caster_host) - 1);

        char *end = strchr(port_sep + 1, '\n');
        if (end) *end = '\0';
        end = strchr(port_sep + 1, '\r');
        if (end) *end = '\0';
        strncpy(caster_port, port_sep + 1, sizeof(caster_port) - 1);
    } else {
        BLE_RESP_SEND_ERR();
        return;
    }

    flash_params_set_ntrip_url(caster_host);
    flash_params_set_ntrip_port(caster_port);

    snprintf(buf, sizeof(buf), "Set %s:%s Complete\n\r",
             params->ntrip_url, params->ntrip_port);
    ble_app_send(buf, strlen(buf));
}

/**
 * @brief Mountpoint 설정 (SM+<mountpoint>)
 */
static void sm_handler(ble_instance_t *inst, const char *param)
{
    user_params_t *params = flash_params_get_current();
    char buf[100];
    char mountpoint[32];

    sscanf(param, "%[^\n\r]", mountpoint);

    flash_params_set_ntrip_mountpoint(mountpoint);

    snprintf(buf, sizeof(buf), "Set %s Complete\n\r", params->ntrip_mountpoint);
    ble_app_send(buf, strlen(buf));
}

/**
 * @brief ID 설정 (SI+<id>)
 */
static void si_handler(ble_instance_t *inst, const char *param)
{
    user_params_t *params = flash_params_get_current();
    char buf[100];
    char ntrip_id[32];

    sscanf(param, "%[^\n\r]", ntrip_id);

    flash_params_set_ntrip_id(ntrip_id);

    snprintf(buf, sizeof(buf), "Set %s Complete\n\r", params->ntrip_id);
    ble_app_send(buf, strlen(buf));
}

/**
 * @brief Password 설정 (SP+<password>)
 */
static void sp_handler(ble_instance_t *inst, const char *param)
{
    user_params_t *params = flash_params_get_current();
    char buf[100];
    char password[32];

    sscanf(param, "%[^\n\r]", password);

    flash_params_set_ntrip_pw(password);

    snprintf(buf, sizeof(buf), "Set %s Complete\n\r", params->ntrip_pw);
    ble_app_send(buf, strlen(buf));
}

/**
 * @brief GPS 위치 설정 (SG+<flag>,<lat>,<ns>,<lon>,<ew>,<alt>)
 */
static void sg_handler(ble_instance_t *inst, const char *param)
{
    user_params_t *params = flash_params_get_current();
    char buf[100];
    uint32_t flag;
    char lat_str[20], lon_str[20], alt_str[16];
    char ns, ew;

    int ret = sscanf(param, "%lu,%[^,],%c,%[^,],%c,%[^\n\r]",
                     &flag, lat_str, &ns, lon_str, &ew, alt_str);

    if (ret != 6) {
        BLE_RESP_SEND_ERR();
        return;
    }

    /* 숫자 변환 검증 */
    char *end;
    double lat = strtod(lat_str, &end);
    if (end == lat_str) {
        BLE_RESP_SEND_ERR();
        return;
    }

    double lon = strtod(lon_str, &end);
    if (end == lon_str) {
        BLE_RESP_SEND_ERR();
        return;
    }

    double alt = strtod(alt_str, &end);
    if (end == alt_str) {
        BLE_RESP_SEND_ERR();
        return;
    }

    (void)lat; (void)lon; (void)alt;  /* 미사용 경고 억제 */

    flash_params_set_manual_position(flag, lat_str, lon_str, alt_str);

    snprintf(buf, sizeof(buf), "Set %lu,%s,%s,%s\n\r",
             (unsigned long)params->use_manual_position,
             params->lat, params->lon, params->alt);
    ble_app_send(buf, strlen(buf));
}

/**
 * @brief 설정 저장 (SS)
 */
static void ss_handler(ble_instance_t *inst, const char *param)
{
    (void)param;

    user_params_t *params = flash_params_get_current();
    flash_params_erase();
    flash_params_write(params);

    ble_app_send("Save Complete!\n\r", strlen("Save Complete!\n\r"));
    vTaskDelay(pdMS_TO_TICKS(100));
    NVIC_SystemReset();
}

/*===========================================================================
 * 조회 명령어 핸들러
 *===========================================================================*/

/**
 * @brief 디바이스 이름 조회 (GD)
 */
static void gd_handler(ble_instance_t *inst, const char *param)
{
    (void)param;

    user_params_t *params = flash_params_get_current();
    char buf[64];

    snprintf(buf, sizeof(buf), "Get %s\n\r", params->ble_device_name);
    ble_app_send(buf, strlen(buf));
}

/**
 * @brief ID 조회 (GI)
 */
static void gi_handler(ble_instance_t *inst, const char *param)
{
    (void)param;

    user_params_t *params = flash_params_get_current();
    char buf[64];

    snprintf(buf, sizeof(buf), "Get %s\n\r", params->ntrip_id);
    ble_app_send(buf, strlen(buf));
}

/**
 * @brief Password 조회 (GP)
 */
static void gp_handler(ble_instance_t *inst, const char *param)
{
    (void)param;

    user_params_t *params = flash_params_get_current();
    char buf[64];

    snprintf(buf, sizeof(buf), "Get %s\n\r", params->ntrip_pw);
    ble_app_send(buf, strlen(buf));
}

/**
 * @brief GPS 위치 조회 (GG)
 */
static void gg_handler(ble_instance_t *inst, const char *param)
{
    (void)param;

    user_params_t *params = flash_params_get_current();
    char buf[100];

    snprintf(buf, sizeof(buf), "Get %s,%s,%s\n\r",
             params->lat, params->lon, params->alt);
    ble_app_send(buf, strlen(buf));
}

/**
 * @brief 리셋 (RS)
 */
static void rs_handler(ble_instance_t *inst, const char *param)
{
    (void)param;

    ble_app_send("Device Reset\n\r", strlen("Device Reset\n\r"));
    vTaskDelay(pdMS_TO_TICKS(100));
    NVIC_SystemReset();
}
