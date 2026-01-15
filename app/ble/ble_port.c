/**
 * @file ble_port.c
 * @brief BLE UART 포트 구현 (STM32 HAL)
 */

#include "ble_port.h"
#include "ble_app.h"
#include "ble.h"
#include "board_config.h"
#include "board_type.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "flash_params.h"

#include <string.h>
#include <stdio.h>

#ifndef TAG
#define TAG "BLE_PORT"
#endif

#include "log.h"

/*===========================================================================
 * 내부 함수 선언
 *===========================================================================*/
static int ble_uart5_hw_init(void);
static int ble_uart5_comm_start(void);
static int ble_uart5_send(const char *data, size_t len);
static int ble_set_at_cmd_mode(void);
static int ble_set_bypass_mode(void);
static int ble_uart5_change_baudrate(uint32_t baudrate);
static int ble_configure_module(void);

/*===========================================================================
 * 정적 변수
 *===========================================================================*/
#define BLE_PORT_UART UART5
#define BLE_PORT_UART_DMA DMA1
#define BLE_PORT_UART_DMA_STREAM LL_DMA_STREAM_0

static char ble_recv_buf[BLE_RX_BUF_SIZE];
static QueueHandle_t ble_rx_queue = NULL;
static ble_t *ble_handle = NULL;
static size_t ble_rx_old_pos = 0;

/*===========================================================================
 * HAL ops 정의
 *===========================================================================*/
static const ble_hal_ops_t ble_uart5_ops = {
    .init = ble_uart5_hw_init,
    .reset = NULL,
    .start = ble_uart5_comm_start,
    .stop = NULL,
    .send = ble_uart5_send,
    .recv = NULL,
    .at_mode = ble_set_at_cmd_mode,
    .bypass_mode = ble_set_bypass_mode,
};

/*===========================================================================
 * 포트 API 구현
 *===========================================================================*/

int ble_port_init_instance(ble_t *ble)
{
    const board_config_t *config = board_get_config();

    LOG_INFO("BLE 포트 초기화 시작 (보드: %d)", config->board);

    if (!config->use_ble) {
        LOG_ERR("BLE 포트 초기화 실패: BLE 비활성화");
        return -1;
    }

    /* BLE 구조체 초기화 (lib/ble) */
    if (!ble_init(ble, &ble_uart5_ops)) {
        LOG_ERR("BLE 구조체 초기화 실패");
        return -1;
    }

    /* HAL 초기화 */
    if (ble->ops->init) {
        ble->ops->init();
    }

    LOG_INFO("BLE 포트 초기화 완료");
    return 0;
}

void ble_port_start(ble_t *ble)
{
    if (!ble || !ble->ops || !ble->ops->start) {
        LOG_ERR("BLE start failed: invalid handle or ops");
        return;
    }

    ble->ops->start();
}

void ble_port_stop(ble_t *ble)
{
    if (!ble || !ble->ops || !ble->ops->stop) {
        return;
    }

    ble->ops->stop();
}

uint32_t ble_port_get_rx_pos(void)
{
    uint32_t pos = sizeof(ble_recv_buf) - LL_DMA_GetDataLength(BLE_PORT_UART_DMA, BLE_PORT_UART_DMA_STREAM);
    return pos;
}

char *ble_port_get_recv_buf(void)
{
    return ble_recv_buf;
}

void ble_port_set_queue(QueueHandle_t queue)
{
    ble_rx_queue = queue;
}

void ble_port_set_ble_handle(ble_t *ble)
{
    ble_handle = ble;
}

/*===========================================================================
 * UART/DMA 초기화
 *===========================================================================*/

static void ble_uart5_dma_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
}

static void ble_uart5_gpio_init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

    /* UART5: PC12 (TX), PD2 (RX) */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_8;
    LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
    LL_GPIO_Init(GPIOD, &GPIO_InitStruct);
}

static void ble_uart5_uart_init(void)
{
    LL_USART_InitTypeDef USART_InitStruct = {0};

    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);
    ble_uart5_gpio_init();

    /* DMA 설정 */
    LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_0, LL_DMA_CHANNEL_4);
    LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);
    LL_DMA_SetMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_CIRCULAR);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_BYTE);
    LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_0);

    NVIC_SetPriority(UART5_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));

    /* USART 설정 (9600 bps 기본) */
    USART_InitStruct.BaudRate = 9600;
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    LL_USART_Init(UART5, &USART_InitStruct);
    LL_USART_ConfigAsyncMode(UART5);
}

static int ble_uart5_hw_init(void)
{
    ble_uart5_dma_init();
    ble_uart5_uart_init();
    ble_set_bypass_mode();

    LOG_INFO("BLE 하드웨어 초기화 완료 (UART5 @ 9600 bps)");
    return 0;
}

/*===========================================================================
 * 모듈 설정 (태스크 컨텍스트)
 *===========================================================================*/

static int ble_uart5_comm_start(void)
{
    /* 모듈 설정 (baudrate 변경 등) */
    ble_configure_module();

    /* DMA 설정 */
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)&UART5->DR);
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)ble_recv_buf);
    LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, sizeof(ble_recv_buf));

    /* DMA 에러 인터럽트 */
    LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_0);
    LL_DMA_EnableIT_FE(DMA1, LL_DMA_STREAM_0);
    LL_DMA_EnableIT_DME(DMA1, LL_DMA_STREAM_0);

    /* 인터럽트 활성화 */
    NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    NVIC_EnableIRQ(UART5_IRQn);

    /* USART 인터럽트 및 DMA 활성화 */
    LL_USART_EnableIT_IDLE(UART5);
    LL_USART_EnableIT_PE(UART5);
    LL_USART_EnableIT_ERROR(UART5);
    LL_USART_EnableDMAReq_RX(UART5);

    /* DMA 및 USART 활성화 */
    LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
    LL_USART_Enable(UART5);

    LOG_INFO("BLE UART5 DMA 통신 시작");
    return 0;
}

static int ble_configure_module(void)
{
    uint32_t current_baudrate = 9600;
    char buffer[64];

    LOG_INFO("BLE 모듈 설정 시작");

    LL_USART_Enable(UART5);
    ble_set_bypass_mode();
    vTaskDelay(pdMS_TO_TICKS(100));
    ble_set_at_cmd_mode();
    vTaskDelay(pdMS_TO_TICKS(300));

    /* 부팅 메시지 버퍼 클리어 */
    while (LL_USART_IsActiveFlag_RXNE(UART5)) {
        (void)LL_USART_ReceiveData8(UART5);
    }

    /* 9600 bps 테스트 */
    LOG_INFO("9600 bps 테스트...");
    ble_uart5_send("AT\r", 3);
    int len = ble_uart5_recv_line_poll(buffer, sizeof(buffer), 200);
    ble_uart5_send("AT\r", 3);
    len = ble_uart5_recv_line_poll(buffer, sizeof(buffer), 200);

    if (len > 0 && strncmp(buffer, "+OK", 3) == 0) {
        LOG_INFO("9600 bps 응답 OK, 115200 bps로 변경...");
        vTaskDelay(pdMS_TO_TICKS(50));

        /* UART 속도 변경 명령 */
        ble_uart5_send("AT+UART=115200\r", 15);
        len = ble_uart5_recv_line_poll(buffer, sizeof(buffer), 500);

        if (len > 0 && strstr(buffer, "+OK") != NULL) {
            LOG_INFO("모듈 UART 변경 완료, MCU 속도 변경...");
            ble_uart5_change_baudrate(115200);
            current_baudrate = 115200;

            /* 모듈 재시작 대기 */
            vTaskDelay(pdMS_TO_TICKS(2000));

            /* +READY 대기 */
            len = ble_uart5_recv_line_poll(buffer, sizeof(buffer), 500);
            if (len > 0) {
                LOG_INFO("모듈 응답: %s", buffer);
            }
        } else {
            LOG_WARN("UART 변경 실패, 9600 유지");
        }
    } else {
        LOG_INFO("9600 응답 없음, 115200 가정...");
        ble_uart5_change_baudrate(115200);
        current_baudrate = 115200;

        ble_uart5_send("AT\r", 3);
        len = ble_uart5_recv_line_poll(buffer, sizeof(buffer), 200);
    }

    LOG_INFO("BLE 모듈 %lu bps 설정됨", current_baudrate);

    /* 디바이스 이름 확인/설정 */
    vTaskDelay(pdMS_TO_TICKS(500));
    ble_uart5_send("AT\r", 3);
    len = ble_uart5_recv_line_poll(buffer, sizeof(buffer), 200);

    vTaskDelay(pdMS_TO_TICKS(300));
    ble_uart5_send("AT+MANUF?\r", 10);
    len = ble_uart5_recv_line_poll(buffer, sizeof(buffer), 500);

    if (len > 0 && strncmp(buffer, "+ERR", 4) != 0) {
        LOG_INFO("BLE 디바이스 이름: %s", buffer);

        user_params_t *params = flash_params_get_current();
        if (strncmp(params->ble_device_name, buffer, strlen(params->ble_device_name)) != 0) {
            char cmd[100];
            snprintf(cmd, sizeof(cmd), "AT+MANUF=%s\r", params->ble_device_name);
            ble_uart5_send(cmd, strlen(cmd));
            len = ble_uart5_recv_line_poll(buffer, sizeof(buffer), 500);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    } else {
        LOG_WARN("디바이스 이름 읽기 실패");
    }

    LL_USART_Disable(UART5);
    ble_set_bypass_mode();

    return 0;
}

/*===========================================================================
 * UART 송수신
 *===========================================================================*/

static int ble_uart5_send(const char *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        while (!LL_USART_IsActiveFlag_TXE(UART5));
        LL_USART_TransmitData8(UART5, data[i]);
    }
    while (!LL_USART_IsActiveFlag_TC(UART5));
    return 0;
}

static int ble_uart5_recv_poll(uint8_t *byte, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    while (!LL_USART_IsActiveFlag_RXNE(UART5)) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return -1;
        }
        if ((HAL_GetTick() - start) > 10) {
            taskYIELD();
        }
    }

    *byte = LL_USART_ReceiveData8(UART5);
    return 0;
}

int ble_uart5_recv_line_poll(char *buf, size_t buf_size, uint32_t timeout_ms)
{
    size_t pos = 0;
    uint32_t start = HAL_GetTick();

    while (pos < buf_size - 1) {
        uint8_t byte;
        uint32_t elapsed = HAL_GetTick() - start;

        if (elapsed > timeout_ms) {
            buf[pos] = '\0';
            return -1;
        }

        if (ble_uart5_recv_poll(&byte, timeout_ms - elapsed) != 0) {
            buf[pos] = '\0';
            return -1;
        }

        buf[pos++] = (char)byte;

        if (byte == '\r') {
            buf[pos] = '\0';
            return pos;
        }
    }

    buf[buf_size - 1] = '\0';
    return pos;
}

static int ble_uart5_change_baudrate(uint32_t baudrate)
{
    LL_USART_Disable(UART5);
    LL_USART_SetBaudRate(UART5, HAL_RCC_GetPCLK1Freq(), LL_USART_OVERSAMPLING_16, baudrate);
    LL_USART_Enable(UART5);

    LOG_INFO("UART5 baudrate 변경: %lu", baudrate);
    return 0;
}

/*===========================================================================
 * 모드 전환
 *===========================================================================*/

static int ble_set_at_cmd_mode(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
    return 0;
}

static int ble_set_bypass_mode(void)
{
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
    return 0;
}

/*===========================================================================
 * 인터럽트 핸들러
 *===========================================================================*/

#if defined(BOARD_TYPE_BASE_UNICORE) || defined(BOARD_TYPE_BASE_UBLOX)

void UART5_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (LL_USART_IsActiveFlag_IDLE(UART5)) {
        LL_USART_ClearFlag_IDLE(UART5);

        if (ble_handle != NULL) {
            /* DMA 현재 위치 계산 */
            size_t pos = sizeof(ble_recv_buf) - LL_DMA_GetDataLength(BLE_PORT_UART_DMA, BLE_PORT_UART_DMA_STREAM);

            if (pos != ble_rx_old_pos) {
                if (pos > ble_rx_old_pos) {
                    /* 선형 데이터 */
                    size_t len = pos - ble_rx_old_pos;
                    ble_rx_write(ble_handle, (const uint8_t *)&ble_recv_buf[ble_rx_old_pos], len);
                } else {
                    /* 래핑된 데이터 (circular buffer wrap-around) */
                    size_t len1 = sizeof(ble_recv_buf) - ble_rx_old_pos;
                    ble_rx_write(ble_handle, (const uint8_t *)&ble_recv_buf[ble_rx_old_pos], len1);

                    if (pos > 0) {
                        ble_rx_write(ble_handle, (const uint8_t *)ble_recv_buf, pos);
                    }
                }

                ble_rx_old_pos = pos;
            }
        }

        /* 태스크에 처리 신호 전송 */
        if (ble_rx_queue != NULL) {
            uint8_t dummy = 0;
            xQueueSendFromISR(ble_rx_queue, &dummy, &xHigherPriorityTaskWoken);
        }
    }

    if (LL_USART_IsActiveFlag_PE(UART5)) {
        LL_USART_ClearFlag_PE(UART5);
    }
    if (LL_USART_IsActiveFlag_FE(UART5)) {
        LL_USART_ClearFlag_FE(UART5);
    }
    if (LL_USART_IsActiveFlag_ORE(UART5)) {
        LL_USART_ClearFlag_ORE(UART5);
    }
    if (LL_USART_IsActiveFlag_NE(UART5)) {
        LL_USART_ClearFlag_NE(UART5);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void DMA1_Stream0_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_TE0(DMA1)) {
        LL_DMA_ClearFlag_TE0(DMA1);
        LOG_ERR("DMA Transfer Error");
    }
    if (LL_DMA_IsActiveFlag_FE0(DMA1)) {
        LL_DMA_ClearFlag_FE0(DMA1);
    }
    if (LL_DMA_IsActiveFlag_DME0(DMA1)) {
        LL_DMA_ClearFlag_DME0(DMA1);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_11) {
        GPIO_PinState pin_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);

        if (pin_state == GPIO_PIN_RESET) {
            ble_app_set_conn_state(BLE_CONN_DISCONNECTED);
        } else {
            ble_app_set_conn_state(BLE_CONN_CONNECTED);
        }
    }
}

void EXTI15_10_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
}

#endif
