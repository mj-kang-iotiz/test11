#include "gps_port.h"
#include "ringbuffer.h"
#include "board_config.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_utils.h"

#ifndef TAG
#define TAG "GPS_PORT"
#endif

#include "log.h"

static char gps_recv_buf[2048];
static char gps_rb_buffer[2048];
static gps_t *g_gps_instance = NULL;
static volatile size_t gps_dma_old_pos = 0;

static void gps_dma_process_data(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint8_t dummy = 0;

  if (!g_gps_instance || !g_gps_instance->pkt_queue)
    return;

  // 현재 DMA 위치 계산
  size_t pos = sizeof(gps_recv_buf) - LL_DMA_GetDataLength(DMA1, LL_DMA_STREAM_5);

  if (pos != gps_dma_old_pos)
  {
    if (pos > gps_dma_old_pos)
    {
      // 선형: old_pos ~ pos
      ringbuffer_write(&g_gps_instance->rx_buf, &gps_recv_buf[gps_dma_old_pos], pos - gps_dma_old_pos);
    }
    else
    {
      // 순환: old_pos ~ 끝, 0 ~ pos
      ringbuffer_write(&g_gps_instance->rx_buf, &gps_recv_buf[gps_dma_old_pos], sizeof(gps_recv_buf) - gps_dma_old_pos);
      if (pos > 0)
      {
        ringbuffer_write(&g_gps_instance->rx_buf, gps_recv_buf, pos);
      }
    }

    gps_dma_old_pos = pos;
    g_gps_instance->parser_ctx.stats.last_rx_tick = xTaskGetTickCountFromISR();
    xQueueSendFromISR(g_gps_instance->pkt_queue, &dummy, &xHigherPriorityTaskWoken);
  }

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void gps_uart2_init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2 | LL_GPIO_PIN_3;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 DMA Init */

  /* USART2_RX Init */
  LL_DMA_SetChannelSelection(DMA1, LL_DMA_STREAM_5, LL_DMA_CHANNEL_4);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_5,
                                  LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_5, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_5, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_5, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_5, LL_DMA_MDATAALIGN_BYTE);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_5);

  /* USART2 interrupt Init */
  NVIC_SetPriority(USART2_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
  NVIC_EnableIRQ(USART2_IRQn);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = 115200;

  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART2, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART2);
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void gps_uart2_dma_init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream5_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
  NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

/**
 * @brief GPS 하드웨어 초기화
 *
 */
int gps_rtk_uart2_init(void)
{
  gps_uart2_dma_init();
  gps_uart2_init();

  return 0;
}

/**
 * @brief GPS 통신 시작
 *
 */
void gps_uart2_comm_start(void)
{
  LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_5, (uint32_t)&USART2->DR);
  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_5,
                          (uint32_t)&gps_recv_buf);
  LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_5,
                       sizeof(gps_recv_buf));
  LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_5);
  LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_5);
  LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_5);
  LL_DMA_EnableIT_FE(DMA1, LL_DMA_STREAM_5);
  LL_DMA_EnableIT_DME(DMA1, LL_DMA_STREAM_5);

  LL_USART_EnableIT_IDLE(USART2);
  LL_USART_EnableIT_PE(USART2);
  LL_USART_EnableIT_ERROR(USART2);
  LL_USART_EnableDMAReq_RX(USART2);

  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_5);
  LL_USART_Enable(USART2);
}

/**
 * @brief GPS GPIO 핀 동작
 *
 */
void gps_rtk_gpio_start(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // RTK Reset pin
  //  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // RTK WAKEUP pin
}

int gps_rtk_reset(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  return 0;
}

/**
 * @brief GPS enable
 *
 */
int gps_rtk_start(void)
{
  gps_uart2_comm_start();
  gps_rtk_gpio_start();

  return 0;
}

int gps_uart2_send(const char *data, size_t len)
{
  for (int i = 0; i < len; i++)
  {
    while (!LL_USART_IsActiveFlag_TXE(USART2))
      ;
    LL_USART_TransmitData8(USART2, *(data + i));
  }

  while (!LL_USART_IsActiveFlag_TC(USART2))
    ;

  return 0;
}

static const gps_hal_ops_t gps_rtk_uart2_ops = {
    .init = gps_rtk_uart2_init,
    .reset = gps_rtk_reset,
    .start = gps_rtk_start,
    .stop = NULL,
    .send = gps_uart2_send,
    .recv = NULL,
};

/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler(void)
{
  if (LL_USART_IsActiveFlag_IDLE(USART2))
  {
    LL_USART_ClearFlag_IDLE(USART2);
    gps_dma_process_data();
  }
  if (LL_USART_IsActiveFlag_PE(USART2))
  {
    LL_USART_ClearFlag_PE(USART2);
  }
  if (LL_USART_IsActiveFlag_FE(USART2))
  {
    LL_USART_ClearFlag_FE(USART2);
  }
  if (LL_USART_IsActiveFlag_ORE(USART2))
  {
    LL_USART_ClearFlag_ORE(USART2);
  }
  if (LL_USART_IsActiveFlag_NE(USART2))
  {
    LL_USART_ClearFlag_NE(USART2);
  }
}

/**
 * @brief This function handles DMA1 stream5 global interrupt.
 */
void DMA1_Stream5_IRQHandler(void)
{
	if (LL_DMA_IsActiveFlag_HT5(DMA1))
	{
		LL_DMA_ClearFlag_HT5(DMA1);
		gps_dma_process_data();
	}

	if (LL_DMA_IsActiveFlag_TC5(DMA1))
	{
		LL_DMA_ClearFlag_TC5(DMA1);
	    gps_dma_process_data();
	}

	  if (LL_DMA_IsActiveFlag_TE5(DMA1))
	  {
	    LL_DMA_ClearFlag_TE5(DMA1);
	  }

	  // FIFO Error
	  if (LL_DMA_IsActiveFlag_FE5(DMA1))
	  {
	    LL_DMA_ClearFlag_FE5(DMA1);
	  }

	  // Direct Mode Error
	  if (LL_DMA_IsActiveFlag_DME5(DMA1))
	  {
	    LL_DMA_ClearFlag_DME5(DMA1);
	  }
}

int gps_port_init(gps_t *gps_handle)
{
  if(gps_handle)
  {
    g_gps_instance = gps_handle;
  }

  ringbuffer_init(&gps_handle->rx_buf, gps_rb_buffer, sizeof(gps_rb_buffer));

  gps_handle->ops = &gps_rtk_uart2_ops;
  if (gps_handle->ops->init)
  {
    gps_handle->ops->init();
  }
 
  return 0;
}

/**
 * @brief GPS 통신 시작
 */
void gps_port_start(gps_t *gps_handle)
{
  if (!gps_handle || !gps_handle->ops || !gps_handle->ops->start)
  {
    LOG_ERR("GPS start failed: invalid handle or ops");
    return;
  }

  gps_handle->ops->start();
}

/**
 * @brief GPS UART2 통신 정지
 *
 * UART 페리페럴을 리셋하여 모든 통신을 정지합니다.
 * 클럭 리셋으로 UART 레지스터가 초기화되고 DMA 요청도 비활성화됩니다.
 */
static void gps_uart2_comm_stop(void)
{
  /* UART2 페리페럴 리셋 (LL 사용) */
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_USART2);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_USART2);

  /* DMA 위치 초기화 */
  gps_dma_old_pos = 0;

  LOG_INFO("GPS UART2 통신 정지 완료");
}

/**
 * @brief GPS 전원 OFF (GPIO 리셋)
 */
static void gps_rtk_gpio_stop(void)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  /* RTK Reset pin LOW */
  LOG_INFO("GPS RTK 전원 OFF");
}

/**
 * @brief GPS 하드웨어 정지 (통신 + 전원)
 */
static int gps_rtk_stop(void)
{
  gps_uart2_comm_stop();
  gps_rtk_gpio_stop();
  return 0;
}

/**
 * @brief GPS 통신 정지
 *
 * HAL ops에 stop 함수가 등록되어 있으면 호출하고,
 * 없으면 기본 정지 로직(gps_rtk_stop)을 실행합니다.
 */
void gps_port_stop(gps_t *gps_handle)
{
  if (!gps_handle) {
    LOG_ERR("GPS stop failed: invalid handle");
    return;
  }

  /* ops->stop이 있으면 사용, 없으면 기본 정지 로직 */
  if (gps_handle->ops && gps_handle->ops->stop) {
    gps_handle->ops->stop();
  } else {
    gps_rtk_stop();
  }

  /* 글로벌 인스턴스 정리 */
  g_gps_instance = NULL;
}

/**
 * @brief GPS 포트 리소스 정리
 */
void gps_port_cleanup(void)
{
  /* 통신이 아직 활성화되어 있으면 정지 */
  if (g_gps_instance) {
    gps_port_stop(g_gps_instance);
  }
}
