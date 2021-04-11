#include "stm32g4xx_hal.h"
#include "main.h"

#include "serial_esp.h"

UART_HandleTypeDef huart2;

static void UART_MspInit(UART_HandleTypeDef* huart);
static void UART_MspDeInit(UART_HandleTypeDef* huart);

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void serial_esp_init(void)
{
  huart2.MspInitCallback = UART_MspInit;
  huart2.MspDeInitCallback = UART_MspDeInit;

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

void serial_esp_tx(uint8_t* data, size_t len)
{
  if (HAL_UART_Transmit_DMA(&huart2, data, len) != HAL_OK)
  {
    /* Transfer error in transmission process */
    Error_Handler();
  }
}

bool serial_esp_tx_ready()
{
  HAL_UART_StateTypeDef state = HAL_UART_GetState(&huart2);
  return state == HAL_UART_STATE_READY ||
         state == HAL_UART_STATE_BUSY_RX;
}

static GPIO_InitTypeDef GPIO_InitStruct;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

static void UART_MspInit(UART_HandleTypeDef* huart)
{
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**USART2 GPIO Configuration
  PA2     ------> USART2_TX
  PA3     ------> USART2_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART2 DMA Init */
  /* USART2_RX Init */
  hdma_usart2_rx.Instance = DMA1_Channel2;
  hdma_usart2_rx.Init.Request = DMA_REQUEST_USART2_RX;
  hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart2_rx.Init.Mode = DMA_NORMAL;
  hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
  if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(huart,hdmarx,hdma_usart2_rx);

  /* USART2_TX Init */
  hdma_usart2_tx.Instance = DMA1_Channel1;
  hdma_usart2_tx.Init.Request = DMA_REQUEST_USART2_TX;
  hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart2_tx.Init.Mode = DMA_NORMAL;
  hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
  if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(huart,hdmatx,hdma_usart2_tx);

  /* USART1 interrupt Init */
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

static void UART_MspDeInit(UART_HandleTypeDef* huart)
{
  /* Peripheral clock disable */
  __HAL_RCC_USART2_CLK_DISABLE();

  /**USART2 GPIO Configuration
  PA2     ------> USART2_TX
  PA3     ------> USART2_RX
  */
  HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

  // TODO: Deinit DMA
}
