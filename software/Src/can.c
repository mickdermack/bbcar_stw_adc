#include "main.h"
#include "can.h"

#include "can_feedc0de.h"


/* Definition for CANx clock resources */
#define FDCANx                           FDCAN1
#define FDCANx_CLK_ENABLE()              __HAL_RCC_FDCAN_CLK_ENABLE()
#define FDCANx_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()

#define FDCANx_FORCE_RESET()             __HAL_RCC_FDCAN_FORCE_RESET()
#define FDCANx_RELEASE_RESET()           __HAL_RCC_FDCAN_RELEASE_RESET()

/* Definition for CANx Pins */
#define FDCANx_TX_PIN                    GPIO_PIN_12
#define FDCANx_RX_PIN                    GPIO_PIN_11
#define FDCANx_GPIO_PORT                 GPIOA

/* Definition for CAN's NVIC */
#define FDCANx_IT0_IRQn                  FDCAN1_IT0_IRQn
#define FDCANx_IT0_IRQHandler            FDCAN1_IT0_IRQHandler


FDCAN_HandleTypeDef hfdcan;

uint8_t ubKeyNumber = 0x0;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
FDCAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];


// Forward definitions for callbacks
static void FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan);
static void FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan);
static void FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
static void FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs);


static uint32_t len_to_dlc(uint8_t len)
{
  uint32_t len_u32 = len;

  if (len_u32 <= 8)
    return len_u32 << 16;

  // CAN-FD-only lengths currently not supported
  Error_Handler();
}

static uint8_t dlc_to_len(uint32_t dlc)
{
  // Check if DLC valid
  if (dlc & ~0x000F0000u)
    Error_Handler();

  // CAN-FD-only lengths currently not supported
  if (dlc >> 16 > 8)
    Error_Handler();

  return dlc >> 16;
}

static void FDCAN_Tx(uint16_t address, const uint8_t* data, uint8_t len)
{
  TxHeader.Identifier = address;
  TxHeader.DataLength = len_to_dlc(len);

  while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan) < 1)
  {
  }

  /* Start the Transmission process */
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &TxHeader, (uint8_t *)data) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void FDCAN_Init(void)
{
  hfdcan.Instance = FDCANx;
  hfdcan.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan.Init.AutoRetransmission = ENABLE;
  hfdcan.Init.TransmitPause = DISABLE;
  hfdcan.Init.ProtocolException = DISABLE;

  hfdcan.Init.NominalPrescaler = 1;
  hfdcan.Init.NominalSyncJumpWidth = 1;
  hfdcan.Init.NominalTimeSeg1 = 6;
  hfdcan.Init.NominalTimeSeg2 = 1;

  hfdcan.Init.DataPrescaler = 1;
  hfdcan.Init.DataSyncJumpWidth = 1;
  hfdcan.Init.DataTimeSeg1 = 6;
  hfdcan.Init.DataTimeSeg2 = 1;

  hfdcan.Init.StdFiltersNbr = 1;
  hfdcan.Init.ExtFiltersNbr = 0;
  hfdcan.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  hfdcan.MspInitCallback = FDCAN_MspInit;
  hfdcan.MspDeInitCallback = FDCAN_MspDeInit;

  if (HAL_FDCAN_Init(&hfdcan) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Configures the FDCAN.
  * @param  None
  * @retval None
  */
static void FDCAN_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;

  /* Configure Rx filter */
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x300;
  sFilterConfig.FilterID2 = 0x700;
  if (HAL_FDCAN_ConfigFilter(&hfdcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure global filter:
     Filter all remote frames with STD and EXT ID
     Reject non matching frames with STD ID and EXT ID */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_RegisterRxFifo0Callback(&hfdcan, FDCAN_RxFifo0Callback) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_RegisterErrorStatusCallback(&hfdcan, FDCAN_ErrorStatusCallback) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_FDCAN_ActivateNotification(&hfdcan,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Start the FDCAN module */
  if (HAL_FDCAN_Start(&hfdcan) != HAL_OK)
  {
    Error_Handler();
  }

  /* Prepare Tx Header */
  TxHeader.Identifier = 0x321;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_2;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
}

/**
* @brief FDCAN MSP Initialization
* This function configures the hardware resources used in this example
* @param hfdcan: FDCAN handle pointer
* @retval None
*/
void FDCAN_MspInit(FDCAN_HandleTypeDef* hfdcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* Peripheral clock enable */
  __HAL_RCC_FDCAN_CLK_ENABLE();

  /**FDCAN1 GPIO Configuration
  PA11     ------> FDCAN1_RX
  PA12     ------> FDCAN1_TX
  */
  FDCANx_GPIO_CLK_ENABLE();

  GPIO_InitStruct.Pin = FDCANx_TX_PIN | FDCANx_RX_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(FDCANx_GPIO_PORT, &GPIO_InitStruct);

  /* FDCANx interrupt Init */
  HAL_NVIC_SetPriority(FDCANx_IT0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FDCANx_IT0_IRQn);
}

/**
* @brief FDCAN MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hfdcan: FDCAN handle pointer
* @retval None
*/
void FDCAN_MspDeInit(FDCAN_HandleTypeDef* hfdcan)
{
  __HAL_RCC_FDCAN_FORCE_RESET();
  __HAL_RCC_FDCAN_RELEASE_RESET();

  /* Peripheral clock disable */
  __HAL_RCC_FDCAN_CLK_DISABLE();

  /**FDCAN1 GPIO Configuration
  PA11     ------> FDCAN1_RX
  PA12     ------> FDCAN1_TX
  */
  HAL_GPIO_DeInit(GPIOA, FDCANx_TX_PIN | FDCANx_RX_PIN);
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs: indicates which Rx FIFO 0 interrupts are signalled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
  {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      Error_Handler();
    }

    if (RxHeader.IdType == FDCAN_STANDARD_ID && (RxHeader.Identifier & 0x300) == 0x300)
    {
      can_feedc0de_handle_frame(RxHeader.Identifier, RxData, dlc_to_len(RxHeader.DataLength));
    }
  }
}

void FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs)
{
  if (ErrorStatusITs & FDCAN_IR_BO)
  {
    // Try to exit Bus_Off state
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
  }
}

void can_init(void)
{
  FDCAN_Init();
  FDCAN_Config();
}

void can_tx(uint16_t address, const uint8_t* data, uint8_t len)
{
  FDCAN_Tx(address, data, len);
}
