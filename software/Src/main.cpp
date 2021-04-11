/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include <vector>
#include <algorithm>

#include "main.h"

#include "can.h"
#include "can_feedc0de.h"
#include "serial_esp.h"
#include "serial_feedc0de.h"
#include "protocol.h"
#include "cruise_controller.h"
#include "debouncer.h"
#include "config.h"

// PA4 as cruise control button
#define GPIO_CRUISE_CONTROL GPIOA
#define GPIO_PIN_CRUISE_CONTROL GPIO_PIN_4

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;


UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC12_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA_Init(void);

CANFeedc0de<Command, Feedback> can_instances[NUM_BOARDS_MAX] = {
  {
    CAN_ID_COMMAND_STW_TO_BOARD(0),
    CAN_ID_COMMAND_BOARD_TO_STW(0),
    CAN_ID_FEEDBACK_STW_TO_BOARD(0),
    CAN_ID_FEEDBACK_BOARD_TO_STW(0)
  },
  {
    CAN_ID_COMMAND_STW_TO_BOARD(1),
    CAN_ID_COMMAND_BOARD_TO_STW(1),
    CAN_ID_FEEDBACK_STW_TO_BOARD(1),
    CAN_ID_FEEDBACK_BOARD_TO_STW(1)
  },
  {
    CAN_ID_COMMAND_STW_TO_BOARD(2),
    CAN_ID_COMMAND_BOARD_TO_STW(2),
    CAN_ID_FEEDBACK_STW_TO_BOARD(2),
    CAN_ID_FEEDBACK_BOARD_TO_STW(2)
  }
};

SerialFeedc0de serial_feedc0de;

static Feedback feedback;
static Command command;

Debouncer cc_button_debouncer(50 / MAIN_LOOP_DELAY_MS);
CruiseController cruise_controller;
static bool controlling_cruise = false;
Debouncer cc_disengage(500 / MAIN_LOOP_DELAY_MS);
Debouncer cc_buzzer(1000 / MAIN_LOOP_DELAY_MS);

static void check_cruise_control(int16_t current_pwm)
{
  bool cc_button_state = !HAL_GPIO_ReadPin(GPIO_CRUISE_CONTROL, GPIO_PIN_CRUISE_CONTROL);
  if (cc_button_debouncer.update(cc_button_state))
  {
    if (!controlling_cruise)
    {
      controlling_cruise = true;
      int16_t current_speed = (feedback.left.speed + feedback.right.speed) / 2;
      cruise_controller.start(current_speed, current_pwm);
    }
    else
    {
      controlling_cruise = false;
    }

    command.buzzer.freq = 4;
    cc_buzzer.update(false);
  }
}

// Assumes this is called with constant frequency
static void adc_to_motorstate(uint16_t gas, uint16_t brake, MotorState& motorState)
{
  // Clamp poti values if out of range
  gas = std::clamp(gas, (uint16_t)ADC_GAS_MIN, (uint16_t)ADC_GAS_MAX);
  brake = std::clamp(brake, (uint16_t)ADC_BRAKE_MIN, (uint16_t)ADC_BRAKE_MAX);

  motorState.iMotMax = 30;
  motorState.iDcMax = 35;
  motorState.enable = true;
  motorState.ctrlTyp = ControlType::FieldOrientedControl;
  motorState.ctrlMod = ControlMode::Voltage;
  motorState.fieldWeakMax = 10;
  motorState.nMotMax = 2000;

  int16_t gas_of_1500 = -1500 * (gas - ADC_GAS_MIN) / (ADC_GAS_MAX - ADC_GAS_MIN);
  int16_t brake_of_1500 = 1500 * (brake - ADC_BRAKE_MIN) / (ADC_BRAKE_MAX - ADC_BRAKE_MIN);
  int16_t combined = gas_of_1500 + brake_of_1500;

  int16_t current_speed = (feedback.left.speed + feedback.right.speed) / 2;

  int16_t control_input = controlling_cruise ? cruise_controller.step(current_speed) : combined;

  static float set = 0;
  static bool initialized = false;

  // TODO tau = dT / ln(1 - alpha)
  if (initialized)
  {
    set = set * ADC_SMOOTHING + control_input * (1 - ADC_SMOOTHING);
  }
  else
  {
    set = control_input;
    initialized = true;
  }
  set = std::clamp(set, -1500.f, 1500.f);

  // If cruise control button is pressed, enable cruise control
  // with set as inital value
  check_cruise_control(set);

  if (controlling_cruise)
  {
    bool control_pressed = gas_of_1500 < -100 || brake_of_1500 > 100;
    if (cc_disengage.update(control_pressed))
    {
      controlling_cruise = false;
      command.buzzer.freq = 4;
      cc_disengage.update(false);
      cc_buzzer.update(false);
    }
  }

  if (cc_buzzer.update(true))
  {
    command.buzzer.freq = 0;
  }

  motorState.pwm = set;
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC12_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();

  serial_esp_init();
  can_init();

  // FDCAN fails after cold start if we try sending frames too early
  // (5 V power supply not stable yet?)
  HAL_Delay(50);

  // Initial conversion
  if (HAL_ADC_Start(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  // Initial conversion
  if (HAL_ADC_Start(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  uint32_t next_update = HAL_GetTick();

  /* Infinite loop */
  while (1)
  {
    if (HAL_GetTick() >= next_update)
    {
      if (HAL_ADC_PollForConversion(&hadc1, 1) != HAL_OK)
      {
        Error_Handler();
      }

      if (HAL_ADC_PollForConversion(&hadc2, 1) != HAL_OK)
      {
        Error_Handler();
      }

      uint32_t gas = HAL_ADC_GetValue(&hadc1);
      uint32_t brake = HAL_ADC_GetValue(&hadc2);

      adc_to_motorstate(gas, brake, command.left);
      command.right = command.left;

      for (size_t i = 0; i < NUM_BOARDS_MAX; i++)
        can_instances[i].send(command);

      // Start conversion
      if (HAL_ADC_Start(&hadc1) != HAL_OK)
      {
        Error_Handler();
      }

      if (HAL_ADC_Start(&hadc2) != HAL_OK)
      {
        Error_Handler();
      }

      next_update += MAIN_LOOP_DELAY_MS;
    }

    for (size_t i = 0; i < NUM_BOARDS_MAX; i++)
    {
      can_instances[i].poll();
    }

    if (NUM_BOARDS_MAX < 1)
      Error_Handler();

    // Got new feedback?
    // TODO: Other controllers
    if (can_instances[0].get(feedback))
    {
      serial_feedc0de.send_feedback(feedback);
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC12_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  ADC_ChannelConfTypeDef sConfig2 = {0};

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init = hadc1.Init;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig2 = sConfig;
  sConfig2.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_DMA_Init(void)
{
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

GPIO_InitTypeDef GPIO_InitStruct;
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure input for cruise control button
  GPIO_InitStruct.Pin = GPIO_PIN_CRUISE_CONTROL;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIO_CRUISE_CONTROL, &GPIO_InitStruct);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void __NO_RETURN Error_Handler(void)
{
  while(1);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
