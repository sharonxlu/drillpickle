/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void ChangeLaunchSpeed(uint8_t lcdspeed, uint8_t lcdspin); // LCD input
static void ChangeSpin(uint8_t lcdspin, uint8_t lcdspeed); // LCD Input - 9 different spins, middle one is net zero
static void ChangeVerticalAngle(uint8_t newAngle, uint8_t currentAngle);
static void ChangeHorizontalAngle(uint8_t newAngle, uint8_t currentAngle);
static void ChangeDispenseFrequency(uint8_t lcdperiod);
static void BeginLaunching(void);
static void EndLaunching(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t speed = 1;
uint8_t spin = 5;
uint8_t vangle = 1;
uint8_t hangle = 6;
uint8_t period = 10;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_Delay(5000);
//	  ChangeLaunchSpeed(10, 5);
//	  BeginLaunching();
//	  HAL_Delay(10000);
//	  ChangeLaunchSpeed(1, 5);
//	  HAL_Delay(10000);
//	  EndLaunching();
//	  HAL_Delay(5000);


	  ChangeVerticalAngle(5, 1);
	  HAL_Delay(6000);
	  ChangeVerticalAngle(1,5);
	  HAL_Delay(6000);


//	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, 1);
//
//	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//	  int t = HAL_GPIO_ReadPin(GPIOB_GPIO_PIN_14);
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
//	  HAL_Delay(1000);
//	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
//	  HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 72-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
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
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_GREEN_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  This function changes the speed based on an input value
  * @param takes in int speed (8 bits, but only 9 speed presets will be used)
  * @param takes in int lcdspin, int 1 to 9, 1 being backspin, 5 being neutral, 9 being topspin (scales accordingly)
  * @retval None
  */
static void ChangeLaunchSpeed(uint8_t lcdspeed, uint8_t lcdspin){
	// TODO - fix spin issues when at top speed
	// TIM1->CCR1 is top wheel, CCR2 is Bottom wheel
	if(lcdspeed == 1){
		TIM1->CCR1 = 50;
		TIM1->CCR2 = 50;
	}
	else if(lcdspeed == 2){
		TIM1->CCR1 = 56;
		TIM1->CCR2 = 56;
	}
	else if(lcdspeed == 3){
		TIM1->CCR1 = 63;
		TIM1->CCR2 = 63;		}
	else if(lcdspeed == 4){
		TIM1->CCR1 = 69;
		TIM1->CCR2 = 69;		}
	else if(lcdspeed == 5){
		TIM1->CCR1 = 75;
		TIM1->CCR2 = 75;		}
	else if(lcdspeed == 6){
		TIM1->CCR1 = 75;
		TIM1->CCR2 = 75;		}
	else if(lcdspeed == 7){
			TIM1->CCR1 = 75;
			TIM1->CCR2 = 75;
//		switch(lcdspin) {
//				case 1: // full backspin
//					TIM1->CCR1 = 88;
//					TIM1->CCR2 = 72;
//				case 2:
//					TIM1->CCR1 = 88;
//					TIM1->CCR2 = 76;
//				case 3: //
//					TIM1->CCR1 = 88;
//					TIM1->CCR2 = 80;
//				case 4: //
//					TIM1->CCR1 = 88;
//					TIM1->CCR2 = 84;
//				case 5: // no spin
//					TIM1->CCR1 = 88;
//					TIM1->CCR2 = 88;
//				case 6: //
//					TIM1->CCR1 = 84;
//					TIM1->CCR2 = 88;
//				case 7: //
//					TIM1->CCR1 = 80;
//					TIM1->CCR2 = 88;
//				case 8: //
//					TIM1->CCR1 = 76;
//					TIM1->CCR2 = 88;
//				case 9: // full topspin
//					TIM1->CCR1 = 72;
//					TIM1->CCR2 = 88;
//				}
	}
	else if(lcdspeed == 8){
		TIM1->CCR1 = 75;
		TIM1->CCR2 = 75;
//		switch(lcdspin) {
//			case 1: // full backspin
//				TIM1->CCR1 = 94;
//				TIM1->CCR2 = 79;
//			case 2:
//				TIM1->CCR1 = 94;
//				TIM1->CCR2 = 83;
//			case 3: //
//				TIM1->CCR1 = 94;
//				TIM1->CCR2 = 87;
//			case 4: //
//				TIM1->CCR1 = 94;
//				TIM1->CCR2 = 91;
//			case 5: // no spin
//				TIM1->CCR1 = 94;
//				TIM1->CCR2 = 94;
//			case 6: //
//				TIM1->CCR1 = 91;
//				TIM1->CCR2 = 94;
//			case 7: //
//				TIM1->CCR1 = 87;
//				TIM1->CCR2 = 94;
//			case 8: //
//				TIM1->CCR1 = 83;
//				TIM1->CCR2 = 94;
//			case 9: // full topspin
//				TIM1->CCR1 = 79;
//				TIM1->CCR2 = 94;
//		}
	}
	else if(lcdspeed == 9){
		TIM1->CCR1 = 75;
		TIM1->CCR2 = 75;
	}
//		switch(lcdspin) {
//			case 1: // full backspin
//				TIM1->CCR1 = 99;
//				TIM1->CCR2 = 83;
//			case 2:
//				TIM1->CCR1 = 99;
//				TIM1->CCR2 = 87;
//			case 3: //
//				TIM1->CCR1 = 99;
//				TIM1->CCR2 = 91;
//			case 4: //
//				TIM1->CCR1 = 99;
//				TIM1->CCR2 = 95;
//			case 5: // no spin
//				TIM1->CCR1 = 99;
//				TIM1->CCR2 = 99;
//			case 6: //
//				TIM1->CCR1 = 95;
//				TIM1->CCR2 = 99;
//			case 7: //
//				TIM1->CCR1 = 91;
//				TIM1->CCR2 = 99;
//			case 8: //
//				TIM1->CCR1 = 87;
//				TIM1->CCR2 = 99;
//			case 9: // full topspin
//				TIM1->CCR1 = 81;
//				TIM1->CCR2 = 99;
//		}
	else if(lcdspeed == 10){
		TIM1->CCR1 = 100;
		TIM1->CCR2 = 100;
	}


}
static void ChangeSpin(uint8_t lcdspin, uint8_t lcdspeed){

}
/**
  * @brief  This function changes the vertical angle based on LCD input value
  * @param newAngle takes in the desired vertical angle (an int 1 to 11, 1 being lowest, 11 highest)
  * @param currentAngle takes in the current vertical angle (an int 1 to 11, 1 being lowest, 11 highest)
  * The actuator takes 10 seconds to move from bottom to top or top to bottom
  * @retval None
  */
static void ChangeVerticalAngle(uint8_t newAngle, uint8_t currentAngle){
	int diff = 0;
	int timedelay = 0;
//	TIM2->CCR1 = 99;
	if(newAngle > currentAngle){
		diff = newAngle - currentAngle; // this will be the # seconds to move the actuator for

		// Set Directional pins
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 1);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 0);

		// enable pin for x seconds, then disable
		timedelay = diff * 1000;
		TIM2->CCR1 = 99;
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_Delay(1000);
		TIM2->CCR1 = 0;
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

	}
	else if(newAngle < currentAngle){
		diff = currentAngle - newAngle; // this will be the # seconds to move the actuator for

		// Set Directional pins (opposite as above)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, 0);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, 1);

		// enable pin for x seconds, then disable
		timedelay = diff * 1000;
		TIM2->CCR1 = 99;
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_Delay(1000);
		TIM2->CCR1 = 0;
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	}
	// update stored value
	vangle = newAngle;
	// other else is if nothing changes, but we need to do nothing in this case

}
/**
  * @brief  This function changes the horizontal angle based on LCD input value
  * @param newAngle takes in the desired horizontal angle (an int 1 to 11, 1 being lowest, 11 highest)
  * @param currentAngle takes in the current horizontal angle (an int 1 to 11, 1 being lowest, 11 highest)
  * The actuator takes 10 seconds to move from left to right or right to left
  * @retval None
  */
static void ChangeHorizontalAngle(uint8_t newAngle, uint8_t currentAngle){
	int diff = 0;
	int timedelay = 0;
//	TIM2->CCR2 = 99;
	if(newAngle > currentAngle){
		diff = newAngle - currentAngle; // this will be the # seconds to move the actuator for

		// Set Directional pins
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);

		// enable pin for x seconds, then disable
		timedelay = diff * 1000;
		TIM2->CCR2 = 99;
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		HAL_Delay(1000);
		TIM2->CCR2 = 0;
		HAL_Delay(1000);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);



	}
	else if(newAngle < currentAngle){
		diff = currentAngle - newAngle; // this will be the # seconds to move the actuator for

		// Set Directional pins (opposite as above)
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 0);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);

		// enable pin for x seconds, then disable
		timedelay = diff * 1000;
		TIM2->CCR2 = 99;
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		HAL_Delay(1000);
		TIM2->CCR2 = 0;
		HAL_Delay(1000);
		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
	}
	// update stored value
	hangle = newAngle;
	// other else is if nothing changes, but we need to do nothing in this case

}
/**
  * @brief  This function changes the dispensing motor's frequency using varying PWM signal duty cycles
  * @param lcdperiod is the desired period in seconds
  * @retval None
  * TODO - Figure out PWM duty cycles that give desired frequency
  */
static void ChangeDispenseFrequency(uint8_t lcdperiod){
	switch(lcdperiod){
	case 5:
		TIM1->CCR3 = 50;
	case 6:
		TIM1->CCR3 = 60;
	case 7:
		TIM1->CCR3 = 70;
	case 8:
		TIM1->CCR3 = 80;
	case 9:
		TIM1->CCR3 = 90;
	case 10:
		TIM1->CCR3 = 100;
	}

}
static void BeginLaunching(void){
	//write the directional signals
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);

	// write the enable signals to the motor drivers
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
//	HAL_Delay(8000);

}
static void EndLaunching(void){
	// write the enable signal as 0 for the motor drivers
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);



}
static void ShutDown(void){
	// reset the actuators so that the machine knows where they are always
	ChangeHorizontalAngle(6, hangle);
	ChangeVerticalAngle(1, vangle);


}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
