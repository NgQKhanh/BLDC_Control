/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AH TIM_CHANNEL_1
#define BH TIM_CHANNEL_2
#define CH TIM_CHANNEL_3

#define AL GPIO_PIN_11
#define BL GPIO_PIN_12
#define CL GPIO_PIN_15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t BA, BB, BC, state;
uint8_t run;
uint8_t dir;
uint8_t isStart;
uint8_t dataRx;
uint16_t dutyCycle = 899;
uint32_t ICVal = 0, preICVal = 0, dif;
uint32_t last = 0, cur = 0;
char time[5];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void rotate_cw(void);
void rotate_ccw(void);
void stop(void);
void setDutyCycle(uint32_t dc);
void motorSpeed(void);
void debugWrite(char *s);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	 uint32_t i;
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
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	run = 0;
	isStart = 0;
	dir = 0;
	state = 1;

	stop();
	setDutyCycle(dutyCycle);


	HAL_UART_Receive_IT(&huart3, &dataRx, sizeof(dataRx));
	run = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
		{

		//Quay thuan
		if(run && (dir == 1)){
				i = 200;
				while(i > 1) {
					HAL_Delay(1);
					rotate_cw();
					state++;
					if(state > 6) state = 1;
					i = i - 1;
					motorSpeed();
					}
				isStart = 1;

				while(run){
					rotate_cw();
					motorSpeed();
				}
			}

			// Quay nguoc
		else if(run && (dir == 2)){
				i = 200;
				while(i > 1) {
					HAL_Delay(1);
					rotate_ccw();
					state++;
					if(state > 6) state = 1;
					i = i - 1;
					}
				isStart = 1;

				while(run){
					rotate_ccw();
					motorSpeed();
				}
		}

		if(!run){
			  stop();
			  isStart = 0;
		}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CL_Pin|BL_Pin|AL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BEMF_A_Pin BEMF_B_Pin BEMF_C_Pin */
  GPIO_InitStruct.Pin = BEMF_A_Pin|BEMF_B_Pin|BEMF_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CL_Pin BL_Pin AL_Pin */
  GPIO_InitStruct.Pin = CL_Pin|BL_Pin|AL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

// Ham quay thuan
void rotate_cw(void)
{
	switch(state)
	{
		case 1:
			HAL_TIM_PWM_Stop(&htim4,BH);				//B-
			HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_RESET);

			HAL_TIM_PWM_Stop(&htim4,CH);				//C
			HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim4,AH);				//A+
			break;

		case 2:
			HAL_TIM_PWM_Stop(&htim4,CH);				//C-
			HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_RESET);

			HAL_TIM_PWM_Stop(&htim4,BH);				//B
			HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim4,AH);				//A+
			break;

		case 3:
			HAL_TIM_PWM_Stop(&htim4,CH);				//C-
			HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_RESET);

			HAL_TIM_PWM_Stop(&htim4,AH);				//A
			HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim4,BH);				//B+
			break;

		case 4:
			HAL_TIM_PWM_Stop(&htim4,AH);				//A-
			HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_RESET);

			HAL_TIM_PWM_Stop(&htim4,CH);				//C
			HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim4,BH);				//B+
			break;

		case 5:
			HAL_TIM_PWM_Stop(&htim4,BH);				//B
			HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim4,CH);				//C+

			HAL_TIM_PWM_Stop(&htim4,AH);				//A-
			HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_RESET);
			break;

		case 6:
			HAL_TIM_PWM_Stop(&htim4,AH);				//A
			HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_SET);

			HAL_TIM_PWM_Stop(&htim4,BH);				//B-
			HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim4,CH);				//C+
			break;

		default:
			HAL_TIM_PWM_Stop(&htim4,AH);				//A
			HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_SET);

			HAL_TIM_PWM_Stop(&htim4,BH);				//B
			HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_SET);

			HAL_TIM_PWM_Stop(&htim4,CH);				//C
			HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_SET);
			break;
	}
}

//Ham quay nguoc
void rotate_ccw(void)
{
	switch(state)
		{
		case 6:
			HAL_TIM_PWM_Stop(&htim4,BH);				//B-
			HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_RESET);

			HAL_TIM_PWM_Stop(&htim4,CH);				//C
			HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim4,AH);				//A+
			break;

		case 5:
			HAL_TIM_PWM_Stop(&htim4,CH);				//C-
			HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_RESET);

			HAL_TIM_PWM_Stop(&htim4,BH);				//B
			HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim4,AH);				//A+
			break;

		case 4:
			HAL_TIM_PWM_Stop(&htim4,CH);				//C-
			HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_RESET);

			HAL_TIM_PWM_Stop(&htim4,AH);				//A
			HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim4,BH);				//B+
			break;

		case 3:
			HAL_TIM_PWM_Stop(&htim4,AH);				//A-
			HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_RESET);

			HAL_TIM_PWM_Stop(&htim4,CH);				//C
			HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim4,BH);				//B+
			break;

		case 2:
			HAL_TIM_PWM_Stop(&htim4,BH);				//B
			HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_SET);

			HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim4,CH);				//C+

			HAL_TIM_PWM_Stop(&htim4,AH);				//A-
			HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_RESET);
			break;

		case 1:
			HAL_TIM_PWM_Stop(&htim4,AH);				//A
			HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_SET);

			HAL_TIM_PWM_Stop(&htim4,BH);				//B-
			HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_RESET);

			HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_SET);
			HAL_TIM_PWM_Start(&htim4,CH);				//C+
			break;

	default:
		HAL_TIM_PWM_Stop(&htim4,AH);				//A
		HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_SET);

		HAL_TIM_PWM_Stop(&htim4,BH);				//B
		HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_SET);

		HAL_TIM_PWM_Stop(&htim4,CH);				//C
		HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_SET);
		break;
		}
}

// Dung dong co
void stop(void){
		HAL_TIM_PWM_Stop(&htim4,AH);				//A
		HAL_GPIO_WritePin(GPIOA,AL,GPIO_PIN_SET);

		HAL_TIM_PWM_Stop(&htim4,BH);				//B
		HAL_GPIO_WritePin(GPIOA,BL,GPIO_PIN_SET);

		HAL_TIM_PWM_Stop(&htim4,CH);				//C
		HAL_GPIO_WritePin(GPIOA,CL,GPIO_PIN_SET);
}

void setDutyCycle(uint32_t dc){
	__HAL_TIM_SET_COMPARE(&htim4, AH, dc);
	__HAL_TIM_SET_COMPARE(&htim4, BH, dc);
	__HAL_TIM_SET_COMPARE(&htim4, CH, dc);
}

// Phan hoi toc do 
void motorSpeed(void)
{
	cur = HAL_GetTick();
	if(cur - last > 1000)
	{
		last = cur;
		sprintf(time,"%d", dif);
		HAL_UART_Transmit(&huart3, (uint8_t*)time, strlen(time), HAL_MAX_DELAY);
	}
}

// Xac dinh chuyen mach
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if ((GPIO_Pin == GPIO_PIN_8)||
			(GPIO_Pin == GPIO_PIN_9)||
			(GPIO_Pin == GPIO_PIN_10)){
			if(isStart){
				state ++;
				if(state > 6) state = 1;
		}
	}
}

// Xac dinh thoi gian 1 chu ky
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	if((htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)){

		ICVal = HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_3);
		if(ICVal > preICVal){
			dif = ICVal - preICVal;
		}
		if(ICVal < preICVal){
			dif = 0xffff + ICVal - preICVal;
		}
	}
}

void debugWrite(char *s)
{
	  HAL_UART_Transmit(&huart3, (uint8_t*)s, strlen(s), HAL_MAX_DELAY);
}

// Nhan tin hieu dieu khien tu may tinh
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 if(huart->Instance == huart3.Instance)
	 {
		 // Dung dong co
	   if(dataRx == '0'){
				run = 0;
				stop();
	   }
		 // Quay thuan
	   else if(dataRx == '1') {
				run = 1;
				dir = 1;
			 	sprintf(time,"%d", 12500);
				HAL_UART_Transmit(&huart3, (uint8_t*)time, strlen(time), HAL_MAX_DELAY);
	   }
		 // Quay nguoc
	   else if(dataRx == '2') {
				run = 1;
				dir = 1;
	   }
		 // Tang toc do
	   else if(dataRx == '3') {
				dutyCycle += 100;
			 	if(dutyCycle > 999) dutyCycle = 999;
				setDutyCycle(dutyCycle);
	   }
		 // Giam toc do
	   else if(dataRx == '4') {
				dutyCycle -= 100;
			 	if(dutyCycle < 0) dutyCycle = 0;
			 	setDutyCycle(dutyCycle);
	   }
	 }
	   HAL_UART_Receive_IT(&huart3, &dataRx, 1);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
