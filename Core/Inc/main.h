/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BEMF_A_Pin GPIO_PIN_8
#define BEMF_A_GPIO_Port GPIOA
#define BEMF_A_EXTI_IRQn EXTI9_5_IRQn
#define BEMF_B_Pin GPIO_PIN_9
#define BEMF_B_GPIO_Port GPIOA
#define BEMF_B_EXTI_IRQn EXTI9_5_IRQn
#define BEMF_C_Pin GPIO_PIN_10
#define BEMF_C_GPIO_Port GPIOA
#define BEMF_C_EXTI_IRQn EXTI15_10_IRQn
#define CL_Pin GPIO_PIN_11
#define CL_GPIO_Port GPIOA
#define BL_Pin GPIO_PIN_12
#define BL_GPIO_Port GPIOA
#define AL_Pin GPIO_PIN_15
#define AL_GPIO_Port GPIOA
#define CH_Pin GPIO_PIN_6
#define CH_GPIO_Port GPIOB
#define BH_Pin GPIO_PIN_7
#define BH_GPIO_Port GPIOB
#define AH_Pin GPIO_PIN_8
#define AH_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
