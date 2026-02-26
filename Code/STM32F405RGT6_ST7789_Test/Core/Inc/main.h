/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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
#define C_RST_Pin GPIO_PIN_0
#define C_RST_GPIO_Port GPIOC
#define C_INT_Pin GPIO_PIN_2
#define C_INT_GPIO_Port GPIOC
#define C_INT_EXTI_IRQn EXTI2_IRQn
#define LCD_BL_Pin GPIO_PIN_1
#define LCD_BL_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_3
#define SPI_CS_GPIO_Port GPIOA
#define BUTTON_Pin GPIO_PIN_15
#define BUTTON_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_12
#define RST_GPIO_Port GPIOC
#define TFT_CS_Pin GPIO_PIN_8
#define TFT_CS_GPIO_Port GPIOB
#define DC_Pin GPIO_PIN_9
#define DC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
