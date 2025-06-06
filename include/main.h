        



/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#define USB_OUT_BUFFER_SIZE 64


#define ICM_CS_Pin GPIO_PIN_4
#define ICM_CS_GPIO_Port GPIOA
#define ICM_SCK_Pin GPIO_PIN_5
#define ICM_SCK_GPIO_Port GPIOA
#define ICM_MISO_Pin GPIO_PIN_6
#define ICM_MISO_GPIO_Port GPIOA
#define ICM_MOSI_Pin GPIO_PIN_7
#define ICM_MOSI_GPIO_Port GPIOA
#define ICM_INT_Pin GPIO_PIN_0
#define ICM_INT_GPIO_Port GPIOB
#define SCOPE_PIN_Pin GPIO_PIN_1
#define SCOPE_PIN_GPIO_Port GPIOB
#define STATUS_LED_Pin GPIO_PIN_2
#define STATUS_LED_GPIO_Port GPIOB


#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOC


void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
