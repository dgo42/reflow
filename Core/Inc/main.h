/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
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

/* USER CODE BEGIN Private defines */
/* Private defines -----------------------------------------------------------*/
// define custom periphery pins
#define USER_KEYA_Pin GPIO_PIN_13
#define USER_KEYA_GPIO_Port GPIOC

#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define USER_KEYB_Pin GPIO_PIN_2
#define USER_KEYB_GPIO_Port GPIOB

#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOB

//#define OVEN_EN_Pin GPIO_PIN_12
//#define OVEN_EN_GPIO_Port GPIOD
#define COOLER_EN_Pin GPIO_PIN_13
#define COOLER_EN_GPIO_Port GPIOD

#define SD_CD_Pin GPIO_PIN_3
#define SD_CD_GPIO_Port GPIOD

#define BACK_LIGHT_Pin GPIO_PIN_5
#define BACK_LIGHT_GPIO_Port GPIOB

#define TOUCH_IRQ_Pin GPIO_PIN_6
#define TOUCH_IRQ_GPIO_Port GPIOB

#define USB_EN_Pin GPIO_PIN_7
#define USB_EN_GPIO_Port GPIOB

// define 50Hz phase control (switch triac from 0° to 180° phase) of the oven
#define PHASE_POWER
// define 50Hz pulse control (switch triac only by crossing zero)
//#define PULSE_POWER
#define PID_DEBUG
/* USER CODE END Private defines */

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
