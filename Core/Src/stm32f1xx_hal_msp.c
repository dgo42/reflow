/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : stm32f1xx_hal_msp.c
 * Description        : This file provides code for the MSP Initialization
 *                      and de-Initialization codes.
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

/* Includes ------------------------------------------------------------------*/

#include <main.h>
#include <stm32f103xe.h>
#include <stm32f1xx_hal_cortex.h>
#include <stm32f1xx_hal_crc.h>
#include <stm32f1xx_hal_gpio.h>
#include <stm32f1xx_hal_gpio_ex.h>
#include <stm32f1xx_hal_pwr.h>
#include <stm32f1xx_hal_rcc.h>
#include <stm32f1xx_hal_rcc_ex.h>
#include <stm32f1xx_hal_rtc.h>
#include <stm32f1xx_hal_sd.h>
#include <stm32f1xx_hal_spi.h>
#include <stm32f1xx_hal_sram.h>
#include <stm32f1xx_hal_tim.h>
#include <stm32f1xx_hal_uart.h>
#include <sys/_stdint.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void) {
	/* USER CODE BEGIN MspInit 0 */

	/* USER CODE END MspInit 0 */

	__HAL_RCC_AFIO_CLK_ENABLE()
	;
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	/* System interrupt init*/
	/* PendSV_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

	/** NONJTRST: Full SWJ (JTAG-DP + SW-DP) but without NJTRST
	 */
	__HAL_AFIO_REMAP_SWJ_NONJTRST()
	;

	/* USER CODE BEGIN MspInit 1 */

	/* USER CODE END MspInit 1 */
}

/**
 * @brief CRC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hcrc: CRC handle pointer
 * @retval None
 */
void HAL_CRC_MspInit(CRC_HandleTypeDef* hcrc) {
	if (hcrc->Instance == CRC) {
		/* USER CODE BEGIN CRC_MspInit 0 */

		/* USER CODE END CRC_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_CRC_CLK_ENABLE()
		;
		/* USER CODE BEGIN CRC_MspInit 1 */

		/* USER CODE END CRC_MspInit 1 */
	}

}

/**
 * @brief CRC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hcrc: CRC handle pointer
 * @retval None
 */
void HAL_CRC_MspDeInit(CRC_HandleTypeDef* hcrc) {
	if (hcrc->Instance == CRC) {
		/* USER CODE BEGIN CRC_MspDeInit 0 */

		/* USER CODE END CRC_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_CRC_CLK_DISABLE();
		/* USER CODE BEGIN CRC_MspDeInit 1 */

		/* USER CODE END CRC_MspDeInit 1 */
	}

}

/**
 * @brief RTC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hrtc: RTC handle pointer
 * @retval None
 */
void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc) {
	if (hrtc->Instance == RTC) {
		/* USER CODE BEGIN RTC_MspInit 0 */

		/* USER CODE END RTC_MspInit 0 */
		HAL_PWR_EnableBkUpAccess();
		/* Enable BKP CLK enable for backup registers */
		__HAL_RCC_BKP_CLK_ENABLE()
		;
		/* Peripheral clock enable */
		__HAL_RCC_RTC_ENABLE();
		/* RTC interrupt Init */
		HAL_NVIC_SetPriority(RTC_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(RTC_IRQn);
		/* USER CODE BEGIN RTC_MspInit 1 */

		/* USER CODE END RTC_MspInit 1 */
	}

}

/**
 * @brief RTC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hrtc: RTC handle pointer
 * @retval None
 */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc) {
	if (hrtc->Instance == RTC) {
		/* USER CODE BEGIN RTC_MspDeInit 0 */

		/* USER CODE END RTC_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_RTC_DISABLE();

		/* RTC interrupt DeInit */
		HAL_NVIC_DisableIRQ(RTC_IRQn);
		/* USER CODE BEGIN RTC_MspDeInit 1 */

		/* USER CODE END RTC_MspDeInit 1 */
	}

}

/**
 * @brief SD MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hsd: SD handle pointer
 * @retval None
 */
void HAL_SD_MspInit(SD_HandleTypeDef* hsd) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (hsd->Instance == SDIO) {
		/* USER CODE BEGIN SDIO_MspInit 0 */

		/* USER CODE END SDIO_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_SDIO_CLK_ENABLE()
		;

		__HAL_RCC_GPIOC_CLK_ENABLE()
		;
		__HAL_RCC_GPIOD_CLK_ENABLE()
		;
		/**SDIO GPIO Configuration
		 PC8     ------> SDIO_D0
		 PC9     ------> SDIO_D1
		 PC10     ------> SDIO_D2
		 PC11     ------> SDIO_D3
		 PC12     ------> SDIO_CK
		 PD2     ------> SDIO_CMD
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_2;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

		/* USER CODE BEGIN SDIO_MspInit 1 */

		/* USER CODE END SDIO_MspInit 1 */
	}

}

/**
 * @brief SD MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hsd: SD handle pointer
 * @retval None
 */
void HAL_SD_MspDeInit(SD_HandleTypeDef* hsd) {
	if (hsd->Instance == SDIO) {
		/* USER CODE BEGIN SDIO_MspDeInit 0 */

		/* USER CODE END SDIO_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_SDIO_CLK_DISABLE();

		/**SDIO GPIO Configuration
		 PC8     ------> SDIO_D0
		 PC9     ------> SDIO_D1
		 PC10     ------> SDIO_D2
		 PC11     ------> SDIO_D3
		 PC12     ------> SDIO_CK
		 PD2     ------> SDIO_CMD
		 */
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);

		HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

		/* USER CODE BEGIN SDIO_MspDeInit 1 */

		/* USER CODE END SDIO_MspDeInit 1 */
	}

}

/**
 * @brief SPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (hspi->Instance == SPI1) {
		/* USER CODE BEGIN SPI1_MspInit 0 */

		/* USER CODE END SPI1_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_SPI1_CLK_ENABLE()
		;

		__HAL_RCC_GPIOA_CLK_ENABLE()
		;
		/**SPI1 GPIO Configuration
		 PA4     ------> SPI1_NSS
		 PA5     ------> SPI1_SCK
		 PA6     ------> SPI1_MISO
		 PA7     ------> SPI1_MOSI
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USER CODE BEGIN SPI1_MspInit 1 */

		/* USER CODE END SPI1_MspInit 1 */
	} else if (hspi->Instance == SPI2) {
		/* USER CODE BEGIN SPI2_MspInit 0 */

		/* USER CODE END SPI2_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_SPI2_CLK_ENABLE()
		;

		__HAL_RCC_GPIOB_CLK_ENABLE()
		;
		/**SPI2 GPIO Configuration
		 PB12     ------> SPI2_NSS
		 PB13     ------> SPI2_SCK
		 PB14     ------> SPI2_MISO
		 PB15     ------> SPI2_MOSI
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_14;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* USER CODE BEGIN SPI2_MspInit 1 */

		/* USER CODE END SPI2_MspInit 1 */
	}

}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi) {
	if (hspi->Instance == SPI1) {
		/* USER CODE BEGIN SPI1_MspDeInit 0 */

		/* USER CODE END SPI1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_SPI1_CLK_DISABLE();

		/**SPI1 GPIO Configuration
		 PA4     ------> SPI1_NSS
		 PA5     ------> SPI1_SCK
		 PA6     ------> SPI1_MISO
		 PA7     ------> SPI1_MOSI
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);

		/* USER CODE BEGIN SPI1_MspDeInit 1 */

		/* USER CODE END SPI1_MspDeInit 1 */
	} else if (hspi->Instance == SPI2) {
		/* USER CODE BEGIN SPI2_MspDeInit 0 */

		/* USER CODE END SPI2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_SPI2_CLK_DISABLE();

		/**SPI2 GPIO Configuration
		 PB12     ------> SPI2_NSS
		 PB13     ------> SPI2_SCK
		 PB14     ------> SPI2_MISO
		 PB15     ------> SPI2_MOSI
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

		/* USER CODE BEGIN SPI2_MspDeInit 1 */

		/* USER CODE END SPI2_MspDeInit 1 */
	}

}

/**
 * @brief TIM_Base MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
	if (htim_base->Instance == TIM2) {
		/* USER CODE BEGIN TIM2_MspInit 0 */

		/* USER CODE END TIM2_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_TIM2_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**TIM2 GPIO Configuration
		 PA0-WKUP     ------> TIM2_ETR
		 */

		/* USER CODE BEGIN TIM2_MspInit 1 */

		/* USER CODE END TIM2_MspInit 1 */
	}
}

/**
 * @brief TIM_PWM MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_pwm: TIM_PWM handle pointer
 * @retval None
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm) {
	if (htim_pwm->Instance == TIM3) {
		/* USER CODE BEGIN TIM3_MspInit 0 */

		/* USER CODE END TIM3_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_TIM3_CLK_ENABLE()
		;
		/* USER CODE BEGIN TIM3_MspInit 1 */

		/* USER CODE END TIM3_MspInit 1 */
	}

}

/**
 * @brief TIM_OC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_oc: TIM_OC handle pointer
 * @retval None
 */
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* htim_oc) {
	if (htim_oc->Instance == TIM8) {
		/* USER CODE BEGIN TIM8_MspInit 0 */

		/* USER CODE END TIM8_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_TIM8_CLK_ENABLE()
		;
		/* USER CODE BEGIN TIM8_MspInit 1 */

		/* USER CODE END TIM8_MspInit 1 */
	}

}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (htim->Instance == TIM2) {
		/* USER CODE BEGIN TIM2_MspPostInit 0 */

		/* USER CODE END TIM2_MspPostInit 0 */
		__HAL_RCC_GPIOA_CLK_ENABLE()
		;
		/**TIM2 GPIO Configuration
		 PA1     ------> TIM2_CH2
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USER CODE BEGIN TIM2_MspPostInit 1 */

		/* USER CODE END TIM2_MspPostInit 1 */
	} else if (htim->Instance == TIM3) {
		/* USER CODE BEGIN TIM3_MspPostInit 0 */

		/* USER CODE END TIM3_MspPostInit 0 */

		__HAL_RCC_GPIOB_CLK_ENABLE()
		;
		/**TIM3 GPIO Configuration
		 PB5     ------> TIM3_CH2
		 */
		GPIO_InitStruct.Pin = BACK_LIGHT_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(BACK_LIGHT_GPIO_Port, &GPIO_InitStruct);

		__HAL_AFIO_REMAP_TIM3_PARTIAL()
		;

		/* USER CODE BEGIN TIM3_MspPostInit 1 */

		/* USER CODE END TIM3_MspPostInit 1 */
	} else if (htim->Instance == TIM8) {
		/* USER CODE BEGIN TIM8_MspPostInit 0 */

		/* USER CODE END TIM8_MspPostInit 0 */

		__HAL_RCC_GPIOC_CLK_ENABLE()
		;
		/**TIM8 GPIO Configuration
		 PC6     ------> TIM8_CH1
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		/* USER CODE BEGIN TIM8_MspPostInit 1 */

		/* USER CODE END TIM8_MspPostInit 1 */
	}

}
/**
 * @brief TIM_PWM MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_pwm: TIM_PWM handle pointer
 * @retval None
 */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm) {
	if (htim_pwm->Instance == TIM3) {
		/* USER CODE BEGIN TIM3_MspDeInit 0 */

		/* USER CODE END TIM3_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM3_CLK_DISABLE();
		/* USER CODE BEGIN TIM3_MspDeInit 1 */

		/* USER CODE END TIM3_MspDeInit 1 */
	}

}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (huart->Instance == USART1) {
		/* USER CODE BEGIN USART1_MspInit 0 */

		/* USER CODE END USART1_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_USART1_CLK_ENABLE()
		;

		__HAL_RCC_GPIOA_CLK_ENABLE()
		;
		/**USART1 GPIO Configuration
		 PA9     ------> USART1_TX
		 PA10     ------> USART1_RX
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USER CODE BEGIN USART1_MspInit 1 */

		/* USER CODE END USART1_MspInit 1 */
	}

}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
	if (huart->Instance == USART1) {
		/* USER CODE BEGIN USART1_MspDeInit 0 */

		/* USER CODE END USART1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USART1_CLK_DISABLE();

		/**USART1 GPIO Configuration
		 PA9     ------> USART1_TX
		 PA10     ------> USART1_RX
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);

		/* USER CODE BEGIN USART1_MspDeInit 1 */

		/* USER CODE END USART1_MspDeInit 1 */
	}

}

static uint32_t FSMC_Initialized = 0;

static void HAL_FSMC_MspInit(void) {
	/* USER CODE BEGIN FSMC_MspInit 0 */

	/* USER CODE END FSMC_MspInit 0 */
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (FSMC_Initialized) {
		return;
	}
	FSMC_Initialized = 1;

	/* Peripheral clock enable */
	__HAL_RCC_FSMC_CLK_ENABLE()
	;

	/** FSMC GPIO Configuration
	 PE7   ------> FSMC_D4
	 PE8   ------> FSMC_D5
	 PE9   ------> FSMC_D6
	 PE10   ------> FSMC_D7
	 PE11   ------> FSMC_D8
	 PE12   ------> FSMC_D9
	 PE13   ------> FSMC_D10
	 PE14   ------> FSMC_D11
	 PE15   ------> FSMC_D12
	 PD8   ------> FSMC_D13
	 PD9   ------> FSMC_D14
	 PD10   ------> FSMC_D15
	 PD11   ------> FSMC_A16
	 PD14   ------> FSMC_D0
	 PD15   ------> FSMC_D1
	 PD0   ------> FSMC_D2
	 PD1   ------> FSMC_D3
	 PD4   ------> FSMC_NOE
	 PD5   ------> FSMC_NWE
	 PD7   ------> FSMC_NE1
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5
			| GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/* USER CODE BEGIN FSMC_MspInit 1 */

	/* USER CODE END FSMC_MspInit 1 */
}

void HAL_SRAM_MspInit(SRAM_HandleTypeDef* hsram) {
	/* USER CODE BEGIN SRAM_MspInit 0 */

	/* USER CODE END SRAM_MspInit 0 */
	HAL_FSMC_MspInit();
	/* USER CODE BEGIN SRAM_MspInit 1 */

	/* USER CODE END SRAM_MspInit 1 */
}

static uint32_t FSMC_DeInitialized = 0;

static void HAL_FSMC_MspDeInit(void) {
	/* USER CODE BEGIN FSMC_MspDeInit 0 */

	/* USER CODE END FSMC_MspDeInit 0 */
	if (FSMC_DeInitialized) {
		return;
	}
	FSMC_DeInitialized = 1;
	/* Peripheral clock enable */
	__HAL_RCC_FSMC_CLK_DISABLE();

	/** FSMC GPIO Configuration
	 PE7   ------> FSMC_D4
	 PE8   ------> FSMC_D5
	 PE9   ------> FSMC_D6
	 PE10   ------> FSMC_D7
	 PE11   ------> FSMC_D8
	 PE12   ------> FSMC_D9
	 PE13   ------> FSMC_D10
	 PE14   ------> FSMC_D11
	 PE15   ------> FSMC_D12
	 PD8   ------> FSMC_D13
	 PD9   ------> FSMC_D14
	 PD10   ------> FSMC_D15
	 PD11   ------> FSMC_A16
	 PD14   ------> FSMC_D0
	 PD15   ------> FSMC_D1
	 PD0   ------> FSMC_D2
	 PD1   ------> FSMC_D3
	 PD4   ------> FSMC_NOE
	 PD5   ------> FSMC_NWE
	 PD7   ------> FSMC_NE1
	 */
	HAL_GPIO_DeInit(GPIOE, GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

	HAL_GPIO_DeInit(GPIOD,
	GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7);

	/* USER CODE BEGIN FSMC_MspDeInit 1 */

	/* USER CODE END FSMC_MspDeInit 1 */
}

void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef* hsram) {
	/* USER CODE BEGIN SRAM_MspDeInit 0 */

	/* USER CODE END SRAM_MspDeInit 0 */
	HAL_FSMC_MspDeInit();
	/* USER CODE BEGIN SRAM_MspDeInit 1 */

	/* USER CODE END SRAM_MspDeInit 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
