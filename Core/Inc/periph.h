#ifndef __PERIFINIT_H__
#define __PERIFINIT_H__

#include <string.h>

#define PIN0                    0
#define PIN1                    1
#define PIN2                    2
#define PIN3                    3
#define PIN4                    4
#define PIN5                    5
#define PIN6                    6
#define PIN7                    7
#define PIN8                    8
#define PIN9                    9
#define PIN10                   10
#define PIN11                   11
#define PIN12                   12
#define PIN13                   13
#define PIN14                   14
#define PIN15                   15

#define GPIOA_OFFSET            (GPIOA_BASE - PERIPH_BASE)
#define GPIOB_OFFSET            (GPIOB_BASE - PERIPH_BASE)
#define GPIOC_OFFSET            (GPIOC_BASE - PERIPH_BASE)
#define GPIOD_OFFSET            (GPIOD_BASE - PERIPH_BASE)
#define GPIOE_OFFSET            (GPIOE_BASE - PERIPH_BASE)
#define GPIOF_OFFSET            (GPIOF_BASE - PERIPH_BASE)

#define DMA1_Channel3_OFFSET    (DMA1_Channel3_BASE - PERIPH_BASE)

#define TIM2_OFFSET             (TIM2_BASE - PERIPH_BASE)
#define TIM3_OFFSET             (TIM3_BASE - PERIPH_BASE)

// bit bang access
// GRIO pins

#define GPIO_IDR                0x8
#define GPIO_ODR                0xC

#define LED1                    *(uint32_t*)(PERIPH_BB_BASE + ((GPIOB_OFFSET + GPIO_ODR) * 32) + (PIN0 * 4))
#define LED2                    *(uint32_t*)(PERIPH_BB_BASE + ((GPIOB_OFFSET + GPIO_ODR) * 32) + (PIN1 * 4))

#define USB_EN                  *(uint32_t*)(PERIPH_BB_BASE + ((GPIOB_OFFSET + GPIO_ODR) * 32) + (PIN7 * 4))
#define COOLER_EN               *(uint32_t*)(PERIPH_BB_BASE + ((GPIOD_OFFSET + GPIO_ODR) * 32) + (PIN13 * 4))

#define TOUCH_IRQ               *(uint32_t*)(PERIPH_BB_BASE + ((GPIOB_OFFSET + GPIO_IDR) * 32) + (PIN6 * 4))

#define USER_KEYA               *(uint32_t*)(PERIPH_BB_BASE + ((GPIOC_OFFSET + GPIO_IDR) * 32) + (PIN13 * 4))
#define USER_KEYB               *(uint32_t*)(PERIPH_BB_BASE + ((GPIOB_OFFSET + GPIO_IDR) * 32) + (PIN2 * 4))

#define ZERO_CROSS              *(uint32_t*)(PERIPH_BB_BASE + ((GPIOA_OFFSET + GPIO_IDR) * 32) + (PIN0 * 4))

#define TC1_CS                  *(uint32_t*)(PERIPH_BB_BASE + ((GPIOB_OFFSET + GPIO_ODR) * 32) + (PIN12 * 4))
#define TC2_CS                  *(uint32_t*)(PERIPH_BB_BASE + ((GPIOD_OFFSET + GPIO_ODR) * 32) + (PIN12 * 4))

#define KEY1                    *(uint32_t*)(PERIPH_BB_BASE + ((GPIOE_OFFSET + GPIO_IDR) * 32) + (PIN3 * 4))
#define KEY2                    *(uint32_t*)(PERIPH_BB_BASE + ((GPIOE_OFFSET + GPIO_IDR) * 32) + (PIN2 * 4))
#define KEY3                    *(uint32_t*)(PERIPH_BB_BASE + ((GPIOE_OFFSET + GPIO_IDR) * 32) + (PIN1 * 4))
#define KEYS_GND                *(uint32_t*)(PERIPH_BB_BASE + ((GPIOE_OFFSET + GPIO_ODR) * 32) + (PIN0 * 4))

// TIM2 CR1 enable bit
#define TIM2_CR1_CEN            *(uint32_t*)(PERIPH_BB_BASE + (TIM2_OFFSET * 32) + (PIN0 * 4))

// TIM3 CR1 enable bit
#define TIM3_CR1_CEN            *(uint32_t*)(PERIPH_BB_BASE + (TIM3_OFFSET * 32) + (PIN0 * 4))

// DMA Channel3 (SPI) enable bit
#define SPI_DMA_EN              *(uint32_t*)(PERIPH_BB_BASE + (DMA1_Channel3_OFFSET * 32) + (PIN0 * 4))

#endif // __PERIFINIT_H__