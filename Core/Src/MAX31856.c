/**
 * Olivier Van den Eede (4ilo) 2019
 */
#include "stdint.h"
#include "stm32f103xe.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"
#include "cmsis_os.h"
#include "MAX31856.h"

#define CR0 0x0
#define CR1 0x1
#define MASK 0x2
#define CJ_HIGH_TH 0x3
#define CJ_LOW_TH 0x4
#define TC_HIGH_MSB 0x5
#define TC_HIGH_LSB 0x6
#define TC_LOW_MSB 0x7
#define TC_LOW_LSB 0x8
#define CJ_OFFSET 0x9
#define CJ_HIGH 0xA
#define CJ_LOW 0xB
#define TC_B2 0xC
#define TC_B1 0xD
#define TC_B0 0xE
#define FAULT 0xF

/*********************** Global variables *************************/
#define DELAY(x) osDelay(x)

void MAX31856_readMultibyte(MAX31856_GPIO *gpio, uint8_t addr, uint8_t *data, uint8_t len);
void MAX31856_writeMultibyte(MAX31856_GPIO *gpio, uint8_t addr, uint8_t *data, uint8_t len);

void MAX31856_readMultibyte(MAX31856_GPIO *gpio, uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t l_addr = addr & 0x7F;
    uint8_t temp = 0;

    HAL_GPIO_WritePin(gpio->CE_PORT, gpio->CE_PIN, GPIO_PIN_RESET); // Enable CE
    HAL_SPI_Transmit(gpio->SPI, &l_addr, 1, 50);
    HAL_SPI_TransmitReceive(gpio->SPI, &temp, data, len, 50);
    HAL_GPIO_WritePin(gpio->CE_PORT, gpio->CE_PIN, GPIO_PIN_SET); // Disable CE
}

void MAX31856_writeMultibyte(MAX31856_GPIO *gpio, uint8_t addr, uint8_t *data, uint8_t len)
{
    uint8_t l_addr = addr | 0x80;

    HAL_GPIO_WritePin(gpio->CE_PORT, gpio->CE_PIN, GPIO_PIN_RESET); // Enable CE
    HAL_SPI_Transmit(gpio->SPI, &l_addr, 1, 50);
    HAL_SPI_Transmit(gpio->SPI, data, len, 50);
    HAL_GPIO_WritePin(gpio->CE_PORT, gpio->CE_PIN, GPIO_PIN_SET); // Disable CE
}

void MAX31856_config(MAX31856_GPIO *gpio, uint8_t cr0, uint8_t cr1)
{
    MAX31856_writeMultibyte(gpio, CR0, &cr0, 1);
    MAX31856_writeMultibyte(gpio, CR1, &cr1, 1);
}

void MAX31856_setFaultMask(MAX31856_GPIO *gpio, uint8_t mask)
{
    MAX31856_writeMultibyte(gpio, MASK, &mask, 1);
}

void MAX31856_setCJThreshold(MAX31856_GPIO *gpio, uint8_t lowThr, uint8_t highThr)
{
    MAX31856_writeMultibyte(gpio, CJ_HIGH_TH, (uint8_t *)&highThr, 1);
    MAX31856_writeMultibyte(gpio, CJ_LOW_TH, (uint8_t *)&lowThr, 1);
}

void MAX31856_setCJOffset(MAX31856_GPIO *gpio, float offset)
{
    uint8_t l_offset = (uint8_t)(offset * 16);
    MAX31856_writeMultibyte(gpio, CJ_OFFSET, &l_offset, 1);
}

void MAX31856_setThreshold(MAX31856_GPIO *gpio, float lowThr, float highThr)
{
    uint16_t l_thr = (uint16_t)(lowThr * 16);
    uint8_t data[2] = { l_thr >> 8, l_thr & 0xFF };
    MAX31856_writeMultibyte(gpio, TC_LOW_MSB, (uint8_t *)data, 2);

    l_thr = (uint16_t)(highThr * 16);
    data[0] = l_thr >> 8;
    data[1] = l_thr & 0xFF;
    MAX31856_writeMultibyte(gpio, TC_HIGH_MSB, (uint8_t *)&data, 2);
}

float MAX31856_getThresholdLow(MAX31856_GPIO *gpio)
{
    uint8_t l_thr[2] = {0};
    float result = 0;
    MAX31856_readMultibyte(gpio, TC_LOW_MSB, (uint8_t *)l_thr, 2);
    result = (float)(((uint16_t)l_thr[0] << 8) + (uint16_t)l_thr[1]) / 16.f;
    return result;
}

float MAX31856_getThresholdHigh(MAX31856_GPIO *gpio)
{
    uint8_t l_thr[2] = {0};
    float result = 0;
    MAX31856_readMultibyte(gpio, TC_HIGH_MSB, (uint8_t *)l_thr, 2);
    result = (float)(((uint16_t)l_thr[0] << 8) + (uint16_t)l_thr[1]) / 16.f;
    return result;
}

float MAX31856_readCJ(MAX31856_GPIO *gpio)
{
    uint8_t l_cj[2] = {0};
    float result = 0;
    MAX31856_readMultibyte(gpio, CJ_HIGH, (uint8_t *)&l_cj[0], 2);
    uint16_t val = ((uint16_t)l_cj[0] << 6) + (uint16_t)(l_cj[1] >> 2);
    result = (float)val / 64.f;
    return result;
}

void MAX31856_writeCJ(MAX31856_GPIO *gpio, float temp)
{
    uint16_t l_value = (uint16_t)(temp * 256.f) & 0xFFC0;
    uint8_t l_cj[2] = { l_value & 0xFF, l_value >> 8 };
    MAX31856_writeMultibyte(gpio, CJ_HIGH, (uint8_t *)&l_cj[1], 1);
    MAX31856_writeMultibyte(gpio, CJ_LOW, (uint8_t *)&l_cj[0], 1);
}

float MAX31856_readTC(MAX31856_GPIO *gpio)
{
    uint8_t l_tc[3] = {0};
    float result = 0;
    MAX31856_readMultibyte(gpio, TC_B2, (uint8_t *)l_tc, 3);
    uint32_t val = (float)(((uint32_t)l_tc[0] << 11) + ((uint32_t)l_tc[1] << 3) + (uint32_t)(l_tc[2] >> 5));
    result = val / 128.f;
    return result;
}

uint8_t MAX31856_readFault(MAX31856_GPIO *gpio)
{
    uint8_t l_fault = 0;
    MAX31856_readMultibyte(gpio, FAULT, &l_fault, 1);
    return l_fault;
}

/*********************** Begin Public functions *************************/