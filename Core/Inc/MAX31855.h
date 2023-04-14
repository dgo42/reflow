/**
 * Dimitrij Goldstein (dgo42) 2020
 * This library uses the MAX6675 Converter to measure the temperature using a thermocouple K-type.
 *
 * This library is written for Stm32f4 microcontrollers using the stm32-HAL software library.
 * The usage of software SPI in this library is due to issues with reading data using the SPI-HAL functions.
 */
#ifndef MAX31855_H
#define MAX31855_H

/********************* MAX31855_GPIO pinout struct *********************/
typedef struct {
    GPIO_TypeDef *CE_PORT;
    uint16_t CE_PIN;

    SPI_HandleTypeDef *SPI;
	
    GPIO_TypeDef *CLK_PORT;
    uint16_t CLK_PIN;

    GPIO_TypeDef *MOSI_PORT;
    uint16_t MOSI_PIN;

    GPIO_TypeDef *MISO_PORT;
    uint16_t MISO_PIN;
} MAX31855_GPIO;

extern MAX31855_GPIO max31855_gpio;

/********************* Public functions *********************/
float MAX31855_readTemp(void);
void MAX31855_init(MAX31855_GPIO *max_gpio);
#endif //MAX31855_H
