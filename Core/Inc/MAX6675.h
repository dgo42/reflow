/**
 * Dimitrij Goldstein (dgo42) 2020
 * This library uses the MAX6675 Converter to measure the temperature using a thermocouple K-type.
 *
 * This library is written for Stm32f4 microcontrollers using the stm32-HAL software library.
 * The usage of software SPI in this library is due to issues with reading data using the SPI-HAL functions.
 */
#ifndef MAX6675_H
#define MAX6675_H

/********************* MAX31865_GPIO pinout struct *********************/
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
} MAX6675_GPIO;

extern MAX6675_GPIO max6675_gpio;

/********************* Public functions *********************/
float MAX6675_readTemp(void);
void MAX6675_init(MAX6675_GPIO *max_gpio);
#endif //MAX6675_H
