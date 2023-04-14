/**
 * Olivier Van den Eede (4ilo) 2019
 */
#include "stdint.h"
#include "stm32f103xe.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"
#include "cmsis_os.h"
#include "MAX6675.h"


/*********************** Global variables *************************/
#define DELAY(x) osDelay(x)

MAX6675_GPIO *_max6675_gpio;

/**
 * Read x bytes from MAX31865 starting from addr
 *
 * @param addr      Register addr to read from
 * @param buffer    Pointer to rx buffer
 * @param len       Amount of bytes to read
 */
uint16_t MAX6675_read()
{
    uint16_t temp = 0;
    uint16_t result = 0;

    HAL_GPIO_WritePin(_max6675_gpio->CE_PORT, _max6675_gpio->CE_PIN, GPIO_PIN_RESET);          // Enable CE
   	HAL_SPI_TransmitReceive(_max6675_gpio->SPI, (uint8_t *) &temp, (uint8_t *) &result, 1, 250);
   	//HAL_SPI_Receive(_max6675_gpio->SPI, (uint8_t *) &result, 1, 500);
    //for(temp = 0; temp < 100; temp++) {}
    HAL_GPIO_WritePin(_max6675_gpio->CE_PORT, _max6675_gpio->CE_PIN, GPIO_PIN_SET);          // Disable CE

    return result;
}

/**
 * Perform a single temperature conversion, and calculate the value
 *
 * @return  Temperature as float
 */
float MAX6675_singleReadTemp()
{
	uint16_t data = MAX6675_read();
    float temp = (float)(0x7ff & (data >> 3)) / 4;
    return temp;
}
/*********************** End Private functions *************************/


/*********************** Begin Public functions *************************/

float MAX6675_readTemp()
{
    float result = 0;
    result = MAX6675_singleReadTemp();
    return result;
}

void MAX6675_init(MAX6675_GPIO *max_gpio)
{
    _max6675_gpio = max_gpio;
}

/*********************** Begin Public functions *************************/