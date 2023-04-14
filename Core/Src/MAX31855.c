/**
 * Olivier Van den Eede (4ilo) 2019
 */
#include "stdint.h"
#include "stm32f103xe.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"
#include "cmsis_os.h"
#include "MAX31855.h"


/*********************** Global variables *************************/
#define DELAY(x) osDelay(x)

MAX31855_GPIO *_max31855_gpio;

/**
 * Read x bytes from MAX31865 starting from addr
 *
 * @param addr      Register addr to read from
 * @param buffer    Pointer to rx buffer
 * @param len       Amount of bytes to read
 */
uint32_t MAX31855_read()
{
    uint16_t temp = 0;
    uint16_t result[2] = {0};

    HAL_GPIO_WritePin(_max31855_gpio->CE_PORT, _max31855_gpio->CE_PIN, GPIO_PIN_RESET);          // Enable CE
   	HAL_SPI_TransmitReceive(_max31855_gpio->SPI, (uint8_t *) &temp, (uint8_t *) &result[0], 2, 250);
    HAL_GPIO_WritePin(_max31855_gpio->CE_PORT, _max31855_gpio->CE_PIN, GPIO_PIN_SET);          // Disable CE

    return (uint32_t)(result[1] << 16) + (uint32_t)result[0];
}

/**
 * Perform a single temperature conversion, and calculate the value
 *
 * @return  Temperature as float
 */
float MAX31855_singleReadTemp()
{
	uint32_t data = MAX31855_read();
    if ((data & (uint32_t)0x10005) != 0) {
        // fault
        return 0.;
    }
    float temp = (float)(0x1fff & (data >> 18)) / 4.;
    //float temp = (float)(0x7ff & (data >> 3)) / 4.;
    return temp;
}
/*********************** End Private functions *************************/


/*********************** Begin Public functions *************************/

float MAX31855_readTemp()
{
    float result = 0;
    result = MAX31855_singleReadTemp();
    return result;
}

void MAX31855_init(MAX31855_GPIO *max_gpio)
{
    _max31855_gpio = max_gpio;
}

/*********************** Begin Public functions *************************/