/**
 * Olivier Van den Eede (4ilo) 2019
 */
#include "stdint.h"
#include "stm32f103xe.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"
#include "cmsis_os.h"
#include "MAX31865.h"

/*********************** Global variables *************************/
#define DELAY(x) osDelay(x)

MAX31865_GPIO *_max31865_gpio;

static void spi_write(uint8_t *buffer, uint8_t len)
{
    uint8_t x, temp;
    for (x = 0; x < len; x++)
    {
        HAL_SPI_TransmitReceive(_max31865_gpio->SPI, (uint8_t *)&buffer[x], (uint8_t *)&temp, 1, 50);
    }
}

static void spi_read(uint8_t *buffer, uint8_t len)
{
    uint8_t x, temp = 0;
    for (x = 0; x < len; x++)
    {
        buffer[x] = 0;
        HAL_SPI_TransmitReceive(_max31865_gpio->SPI, (uint8_t *)&temp, (uint8_t *)&buffer[x], 1, 50);
    }
}

/**
 * Read x bytes from MAX31865 starting from addr
 *
 * @param addr      Register addr to read from
 * @param buffer    Pointer to rx buffer
 * @param len       Amount of bytes to read
 */
void MAX31865_read(uint8_t addr, uint8_t *buffer, uint8_t len)
{
    addr &= ~MAX31865_READ; // Force read bit on address

    HAL_GPIO_WritePin(_max31865_gpio->CE_PORT, _max31865_gpio->CE_PIN, GPIO_PIN_RESET); // Enable CE

    spi_write(&addr, 1);   // Write addr
    spi_read(buffer, len); // Read data

    HAL_GPIO_WritePin(_max31865_gpio->CE_PORT, _max31865_gpio->CE_PIN, GPIO_PIN_SET); // Disable CE
}

/**
 * Write a byte in a MAX13865 register
 *
 * @param addr      Register addr to write to
 * @param buffer    Tx data
 */
void MAX31865_write(uint8_t addr, uint8_t data)
{
    addr |= MAX31865_WRITE; // Force write bit on address

    HAL_GPIO_WritePin(_max31865_gpio->CE_PORT, _max31865_gpio->CE_PIN, GPIO_PIN_RESET); // Enable CE

    spi_write(&addr, 1); // Write addr
    spi_write(&data, 1); // Write data

    HAL_GPIO_WritePin(_max31865_gpio->CE_PORT, _max31865_gpio->CE_PIN, GPIO_PIN_SET); // Disable CE
}

/**
 * Enable of disable MAX831865 bias voltage
 * @param enable Enable of disable
 */
void enableBias(uint8_t enable)
{
    uint8_t status;
    MAX31865_read(MAX31856_CONFIG_REG, &status, 1);

    if (enable)
    {
        status |= MAX31856_CONFIG_BIAS;
    }
    else
    {
        status &= ~MAX31856_CONFIG_BIAS;
    }

    MAX31865_write(MAX31856_CONFIG_REG, status);
}

/**
 * Enable of disable MAX831865 auto convert
 * @param enable Enable of disable
 */
void autoConvert(uint8_t enable)
{
    uint8_t status;
    MAX31865_read(MAX31856_CONFIG_REG, &status, 1);

    if (enable)
    {
        status |= MAX31856_CONFIG_MODEAUTO;
    }
    else
    {
        status &= ~MAX31856_CONFIG_MODEAUTO;
    }

    MAX31865_write(MAX31856_CONFIG_REG, status);
}

/**
 * Set the amount of wires the temperature sensor uses
 * @param numwires 2,3 or 4 wires
 */
void setWires(uint8_t numwires)
{
    uint8_t status;
    MAX31865_read(MAX31856_CONFIG_REG, &status, 1);

    if (numwires == 3) // 3-wire
    {
        status |= MAX31856_CONFIG_3WIRE;
    }
    else // 2-4 wire
    {
        status &= ~MAX31856_CONFIG_3WIRE;
    }

    MAX31865_write(MAX31856_CONFIG_REG, status);
}

/**
 * Perform a single shot conversion
 */
void single_shot(void)
{
    uint8_t status;

    // Read config register
    MAX31865_read(MAX31856_CONFIG_REG, &status, 1);

    // Enable 1shot bit, and write back
    status |= MAX31856_CONFIG_1SHOT;
    MAX31865_write(MAX31856_CONFIG_REG, status);
}

/**
 * Perform a single temperature conversion, and calculate the value
 *
 * @return  Temperature as float
 */
float MAX31865_singleReadTemp()
{
    uint8_t buffer[2];
    uint16_t data;
    float resistance;
    float temp;

    // Perform a single conversion, and wait for the result
    single_shot();
    osDelay(21);
    // Read data from max31865 data registers
    MAX31865_read(MAX31856_RTDMSB_REG, buffer, 2);

    // Combine 2 bytes into 1 number, and shift 1 down to remove fault bit
    data = buffer[0] << 8;
    data |= buffer[1];
    data >>= 1;

    // Calculate the actual resistance of the sensor
    resistance = ((float)data * RREF) / FACTOR;

    // Calculate the temperature from the measured resistance
    temp = ((resistance / 100) - 1) / ALPHA;

    // Disable bias voltage to reduce power usage
    return temp;
}
/*********************** End Private functions *************************/

/*********************** Begin Public functions *************************/

/**
 * Initialise MAX31865 for single shot temperature conversion
 *
 * @param max_gpio  MAX31865_GPIO structure with pinout
 * @param wires     Amount of wires on the temperature probe (2,3 or 4)
 */
void MAX31865_init(MAX31865_GPIO *max_gpio, uint8_t wires)
{
    _max31865_gpio = max_gpio;

    // Datalines in reset state
    HAL_GPIO_WritePin(_max31865_gpio->CE_PORT, _max31865_gpio->CE_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(_max31865_gpio->CLK_PORT, _max31865_gpio->CLK_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(_max31865_gpio->MOSI_PORT, _max31865_gpio->MOSI_PIN, GPIO_PIN_SET);

    setWires(wires); // Set 2,3 or 4 wire sensor
    enableBias(ON);  // Disable bias voltage
    autoConvert(ON); // Disable auto conversion
}

#define LOOPS 5

float MAX31865_readTemp()
{
    float result = 0;
    uint8_t i = 0;
    // Activate bias voltage to read sensor data, and wait for the capacitors to fill
    //enableBias(ON);
    //osDelay(10);

    for (i = 0; i < LOOPS; i++)
    {
        result += MAX31865_singleReadTemp();
    }
    result /= LOOPS;

    //enableBias(OFF);
    return result;
}

/*********************** Begin Public functions *************************/