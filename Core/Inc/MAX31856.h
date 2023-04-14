/**
 * Dimitrij Goldstein (dgo42) 2020
 * This library uses the MAX6675 Converter to measure the temperature using a thermocouple K-type.
 *
 * This library is written for Stm32f4 microcontrollers using the stm32-HAL software library.
 * The usage of software SPI in this library is due to issues with reading data using the SPI-HAL functions.
 */
#ifndef MAX31856_H
#define MAX31856_H

/********************* MAX31865_GPIO pinout struct *********************/
typedef struct {
    GPIO_TypeDef *CE_PORT;
    uint16_t CE_PIN;

    SPI_HandleTypeDef *SPI;
	
} MAX31856_GPIO;

/********************* Public functions *********************/

// CR0 flags
#define MAX31856_SINGLE_SHOT 0x0
#define MAX31856_CONTINOUS 0x80 // continously every 100ms (with 1 sample, each sample adds 16.6ms (60Hz) or 20ms (50Hz))

#define MAX31856_START 0x40

#define MAX31856_OPENCIRCUIT_NONE 0x0
#define MAX31856_OPENCIRCUIT_LESS_5K 0x10
#define MAX31856_OPENCIRCUIT_BETWEEN_5K_40K_2ms 0x20
#define MAX31856_OPENCIRCUIT_BETWEEN_5K_40K_ABOVE_2ms 0x30

#define MAX31856_COLDJUNCTION_ENABLE 0x0
#define MAX31856_COLDJUNCTION_DISABLE 0x8

#define MAX31856_FAULT_COMPARATOR_MODE 0x0
#define MAX31856_FAULT_INTERRUPT_MODE 0x4

#define MAX31856_FAULT_INTERRUPT_CLEAR 0x2

#define MAX31856_FILTER_60HZ 0x0
#define MAX31856_FILTER_50HZ 0x1

// CR1 flags
#define MAX31856_BTYPE 0x0
#define MAX31856_ETYPE 0x1
#define MAX31856_JTYPE 0x2
#define MAX31856_KTYPE 0x3
#define MAX31856_NTYPE 0x4
#define MAX31856_RTYPE 0x5
#define MAX31856_STYPE 0x6
#define MAX31856_TTYPE 0x7
#define MAX31856_VMODE_x8 0x8 // voltage mode gain 8
#define MAX31856_VMODE_x32 0xC // voltage mode gain 32

#define MAX31856_AV1 0x0 // 1 sample
#define MAX31856_AV2 0x10 // 2 sample average
#define MAX31856_AV4 0x20 // 4 sample average
#define MAX31856_AV8 0x30 // 8 sample average
#define MAX31856_AV16 0x40 // 16 sample average

// mask register
#define MAX31856_CJ_HIGH_FAULT_MASK 0x20
#define MAX31856_CJ_LOW_FAULT_MASK 0x10
#define MAX31856_TC_HIGH_FAULT_MASK 0x8
#define MAX31856_TC_LOW_FAULT_MASK 0x4
#define MAX31856_OV_FAULT_MASK 0x2
#define MAX31856_OC_FAULT_MASK 0x1

// status register
#define MAX31856_CJ_RANGE 0x80
#define MAX31856_TC_RANGE 0x40
#define MAX31856_CJ_HIGH 0x20
#define MAX31856_CJ_LOW 0x10
#define MAX31856_TC_HIGH 0x8
#define MAX31856_TC_LOW 0x4
#define MAX31856_OV 0x2
#define MAX31856_OC 0x1

void MAX31856_config(MAX31856_GPIO *gpio, uint8_t cr0, uint8_t cr1);
void MAX31856_setFaultMask(MAX31856_GPIO *gpio, uint8_t mask);
void MAX31856_setCJThreshold(MAX31856_GPIO *gpio, uint8_t lowThr, uint8_t highThr);
void MAX31856_setCJOffset(MAX31856_GPIO *gpio, float offset);
void MAX31856_setThreshold(MAX31856_GPIO *gpio, float lowThr, float highThr);
float MAX31856_getThresholdLow(MAX31856_GPIO *gpio);
float MAX31856_getThresholdHigh(MAX31856_GPIO *gpio);
float MAX31856_readCJ(MAX31856_GPIO *gpio);
void MAX31856_writeCJ(MAX31856_GPIO *gpio, float temp);
float MAX31856_readTC(MAX31856_GPIO *gpio);
uint8_t MAX31856_readFault(MAX31856_GPIO *gpio);

#endif //MAX31856_H
