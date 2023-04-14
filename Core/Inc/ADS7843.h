#ifndef ADS7843_H
#define ADS7843_H

#include "Global.h"

void ADS7843SendCmd(U8 Data);     // Sends a 8-bit command to the peripheral
U16 ADS7843GetResult(void);        // Retrieves the result of the AD conversion. 4 dummy bytes have to be shifted out to the left.
char ADS7843GetBusy(void);        // Retrieves the status of the busy line. 0: Not busy; 1: Busy
void ADS7843SetCS(char OnOff);
char ADS7843GetPENIRQ(void);        // Retrieves the status of the PENIRQ line to detect a touch event.

void _ExecCalibration(int renew);
void _TestCalibration(void);

typedef struct POINT {
	uint16_t x;
	uint16_t y;
} Coordinate;

#define TP_CS(a)	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, (GPIO_PinState)a)

#define TP_INT_IN   HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)

#endif /* ADS7843_H */

