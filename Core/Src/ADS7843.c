/*
 * ADS7843.c
 *
 *  Created on: 17.02.2020
 *      Author: dgo
 */

#include "GUI.h"
#include "main.h"
#include "cmsis_os.h"
#include <sys/_stdint.h>
#include <ADS7843.h>
#include <stm32f1xx.h>
#include <stm32f1xx_hal_spi.h>
#include <stm32f1xx_hal_flash.h>
#include <stm32f1xx_hal_flash_ex.h>
#include "GUITDRV_ADS7846.h"
#include "BUTTON.h"

extern SPI_HandleTypeDef hspi1;

#ifdef USE_Delay
#include "main.h"
#define _delay_ Delay /* !< User can provide more timing precise _delay_ function \ \ \ \
							  (with 10ms time base), using SysTick for example */
#else
/**
 * @brief  Inserts a delay time.
 * @param  nCount: specifies the delay time length.
 * @retval None
 */
void ts_delay(__IO uint32_t nCount)
{
	uint16_t i, us;
	for (i = 0; i < nCount; i++)
	{
		us = 1;
		while (us--)
		{
		}
	}
}
#define _delay_ ts_delay /* !< Default _delay_ function with less precise timing */
#endif

/* Private typedef -----------------------------------------------------------*/

extern int CalibrationComplete;
extern float temp;
extern int prevPressed;
extern int nextPressed;
extern int enterPressed;

/* Private define ------------------------------------------------------------*/
#define THRESHOLD_CALIBRATE 2
#define THRESHOLD 2

#define CHX 0x90
#define CHY 0xd0

// Sends a 8-bit command to the peripheral
void ADS7843SendCmd(U8 Data)
{
	uint8_t temp;
	_delay_(1);
	/* Send SPI1 data */
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&Data, &temp, 1, 50);
}

// Retrieves the result of the AD conversion. 4 dummy bytes have to be shifted out to the left.
U16 ADS7843GetResult(void)
{
	unsigned short buf, data = 0, temp;
	/* Read SPI1 received data */
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&data, (uint8_t *)&temp, 1, 50);
	buf = temp << 8;
	_delay_(1);
	/* Read SPI1 received data */
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&data, (uint8_t *)&temp, 1, 50);
	buf |= temp;
	buf >>= 3;
	buf &= 0xfff;
	return buf;
}

// Retrieves the status of the busy line. 0: Not busy; 1: Busy
char ADS7843GetBusy(void)
{
	return 0;
}

void ADS7843SetCS(char OnOff)
{
	TP_CS(!OnOff);
}

// Retrieves the status of the PENIRQ line to detect a touch event.
char ADS7843GetPENIRQ(void)
{
	return !TP_INT_IN;
}

void GUI_TOUCH_X_ActivateX(void)
{
}

void GUI_TOUCH_X_ActivateY(void)
{
}

void GUI_TOUCH_X_Disable(void)
{
}

int Read_X(void)
{
	int i;
	TP_CS(0);
	_delay_(1);
	ADS7843SendCmd(CHX);
	_delay_(1);
	i = ADS7843GetResult();
	TP_CS(1);
	return i;
}

int Read_Y(void)
{
	int i;
	TP_CS(0);
	_delay_(1);
	ADS7843SendCmd(CHY);
	_delay_(1);
	i = ADS7843GetResult();
	TP_CS(1);
	return i;
}

Coordinate *Read_Ads7843(void)
{
	static Coordinate screen;
	int m0, m1, m2, temp[3];
	uint8_t count = 0;
	int threshold = THRESHOLD;
	if (CalibrationComplete == 0)
	{
		threshold = THRESHOLD_CALIBRATE;
	}
	int buffer[2][9] = {{0}, {0}};
	do
	{
		buffer[0][count] = Read_X();
		buffer[1][count] = Read_Y();
		count++;
	} while (!TP_INT_IN && count < 9);
	if (count == 9)
	{
		temp[0] = (buffer[0][0] + buffer[0][1] + buffer[0][2]) / 3;
		temp[1] = (buffer[0][3] + buffer[0][4] + buffer[0][5]) / 3;
		temp[2] = (buffer[0][6] + buffer[0][7] + buffer[0][8]) / 3;

		m0 = temp[0] - temp[1];
		m1 = temp[1] - temp[2];
		m2 = temp[2] - temp[0];

		m0 = m0 > 0 ? m0 : (-m0);
		m1 = m1 > 0 ? m1 : (-m1);
		m2 = m2 > 0 ? m2 : (-m2);

		if (m0 > threshold && m1 > threshold && m2 > threshold)
			return 0;

		if (m0 < m1)
		{
			if (m2 < m0)
				screen.x = (temp[0] + temp[2]) / 2;
			else
				screen.x = (temp[0] + temp[1]) / 2;
		}
		else if (m2 < m1)
			screen.x = (temp[0] + temp[2]) / 2;
		else
			screen.x = (temp[1] + temp[2]) / 2;

		temp[0] = (buffer[1][0] + buffer[1][1] + buffer[1][2]) / 3;
		temp[1] = (buffer[1][3] + buffer[1][4] + buffer[1][5]) / 3;
		temp[2] = (buffer[1][6] + buffer[1][7] + buffer[1][8]) / 3;
		m0 = temp[0] - temp[1];
		m1 = temp[1] - temp[2];
		m2 = temp[2] - temp[0];
		m0 = m0 > 0 ? m0 : (-m0);
		m1 = m1 > 0 ? m1 : (-m1);
		m2 = m2 > 0 ? m2 : (-m2);
		if (m0 > threshold && m1 > threshold && m2 > threshold)
			return 0;

		if (m0 < m1)
		{
			if (m2 < m0)
				screen.y = (temp[0] + temp[2]) / 2;
			else
				screen.y = (temp[0] + temp[1]) / 2;
		}
		else if (m2 < m1)
			screen.y = (temp[0] + temp[2]) / 2;
		else
			screen.y = (temp[1] + temp[2]) / 2;

		return &screen;
	}
	return 0;
}

int _xPhys, _yPhys;

int GUI_TOUCH_X_MeasureX(void)
{
	Coordinate *Ptr;

	_xPhys = -1;
	_yPhys = -1;

	Ptr = Read_Ads7843();

	if (Ptr != (void *)0)
	{
		_xPhys = Ptr->x;
		_yPhys = Ptr->y;
	}
	return _xPhys;
}

int GUI_TOUCH_X_MeasureY(void)
{
	Coordinate *Ptr;

	_xPhys = -1;
	_yPhys = -1;

	Ptr = Read_Ads7843();

	if (Ptr != (void *)0)
	{
		_xPhys = Ptr->x;
		_yPhys = Ptr->y;
	}
	return _yPhys;
}

/*********************************************************************
 *
 *       Static functions
 *
 *********************************************************************/

static int _Log2Phys(int l, I32 l0, I32 l1, I32 p0, I32 p1)
{
	return p0 + ((p1 - p0) * (l - l0)) / (l1 - l0);
}

static void _Calibrate(int Coord, int Log0, int Log1, int Phys0, int Phys1, int *p0, int *p1)
{
	int l0 = 0;
	int l1 = (Coord == GUI_COORD_X) ? LCD_GetXSize() - 1 : LCD_GetYSize() - 1;
	*p0 = _Log2Phys(l0, Log0, Log1, Phys0, Phys1);
	*p1 = _Log2Phys(l1, Log0, Log1, Phys0, Phys1);
}

/*********************************************************************
 *
 *       _ExecCalibration
 *
 **********************************************************************
 */
#define FLASH_START 0x8000000
#define TOTAL_PAGES 128
#define PAGE_SIZE 2048
#define FLASH_SIZE (PAGE_SIZE * TOTAL_PAGES)
#define PHYS ((int *)(FLASH_START + FLASH_SIZE - PAGE_SIZE))

void _ExecCalibration(int renew)
{
	/* calculate log. Positions */
	int ax[2] = {15, LCD_GetXSize() - 1 - 15};
	//  const int ay[2] = { 15, LCD_YSIZE-1-15};
	int ay[2] = {15, LCD_GetYSize() - 1 - 15};
	int x0, x1;
	int y0, y1;

	int ax_Phys[2], ay_Phys[2];
	FLASH_EraseInitTypeDef flashErase;
	Coordinate *coord;
	
	ax_Phys[0] = PHYS[0] & 0xFFFF;
	ax_Phys[1] = PHYS[1] & 0xFFFF;
	ay_Phys[0] = PHYS[2] & 0xFFFF;
	ay_Phys[1] = PHYS[3] & 0xFFFF;
	//int sig = PHYS[4] & 0xFFFF;

	if (renew || (PHYS[4] & 0xFFFF) != 0xaa55)
	{

		CalibrationComplete = 0;

		/* _Calibrate upper left */
		GUI_SetBkColor(GUI_RED);
		GUI_Clear();
		GUI_SetColor(GUI_WHITE);
		GUI_FillCircle(ax[0], ay[0], 10);
		GUI_SetColor(GUI_RED);
		GUI_FillCircle(ax[0], ay[0], 5);
		GUI_SetColor(GUI_WHITE);
		GUI_DispStringAt("Press here", ax[0] + 20, ay[0]);
		do
		{
			coord = Read_Ads7843();
			if (coord != (void *)0)
			{
				ax_Phys[0] = coord->x;
				ay_Phys[0] = coord->y;
				break;
			}
			osDelay(20);
		} while (1);
		/* Tell user to release */
		GUI_Clear();
		GUI_DispStringAt("OK", ax[0] + 20, ay[0]);
		do
		{
			coord = Read_Ads7843();
			if (coord == (void *)0)
			{
				break;
			}
			osDelay(100);
		} while (1);
		/* _Calibrate lower right */
		GUI_SetBkColor(GUI_RED);
		GUI_Clear();
		GUI_SetColor(GUI_WHITE);
		GUI_FillCircle(ax[1], ay[1], 10);
		GUI_SetColor(GUI_RED);
		GUI_FillCircle(ax[1], ay[1], 5);
		GUI_SetColor(GUI_WHITE);
		GUI_SetTextAlign(GUI_TA_RIGHT);
		GUI_DispStringAt("Press here", ax[1] - 20, ay[1]);
		do
		{
			coord = Read_Ads7843();
			if (coord != (void *)0)
			{
				ax_Phys[1] = coord->x;
				ay_Phys[1] = coord->y;
				break;
			}
			osDelay(100);
		} while (1);
		uint32_t PAGEError = 0;
		flashErase.TypeErase = FLASH_PROC_PAGEERASE;
		flashErase.PageAddress = (uint32_t)PHYS;
		flashErase.NbPages = 1;
		HAL_FLASH_Unlock();
		/*HAL_StatusTypeDef result = */HAL_FLASHEx_Erase(&flashErase, &PAGEError);
		CLEAR_BIT (FLASH->CR, (FLASH_CR_PER));
		/*result = */HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)&PHYS[0], (uint64_t)ax_Phys[0]);
		CLEAR_BIT (FLASH->CR, (FLASH_CR_PG));
		/*result = */HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)&PHYS[1], (uint64_t)ax_Phys[1]);
		CLEAR_BIT (FLASH->CR, (FLASH_CR_PG));
		/*result = */HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)&PHYS[2], (uint64_t)ay_Phys[0]);
		CLEAR_BIT (FLASH->CR, (FLASH_CR_PG));
		/*result = */HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)&PHYS[3], (uint64_t)ay_Phys[1]);
		CLEAR_BIT (FLASH->CR, (FLASH_CR_PG));
		/*result = */HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, (uint32_t)&PHYS[4], (uint64_t)0xAA55);
		CLEAR_BIT (FLASH->CR, (FLASH_CR_PG));
	}
	else
	{
		ax_Phys[0] = PHYS[0] & 0xFFFF;
		ax_Phys[1] = PHYS[1] & 0xFFFF;
		ay_Phys[0] = PHYS[2] & 0xFFFF;
		ay_Phys[1] = PHYS[3] & 0xFFFF;
	}
	/* calculate and display values for configuration file */
	_Calibrate(GUI_COORD_X, ax[0], ax[1], ax_Phys[0], ax_Phys[1], &x0, &x1);
	_Calibrate(GUI_COORD_Y, ay[0], ay[1], ay_Phys[0], ay_Phys[1], &y0, &y1);

	/*GUI_TOUCH_Calibrate(GUI_COORD_X, 0, 319, x0, x1);
	 GUI_TOUCH_Calibrate(GUI_COORD_Y, 0, 239, y0, y1);*/
	GUI_TOUCH_Calibrate(GUI_COORD_X, ax[0], ax[1], ax_Phys[0], ax_Phys[1]);
	GUI_TOUCH_Calibrate(GUI_COORD_Y, ay[0], ay[1], ay_Phys[0], ay_Phys[1]);

	if (renew || (PHYS[4] & 0xFFFF) != 0xaa55)
	{
		GUI_Clear();
		GUI_DispStringAt("x0: ", 0, 0);
		GUI_DispDec(x0, 4);
		GUI_DispNextLine();
		GUI_DispString("x1: ");
		GUI_DispDec(x1, 4);
		GUI_DispNextLine();
		GUI_DispString("y0: ");
		GUI_DispDec(y0, 4);
		GUI_DispNextLine();
		GUI_DispString("y1: ");
		GUI_DispDec(y1, 4);
		GUI_DispNextLine();

		GUI_DispString("Please touch display to continue...");
		osDelay(1000);
		do
		{
			coord = Read_Ads7843();
			if (coord != (void *)0)
				break;
			osDelay(10);
		} while (1);
	}
	CalibrationComplete = 1;
}
