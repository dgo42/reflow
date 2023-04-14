/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "periph.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"
#include "GUI.h"
#include "GUITDRV_ADS7846.h"
#include <sys/_stdint.h>
#include <math.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include "ADS7843.h"
#include "GUI.h"
#include "TEXT.h"
#include "BUTTON.h"
#include "DIALOG.h"
#include "IMAGE.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// Use MAX31865 as PT100/PT1000 - digital converter 15 bit, 0.03125°C
//#define MAX31865_SENS

// Use MAX6675 as K-thermocouple - digital converter 12 bit, -200°C - 550°C, 0.25°C
//#define MAX6675_SENS

// Use MAX31856 as K-, J-, N-, R-, S-, T-, E-, B-thermocouple - digital converter 19 bit, -210°C - 1800°C, 0.0078125°C
#define MAX31856_SENS

//#define MARLIN_AUTOTUNE
#define KLIPPER_AUTOTUNE

#ifdef KLIPPER_AUTOTUNE
	#define TUNE_PID_DELTA	5 // 5°C temperature delta for auto tune
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

SRAM_HandleTypeDef hsram1;

osThreadId defaultTaskHandle;
osThreadId tempTaskHandle;
osThreadId touchTaskHandle;
osThreadId guiTaskHandle;
/* USER CODE BEGIN PV */

// Define SPI pinout
#ifdef MAX31865_SENS
#include "MAX31865.h"

MAX31865_GPIO max_gpio;
#elif defined MAX6675_SENS
#include "MAX6675.h"

MAX6675_GPIO max_gpio1;
#elif defined MAX31856_SENS
#include "MAX31856.h"

MAX31856_GPIO tc0;
MAX31856_GPIO tc1;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_FSMC_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
void StartDefaultTask(void const *argument);
void TouchTask(void const *argument);
void TempTask(void const *argument);
void GuiTask(void const *argument);

void SetOven(float power);
float GetPidOutput(int time);

/* USER CODE BEGIN PFP */
#define K1 0.95 //smoothing factor within the PID
#define K2 (1.0 - K1)

// default PID coefficient parameter
#define DEFAULT_Kp 38.197186
#define DEFAULT_Ki 2.75126481
#define DEFAULT_Kd 132.577652

//#define DEFAULT_Kc (1) //heating power=Kc*(e_speed)
//#define OVERSAMPLENR 16

#ifdef PHASE_POWER
#define CYCLES_PER_SECOND 10
#elif defined PULSE_POWER
#define CYCLES_PER_SECOND 1
#endif

#define CYCLES_PER_HALFSECOND (CYCLES_PER_SECOND / 2)
#define CYCLES_PER_2SECOND (CYCLES_PER_SECOND * 2)
#define CYCLES_PER_3SECOND (CYCLES_PER_SECOND * 3)
#define CYCLES_PER_5SECOND (CYCLES_PER_SECOND * 5)

#define PID_dT (1. / (float)CYCLES_PER_SECOND)

#define CYCLE_DELAY (uint16_t)(1000 / CYCLES_PER_SECOND)

// define min/max power in percent
#define MAX_POWER 100.
#define MIN_POWER 0.

// CPU flash start address
#define FLASH_START 0x8000000
// CPU total flash pages
#define TOTAL_PAGES 128
// CPU flash page size
#define PAGE_SIZE 2048
// CPU total flash size
#define FLASH_SIZE (PAGE_SIZE * TOTAL_PAGES)
// CPU begin of the last flash size - to store PID parameters after autotune
#define PHYS ((int *)(FLASH_START + FLASH_SIZE - PAGE_SIZE))

int prevPressed = 0;
int nextPressed = 0;
int enterPressed = 0;
bool showDebug = false;

// PID coefficient parameter, initialized with default
float Kp = DEFAULT_Kp;
float Ki = DEFAULT_Ki * PID_dT;
float Kd = DEFAULT_Kd / PID_dT;

//static cannot be external:
static float temp_iState = 0;
static float temp_dState = 0;
static float dTerm;

static float temp_iState_min;
static float temp_iState_max;

float target_temperature = 0;
float current_temperature = 0.0;

static const GUI_COLOR IndColorPrep[] = {GUI_BLACK, 0x0000D000};
static const GUI_COLOR IndColorOn[] = {GUI_BLACK, GUI_RED};
static const GUI_COLOR IndColorOff[] = {GUI_BLACK, GUI_DARKRED};

static const GUI_LOGPALETTE IndPalettePrep = {2, 1, IndColorPrep};
static const GUI_LOGPALETTE IndPaletteOn = {2, 1, IndColorOn};
static const GUI_LOGPALETTE IndPaletteOff = {2, 1, IndColorOff};

static const unsigned char light[] = {
	________, ____XXXX, XXXX____, ________,
	________, _XXXXXXX, XXXXXXX_, ________,
	_______X, XXXXXXXX, XXXXXXXX, X_______,
	______XX, XXXXXXXX, XXXXXXXX, XX______,
	_____XXX, XXXXXXXX, XXXXXXXX, XXX_____,
	____XXXX, XXXXXXXX, XXXXXXXX, XXXX____,
	___XXXXX, XXXXXXXX, XXXXXXXX, XXXXX___,
	__XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
	__XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
	_XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_,
	_XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_,
	_XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_,
	XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
	XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
	XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
	XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
	XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
	XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
	XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
	XXXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXXX,
	_XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_,
	_XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_,
	_XXXXXXX, XXXXXXXX, XXXXXXXX, XXXXXXX_,
	__XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
	__XXXXXX, XXXXXXXX, XXXXXXXX, XXXXXX__,
	___XXXXX, XXXXXXXX, XXXXXXXX, XXXXX___,
	____XXXX, XXXXXXXX, XXXXXXXX, XXXX____,
	_____XXX, XXXXXXXX, XXXXXXXX, XXX_____,
	______XX, XXXXXXXX, XXXXXXXX, XX______,
	_______X, XXXXXXXX, XXXXXXXX, X_______,
	________, _XXXXXXX, XXXXXXX_, ________,
	________, ____XXXX, XXXX____, ________};

static const GUI_BITMAP bm_1bpp_prep = {32, 32, 4, 1, light, &IndPalettePrep};
static const GUI_BITMAP bm_1bpp_on = {32, 32, 4, 1, light, &IndPaletteOn};
static const GUI_BITMAP bm_1bpp_off = {32, 32, 4, 1, light, &IndPaletteOff};

#define LEADFREE_MELT_POINT 217
#define LEADED_MELT_POINT 183

int leadFreePoints[][3] = {
	// ramp to soak
	{90, 150, 0},  // delta 90s, target 150°C 	 - first ramp (1,38°C/s)
	// preheat/soak
	{120, 200, 0}, // delta 120s, target 200°C  - second ramp (0,42°C/s)
	// ramp to peak
	{30, 240, 0},  // delta 30s, target 240°C   - ramp up to the reflow plateau (1,33°C/s)
	// reflow
	{40, 240, 0},  // delta 30s, target 240°C   - reflow plateau
	// cooling
	{180, 0, 1}	// delta 180s, target 0°C    - cool down ramp - controlled manually (target -1,33°C/s)
};

int leadedPoints[][3] = {
	// ramp to soak
	{70, 120, 0},  // delta 70s, target 120°C   - first ramp (1,35°C/s)
	// preheat/soak
	{120, 185, 0}, // delta 120s, target 185°C  - second ramp (0,54°C/s)
	// ramp to peak
	{30, 225, 0},  // delta 30s, target 225°C   - ramp up to the reflow plateau (1,33°C/s)
	// reflow
	{40, 225, 0},  // delta 30s, target 225°C   - reflow plateau
	// cooling
	{180, 0, 1}    // delta 180s, target 0°C    - cool down ramp - controlled manually (target -1,33°C/s)
};

typedef struct
{
	WM_HMEM handles[10];
	int keys[10];
	int maxControls;
	int focusedControl;
	int firstPressed;
	int controlPressed;
} WINDOW_CONTROLS;

void processButtons(int time, WINDOW_CONTROLS *controls);
void unFocus(BUTTON_Handle btn);
void focus(BUTTON_Handle btn);

static BUTTON_SKINFLEX_PROPS _aProps[] = {
	// BUTTON_SKINFLEX_PI_PRESSED
	{
		{GUI_RED, GUI_RED, GUI_DARKRED},
		{GUI_LIGHTRED, 0x004040FF},
		{0x004040FF, 0x000000FF},
		3},
	// BUTTON_SKINFLEX_PI_FOCUSSED
	{
		{GUI_DARKRED, GUI_RED, GUI_DARKGRAY},
		{GUI_WHITE, 0x00E9E9E9},
		{0x00E9E9E9, GUI_LIGHTGRAY},
		3},
	// BUTTON_SKINFLEX_PI_ENABLED
	{
		{GUI_DARKRED, GUI_GRAY, GUI_DARKGRAY},
		{GUI_WHITE, 0x00E9E9E9},
		{0x00E9E9E9, GUI_LIGHTGRAY},
		3},
	// BUTTON_SKINFLEX_PI_DISABLED
	{
		{GUI_DARKGRAY, GUI_DARKGRAY, GUI_DARKGRAY},
		{0x00707070, 0x00606060},
		{0x00606060, 0x00505050},
		3},
};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_CRC_Init();
	MX_FSMC_Init();
	MX_RTC_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_USART1_UART_Init();
	MX_SDIO_SD_Init();
	MX_TIM3_Init();
	MX_FATFS_Init();
	MX_TIM2_Init();
	MX_TIM8_Init();
	/* USER CODE BEGIN 2 */
	//HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	printf("Start reflow oven\r\n");

	/* definition and creation of guiTask */
	osThreadDef(guiTask, GuiTask, osPriorityAboveNormal, 0, 2048);
	guiTaskHandle = osThreadCreate(osThread(guiTask), NULL);

	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of touchTask */
	osThreadDef(touchTask, TouchTask, osPriorityBelowNormal, 0, 128);
	touchTaskHandle = osThreadCreate(osThread(touchTask), NULL);

	/* definition and creation of touchTask */
	osThreadDef(tempTask, TempTask, osPriorityBelowNormal, 0, 128);
	tempTaskHandle = osThreadCreate(osThread(tempTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */
		GuiTask(NULL);
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;)
	{
		osDelay(100);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_GuiTask */
/**
 * @brief Function implementing the guiTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GuiTask */
int CalibrationComplete = 0;
Coordinate *coord;
void TouchTask(void const *argument)
{
	for (;;)
	{
		/*coord = Read_Ads7843();
		osDelay(10);*/
		/* 20 MS  */
		GUI_TOUCH_Exec();
		osDelay(10); /* 20 MS  */
		if (KEY1)
		{
			prevPressed++;
		}
		else
		{
			prevPressed = 0;
		}
		if (KEY2)
		{
			nextPressed++;
		}
		else
		{
			nextPressed = 0;
		}
		if (KEY3)
		{
			enterPressed++;
		}
		else
		{
			enterPressed = 0;
		}
	}
}

float temp = 0;
float temp1 = 0;
float temp2 = 0;

void TempTask(void const *argument)
{
#ifdef MAX31865_SENS
	MAX31865_init(&max_gpio, 2);

	for (;;)
	{
		temp1 = MAX31865_readTemp();
		temp_meas_ready = true;
	}
#elif defined MAX6675_SENS
	MAX6675_init(&max_gpio1);
	for (;;)
	{
		temp1 = MAX6675_readTemp();
		temp_meas_ready = true;
		osDelay(CYCLE_DELAY);
	}
#elif defined MAX31856_SENS
	MAX31856_config(&tc0, MAX31856_CONTINOUS | MAX31856_FILTER_50HZ, MAX31856_KTYPE | MAX31856_AV1);
	MAX31856_config(&tc1, MAX31856_CONTINOUS | MAX31856_FILTER_50HZ, MAX31856_KTYPE | MAX31856_AV1);
	for (;;)
	{
		temp1 = MAX31856_readTC(&tc0);
		temp2 = MAX31856_readTC(&tc1);
		temp = (temp1 + temp2) / 2.;
		//temp = temp1;
		osDelay(CYCLE_DELAY);
	}
#else
	for (;;)
	{
		osDelay(CYCLE_DELAY);
	}
#endif
}

static int time = 0;
static int xPos = 0, yPos = 0, yPos2 = 0, yTargetPos = 0, yPowerPos = 0;

#define MAIN_WIN_ID 1
#define REFLOW_WIN_ID 2
#define AUTOTUNE_WIN_ID 3

#define MAIN_LEAD_FREE 126
#define MAIN_LEADED 125
#define MAIN_AUTOTUNE 124
#define MAIN_DEBUG 123

#define REFLOW_START 122
#define REFLOW_STOP 121

#define AUTOTUNE_START 120
#define AUTOTUNE_STOP 119

void ReflowProfile(int points[][3], int meltingPoint);
void AutotuneMenu(void);

void GuiTask(void const *argument)
{
	int i;
	char buf[25];
	/* USER CODE BEGIN GuiTask */
	__HAL_RCC_CRC_CLK_ENABLE();
	// init GUI
	GUI_Init();
	// enable back light
	HAL_TIM_OC_Start(&htim3, TIM_CHANNEL_2);
	// enable UTF-8
	GUI_UC_SetEncodeUTF8();
	GUITDRV_ADS7846_CONFIG Config = {0};

	Config.pfSendCmd = ADS7843SendCmd;
	Config.pfGetResult = ADS7843GetResult;
	Config.pfGetBusy = ADS7843GetBusy;
	Config.pfGetPENIRQ = ADS7843GetPENIRQ;
	Config.pfSetCS = ADS7843SetCS;
	Config.xLog0 = 0;
	Config.xLog1 = 319;
	Config.xPhys0 = 300;
	Config.xPhys1 = 3600;
	Config.yLog0 = 0;
	Config.yLog1 = 239;
	Config.yPhys0 = 3400;
	Config.yPhys1 = 400;
	Config.Orientation = GUI_SWAP_XY | GUI_MIRROR_Y;
	GUITDRV_ADS7846_Config(&Config);

	GUI_TOUCH_Calibrate(GUI_COORD_X, 0, 319, 300, 3600);
	GUI_TOUCH_Calibrate(GUI_COORD_Y, 0, 239, 3400, 400);

	osDelay(1000); // wait for first temperarure measurement

	/* Infinite loop */

	time = 0;
	xPos = yPos = yPos2 = yTargetPos = yPowerPos = 0;

	// initialize main menu
	GUI_SetBkColor(GUI_DARKGRAY);
	GUI_SetColor(GUI_WHITE);
	GUI_Clear();

	// change button skins
	for (i = 0; i < 4; i++)
	{
		//BUTTON_GetSkinFlexProps(&aPropsOld[i], i);  // Get default properties
		BUTTON_SetSkinFlexProps(&_aProps[i], i); // Set new properties
	}

	WINDOW_CONTROLS controls;
	controls.firstPressed = time;
	controls.focusedControl = 0;
	controls.controlPressed = -1;

	WM_HWIN mainWnd = WINDOW_CreateEx(0, 0, 319, 239, (WM_HWIN)NULL, WM_CF_SHOW, 0, MAIN_WIN_ID, NULL);
	WM_EnableMemdev(mainWnd);
	WM_HWIN handle;
	WINDOW_SetBkColor(mainWnd, GUI_BLACK);

	i = 0;
	handle = BUTTON_CreateEx(5, 5, 80, 80, mainWnd, BUTTON_CF_SHOW, 0, MAIN_LEAD_FREE);
	BUTTON_SetText(handle, "Lead\r\nfree");
	BUTTON_SetFont(handle, &GUI_FontComic18B_ASCII);
	WM_EnableMemdev(handle);
	controls.handles[i] = handle;
	controls.keys[i] = MAIN_LEAD_FREE;
	i++;

	handle = BUTTON_CreateEx(90, 5, 80, 80, mainWnd, BUTTON_CF_SHOW, 0, MAIN_LEADED);
	BUTTON_SetText(handle, "Leaded");
	BUTTON_SetFont(handle, &GUI_FontComic18B_ASCII);
	WM_EnableMemdev(handle);
	controls.handles[i] = handle;
	controls.keys[i] = MAIN_LEADED;
	i++;

	handle = BUTTON_CreateEx(175, 5, 80, 80, mainWnd, BUTTON_CF_SHOW, 0, MAIN_AUTOTUNE);
	BUTTON_SetText(handle, "PID\r\nautotune");
	BUTTON_SetFont(handle, &GUI_FontComic18B_ASCII);
	WM_EnableMemdev(handle);
	controls.handles[i] = handle;
	controls.keys[i] = MAIN_AUTOTUNE;
	i++;

	WM_HWIN debugChk = handle = CHECKBOX_CreateEx(5, 85, 110, 20, mainWnd, BUTTON_CF_SHOW, 0, MAIN_DEBUG);
	CHECKBOX_SetText(handle, "PID debug");
	CHECKBOX_SetFont(handle, &GUI_FontComic18B_ASCII);
	CHECKBOX_SetFocusColor(handle, GUI_RED);
	CHECKBOX_SetTextColor(handle, GUI_WHITE);
	CHECKBOX_SetState(handle, showDebug ? 1 : 0);

	WM_EnableMemdev(handle);
	controls.handles[i] = handle;
	controls.keys[i] = MAIN_DEBUG;
	controls.maxControls = i;

	sprintf(buf, "Temp %.1f°C  ", temp);
	WM_HWIN tempHandle = TEXT_CreateEx(5, 202, 110, 16, mainWnd, WM_CF_SHOW, TEXT_CF_LEFT | TEXT_CF_VCENTER, 0, buf);
	TEXT_SetTextColor(tempHandle, GUI_WHITE);
	TEXT_SetFont(tempHandle, &GUI_Font8x13_1);
	WM_EnableMemdev(tempHandle);

	focus(controls.handles[controls.focusedControl]);
	for (;;)
	{
		GUI_Exec();
		int res = GUI_GetKey();
		if (res > 0)
		{
			switch (res)
			{
			case MAIN_LEAD_FREE:
				ReflowProfile(leadFreePoints, LEADFREE_MELT_POINT);
				break;
			case MAIN_LEADED:
				ReflowProfile(leadedPoints, LEADED_MELT_POINT);
				break;
			case MAIN_AUTOTUNE:
				AutotuneMenu();
				break;
			case MAIN_DEBUG:
				showDebug = !showDebug;
				CHECKBOX_SetState(debugChk, showDebug ? 1 : 0);
				break;
			}
		}
		processButtons(time, &controls);
		if ((time % CYCLES_PER_SECOND) == 0 || res > 0)
		{
			sprintf(buf, "Temp %.1f°C  ", temp);
			TEXT_SetText(tempHandle, buf);
		}

		time++;
		osDelay(CYCLE_DELAY);
	}
	/* USER CODE END GuiTask */
}

void unFocus(BUTTON_Handle btn)
{
	WM_MESSAGE focus;
	focus.MsgId = WM_SET_FOCUS;
	focus.hWin = btn;
	focus.hWinSrc = btn;
	focus.Data.v = 0;
	WM_SendMessage(btn, &focus);
}

void focus(BUTTON_Handle btn)
{
	WM_MESSAGE focus;
	focus.MsgId = WM_SET_FOCUS;
	focus.hWin = btn;
	focus.hWinSrc = btn;
	focus.Data.v = 1;
	WM_SendMessage(btn, &focus);
}

void prevButton(WINDOW_CONTROLS *controls)
{
	int temp = controls->focusedControl;
	while (1)
	{
		temp = max(temp - 1, 0);
		// is prev button disabled?
		if (!WM_IsEnabled(controls->handles[temp]))
		{
			// if prev button is the first button - exit
			if (temp == 0)
			{
				return;
			}
		}
		else
		{
			break;
		}
	}
	unFocus(controls->handles[controls->focusedControl]);
	controls->focusedControl = temp;
	focus(controls->handles[controls->focusedControl]);
}

void nextButton(WINDOW_CONTROLS *controls)
{
	int temp = controls->focusedControl;
	while (1)
	{
		temp = min(temp + 1, controls->maxControls);
		// is next button disabled?
		if (!WM_IsEnabled(controls->handles[temp]))
		{
			// if next button is the last button - exit
			if (temp == controls->maxControls)
			{
				return;
			}
		}
		else
		{
			break;
		}
	}
	unFocus(controls->handles[controls->focusedControl]);
	controls->focusedControl = temp;
	focus(controls->handles[controls->focusedControl]);
}

void processButtons(int time, WINDOW_CONTROLS *controls)
{
	if (!controls->firstPressed && prevPressed > 1)
	{
		prevButton(controls);
		controls->firstPressed = time;
	}
	if (!controls->firstPressed && nextPressed > 1)
	{
		nextButton(controls);
		controls->firstPressed = time;
	}
	if (!controls->firstPressed && enterPressed > 2)
	{
		BUTTON_SetPressed(controls->handles[controls->focusedControl], 1);
		controls->controlPressed = controls->focusedControl;
		GUI_StoreKey(controls->keys[controls->focusedControl]);
		controls->firstPressed = time;
	}
	if (controls->firstPressed && !prevPressed && !nextPressed && !enterPressed)
	{
		controls->firstPressed = 0;
	}
	// if enter was pressed and button was marked as pressed longer than 0.3 second - unset pressed state
	if (time - controls->firstPressed > 3 && controls->controlPressed >= 0)
	{
		BUTTON_SetPressed(controls->handles[controls->controlPressed], 0);
		controls->controlPressed = -1;
	}
	// pressed longer than 1 second - repeat
	if (time - controls->firstPressed > 10)
	{
		controls->firstPressed = 0;
	}
}

void ReflowProfile(int points[][3], int meltingPoint)
{
	int i;
	char buf[25];

	Kp = DEFAULT_Kp;
	Ki = DEFAULT_Ki * PID_dT;
	Kd = DEFAULT_Kd / PID_dT;

	float tempKp = *(float*)&PHYS[0];
	float tempKi = *(float*)&PHYS[1];
	float tempKd = *(float*)&PHYS[2];
	uint32_t sig = PHYS[3];

	if (sig == 0xDEADBEEF)
	{
		Kp = tempKp;
		Ki = tempKi * PID_dT;
		Kd = tempKd / PID_dT;
	}

	temp_iState_min = 0.0;
	temp_iState_max = MAX_POWER / Ki;

	int seconds = 0;
	float power = MIN_POWER;
	float startTemp, endTemp;
	int openDoor = 0;
	float ramp;
	float oldTemp = temp, deltaTemp = 0.;
	int currentPoint = 0;
	int tempCounter = 0;
	int nextTime;

	WINDOW_CONTROLS controls;
	controls.firstPressed = time;
	controls.focusedControl = 0;
	controls.controlPressed = -1;

	yPos = roundf(temp * 0.8f);

	WM_HWIN reflowWnd = WINDOW_CreateEx(0, 0, 319, 239, (WM_HWIN)NULL, WM_CF_SHOW, 0, REFLOW_WIN_ID, NULL);
	WM_EnableMemdev(reflowWnd);
	WM_HWIN handle;
	WINDOW_SetBkColor(reflowWnd, GUI_BLACK);

	i = 0;
	handle = BUTTON_CreateEx(155, 202, 80, 38, reflowWnd, BUTTON_CF_SHOW, 0, REFLOW_START);
	BUTTON_SetText(handle, "Start");
	BUTTON_SetFont(handle, &GUI_FontComic18B_ASCII);
	WM_EnableMemdev(handle);
	controls.handles[i] = handle;
	controls.keys[i] = REFLOW_START;
	i++;

	handle = BUTTON_CreateEx(240, 202, 80, 38, reflowWnd, BUTTON_CF_SHOW, 0, REFLOW_STOP);
	BUTTON_SetText(handle, "Exit");
	BUTTON_SetFont(handle, &GUI_FontComic18B_ASCII);
	WM_EnableMemdev(handle);
	controls.handles[i] = handle;
	controls.keys[i] = REFLOW_STOP;
	controls.maxControls = i;

	sprintf(buf, "Temp %.1f°C  ", temp);
	WM_HWIN tempHandle = TEXT_CreateEx(5, 202, 110, 16, reflowWnd, WM_CF_SHOW, TEXT_CF_LEFT | TEXT_CF_VCENTER, 0, buf);
	TEXT_SetTextColor(tempHandle, GUI_WHITE);
	TEXT_SetFont(tempHandle, &GUI_Font8x13_1);
	WM_EnableMemdev(tempHandle);

	sprintf(buf, "Time %lds  ", (long)(time / CYCLES_PER_SECOND));
	WM_HWIN timeHandle = TEXT_CreateEx(5, 222, 110, 16, reflowWnd, WM_CF_SHOW, TEXT_CF_LEFT | TEXT_CF_VCENTER, 0, buf);
	TEXT_SetTextColor(timeHandle, GUI_WHITE);
	TEXT_SetFont(timeHandle, &GUI_Font8x13_1);
	WM_EnableMemdev(timeHandle);

	IMAGE_Handle indHandle = IMAGE_CreateEx(118, 205, 32, 32, reflowWnd, WM_CF_SHOW, IMAGE_CF_MEMDEV, 0);
	IMAGE_SetBitmap(indHandle, &bm_1bpp_prep);
	WM_EnableMemdev(indHandle);

	WM_HWIN graphHandle = GRAPH_CreateEx(0, 0, 320, 200, reflowWnd, WM_CF_SHOW, GRAPH_CF_GRID_FIXED_X, 0);
	GRAPH_SetGridVis(graphHandle, 1);
	GRAPH_SetColor(graphHandle, 0x00E0E0E0, GRAPH_CI_GRID);
	GRAPH_SetLineStyleH(graphHandle, GUI_LS_DOT);
	GRAPH_SetLineStyleV(graphHandle, GUI_LS_DOT);
	GRAPH_SetGridDistX(graphHandle, 30);
	GRAPH_SetGridDistY(graphHandle, 16);
	WM_EnableMemdev(graphHandle);

	int16_t point = (int16_t)((float)meltingPoint * 0.8);
	GUI_POINT thrRawData[] = {
		{0, point},
		{319, point}};

	GRAPH_DATA_Handle thrData = GRAPH_DATA_XY_Create(GUI_RED, 2, thrRawData, 2);
	GRAPH_DATA_XY_SetPenSize(thrData, 2);
	GRAPH_AttachData(graphHandle, thrData);

	GRAPH_DATA_Handle pwrData;
	GRAPH_DATA_Handle deltaData;
	if (showDebug)
	{
		int16_t pwrRawData[320];
		pwrData = GRAPH_DATA_YT_Create(GUI_BLUE, 320, pwrRawData, 0);
		GRAPH_DATA_YT_SetAlign(pwrData, GRAPH_ALIGN_LEFT);
		GRAPH_AttachData(graphHandle, pwrData);

		int16_t deltaRawData[320];
		deltaData = GRAPH_DATA_YT_Create(GUI_YELLOW, 320, deltaRawData, 0);
		GRAPH_DATA_YT_SetAlign(deltaData, GRAPH_ALIGN_LEFT);
		GRAPH_DATA_YT_SetOffY(deltaData, 96);
		GRAPH_AttachData(graphHandle, deltaData);
	}

	int16_t targetRawData[320];
	GRAPH_DATA_Handle targetData = GRAPH_DATA_YT_Create(GUI_GREEN, 320, targetRawData, 0);
	GRAPH_DATA_YT_SetAlign(targetData, GRAPH_ALIGN_LEFT);
	GRAPH_AttachData(graphHandle, targetData);

	int16_t tempRawData[320];
	GRAPH_DATA_Handle tempData = GRAPH_DATA_YT_Create(GUI_RED, 320, tempRawData, 0);
	GRAPH_DATA_YT_SetAlign(tempData, GRAPH_ALIGN_LEFT);
	GRAPH_AttachData(graphHandle, tempData);

	WM_HWIN overlayHandle = TEXT_CreateEx(40, 60, 280, 80, reflowWnd, WM_CF_SHOW, TEXT_CF_LEFT | TEXT_CF_VCENTER, 0, "");
	TEXT_SetTextColor(overlayHandle, GUI_WHITE);
	TEXT_SetFont(overlayHandle, &GUI_Font20_1);
	WM_EnableMemdev(overlayHandle);

	bool showMessage = false;
	bool startProcess = false;

	bool started = false;
	bool ended = false;
	bool openOven = false;
	bool colorSwap = false;

	int deltaTime = 0;

	focus(controls.handles[controls.focusedControl]);

	// switch power off
	SetOven(power);

	for (;;)
	{
		GUI_Exec();
		int res = GUI_GetKey();
		if (res > 0)
		{
			switch (res)
			{
			case REFLOW_START:
				startProcess = true;
				break;
			case REFLOW_STOP:
				SetOven(MIN_POWER);
				WM_DeleteWindow(reflowWnd);
				return;
			}
		}
		processButtons(time, &controls);

		// if process is initiated, but not started and not ended
		if (startProcess && !started && !ended) {
			if (temp > 40. && !showMessage) {
				showMessage = true;
				// show message
				TEXT_SetText(overlayHandle, "Waiting temperature < 40°");
			} else {
				// initialize dTemp
				dTerm = 0;
				temp_iState = 0;
				// reset start process flag
				startProcess = false;
				// hide message
				showMessage = false;
				TEXT_SetText(overlayHandle, "");
				// start process
				started = true;
				// initiate start & target temperature with current
				target_temperature = startTemp = temp;
				// set temperature for PID regulator
				temp_dState = temp;
				// obtain first target temperature & next time point
				endTemp = (float)points[currentPoint][1];
				openDoor = points[currentPoint][2];
				nextTime = points[currentPoint][0];
				// calculate temperature ramp using PID_dT times per second
				ramp = (endTemp - target_temperature) * PID_dT / nextTime;
				// switch pointer to the next profile point
				currentPoint++;
			}
		}
		// get current temperature
		current_temperature = temp;
		// process
		if (started && !ended)
		{
			// if PID mode and not start boost and temperature <
			if (endTemp > 0 && target_temperature < endTemp)
			{
				target_temperature = fminf(target_temperature + ramp, endTemp);
			}
			if (endTemp == 0 && target_temperature > 0)
			{
				target_temperature += ramp;
			}

			// if next target temperature > 0 - power oven using PID
			if (endTemp > 0)
			{
				power = GetPidOutput(time);
				SetOven(power);
			}
			else
			{
				// if we on the last profile point - switch PID off
				power = MIN_POWER;
				SetOven(MIN_POWER);
			}

			// highlight red bubble if the power > 0
			if (power == MIN_POWER)
			{
				IMAGE_SetBitmap(indHandle, &bm_1bpp_off);
			}
			else
			{
				IMAGE_SetBitmap(indHandle, &bm_1bpp_on);
			}

			// wait until next time point and target temperature of the current profile point is reached
			if (nextTime <= seconds && temp > target_temperature && endTemp > 0)
			{
				// get delta time of the next profile point
				deltaTime = points[currentPoint][0];
				// get next target temperature
				endTemp = (float)points[currentPoint][1];
				openDoor = points[currentPoint][2];
				// log next profile point to the serial
				printf("next point %d delta T %d end temp %.3f\r\n", currentPoint, deltaTime, endTemp);
				// if it is not the last point (last point has 0 target temperature)
				ramp = (endTemp - temp) * PID_dT / deltaTime;
				nextTime = seconds + deltaTime;
				currentPoint++;
			}
			// if we on the last profile point and temperature is lower melting point
			// show message "open the oven"
			if (endTemp == 0 || openDoor == 1) {
				if ((temp < (float)meltingPoint || openDoor == 1) && !openOven)
				{
					openOven = true;
					showMessage = true;
					TEXT_SetText(overlayHandle, "PLEASE OPEN THE OVEN!!!");
					// log the state to the serial
					printf("Reflow process ended, please open the oven!!!\r\n");
				}

				// if we on the last profile point and temperature is lower 50° - end the process
				if (temp < 50. && openOven) {
					// end the process
					ended = true;
					// hide the message
					showMessage = false;
					TEXT_SetText(overlayHandle, "");
				}
			}
		}

		// indication & logging
		// if reflow ended and oven should be opened
		if (showMessage && (time % CYCLES_PER_HALFSECOND) == 0)
		{
			if (colorSwap)
			{
				TEXT_SetTextColor(overlayHandle, GUI_WHITE);
				colorSwap = false;
			}
			else
			{
				TEXT_SetTextColor(overlayHandle, GUI_RED);
				colorSwap = true;
			}
		}
		// each second
		if ((time % CYCLES_PER_SECOND) == 0)
		{
			sprintf(buf, "Temp %.1f°C  ", temp);
			TEXT_SetText(tempHandle, buf);

			sprintf(buf, "Time %ds  ", seconds);
			TEXT_SetText(timeHandle, buf);

			printf("%.3f; 0.000\r\n", temp);

			seconds++;
			deltaTemp = temp - oldTemp;
			oldTemp = temp;
		}
		// each 2 seconds
		// draw next points on the graph
		if ((time % CYCLES_PER_2SECOND) == 0)
		{
			if (tempCounter < 320)
			{
				// red graphics shows current temperature 20°C/div
				yPos = roundf(current_temperature * 0.8f);
				GRAPH_DATA_YT_AddValue(tempData, yPos);

				// green graphics shows current temperature 20°C/div
				yTargetPos = roundf(target_temperature * 0.8f);
				GRAPH_DATA_YT_AddValue(targetData, yTargetPos);

				if (showDebug)
				{
					// yellow graphic shows delta temperature @middle of the graph 1°C/div
					yPos2 = roundf(deltaTemp * 16.f);
					GRAPH_DATA_YT_AddValue(deltaData, yPos2);

					// blue graphic shows power in %, 20%/div
					yPowerPos = roundf(power * 0.8f);
					GRAPH_DATA_YT_AddValue(pwrData, yPowerPos);
				}
				tempCounter++;
			}
		}

		time++;
		osDelay(CYCLE_DELAY);
	}
}

void flashPidParams(float Kp, float Ki, float Kd)
{
	uint32_t PAGEError = 0;
	FLASH_EraseInitTypeDef flashErase;
	flashErase.TypeErase = FLASH_PROC_PAGEERASE;
	flashErase.PageAddress = (uint32_t)PHYS;
	flashErase.NbPages = 1;
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&flashErase, &PAGEError);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&PHYS[0], *(uint64_t*)&Kp);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&PHYS[1], *(uint64_t*)&Ki);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&PHYS[2], *(uint64_t*)&Kd);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)&PHYS[3], (uint64_t)0xDEADBEEF);
}

#ifdef KLIPPER_AUTOTUNE

	#define PEAKS_CYCLES 12

#endif

void swapFloat(float* xp, float* yp)
{
    float temp = *xp;
    *xp = *yp;
    *yp = temp;
}

void swapInt(int* xp, int* yp)
{
	int temp = *xp;
    *xp = *yp;
    *yp = temp;
}

// Function to perform Selection Sort
void selectionSort(float time[], int pos[], float n)
{
    int i, j, min_idx;

    // One by one move boundary of unsorted subarray
    for (i = 0; i < n - 1; i++) {

        // Find the minimum element in unsorted array
        min_idx = i;
        for (j = i + 1; j < n; j++)
            if (time[j] < time[min_idx])
                min_idx = j;

        // Swap the found minimum element
        // with the first element
        swapFloat(&time[min_idx], &time[i]);
        swapInt(&pos[min_idx], &pos[i]);
    }
}

void AutotuneMenu(void)
{
	int i;
	bool start = false;
	bool stop = false;
	int seconds = 0;
	float targetTemp = 200.;
	bool colorSwap = false;
	float currTargetTemp = targetTemp;

#ifdef MARLIN_AUTOTUNE
	int ncycles = 3;
	float bias, d;
	float maxVal = 0, minVal = 10000;
	long t_high = 0, t_low = 0;
	unsigned long temp_millis = osKernelSysTick();
	unsigned long t1 = temp_millis, t2 = temp_millis;
	int cycles = 0;
#elif defined KLIPPER_AUTOTUNE
	float peak = 0;
	unsigned long peakTime = 0;
	int currPeak = 0;
	float peaks[PEAKS_CYCLES];
	unsigned long peakTimes[PEAKS_CYCLES];
	float Ti, Td;
#endif
	char buf[25];

	float input = 0.0;
	bool heating = true;


	float power = 0;
	float Ku, Tu;
	float Kp, Ki, Kd;
	unsigned long ms;

	WINDOW_CONTROLS controls;
	controls.firstPressed = time;
	controls.focusedControl = 0;
	controls.controlPressed = -1;

	yPos = roundf(temp * 0.8f);

	WM_HWIN leadFreeWnd = WINDOW_CreateEx(0, 0, 319, 239, (WM_HWIN)NULL, WM_CF_SHOW, 0, REFLOW_WIN_ID, NULL);
	WM_EnableMemdev(leadFreeWnd);
	WM_HWIN handle;
	WINDOW_SetBkColor(leadFreeWnd, GUI_BLACK);

	i = 0;
	handle = BUTTON_CreateEx(155, 202, 80, 38, leadFreeWnd, BUTTON_CF_SHOW, 0, REFLOW_START);
	BUTTON_SetText(handle, "Start");
	BUTTON_SetFont(handle, &GUI_FontComic18B_ASCII);
	WM_EnableMemdev(handle);
	controls.handles[i] = handle;
	controls.keys[i] = REFLOW_START;
	i++;

	handle = BUTTON_CreateEx(240, 202, 80, 38, leadFreeWnd, BUTTON_CF_SHOW, 0, REFLOW_STOP);
	BUTTON_SetText(handle, "Exit");
	BUTTON_SetFont(handle, &GUI_FontComic18B_ASCII);
	WM_EnableMemdev(handle);
	controls.handles[i] = handle;
	controls.keys[i] = REFLOW_STOP;
	controls.maxControls = i;

	sprintf(buf, "Temp %.1f°C  ", temp1);
	WM_HWIN temp1Handle = TEXT_CreateEx(5, 200, 110, 13, leadFreeWnd, WM_CF_SHOW, TEXT_CF_LEFT | TEXT_CF_VCENTER, 0, buf);
	TEXT_SetTextColor(temp1Handle, GUI_WHITE);
	TEXT_SetFont(temp1Handle, &GUI_Font8x13_1);
	WM_EnableMemdev(temp1Handle);

	sprintf(buf, "Temp %.1f°C  ", temp2);
	WM_HWIN temp2Handle = TEXT_CreateEx(5, 213, 110, 13, leadFreeWnd, WM_CF_SHOW, TEXT_CF_LEFT | TEXT_CF_VCENTER, 0, buf);
	TEXT_SetTextColor(temp2Handle, GUI_WHITE);
	TEXT_SetFont(temp2Handle, &GUI_Font8x13_1);
	WM_EnableMemdev(temp2Handle);

	sprintf(buf, "Time %lds  ", (long)(time / CYCLES_PER_SECOND));
	WM_HWIN timeHandle = TEXT_CreateEx(5, 226, 110, 13, leadFreeWnd, WM_CF_SHOW, TEXT_CF_LEFT | TEXT_CF_VCENTER, 0, buf);
	TEXT_SetTextColor(timeHandle, GUI_WHITE);
	TEXT_SetFont(timeHandle, &GUI_Font8x13_1);
	WM_EnableMemdev(timeHandle);

	IMAGE_Handle indHandle = IMAGE_CreateEx(118, 205, 32, 32, leadFreeWnd, WM_CF_SHOW, IMAGE_CF_MEMDEV, 0);
	IMAGE_SetBitmap(indHandle, &bm_1bpp_prep);
	WM_EnableMemdev(indHandle);

	WM_HWIN graphHandle = GRAPH_CreateEx(0, 0, 320, 200, leadFreeWnd, WM_CF_SHOW, GRAPH_CF_GRID_FIXED_X, 0);
	GRAPH_SetGridVis(graphHandle, 1);
	GRAPH_SetColor(graphHandle, 0x00E0E0E0, GRAPH_CI_GRID);
	GRAPH_SetLineStyleH(graphHandle, GUI_LS_DOT);
	GRAPH_SetLineStyleV(graphHandle, GUI_LS_DOT);
	GRAPH_SetGridDistX(graphHandle, 30);
	GRAPH_SetGridDistY(graphHandle, 16);
	WM_EnableMemdev(graphHandle);

	GRAPH_DATA_Handle pwrData;
	GRAPH_DATA_Handle targetData;
	if (showDebug)
	{
		int16_t pwrRawData[320];
		pwrData = GRAPH_DATA_YT_Create(GUI_BLUE, 320, pwrRawData, 0);
		GRAPH_DATA_YT_SetAlign(pwrData, GRAPH_ALIGN_LEFT);
		GRAPH_AttachData(graphHandle, pwrData);
	}

	int16_t targetRawData[320];
	targetData = GRAPH_DATA_YT_Create(GUI_GREEN, 320, targetRawData, 0);
	GRAPH_DATA_YT_SetAlign(targetData, GRAPH_ALIGN_LEFT);
	GRAPH_AttachData(graphHandle, targetData);

	int16_t temp1RawData[320];
	GRAPH_DATA_Handle temp1Data = GRAPH_DATA_YT_Create(GUI_RED, 320, temp1RawData, 0);
	GRAPH_DATA_YT_SetAlign(temp1Data, GRAPH_ALIGN_LEFT);
	GRAPH_AttachData(graphHandle, temp1Data);

	int16_t temp2RawData[320];
	GRAPH_DATA_Handle temp2Data = GRAPH_DATA_YT_Create(GUI_YELLOW, 320, temp2RawData, 0);
	GRAPH_DATA_YT_SetAlign(temp2Data, GRAPH_ALIGN_LEFT);
	GRAPH_AttachData(graphHandle, temp2Data);

	WM_HWIN overlayHandle = TEXT_CreateEx(40, 60, 280, 80, leadFreeWnd, WM_CF_SHOW, TEXT_CF_LEFT | TEXT_CF_VCENTER, 0, "");
	TEXT_SetTextColor(overlayHandle, GUI_WHITE);
	TEXT_SetFont(overlayHandle, &GUI_Font20_1);
	WM_EnableMemdev(overlayHandle);

	focus(controls.handles[controls.focusedControl]);

	printf("Start PID autotune\r\n");

	bool startPressed = false;
	bool reflow = false;
	int tempCounter = 0;
#ifdef MARLIN_AUTOTUNE
	bias = d = MAX_POWER / 2.;
#endif
	power = 0;
	SetOven(power); // switch off all heaters.

	for (;;)
	{
		GUI_Exec();
		int res = GUI_GetKey();
		if (res > 0)
		{
			switch (res)
			{
			case REFLOW_START:
				startPressed = true;
				break;
			case REFLOW_STOP:
				SetOven(MIN_POWER);
				startPressed = false;
				WM_DeleteWindow(leadFreeWnd);
				return;
			}
		}
		processButtons(time, &controls);

		ms = osKernelSysTick();
		// wait for 40°C
		if (!startPressed && !stop && temp < 40.)
		{
			startPressed = false;
			start = true;
			reflow = true;
			//power = MAX_POWER / 2.;
			power = MAX_POWER;
			SetOven(power);
		}
		if (!stop && start)
		{
			input = temp;
#ifdef MARLIN_AUTOTUNE

			maxVal = fmaxf(maxVal, input);
			minVal = fminf(minVal, input);

			// if currently heating and sensor temperature exceed target
			if (heating == true && input > targetTemp)
			{
				if (ms - t2 > 5000)
				{
					heating = false;
					power = (bias - d) / 2.f;
					SetOven(power);
					t1 = ms;
					t_high = t1 - t2;
					maxVal = targetTemp;
				}
			}
			if (heating == false && input < targetTemp)
			{
				if (ms - t1 > 5000)
				{
					heating = true;
					t2 = ms;
					t_low = t2 - t1;
					if (cycles > 0)
					{
						float max_pow = MAX_POWER;
						bias += (d * (t_high - t_low)) / (t_low + t_high);
						bias = fminf(fmaxf(bias, 20.), max_pow - 20.);
						d = (bias > max_pow / 2.) ? max_pow - 1. - bias : bias;

						printf(" bias: %.4f d: %.4f min: %.2f max: %.2f\r\n", bias, d, minVal, maxVal);
						if (cycles > 2)
						{
							Ku = (4.0 * d) / (M_PI * (maxVal - minVal) / 2.0);
							Tu = ((float)(t_low + t_high) / 1000.0);
							printf(" Ku: %.6f Tu: %.6f\r\n", Ku, Tu);
							/*
							Kp = 0.6 * Ku;
							Ki = 2 * Kp / Tu;
							Kd = Kp * Tu / 8;
							printf(" Classic PID Kp:\r\n %.6f\r\n Ki: %.6f\r\n Kd: %.6f\r\n", Kp, Ki, Kd);
							*/
							/*
							Kp = 0.33 * Ku;
							Ki = Kp / Tu;
							Kd = Kp * Tu / 3;
							printf(" Some overshoot Kp:\r\n %.6f\r\n Ki: %.6f\r\n Kd: %.6f\r\n", Kp, Ki, Kd);
							*/
							Kp = 0.2 * Ku;
							Ki = 2 * Kp / Tu;
							Kd = Kp * Tu / 3;
							printf(" No overshoot\r\n Kp: %.6f\r\n Ki: %.6f\r\n Kd: %.6f\r\n", Kp, Ki, Kd);
						}
					}
					power = (bias + d) / 2.f;
					SetOven(power);
					cycles++;
					minVal = targetTemp;
				}
			}
			if (input > targetTemp + 80)
			{
				printf("PID Autotune failed! Temperature too high %.2f > %.2f\r\n", input, targetTemp);
				SetOven(MIN_POWER);
				return;
			}
			// Every 2 seconds...
			if (ms > temp_millis + 2000)
			{
				//printf("ok T:%.2f  @%.4f\r\n", input, power);
				temp_millis = ms;
			}
			// Over 2 minutes?
			if (((ms - t1) + (ms - t2)) > (10L * 60L * 1000L * 2L))
			{
				printf("PID Autotune failed! timeout\r\n");
				SetOven(MIN_POWER);
				return;
			}
			if (cycles > ncycles)
			{
				flashPidParams(Kp, Ki, Kd);
				printf("PID Autotune finished!\r\n");
				power = MIN_POWER;
				SetOven(power);
				stop = true;
				reflow = false;
				TEXT_SetText(overlayHandle, "PID Autotune finished!!!");
			}
#elif defined KLIPPER_AUTOTUNE
			if (heating == true && input >= currTargetTemp)
			{
				heating = false;
				currTargetTemp -= TUNE_PID_DELTA;
				peaks[currPeak] = peak;
				peakTimes[currPeak] = peakTime;
				currPeak++;
				peak = 9999999.;
			} else if (heating == false && input <= currTargetTemp) {
				heating = true;
				currTargetTemp = targetTemp;
				peaks[currPeak] = peak;
				peakTimes[currPeak] = peakTime;
				currPeak++;
				peak = -9999999.;
			}
			if (currPeak == PEAKS_CYCLES) {
				// calculate Kp, Ki, Kd
				float cycle_times[PEAKS_CYCLES - 4];
				int positions[PEAKS_CYCLES - 4];

				for (int i = 4; i < PEAKS_CYCLES - 1; i++) {
					cycle_times[i - 4] = peakTimes[i] - peakTimes[i - 2];
					positions[i - 4] = 4;
				}
				selectionSort(cycle_times, positions, PEAKS_CYCLES - 4);
				int midpoint_pos = positions[(PEAKS_CYCLES - 4) / 2];

		        float temp_diff = peaks[midpoint_pos] - peaks[midpoint_pos - 1];
		        // calculate time difference in seconds
		        float time_diff = (peakTimes[midpoint_pos] - peakTimes[midpoint_pos - 2]) / 1000.;
		        // Use Astrom-Hagglund method to estimate Ku and Tu
		        float amplitude = .5 * abs(temp_diff);
		        Ku = 4. * MAX_POWER / (M_PI * amplitude);
		        Tu = time_diff;
		        // Use Ziegler-Nichols method to generate PID parameters
		        Ti = 0.5 * Tu;
				Td = 0.125 * Tu;
		        Kp = 0.6 * Ku;
		        Ki = Kp / Ti;
		        Kd = Kp * Td;
		        printf("Autotune: raw=%f/%f Ku=%f Tu=%f  Kp=%f Ki=%f Kd=%f\r\n", temp_diff, MAX_POWER, Ku, Tu, Kp, Ki, Kd);

				flashPidParams(Kp, Ki, Kd);
				printf("PID Autotune finished!\r\n");
				power = MIN_POWER;
				SetOven(power);
				stop = true;
				reflow = false;
				TEXT_SetText(overlayHandle, "PID Autotune finished!!!");
			}
			if (!stop) {
				if (heating)
				{
					if (power != MAX_POWER)
					{
						power = MAX_POWER;
						SetOven(power);
					}
					if (input > peak)
					{
						peak = input;
						peakTime = ms;
					}
				} else {
					if (power != MIN_POWER)
					{
						power = MIN_POWER;
						SetOven(power);
					}
					if (input < peak)
					{
						peak = input;
						peakTime = ms;
					}
				}
			}
#endif
		}
		// each second show both sensor temperature and time
		if (!stop && (time % CYCLES_PER_SECOND) == 0)
		{
			sprintf(buf, "Temp %.1f°C  ", temp1);
			TEXT_SetText(temp1Handle, buf);

			sprintf(buf, "Temp %.1f°C  ", temp2);
			TEXT_SetText(temp2Handle, buf);

			sprintf(buf, "Time %ds  ", seconds);
			TEXT_SetText(timeHandle, buf);

			printf("%d; %.3f; %.3f\r\n", seconds, power, temp);
		}
		if (stop && (time % CYCLES_PER_HALFSECOND) == 0)
		{
			if (colorSwap)
			{
				TEXT_SetTextColor(overlayHandle, GUI_WHITE);
				colorSwap = false;
			}
			else
			{
				TEXT_SetTextColor(overlayHandle, GUI_RED);
				colorSwap = true;
			}
		}
		if (reflow)
		{
			// hide power indicator if oven is off
			if (power == MIN_POWER)
			{
				IMAGE_SetBitmap(indHandle, &bm_1bpp_off);
			}
			else
			{
				// show power indicator if oven power > 0
				IMAGE_SetBitmap(indHandle, &bm_1bpp_on);
			}
			// each second
			if ((time % CYCLES_PER_SECOND) == 0)
			{
				seconds++;
			}
			// each 2 seconds
			if ((time % CYCLES_PER_SECOND) == 0)
			{
				// add next point to graphs
				if (tempCounter < 320)
				{
					yPos = roundf(temp1 * 0.8f);
					GRAPH_DATA_YT_AddValue(temp1Data, yPos);

					yPos2 = roundf(temp2 * 0.8f);
					GRAPH_DATA_YT_AddValue(temp2Data, yPos2);

					yTargetPos = roundf(currTargetTemp * 0.8f);
					GRAPH_DATA_YT_AddValue(targetData, yTargetPos);

					if (showDebug)
					{
						yPowerPos = roundf((float)power * 0.8f);
						GRAPH_DATA_YT_AddValue(pwrData, yPowerPos);
					}

					tempCounter++;
				}
			}
		}
		time++;
		osDelay(CYCLE_DELAY);
	}
}

float GetPidOutput(int time)
{
	float pid_output;
	float pTerm;
	float iTerm;
	float pid_error = target_temperature - current_temperature;
	pTerm = Kp * pid_error;
	temp_iState += pid_error;
	temp_iState = fminf(fmaxf(temp_iState, temp_iState_min), temp_iState_max);
	iTerm = Ki * temp_iState;

	dTerm = K2 * Kd * (current_temperature - temp_dState) + K1 * dTerm;
	temp_dState = current_temperature;

	pid_output = pTerm + iTerm - dTerm;
	if (pid_output > MAX_POWER)
	{
		if (pid_error > 0)
			temp_iState -= pid_error; // conditional un-integration
		pid_output = MAX_POWER;
	}
	else if (pid_output < 0)
	{
		if (pid_error < 0)
			temp_iState -= pid_error; // conditional un-integration
		pid_output = 0;
	}

	if (showDebug && (time % CYCLES_PER_SECOND) == 0)
	{
		printf("PID_DEBUG Input %.2f°C Target %.2f°C Output %.2f%% pTerm %.6f iTerm %.6f dTerm %.6f\r\n", temp, target_temperature, pid_output, pTerm, iTerm, dTerm);
	}

	return pid_output;
}

#ifdef PHASE_POWER
#define MAX_POWER_PULSE_WIDTH 9800.f
#elif defined PULSE_POWER
#define MAX_POWER_PULSE_WIDTH 100.f
#endif
/**
 * @power can be between 0 - 100.0
 */
void SetOven(float power)
{
	static int oldPulse = -1;
	//static int oldPower = 0;
#ifdef PHASE_POWER
	float cosine = (sqrtf(power / 25.f)) - 1.f;

	int pulse = MAX_POWER_PULSE_WIDTH;
	// cosine 1 - Phase 0° - maximal power
	// cosine -1 - Phase 180° - power 0W
	if (cosine > 1.f)
	{
		cosine = 1;
	}
	if (cosine < -1.f)
	{
		pulse = MAX_POWER_PULSE_WIDTH;
	}
	else
	{
		float x = acosf(cosine);
		pulse = floorf(x * (float)MAX_POWER_PULSE_WIDTH / M_PI);
	}
	if (oldPulse != pulse)
	{
		LED1 = ~LED1;
		/* Set the Capture Compare Register value */
		TIM2->CCR2 = pulse;
#elif defined PULSE_POWER
	int pulse = (int)roundf((MAX_POWER_PULSE_WIDTH - (power * MAX_POWER_PULSE_WIDTH / MAX_POWER)));
	pulse = min(max(pulse, 0), MAX_POWER_PULSE_WIDTH);
	if (oldPulse != pulse)
	{
		LED1 = ~LED1;
		/* Set the Capture Compare Register value */
		TIM2->CCR2 = pulse;
#endif
		if (pulse == MAX_POWER_PULSE_WIDTH)
		{
			HAL_TIM_OC_Stop(&htim2, TIM_CHANNEL_2);
		}
		else
		{
			HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);
		}
		oldPulse = pulse;
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_USB;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief CRC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CRC_Init(void)
{

	/* USER CODE BEGIN CRC_Init 0 */

	/* USER CODE END CRC_Init 0 */

	/* USER CODE BEGIN CRC_Init 1 */

	/* USER CODE END CRC_Init 1 */
	hcrc.Instance = CRC;
	if (HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN CRC_Init 2 */

	/* USER CODE END CRC_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */
}

/**
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDIO_SD_Init(void)
{

	/* USER CODE BEGIN SDIO_Init 0 */

	/* USER CODE END SDIO_Init 0 */

	/* USER CODE BEGIN SDIO_Init 1 */

	/* USER CODE END SDIO_Init 1 */
	hsd.Instance = SDIO;
	hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
	hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd.Init.ClockDiv = 0;
	/* USER CODE BEGIN SDIO_Init 2 */

	/* USER CODE END SDIO_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/**
	 *	SPI
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_7;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN SPI1_Init 2 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	__HAL_SPI_ENABLE(&hspi1);
	/* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* USER CODE BEGIN SPI2_Init 0 */
	/**
	 *	SPI2
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_15;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
#ifdef MAX31865_SENS
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
#elif defined MAX6675_SENS
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
#elif defined MAX31856_SENS
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
#endif
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 7;

	if (HAL_SPI_Init(&hspi2) != HAL_OK)
	{
		Error_Handler();
	}

#ifdef MAX6675_SENS
	max_gpio1.MISO_PIN = GPIO_PIN_14;
	max_gpio1.MISO_PORT = GPIOB;
	max_gpio1.MOSI_PIN = GPIO_PIN_15;
	max_gpio1.MOSI_PORT = GPIOB;
	max_gpio1.CLK_PIN = GPIO_PIN_13;
	max_gpio1.CLK_PORT = GPIOB;
	max_gpio1.SPI = &hspi2;
	max_gpio1.CE_PIN = GPIO_PIN_12;
	max_gpio1.CE_PORT = GPIOB;
#elif defined MAX31856_SENS
	tc1.SPI = tc0.SPI = &hspi2;
	tc1.CE_PIN = tc0.CE_PIN = GPIO_PIN_12;
	tc0.CE_PORT = GPIOB;
	tc1.CE_PORT = GPIOD;
#endif

	/* USER CODE BEGIN SPI2_Init 2 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	TC1_CS = 1;

	/* USER CODE BEGIN SPI2_Init 2 */
	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	TC2_CS = 1;

	__HAL_SPI_ENABLE(&hspi2);
	/* USER CODE END SPI2_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	GPIO_InitTypeDef GPIO_InitStruct = {0};

#ifdef PHASE_POWER
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 70;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = MAX_POWER_PULSE_WIDTH;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_SINGLE) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
	sSlaveConfig.InputTrigger = TIM_TS_ETRF;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_INVERTED;
	sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = MAX_POWER_PULSE_WIDTH;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);
#elif defined PULSE_POWER
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = MAX_POWER_PULSE_WIDTH;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE1;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_OnePulse_Init(&htim2, TIM_OPMODE_REPETITIVE) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
	sSlaveConfig.InputTrigger = TIM_TS_ETRF;
	sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_NONINVERTED;
	sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = MAX_POWER_PULSE_WIDTH;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);
#endif

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);
}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void)
{

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 718;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 1000;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 999;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);
	HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_1);
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 921600;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */
	__HAL_UART_ENABLE(&huart1);
	/* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED1_Pin | LED2_Pin | USB_EN_Pin, GPIO_PIN_RESET);
	LED1 = LED2 = USB_EN = GPIO_PIN_RESET;

	/*Configure GPIO pin Output Level */
	//HAL_GPIO_WritePin(GPIOD, OVEN_EN_Pin | COOLER_EN_Pin, GPIO_PIN_RESET);
	COOLER_EN = GPIO_PIN_RESET;

	/*Configure GPIO pin : USER_KEYA_Pin */
	GPIO_InitStruct.Pin = USER_KEYA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_KEYA_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED1_Pin USB_EN_Pin */
	GPIO_InitStruct.Pin = LED1_Pin | USB_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : LED2_Pin */
	GPIO_InitStruct.Pin = LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USER_KEYB_Pin */
	GPIO_InitStruct.Pin = USER_KEYB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_KEYB_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OVEN_EN_Pin COOLER_EN_Pin */
	GPIO_InitStruct.Pin = /*OVEN_EN_Pin | */ COOLER_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : SD_CD_Pin */
	GPIO_InitStruct.Pin = SD_CD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SD_CD_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : TOUCH_IRQ_Pin */
	GPIO_InitStruct.Pin = TOUCH_IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(TOUCH_IRQ_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : KEY1, KEY2, KEY3 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : KEYS_GND */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	KEYS_GND = 1;
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

	/* USER CODE BEGIN FSMC_Init 0 */

	/* USER CODE END FSMC_Init 0 */

	FSMC_NORSRAM_TimingTypeDef Timing = {0};

	/* USER CODE BEGIN FSMC_Init 1 */

	/* USER CODE END FSMC_Init 1 */

	/** Perform the SRAM1 memory initialization sequence
	 */
	hsram1.Instance = FSMC_NORSRAM_DEVICE;
	hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
	/* hsram1.Init */
	hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
	hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
	hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
	hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
	hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
	hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
	hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
	hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
	hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
	hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
	hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
	hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
	hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
	/* Timing */
	Timing.AddressSetupTime = 15;
	Timing.AddressHoldTime = 15;
	Timing.DataSetupTime = 255;
	Timing.BusTurnAroundDuration = 15;
	Timing.CLKDivision = 16;
	Timing.DataLatency = 17;
	Timing.AccessMode = FSMC_ACCESS_MODE_A;
	/* ExtTiming */

	if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
	{
		Error_Handler();
	}

	/** Disconnect NADV
	 */

	__HAL_AFIO_FSMCNADV_DISCONNECTED();

	/* USER CODE BEGIN FSMC_Init 2 */

	/* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1)
	{
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

	/* USER CODE END Error_Handler_Debug */
}

int _write(int file, char *data, int len)
{
	if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
	{
		errno = EBADF;
		return -1;
	}

	// arbitrary timeout 1000
	HAL_StatusTypeDef status =
		HAL_UART_Transmit(&huart1, (uint8_t *)data, len, 1000);

	// return # of bytes written - as best we can tell
	return (status == HAL_OK ? len : 0);
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
