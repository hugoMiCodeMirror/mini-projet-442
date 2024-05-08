/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include "stm32746g_discovery_audio.h"
#include "stdio.h"
#include "math.h"
#include "fatfs_storage.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "Images/images_h/apple_#81CD4B.h"
#include "Images/images_h/bottom-left_#81CD4B.h"
#include "Images/images_h/bottom-right_#81CD4B.h"
#include "Images/images_h/bottom-top_#81CD4B.h"
#include "Images/images_h/head-bottom_#81CD4B.h"
#include "Images/images_h/head-top_#81CD4B.h"
#include "Images/images_h/head-left_#81CD4B.h"
#include "Images/images_h/head-right_#81CD4B.h"
#include "Images/images_h/left-right_#81CD4B.h"
#include "Images/images_h/left-top_#81CD4B.h"
#include "Images/images_h/right-top_#81CD4B.h"
#include "Images/images_h/tail-bottom_#81CD4B.h"
#include "Images/images_h/tail-top_#81CD4B.h"
#include "Images/images_h/tail-left_#81CD4B.h"
#include "Images/images_h/tail-right_#81CD4B.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARBG8888_BYTE_PER_PIXEL 4
#define SDRAM_WRITE_READ_ADDR ((uint32_t)(LCD_FB_START_ADDRESS + (RK043FN48H_WIDTH * RK043FN48H_HEIGHT * ARBG8888_BYTE_PER_PIXEL)))

#define AUDIO_REC_START_ADDR SDRAM_WRITE_READ_ADDR
#define AUDIO_BLOCK_SIZE ((uint32_t)512)
#define AUDIO_BUFFER_OUT (AUDIO_REC_START_ADDR + (AUDIO_BLOCK_SIZE * 2))

#define Song_Name (const TCHAR *)"song.WAV"
#define NB_APPLES 4
#define NB_PALIERS 4
#define GRID_SIZE_X 15
#define GRID_SIZE_Y 8
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;

DMA2D_HandleTypeDef hdma2d;

LTDC_HandleTypeDef hltdc;

SAI_HandleTypeDef hsai_BlockA2;
SAI_HandleTypeDef hsai_BlockB2;
DMA_HandleTypeDef hdma_sai2_a;
DMA_HandleTypeDef hdma_sai2_b;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;
SDRAM_HandleTypeDef hsdram2;

osThreadId defaultTaskHandle;
osThreadId displayTaskHandle;
osThreadId manageBodyPartsHandle;
osThreadId tsHandlerTaskHandle;
osThreadId playSongTaskHandle;
osMessageQId WakeUpHandle;
osMutexId displayMutexHandle;
/* USER CODE BEGIN PV */
uint32_t numberOfBlocks = 0;
uint32_t blockPointer = 0;
uint32_t audioFrequency = 44100;
uint32_t bytesPerSecond = 1;

char *pDirectoryFiles[MAX_BMP_FILES];
uint8_t *uwInternelBuffer; // Buffer pour la mémoire SDRAM
uint8_t *uwInternelBuffer2;

static TS_StateTypeDef TS_State;
uint32_t screenPressed = 0;
uint32_t screenReleased = 0;

const uint8_t initSpeed = 2; // Fréquence de rafraîchissement en Hz
const uint8_t palierIncreaseSpeed[NB_PALIERS] = {4, 12, 22, 50};

uint8_t speed = initSpeed;

uint32_t joystick_v;
uint32_t joystick_h;
extern ADC_ChannelConfTypeDef sConfig;

enum Direction
{
	Up,
	Down,
	Left,
	Right
};
enum Direction direction = Up;

enum HeadPart
{
	HeadTop,
	HeadBottom,
	HeadLeft,
	HeadRight
};
enum HeadPart headPart = HeadTop;

enum BodyPart
{
	BottomLeft,
	BottomRight,
	BottomTop,
	LeftRight,
	LeftTop,
	RightTop
};
enum BodyPart snakeBodyParts[GRID_SIZE_X * GRID_SIZE_Y];

enum TailPart
{
	TailBottom,
	TailTop,
	TailLeft,
	TailRight
};
enum TailPart tailPart = TailTop;

uint8_t snakeSize = 0;
uint32_t appleEaten = 0;
uint32_t gameStarted = 0;
uint32_t gamePaused = 0;
uint32_t gameOver = 0;
uint32_t lastMove = 1; // dernier déplacement du snake avant la mort

uint8_t snakeHeadPosition[2] = {7, 6};
uint8_t snakeBodyPosition[GRID_SIZE_X * GRID_SIZE_Y][2] = {};
uint8_t snakeTailPosition[2] = {7, 7};
uint8_t oldTailPosition[2];
int8_t applePosition[NB_APPLES][2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DMA2D_Init(void);
static void MX_SAI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
void StartDefaultTask(void const *argument);
void StartDisplayTask(void const *argument);
void StartManageBodyParts(void const *argument);
void StartTsHandler(void const *argument);
void StartPlaySongTask(void const *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
enum BodyPart whatBodyPart(uint8_t frontX, uint8_t frontY, uint8_t backX, uint8_t backY, uint8_t missingX, uint8_t missingY)
{
	// Cette fonction permet de déterminer le type de partie du corps du snake en fonction de la position de celle de devant et de celle de derrière
	/*
	  . F .
	  . X .
	  . B .
	*/
	if (frontX == backX)
		return BottomTop;

	/*
	  . . .
	  F X B
	  . . .
	*/
	if (frontY == backY)
		return LeftRight;

	/*
	  . B .       . . .
	  F X .       . X B
	  . . .       . F .
	*/
	if (frontX < backX && frontY > backY)
	{
		if (missingX == frontX)
			return BottomRight;
		else
			return LeftTop;
	}

	/*
	  . . .       . F .
	  F X .       . X B
	  . B .       . . .
	*/
	if (frontX < backX && frontY < backY)
	{
		if (missingX == frontX)
			return RightTop;
		else
			return BottomLeft;
	}

	/*
	  . . .       . F .
	  . X F       B X .
	  . B .       . . .
	*/
	if (frontX > backX && frontY < backY)
	{
		if (missingX == frontX)
			return LeftTop;
		else
			return BottomRight;
	}

	/*
	  . B .       . . .
	  . X F       B X .
	  . . .       . F .
	*/
	if (frontX > backX && frontY > backY)
	{
		if (missingX == frontX)
			return BottomLeft;
		else
			return RightTop;
	}

	return BottomTop; // ne devrait jamais arriver
}

uint8_t isSnakePosition(uint8_t x, uint8_t y)
{
	if (x == snakeHeadPosition[0] && y == snakeHeadPosition[1])
		return 1;

	for (int i = 0; i < snakeSize; i++)
	{
		if (x == snakeBodyPosition[i][0] && y == snakeBodyPosition[i][1])
			return 1;
	}

	if (x == snakeTailPosition[0] && y == snakeTailPosition[1])
		return 1;

	return 0;
}

uint8_t isApplePosition(uint8_t x, uint8_t y, uint8_t appleIndex)
{
	for (int i = 0; i < NB_APPLES; i++)
	{
		if (i == appleIndex)
			continue;

		if (x == applePosition[i][0] && y == applePosition[i][1])
			return 1;
	}

	return 0;
}

void updateJoystickDirection()
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	sConfig.Channel = ADC_CHANNEL_8;

	HAL_ADC_ConfigChannel(&hadc3, &sConfig);
	HAL_ADC_Start(&hadc3);
	while (HAL_ADC_PollForConversion(&hadc3, 100) != HAL_OK)
		;
	joystick_v = HAL_ADC_GetValue(&hadc3);

	HAL_ADC_Start(&hadc1);
	while (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK)
		;
	joystick_h = HAL_ADC_GetValue(&hadc1);

	// On actualise la direction du snake
	if (joystick_v < 1000 && headPart != HeadTop)
	{
		direction = Down;
	}
	else if (joystick_v > 3000 && headPart != HeadBottom)
	{
		direction = Up;
	}
	else if (joystick_h < 1000 && headPart != HeadLeft)
	{
		direction = Right;
	}
	else if (joystick_h > 3000 && headPart != HeadRight)
	{
		direction = Left;
	}
}

void restartGame()
{
	gameOver = 0;
	lastMove = 1;
	snakeSize = 0;
	speed = initSpeed;
	snakeHeadPosition[0] = 7;
	snakeHeadPosition[1] = 6;
	snakeTailPosition[0] = 7;
	snakeTailPosition[1] = 7;
	direction = Up;
	headPart = HeadTop;
	tailPart = TailTop;

	for (int i = 0; i < NB_APPLES; i++)
	{
		do
		{
			applePosition[i][0] = rand() % GRID_SIZE_X;
			applePosition[i][1] = rand() % GRID_SIZE_Y;
		} while (isSnakePosition(applePosition[i][0], applePosition[i][1]) || isApplePosition(applePosition[i][0], applePosition[i][1], i));
	}

	BSP_LCD_Clear((uint32_t)0xFF81CD4B);
}

void displayGameStatus()
{
	// On affiche le texte en fonction de l'état du jeu
	if (gameOver)
	{
		BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
		BSP_LCD_SetFont(&Font24);
		BSP_LCD_DisplayStringAt(0, 100, (uint8_t *)"Game Over", CENTER_MODE);
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_DisplayStringAt(0, 130, (uint8_t *)"Touch the screen to restart", CENTER_MODE);
	}
	else if (gamePaused)
	{
		BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
		BSP_LCD_SetFont(&Font24);
		BSP_LCD_DisplayStringAt(0, 100, (uint8_t *)"Game Paused", CENTER_MODE);
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_DisplayStringAt(0, 130, (uint8_t *)"Touch the screen to resume", CENTER_MODE);
	}
	else if (!gameStarted)
	{
		BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
		BSP_LCD_SetFont(&Font24);
		BSP_LCD_DisplayStringAt(0, 100, (uint8_t *)"Snake Game", CENTER_MODE);
		BSP_LCD_SetFont(&Font16);
		BSP_LCD_DisplayStringAt(0, 130, (uint8_t *)"Touch the screen to start", CENTER_MODE);
	}
}

void initializeSD()
{
	if (f_mount(&SDFatFS, (TCHAR const *)SDPath, 0) != FR_OK)
	{
		Error_Handler();
	}
}

void initializeAudio(uint32_t freq)
{
	// Initialisation de l'audio
	if (BSP_AUDIO_IN_OUT_Init(INPUT_DEVICE_INPUT_LINE_1,
							  OUTPUT_DEVICE_HEADPHONE, freq,
							  DEFAULT_AUDIO_IN_BIT_RESOLUTION,
							  DEFAULT_AUDIO_IN_CHANNEL_NBR) != AUDIO_OK)
	{
		Error_Handler();
	}

	// Initialisation du buffer audio
	memset((uint16_t *)AUDIO_BUFFER_OUT, 0, AUDIO_BLOCK_SIZE * 2);

	// Démarrage de l'audio
	BSP_AUDIO_OUT_SetVolume(40);
	BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);

	if (BSP_AUDIO_OUT_Play((uint16_t *)AUDIO_BUFFER_OUT,
						   AUDIO_BLOCK_SIZE * 2) != AUDIO_OK)
	{
		Error_Handler();
	}
}

void extractHeaderInfo()
{
	uint32_t data = 0;
	uint32_t bytesread;

	// Lecture de la taille du fichier
	f_lseek(&SDFile, 04);
	f_read(&SDFile, &data, 4, (void *)&bytesread);
	numberOfBlocks = data / 512;
	data = 0;

	// Lecture de la fréquence d'échantillonnage
	f_lseek(&SDFile, 24);
	f_read(&SDFile, &data, 4, (void *)&bytesread);
	audioFrequency = data;
	data = 0;

	// Nombre d'octets par secondes
	f_lseek(&SDFile, 28);
	f_read(&SDFile, (uint8_t *)&data, 4, (void *)&bytesread);
	bytesPerSecond = data;
}

void loadWav()
{
	f_close(&SDFile);
	f_open(&SDFile, Song_Name, FA_READ);
	extractHeaderInfo();
	initializeAudio(audioFrequency);
	f_lseek(&SDFile, 44);
	blockPointer = 0;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	/* USER CODE END 1 */

	/* Enable I-Cache---------------------------------------------------------*/
	SCB_EnableICache();

	/* Enable D-Cache---------------------------------------------------------*/
	SCB_EnableDCache();

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SDMMC1_SD_Init();
	MX_FATFS_Init();
	MX_FMC_Init();
	MX_LTDC_Init();
	MX_USART1_UART_Init();
	MX_DMA2D_Init();
	MX_SAI2_Init();
	MX_ADC1_Init();
	MX_ADC3_Init();
	/* USER CODE BEGIN 2 */

	// Initialisation de l'écran LCD
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
	BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS + BSP_LCD_GetXSize() * BSP_LCD_GetYSize() * 4);
	BSP_LCD_DisplayOn();
	BSP_LCD_SelectLayer(0);
	BSP_LCD_Clear((uint32_t)0xFF81CD4B);
	BSP_LCD_SelectLayer(1);
	BSP_LCD_Clear(00);
	BSP_LCD_SetFont(&Font16);
	BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
	BSP_LCD_SetBackColor(00);
	BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	/* USER CODE END 2 */

	/* Create the mutex(es) */
	/* definition and creation of displayMutex */
	osMutexDef(displayMutex);
	displayMutexHandle = osMutexCreate(osMutex(displayMutex));

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* definition and creation of WakeUp */
	osMessageQDef(WakeUp, 1, uint8_t);
	WakeUpHandle = osMessageCreate(osMessageQ(WakeUp), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

	/* definition and creation of displayTask */
	osThreadDef(displayTask, StartDisplayTask, osPriorityLow, 0, 1024);
	displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);

	/* definition and creation of manageBodyParts */
	osThreadDef(manageBodyParts, StartManageBodyParts, osPriorityHigh, 0, 256);
	manageBodyPartsHandle = osThreadCreate(osThread(manageBodyParts), NULL);

	/* definition and creation of tsHandlerTask */
	osThreadDef(tsHandlerTask, StartTsHandler, osPriorityHigh, 0, 256);
	tsHandlerTaskHandle = osThreadCreate(osThread(tsHandlerTask), NULL);

	/* definition and creation of playSongTask */
	osThreadDef(playSongTask, StartPlaySongTask, osPriorityHigh, 0, 256);
	playSongTaskHandle = osThreadCreate(osThread(playSongTask), NULL);

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
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 400;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC | RCC_PERIPHCLK_SAI2 | RCC_PERIPHCLK_SDMMC1 | RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 5;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
	PeriphClkInitStruct.PLLSAIDivQ = 1;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
	PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
	PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */
}

/**
 * @brief ADC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC3_Init(void)
{

	/* USER CODE BEGIN ADC3_Init 0 */

	/* USER CODE END ADC3_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC3_Init 1 */

	/* USER CODE END ADC3_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc3.Init.Resolution = ADC_RESOLUTION_12B;
	hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc3) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC3_Init 2 */

	/* USER CODE END ADC3_Init 2 */
}

/**
 * @brief DMA2D Initialization Function
 * @param None
 * @retval None
 */
static void MX_DMA2D_Init(void)
{

	/* USER CODE BEGIN DMA2D_Init 0 */

	/* USER CODE END DMA2D_Init 0 */

	/* USER CODE BEGIN DMA2D_Init 1 */

	/* USER CODE END DMA2D_Init 1 */
	hdma2d.Instance = DMA2D;
	hdma2d.Init.Mode = DMA2D_M2M;
	hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
	hdma2d.Init.OutputOffset = 0;
	hdma2d.LayerCfg[1].InputOffset = 0;
	hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
	hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	hdma2d.LayerCfg[1].InputAlpha = 0;
	if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DMA2D_Init 2 */

	/* USER CODE END DMA2D_Init 2 */
}

/**
 * @brief LTDC Initialization Function
 * @param None
 * @retval None
 */
static void MX_LTDC_Init(void)
{

	/* USER CODE BEGIN LTDC_Init 0 */

	/* USER CODE END LTDC_Init 0 */

	LTDC_LayerCfgTypeDef pLayerCfg = {0};

	/* USER CODE BEGIN LTDC_Init 1 */

	/* USER CODE END LTDC_Init 1 */
	hltdc.Instance = LTDC;
	hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
	hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
	hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
	hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
	hltdc.Init.HorizontalSync = 40;
	hltdc.Init.VerticalSync = 9;
	hltdc.Init.AccumulatedHBP = 53;
	hltdc.Init.AccumulatedVBP = 11;
	hltdc.Init.AccumulatedActiveW = 533;
	hltdc.Init.AccumulatedActiveH = 283;
	hltdc.Init.TotalWidth = 565;
	hltdc.Init.TotalHeigh = 285;
	hltdc.Init.Backcolor.Blue = 0;
	hltdc.Init.Backcolor.Green = 0;
	hltdc.Init.Backcolor.Red = 0;
	if (HAL_LTDC_Init(&hltdc) != HAL_OK)
	{
		Error_Handler();
	}
	pLayerCfg.WindowX0 = 0;
	pLayerCfg.WindowX1 = 480;
	pLayerCfg.WindowY0 = 0;
	pLayerCfg.WindowY1 = 272;
	pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
	pLayerCfg.Alpha = 255;
	pLayerCfg.Alpha0 = 0;
	pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
	pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
	pLayerCfg.FBStartAdress = 0xC0000000;
	pLayerCfg.ImageWidth = 480;
	pLayerCfg.ImageHeight = 272;
	pLayerCfg.Backcolor.Blue = 0;
	pLayerCfg.Backcolor.Green = 0;
	pLayerCfg.Backcolor.Red = 0;
	if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN LTDC_Init 2 */

	/* USER CODE END LTDC_Init 2 */
}

/**
 * @brief SAI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SAI2_Init(void)
{

	/* USER CODE BEGIN SAI2_Init 0 */

	/* USER CODE END SAI2_Init 0 */

	/* USER CODE BEGIN SAI2_Init 1 */

	/* USER CODE END SAI2_Init 1 */
	hsai_BlockA2.Instance = SAI2_Block_A;
	hsai_BlockA2.Init.Protocol = SAI_FREE_PROTOCOL;
	hsai_BlockA2.Init.AudioMode = SAI_MODEMASTER_TX;
	hsai_BlockA2.Init.DataSize = SAI_DATASIZE_8;
	hsai_BlockA2.Init.FirstBit = SAI_FIRSTBIT_MSB;
	hsai_BlockA2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	hsai_BlockA2.Init.Synchro = SAI_ASYNCHRONOUS;
	hsai_BlockA2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockA2.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
	hsai_BlockA2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockA2.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
	hsai_BlockA2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockA2.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_BlockA2.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockA2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	hsai_BlockA2.FrameInit.FrameLength = 8;
	hsai_BlockA2.FrameInit.ActiveFrameLength = 1;
	hsai_BlockA2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
	hsai_BlockA2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
	hsai_BlockA2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
	hsai_BlockA2.SlotInit.FirstBitOffset = 0;
	hsai_BlockA2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
	hsai_BlockA2.SlotInit.SlotNumber = 1;
	hsai_BlockA2.SlotInit.SlotActive = 0x00000000;
	if (HAL_SAI_Init(&hsai_BlockA2) != HAL_OK)
	{
		Error_Handler();
	}
	hsai_BlockB2.Instance = SAI2_Block_B;
	hsai_BlockB2.Init.Protocol = SAI_FREE_PROTOCOL;
	hsai_BlockB2.Init.AudioMode = SAI_MODESLAVE_RX;
	hsai_BlockB2.Init.DataSize = SAI_DATASIZE_8;
	hsai_BlockB2.Init.FirstBit = SAI_FIRSTBIT_MSB;
	hsai_BlockB2.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
	hsai_BlockB2.Init.Synchro = SAI_SYNCHRONOUS;
	hsai_BlockB2.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
	hsai_BlockB2.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
	hsai_BlockB2.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
	hsai_BlockB2.Init.MonoStereoMode = SAI_STEREOMODE;
	hsai_BlockB2.Init.CompandingMode = SAI_NOCOMPANDING;
	hsai_BlockB2.Init.TriState = SAI_OUTPUT_NOTRELEASED;
	hsai_BlockB2.FrameInit.FrameLength = 8;
	hsai_BlockB2.FrameInit.ActiveFrameLength = 1;
	hsai_BlockB2.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
	hsai_BlockB2.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
	hsai_BlockB2.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
	hsai_BlockB2.SlotInit.FirstBitOffset = 0;
	hsai_BlockB2.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
	hsai_BlockB2.SlotInit.SlotNumber = 1;
	hsai_BlockB2.SlotInit.SlotActive = 0x00000000;
	if (HAL_SAI_Init(&hsai_BlockB2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SAI2_Init 2 */

	/* USER CODE END SAI2_Init 2 */
}

/**
 * @brief SDMMC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDMMC1_SD_Init(void)
{

	/* USER CODE BEGIN SDMMC1_Init 0 */

	/* USER CODE END SDMMC1_Init 0 */

	/* USER CODE BEGIN SDMMC1_Init 1 */

	/* USER CODE END SDMMC1_Init 1 */
	hsd1.Instance = SDMMC1;
	hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
	hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
	hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
	hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
	hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd1.Init.ClockDiv = 0;
	/* USER CODE BEGIN SDMMC1_Init 2 */

	/* USER CODE END SDMMC1_Init 2 */
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
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA2_Stream3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	/* DMA2_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
	/* DMA2_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
	/* DMA2_Stream7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
}

/* FMC initialization function */
static void MX_FMC_Init(void)
{

	/* USER CODE BEGIN FMC_Init 0 */

	/* USER CODE END FMC_Init 0 */

	FMC_SDRAM_TimingTypeDef SdramTiming = {0};

	/* USER CODE BEGIN FMC_Init 1 */

	/* USER CODE END FMC_Init 1 */

	/** Perform the SDRAM1 memory initialization sequence
	 */
	hsdram1.Instance = FMC_SDRAM_DEVICE;
	/* hsdram1.Init */
	hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
	hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
	hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
	hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
	hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
	hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
	hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
	hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
	hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
	/* SdramTiming */
	SdramTiming.LoadToActiveDelay = 16;
	SdramTiming.ExitSelfRefreshDelay = 16;
	SdramTiming.SelfRefreshTime = 16;
	SdramTiming.RowCycleDelay = 16;
	SdramTiming.WriteRecoveryTime = 16;
	SdramTiming.RPDelay = 16;
	SdramTiming.RCDDelay = 16;

	if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
	{
		Error_Handler();
	}

	/** Perform the SDRAM2 memory initialization sequence
	 */
	hsdram2.Instance = FMC_SDRAM_DEVICE;
	/* hsdram2.Init */
	hsdram2.Init.SDBank = FMC_SDRAM_BANK2;
	hsdram2.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
	hsdram2.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
	hsdram2.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
	hsdram2.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
	hsdram2.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_1;
	hsdram2.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	hsdram2.Init.SDClockPeriod = FMC_SDRAM_CLOCK_DISABLE;
	hsdram2.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
	hsdram2.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
	/* SdramTiming */
	SdramTiming.LoadToActiveDelay = 16;
	SdramTiming.ExitSelfRefreshDelay = 16;
	SdramTiming.SelfRefreshTime = 16;
	SdramTiming.RowCycleDelay = 16;
	SdramTiming.WriteRecoveryTime = 16;
	SdramTiming.RPDelay = 16;
	SdramTiming.RCDDelay = 16;

	if (HAL_SDRAM_Init(&hsdram2, &SdramTiming) != HAL_OK)
	{
		Error_Handler();
	}

	/* USER CODE BEGIN FMC_Init 2 */

	/* USER CODE END FMC_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOJ_CLK_ENABLE();
	__HAL_RCC_GPIOI_CLK_ENABLE();
	__HAL_RCC_GPIOK_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOI, ARDUINO_D7_Pin | ARDUINO_D8_Pin | LCD_DISP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOH, DCMI_PWR_EN_Pin | LED2_Pin | LED1_Pin | LED3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin | ARDUINO_D2_Pin | EXT_RST_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : QSPI_D2_Pin */
	GPIO_InitStruct.Pin = QSPI_D2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
	HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_TXD1_Pin RMII_TXD0_Pin RMII_TX_EN_Pin */
	GPIO_InitStruct.Pin = RMII_TXD1_Pin | RMII_TXD0_Pin | RMII_TX_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : ARDUINO_SCL_D15_Pin ARDUINO_SDA_D14_Pin */
	GPIO_InitStruct.Pin = ARDUINO_SCL_D15_Pin | ARDUINO_SDA_D14_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : ARDUINO_PWM_D3_Pin */
	GPIO_InitStruct.Pin = ARDUINO_PWM_D3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(ARDUINO_PWM_D3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SPDIF_RX0_Pin */
	GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
	HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ARDUINO_PWM_D9_Pin */
	GPIO_InitStruct.Pin = ARDUINO_PWM_D9_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	HAL_GPIO_Init(ARDUINO_PWM_D9_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DCMI_D6_Pin DCMI_D7_Pin */
	GPIO_InitStruct.Pin = DCMI_D6_Pin | DCMI_D7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_VBUS_Pin */
	GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Audio_INT_Pin */
	GPIO_InitStruct.Pin = Audio_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : OTG_FS_P_Pin OTG_FS_N_Pin OTG_FS_ID_Pin */
	GPIO_InitStruct.Pin = OTG_FS_P_Pin | OTG_FS_N_Pin | OTG_FS_ID_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DCMI_D5_Pin */
	GPIO_InitStruct.Pin = DCMI_D5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	HAL_GPIO_Init(DCMI_D5_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin LCD_DISP_Pin */
	GPIO_InitStruct.Pin = ARDUINO_D7_Pin | ARDUINO_D8_Pin | LCD_DISP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	/*Configure GPIO pin : uSD_Detect_Pin */
	GPIO_InitStruct.Pin = uSD_Detect_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_BL_CTRL_Pin */
	GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DCMI_VSYNC_Pin */
	GPIO_InitStruct.Pin = DCMI_VSYNC_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	HAL_GPIO_Init(DCMI_VSYNC_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
	GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : TP3_Pin NC2_Pin */
	GPIO_InitStruct.Pin = TP3_Pin | NC2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pin : ARDUINO_SCK_D13_Pin */
	GPIO_InitStruct.Pin = ARDUINO_SCK_D13_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(ARDUINO_SCK_D13_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DCMI_PWR_EN_Pin LED2_Pin LED1_Pin LED3_Pin */
	GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin | LED2_Pin | LED1_Pin | LED3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pins : DCMI_D4_Pin DCMI_D0_Pin */
	GPIO_InitStruct.Pin = DCMI_D4_Pin | DCMI_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pin : ARDUINO_PWM_CS_D5_Pin */
	GPIO_InitStruct.Pin = ARDUINO_PWM_CS_D5_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
	HAL_GPIO_Init(ARDUINO_PWM_CS_D5_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ARDUINO_PWM_D10_Pin */
	GPIO_InitStruct.Pin = ARDUINO_PWM_D10_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(ARDUINO_PWM_D10_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_INT_Pin */
	GPIO_InitStruct.Pin = LCD_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ARDUINO_RX_D0_Pin ARDUINO_TX_D1_Pin */
	GPIO_InitStruct.Pin = ARDUINO_RX_D0_Pin | ARDUINO_TX_D1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : ULPI_NXT_Pin */
	GPIO_InitStruct.Pin = ULPI_NXT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
	GPIO_InitStruct.Pin = ARDUINO_D4_Pin | ARDUINO_D2_Pin | EXT_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin ULPI_D2_Pin
							 ULPI_D1_Pin ULPI_D4_Pin */
	GPIO_InitStruct.Pin = ULPI_D6_Pin | ULPI_D5_Pin | ULPI_D3_Pin | ULPI_D2_Pin | ULPI_D1_Pin | ULPI_D4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
	GPIO_InitStruct.Pin = ULPI_STP_Pin | ULPI_DIR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
	GPIO_InitStruct.Pin = RMII_MDC_Pin | RMII_RXD0_Pin | RMII_RXD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PB2 */
	GPIO_InitStruct.Pin = GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : QSPI_D1_Pin QSPI_D3_Pin QSPI_D0_Pin */
	GPIO_InitStruct.Pin = QSPI_D1_Pin | QSPI_D3_Pin | QSPI_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : RMII_RXER_Pin */
	GPIO_InitStruct.Pin = RMII_RXER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
	GPIO_InitStruct.Pin = RMII_REF_CLK_Pin | RMII_MDIO_Pin | RMII_CRS_DV_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : DCMI_HSYNC_Pin PA6 */
	GPIO_InitStruct.Pin = DCMI_HSYNC_Pin | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
	GPIO_InitStruct.Pin = ULPI_CLK_Pin | ULPI_D0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_SDA_Pin */
	GPIO_InitStruct.Pin = LCD_SDA_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
	HAL_GPIO_Init(LCD_SDA_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : ARDUINO_MISO_D12_Pin ARDUINO_MOSI_PWM_D11_Pin */
	GPIO_InitStruct.Pin = ARDUINO_MISO_D12_Pin | ARDUINO_MOSI_PWM_D11_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
	char a = 1;
	xQueueSendFromISR(WakeUpHandle, &a, 0);
}

void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{
	char a = 0;
	xQueueSendFromISR(WakeUpHandle, &a, 0);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;)
	{
		updateJoystickDirection();
		osDelay(10);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
 * @brief Function implementing the displayTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const *argument)
{
	/* USER CODE BEGIN StartDisplayTask */
	vTaskSuspend(manageBodyPartsHandle);
	vTaskSuspend(tsHandlerTaskHandle);
	vTaskSuspend(playSongTaskHandle);
	vTaskDelay(100);

	uwInternelBuffer = (uint8_t *)0xC0260000;
	uwInternelBuffer2 = (uint8_t *)0xC0360000;

	uint8_t counter;

	/*##- Initialize the Directory Files pointers (heap) ###################*/
	for (counter = 0; counter < MAX_BMP_FILES; counter++)
	{
		pDirectoryFiles[counter] = malloc(MAX_BMP_FILE_NAME);
		if (pDirectoryFiles[counter] == NULL)
		{
			/* Set the Text Color */
			BSP_LCD_SetTextColor(LCD_COLOR_RED);

			BSP_LCD_DisplayStringAtLine(8,
										(uint8_t *)"  Cannot allocate memory ");

			while (1)
			{
			}
		}
	}

	// Lancement de la musique
	initializeSD();
	loadWav(0);

	vTaskResume(manageBodyPartsHandle);
	vTaskResume(tsHandlerTaskHandle);
	vTaskResume(playSongTaskHandle);

	vTaskDelay(1000);
	/* Infinite loop */
	for (;;)
	{
		if ((gameOver && !lastMove) || gamePaused || !gameStarted)
		{ // revoir la condition
			xSemaphoreTake(displayMutexHandle, portMAX_DELAY);
			displayGameStatus();
			xSemaphoreGive(displayMutexHandle);
		}
		else
		{
			// On convertit la direction en texte
			char directionText[100];
			switch (direction)
			{
			case Up:
				sprintf(directionText, (char *)"Direction: Up   ");
				break;
			case Down:
				sprintf(directionText, (char *)"Direction: Down ");
				break;
			case Left:
				sprintf(directionText, (char *)"Direction: Left ");
				break;
			case Right:
				sprintf(directionText, (char *)"Direction: Right");
				break;
			}
			// On convertit le score en texte
			char scoreText[100];
			sprintf(scoreText, (char *)"Score: %d", snakeSize);

			// On convertit la vitesse en texte
			char speedText[100];
			sprintf(speedText, (char *)"Speed: %d Hz", speed);

			xSemaphoreTake(displayMutexHandle, portMAX_DELAY);
			BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
			BSP_LCD_DrawHLine(0, 8 * 32, BSP_LCD_GetXSize());
			BSP_LCD_DrawHLine(0, 8 * 32 + 1, BSP_LCD_GetXSize());

			// On affiche la direction, le score et la vitesse
			BSP_LCD_DisplayStringAt(0, 8 * 32 + 2, (uint8_t *)directionText, LEFT_MODE);
			BSP_LCD_DisplayStringAt(200, 8 * 32 + 2, (uint8_t *)scoreText, LEFT_MODE);
			BSP_LCD_DisplayStringAt(350, 8 * 32 + 2, (uint8_t *)speedText, LEFT_MODE);

			// On affiche la tête du snake
			if (!gameOver)
				switch (headPart)
				{
				case HeadBottom:
					BSP_LCD_DrawBitmap(snakeHeadPosition[0] * 32, snakeHeadPosition[1] * 32, (uint8_t *)images_bmp_color_head_bottom_81CD4B_bmp);
					break;
				case HeadTop:
					BSP_LCD_DrawBitmap(snakeHeadPosition[0] * 32, snakeHeadPosition[1] * 32, (uint8_t *)images_bmp_color_head_top_81CD4B_bmp);
					break;
				case HeadLeft:
					BSP_LCD_DrawBitmap(snakeHeadPosition[0] * 32, snakeHeadPosition[1] * 32, (uint8_t *)images_bmp_color_head_left_81CD4B_bmp);
					break;
				case HeadRight:
					BSP_LCD_DrawBitmap(snakeHeadPosition[0] * 32, snakeHeadPosition[1] * 32, (uint8_t *)images_bmp_color_head_right_81CD4B_bmp);
					break;
				}

			// On affiche le corps du snake
			for (int i = 0; i < snakeSize; i++)
			{
				switch (snakeBodyParts[i])
				{
				case BottomLeft:
					BSP_LCD_DrawBitmap(snakeBodyPosition[i][0] * 32, snakeBodyPosition[i][1] * 32, (uint8_t *)images_bmp_color_bottom_left_81CD4B_bmp);
					break;
				case BottomRight:
					BSP_LCD_DrawBitmap(snakeBodyPosition[i][0] * 32, snakeBodyPosition[i][1] * 32, (uint8_t *)images_bmp_color_bottom_right_81CD4B_bmp);
					break;
				case BottomTop:
					BSP_LCD_DrawBitmap(snakeBodyPosition[i][0] * 32, snakeBodyPosition[i][1] * 32, (uint8_t *)images_bmp_color_bottom_top_81CD4B_bmp);
					break;
				case LeftRight:
					BSP_LCD_DrawBitmap(snakeBodyPosition[i][0] * 32, snakeBodyPosition[i][1] * 32, (uint8_t *)images_bmp_color_left_right_81CD4B_bmp);
					break;
				case LeftTop:
					BSP_LCD_DrawBitmap(snakeBodyPosition[i][0] * 32, snakeBodyPosition[i][1] * 32, (uint8_t *)images_bmp_color_left_top_81CD4B_bmp);
					break;
				case RightTop:
					BSP_LCD_DrawBitmap(snakeBodyPosition[i][0] * 32, snakeBodyPosition[i][1] * 32, (uint8_t *)images_bmp_color_right_top_81CD4B_bmp);
					break;
				}
			}

			// On affiche la queue du snake
			switch (tailPart)
			{
			case TailBottom:
				BSP_LCD_DrawBitmap(snakeTailPosition[0] * 32, snakeTailPosition[1] * 32, (uint8_t *)images_bmp_color_tail_bottom_81CD4B_bmp);
				break;
			case TailTop:
				BSP_LCD_DrawBitmap(snakeTailPosition[0] * 32, snakeTailPosition[1] * 32, (uint8_t *)images_bmp_color_tail_top_81CD4B_bmp);
				break;
			case TailLeft:
				BSP_LCD_DrawBitmap(snakeTailPosition[0] * 32, snakeTailPosition[1] * 32, (uint8_t *)images_bmp_color_tail_left_81CD4B_bmp);
				break;
			case TailRight:
				BSP_LCD_DrawBitmap(snakeTailPosition[0] * 32, snakeTailPosition[1] * 32, (uint8_t *)images_bmp_color_tail_right_81CD4B_bmp);
				break;
			}

			// On efface l'ancienne queue avec un carré vert
			if (!appleEaten && (snakeHeadPosition[0] != oldTailPosition[0] || snakeHeadPosition[1] != oldTailPosition[1]))
			{
				// On efface l'ancienne queue si:
				//    - le snake a avancé et n'a pas mangé de pomme
				// et
				//    - la tête n'est pas à la position de l'ancienne queue (sinon on efface la tête)
				BSP_LCD_SetTextColor((uint32_t)0xFF81CD4B); // 0xFF81CD4B
				BSP_LCD_FillRect(oldTailPosition[0] * 32, oldTailPosition[1] * 32, 32, 32);
			}

			// On affiche les pommes
			for (int i = 0; i < NB_APPLES; i++)
				if (applePosition[i][0] != -1)
					BSP_LCD_DrawBitmap(applePosition[i][0] * 32, applePosition[i][1] * 32, (uint8_t *)images_bmp_color_apple_81CD4B_bmp);
			xSemaphoreGive(displayMutexHandle);

			if (gameOver)
				lastMove = 0;
		}

		osDelay(90);
	}
	/* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartManageBodyParts */
/**
 * @brief Function implementing the manageBodyParts thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartManageBodyParts */
void StartManageBodyParts(void const *argument)
{
	/* USER CODE BEGIN StartManageBodyParts */
	TickType_t xLastWakeTime = xTaskGetTickCount();
	TickType_t delay = pdMS_TO_TICKS(1000. / initSpeed);

	/* Infinite loop */
	for (;;)
	{
		delay = pdMS_TO_TICKS(1000. / speed);

		if (!gameOver && !gamePaused && gameStarted)
		{
			// Le jeu est en cours

			// On sauvegarde la position de la queue et de la tête
			oldTailPosition[0] = snakeTailPosition[0];
			oldTailPosition[1] = snakeTailPosition[1];

			uint8_t oldHeadPosition[2];
			oldHeadPosition[0] = snakeHeadPosition[0];
			oldHeadPosition[1] = snakeHeadPosition[1];

			// On met a jour la position de la tête en fonction de la direction
			switch (direction)
			{
			case Up:
				headPart = HeadTop;
				snakeHeadPosition[1]--;
				break;
			case Down:
				headPart = HeadBottom;
				snakeHeadPosition[1]++;
				break;
			case Left:
				headPart = HeadLeft;
				snakeHeadPosition[0]--;
				break;
			case Right:
				headPart = HeadRight;
				snakeHeadPosition[0]++;
				break;
			}

			// on vérifie si on est mort
			if (snakeHeadPosition[0] >= GRID_SIZE_X || snakeHeadPosition[1] >= GRID_SIZE_Y || snakeHeadPosition[0] < 0 || snakeHeadPosition[1] < 0)
			{
				// On a touché un mur
				gameOver = 1;
			}
			else
			{
				for (int i = 0; i < snakeSize; i++)
				{
					if (snakeHeadPosition[0] == snakeBodyPosition[i][0] && snakeHeadPosition[1] == snakeBodyPosition[i][1])
					{
						// On a touché notre corps
						gameOver = 1;
					}
				}
			}
			// Note: Inutile de vérifier si on a touché la queue. Celle ci n'a pas encore avancé.

			// On vérifie si on a mangé la pomme avant de bouger le corps et la queue
			appleEaten = 0;
			for (int i = 0; i < NB_APPLES; i++)
			{
				if (snakeHeadPosition[0] == applePosition[i][0] && snakeHeadPosition[1] == applePosition[i][1])
				{
					snakeSize++;
					appleEaten = 1;

					// On augmente la vitesse de 1Hz a chaque palier
					for (int i = 0; i < NB_PALIERS; i++)
					{
						if (snakeSize == palierIncreaseSpeed[i])
						{
							speed++;
							break;
						}
					}

					// On ajoute un bodyPart à la queue
					for (int i = snakeSize - 1; i > 0; i--)
					{
						snakeBodyParts[i] = snakeBodyParts[i - 1];
						snakeBodyPosition[i][0] = snakeBodyPosition[i - 1][0];
						snakeBodyPosition[i][1] = snakeBodyPosition[i - 1][1];
					}

					// On met a jour le premier bodyPart
					uint8_t backX;
					uint8_t backY;

					if (snakeSize > 1)
					{
						backX = snakeBodyPosition[0][0];
						backY = snakeBodyPosition[0][1];
					}
					else
					{
						backX = snakeTailPosition[0];
						backY = snakeTailPosition[1];
					}

					// Trouve le type de bodyPart à mettre
					snakeBodyParts[0] = whatBodyPart(snakeHeadPosition[0], snakeHeadPosition[1], backX, backY, oldHeadPosition[0], oldHeadPosition[1]);
					snakeBodyPosition[0][0] = oldHeadPosition[0];
					snakeBodyPosition[0][1] = oldHeadPosition[1];

					// Si on a de la place pour une nouvelle pomme
					uint8_t NBFreeCells = GRID_SIZE_X * GRID_SIZE_Y - snakeSize - 1;
					if (NBFreeCells > NB_APPLES)
					{
						// On génère une nouvelle pomme à une position aléatoire qui n'est pas sur le snake ou une autre pomme
						do
						{
							applePosition[i][0] = rand() % GRID_SIZE_X;
							applePosition[i][1] = rand() % GRID_SIZE_Y;
						} while (isSnakePosition(applePosition[i][0], applePosition[i][1]) || isApplePosition(applePosition[i][0], applePosition[i][1], i));
					}
					else
					{
						applePosition[i][0] = -1;
						applePosition[i][1] = -1;
					}
				}
			}

			// On n'a pas mangé de pomme on avance le corps et la queue sans ajouter de bodyPart
			if (!appleEaten)
			{
				// on met a jour la position de la queue
				if (snakeSize > 0)
				{
					snakeTailPosition[0] = snakeBodyPosition[snakeSize - 1][0];
					snakeTailPosition[1] = snakeBodyPosition[snakeSize - 1][1];
				}
				else
				{
					snakeTailPosition[0] = oldHeadPosition[0];
					snakeTailPosition[1] = oldHeadPosition[1];
				}

				// On avance le corps
				for (int i = snakeSize - 1; i > 0; i--)
				{
					snakeBodyParts[i] = snakeBodyParts[i - 1];
					snakeBodyPosition[i][0] = snakeBodyPosition[i - 1][0];
					snakeBodyPosition[i][1] = snakeBodyPosition[i - 1][1];
				}

				// On met a jour le corps
				if (snakeSize > 0)
				{
					// On trouve le type de bodyPart à mettre juste derrière la tête
					snakeBodyParts[0] = whatBodyPart(snakeHeadPosition[0], snakeHeadPosition[1], snakeBodyPosition[0][0], snakeBodyPosition[0][1], oldHeadPosition[0], oldHeadPosition[1]);
					snakeBodyPosition[0][0] = oldHeadPosition[0];
					snakeBodyPosition[0][1] = oldHeadPosition[1];

					// On met a jour le l'orientation de la queue
					switch (snakeBodyParts[snakeSize - 1])
					{
					case BottomLeft:
						// TailTop ou TailRight
						if (snakeBodyPosition[snakeSize - 1][0] == snakeTailPosition[0])
						{
							tailPart = TailTop;
						}
						else
						{
							tailPart = TailRight;
						}
						break;
					case BottomRight:
						// TailBottom ou TailLeft
						if (snakeBodyPosition[snakeSize - 1][0] == snakeTailPosition[0])
						{
							tailPart = TailTop;
						}
						else
						{
							tailPart = TailLeft;
						}
						break;
					case BottomTop:
						// TailTop ou TailBottom
						if (snakeBodyPosition[snakeSize - 1][1] < snakeTailPosition[1])
						{
							tailPart = TailTop;
						}
						else
						{
							tailPart = TailBottom;
						}
						break;
					case LeftRight:
						// TailRight ou TailLeft
						if (snakeBodyPosition[snakeSize - 1][0] < snakeTailPosition[0])
						{
							tailPart = TailLeft;
						}
						else
						{
							tailPart = TailRight;
						}
						break;
					case LeftTop:
						// TailRight ou TailBottom
						if (snakeBodyPosition[snakeSize - 1][1] == snakeTailPosition[1])
						{
							tailPart = TailRight;
						}
						else
						{
							tailPart = TailBottom;
						}
						break;
					case RightTop:
						// TailLeft ou TailBottom
						if (snakeBodyPosition[snakeSize - 1][1] == snakeTailPosition[1])
						{
							tailPart = TailLeft;
						}
						else
						{
							tailPart = TailBottom;
						}
						break;
					}
				}
				else
				{
					switch (headPart)
					{
					case HeadTop:
						tailPart = TailTop;
						break;
					case HeadBottom:
						tailPart = TailBottom;
						break;
					case HeadLeft:
						tailPart = TailLeft;
						break;
					case HeadRight:
						tailPart = TailRight;
						break;
					}
				}
			}
		}

		vTaskDelayUntil(&xLastWakeTime, delay);
	}
	/* USER CODE END StartManageBodyParts */
}

/* USER CODE BEGIN Header_StartTsHandler */
/**
 * @brief Function implementing the tsHandlerTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTsHandler */
void StartTsHandler(void const *argument)
{
	/* USER CODE BEGIN StartTsHandler */
	/* Infinite loop */
	for (;;)
	{
		if (!gameStarted)
			rand();

		BSP_TS_GetState(&TS_State);

		if (!TS_State.touchDetected && screenPressed)
		{
			if (gameOver)
			{
				BSP_LCD_Clear((uint32_t)0xFF81CD4B);
				restartGame();
			}
			else if (gamePaused)
			{
				xSemaphoreTake(displayMutexHandle, portMAX_DELAY);
				BSP_LCD_Clear((uint32_t)0xFF81CD4B);
				xSemaphoreGive(displayMutexHandle);
				gamePaused = 0;
			}
			else if (!gameStarted)
			{
				BSP_LCD_Clear((uint32_t)0xFF81CD4B);
				restartGame();
				gameStarted = 1;
			}
			else
			{
				gamePaused = 1;
			}
		}

		if (TS_State.touchDetected)
		{
			screenPressed = 1;
		}
		else
		{
			screenPressed = 0;
		}

		osDelay(10);
	}
	/* USER CODE END StartTsHandler */
}

/* USER CODE BEGIN Header_StartPlaySongTask */
/**
 * @brief Function implementing the playSongTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPlaySongTask */
void StartPlaySongTask(void const *argument) {
    char i;
    uint32_t bytesread;
    
    /* Infinite loop */
    for (;;) {
        // On attend le signal de réveil
        xQueueReceive(WakeUpHandle, &i, portMAX_DELAY);
        
        // On arrive ici si la musique est terminée
        if (blockPointer++ == numberOfBlocks - 1) {
            loadWav();
        }
        
        // On détermine la position du buffer à remplir
        uint8_t *bufferPosition = (i == 0) ? AUDIO_BUFFER_OUT : (AUDIO_BUFFER_OUT + AUDIO_BLOCK_SIZE);
        
        // On lit le bloc de données
        f_read(&SDFile, bufferPosition, AUDIO_BLOCK_SIZE, (void *)&bytesread);
    }
}


/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6)
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
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
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
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
