/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery_lcd.h"
#include "adc.h"
#include <stdio.h>

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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
const uint8_t gridSizeX = 15;
const uint8_t gridSizeY = 8;

uint32_t joystick_v;
uint32_t joystick_h;
extern ADC_ChannelConfTypeDef sConfig;

enum Direction {
  Up,
  Down,
  Left,
  Right
};
enum Direction direction = Up;

enum HeadPart {
  HeadTop,
  HeadBottom,
  HeadLeft,
  HeadRight
};
enum HeadPart headPart = HeadTop;

enum BodyPart {
  BottomLeft,
  BottomRight,
  BottomTop,
  LeftRight,
  LeftTop,
  RightTop
};
enum BodyPart snakeBodyParts[15 * 8];

enum TailPart {
  TailBottom,
  TailTop,
  TailLeft,
  TailRight
};
enum TailPart tailPart = TailTop;

uint8_t snakeSize = 0;

uint8_t snakeHeadPosition[2] = {7, 6};
uint8_t snakeBodyPosition[15 * 8][2] = {};
uint8_t snakeTailPosition[2] = {7, 7};
uint8_t oldTailPosition[2];
uint8_t applePosition[2] = {7, 3};
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId displayTaskHandle;
osThreadId joystickTaskHandle;
osThreadId manageBodyPartsHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartDisplayTask(void const * argument);
void StartJoystickTask(void const * argument);
void StartManageBodyParts(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of displayTask */
  osThreadDef(displayTask, StartDisplayTask, osPriorityNormal, 0, 1024);
  displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);

  /* definition and creation of joystickTask */
  osThreadDef(joystickTask, StartJoystickTask, osPriorityAboveNormal, 0, 128);
  joystickTaskHandle = osThreadCreate(osThread(joystickTask), NULL);

  /* definition and creation of manageBodyParts */
  osThreadDef(manageBodyParts, StartManageBodyParts, osPriorityAboveNormal, 0, 128);
  manageBodyPartsHandle = osThreadCreate(osThread(manageBodyParts), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const * argument)
{
  /* USER CODE BEGIN StartDisplayTask */
  /* Infinite loop */
  for(;;)
  {
    BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
    BSP_LCD_DrawHLine(0, 8*32, BSP_LCD_GetXSize());
    BSP_LCD_DrawHLine(0, 8*32 + 1, BSP_LCD_GetXSize());

    char directionText[100];
    // on affiche la direction
    switch (direction) {
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
    BSP_LCD_DisplayStringAt(0, 8*32 + 2, (uint8_t *)directionText, LEFT_MODE);

    // On affiche le score a la suite de la direction
    char scoreText[100];
    sprintf(scoreText, (char *)"Score: %d", snakeSize);
    BSP_LCD_DisplayStringAt(200, 8*32 + 2, (uint8_t *)scoreText, LEFT_MODE);


    // On affiche la tête du snake
    switch (headPart) {
      case HeadBottom:
        BSP_LCD_DrawBitmap(snakeHeadPosition[0]*32, snakeHeadPosition[1]*32, (uint8_t*)images_bmp_color_head_bottom_81CD4B_bmp);
        break;
      case HeadTop:
        BSP_LCD_DrawBitmap(snakeHeadPosition[0]*32, snakeHeadPosition[1]*32, (uint8_t*)images_bmp_color_head_top_81CD4B_bmp);
        break;
      case HeadLeft:
        BSP_LCD_DrawBitmap(snakeHeadPosition[0]*32, snakeHeadPosition[1]*32, (uint8_t*)images_bmp_color_head_left_81CD4B_bmp);
        break;
      case HeadRight:
        BSP_LCD_DrawBitmap(snakeHeadPosition[0]*32, snakeHeadPosition[1]*32, (uint8_t*)images_bmp_color_head_right_81CD4B_bmp);
        break;
    }


    // On affiche le corps du snake
    for (int i = 1; i <= snakeSize; i++) {
      switch (snakeBodyParts[i]) {
        case BottomLeft:
          BSP_LCD_DrawBitmap(snakeBodyPosition[i][0]*32, snakeBodyPosition[i][1]*32, (uint8_t*)images_bmp_color_bottom_left_81CD4B_bmp);
          break;
        case BottomRight:
          BSP_LCD_DrawBitmap(snakeBodyPosition[i][0]*32, snakeBodyPosition[i][1]*32, (uint8_t*)images_bmp_color_bottom_right_81CD4B_bmp);
          break;
        case BottomTop:
          BSP_LCD_DrawBitmap(snakeBodyPosition[i][0]*32, snakeBodyPosition[i][1]*32, (uint8_t*)images_bmp_color_bottom_top_81CD4B_bmp);
          break;
        case LeftRight:
          BSP_LCD_DrawBitmap(snakeBodyPosition[i][0]*32, snakeBodyPosition[i][1]*32, (uint8_t*)images_bmp_color_left_right_81CD4B_bmp);
          break;
        case LeftTop:
          BSP_LCD_DrawBitmap(snakeBodyPosition[i][0]*32, snakeBodyPosition[i][1]*32, (uint8_t*)images_bmp_color_left_top_81CD4B_bmp);
          break;
        case RightTop:
          BSP_LCD_DrawBitmap(snakeBodyPosition[i][0]*32, snakeBodyPosition[i][1]*32, (uint8_t*)images_bmp_color_right_top_81CD4B_bmp);
          break;
      }
    }

    // On affiche la queue du snake
    switch (tailPart) {
      case TailBottom:
        BSP_LCD_DrawBitmap(snakeTailPosition[0]*32, snakeTailPosition[1]*32, (uint8_t*)images_bmp_color_tail_bottom_81CD4B_bmp);
        break;
      case TailTop:
        BSP_LCD_DrawBitmap(snakeTailPosition[0]*32, snakeTailPosition[1]*32, (uint8_t*)images_bmp_color_tail_top_81CD4B_bmp);
        break;
      case TailLeft:
        BSP_LCD_DrawBitmap(snakeTailPosition[0]*32, snakeTailPosition[1]*32, (uint8_t*)images_bmp_color_tail_left_81CD4B_bmp);
        break;
      case TailRight:
        BSP_LCD_DrawBitmap(snakeTailPosition[0]*32, snakeTailPosition[1]*32, (uint8_t*)images_bmp_color_tail_right_81CD4B_bmp);
        break;
    }

    // On efface l'ancienne queue avec un carré vert
    BSP_LCD_SetTextColor((uint32_t)0xFF81CD4B);
    BSP_LCD_FillRect(oldTailPosition[0]*32, oldTailPosition[1]*32, 32, 32);

    // On affiche la pomme
    BSP_LCD_DrawBitmap(applePosition[0]*32, applePosition[1]*32, (uint8_t*)images_bmp_color_apple_81CD4B_bmp);


    osDelay(100);
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartJoystickTask */
/**
* @brief Function implementing the joystickTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartJoystickTask */
void StartJoystickTask(void const * argument)
{
  /* USER CODE BEGIN StartJoystickTask */
  /* Infinite loop */
  for(;;)
  {
    sConfig.Channel = ADC_CHANNEL_8;
	HAL_ADC_ConfigChannel(&hadc3, &sConfig);
	HAL_ADC_Start(&hadc3);
	while(HAL_ADC_PollForConversion(&hadc3, 100)!=HAL_OK);
	joystick_v = HAL_ADC_GetValue(&hadc3);

	HAL_ADC_Start(&hadc1);
	while(HAL_ADC_PollForConversion(&hadc1, 100)!=HAL_OK);
	joystick_h = HAL_ADC_GetValue(&hadc1);

    // max range of joystick is 0 to 4095

    if (joystick_v < 1000 && direction != Up) {
      direction = Down;
    } else if (joystick_v > 3000 && direction != Down) {
      direction = Up;
    } else if (joystick_h < 1000 && direction != Left) {
      direction = Right;
    } else if (joystick_h > 3000 && direction != Right) {
      direction = Left;
    }
    osDelay(1);
  }
  /* USER CODE END StartJoystickTask */
}

/* USER CODE BEGIN Header_StartManageBodyParts */
/**
* @brief Function implementing the manageBodyParts thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartManageBodyParts */
void StartManageBodyParts(void const * argument)
{
  /* USER CODE BEGIN StartManageBodyParts */
  /* Infinite loop */
  for(;;)
  {
    // remplir le tableau snakeBodyParts en fonction de snakePosition
    // on doit changer la tete et la bodyPart juste apres la tete, le reste ne change pas
    // on doit aussi faire avancer la queue
    // on avance le snake

    oldTailPosition[0] = snakeTailPosition[0];
    oldTailPosition[1] = snakeTailPosition[1];

    uint8_t oldHeadPosition[2] = {snakeHeadPosition[0], snakeHeadPosition[1]};


    // la tête
    switch (direction) {
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
    
    // on vérifie si on a mangé la pomme avant de bouger le corps et la queue
    if (snakeHeadPosition[0] == applePosition[0] && snakeHeadPosition[1] == applePosition[1]) {
      snakeSize++;

      // il ne faut pas que la pomme apparaisse sur le snake
      applePosition[0] = rand() % gridSizeX;
      applePosition[1] = rand() % gridSizeY;

      // on ajoute un bodyPart juste derrière la tête
      snakeBodyParts[0] = BottomTop;
      snakeBodyPosition[0][0] = oldHeadPosition[0];
      snakeBodyPosition[0][1] = oldHeadPosition[1];
    }
    else {
      // On avance le corps
      for (int i = snakeSize - 1; i > 0; i--) {
        snakeBodyParts[i] = snakeBodyParts[i - 1];
        // on met a jour snakeBodyPosition
        snakeBodyPosition[i][0] = snakeBodyPosition[i - 1][0];
        snakeBodyPosition[i][1] = snakeBodyPosition[i - 1][1];
      }

      // On avance la queue
      if (snakeSize > 0) {
        switch (snakeBodyParts[snakeSize - 1]) {
          case BottomLeft:
            // TailBottom ou TailLeft
            break;
          case BottomRight:
            // TailBottom ou TailRight
            break;
          case BottomTop:
            // TailBottom ou TailTop
            break;
          case LeftRight:
            // TailLeft ou TailRight
            break;
          case LeftTop:
            // TailLeft ou TailTop
            break;
          case RightTop:
            // TailRight ou TailTop
            break;
        }
      }
      else {
        switch (headPart) {
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
      
        snakeTailPosition[0] = oldHeadPosition[0];
        snakeTailPosition[1] = oldHeadPosition[1];
      }
    }





    osDelay(300);
  }
  /* USER CODE END StartManageBodyParts */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

