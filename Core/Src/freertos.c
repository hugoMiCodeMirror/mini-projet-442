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
#include "stm32746g_discovery_ts.h"
#include "adc.h"
#include <stdio.h>
#include <stdlib.h>

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
static TS_StateTypeDef  TS_State;
uint32_t screenPressed = 0;
uint32_t screenReleased = 0;

const uint8_t gridSizeX = 15;
const uint8_t gridSizeY = 8;
const uint8_t NBApple = 4;
const uint8_t speed = 250; // période de rafraîchissement en ms

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
uint32_t appleEaten = 0;
uint32_t gameOver = 0;
uint32_t gamePaused = 0;
uint32_t gameStarted = 0;

uint8_t snakeHeadPosition[2] = {7, 6};
uint8_t snakeBodyPosition[15 * 8][2] = {};
uint8_t snakeTailPosition[2] = {7, 7};
uint8_t oldTailPosition[2];
uint8_t applePosition[4][2];
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId displayTaskHandle;
osThreadId joystickTaskHandle;
osThreadId manageBodyPartsHandle;
osThreadId tsHandlerTaskHandle;
osMutexId displayMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
enum BodyPart whatBodyPart(uint8_t frontX, uint8_t frontY, uint8_t backX, uint8_t backY, uint8_t missingX, uint8_t missingY);
uint8_t isSnakePosition(uint8_t x, uint8_t y);
uint8_t isApplePosition(uint8_t x, uint8_t y, uint8_t appleIndex);
void restartGame();
void displayGameStatus();
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartDisplayTask(void const * argument);
void StartJoystickTask(void const * argument);
void StartManageBodyParts(void const * argument);
void StartTsHandlerTask(void const * argument);

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

  /* definition and creation of tsHandlerTask */
  osThreadDef(tsHandlerTask, StartTsHandlerTask, osPriorityHigh, 0, 512);
  tsHandlerTaskHandle = osThreadCreate(osThread(tsHandlerTask), NULL);

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
    if (gameOver || gamePaused || !gameStarted) {
      xSemaphoreTake(displayMutexHandle, portMAX_DELAY);
      displayGameStatus();
      xSemaphoreGive(displayMutexHandle);
    }
    else {
      // On convertit la direction en texte
      char directionText[100];
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
      // On convertit le score en texte
      char scoreText[100];
      sprintf(scoreText, (char *)"Score: %d", snakeSize);

      xSemaphoreTake(displayMutexHandle, portMAX_DELAY);
      BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
      BSP_LCD_DrawHLine(0, 8*32, BSP_LCD_GetXSize());
      BSP_LCD_DrawHLine(0, 8*32 + 1, BSP_LCD_GetXSize());

      // On affiche la direction et le score
      BSP_LCD_DisplayStringAt(0, 8*32 + 2, (uint8_t *)directionText, LEFT_MODE);
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
      for (int i = 0; i < snakeSize; i++) {
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
      if (!appleEaten && (snakeHeadPosition[0] != oldTailPosition[0] || snakeHeadPosition[1] != oldTailPosition[1])) {
        // On efface l'ancienne queue si:
        //    - le snake a avancé (la queue n'est pas restée à la même position)
        // ou
        //    - la tête n'est pas à la position de l'ancienne queue (le snake suit sa queue)
        BSP_LCD_SetTextColor((uint32_t)0xFF81CD4B); // 0xFF81CD4B
        BSP_LCD_FillRect(oldTailPosition[0]*32, oldTailPosition[1]*32, 32, 32);
      }

      // On affiche les pommes
      for (int i = 0; i < NBApple; i++) {
        BSP_LCD_DrawBitmap(applePosition[i][0]*32, applePosition[i][1]*32, (uint8_t*)images_bmp_color_apple_81CD4B_bmp);
      }
      xSemaphoreGive(displayMutexHandle);
    }

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

    if (joystick_v < 1000 && headPart != HeadTop) {
      direction = Down;
    } else if (joystick_v > 3000 && headPart != HeadBottom) {
      direction = Up;
    } else if (joystick_h < 1000 && headPart != HeadLeft) {
      direction = Right;
    } else if (joystick_h > 3000 && headPart != HeadRight) {
      direction = Left;
    }
    osDelay(10);
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
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t delay = pdMS_TO_TICKS(speed);

  /* Infinite loop */
  for(;;)
  {
    if (!gameOver && !gamePaused && gameStarted) {
      // Le jeu est en cours

      // On sauvegarde la position de la queue et de la tête
      oldTailPosition[0] = snakeTailPosition[0];
      oldTailPosition[1] = snakeTailPosition[1];

      uint8_t oldHeadPosition[2];
      oldHeadPosition[0] = snakeHeadPosition[0];
      oldHeadPosition[1] = snakeHeadPosition[1];


      // On met a jour la position de la tête en fonction de la direction
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

      // on vérifie si on est mort
      if (snakeHeadPosition[0] >= gridSizeX || snakeHeadPosition[1] >= gridSizeY || snakeHeadPosition[0] < 0 || snakeHeadPosition[1] < 0) {
        // On a touché un mur
        gameOver = 1;

        // displayGameStatus();
      }
      else  {
        for (int i = 0; i < snakeSize; i++) {
          if (snakeHeadPosition[0] == snakeBodyPosition[i][0] && snakeHeadPosition[1] == snakeBodyPosition[i][1]) {
            // On a touché notre corps
            gameOver = 1;

            // displayGameStatus();
          }
        }
      }
      /*
        Note: Inutile de vérifier si on a touché la queue. Celle ci n'a pas encore avancé.
      */


      
      // On vérifie si on a mangé la pomme avant de bouger le corps et la queue
      appleEaten = 0;
      for (int i = 0; i < NBApple; i++) {
        if (snakeHeadPosition[0] == applePosition[i][0] && snakeHeadPosition[1] == applePosition[i][1]) {
          snakeSize++;
          appleEaten = 1;

          // On ajoute un bodyPart à la queue
          for (int i = snakeSize - 1; i > 0; i--) {
            snakeBodyParts[i] = snakeBodyParts[i - 1];
            snakeBodyPosition[i][0] = snakeBodyPosition[i - 1][0];
            snakeBodyPosition[i][1] = snakeBodyPosition[i - 1][1];
          }

          // On met a jour le premier bodyPart
          uint8_t backX;
          uint8_t backY;

          if (snakeSize > 1) {
            backX = snakeBodyPosition[0][0];
            backY = snakeBodyPosition[0][1];
          }
          else {
            backX = snakeTailPosition[0];
            backY = snakeTailPosition[1];
          }

          // Trouve le type de bodyPart à mettre
          snakeBodyParts[0] = whatBodyPart(snakeHeadPosition[0], snakeHeadPosition[1], backX, backY, oldHeadPosition[0], oldHeadPosition[1]);
          snakeBodyPosition[0][0] = oldHeadPosition[0];
          snakeBodyPosition[0][1] = oldHeadPosition[1];

          // On génère une nouvelle pomme à une position aléatoire qui n'est pas sur le snake ou une autre pomme
          // TODO: Si on arrive a la fin du jeu, on ne peut plus générer de pomme car toutes les cases sont prises
          do {
            applePosition[i][0] = rand() % gridSizeX;
            applePosition[i][1] = rand() % gridSizeY;
          } while (isSnakePosition(applePosition[i][0], applePosition[i][1]) || isApplePosition(applePosition[i][0], applePosition[i][1], i));
        }
      }


      // On n'a pas mangé de pomme on avance le corps et la queue sans ajouter de bodyPart
      if (!appleEaten) {
        // on met a jour la position de la queue
        if (snakeSize > 0) {
          snakeTailPosition[0] = snakeBodyPosition[snakeSize - 1][0];
          snakeTailPosition[1] = snakeBodyPosition[snakeSize - 1][1];
        }
        else {
          snakeTailPosition[0] = oldHeadPosition[0];
          snakeTailPosition[1] = oldHeadPosition[1];
        }


        // On avance le corps
        for (int i = snakeSize - 1; i > 0; i--) {
          snakeBodyParts[i] = snakeBodyParts[i - 1];
          snakeBodyPosition[i][0] = snakeBodyPosition[i - 1][0];
          snakeBodyPosition[i][1] = snakeBodyPosition[i - 1][1];
        }


        // On met a jour le corps
        if (snakeSize > 0) {
          // On trouve le type de bodyPart à mettre juste derrière la tête
          snakeBodyParts[0] = whatBodyPart(snakeHeadPosition[0], snakeHeadPosition[1], snakeBodyPosition[0][0], snakeBodyPosition[0][1], oldHeadPosition[0], oldHeadPosition[1]);
          snakeBodyPosition[0][0] = oldHeadPosition[0];
          snakeBodyPosition[0][1] = oldHeadPosition[1];

          // On met a jour le l'orientation de la queue
          switch (snakeBodyParts[snakeSize - 1]) {
            case BottomLeft:
              // TailTop ou TailRight
              if (snakeBodyPosition[snakeSize - 1][0] == snakeTailPosition[0]) {
                tailPart = TailTop;
              }
              else {
                tailPart = TailRight;
              }
              break;
            case BottomRight:
              // TailBottom ou TailLeft
              if (snakeBodyPosition[snakeSize - 1][0] == snakeTailPosition[0]) {
                tailPart = TailTop;
              }
              else {
                tailPart = TailLeft;
              }
              break;
            case BottomTop:
              // TailTop ou TailBottom
              if (snakeBodyPosition[snakeSize - 1][1] < snakeTailPosition[1]) {
                tailPart = TailTop;
              }
              else {
                tailPart = TailBottom;
              }
              break;
            case LeftRight:
              // TailRight ou TailLeft
              if (snakeBodyPosition[snakeSize - 1][0] < snakeTailPosition[0]) {
                tailPart = TailLeft;
              }
              else {
                tailPart = TailRight;
              }
              break;
            case LeftTop:
              // TailRight ou TailBottom
              if (snakeBodyPosition[snakeSize - 1][1] == snakeTailPosition[1]) {
                tailPart = TailRight;
              }
              else {
                tailPart = TailBottom;
              }
              break;
            case RightTop:
              // TailLeft ou TailBottom
              if (snakeBodyPosition[snakeSize - 1][1] == snakeTailPosition[1]) {
                tailPart = TailLeft;
              }
              else {
                tailPart = TailBottom;
              }
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
        }
      }
    }

    vTaskDelayUntil(&xLastWakeTime, delay);
  }
  /* USER CODE END StartManageBodyParts */
}

/* USER CODE BEGIN Header_StartTsHandlerTask */
/**
* @brief Function implementing the tsHandlerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTsHandlerTask */
void StartTsHandlerTask(void const * argument)
{
  /* USER CODE BEGIN StartTsHandlerTask */
  /* Infinite loop */
  for(;;)
  {
    if (!gameStarted)
      rand();
    

    BSP_TS_GetState(&TS_State);

    if (!TS_State.touchDetected && screenPressed) {
      if (gameOver) {
        BSP_LCD_Clear((uint32_t)0xFF81CD4B);
        gameOver = 0;
        restartGame();
      }
      else if (gamePaused) {
        xSemaphoreTake(displayMutexHandle, portMAX_DELAY);
        BSP_LCD_Clear((uint32_t)0xFF81CD4B);
        xSemaphoreGive(displayMutexHandle);
        gamePaused = 0;
      }
      else if (!gameStarted) {
        BSP_LCD_Clear((uint32_t)0xFF81CD4B);
        restartGame();
        gameStarted = 1;
      }
      else {
        gamePaused = 1;
      }
    }

    if (TS_State.touchDetected) {
      screenPressed = 1;
    }
    else {
      screenPressed = 0;
    }

    osDelay(10);
  }
  /* USER CODE END StartTsHandlerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
enum BodyPart whatBodyPart(uint8_t frontX, uint8_t frontY, uint8_t backX, uint8_t backY, uint8_t missingX, uint8_t missingY)
{
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
  if (frontX < backX && frontY > backY) {
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
  if (frontX < backX && frontY < backY) {
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
  if (frontX > backX && frontY < backY) {
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
  if (frontX > backX && frontY > backY) {
    if (missingX == frontX)
      return BottomLeft;
    else
      return RightTop;
  }

  return BottomTop; // should never happen
}


uint8_t isSnakePosition(uint8_t x, uint8_t y)
{
  if (x == snakeHeadPosition[0] && y == snakeHeadPosition[1])
    return 1;

  for (int i = 0; i < snakeSize; i++) {
    if (x == snakeBodyPosition[i][0] && y == snakeBodyPosition[i][1])
      return 1;
  }

  if (x == snakeTailPosition[0] && y == snakeTailPosition[1])
    return 1;

  return 0;
}

uint8_t isApplePosition(uint8_t x, uint8_t y, uint8_t appleIndex)
{
  for (int i = 0; i < NBApple; i++) {
    if (i == appleIndex)
      continue;

    if (x == applePosition[i][0] && y == applePosition[i][1])
      return 1;
  }

  return 0;
}

void restartGame()
{
  snakeSize = 0;
  snakeHeadPosition[0] = 7;
  snakeHeadPosition[1] = 6;
  snakeTailPosition[0] = 7;
  snakeTailPosition[1] = 7;
  direction = Up;
  headPart = HeadTop;
  tailPart = TailTop;

  for (int i = 0; i < NBApple; i++) {
    do {
      applePosition[i][0] = rand() % gridSizeX;
      applePosition[i][1] = rand() % gridSizeY;
    } while (isSnakePosition(applePosition[i][0], applePosition[i][1]) || isApplePosition(applePosition[i][0], applePosition[i][1], i));
  }

  BSP_LCD_Clear((uint32_t)0xFF81CD4B);
}

void displayGameStatus()
{
  if (gameOver) {
    BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
    BSP_LCD_SetFont(&Font24);
    BSP_LCD_DisplayStringAt(0, 100, (uint8_t *)"Game Over", CENTER_MODE);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAt(0, 130, (uint8_t *)"Touch the screen to restart", CENTER_MODE);
  }
  else if (gamePaused) {
    BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
    BSP_LCD_SetFont(&Font24);
    BSP_LCD_DisplayStringAt(0, 100, (uint8_t *)"Game Paused", CENTER_MODE);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAt(0, 130, (uint8_t *)"Touch the screen to resume", CENTER_MODE);
  }
  else if (!gameStarted) {
    BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
    BSP_LCD_SetFont(&Font24);
    BSP_LCD_DisplayStringAt(0, 100, (uint8_t *)"Snake Game", CENTER_MODE);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAt(0, 130, (uint8_t *)"Touch the screen to start", CENTER_MODE);
  }
}
/* USER CODE END Application */

