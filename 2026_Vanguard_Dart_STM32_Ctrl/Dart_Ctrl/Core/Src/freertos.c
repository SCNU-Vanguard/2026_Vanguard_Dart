/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "UserTask.h"
#include <stdio.h>
#include <string.h>

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

static StaticSemaphore_t g_xRmBufferMutexBuffer; /* 保存 g_xRmBufferMutexBuffer。 */
SemaphoreHandle_t g_xRmBufferMutexHandle; /* 保存 g_xRmBufferMutexHandle。 */
float g_speed_right_test = 0.0f; /* 初始化 g_speed_right_test。 */
float g_speed_left_test = 0.0f; /* 初始化 g_speed_left_test。 */
float g_right_target = 0.0f; /* 初始化 g_right_target。 */
float g_left_target = 0.0f; /* 初始化 g_left_target。 */
float g_left_out_output = 0.0f; /* 初始化 g_left_out_output。 */
float g_right_out_output = 0.0f; /* 初始化 g_right_out_output。 */
float g_right_inner_test = 0.0f; /* 初始化 g_right_inner_test。 */
float g_left_inner_test = 0.0f; /* 初始化 g_left_inner_test。 */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  g_xRmBufferMutexHandle = xSemaphoreCreateMutexStatic(&g_xRmBufferMutexBuffer); /* 更新 g_xRmBufferMutexHandle。 */
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  // 飞镖任务初始化
  TaskInitFunc(); /* 调用 TaskInitFunc。 */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  // 舵机初始化（只调用一次，在循环外）
  // ServoInit();
  // vTaskDelay(100); // 等待初始化完成
  // uint8_t servo_ids[3] = {0x01, 0x02, 0x03};
  // uint16_t angles[3] = {0, 0, 0};

  // ServoControlPos(0x01, 300, 1000);
  // vTaskDelay(4000);
  // ServoControlPos(0x02, 300, 1000);
  // vTaskDelay(1200);
  // ServoControlPos(0x03, 300, 1000);
  // vTaskDelay(1200);
  // ServoControlMulti(3, servo_ids, angles, 1000);

  /* Infinite loop */
  for (;;) /* 遍历当前数据集合。 */
  {
#if 0
    HAL_GPIO_TogglePin(Green_GPIO_Port, Green_Pin); /* 调用 HAL_GPIO_TogglePin。 */
    vTaskDelay(250); /* 调用 vTaskDelay。 */
#else
    g_speed_right_test = MotorManager.MotorList[2].motor_data.solved_data[3]; /* 更新 g_speed_right_test。 */
    g_speed_left_test = MotorManager.MotorList[3].motor_data.solved_data[3]; /* 更新 g_speed_left_test。 */
    g_right_target = MotorManager.MotorList[2].cascade_pid.outer.target; /* 更新 g_right_target。 */
    g_left_target = MotorManager.MotorList[3].cascade_pid.outer.target; /* 更新 g_left_target。 */
    g_left_inner_test = MotorManager.MotorList[3].motor_data.solved_data[4]; /* 更新 g_left_inner_test。 */
    g_right_inner_test = MotorManager.MotorList[2].motor_data.solved_data[4]; /* 更新 g_right_inner_test。 */
    g_left_out_output = MotorManager.MotorList[3].cascade_pid.outer.output; /* 更新 g_left_out_output。 */
    g_right_out_output = MotorManager.MotorList[2].cascade_pid.outer.output; /* 更新 g_right_out_output。 */
    vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
#endif
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
