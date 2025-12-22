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

float Target = 12960.0f; // 360° * 19.2 (度数 * 减速比)| 360 * 36.0(度数 * 减速比)
static uint32_t lastServoTime = 0;
static uint16_t g_rxDataCnt = 0; // 接收到的数据数量

static StackType_t g_pxChangeTarget[32];
static StaticTask_t g_TCBChangeTarget;
static TaskHandle_t g_HandleChangeTarget;

/// @brief 改变电机目标数值
/// @param arg 任务参数（未使用）
/// @return 无
void pxChangeTarget(void *arg)
{
  (void)arg; // 消除未使用参数警告

  while (1)
  {
    vTaskDelay(4000);
    if (Target < 25000.0f)
    {
      Target += 5000.0f;
    }
    else
    {
      Target = 0.0f;
      vTaskDelay(8000);
    }

    // RmMotorRemoveBias(RM_3508_GRIPPER, Target);
  }
}

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
  // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  // 改变电机目标数值的一个乐色任务
  // g_HandleChangeTarget = xTaskCreateStatic(pxChangeTarget, "ChangeTarget", 32, NULL, osPriorityNormal, g_pxChangeTarget, &g_TCBChangeTarget);

  // 飞镖任务初始化
  TaskInitFunc();

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

  // 定义数据包接收变量（无需手动初始化，UART_GetServoPacket会完全覆盖）
  //  ServoPacket_t HxFb;
  //  DartPacket_t UpcFb;

  // 舵机初始化（只调用一次，在循环外）
  //  ServoInit();
  //  vTaskDelay(100); // 等待初始化完成
  //  uint8_t servo_ids[3] = {0x01, 0x02, 0x03};
  //  uint16_t angles[3] = {0, 0, 0};

  /* Infinite loop */
  for (;;)
  {
    // ===== 舵机数据读取示例 =====
    // 检查是否有完整的数据包
    // 读取数据包
    // if (UART_GetServoPacket(BSP_UART3, &HxFb))
    // {
    //   // printf("ID: %d, Cmd: 0x%02X\n", HxFb.id, HxFb.cmd);

    //   g_rxDataCnt++;
    //   ServoControlPos(0x01, 300, 1000);
    //   vTaskDelay(1200);
    //   ServoControlPos(0x02, 300, 1000);
    //   vTaskDelay(1200);
    //   ServoControlPos(0x03, 300, 1000);
    //   vTaskDelay(1200);
    //   ServoControlMulti(3, servo_ids, angles, 1000);

    //   // 清除数据包标志，准备接收下一个
    //   UART_ClearPacket(BSP_UART3);
    // }

    // 舵机控制示例（每1秒执行一次）
    // if (HAL_GetTick() - lastServoTime >= 1000)
    // {
    //   lastServoTime = HAL_GetTick();
    //   // ServoControlPos(1, 500, 500); // 控制1号舵机
    // }

    // RmMotorPID_Calc(RM_2006_TRIGGER, Target);
    // RmMotorSendCfg(RM_2006_TRIGGER, 1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
