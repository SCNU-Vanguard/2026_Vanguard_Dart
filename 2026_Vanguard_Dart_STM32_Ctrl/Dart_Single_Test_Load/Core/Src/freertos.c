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
#include "semphr.h"
#include <arm_math.h>
#include "ia6b_task.h"
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
static StaticSemaphore_t g_xRmBufferMutexBuffer;
SemaphoreHandle_t g_xRmBufferMutexHandle;
// extern float sine_output;
// extern float trap_output;
extern float RmMotorAngleData;
extern float RmMotorSpeedData;

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
static void DefaultTask_Control3508(void);
static void DefaultTask_Control2006(void);

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
  g_xRmBufferMutexHandle = xSemaphoreCreateMutexStatic(&g_xRmBufferMutexBuffer);
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
  // TaskInitFunc();

  /* 正弦波生成 */
  // TaskHandle_t SineWaveTaskHandle = NULL;
  // xTaskCreate(SineWaveTask, "SinWaveOut", 64 * 4, NULL, osPriorityBelowNormal7, &SineWaveTaskHandle);

  /* 阶跃波生成 */
  // TaskHandle_t TrapWaveTaskHandle = NULL;
  // xTaskCreate(TrapWaveTask, "TrapWaveOut", 64 * 4, NULL, osPriorityBelowNormal7, &TrapWaveTaskHandle);

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
#if ENABLE_DEFAULTTASK_RC_DEBUG
  vTaskDelay(2000);
  IA6BTask_Init();

#endif

  /* Infinite loop */
  for (;;)
  {
#if ENABLE_DEFAULTTASK_RC_DEBUG
    IA6BTask_ProcessAndControl();
    DefaultTask_Control3508();
    DefaultTask_Control2006();
#endif
    vTaskDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void DefaultTask_Control3508(void)
{
#if ENABLE_RC_DEBUG_3508
  RmMotorAngleData = Motor_GetTotalAngle(RM_3508_GRIPPER);
  RmMotorSpeedData = Motor_GetSpeedRPM(RM_3508_GRIPPER);

#if LOAD3508_OUTPUT_MODE == LOAD3508_OUTPUT_CASCADE_POS
  if (RcLoad3508PosTargetInitialized)
  {
    RmMotorPID_Calc(RM_3508_GRIPPER, RmMotorTargetPosData);
  }
  else
  {
    RmMotorSendCfg(RM_3508_GRIPPER, 0);
  }
#else
  RmMotorSpeedPID_Calc(RM_3508_GRIPPER, RmMotorTargetSpeedData);
#endif
#endif
}

static void DefaultTask_Control2006(void)
{
#if ENABLE_RC_DEBUG_2006
  static uint8_t s_2006_limit_inited = 0U;
  static float s_2006_zero_pos_deg = 0.0f;

  float motor_2006_pos_deg_abs = Motor_GetTotalAngle(RM_2006_TRIGGER);
  float target_pos_rel_deg = RmMotor2006TargetPosData;
  float target_pos_abs_deg = 0.0f;

  if (s_2006_limit_inited == 0U)
  {
    s_2006_zero_pos_deg = motor_2006_pos_deg_abs;
    s_2006_limit_inited = 1U;
  }
  target_pos_abs_deg = s_2006_zero_pos_deg + target_pos_rel_deg;

  float motor_2006_speed_rpm = Motor_GetSpeedRPM(RM_2006_TRIGGER);
  RmMotor2006SpeedData = motor_2006_speed_rpm;
  RmMotorPID_Calc(RM_2006_TRIGGER, target_pos_abs_deg);

#if (ENABLE_RC_DEBUG_3508 == 0U)
  RmMotorSpeedData = motor_2006_speed_rpm;
  RmMotorAngleData = motor_2006_pos_deg_abs;
#endif
#endif
}

/* USER CODE END Application */
