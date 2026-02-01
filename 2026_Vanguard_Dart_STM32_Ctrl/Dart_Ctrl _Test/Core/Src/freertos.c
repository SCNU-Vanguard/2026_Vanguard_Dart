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
#include "config.h"
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

// float Target = 400.0f; // 360° * 19.2 (度数 * 减速比)| 360 * 36.0(度数 * 减速比) 12960
// static uint32_t lastServoTime = 0;
// float frequence = 0.0f;
// float time = 0.0f;

// static StackType_t g_pxChangeTarget[128];
// static StaticTask_t g_TCBChangeTarget;
// static TaskHandle_t g_HandleChangeTarget;

// uint16_t Gripper1Angle = 0;
// uint16_t Gripper2Angle = 0;
// uint16_t Gripper3Angle = 0;

static StaticSemaphore_t g_xRmBufferMutexBuffer;
SemaphoreHandle_t g_xRmBufferMutexHandle;

/// @brief 改变电机目标数值
/// @param arg 任务参数（未使用）
/// @return 无
// void pxChangeTarget(void *arg)
// {
//   (void)arg; // 消除未使用参数警告
//   vTaskDelay(20);
//   while (1)
//   {
//     vTaskDelay(1500);
//     if (Target < 500.0f)
//     {
//       Target += 100.0f;
//     }
//     else
//     {
//       Target = 0.0f;
//       vTaskDelay(4000);
//     }
//   }
// }

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

  // 改变电机目标数值的一个乐色任务
  // g_HandleChangeTarget = xTaskCreateStatic(pxChangeTarget, "ChangeTarget", 128, NULL, osPriorityNormal, g_pxChangeTarget, &g_TCBChangeTarget);

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
  // ServoInit();
  // vTaskDelay(100); // 等待初始化完成
  // uint8_t servo_ids[3] = {0x01, 0x02, 0x03};
  // uint16_t angles[3] = {0, 0, 0};
  // uint16_t data[3] = {375, 375, 375};
  // ServoControlPos(3, 375, 1000);
  // vTaskDelay(1);
  // ServoControlPos(2, 375, 1000);
  // vTaskDelay(1);
  // ServoControlPos(1, 375, 1000);
  // vTaskDelay(2000);
  // ServoControlMulti(3, servo_ids, angles, 1000);
  // uint32_t last_cnt = 0x0000;
  // float dt = 0.0f;
  // uint8_t test = 0;
  // float offset_angle = MotorManager.MotorList[RM_3508_GRIPPER - 1].motor_data.offset_ecd_angle;
  // bool temp = true;
  // DWT_GetDeltaT(&last_cnt);
  // uint32_t b = 0;
  // b = HAL_GetTick();
  // while (temp)
  // {
  //   DM_Motor_RefreshData(DM_4310_YAW);
  //   if (b + 1000 < HAL_GetTick())
  //   {
  //     temp = false;
  //   }
  // }
  /* Infinite loop */
  for (;;)
  {
    // 中期文档测量
    /* 1.RM3508所在换弹程序的时长以及曲线 */
    /* 3508PID响应曲线 */
    /* 基于飞镖换弹的阶跃响应 */
    // RmMotorPID_Calc(RM_3508_GRIPPER, Target);
    // dt = DWT_GetDeltaT(&last_cnt);
    // frequence = 1.0f / dt;

    /* 飞镖换弹的实际任务测试频率 */
    // for (uint8_t a = 0; a < 3; a++)
    // {
    //   if (a == 0)
    //   {
    //     Target = FirstServoLoc;
    //     Gripper1Angle = 375;
    //   }
    //   else if (a == 1)
    //   {
    //     Target = SecondServoLoc;
    //     Gripper1Angle = 375;
    //   }
    //   else
    //   {
    //     Target = ThirdServoLoc;
    //     Gripper3Angle = 375;
    //   }
    //   while (!IS_IN_DEADZONE(Motor_GetTotalAngle(RM_3508_GRIPPER), Target, MOTOR_DEAD_ZONE))
    //   {
    //     RmMotorPID_Calc(RM_3508_GRIPPER, Target);
    //   }
    //   CASCADE_PID_Clear_Integral(&MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid);
    //   float GripperTarget = offset_angle;
    //   float MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
    // while (!IS_IN_DEADZONE(MotorData, 0.0f, MOTOR_DEAD_ZONE))
    //   {
    //     RmMotorPID_Calc(RM_3508_GRIPPER, GripperTarget);
    //     MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
    //   }
    // 	osDelay(310);
    //   dt = DWT_GetDeltaT(&last_cnt);
    //   time = 1.0f / dt;
    // }
    /* 2.RM2006所在的扳机响应时长以及曲线 */
    // RmMotorPID_Calc(RM_2006_TRIGGER, Target);
    // dt = DWT_GetDeltaT(&last_cnt);
    // frequence = 1.0f / dt;

    /* 3.云台电机响应时长以及曲线 */
    // float degree = 0.0f;
    // DmMotorSendCfg(DM_4310_YAW, Target, 0.0f, DM_MIT); // 调节Yaw轴位置
    // degree = Motor_GetTotalAngle(DM_4310_YAW);
    // printf("torque=%f,target=%f,feedback=%f,", MotorManager.MotorList[4].motor_data.solved_data[2], Target, degree);
    // vTaskDelay(3);
    // dt = DWT_GetDeltaT(&last_cnt);
    // frequence = 1.0f / dt;
    // while (degree = Motor_GetTotalAngle(DM_4310_YAW), !IS_IN_DEADZONE(degree, Target, 1.0f))
    // {
    //   DM_Motor_RefreshData(DM_4310_YAW);
    //   vTaskDelay(1);
    //   temp = true;
    // }
    // if (temp)
    // {
    //   time = dt;
    //   temp = false;
    // 	b = HAL_GetTick();
    // }
    // if (b + 1000 < HAL_GetTick())
    // {
    // 	Target = 0.0f;
    // }
    /* 4.储能电机对应的位置响应以及力矩响应 */

    // static float last_target = 0.0f;
    // static float start_time = 0.0f;
    // if (Target != last_target)
    // {
    //   last_target = Target;
    //   start_time = DWT_GetTimeline_ms();
    // }
    // float degree = 0.0f;
    // DmMotorSendCfg(DM_3519_STRENTH_LEFT, -Target, 5.0f, DM_LOCATION_SPEED);
    // DmMotorSendCfg(DM_3519_STRENTH_RIGHT, Target, 5.0f, DM_LOCATION_SPEED);
    // if (IS_IN_DEADZONE(-MotorManager.MotorList[2].motor_data.solved_data[0], Target, 1.0f))
    // {
    //   if (Target < 500.0f)
    //   {
    //     Target += 100.0f;
    //   }
    //   else
    //   {
    //     Target = 0.0f;
    //     vTaskDelay(4000);
    //   }
    //   dt = DWT_GetDeltaT(&last_cnt);
    //   frequence = 1.0f / dt;
    // }
    // printf("leftAngle=%.1f,rightAngle=%.1f,target=%.1f,feedback=%.1f,leftTorque=%.1f,rightTorque=%.1f,", -MotorManager.MotorList[2].motor_data.solved_data[0], MotorManager.MotorList[3].motor_data.solved_data[0], Target, degree, -MotorManager.MotorList[2].motor_data.solved_data[2], MotorManager.MotorList[3].motor_data.solved_data[2]);
    // vTaskDelay(1);

    // RmMotorPID_Calc(RM_2006_TRIGGER, Target);
    // RmMotorSendCfg(RM_2006_TRIGGER, 1000);
    HAL_GPIO_TogglePin(Green_GPIO_Port, Green_Pin);
    vTaskDelay(250);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
