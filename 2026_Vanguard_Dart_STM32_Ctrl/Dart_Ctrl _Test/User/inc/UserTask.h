#ifndef __USER_TASK_H_
#define __USER_TASK_H_

#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "event_groups.h" // ARM.FreeRTOS::RTOS:Event Groups
#include "semphr.h"       // ARM.FreeRTOS::RTOS:Core
#include "queue.h"
#include "stream_buffer.h"

#include "config.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_pwr.h"

#include "HX06L.h"
#include "CanMotor.h"
#include "DM_Motor.h"
#include "RM_Motor.h"
#include "UartModule.h"
#include "ControlState.h"

// 模块驱动初始化
void Module_Init(void);

// 任务初始化
void TaskInitFunc(void);

#endif
