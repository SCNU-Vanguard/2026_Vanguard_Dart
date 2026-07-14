#ifndef __USER_TASK_H_ /* 按 __USER_TASK_H_ 选择编译分支。 */
#define __USER_TASK_H_ /* 定义 __USER_TASK_H_。 */

#include "main.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "event_groups.h"
#include "semphr.h"
#include "queue.h"
#include "stream_buffer.h"
#include "config.h"
#include "bsp_dwt.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "bsp_pwr.h"
#include "IA6B.h"
#include "motor_algrothim.h"
#include "referee.h"
#include "MotorControlTask.h"
#include "FireControl.h"
#include "SoftwareWatchdog.h"
#include <string.h>
#include "HX06L.h"
#include "CanMotor.h"
#include "RM_Motor.h"
#include "DM_Motor.h"
#include "ControlState.h"
#include "StateSetFunc.h"
#include "StoreEnergyTaskFunc.h"
#include "LoadTaskFunc.h"
#include "RefereeTaskFunc.h"

// 拉簧储能任务
#if 0 /* 按 0 选择编译分支。 */
osThreadId_t StoreEnergyTaskHandle; /* 保存 StoreEnergyTaskHandle。 */
const osThreadAttr_t StoreEnergyTask_attributes = { /* 初始化 StoreEnergyTask_attributes。 */
    .name = "StoreEnergyTask", /* 配置 name。 */
    .stack_size = 512 * 4, /* 配置 stack_size。 */
    .priority = (osPriority_t)osPriorityNormal1, /* 配置 priority。 */
};

// 换弹任务
osThreadId_t LoadTaskHandle; /* 保存 LoadTaskHandle。 */
const osThreadAttr_t LoadTask_attributes = { /* 初始化 LoadTask_attributes。 */
    .name = "LoadTask", /* 配置 name。 */
    .stack_size = 256 * 4, /* 配置 stack_size。 */
    .priority = (osPriority_t)osPriorityBelowNormal7, /* 配置 priority。 */
};

// 飞镖状态设置(设置Yaw和射程)
osThreadId_t StateSetTaskHandle; /* 保存 StateSetTaskHandle。 */
const osThreadAttr_t StateSetTask_attributes = { /* 初始化 StateSetTask_attributes。 */
    .name = "StateSetTask", /* 配置 name。 */
    .stack_size = 512 * 4, /* 配置 stack_size。 */
    .priority = (osPriority_t)osPriorityNormal, /* 配置 priority。 */
};

// 裁判系统轮询解析任务
osThreadId_t RefereeTaskHandle; /* 保存 RefereeTaskHandle。 */
const osThreadAttr_t RefereeTask_attributes = { /* 初始化 RefereeTask_attributes。 */
    .name = "RefereeTask", /* 配置 name。 */
    .stack_size = 256 * 4, /* 配置 stack_size。 */
    .priority = (osPriority_t)osPriorityAboveNormal, /* 配置 priority。 */
};

// 统一电机控制任务（原 3508/2006/StoreSync/6020 四个塌合为一）
osThreadId_t g_MotorCtrlHandle; /* 保存 g_MotorCtrlHandle。 */
const osThreadAttr_t MotorCtrl_Task_attributes = { /* 初始化 MotorCtrl_Task_attributes。 */
    .name = "MotorCtrlTask", /* 配置 name。 */
    .stack_size = 512 * 4, /* 配置 stack_size。 */
    .priority = (osPriority_t)osPriorityAboveNormal, /* 配置 priority。 */
};
#endif /* 结束条件编译。 */

extern osThreadId_t StoreEnergyTaskHandle; /* 声明外部变量 StoreEnergyTaskHandle。 */
extern const osThreadAttr_t StoreEnergyTask_attributes; /* 声明外部变量 StoreEnergyTask_attributes。 */
extern osThreadId_t LoadTaskHandle; /* 声明外部变量 LoadTaskHandle。 */
extern const osThreadAttr_t LoadTask_attributes; /* 声明外部变量 LoadTask_attributes。 */
extern osThreadId_t StateSetTaskHandle; /* 声明外部变量 StateSetTaskHandle。 */
extern const osThreadAttr_t StateSetTask_attributes; /* 声明外部变量 StateSetTask_attributes。 */
extern osThreadId_t RefereeTaskHandle; /* 声明外部变量 RefereeTaskHandle。 */
extern const osThreadAttr_t RefereeTask_attributes; /* 声明外部变量 RefereeTask_attributes。 */
extern osThreadId_t g_MotorCtrlHandle; /* 声明外部变量 g_MotorCtrlHandle。 */
extern const osThreadAttr_t MotorCtrl_Task_attributes; /* 声明外部变量 MotorCtrl_Task_attributes。 */
typedef enum /* 开始定义数据类型。 */
{
    STORE_FLOW_WAIT_SETUP = 0, /* 定义 STORE_FLOW_WAIT_SETUP 枚举项。 */
    STORE_FLOW_READY, /* 定义 STORE_FLOW_READY 枚举项。 */
    STORE_FLOW_STORING, /* 定义 STORE_FLOW_STORING 枚举项。 */
    STORE_FLOW_WAIT_SHOOT_PERMISSION, /* 定义 STORE_FLOW_WAIT_SHOOT_PERMISSION 枚举项。 */
    STORE_FLOW_FIRING, /* 定义 STORE_FLOW_FIRING 枚举项。 */
    STORE_FLOW_SAFE_RETURN, /* 定义 STORE_FLOW_SAFE_RETURN 枚举项。 */
} StoreEnergyFlowState_t; /* 结束 StoreEnergyFlowState_t 类型定义。 */

#define LOAD_REQUEST_MAGIC 0x4C445254UL /* 定义 LOAD_REQUEST_MAGIC。 */
#define LOAD_REQUEST_PRIORITY_NORMAL 80U /* 定义 LOAD_REQUEST_PRIORITY_NORMAL。 */
#define LOAD_REQUEST_MAX_AGE_MS 1000U /* 定义 LOAD_REQUEST_MAX_AGE_MS。 */
#define LOAD_REQUEST_DONE_TIMEOUT_MS 15000U /* 定义 LOAD_REQUEST_DONE_TIMEOUT_MS。 */

typedef enum /* 开始定义数据类型。 */
{
    LOAD_RESULT_NONE = 0, /* 定义 LOAD_RESULT_NONE 枚举项。 */
    LOAD_RESULT_ACCEPTED, /* 定义 LOAD_RESULT_ACCEPTED 枚举项。 */
    LOAD_RESULT_DONE, /* 定义 LOAD_RESULT_DONE 枚举项。 */
    LOAD_RESULT_FAILED, /* 定义 LOAD_RESULT_FAILED 枚举项。 */
    LOAD_RESULT_ABORTED, /* 定义 LOAD_RESULT_ABORTED 枚举项。 */
} LoadResult_e; /* 结束 LoadResult_e 类型定义。 */

typedef struct /* 开始定义数据类型。 */
{
    uint32_t magic; /* 保存 magic。 */
    uint32_t seq; /* 保存 seq。 */
    uint32_t timestamp_ms; /* 保存 timestamp_ms。 */
    uint8_t dart_num; /* 保存 dart_num。 */
    uint8_t priority; /* 保存 priority。 */
    uint16_t checksum; /* 保存 checksum。 */
} LoadRequest_t; /* 结束 LoadRequest_t 类型定义。 */

typedef struct /* 开始定义数据类型。 */
{
    uint32_t seq; /* 保存 seq。 */
    LoadResult_e result; /* 保存 result。 */
} LoadAck_t; /* 结束 LoadAck_t 类型定义。 */

// 模块驱动初始化
void Module_Init(void); /* 声明 Module_Init 接口。 */

// 任务初始化
void TaskInitFunc(void); /* 声明 TaskInitFunc 接口。 */

#endif /* 结束条件编译。 */
