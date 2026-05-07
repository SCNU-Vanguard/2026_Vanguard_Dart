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

#include "IA6B.h"
#include "motor_algrothim.h"
#include "referee.h"
#include "MotorControlTask.h"
#include "FireControl.h"
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
#if 0
osThreadId_t StoreEnergyTaskHandle;
const osThreadAttr_t StoreEnergyTask_attributes = {
    .name = "StoreEnergyTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};

// 换弹任务
osThreadId_t LoadTaskHandle;
const osThreadAttr_t LoadTask_attributes = {
    .name = "LoadTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal7,
};

// 飞镖状态设置(设置Yaw和射程)
osThreadId_t StateSetTaskHandle;
const osThreadAttr_t StateSetTask_attributes = {
    .name = "StateSetTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

// 裁判系统轮询解析任务
osThreadId_t RefereeTaskHandle;
const osThreadAttr_t RefereeTask_attributes = {
    .name = "RefereeTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

// 3508单独控制任务
osThreadId_t g_3508_CtrlHandle;
const osThreadAttr_t Ctrl_3508_Task_attributes = {
    .name = "3508CtrlTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

// 2006单独控制任务
osThreadId_t g_2006_CtrlHandle;
const osThreadAttr_t Ctrl_2006_Task_attributes = {
    .name = "2006CtrlTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

osThreadId_t g_LeftStore_CtrlHandle;
const osThreadAttr_t Ctrl_Left_Store_Task_attributes = {
    .name = "LStoreCtrlTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

osThreadId_t g_RightStore_CtrlHandle;
const osThreadAttr_t Ctrl_Right_Store_Task_attributes = {
    .name = "RStoreCtrlTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

// 6020单独控制任务
osThreadId_t g_6020_CtrlHandle;
const osThreadAttr_t Ctrl_6020_Task_attributes = {
    .name = "6020CtrlTask",
    .stack_size = 64 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};
#endif

extern osThreadId_t StoreEnergyTaskHandle;
extern const osThreadAttr_t StoreEnergyTask_attributes;
extern osThreadId_t LoadTaskHandle;
extern const osThreadAttr_t LoadTask_attributes;
extern osThreadId_t StateSetTaskHandle;
extern const osThreadAttr_t StateSetTask_attributes;
extern osThreadId_t RefereeTaskHandle;
extern const osThreadAttr_t RefereeTask_attributes;
extern osThreadId_t g_3508_CtrlHandle;
extern const osThreadAttr_t Ctrl_3508_Task_attributes;
extern osThreadId_t g_2006_CtrlHandle;
extern const osThreadAttr_t Ctrl_2006_Task_attributes;
extern osThreadId_t g_LeftStore_CtrlHandle;
extern const osThreadAttr_t Ctrl_Left_Store_Task_attributes;
extern osThreadId_t g_RightStore_CtrlHandle;
extern const osThreadAttr_t Ctrl_Right_Store_Task_attributes;
extern osThreadId_t g_6020_CtrlHandle;
extern const osThreadAttr_t Ctrl_6020_Task_attributes;
typedef enum
{
    STORE_FLOW_WAIT_SETUP = 0,
    STORE_FLOW_READY,
    STORE_FLOW_STORING,
    STORE_FLOW_WAIT_SHOOT_PERMISSION,
    STORE_FLOW_FIRING,
    STORE_FLOW_SAFE_RETURN,
} StoreEnergyFlowState_t;

#define LOAD_REQUEST_MAGIC 0x4C445254UL
#define LOAD_REQUEST_PRIORITY_NORMAL 80U
#define LOAD_REQUEST_MAX_AGE_MS 1000U
#define LOAD_REQUEST_DONE_TIMEOUT_MS 15000U

typedef enum
{
    LOAD_RESULT_NONE = 0,
    LOAD_RESULT_ACCEPTED,
    LOAD_RESULT_DONE,
    LOAD_RESULT_FAILED,
    LOAD_RESULT_ABORTED,
} LoadResult_e;

typedef struct
{
    uint32_t magic;
    uint32_t seq;
    uint32_t timestamp_ms;
    uint8_t dart_num;
    uint8_t priority;
    uint16_t checksum;
} LoadRequest_t;

typedef struct
{
    uint32_t seq;
    LoadResult_e result;
} LoadAck_t;

// 模块驱动初始化
void Module_Init(void);

// 任务初始化
void TaskInitFunc(void);

#endif
