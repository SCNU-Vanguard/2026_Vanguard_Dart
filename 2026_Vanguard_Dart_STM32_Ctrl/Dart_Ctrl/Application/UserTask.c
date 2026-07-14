/************************************************************************************************************************
 * 文件：UserTask.c
 * 用途：用户任务定义，所有的任务都在此
 * 创建者：bale
 * 创建日期：忘了
 * 目标：35s内完成4次发射过程
 * 流程：视觉/遥控器(可调射程)纠正方向->第一发飞镖->滑台下滑(同时扳机要放下来)->到达地点(扳机绷紧)->根据所需射程调节电机->释放扳机
 *
 * 框图：
 *
 *
 *                                                                        \===========================================\
 *                                                                         \                    0x201                  \
 *                                                                          \                  RM-M3508                 \
 *                                                                           \                                           \
 *                                                                            \      0x01          0x02          0x03     \
 *                                                                             \  SerialServo   SerialServo   SerialServo  \
 *                                                                              \                                           \
 *                                                                               \                                           \
 *                                                                              |=============================================|
 *                                                                              |                                             |
 *                                                                              |                                             |
 *                                                                              |                    0x03                     |
 * 遥控器(FS-I6)----------------------------------------------->FX-16AB接收机--------------------DM-J4310-YAW                  |
 *                                                                              |                                             |
 *                                                                              |     0x01                          0x02      |
 *                                                                              |(DM-S3519-Left)              (DM-S3519-Right)|
 *                                                                              |                                             |
 *                                                                              |                   0x205                     |
 *                                                                              |               RM-M2006-Trigger              |
 *                                                                              |                                             |
 *                                                                              |                  TriggerServo               |
 *                                                                              |=============================================|
 *
 * 现在的毛胚房版本是没有加上视觉和上位机的,但是要用的话那么,首先要注释掉串口通信部份的代码,也就说全靠自己
 * 无视觉方案:
 * 上电、初始化后根据预设方案调节扳机位置和Yaw角度,让储能电机开始工作,拉到指定位置之后电机停止,之后等待换弹
 * 第一发可以不用换弹,直接拉到舵机位置,之后扳机处于位置之后电机不用动,扳机释放飞镖(电机处于失能状态),这里还
 * 要等待测试达妙电机的具体逻辑
 *              // TODO力矩异常检测
 ***********************************************************************************************************************/
#include "UserTask.h"

// 任务：
// 状态设置任务(云台Yaw轴角度和扳机位置调节任务)
// 换弹任务
// 存储拉簧动能任务
// yaw调整任务

/************************全局或静态作用域*********************/

volatile bool g_bStoreMotorSafeReturnPending = false; /* 初始化 g_bStoreMotorSafeReturnPending。 */

// 任务句柄和任务属性只在这里定义一次，头文件只保留 extern 声明。
osThreadId_t StoreEnergyTaskHandle = NULL; /* 初始化 StoreEnergyTaskHandle。 */
const osThreadAttr_t StoreEnergyTask_attributes = { /* 初始化 StoreEnergyTask_attributes。 */
    .name = "StoreEnergyTask", /* 配置 name。 */
    .stack_size = 512 * 4, /* 配置 stack_size。 */
    .priority = (osPriority_t)osPriorityNormal1, /* 配置 priority。 */
};

osThreadId_t LoadTaskHandle = NULL; /* 初始化 LoadTaskHandle。 */
const osThreadAttr_t LoadTask_attributes = { /* 初始化 LoadTask_attributes。 */
    .name = "LoadTask", /* 配置 name。 */
    .stack_size = 256 * 4, /* 配置 stack_size。 */
    .priority = (osPriority_t)osPriorityBelowNormal7, /* 配置 priority。 */
};

osThreadId_t StateSetTaskHandle = NULL; /* 初始化 StateSetTaskHandle。 */
const osThreadAttr_t StateSetTask_attributes = { /* 初始化 StateSetTask_attributes。 */
    .name = "StateSetTask", /* 配置 name。 */
    .stack_size = 512 * 4, /* 配置 stack_size。 */
    .priority = (osPriority_t)osPriorityNormal, /* 配置 priority。 */
};

osThreadId_t RefereeTaskHandle = NULL; /* 初始化 RefereeTaskHandle。 */
const osThreadAttr_t RefereeTask_attributes = { /* 初始化 RefereeTask_attributes。 */
    .name = "RefereeTask", /* 配置 name。 */
    .stack_size = 256 * 4, /* 配置 stack_size。 */
    .priority = (osPriority_t)osPriorityAboveNormal, /* 配置 priority。 */
};

osThreadId_t g_MotorCtrlHandle = NULL; /* 初始化 g_MotorCtrlHandle。 */
const osThreadAttr_t MotorCtrl_Task_attributes = { /* 初始化 MotorCtrl_Task_attributes。 */
    .name = "MotorCtrlTask", /* 配置 name。 */
    /* 塌合原来 3508/2006/StoreSync/6020 四个任务，栈给 512*4
       （4 个电机 tick + S 型规划器 + stream-buffer 处理同时跑，单个 2ms
       周期内用量远小于原 4*128*4 合计，但留足裕度防 ISR 抢占时溢出）。*/
    .stack_size = 512 * 4, /* 配置 stack_size。 */
    .priority = (osPriority_t)osPriorityAboveNormal, /* 配置 priority。 */
};

// Store <-> Load 双向同步信号量
StaticSemaphore_t g_xStore2LoadSemBuffer; /* 保存 g_xStore2LoadSemBuffer。 */
SemaphoreHandle_t g_xStore2LoadSemHandle; // Store通知Load开始
StaticSemaphore_t g_xLoad2StoreSemBuffer; /* 保存 g_xLoad2StoreSemBuffer。 */
SemaphoreHandle_t g_xLoad2StoreSemHandle; // Load通知Store完成

// Store -> Load: 储能电机到位通知信号量（换弹时舵机需等储能机构到位才释放）
StaticSemaphore_t g_xStoreMotorArrivedSemBuffer; /* 保存 g_xStoreMotorArrivedSemBuffer。 */
SemaphoreHandle_t g_xStoreMotorArrivedSemHandle; /* 保存 g_xStoreMotorArrivedSemHandle。 */

// 储能任务信号量
static StaticSemaphore_t g_xStoreSemaphore; /* 保存 g_xStoreSemaphore。 */
SemaphoreHandle_t g_xStoreSemaphoreHandle; /* 保存 g_xStoreSemaphoreHandle。 */

// 储能任务消息
StreamBufferHandle_t xLoadStreamBuf; /* 保存 xLoadStreamBuf。 */

// 换弹结构电机队列
static uint8_t g_ucReloadQueueStorage[4 * sizeof(uint8_t)]; /* 保存 g_ucReloadQueueStorage。 */
static StaticQueue_t g_xReloadQueue; /* 保存 g_xReloadQueue。 */
QueueHandle_t g_xLoad3508QueueHandler; /* 保存 g_xLoad3508QueueHandler。 */

static uint8_t g_ucStateSetRequestQueueStorage[sizeof(StateSetRequest_t)]; /* 保存 g_ucStateSetRequestQueueStorage。 */
static StaticQueue_t g_xStateSetRequestQueueBuffer; /* 保存 g_xStateSetRequestQueueBuffer。 */
QueueHandle_t g_xStateSetRequestQueueHandle; /* 保存 g_xStateSetRequestQueueHandle。 */
static StaticSemaphore_t g_xStateSetDoneSemBuffer; /* 保存 g_xStateSetDoneSemBuffer。 */
SemaphoreHandle_t g_xStateSetDoneSemHandle; /* 保存 g_xStateSetDoneSemHandle。 */

// StateSet任务的事件组
static StaticEventGroup_t g_pxStateSetEventGroupBuffer; /* 保存 g_pxStateSetEventGroupBuffer。 */
EventGroupHandle_t g_pxStateSetEventGroupHandle; /* 保存 g_pxStateSetEventGroupHandle。 */
/*---------------------------------------------------------------------------------------*/

// 模块驱动初始化
void Module_Init(void) /* 实现 Module_Init。 */
{
    BSP_POWER_DeInit(); // 失能无绿灯，亮红灯
    DWT_Init(180); /* 调用 DWT_Init。 */
    SoftwareWatchdog_Init();  // 先建立统一 1 ms 软件看门狗
    MotorInit();              // 注册电机并初始化 CAN 外设
    CanFilterCfg();           // 启动过滤器后再开始 10 ms 在线监控
    CanMotor_WatchdogInit();  // 注册 RM 连续反馈和 DM 单次回复通道
    BSP_UART_Init(); /* 调用 BSP_UART_Init。 */
    Servo_RegisterDartProfiles(); // 注册时机要求在ServoInit之前
    // while (!ServoInit())
    //    ; // 这个地方有一个回调,需要进行数据读取
    UART_SetProtocolType(BSP_UART6, PROTOCOL_IBUS); /* 调用 UART_SetProtocolType。 */
    (void)Referee_Register(&huart8); // 仅做裁判数据结构注册，不再扰动UART8接收状态
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); /* 调用 HAL_TIM_PWM_Start。 */
    HAL_TIM_Base_Start(&htim8); /* 调用 HAL_TIM_Base_Start。 */
    HAL_Delay(100); /* 调用 HAL_Delay。 */
}

/***********************************************TASK******************************************************/
void StoreEnergyTaskFunc(void *argument); /* 声明 StoreEnergyTaskFunc 接口。 */
void LoadTaskFunc(void *argument); /* 声明 LoadTaskFunc 接口。 */
void LoadServoTaskFunc(void *argument); /* 声明 LoadServoTaskFunc 接口。 */
void StateSetTaskFunc(void *argument); /* 声明 StateSetTaskFunc 接口。 */
void RefereeTaskFunc(void *argument); /* 声明 RefereeTaskFunc 接口。 */

/***********************************
 * 函数名: TaskInitFunc
 * 作用:   任务初始化创建
 * 参数:   无
 **********************************/
void TaskInitFunc(void) /* 实现 TaskInitFunc。 */
{
    MotorControl_Init(); /* 调用 MotorControl_Init。 */

    g_xLoad3508QueueHandler = xQueueCreateStatic(4, sizeof(uint8_t), g_ucReloadQueueStorage, &g_xReloadQueue); /* 更新 g_xLoad3508QueueHandler。 */
    g_xStateSetRequestQueueHandle = xQueueCreateStatic(1, sizeof(StateSetRequest_t), g_ucStateSetRequestQueueStorage, &g_xStateSetRequestQueueBuffer); /* 更新 g_xStateSetRequestQueueHandle。 */
    g_xStoreSemaphoreHandle = xSemaphoreCreateCountingStatic(5, sizeof(uint8_t), &g_xStoreSemaphore); /* 更新 g_xStoreSemaphoreHandle。 */
    g_xStore2LoadSemHandle = xSemaphoreCreateBinaryStatic(&g_xStore2LoadSemBuffer); /* 更新 g_xStore2LoadSemHandle。 */
    g_xLoad2StoreSemHandle = xSemaphoreCreateBinaryStatic(&g_xLoad2StoreSemBuffer); /* 更新 g_xLoad2StoreSemHandle。 */
    g_xStoreMotorArrivedSemHandle = xSemaphoreCreateBinaryStatic(&g_xStoreMotorArrivedSemBuffer); /* 更新 g_xStoreMotorArrivedSemHandle。 */
    g_xStateSetDoneSemHandle = xSemaphoreCreateBinaryStatic(&g_xStateSetDoneSemBuffer); /* 更新 g_xStateSetDoneSemHandle。 */
    xLoadStreamBuf = xStreamBufferCreate(sizeof(LoadRequest_t), sizeof(LoadRequest_t)); /* 更新 xLoadStreamBuf。 */
    g_pxStateSetEventGroupHandle = xEventGroupCreateStatic(&g_pxStateSetEventGroupBuffer); /* 更新 g_pxStateSetEventGroupHandle。 */

    ControlState_Init(); /* 调用 ControlState_Init。 */
    FireControl_Init(); /* 调用 FireControl_Init。 */
    ControlState_WatchdogInit(); // 注册 100 ms IBUS 看门狗
    FireControl_WatchdogInit();  // 注册 1500 ms 裁判看门狗

    // 任务初始化
    LoadTaskHandle = osThreadNew(LoadTaskFunc, NULL, &LoadTask_attributes); /* 更新 LoadTaskHandle。 */
    StoreEnergyTaskHandle = osThreadNew(StoreEnergyTaskFunc, NULL, &StoreEnergyTask_attributes); /* 更新 StoreEnergyTaskHandle。 */
    RefereeTaskHandle = osThreadNew(RefereeTaskFunc, NULL, &RefereeTask_attributes); /* 更新 RefereeTaskHandle。 */
    StateSetTaskHandle = osThreadNew(StateSetTaskFunc, NULL, &StateSetTask_attributes); /* 更新 StateSetTaskHandle。 */

    // 所有电机由统一的 MotorCtrlTask 2ms 周期遍历驱动
    g_MotorCtrlHandle = osThreadNew(MotorCtrlTask, NULL, &MotorCtrl_Task_attributes); /* 更新 g_MotorCtrlHandle。 */

    // 创建ControlState的IbusTask和ControlTask
    ControlState_CreateTasks(); /* 调用 ControlState_CreateTasks。 */
}

/***********************************
 * 函数名: StoreEnergyTaskFunc
 * 作用:   拉簧储能任务(控制左右储能 M3508 电机转动)
 * 参数:   无
 **********************************/
void StoreEnergyTaskFunc(void *argument) /* 实现 StoreEnergyTaskFunc。 */
{
    (void)argument; /* 显式忽略参数 argument。 */
    // 等待事件组：云台和扳机都就绪后才开始
    xEventGroupWaitBits(g_pxStateSetEventGroupHandle, EVENT_ALL_READY, pdFALSE, pdTRUE, portMAX_DELAY); /* 调用 xEventGroupWaitBits。 */

    // 接收计数型信号量之后才可以正常
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_shoot); /* 调用 __HAL_TIM_SET_COMPARE。 */
    vTaskDelay(pdMS_TO_TICKS(1500)); /* 调用 vTaskDelay。 */
    StoreEnergyTaskMainLoopFuc(); /* 调用 StoreEnergyTaskMainLoopFuc。 */
}

/***********************************
 * 函数名: LoadTaskFunc
 * 作用:   换弹任务（传送带电机控制以及电机目标设定）
 * 参数:   无
 **********************************/
void LoadTaskFunc(void *argument) /* 实现 LoadTaskFunc。 */
{
    (void)argument; /* 显式忽略参数 argument。 */
    xEventGroupWaitBits(g_pxStateSetEventGroupHandle, EVENT_ALL_READY, pdFALSE, pdTRUE, portMAX_DELAY); /* 调用 xEventGroupWaitBits。 */

    // 直接设置成0°,限位确认
    Servo_MoveAllToZero(300); /* 调用 Servo_MoveAllToZero。 */

    // 任务创建
    TaskHandle_t LoadServoTaskHandle = NULL; /* 初始化 LoadServoTaskHandle。 */
    xTaskCreate(LoadServoTaskFunc, "LoadServo", 64 * 4, NULL, osPriorityBelowNormal7, &LoadServoTaskHandle); /* 调用 xTaskCreate。 */

    // 初始目标保持为当前位置
    (void)LoadMotor_SubmitTarget(LOAD_MOTOR_OWNER_LOAD, /* 开始调用 LoadMotor_SubmitTarget。 */
                                 LOAD_MOTOR_PRIORITY_LOAD, /* 定义 LOAD_MOTOR_PRIORITY_LOAD 枚举项。 */
                                 Motor_GetTotalAngle(RM_3508_GRIPPER), /* 传入下一项参数或数据。 */
                                 LOAD_DEADZONE_TIMEOUT_MS); /* 完成本行操作。 */

    LoadMotorTaskMainLoopFunc(); /* 调用 LoadMotorTaskMainLoopFunc。 */
}

/***********************************
 * 函数名: LoadServoTaskFunc
 * 作用:   换弹任务（舵机释放）
 * 参数:   无
 **********************************/
void LoadServoTaskFunc(void *argument) /* 实现 LoadServoTaskFunc。 */
{
    (void)argument; /* 显式忽略参数 argument。 */
    xEventGroupWaitBits(g_pxStateSetEventGroupHandle, EVENT_ALL_READY, pdFALSE, pdTRUE, portMAX_DELAY); /* 调用 xEventGroupWaitBits。 */

    vTaskDelay(pdMS_TO_TICKS(1)); // 最开始放一个Tick

    // 在这里驱动换弹舵机运动
    LoadServoTaskMainLoopFunc(); /* 调用 LoadServoTaskMainLoopFunc。 */
}

/************************************
 * 函数名: RefereeTaskFunc
 * 作用: 解算裁判系统通信数据
 * 参数: 无
 * TODO: 确认是否可以正常解算数据
 ***********************************/
void RefereeTaskFunc(void *argument) /* 实现 RefereeTaskFunc。 */
{
    (void)argument; /* 显式忽略参数 argument。 */
    RefereeTaskMainLoopFunc(); /* 调用 RefereeTaskMainLoopFunc。 */
}

/************************************
 * 函数名：StateSetTaskFunc
 * 作用：  飞镖状态设置
 * 参数：  无
 * TODO:  等待链接上位机和测试无上位机版本
 ***********************************/
void StateSetTaskFunc(void *argument) /* 实现 StateSetTaskFunc。 */
{
    (void)argument; /* 显式忽略参数 argument。 */
    StateSetTaskMainLoopFunc(); /* 调用 StateSetTaskMainLoopFunc。 */
}
