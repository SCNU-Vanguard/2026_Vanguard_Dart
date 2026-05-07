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

volatile bool g_bStoreMotorSafeReturnPending = false;

// 任务句柄和任务属性只在这里定义一次，头文件只保留 extern 声明。
osThreadId_t StoreEnergyTaskHandle = NULL;
const osThreadAttr_t StoreEnergyTask_attributes = {
    .name = "StoreEnergyTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};

osThreadId_t LoadTaskHandle = NULL;
const osThreadAttr_t LoadTask_attributes = {
    .name = "LoadTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal7,
};

osThreadId_t StateSetTaskHandle = NULL;
const osThreadAttr_t StateSetTask_attributes = {
    .name = "StateSetTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t RefereeTaskHandle = NULL;
const osThreadAttr_t RefereeTask_attributes = {
    .name = "RefereeTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

osThreadId_t g_3508_CtrlHandle = NULL;
const osThreadAttr_t Ctrl_3508_Task_attributes = {
    .name = "3508CtrlTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

osThreadId_t g_2006_CtrlHandle = NULL;
const osThreadAttr_t Ctrl_2006_Task_attributes = {
    .name = "2006CtrlTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

osThreadId_t g_LeftStore_CtrlHandle = NULL;
const osThreadAttr_t Ctrl_Left_Store_Task_attributes = {
    .name = "LStoreCtrlTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

osThreadId_t g_RightStore_CtrlHandle = NULL;
const osThreadAttr_t Ctrl_Right_Store_Task_attributes = {
    .name = "RStoreCtrlTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

osThreadId_t g_6020_CtrlHandle = NULL;
const osThreadAttr_t Ctrl_6020_Task_attributes = {
    .name = "6020CtrlTask",
    .stack_size = 64 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

// Store <-> Load 双向同步信号量
StaticSemaphore_t g_xStore2LoadSemBuffer;
SemaphoreHandle_t g_xStore2LoadSemHandle; // Store通知Load开始
StaticSemaphore_t g_xLoad2StoreSemBuffer;
SemaphoreHandle_t g_xLoad2StoreSemHandle; // Load通知Store完成

// Store -> Load: 储能电机到位通知信号量（换弹时舵机需等储能机构到位才释放）
StaticSemaphore_t g_xStoreMotorArrivedSemBuffer;
SemaphoreHandle_t g_xStoreMotorArrivedSemHandle;

// 储能任务信号量
static StaticSemaphore_t g_xStoreSemaphore;
SemaphoreHandle_t g_xStoreSemaphoreHandle;

// 储能任务消息
StreamBufferHandle_t xLoadStreamBuf;

// 换弹结构电机队列
static uint8_t g_ucReloadQueueStorage[4 * sizeof(uint8_t)];
static StaticQueue_t g_xReloadQueue;
QueueHandle_t g_xLoad3508QueueHandler;

static uint8_t g_ucStateSetRequestQueueStorage[sizeof(StateSetRequest_t)];
static StaticQueue_t g_xStateSetRequestQueueBuffer;
QueueHandle_t g_xStateSetRequestQueueHandle;
static StaticSemaphore_t g_xStateSetDoneSemBuffer;
SemaphoreHandle_t g_xStateSetDoneSemHandle;

// StateSet任务的事件组
static StaticEventGroup_t g_pxStateSetEventGroupBuffer;
EventGroupHandle_t g_pxStateSetEventGroupHandle;
/*---------------------------------------------------------------------------------------*/

// 模块驱动初始化
void Module_Init(void)
{
    BSP_POWER_DeInit(); // 失能无绿灯，亮红灯
    DWT_Init(180);
    MotorInit();
    CanFilterCfg();
    BSP_UART_Init();
    Servo_RegisterDartProfiles(); // 注册时机要求在ServoInit之前
    // while (!ServoInit())
    //    ; // 这个地方有一个回调,需要进行数据读取
    UART_SetProtocolType(BSP_UART6, PROTOCOL_IBUS);
    (void)Referee_Register(&huart8); // 仅做裁判数据结构注册，不再扰动UART8接收状态
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    HAL_TIM_Base_Start(&htim8);
    HAL_Delay(100);
}

/// @brief 创建RTOS的通信量
/// @param  无
void RTOS_ModuleInit(void)
{
    MotorControl_Init();

    // QUEUE（队列）：1.融合到云台调节当中，保证云台调节正常<队列集>; 2.融入到换弹结构当中,确保换弹结构正常
    g_xLoad3508QueueHandler = xQueueCreateStatic(4, sizeof(uint8_t), g_ucReloadQueueStorage, &g_xReloadQueue);
    g_xStateSetRequestQueueHandle = xQueueCreateStatic(1, sizeof(StateSetRequest_t), g_ucStateSetRequestQueueStorage, &g_xStateSetRequestQueueBuffer);

    // SEMAPHORE（信号量）：信号量与dart_num共同决定飞镖换弹的状态
    g_xStoreSemaphoreHandle = xSemaphoreCreateCountingStatic(5, sizeof(uint8_t), &g_xStoreSemaphore);

    // 二值信号量：用于任务间双向同步（创建后初始状态为"空"）
    g_xStore2LoadSemHandle = xSemaphoreCreateBinaryStatic(&g_xStore2LoadSemBuffer);
    g_xLoad2StoreSemHandle = xSemaphoreCreateBinaryStatic(&g_xLoad2StoreSemBuffer);
    g_xStoreMotorArrivedSemHandle = xSemaphoreCreateBinaryStatic(&g_xStoreMotorArrivedSemBuffer);
    g_xStateSetDoneSemHandle = xSemaphoreCreateBinaryStatic(&g_xStateSetDoneSemBuffer);

    // 使用stream流传输固定长度换弹请求，信号量只负责通知“完整请求可读”
    xLoadStreamBuf = xStreamBufferCreate(sizeof(LoadRequest_t), sizeof(LoadRequest_t));

    // EVENTGROUPS（事件组）：用于确认云台电机和射程电机正常设置,只有这两个正常设置之后才可以进行下面的流程
    g_pxStateSetEventGroupHandle = xEventGroupCreateStatic(&g_pxStateSetEventGroupBuffer);
}

/***********************************************TASK******************************************************/
void StoreEnergyTaskFunc(void *argument);
void LoadTaskFunc(void *argument);
void LoadServoTaskFunc(void *argument);
void StateSetTaskFunc(void *argument);
void RefereeTaskFunc(void *argument);

/***********************************
 * 函数名: TaskInitFunc
 * 作用:   任务初始化创建
 * 参数:   无
 **********************************/
void TaskInitFunc(void)
{
    RTOS_ModuleInit();
    ControlState_Init();
    FireControl_Init();

    // 任务初始化
    LoadTaskHandle = osThreadNew(LoadTaskFunc, NULL, &LoadTask_attributes);
    StoreEnergyTaskHandle = osThreadNew(StoreEnergyTaskFunc, NULL, &StoreEnergyTask_attributes);
    RefereeTaskHandle = osThreadNew(RefereeTaskFunc, NULL, &RefereeTask_attributes);
    StateSetTaskHandle = osThreadNew(StateSetTaskFunc, NULL, &StateSetTask_attributes);

    // 所有电机控制单开一个任务保证响应
    g_3508_CtrlHandle = osThreadNew(Motor3508CtrlTask, NULL, &Ctrl_3508_Task_attributes);
    g_2006_CtrlHandle = osThreadNew(Motor2006CtrlTask, NULL, &Ctrl_2006_Task_attributes);
    g_LeftStore_CtrlHandle = osThreadNew(MotorLeftStore3508CtrlTask, NULL, &Ctrl_Left_Store_Task_attributes);
    g_RightStore_CtrlHandle = osThreadNew(MotorRightStore3508CtrlTask, NULL, &Ctrl_Right_Store_Task_attributes);
    // g_6020_CtrlHandle = osThreadNew(Motor6020CtrlTask, NULL, &Ctrl_6020_Task_attributes);

    // 创建ControlState的IbusTask和ControlTask
    ControlState_CreateTasks();
}

/**
 * @brief 将两侧储能 M3508 拉回上方零位（Top/0 位）
 * @note  用于裁判系统禁止发射时的保守回退动作
 */
static void StoreEnergy_ReturnStoreMotorToZero(void);

/**
 * @brief 控制左右储能 M3508 同步移动到指定目标
 * @note  用于裁判系统禁止发射时的保守回退动作
 */
static void StoreEnergy_MoveStoreMotorPairToTargets(float left_target, float right_target);

/// @brief 设置发射机构飞镖状态
/// @param occupied true->发射机构上已有飞镖
/// @param reloaded true->当前飞镖来自弹夹换弹
static void StoreEnergy_SetLauncherDartState(bool occupied, bool reloaded);

/// @brief 确认发射机构上是否已有飞镖
/// @return true->发射机构上已有飞镖
static bool StoreEnergy_HasLauncherDart(void);

/**
 * @brief 禁止发射时的安全退出：先释放扳机，再让两侧储能电机上移
 * @note  不扣弹、不清发射位占用，避免下次允许发射时重复换弹肘击
 */
static void StoreEnergy_ParkLauncherDartSafely(void);
static void StoreEnergy_ReturnLoad3508Home(void);

/// @brief 标记当前储能/换弹联动是否仍由 Store 流程控制
/// @param active true 代表联动进行中
/// @param abort_pending true 代表已经请求中断本轮联动
static void StoreEnergy_SetLoadCycleState(bool active, bool abort_pending);

/// @brief 外部有没有要求立刻中止这轮换弹联动
/// @param 无
/// @return true->打断当前流程，直接安全回退
static bool StoreEnergy_IsLoadAbortRequested(void);

/// @brief 终止换弹同时归还控制权
/// @param  无
static void StoreEnergy_RequestLoadAbort(void);

/// @brief 收回控制权，同时让储能机构回到安全位置
/// @param  无
static void StoreEnergy_HandleSafeReturn(void);

static uint16_t LoadRequest_CalcChecksum(const LoadRequest_t *request);
static bool LoadRequest_IsValid(const LoadRequest_t *request);
static bool StoreEnergy_SendLoadRequest(uint8_t dart_num, LoadRequest_t *request_out);
static void LoadAck_Set(uint32_t seq, LoadResult_e result);
static LoadAck_t LoadAck_Get(void);
static bool LoadTask_ReceiveRequest(LoadRequest_t *request, TickType_t timeout_ticks);

/***********************************
 * 函数名: StoreEnergyTaskFunc
 * 作用:   拉簧储能任务(控制左右储能 M3508 电机转动)
 * 参数:   无
 **********************************/
void StoreEnergyTaskFunc(void *argument)
{
    // 等待事件组：云台和扳机都就绪后才开始
    xEventGroupWaitBits(g_pxStateSetEventGroupHandle, EVENT_ALL_READY, pdFALSE, pdTRUE, portMAX_DELAY);

    // 接收计数型信号量之后才可以正常
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_shoot);
    vTaskDelay(pdMS_TO_TICKS(1500));
    StoreEnergyTaskMainLoopFuc();
}

/***********************************
 * @brief 换弹到位后的通用处理：通知舵机、清积分、回零、通知储能
 * @param dart_num 当前飞镖编号
 * @param m_offset_angle 电机偏移角度
 **********************************/
static inline void LoadDart_ReturnToZero(uint8_t dart_num, float m_offset_angle, uint32_t request_seq);
static float Load3508_GetReloadOffset(uint8_t step_count);
static uint8_t LoadDart_GetServoGroup(uint8_t dart_num);

/***********************************
 * 函数名: LoadTaskFunc
 * 作用:   换弹任务（传送带电机控制以及电机目标设定）
 * 参数:   无
 **********************************/
void LoadTaskFunc(void *argument)
{
    xEventGroupWaitBits(g_pxStateSetEventGroupHandle, EVENT_ALL_READY, pdFALSE, pdTRUE, portMAX_DELAY);

    // 直接设置成0°,限位确认
    Servo_MoveAllToZero(300);

    // 任务创建
    TaskHandle_t LoadServoTaskHandle = NULL;
    xTaskCreate(LoadServoTaskFunc, "LoadServo", 64 * 4, NULL, osPriorityBelowNormal7, &LoadServoTaskHandle);

    // 初始目标保持为当前位置
    (void)LoadMotor_SubmitTarget(LOAD_MOTOR_OWNER_LOAD,
                                 LOAD_MOTOR_PRIORITY_LOAD,
                                 Motor_GetTotalAngle(RM_3508_GRIPPER),
                                 LOAD_DEADZONE_TIMEOUT_MS);

    LoadMotorTaskMainLoopFunc();
}

/***********************************
 * 函数名: LoadServoTaskFunc
 * 作用:   换弹任务（舵机释放）
 * 参数:   无
 **********************************/
void LoadServoTaskFunc(void *argument)
{
    xEventGroupWaitBits(g_pxStateSetEventGroupHandle, EVENT_ALL_READY, pdFALSE, pdTRUE, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(1)); // 最开始放一个Tick

    // 在这里驱动换弹舵机运动
    LoadServoTaskMainLoopFunc();
}

/************************************
 * 函数名: RefereeTaskFunc
 * 作用: 解算裁判系统通信数据
 * 参数: 无
 * TODO: 确认是否可以正常解算数据
 ***********************************/
void RefereeTaskFunc(void *argument)
{
    (void)argument;
    RefereeTaskMainLoopFunc();
}

/************************************
 * 函数名：StateSetTaskFunc
 * 作用：  飞镖状态设置
 * 参数：  无
 * TODO:  等待链接上位机和测试无上位机版本
 ***********************************/
void StateSetTaskFunc(void *argument)
{
    (void)argument;
    StateSetTaskMainLoopFunc();
}
