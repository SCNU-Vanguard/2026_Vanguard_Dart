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
 ***********************************************************************************************************************/
#include "UserTask.h"
#include "IA6B.h"
#include "motor_algrothim.h"

// 任务：
// 状态设置任务(云台Yaw轴角度和扳机位置调节任务)
// 换弹任务
// 存储拉簧动能任务
// yaw调整任务

/************************全局或静态作用域*********************/
extern MotorManager_t MotorManager;
static float MotorData = 0.0f;
static float gripper_offset;
static float trigger_offset;
static float GripperTarget = 0.0f;
static const ServoRegistryItem_t g_dart_servo_registry[3] = {
    {.id = DART_SERVO_ID1, .raw_zero = DART_SERVO1_RAW_ZERO, .raw_release = DART_SERVO1_RAW_RELEASE},
    {.id = DART_SERVO_ID2, .raw_zero = DART_SERVO2_RAW_ZERO, .raw_release = DART_SERVO2_RAW_RELEASE},
    {.id = DART_SERVO_ID3, .raw_zero = DART_SERVO3_RAW_ZERO, .raw_release = DART_SERVO3_RAW_RELEASE}};
volatile uint8_t g_DiagYaw4310EnableOk = 0U;

static MotorTrapPosProfile_t g_LoadTrapProfile = {0};
static uint32_t g_LoadTrapCntLast = 0U;

static void Servo_RegisterDartProfiles(void)
{
    ServoRegistry_Reset();
    (void)ServoRegistry_RegisterBatch(g_dart_servo_registry, 3);
}

static void Servo_MoveAllToZero(uint16_t time_ms)
{
    uint8_t ids[3];
    uint16_t zeros[3];
    for (uint8_t i = 0; i < 3; i++)
    {
        ids[i] = g_dart_servo_registry[i].id;
        zeros[i] = g_dart_servo_registry[i].raw_zero;
    }
    ServoControlMulti(3, ids, zeros, time_ms);
}

/* 3508 目标互斥量 */
static StaticSemaphore_t g_x3508MtxBuf;
static SemaphoreHandle_t g_x3508Mtx = NULL;

/* 2006 目标互斥量 */
static StaticSemaphore_t g_x2006MtxBuf;
static SemaphoreHandle_t g_x2006Mtx = NULL;

/* 3508 独立控制任务 */
static volatile bool g_b3508CtrlEnabled = false;
static StackType_t g_3508CtrlStack[128];
static StaticTask_t g_3508CtrlTCB;
static TaskHandle_t g_3508CtrlHandle = NULL;

/* 2006 独立控制任务 */
static float g_TriggerTarget = 0.0f;
static volatile bool g_b2006CtrlEnabled = false;
static StackType_t g_2006CtrlStack[128];
static StaticTask_t g_2006CtrlTCB;
static TaskHandle_t g_2006CtrlHandle = NULL;

/* =========================================================================================================== */
/* =========================================================================================================== */
/* =========================================================================================================== */

/* 线程安全的目标值访问接口 */
static inline void Set3508Target(float target)
{
    xSemaphoreTake(g_x3508Mtx, portMAX_DELAY);
    GripperTarget = target;
    xSemaphoreGive(g_x3508Mtx);
}

/***********************************
 * 函数名: Get3508Target
 * 作用: 自己看函数名
 **********************************/
static inline float Get3508Target(void)
{
    xSemaphoreTake(g_x3508Mtx, portMAX_DELAY);
    float t = GripperTarget;
    xSemaphoreGive(g_x3508Mtx);
    return t;
}

/***********************************
 * 函数名: Set2006Target
 * 作用: 自己看函数名
 **********************************/
static inline void Set2006Target(float target)
{
    xSemaphoreTake(g_x2006Mtx, portMAX_DELAY);
    g_TriggerTarget = target;
    xSemaphoreGive(g_x2006Mtx);
}

/***********************************
 * 函数名: Get2006Target
 * 作用: 自己看函数名
 **********************************/
static inline float Get2006Target(void)
{
    xSemaphoreTake(g_x2006Mtx, portMAX_DELAY);
    float t = g_TriggerTarget;
    xSemaphoreGive(g_x2006Mtx);
    return t;
}

static volatile uint32_t g_LoadDeadzoneTimeoutCount = 0U;
static volatile float g_LoadTrapCmdPosData = 0.0f;
static volatile float g_LoadTrapCmdVelData = 0.0f;
static volatile float g_LoadTrapCmdAccData = 0.0f;
static volatile float g_LoadTrapDtData = 0.0f;

/***********************************
 * 函数名: LoadMotor_GetRawAngle
 * 作用: 自己看函数名
 **********************************/
static inline float LoadMotor_GetRawAngle(void)
{
    return MotorManager.MotorList[RM_3508_GRIPPER - 1].motor_data.solved_data[3];
}

/***********************************
 * 函数名: LoadMotor_CalcMoveTimeoutMs
 * 作用: 自己看函数名
 **********************************/
static inline uint32_t LoadMotor_CalcMoveTimeoutMs(float start_pos_deg, float target_pos_deg)
{
    float dist = fabsf(target_pos_deg - start_pos_deg);
    float vmax = LOAD_TASK_TRAP_VMAX_DEG_S;
    float amax = LOAD_TASK_TRAP_AMAX_DEG_S2;
    float t_s = 0.0f;

    if (vmax <= 1e-3f || amax <= 1e-3f)
    {
        return LOAD_DEADZONE_TIMEOUT_MS;
    }

    {
        float t_acc = vmax / amax;
        float d_acc = 0.5f * amax * t_acc * t_acc;

        if (dist <= 2.0f * d_acc)
        {
            // 三角速度曲线
            t_s = 2.0f * sqrtf(dist / amax);
        }
        else
        {
            // 梯形速度曲线
            t_s = 2.0f * t_acc + (dist - 2.0f * d_acc) / vmax;
        }
    }

    // 估算时间加安全裕量，避免在长行程时被2.5s固定超时提前放行
    uint32_t est_ms = (uint32_t)(t_s * 1000.0f * 1.6f + 200.0f);
    if (est_ms < LOAD_DEADZONE_TIMEOUT_MS)
    {
        est_ms = LOAD_DEADZONE_TIMEOUT_MS;
    }
    if (est_ms > 12000U)
    {
        est_ms = 12000U;
    }
    return est_ms;
}

/***********************************
 * 函数名: IsInDeadzoneTimedF
 * 作用: 自己看函数名
 **********************************/
static inline bool IsInDeadzoneTimedF(float value, float target, float zone, uint32_t timeout_ms, DeadzoneTimer_t *timer)
{
    if (timer == NULL)
    {
        return false;
    }

    if (!isfinite(value) || !isfinite(target) || !isfinite(zone))
    {
        timer->active = false;
        return false;
    }

    float abs_zone = fabsf(zone);
    if (fabsf(value - target) <= abs_zone)
    {
        timer->active = false;
        timer->last_target = target;
        timer->last_zone = abs_zone;
        return true;
    }

    uint32_t now_tick = HAL_GetTick();
    bool target_changed = (!timer->active) ||
                          (fabsf(target - timer->last_target) > (abs_zone + 1e-3f)) ||
                          (fabsf(abs_zone - timer->last_zone) > 1e-3f);
    if (target_changed)
    {
        timer->start_tick = now_tick;
        timer->last_target = target;
        timer->last_zone = abs_zone;
        timer->active = true;
        return false;
    }

    if ((uint32_t)(now_tick - timer->start_tick) >= timeout_ms)
    {
        // 3508策略：超时后按到位处理，避免流程卡死
        g_LoadDeadzoneTimeoutCount++;
        timer->active = false;
        timer->last_target = target;
        timer->last_zone = abs_zone;
        return true;
    }

    return false;
}

/***********************************
 * 函数名: LoadMotor_SetFinalTarget
 * 作用: 自己看函数名
 **********************************/
static inline void LoadMotor_SetFinalTarget(float new_target_pos_deg)
{
    Set3508Target(new_target_pos_deg);
#if LOAD_TASK_TRAP_RESET_ON_TARGET_CHANGE
    MotorData = LoadMotor_GetRawAngle();
    Motor_TrapPos_Resync(&g_LoadTrapProfile, MotorData);
#endif
}

/***********************************
 * 函数名: LoadMotor_RunTrapTo
 * 作用: 自己看函数名
 **********************************/
static inline void LoadMotor_RunTrapTo(float target_pos_deg)
{
    g_LoadTrapProfile.target_pos = target_pos_deg;
    float dt_s = DWT_GetDeltaT(&g_LoadTrapCntLast);
    if (!isfinite(dt_s) || dt_s <= 0.0f)
    {
        dt_s = 0.001f;
    }
    else if (dt_s > 0.02f)
    {
        dt_s = 0.02f;
    }
    g_LoadTrapDtData = dt_s;
    float cmd_pos = Motor_TrapPos_Update(&g_LoadTrapProfile, dt_s);
    g_LoadTrapCmdPosData = cmd_pos;
    g_LoadTrapCmdVelData = g_LoadTrapProfile.cmd_vel;
    g_LoadTrapCmdAccData = g_LoadTrapProfile.cmd_acc;
    RmMotorPID_Calc(RM_3508_GRIPPER, cmd_pos);
}

/***********************************
 * 函数名: Motor3508CtrlTask
 * 作用:   3508电机独立控制任务，固定2ms周期
 *         只负责梯形规划 + PID，不做任何状态逻辑
 **********************************/
static void Motor3508CtrlTask(void *argument)
{
    MotorData = LoadMotor_GetRawAngle();
    Motor_TrapPos_Init(&g_LoadTrapProfile, MotorData,
                       LOAD_TASK_TRAP_VMAX_DEG_S, LOAD_TASK_TRAP_AMAX_DEG_S2);
    g_LoadTrapProfile.brake_gain = LOAD_TASK_TRAP_BRAKE_GAIN;
    g_LoadTrapProfile.arrive_zone = LOAD_TASK_TRAP_ARRIVE_ZONE;
    g_LoadTrapProfile.decel_zone = LOAD_TASK_TRAP_DECEL_ZONE;
#if LOAD_TASK_TRAP_DISABLE_JERK
    Motor_TrapPos_SetJerk(&g_LoadTrapProfile, 0.0f);
#else
    Motor_TrapPos_SetJerk(&g_LoadTrapProfile,
                          LOAD_TASK_TRAP_AMAX_DEG_S2 * LOAD_TASK_TRAP_JERK_FACTOR);
#endif
    LoadMotor_SetFinalTarget(MotorData);
    DWT_GetDeltaT(&g_LoadTrapCntLast); /* 初始化时间戳 */

    TickType_t xLastWake = xTaskGetTickCount();
    for (;;)
    {
        if (g_b3508CtrlEnabled)
        {
            LoadMotor_RunTrapTo(Get3508Target());
        }
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(2));
    }
}

/***********************************
 * 函数名: Motor2006CtrlTask
 * 作用:   2006扳机电机独立控制任务，固定2ms周期
 *         只负责PID位置跟踪
 **********************************/
static void Motor2006CtrlTask(void *argument)
{
    TickType_t xLastWake = xTaskGetTickCount();
    for (;;)
    {
        if (g_b2006CtrlEnabled)
        {
            RmMotorPID_Calc(RM_2006_TRIGGER, Get2006Target());
        }
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(2));
    }
}
/* =========================================================================================================== */
/* =========================================================================================================== */
/* =========================================================================================================== */

// Store <-> Load 双向同步信号量
static StaticSemaphore_t g_xStore2LoadSemBuffer;
static SemaphoreHandle_t g_xStore2LoadSemHandle; // Store通知Load开始
static StaticSemaphore_t g_xLoad2StoreSemBuffer;
static SemaphoreHandle_t g_xLoad2StoreSemHandle; // Load通知Store完成

// Store <-> Shoot 双向同步信号量
static StaticSemaphore_t g_xStore2ShootSemBuffer;
static SemaphoreHandle_t g_xStore2ShootSemHandle; // Store通知Shoot开始
static StaticSemaphore_t g_xShoot2StoreSemBuffer;
static SemaphoreHandle_t g_xShoot2StoreSemHandle; // Shoot通知Store完成

// Store -> Load: 3519到位通知信号量（换弹时舵机需等3519到位才释放）
static StaticSemaphore_t g_x3519ArrivedSemBuffer;
static SemaphoreHandle_t g_x3519ArrivedSemHandle;

// 储能任务信号量
static StaticSemaphore_t g_xStoreSemaphore;
SemaphoreHandle_t g_xStoreSemaphoreHandle;

// 储能任务消息
static StreamBufferHandle_t xLoadStreamBuf;

// 换弹结构电机队列
static uint8_t g_ucReloadQueueStorage[4 * sizeof(uint8_t)];
static StaticQueue_t g_xReloadQueue;
static QueueHandle_t g_xLoad3508QueueHandler;

// StateSet任务的事件组
static StaticEventGroup_t g_pxStateSetEventGroupBuffer;
static EventGroupHandle_t g_pxStateSetEventGroupHandeler;
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
    // ; // 这个地方有一个回调,需要进行数据读取
    UART_SetProtocolType(BSP_UART6, PROTOCOL_IBUS);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    HAL_TIM_Base_Start(&htim8);
    HAL_Delay(100);
}

/// @brief 创建RTOS的通信量
/// @param  无
void RTOS_ModuleInit(void)
{
    // 电机目标值互斥量
    g_x3508Mtx = xSemaphoreCreateMutexStatic(&g_x3508MtxBuf);
    g_x2006Mtx = xSemaphoreCreateMutexStatic(&g_x2006MtxBuf);

    // QUEUE（队列）：1.融合到云台调节当中，保证云台调节正常<队列集>; 2.融入到换弹结构当中,确保换弹结构正常
    g_xLoad3508QueueHandler = xQueueCreateStatic(4, sizeof(uint8_t), g_ucReloadQueueStorage, &g_xReloadQueue);

    // SEMAPHORE（信号量）：信号量与dart_num共同决定飞镖换弹的状态
    g_xStoreSemaphoreHandle = xSemaphoreCreateCountingStatic(5, sizeof(uint8_t), &g_xStoreSemaphore);

    // 二值信号量：用于任务间双向同步（创建后初始状态为"空"）
    g_xStore2LoadSemHandle = xSemaphoreCreateBinaryStatic(&g_xStore2LoadSemBuffer);
    g_xLoad2StoreSemHandle = xSemaphoreCreateBinaryStatic(&g_xLoad2StoreSemBuffer);
    g_xStore2ShootSemHandle = xSemaphoreCreateBinaryStatic(&g_xStore2ShootSemBuffer);
    g_xShoot2StoreSemHandle = xSemaphoreCreateBinaryStatic(&g_xShoot2StoreSemBuffer);
    g_x3519ArrivedSemHandle = xSemaphoreCreateBinaryStatic(&g_x3519ArrivedSemBuffer);

    // 使用stream流传输,也可以改成使用Message传输
    xLoadStreamBuf = xStreamBufferCreate(1, 1); // 1字节触发

    // EVENTGROUPS（事件组）：用于确认云台电机和射程电机正常设置,只有这两个正常设置之后才可以进行下面的流程
    g_pxStateSetEventGroupHandeler = xEventGroupCreateStatic(&g_pxStateSetEventGroupBuffer);
}

/***********************************************TASK******************************************************/

// 拉簧储能任务
osThreadId_t StoreEnergyTaskHandle;
const osThreadAttr_t StoreEnergyTask_attributes = {
    .name = "StoreEnergyTask",
    .stack_size = 384 * 4,
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

void StoreEnergyTaskFunc(void *argument);
void LoadTaskFunc(void *argument);
void LoadMotorTaskFunc(void *argument);
void StateSetTaskFunc(void *argument);

/***********************************
 * 函数名: TaskInitFunc
 * 作用:   任务初始化创建
 * 参数:   无
 **********************************/
void TaskInitFunc(void)
{
    RTOS_ModuleInit();
    ControlState_Init();

    // 3508 独立控制任务（高优先级，固定2ms周期）
    g_3508CtrlHandle = xTaskCreateStatic(Motor3508CtrlTask, "M3508Ctrl",
                                         128, NULL,
                                         osPriorityAboveNormal,
                                         g_3508CtrlStack, &g_3508CtrlTCB);

    // 2006 独立控制任务（高优先级，固定2ms周期）
    g_2006CtrlHandle = xTaskCreateStatic(Motor2006CtrlTask, "M2006Ctrl",
                                         128, NULL,
                                         osPriorityAboveNormal,
                                         g_2006CtrlStack, &g_2006CtrlTCB);

    // 任务初始化
    LoadTaskHandle = osThreadNew(LoadTaskFunc, NULL, &LoadTask_attributes);
    StoreEnergyTaskHandle = osThreadNew(StoreEnergyTaskFunc, NULL, &StoreEnergyTask_attributes);
    StateSetTaskHandle = osThreadNew(StateSetTaskFunc, NULL, &StateSetTask_attributes);

    // 创建ControlState的IbusTask和ControlTask
    ControlState_CreateTasks();
}

/***********************************
 * 函数名: StoreEnergyTaskFunc
 * 作用:   拉簧储能任务(控制左右3519电机转动)
 * 参数:   无
 **********************************/
// 每发飞镖对应的 YAW 和 Trigger 预设位置映射表
// 索引: [0]=Dart4(第1发), [1]=Dart3(第2发), [2]=Dart2(第3发), [3]=Dart1(第4发)
static const float g_SetupYaw[4] = {
    SETUP_YAW_DART4, SETUP_YAW_DART3, SETUP_YAW_DART2, SETUP_YAW_DART1};
static const float g_SetupTrigger[4] = {
    SETUP_TRIGGER_DART4, SETUP_TRIGGER_DART3, SETUP_TRIGGER_DART2, SETUP_TRIGGER_DART1};

void StoreEnergyTaskFunc(void *argument)
{
    // 等待事件组：云台和扳机都就绪后才开始
    xEventGroupWaitBits(g_pxStateSetEventGroupHandeler, EVENT_ALL_READY, pdFALSE, pdTRUE, portMAX_DELAY);

    // 接收计数型信号量之后才可以正常
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_shoot);
    vTaskDelay(pdMS_TO_TICKS(1500));
    uint8_t StoreState = 0x00;
    uint8_t Dart = 4;
    vTaskDelay(pdMS_TO_TICKS(POWER_ON_DELAY_MS));
    float left_pos = 0.0f, right_pos = 0.0f;
    while (1)
    {
        // 获取电机控制权（与ControlTask互斥）
        xSemaphoreTake(g_xMotorCtrlSemHandle, portMAX_DELAY);

        // 检查是否被手动覆盖，如果是则释放控制权并等待
        if (ControlState_IsManualOverride())
        {
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        switch (StoreState)
        {
        case 0x00:
        {
            // === SetupLoc: 根据 Dart 编号驱动 YAW 和 Trigger 到预设位置 ===
            uint8_t setup_idx = 4 - Dart; // Dart4->idx0, Dart3->idx1, ...
            float target_yaw = g_SetupYaw[setup_idx];
            float target_trigger = g_SetupTrigger[setup_idx];

            // 驱动 4310 YAW 到目标位置
            // DmMotorSendCfg(DM_4310_YAW, target_yaw, 0.0f, DM_MIT);
            // 驱动 2006 Trigger 到目标位置
            Set2006Target(target_trigger + trigger_offset);

            // [4310已移除] 仅等待Trigger到位
            {
                DeadzoneState_t trig_setup_deadzone = {0};
                while (!IsInDeadzoneF(Motor_GetTotalAngle(RM_2006_TRIGGER),
                                      target_trigger + trigger_offset,
                                      MOTOR_DEAD_ZONE, &trig_setup_deadzone, true))
                {
                    if (ControlState_IsManualOverride())
                    {
                        xSemaphoreGive(g_xMotorCtrlSemHandle);
                        while (ControlState_IsManualOverride())
                        {
                            vTaskDelay(pdMS_TO_TICKS(50));
                        }
                        xSemaphoreTake(g_xMotorCtrlSemHandle, portMAX_DELAY);
                        Set2006Target(target_trigger + trigger_offset);
                    }
                    vTaskDelay(pdMS_TO_TICKS(2));
                }
            }

            // === 储能流程开始 ===
            // 清除上一轮可能残留的3519到位信号
            xSemaphoreTake(g_x3519ArrivedSemHandle, 0);

            // 发射滑台下移到达换弹位置
            DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreLoad, StoreSpeed, DM_LOCATION_SPEED);
            DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreLoad, StoreSpeed, DM_LOCATION_SPEED);
            uint32_t load_reissue_tick = HAL_GetTick();
            DeadzoneState_t load_left_deadzone = {0};
            DeadzoneState_t load_right_deadzone = {0};

            // 先通知Load开始
            xStreamBufferSend(xLoadStreamBuf, (const uint8_t *)(&Dart), 1, 0);
            xSemaphoreGive(g_xStore2LoadSemHandle);

            while (1)
            {
                // 手动覆盖检查：中途可被遥控器打断
                if (ControlState_IsManualOverride())
                {
                    xSemaphoreGive(g_xMotorCtrlSemHandle);
                    // 等待手动模式结束
                    while (ControlState_IsManualOverride())
                    {
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                    // 重新获取控制权并重发目标
                    xSemaphoreTake(g_xMotorCtrlSemHandle, portMAX_DELAY);
                    DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreLoad, StoreSpeed, DM_LOCATION_SPEED);
                    DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreLoad, StoreSpeed, DM_LOCATION_SPEED);
                }

                DM_Motor_RefreshData(DM_3519_STRENTH_LEFT);
                DM_Motor_RefreshData(DM_3519_STRENTH_RIGHT);
                left_pos = Motor_GetTotalAngle(DM_3519_STRENTH_LEFT);
                right_pos = Motor_GetTotalAngle(DM_3519_STRENTH_RIGHT);
                if ((uint32_t)(HAL_GetTick() - load_reissue_tick) >= 20U)
                {
                    DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreLoad, StoreSpeed, DM_LOCATION_SPEED);
                    DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreLoad, StoreSpeed, DM_LOCATION_SPEED);
                    load_reissue_tick = HAL_GetTick();
                }
                if (IsInDeadzoneF(left_pos, LeftStoreLoad, MOTOR_DEAD_ZONE, &load_left_deadzone, false) &&
                    IsInDeadzoneF(right_pos, RightStoreLoad, MOTOR_DEAD_ZONE, &load_right_deadzone, false))
                {
                    break;
                }
            }

            // 3519到位，通知LoadTask可以执行舵机释放
            xSemaphoreGive(g_x3519ArrivedSemHandle);
            // 发射前互锁：等待Load侧完成
            xSemaphoreTake(g_xLoad2StoreSemHandle, portMAX_DELAY);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_store);
            vTaskDelay(pdMS_TO_TICKS(1500));
            StoreState++;
            // 释放控制权（case结束）
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            break;
        }

        case 0x01:
        {
            // 发射滑台叉上移
            DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreTop, StoreSpeed, DM_LOCATION_SPEED);
            DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreTop, StoreSpeed, DM_LOCATION_SPEED);
            uint32_t top_reissue_tick = HAL_GetTick();
            DeadzoneState_t top_left_deadzone = {0};
            DeadzoneState_t top_right_deadzone = {0};

            // 等待电机到达目标位置
            while (1)
            {
                // 手动覆盖检查：中途可被遥控器打断
                if (ControlState_IsManualOverride())
                {
                    xSemaphoreGive(g_xMotorCtrlSemHandle);
                    while (ControlState_IsManualOverride())
                    {
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                    xSemaphoreTake(g_xMotorCtrlSemHandle, portMAX_DELAY);
                    DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreTop, StoreSpeed, DM_LOCATION_SPEED);
                    DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreTop, StoreSpeed, DM_LOCATION_SPEED);
                }

                DM_Motor_RefreshData(DM_3519_STRENTH_LEFT);
                DM_Motor_RefreshData(DM_3519_STRENTH_RIGHT);
                left_pos = Motor_GetTotalAngle(DM_3519_STRENTH_LEFT);
                right_pos = Motor_GetTotalAngle(DM_3519_STRENTH_RIGHT);
                if ((uint32_t)(HAL_GetTick() - top_reissue_tick) >= 20U)
                {
                    DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreTop, StoreSpeed, DM_LOCATION_SPEED);
                    DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreTop, StoreSpeed, DM_LOCATION_SPEED);
                    top_reissue_tick = HAL_GetTick();
                }

                if (IsInDeadzoneF(left_pos, LeftStoreTop, MOTOR_DEAD_ZONE, &top_left_deadzone, false) &&
                    IsInDeadzoneF(right_pos, RightStoreTop, MOTOR_DEAD_ZONE, &top_right_deadzone, false))
                {
                    break;
                }
            }
            // 发射
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_shoot);
            vTaskDelay(pdMS_TO_TICKS(1500));
            StoreState = 0x00;
            Dart--;

            // 释放控制权，开启调试窗口
            ControlState_StartDebugWindow(DEBUG_WINDOW_MS);
            // 清除中途手动覆盖可能产生的残留信号（此时仍持有motor sem，安全）
            xSemaphoreTake(g_xDebugFinishedSemHandle, 0);
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            // 许可调试，等待调试结束
            xSemaphoreTake(g_xDebugFinishedSemHandle, portMAX_DELAY);
            break;
        }
        default:
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            break;
        }

        if (Dart == 0)
        {
            vTaskDelay(750);
            HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
            // 重新获取控制权进行收尾
            xSemaphoreTake(g_xMotorCtrlSemHandle, portMAX_DELAY);
            // 3508回到原始位置（offset_ecd_angle）
            float home_pos = MotorManager.MotorList[RM_3508_GRIPPER - 1].motor_data.offset_ecd_angle;
            LoadMotor_SetFinalTarget(home_pos);
            DeadzoneTimer_t home_timer = {0};
            uint32_t home_timeout_ms = LoadMotor_CalcMoveTimeoutMs(LoadMotor_GetRawAngle(), home_pos);
            while (!IsInDeadzoneTimedF(LoadMotor_GetRawAngle(), home_pos, MOTOR_DEAD_ZONE, home_timeout_ms, &home_timer))
            {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
            g_b3508CtrlEnabled = false;
            // g_b2006CtrlEnabled = false;

            // DmMotorSendCfg(DM_4310_YAW, 0.0f, 0.0f, DM_MIT);
            DM_Motor_Disable(&MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1]);
            DM_Motor_Disable(&MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1]);
            // DmMotorSendCfg(DM_4310_YAW, 0.0f, 0.0f, DM_MIT);
            // DM_Motor_Disable(&MotorManager.MotorList[DM_4310_YAW - 1]);
            Servo_MoveAllToZero(300);
            Set2006Target(0.0f + trigger_offset);
            g_b2006CtrlEnabled = true;
            DeadzoneState_t trigger_preset_deadzone = {0};
            while (!IsInDeadzoneF(Motor_GetTotalAngle(RM_2006_TRIGGER), 0.0f + trigger_offset, MOTOR_DEAD_ZONE, &trigger_preset_deadzone, true))
            {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            vTaskSuspend(StoreEnergyTaskHandle);
        }
    }
}

/***********************************
 * @brief 换弹到位后的通用处理：通知舵机、清积分、回零、通知储能
 * @param dart_num 当前飞镖编号
 * @param m_offset_angle 电机偏移角度
 **********************************/
static inline void LoadDart_ReturnToZero(uint8_t dart_num, float m_offset_angle)
{
    DeadzoneTimer_t deadzone_timer = {0};
    float preset_target = PresetLoc + m_offset_angle;
    uint32_t preset_timeout_ms = LoadMotor_CalcMoveTimeoutMs(LoadMotor_GetRawAngle(), preset_target);
    // 等待3519储能电机到位后再释放舵机，避免3508到位但3519未到位就动舵机
    xSemaphoreTake(g_x3519ArrivedSemHandle, portMAX_DELAY);
    xQueueSend(g_xLoad3508QueueHandler, (const void *)&dart_num, 0);
    vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS));

    DmMotorSendCfg(DM_3519_STRENTH_LEFT, (float)(LeftStoreLoad + 350.0f), StoreSpeed, DM_LOCATION_SPEED);
    DmMotorSendCfg(DM_3519_STRENTH_RIGHT, (float)(RightStoreLoad - 350.0f), StoreSpeed, DM_LOCATION_SPEED);
    uint32_t bottom_reissue_tick = HAL_GetTick();
    uint32_t bottom_wait_start_tick = HAL_GetTick();
    DeadzoneState_t bottom_left_deadzone_1 = {0};
    DeadzoneState_t bottom_right_deadzone_1 = {0};
    while (1)
    {
        DM_Motor_RefreshData(DM_3519_STRENTH_LEFT);
        DM_Motor_RefreshData(DM_3519_STRENTH_RIGHT);
        float left_pos = Motor_GetTotalAngle(DM_3519_STRENTH_LEFT);
        float right_pos = Motor_GetTotalAngle(DM_3519_STRENTH_RIGHT);

        if ((uint32_t)(HAL_GetTick() - bottom_reissue_tick) >= 20U)
        {
            DmMotorSendCfg(DM_3519_STRENTH_LEFT, (float)(LeftStoreLoad + 350.0f), StoreSpeed, DM_LOCATION_SPEED);
            DmMotorSendCfg(DM_3519_STRENTH_RIGHT, (float)(RightStoreLoad - 350.0f), StoreSpeed, DM_LOCATION_SPEED);
            bottom_reissue_tick = HAL_GetTick();
        }

        if (IsInDeadzoneF(left_pos, (float)(LeftStoreLoad + 350.0f), MOTOR_DEAD_ZONE, &bottom_left_deadzone_1, false) &&
            IsInDeadzoneF(right_pos, (float)(RightStoreLoad - 350.0f), MOTOR_DEAD_ZONE, &bottom_right_deadzone_1, false))
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    CASCADE_PID_Clear_Integral(&MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid);
    LoadMotor_SetFinalTarget(preset_target);
    while (!IsInDeadzoneTimedF(LoadMotor_GetRawAngle(), preset_target, MOTOR_DEAD_ZONE, preset_timeout_ms, &deadzone_timer))
    {
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    // 舵机动作完成后，先执行3508回零，再让3519后退到底
    DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreBottom, StoreSpeed, DM_LOCATION_SPEED);
    DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreBottom, StoreSpeed, DM_LOCATION_SPEED);
    bottom_reissue_tick = HAL_GetTick();
    bottom_wait_start_tick = HAL_GetTick();
    DeadzoneState_t bottom_left_deadzone_2 = {0};
    DeadzoneState_t bottom_right_deadzone_2 = {0};
    while (1)
    {
        DM_Motor_RefreshData(DM_3519_STRENTH_LEFT);
        DM_Motor_RefreshData(DM_3519_STRENTH_RIGHT);
        float left_pos = Motor_GetTotalAngle(DM_3519_STRENTH_LEFT);
        float right_pos = Motor_GetTotalAngle(DM_3519_STRENTH_RIGHT);

        if ((uint32_t)(HAL_GetTick() - bottom_reissue_tick) >= 20U)
        {
            DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreBottom, StoreSpeed, DM_LOCATION_SPEED);
            DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreBottom, StoreSpeed, DM_LOCATION_SPEED);
            bottom_reissue_tick = HAL_GetTick();
        }

        if (IsInDeadzoneF(left_pos, LeftStoreBottom, MOTOR_DEAD_ZONE, &bottom_left_deadzone_2, false) &&
            IsInDeadzoneF(right_pos, RightStoreBottom, MOTOR_DEAD_ZONE, &bottom_right_deadzone_2, false))
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    xSemaphoreGive(g_xLoad2StoreSemHandle);
    if (dart_num == 1)
        vTaskDelay(pdMS_TO_TICKS(200));
}

/***********************************
 * 函数名: LoadTaskFunc
 * 作用:   换弹任务（传送带电机控制以及电机目标设定）
 * 参数:   无
 **********************************/
void LoadTaskFunc(void *argument)
{
    xEventGroupWaitBits(g_pxStateSetEventGroupHandeler, EVENT_ALL_READY, pdFALSE, pdTRUE, portMAX_DELAY);

    bool MutexTake = false;
    uint8_t dart_num = 0x0000;

    // 直接设置成0°,限位确认
    Servo_MoveAllToZero(300);

    // 获取电机偏移角度，用于目标值补偿
    float offset_angle = MotorManager.MotorList[RM_3508_GRIPPER - 1].motor_data.offset_ecd_angle;

    // 任务创建
    TaskHandle_t Load3508TaskHandle = NULL;
    xTaskCreate(LoadMotorTaskFunc, "LoadMotor", 64 * 4, NULL, osPriorityBelowNormal7, &Load3508TaskHandle);

    // 初始目标保持为当前位置
    LoadMotor_SetFinalTarget(LoadMotor_GetRawAngle());
    while (1)
    {
        xSemaphoreTake(g_xStore2LoadSemHandle, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(500)); // 本来是175的,但是这个时候换弹过快
        xStreamBufferReceive(xLoadStreamBuf, &dart_num, 1, portMAX_DELAY);
        switch (dart_num)
        {
        case 4:
            LoadMotor_SetFinalTarget(PresetLoc + offset_angle);
            MutexTake = false;
            xSemaphoreGive(g_xLoad2StoreSemHandle);
            break;
        case 3:
            LoadMotor_SetFinalTarget(FirstServoLoc + offset_angle);
            MutexTake = true;
            break;
        case 2:
            LoadMotor_SetFinalTarget(SecondServoLoc + offset_angle);
            MutexTake = true;
            break;
        case 1:
            LoadMotor_SetFinalTarget(ThirdServoLoc + offset_angle);
            MutexTake = true;
            break;
        default:
            break;
        }
        DeadzoneTimer_t reach_timer = {0};
        float cur_target = Get3508Target();
        uint32_t reach_timeout_ms = LoadMotor_CalcMoveTimeoutMs(
            LoadMotor_GetRawAngle(), cur_target);
        while (MutexTake)
        {
            MotorData = LoadMotor_GetRawAngle();
            if (IsInDeadzoneTimedF(MotorData, cur_target,
                                   MOTOR_DEAD_ZONE, reach_timeout_ms,
                                   &reach_timer))
            {
                MutexTake = false;
                LoadDart_ReturnToZero(dart_num, offset_angle);
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

/***********************************
 * 函数名: LoadMotorTaskFunc
 * 作用:   换弹任务（舵机释放）
 * 参数:   无
 **********************************/
void LoadMotorTaskFunc(void *argument)
{
    xEventGroupWaitBits(g_pxStateSetEventGroupHandeler, EVENT_ALL_READY, pdFALSE, pdTRUE, portMAX_DELAY);

    uint8_t fQueueDartNum = 0;    // 目标为0最初
    vTaskDelay(pdMS_TO_TICKS(1)); // 最开始放一个Tick
    // 在这里驱动换弹的3508电机运动
    while (1)
    {
        xQueueReceive(g_xLoad3508QueueHandler, &fQueueDartNum, portMAX_DELAY);
        if (fQueueDartNum >= 1 && fQueueDartNum <= 3)
        {
            const ServoRegistryItem_t *item = ServoRegistry_Find(fQueueDartNum);
            if (item != NULL)
            {
                ServoControlPos(item->id, item->raw_release, SERVO_MOVE_TIME_MS);
            }
            vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS));
        }
    }
}

/************************************
 * 函数名：StateSetTaskFunc
 * 作用：  飞镖状态设置
 * 参数：  无
 * // TODO：  等待链接上位机和测试无上位机版本
 ***********************************/
void StateSetTaskFunc(void *argument)
{
    static const uint8_t dm_enable_frame[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};

    // 接收上位机 / 遥控器 传递的数据
    vTaskDelay(1);
    while (!DM_MotorEnable(DM_3519_STRENTH_LEFT))
    {
        MotorTypeDef *motor = &MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1];
        vTaskSuspendAll();
        (void)Motor_GetHAL_Fast()->can_send(&(motor->g_TxHeader), (uint8_t *)dm_enable_frame);
        xTaskResumeAll();
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    vTaskDelay(1);
    while (!DM_MotorEnable(DM_3519_STRENTH_RIGHT))
    {
        MotorTypeDef *motor = &MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1];
        (void)Motor_GetHAL_Fast()->can_send(&(motor->g_TxHeader), (uint8_t *)dm_enable_frame);
        vTaskDelay(pdMS_TO_TICKS(3));
    }
    vTaskDelay(1);
    // preset the target at the base
    float degree = 0.0f;
    gripper_offset = MotorManager.MotorList[RM_3508_GRIPPER - 1].motor_data.offset_ecd_angle;
    trigger_offset = MotorManager.MotorList[RM_2006_TRIGGER - 1].motor_data.offset_ecd_angle;
    float gripper_preset = PresetLoc + gripper_offset;
    float preseting_distance = (0.0f) + trigger_offset; // pay attention to this params, its unit is degree not rad!!!!
    float preseting_yaw = 0.0f;                         // this param is limited at (-160.0f, 160.0f)
    while (1)
    {
        // TODO这个地方一直等待云台手发命令之后进行调整位置
        // TODO默认击打前哨战
        // TODO给每一个重要的电机都应该设置一个单独的控制任务
        // TODO力矩异常检测
        // TODO增加裁判系统通信

        // 2006 扳机定位：设目标，使能控制任务，等到位
        Set2006Target(preseting_distance);
        g_b2006CtrlEnabled = true;
        DeadzoneState_t trigger_preset_deadzone = {0};
        while (!IsInDeadzoneF(Motor_GetTotalAngle(RM_2006_TRIGGER), preseting_distance, MOTOR_DEAD_ZONE, &trigger_preset_deadzone, true))
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // 4310 Yaw 归位
        while (!DM_MotorEnable(DM_4310_YAW))
        {
            MotorTypeDef *motor = &MotorManager.MotorList[DM_4310_YAW - 1];
            vTaskSuspendAll();
            (void)Motor_GetHAL_Fast()->can_send(&(motor->g_TxHeader), (uint8_t *)dm_enable_frame);
            xTaskResumeAll();
            vTaskDelay(pdMS_TO_TICKS(3));
        }
        g_DiagYaw4310EnableOk = 1U;
        vTaskDelay(1);
        DmMotorSendCfg(DM_4310_YAW, preseting_yaw, 0.0f, DM_MIT);
        uint32_t preset_yaw_reissue_tick = HAL_GetTick();
        DeadzoneState_t yaw_preset_deadzone = {0};
        while (degree = Motor_GetTotalAngle(DM_4310_YAW), !IsInDeadzoneF(degree, preseting_yaw, 1.0f, &yaw_preset_deadzone, false))
        {
            DM_Motor_RefreshData(DM_4310_YAW);
            if ((uint32_t)(HAL_GetTick() - preset_yaw_reissue_tick) >= 20U)
            {
                DmMotorSendCfg(DM_4310_YAW, preseting_yaw, 0.0f, DM_MIT);
                preset_yaw_reissue_tick = HAL_GetTick();
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // 3508 初始对位：设目标，使能控制任务，等到位
        g_b3508CtrlEnabled = true;
        LoadMotor_SetFinalTarget(gripper_preset);
        DeadzoneTimer_t preset_timer = {0};
        uint32_t preset_timeout_ms = LoadMotor_CalcMoveTimeoutMs(LoadMotor_GetRawAngle(), gripper_preset);
        while (!IsInDeadzoneTimedF(LoadMotor_GetRawAngle(), gripper_preset, MOTOR_DEAD_ZONE, preset_timeout_ms, &preset_timer))
        {
            vTaskDelay(pdMS_TO_TICKS(2));
        }

        // 事件组唤醒所有其他的任务
        xEventGroupSetBits(g_pxStateSetEventGroupHandeler, EVENT_ALL_READY);
        vTaskDelay(pdMS_TO_TICKS(1));

        // 最后进入阻塞态,等待遥控器解算唤醒 / 串口接收唤醒
        vTaskSuspend(NULL);
    }
}
