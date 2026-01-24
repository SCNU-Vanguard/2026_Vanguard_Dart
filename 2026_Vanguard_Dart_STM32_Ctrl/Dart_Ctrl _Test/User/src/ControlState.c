/**********************************************************
 * 文件名：ControlState.c
 * 用途：遥控器控制状态管理模块实现
 * 创建时间：2026-01-23
 *
 * 任务说明：
 * - IbusTask（高优先级，5ms周期）：解析IBUS数据，更新ControlInput
 * - ControlTask（普通优先级，10ms周期）：根据ControlInput计算目标，下发电机
 *
 * 模式说明：
 * - 正常单发模式（SWB=1 && SWC=1）：任务流程接管电机，忽略摇杆
 * - 手动调试模式（SWB=0）：遥控器直接控制电机
 * - 调试窗口模式：单发完成后20s内可手动调试
 *********************************************************/
#include "ControlState.h"
#include "IA6B.h"
#include "CanMotor.h"
#include "DM_Motor.h"
#include "RM_Motor.h"
#include "config.h"

/*============================== 全局变量定义 ==============================*/

ControlInput_t g_ControlInput = {0};
ControlState_t g_ControlState = {0};

/*============================== 任务相关 ==============================*/

// IbusTask
osThreadId_t IbusTaskHandle;
static const osThreadAttr_t IbusTask_attributes = {
    .name = "IbusTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh, // 高优先级
};

// ControlTask
osThreadId_t ControlTaskHandle;
static const osThreadAttr_t ControlTask_attributes = {
    .name = "ControlTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal, // 普通优先级
};

/*============================== 私有函数声明 ==============================*/

static void IbusTaskFunc(void *argument);
static void ControlTaskFunc(void *argument);
static void UpdateControlInput(void);
static void UpdateControlTargets(float dt);
static void SendMotorCommands(void);
static CtrlMode_e DetermineMode(int8_t swb, int8_t swc);
static void SyncTargetsWithMotorFeedback(void);

/*============================== 初始化函数 ==============================*/

/**
 * @brief 初始化控制状态模块
 */
void ControlState_Init(void)
{
    // 初始化ControlInput
    g_ControlInput.yaw_norm = 0.0f;
    g_ControlInput.load3508_norm = 0.0f;
    g_ControlInput.trigger_dir = 0;
    g_ControlInput.energy_norm = 0.0f;
    g_ControlInput.swb = 1; // 默认状态
    g_ControlInput.swc = 1;
    g_ControlInput.mode = CTRL_MODE_NORMAL_SINGLE;
    g_ControlInput.data_valid = false;

    // 初始化ControlState
    g_ControlState.yaw_target = 0.0f;
    g_ControlState.trigger_target = 0.0f;
    g_ControlState.energy_left_target = 0.0f;
    g_ControlState.energy_right_target = 0.0f;
    g_ControlState.load3508_target = 0.0f;
    g_ControlState.mode = CTRL_MODE_NORMAL_SINGLE;
    g_ControlState.manual_override = false;
    g_ControlState.debug_timer_start = 0;
    g_ControlState.debug_window_active = false;
    g_ControlState.single_shot_done = false;
}

/**
 * @brief 创建IbusTask和ControlTask
 */
void ControlState_CreateTasks(void)
{
    IbusTaskHandle = osThreadNew(IbusTaskFunc, NULL, &IbusTask_attributes);
    ControlTaskHandle = osThreadNew(ControlTaskFunc, NULL, &ControlTask_attributes);
}

/*============================== IbusTask实现 ==============================*/

/**
 * @brief IbusTask任务函数
 * @param argument 任务参数（未使用）
 *
 * 作用:
 * 1. 周期性调用IA6B_ProcessIbusPacket解析IBUS数据
 * 2. 将RawChannel转换为归一化值或方向
 * 3. 判定当前控制模式
 * 4. 更新g_ControlInput
 */
static void IbusTaskFunc(void *argument)
{
    (void)argument;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(IBUS_TASK_PERIOD_MS);

    while (1)
    {
        // 尝试获取并解析IBUS数据包
        if (IA6B_ProcessIbusPacket(BSP_UART6))
        {
            // 更新控制输入
            UpdateControlInput();
        }

        // 固定周期延时
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/**
 * @brief 更新控制输入（从RawChannel解析）
 */
static void UpdateControlInput(void)
{
    // 回中摇杆归一化（YAW - 右摇杆左右）
    g_ControlInput.yaw_norm = ControlState_NormalizeStick(RawChannel[0]);

    // 回中摇杆归一化（3508 - 左摇杆左右）
    g_ControlInput.load3508_norm = ControlState_NormalizeStick(RawChannel[3]);

    // 回中摇杆归一化（储能 - 右摇杆上下）
    g_ControlInput.energy_norm = ControlState_NormalizeStick(RawChannel[1]);

    // 非回中摇杆方向（扳机 - 左摇杆上下）
    g_ControlInput.trigger_dir = ControlState_GetDirection(RawChannel[2]);

    // 开关状态（直接使用已解析的Channel数组）
    g_ControlInput.swb = Channel[4]; // SWB: 1=默认, 0=手动
    g_ControlInput.swc = Channel[5]; // SWC: 1/0/-1

    // 判定控制模式
    g_ControlInput.mode = DetermineMode(g_ControlInput.swb, g_ControlInput.swc);

    // 标记数据有效
    g_ControlInput.data_valid = true;
}

/**
 * @brief 判定控制模式
 * @param swb SWB状态
 * @param swc SWC状态
 * @return 控制模式
 */
static CtrlMode_e DetermineMode(int8_t swb, int8_t swc)
{
    // 检查调试窗口是否激活
    if (g_ControlState.debug_window_active)
    {
        uint32_t elapsed = HAL_GetTick() - g_ControlState.debug_timer_start;
        if (elapsed < DEBUG_WINDOW_MS)
        {
            // 调试窗口内，SWB=0 才允许手动控制
            if (swb == 0)
            {
                return CTRL_MODE_DEBUG_TIMER;
            }
            // 调试窗口内但SWB=1，仍然是调试窗口模式（但不发送电机命令）
            return CTRL_MODE_DEBUG_TIMER;
        }
        else
        {
            // 调试窗口超时，关闭
            g_ControlState.debug_window_active = false;
        }
    }

    // 调试窗口外，SWB=0 无效，始终为正常单发模式
    // 只有在调试窗口内才允许手动控制
    return CTRL_MODE_NORMAL_SINGLE;
}

/**
 * @brief 同步目标值与当前电机反馈
 * @note 在进入手动模式时调用，避免目标突跳
 */
static void SyncTargetsWithMotorFeedback(void)
{
    // 同步YAW目标（DM4310）
    DM_Motor_RefreshData(DM_4310_YAW);
    g_ControlState.yaw_target = Motor_GetTotalAngle(DM_4310_YAW);

    // 同步扳机目标（RM2006）
    g_ControlState.trigger_target = Motor_GetTotalAngle(RM_2006_TRIGGER);

    // 同步储能电机目标（DM3519）
    g_ControlState.energy_left_target = Motor_GetTotalAngle(DM_3519_STRENTH_LEFT);
    g_ControlState.energy_right_target = Motor_GetTotalAngle(DM_3519_STRENTH_RIGHT);

    // 同步3508目标
    g_ControlState.load3508_target = Motor_GetTotalAngle(RM_3508_GRIPPER);
}

/*============================== ControlTask实现 ==============================*/

/**
 * @brief ControlTask任务函数
 * @param argument 任务参数（未使用）
 *
 * 作用:
 * 1. 根据ControlInput计算电机目标值增量
 * 2. 应用增量到目标值（target += dir/norm * weight * dt）
 * 3. 限幅处理
 * 4. 下发电机控制命令（仅在手动模式）
 */
static void ControlTaskFunc(void *argument)
{
    (void)argument;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(CTRL_TASK_PERIOD_MS);
    const float dt = (float)CTRL_TASK_PERIOD_MS / 1000.0f; // 转换为秒

    // 等待一段时间让系统稳定
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1)
    {
        // 更新控制状态模式
        bool prev_manual_override = g_ControlState.manual_override;
        g_ControlState.mode = g_ControlInput.mode;
        // 只有在调试窗口内且SWB=0时才允许手动控制
        g_ControlState.manual_override = (g_ControlState.mode == CTRL_MODE_DEBUG_TIMER &&
                                          g_ControlInput.swb == 0);

        // 检测进入手动模式的瞬间，同步目标值与电机反馈
        if (g_ControlState.manual_override && !prev_manual_override)
        {
            SyncTargetsWithMotorFeedback();
        }

        // 只有在手动模式或调试窗口模式下才更新目标并下发
        if (g_ControlState.manual_override && g_ControlInput.data_valid)
        {
            // 更新目标值
            UpdateControlTargets(dt);

            // 下发电机命令
            SendMotorCommands();
        }

        // 固定周期延时
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

/**
 * @brief 更新控制目标值
 * @param dt 时间步长（秒）
 */
static void UpdateControlTargets(float dt)
{
    // YAW目标更新（回中摇杆，增量式）
    g_ControlState.yaw_target += g_ControlInput.yaw_norm * WEIGHT_YAW * dt;
    g_ControlState.yaw_target = ControlState_Clamp(g_ControlState.yaw_target, YAW_MIN, YAW_MAX);

    // 扳机目标更新（非回中摇杆触发，SWC决定方向）
    // 非回中摇杆默认在底部(1000)，向上推时trigger_dir=1表示激活
    // 只有摇杆激活时才移动，移动方向由SWC决定：
    //   SWC=0  -> 扳机上移（向TRIGGER_MAX方向）
    //   SWC=-1 -> 扳机下移（向TRIGGER_MIN方向）
    //   SWC=1  -> 不移动
    if (g_ControlInput.trigger_dir == 1)
    {
        int8_t move_dir = 0;
        if (g_ControlInput.swc == 0)
        {
            move_dir = 1;  // 上移
        }
        else if (g_ControlInput.swc == -1)
        {
            move_dir = -1; // 下移
        }
        // SWC=1时move_dir=0，不移动
        g_ControlState.trigger_target += (float)move_dir * WEIGHT_TRIGGER * dt;
    }
    g_ControlState.trigger_target = ControlState_Clamp(g_ControlState.trigger_target, TRIGGER_MIN, TRIGGER_MAX);

    // 储能电机目标更新（回中摇杆，增量式）
    // 左右电机联动，方向相反
    float energy_delta = g_ControlInput.energy_norm * WEIGHT_ENERGY * dt;
    g_ControlState.energy_left_target += energy_delta; //
    g_ControlState.energy_right_target -= energy_delta; // 右电机方向相反
    g_ControlState.energy_left_target = ControlState_Clamp(g_ControlState.energy_left_target, ENERGY_LEFT_MIN, ENERGY_LEFT_MAX);
    g_ControlState.energy_right_target = ControlState_Clamp(g_ControlState.energy_right_target, ENERGY_RIGHT_MIN, ENERGY_RIGHT_MAX);

    // 3508目标更新（回中摇杆，增量式）
    g_ControlState.load3508_target += g_ControlInput.load3508_norm * WEIGHT_LOAD3508 * dt;
    g_ControlState.load3508_target = ControlState_Clamp(g_ControlState.load3508_target, LOAD3508_MIN, LOAD3508_MAX);
}

/**
 * @brief 下发电机控制命令
 */
static void SendMotorCommands(void)
{
    // YAW电机（DM4310 MIT模式）
    DmMotorSendCfg(DM_4310_YAW, g_ControlState.yaw_target, 0.0f, DM_MIT);

    // 扳机电机（RM2006 PID控制）
    RmMotorPID_Calc(RM_2006_TRIGGER, g_ControlState.trigger_target);

    // 储能电机（DM3519 位置速度模式）
    DmMotorSendCfg(DM_3519_STRENTH_LEFT, g_ControlState.energy_left_target, 5.0f, DM_LOCATION_SPEED);
    DmMotorSendCfg(DM_3519_STRENTH_RIGHT, g_ControlState.energy_right_target, 5.0f, DM_LOCATION_SPEED);

    // 3508电机（RM3508 PID控制）
    // 注意：这里假设换弹结构使用同一个3508电机
    RmMotorPID_Calc(RM_3508_GRIPPER, g_ControlState.load3508_target);
}

/*============================== API函数实现 ==============================*/

/**
 * @brief 获取当前控制模式
 */
CtrlMode_e ControlState_GetMode(void)
{
    return g_ControlState.mode;
}

/**
 * @brief 检查是否处于手动覆盖模式
 */
bool ControlState_IsManualOverride(void)
{
    return g_ControlState.manual_override;
}

/**
 * @brief 开启调试窗口
 */
void ControlState_StartDebugWindow(uint32_t duration_ms)
{
    (void)duration_ms; // 使用默认值 DEBUG_WINDOW_MS
    // 同步目标值与当前电机反馈，避免进入手动模式时突跳
    SyncTargetsWithMotorFeedback();

    g_ControlState.debug_timer_start = HAL_GetTick();
    g_ControlState.debug_window_active = true;
    g_ControlState.single_shot_done = true;
}

/**
 * @brief 设置YAW目标
 */
void ControlState_SetYawTarget(float target)
{
    g_ControlState.yaw_target = ControlState_Clamp(target, YAW_MIN, YAW_MAX);
}

/**
 * @brief 设置扳机目标
 */
void ControlState_SetTriggerTarget(float target)
{
    g_ControlState.trigger_target = ControlState_Clamp(target, TRIGGER_MIN, TRIGGER_MAX);
}

/**
 * @brief 设置储能电机目标
 */
void ControlState_SetEnergyTarget(float left_target, float right_target)
{
    g_ControlState.energy_left_target = ControlState_Clamp(left_target, ENERGY_LEFT_MIN, ENERGY_LEFT_MAX);
    g_ControlState.energy_right_target = ControlState_Clamp(right_target, ENERGY_RIGHT_MIN, ENERGY_RIGHT_MAX);
}

/**
 * @brief 设置3508目标
 */
void ControlState_SetLoad3508Target(float target)
{
    g_ControlState.load3508_target = ControlState_Clamp(target, LOAD3508_MIN, LOAD3508_MAX);
}

/**
 * @brief 标记单发流程完成
 */
void ControlState_MarkSingleShotDone(void)
{
    g_ControlState.single_shot_done = true;
}
