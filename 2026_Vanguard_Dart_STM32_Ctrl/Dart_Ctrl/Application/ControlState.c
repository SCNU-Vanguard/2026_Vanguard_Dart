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
 * - 调试窗口模式：单发完成后90s内可手动调试
 *********************************************************/
#include "ControlState.h"
#include "IA6B.h"
#include "CanMotor.h"
#include "RM_Motor.h"
#include "HX06L.h"
#include "FireControl.h"
#include "MotorControlTask.h"
#include "config.h"

/*============================== 全局变量定义 ==============================*/

ControlInput_t g_ControlInput = {0}; /* 初始化 g_ControlInput。 */
ControlState_t g_ControlState = {0}; /* 初始化 g_ControlState。 */

// 电机控制权互斥信号量（调试任务和自动任务互斥使用）
static StaticSemaphore_t g_xMotorCtrlSemBuffer; /* 保存 g_xMotorCtrlSemBuffer。 */
SemaphoreHandle_t g_xMotorCtrlSemHandle = NULL; /* 初始化 g_xMotorCtrlSemHandle。 */
SemaphoreHandle_t g_xDebugFinishedSemHandle = NULL; /* 初始化 g_xDebugFinishedSemHandle。 */
static StaticSemaphore_t g_xDebugFinishedSemBuffer; /* 保存 g_xDebugFinishedSemBuffer。 */
static volatile uint8_t s_ibus_lost_pending = 0U; /* IBUS 超时待处理标志。 */

/*============================== 任务相关 ==============================*/

// IbusTask
osThreadId_t IbusTaskHandle; /* 保存 IbusTaskHandle。 */
static const osThreadAttr_t IbusTask_attributes = { /* 初始化 IbusTask_attributes。 */
    .name = "IbusTask", /* 配置 name。 */
    .stack_size = 128 * 4, /* 配置 stack_size。 */
    .priority = (osPriority_t)osPriorityHigh, // 高优先级
};

// ControlTask
osThreadId_t ControlTaskHandle; /* 保存 ControlTaskHandle。 */
static const osThreadAttr_t ControlTask_attributes = { /* 初始化 ControlTask_attributes。 */
    .name = "ControlTask", /* 配置 name。 */
    .stack_size = 256 * 4, /* 配置 stack_size。 */
    .priority = (osPriority_t)osPriorityNormal, // 普通优先级
};

/*============================== 私有函数声明 ==============================*/

static void IbusTaskFunc(void *argument); /* 声明 IbusTaskFunc 接口。 */
static void ControlTaskFunc(void *argument); /* 声明 ControlTaskFunc 接口。 */
static void UpdateControlInput(void); /* 声明 UpdateControlInput 接口。 */
static void UpdateFireIntentFromControlInput(void); /* 声明 UpdateFireIntentFromControlInput 接口。 */
static void UpdateControlTargets(float dt); /* 声明 UpdateControlTargets 接口。 */
static void SendMotorCommands(void); /* 声明 SendMotorCommands 接口。 */
static CtrlMode_e DetermineMode(int8_t swb, int8_t swc); /* 声明 DetermineMode 接口。 */
static void SyncTargetsWithMotorFeedback(void); /* 声明 SyncTargetsWithMotorFeedback 接口。 */
static void DisableManualMotorOutputs(void); /* 声明 DisableManualMotorOutputs 接口。 */

/*============================== 初始化函数 ==============================*/

/**
 * @brief 初始化控制状态模块
 */
void ControlState_Init(void) /* 实现 ControlState_Init。 */
{
    // 初始化ControlInput
    g_ControlInput.yaw_norm = 0.0f; /* 更新 yaw_norm。 */
    g_ControlInput.load3508_norm = 0.0f; /* 更新 load3508_norm。 */
    g_ControlInput.trigger_dir = 0; /* 更新 trigger_dir。 */
    g_ControlInput.load3508_dir = 0; /* 更新 load3508_dir。 */
    g_ControlInput.energy_norm = 0.0f; /* 更新 energy_norm。 */
    g_ControlInput.swb = 1; // 默认状态
    g_ControlInput.swc = 1; /* 更新 swc。 */
    g_ControlInput.mode = CTRL_MODE_NORMAL_SINGLE; /* 更新 mode。 */
    g_ControlInput.data_valid = false; /* 更新 data_valid。 */

    // 初始化ControlState
    g_ControlState.yaw_target = 0.0f; /* 更新 yaw_target。 */
    g_ControlState.trigger_target = 0.0f; /* 更新 trigger_target。 */
    g_ControlState.energy_left_target = 0.0f; /* 更新 energy_left_target。 */
    g_ControlState.energy_right_target = 0.0f; /* 更新 energy_right_target。 */
    g_ControlState.load3508_target = 0.0f; /* 更新 load3508_target。 */
    g_ControlState.mode = CTRL_MODE_NORMAL_SINGLE; /* 更新 mode。 */
    g_ControlState.manual_override = false; /* 更新 manual_override。 */
    g_ControlState.debug_timer_start = 0; /* 更新 debug_timer_start。 */
    g_ControlState.debug_window_active = false; /* 更新 debug_window_active。 */
    g_ControlState.single_shot_done = false; /* 更新 single_shot_done。 */

    // 创建电机控制权互斥信号量（初始状态为可用，自动任务可以先获取）
    g_xMotorCtrlSemHandle = xSemaphoreCreateBinaryStatic(&g_xMotorCtrlSemBuffer); /* 更新 g_xMotorCtrlSemHandle。 */
    g_xDebugFinishedSemHandle = xSemaphoreCreateBinaryStatic(&g_xDebugFinishedSemBuffer); /* 更新 g_xDebugFinishedSemHandle。 */
    if (g_xMotorCtrlSemHandle != NULL) /* 检查当前执行条件。 */
    {
        xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
        // g_xDebugFinishedSemHandle 保持空状态，等待第一次调试窗口触发
    }
    else /* 处理其余情况。 */
    {
        // 信号量创建失败，进入错误处理
        Error_Handler(); /* 调用 Error_Handler。 */
    }
}

/**
 * @brief 创建IbusTask和ControlTask
 */
void ControlState_CreateTasks(void) /* 实现 ControlState_CreateTasks。 */
{
    IbusTaskHandle = osThreadNew(IbusTaskFunc, NULL, &IbusTask_attributes); /* 更新 IbusTaskHandle。 */
    ControlTaskHandle = osThreadNew(ControlTaskFunc, NULL, &ControlTask_attributes); /* 更新 ControlTaskHandle。 */
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
static void IbusTaskFunc(void *argument) /* 实现 IbusTaskFunc。 */
{
    (void)argument; /* 显式忽略参数 argument。 */

    TickType_t xLastWakeTime = xTaskGetTickCount(); /* 初始化 xLastWakeTime。 */
    const TickType_t xPeriod = pdMS_TO_TICKS(IBUS_TASK_PERIOD_MS); /* 初始化 xPeriod。 */

    // LED调试指示（每接收200次数据闪烁一次）
    uint16_t led_toggle_count = 0; /* 初始化 led_toggle_count。 */
    const uint16_t LED_TOGGLE_PERIOD = 200; // 200次 * 5ms = 1秒

    while (1) /* 持续执行当前任务。 */
    {
        // 尝试获取并解析IBUS数据包
        if (IA6B_ProcessIbusPacket(BSP_UART6)) /* 检查当前执行条件。 */
        {
            SoftwareWatchdog_Feed(SOFTWARE_WATCHDOG_IBUS); /* 有效包重置 100 ms 看门狗。 */
            // 更新控制输入
            UpdateControlInput(); /* 调用 UpdateControlInput。 */
            // LED闪烁指示数据接收正常
            led_toggle_count++; /* 递增 led_toggle_count。 */
            if (led_toggle_count >= LED_TOGGLE_PERIOD) /* 检查当前执行条件。 */
            {
                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin); /* 调用 HAL_GPIO_TogglePin。 */
                led_toggle_count = 0; /* 更新 led_toggle_count。 */
            }
        }
        // 固定周期延时
        vTaskDelayUntil(&xLastWakeTime, xPeriod); /* 调用 vTaskDelayUntil。 */
    }
}

/**
 * @brief 更新控制输入（从RawChannel解析）
 */
static void UpdateControlInput(void) /* 实现 UpdateControlInput。 */
{
    // 回中摇杆归一化（YAW - 右摇杆左右）
    g_ControlInput.yaw_norm = ControlState_NormalizeStick(RawChannel[0]); /* 更新 yaw_norm。 */

    // 左摇杆左右：先检测非回中状态，再计算归一化值
    // 非回中状态用于触发舵机，回中状态用于调节3508
    int16_t load3508_raw = RawChannel[3]; /* 初始化 load3508_raw。 */
    if (load3508_raw > NON_CENTER_HIGH) /* 检查当前执行条件。 */
        g_ControlInput.load3508_dir = 1; // 右推（非回中）
    else if (load3508_raw < NON_CENTER_LOW) /* 继续判断下一条件。 */
        g_ControlInput.load3508_dir = -1; // 左推（非回中）
    else /* 处理其余情况。 */
        g_ControlInput.load3508_dir = 0; // 回中状态

    // 回中时才使用归一化值调节3508
    g_ControlInput.load3508_norm = ControlState_NormalizeStick(load3508_raw); /* 更新 load3508_norm。 */

    // 回中摇杆归一化（储能 - 右摇杆上下）
    g_ControlInput.energy_norm = ControlState_NormalizeStick(RawChannel[1]); /* 更新 energy_norm。 */

    // 非回中摇杆方向（扳机 - 左摇杆上下）
    g_ControlInput.trigger_dir = ControlState_GetDirection(RawChannel[2]); /* 更新 trigger_dir。 */

    // 开关状态（直接使用已解析的Channel数组）
    g_ControlInput.swb = Channel[4]; // SWB: 1=默认, 0=手动
    g_ControlInput.swc = Channel[5]; // SWC: 1/0/-1

    // 判定控制模式
    g_ControlInput.mode = DetermineMode(g_ControlInput.swb, g_ControlInput.swc); /* 更新 mode。 */

    // 标记数据有效
    g_ControlInput.data_valid = true; /* 更新 data_valid。 */

    // 将遥控器解析后的业务意图同步到 FireControl
    UpdateFireIntentFromControlInput(); /* 调用 UpdateFireIntentFromControlInput。 */
}

/**
 * @brief 将当前遥控器输入映射为 FireControl 业务意图
 *
 * 当前采用最小映射方案：
 * - fire_enable  : SWB 不在手动调试位时允许自动流程申请发射
 * - target_select: SWC=1 选择前哨站，其余档位选择基地
 * - range_select : 左摇杆左右三态量映射为默认/中档/高档
 */
static void UpdateFireIntentFromControlInput(void) /* 实现 UpdateFireIntentFromControlInput。 */
{
    bool fire_enable = (g_ControlInput.swb != 0); /* 初始化 fire_enable。 */
    uint8_t target_select = (g_ControlInput.swc == 1) ? FIRE_TARGET_OUTPOST : FIRE_TARGET_BASE; // 这里等待更改
    uint8_t range_select = FIRE_RANGE_DEFAULT; /* 初始化 range_select。 */

    if (g_ControlInput.load3508_dir > 0) /* 检查当前执行条件。 */
    {
        range_select = FIRE_RANGE_1; /* 更新 range_select。 */
    }
    else if (g_ControlInput.load3508_dir < 0) /* 继续判断下一条件。 */
    {
        range_select = FIRE_RANGE_2; /* 更新 range_select。 */
    }

    (void)FireControl_UpdateIntent(fire_enable, target_select, range_select); /* 调用 FireControl_UpdateIntent。 */
}

/**
 * @brief 判定控制模式
 * @param swb SWB状态
 * @param swc SWC状态
 * @return 控制模式
 */
static CtrlMode_e DetermineMode(int8_t swb, int8_t swc) /* 实现 DetermineMode。 */
{
    // 【调试模式】：SWB=0 时直接进入手动控制，无需等待调试窗口
    // 正式比赛时可以注释掉这段代码，恢复调试窗口限制
#if DEBUG_MODE_ALWAYS_ALLOW /* 按 DEBUG_MODE_ALWAYS_ALLOW 选择编译分支。 */
    (void)swc; /* 显式忽略参数 swc。 */
    if (swb == 0) /* 检查当前执行条件。 */
    {
        // 如果调试窗口未激活，自动激活它
        if (!g_ControlState.debug_window_active) /* 检查当前执行条件。 */
        {
            g_ControlState.debug_window_active = true; /* 更新 debug_window_active。 */
            g_ControlState.debug_timer_start = HAL_GetTick(); /* 更新 debug_timer_start。 */
        }
        return CTRL_MODE_DEBUG_TIMER; /* 返回当前计算结果。 */
    }
    return CTRL_MODE_NORMAL_SINGLE; /* 返回当前计算结果。 */
#else /* 切换到备用编译分支。 */
    // 检查调试窗口是否激活
    if (g_ControlState.debug_window_active) /* 检查当前执行条件。 */
    {
        uint32_t elapsed = HAL_GetTick() - g_ControlState.debug_timer_start; /* 初始化 elapsed。 */
        if (elapsed < DEBUG_WINDOW_MS) /* 检查当前执行条件。 */
        {
            // 调试窗口内，SWB=0 才允许手动控制
            if (swb == 0) /* 检查当前执行条件。 */
            {
                return CTRL_MODE_DEBUG_TIMER; /* 返回当前计算结果。 */
            }
            // 调试窗口内但SWB=1，仍然是调试窗口模式（但不发送电机命令）
            return CTRL_MODE_DEBUG_TIMER; /* 返回当前计算结果。 */
        }
        else /* 处理其余情况。 */
        {
            // 调试窗口超时，关闭
            g_ControlState.debug_window_active = false; /* 更新 debug_window_active。 */
        }
    }

    // 调试窗口外，SWB=0 无效，始终为正常单发模式
    // 只有在调试窗口内才允许手动控制
    return CTRL_MODE_NORMAL_SINGLE; /* 返回当前计算结果。 */
#endif /* 结束条件编译。 */
}

/**
 * @brief 同步目标值与当前电机反馈
 * @note 在进入手动模式时调用，避免目标突跳
 */
static void SyncTargetsWithMotorFeedback(void) /* 实现 SyncTargetsWithMotorFeedback。 */
{
    // 同步YAW目标（RM6020）
    // g_ControlState.yaw_target = Motor_GetTotalAngle(RM_6020_YAW);

    // 同步扳机目标（RM2006）
    g_ControlState.trigger_target = Motor_GetTotalAngle(RM_2006_TRIGGER); /* 更新 trigger_target。 */

    // 同步储能电机目标（DM3519）
    // 同步两侧储能 M3508 的目标位置
    g_ControlState.energy_left_target = Motor_GetTotalAngle(RM_3508_STORE_LEFT); /* 更新 energy_left_target。 */
    g_ControlState.energy_right_target = Motor_GetTotalAngle(RM_3508_STORE_RIGHT); /* 更新 energy_right_target。 */

    // 同步3508目标
    g_ControlState.load3508_target = Motor_GetTotalAngle(RM_3508_GRIPPER); /* 更新 load3508_target。 */
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
static void ControlTaskFunc(void *argument) /* 实现 ControlTaskFunc。 */
{
    (void)argument; /* 显式忽略参数 argument。 */

    TickType_t xLastWakeTime = xTaskGetTickCount(); /* 初始化 xLastWakeTime。 */
    const TickType_t xPeriod = pdMS_TO_TICKS(CTRL_TASK_PERIOD_MS); /* 初始化 xPeriod。 */
    const float dt = (float)CTRL_TASK_PERIOD_MS / 1000.0f; // 转换为秒

    // 调试模式LED闪烁计数
    uint16_t debug_led_counter = 0; /* 初始化 debug_led_counter。 */
    // 是否持有电机控制权
    bool holding_motor_ctrl = false; /* 初始化 holding_motor_ctrl。 */

    // 等待一段时间让系统稳定
    vTaskDelay(pdMS_TO_TICKS(100)); /* 调用 vTaskDelay。 */

    while (1) /* 持续执行当前任务。 */
    {
        // 更新控制状态模式
        bool ibus_lost = false; /* 本轮是否需要执行丢失清理。 */

        taskENTER_CRITICAL(); /* 与 1 ms LostCallback 原子交换标志。 */
        if (s_ibus_lost_pending != 0U) /* 检查当前执行条件。 */
        {
            s_ibus_lost_pending = 0U;                    /* 消费一次超时事件。 */
            g_ControlInput.yaw_norm = 0.0f;              /* 清 YAW 输入。 */
            g_ControlInput.load3508_norm = 0.0f;         /* 清换弹输入。 */
            g_ControlInput.energy_norm = 0.0f;           /* 清储能输入。 */
            g_ControlInput.trigger_dir = 0;              /* 清扳机方向。 */
            g_ControlInput.load3508_dir = 0;             /* 清换弹方向。 */
            g_ControlInput.swb = 1;                      /* 开关恢复安全默认值。 */
            g_ControlInput.swc = 1;                      /* 档位恢复安全默认值。 */
            g_ControlInput.data_valid = false;           /* 禁止继续消费旧遥控数据。 */
            g_ControlInput.mode = CTRL_MODE_NORMAL_SINGLE; /* 强制退出手动模式。 */
            ibus_lost = true;                            /* 临界区外执行阻塞操作。 */
        }
        taskEXIT_CRITICAL(); /* 结束标志交换。 */

        if (ibus_lost) /* 检查当前执行条件。 */
        {
            (void)FireControl_SetFireEnable(false);        /* 立即撤销遥控发射意图。 */
            g_ControlState.debug_window_active = false;    /* 关闭调试窗口。 */
            if (holding_motor_ctrl) /* 检查当前执行条件。 */
            {
                DisableManualMotorOutputs(); /* 持锁时先停止手动控制电机。 */
            }
        }

        g_ControlState.mode = g_ControlInput.mode; /* 更新 mode。 */
        // 只有在调试窗口内且SWB=0时才允许手动控制
        g_ControlState.manual_override = (g_ControlState.mode == CTRL_MODE_DEBUG_TIMER && /* 继续组合表达式。 */
                                          g_ControlInput.swb == 0); /* 更新 swb。 */

        // 手动模式下持续尝试拿锁（非阻塞），不依赖边沿检测
        if (g_ControlState.manual_override && !holding_motor_ctrl) /* 检查当前执行条件。 */
        {
            if (g_xMotorCtrlSemHandle != NULL && /* 检查当前执行条件。 */
                xSemaphoreTake(g_xMotorCtrlSemHandle, 0) == pdTRUE) /* 继续更新 目标值。 */
            {
                holding_motor_ctrl = true; /* 更新 holding_motor_ctrl。 */
                SyncTargetsWithMotorFeedback(); /* 调用 SyncTargetsWithMotorFeedback。 */
                HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET); /* 调用 HAL_GPIO_WritePin。 */
            }
        }

        // 退出手动模式：释放控制权
        if (!g_ControlState.manual_override && holding_motor_ctrl) /* 检查当前执行条件。 */
        {
            xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
            xSemaphoreGive(g_xDebugFinishedSemHandle); /* 调用 xSemaphoreGive。 */
            holding_motor_ctrl = false; /* 更新 holding_motor_ctrl。 */
            g_ControlState.debug_window_active = false; /* 更新 debug_window_active。 */
            HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET); /* 调用 HAL_GPIO_WritePin。 */
        }

        // 只有在手动模式且持有控制权时才更新目标并下发
        if (g_ControlState.manual_override && holding_motor_ctrl && g_ControlInput.data_valid) /* 检查当前执行条件。 */
        {
            // 更新目标值
            UpdateControlTargets(dt); /* 调用 UpdateControlTargets。 */

            // 下发电机命令
            SendMotorCommands(); /* 调用 SendMotorCommands。 */

            // 调试模式LED闪烁指示（每50次闪一下，约500ms）
            debug_led_counter++; /* 递增 debug_led_counter。 */
            if (debug_led_counter >= 50) /* 检查当前执行条件。 */
            {
                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin); /* 调用 HAL_GPIO_TogglePin。 */
                debug_led_counter = 0; /* 更新 debug_led_counter。 */
            }
        }

        // 调试窗口超时自动放行（未被手动接管时，超时后通知自动任务继续）
        if (g_ControlState.debug_window_active && !holding_motor_ctrl) /* 检查当前执行条件。 */
        {
            // 常开调试模式下：SWB不在手动位时，不阻塞自动流程
#if DEBUG_MODE_ALWAYS_ALLOW /* 按 DEBUG_MODE_ALWAYS_ALLOW 选择编译分支。 */
            if (g_ControlInput.swb != 0) /* 检查当前执行条件。 */
            {
                g_ControlState.debug_window_active = false; /* 更新 debug_window_active。 */
                xSemaphoreGive(g_xDebugFinishedSemHandle); /* 调用 xSemaphoreGive。 */
            }
            else /* 处理其余情况。 */
#endif /* 结束条件编译。 */
            {
                uint32_t elapsed = HAL_GetTick() - g_ControlState.debug_timer_start; /* 初始化 elapsed。 */
                if (elapsed >= DEBUG_WINDOW_MS) /* 检查当前执行条件。 */
                {
                    g_ControlState.debug_window_active = false; /* 更新 debug_window_active。 */
                    xSemaphoreGive(g_xDebugFinishedSemHandle); /* 调用 xSemaphoreGive。 */
                }
            }
        }

        // 固定周期延时（ControlTask）
        vTaskDelayUntil(&xLastWakeTime, xPeriod); /* 调用 vTaskDelayUntil。 */
    }
}

/** @brief IBUS 丢失后停止手动接管的电机输出。 */
static void DisableManualMotorOutputs(void) /* 实现 DisableManualMotorOutputs。 */
{
    Motor_EnableControl(RM_2006_TRIGGER, false);    /* 扳机电机置零。 */
    Motor_EnableControl(RM_3508_STORE_LEFT, false); /* 左储能电机置零。 */
    Motor_EnableControl(RM_3508_STORE_RIGHT, false);/* 右储能电机置零。 */
}

/**
 * @brief 更新控制目标值
 * @param dt 时间步长（秒）
 */
static void UpdateControlTargets(float dt) /* 实现 UpdateControlTargets。 */
{
    // YAW目标更新（回中摇杆，增量式）
    // g_ControlState.yaw_target += g_ControlInput.yaw_norm * WEIGHT_YAW * dt;
    // g_ControlState.yaw_target = ControlState_Clamp(g_ControlState.yaw_target, YAW_MIN, YAW_MAX);

    // 扳机目标更新（非回中摇杆，单方向）
    // IA6B 左摇杆上下默认在底部，仅在激活时向一个方向推进 2006 目标
    g_ControlState.trigger_target -= (float)g_ControlInput.trigger_dir * WEIGHT_TRIGGER * dt; /* 更新 trigger_target。 */
    g_ControlState.trigger_target = ControlState_Clamp(g_ControlState.trigger_target, TRIGGER_MIN, TRIGGER_MAX); /* 更新 trigger_target。 */

    // 储能电机目标更新（回中摇杆，增量式）
    // 左右电机联动，方向相反
    float energy_delta = g_ControlInput.energy_norm * WEIGHT_ENERGY * dt; /* 初始化 energy_delta。 */
    g_ControlState.energy_left_target += energy_delta;  //
    g_ControlState.energy_right_target -= energy_delta; // 右电机方向相反
    g_ControlState.energy_left_target = ControlState_Clamp(g_ControlState.energy_left_target, ENERGY_LEFT_MIN, ENERGY_LEFT_MAX); /* 更新 energy_left_target。 */
    g_ControlState.energy_right_target = ControlState_Clamp(g_ControlState.energy_right_target, ENERGY_RIGHT_MIN, ENERGY_RIGHT_MAX); /* 更新 energy_right_target。 */

    // 3508/舵机在调试窗口中仍然禁用，避免和自动换弹流程混用
}

/**
 * @brief 下发电机控制命令
 */
static void SendMotorCommands(void) /* 实现 SendMotorCommands。 */
{
    // YAW 通过 6020 控制任务下发，避免手动和自动流程各自直接发包
    // 6020 yaw axis is not connected now.
    // Motor6020_SetTarget(g_ControlState.yaw_target);
    // Motor6020_EnableControl(true);

    // 扳机通过 2006 控制任务下发，调试时也走统一控制层
    Motor_SetTarget(RM_2006_TRIGGER, g_ControlState.trigger_target); /* 调用 Motor_SetTarget。 */
    Motor_EnableControl(RM_2006_TRIGGER, true); /* 调用 Motor_EnableControl。 */

    // 储能电机（DM3519 位置速度模式）
    // 通过储能 M3508 兼容控制层下发左右储能目标
    Motor_SetTarget(RM_3508_STORE_LEFT, g_ControlState.energy_left_target); /* 调用 Motor_SetTarget。 */
    Motor_SetTarget(RM_3508_STORE_RIGHT, g_ControlState.energy_right_target); /* 调用 Motor_SetTarget。 */
    Motor_EnableControl(RM_3508_STORE_LEFT, true); /* 调用 Motor_EnableControl。 */
    Motor_EnableControl(RM_3508_STORE_RIGHT, true); /* 调用 Motor_EnableControl。 */

    // 3508/舵机仍不在调试窗口中控制
}

/*============================== API函数实现 ==============================*/

/**
 * @brief 检查是否处于手动覆盖模式
 */
bool ControlState_IsManualOverride(void) /* 实现 ControlState_IsManualOverride。 */
{
    return g_ControlState.manual_override; /* 返回当前计算结果。 */
}

/**
 * @brief 开启调试窗口
 */
void ControlState_StartDebugWindow(uint32_t duration_ms) /* 实现 ControlState_StartDebugWindow。 */
{
    (void)duration_ms; // 使用默认值 DEBUG_WINDOW_MS
    // 同步目标值与当前电机反馈，避免进入手动模式时突跳
    SyncTargetsWithMotorFeedback(); /* 调用 SyncTargetsWithMotorFeedback。 */

    g_ControlState.debug_timer_start = HAL_GetTick(); /* 更新 debug_timer_start。 */
    g_ControlState.debug_window_active = true; /* 更新 debug_window_active。 */
    g_ControlState.single_shot_done = true; /* 更新 single_shot_done。 */
}

void ControlState_WatchdogInit(void) /* 实现 ControlState_WatchdogInit。 */
{
    s_ibus_lost_pending = 0U; /* 注册前清历史超时。 */
    (void)SoftwareWatchdog_Register(SOFTWARE_WATCHDOG_IBUS, /* 开始调用 SoftwareWatchdog_Register。 */
                                    IBUS_WATCHDOG_TIMEOUT_MS, /* 定义 IBUS_WATCHDOG_TIMEOUT_MS 枚举项。 */
                                    ControlState_IbusLostCallback); /* 完成本行操作。 */
}

void ControlState_IbusLostCallback(SoftwareWatchdogId_e id) /* 实现 ControlState_IbusLostCallback。 */
{
    if (id == SOFTWARE_WATCHDOG_IBUS) /* 检查当前执行条件。 */
    {
        s_ibus_lost_pending = 1U; /* 中断内只置任务待处理标志。 */
    }
}
