/***********************************
 * 通用电机控制（统一 Motor_* API）
 *
 * 业务层使用 Motor_SetTarget / Motor_GetTarget / Motor_EnableControl /
 * Motor_SetUseSCurve；保护状态直接查询 angle_motor。
 * 策略（S 型规划、同步 PID、过流保护）由注册时写入 MotorTypeDef 的字段
 * 决定，运行时控制任务统一遍历所有电机 tick。
 *
 * 夹爪 M3508 的多 owner 抢占 + 命令 stream 仍保留 LoadMotor_* 薄封装，
 * 内部最终调 Motor_SetTarget(RM_3508_GRIPPER, ...)。
 **********************************/

#include "./MotorControlTask.h"
#include "../User/inc/angle_motor.h"
#include "../User/inc/DM_Motor.h"
#include "bsp_dwt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

extern MotorManager_t MotorManager; /* 声明外部变量 MotorManager。 */

/* 调试用全局（保留给 JLink/SWV watch） */
float MotorData = 0.0f; /* 初始化 MotorData。 */
float gripper_offset = 0.0f; /* 初始化 gripper_offset。 */
float trigger_offset = 0.0f; /* 初始化 trigger_offset。 */
float target_loc = 0.0f; /* 初始化 target_loc。 */
float Load3508CurrentFilteredData = 0.0f; /* 初始化 Load3508CurrentFilteredData。 */
float Load3508CurrentAbsFilteredData = 0.0f; /* 初始化 Load3508CurrentAbsFilteredData。 */
float Load3508StillOverCurrentData = 0.0f; /* 初始化 Load3508StillOverCurrentData。 */

#define MOTOR_TARGET_EPS_DEG (1e-3f) /* 定义 MOTOR_TARGET_EPS_DEG。 */

static MotorRuntimeState_t s_runtime[g_CanMotorNum]; /* 保存 s_runtime。 */

/// @brief 获取运行时长
/// @param cfg 电机别名
/// @return 运行状态结构体
static inline MotorRuntimeState_t *Motor_GetRuntime(can_motor_cfg cfg); /* 声明 Motor_GetRuntime 接口。 */

/// @brief 获取互斥量操作句柄
/// @param cfg 电机别名
/// @return 互斥量操作句柄
static inline SemaphoreHandle_t Motor_GetMutex(can_motor_cfg cfg); /* 声明 Motor_GetMutex 接口。 */

/// @brief 获取电机转动的角度
/// @param cfg 电机别名
/// @return 对应电机角度
static inline float Motor_GetPosRaw(can_motor_cfg cfg); /* 声明 Motor_GetPosRaw 接口。 */

/* ------------------------------------------------------------------
 * LoadMotor_* 命令 stream（夹爪 M3508 专用）
 * 确认通信过程的密钥,根据密钥进行确认目标值
 * ------------------------------------------------------------------ */
#define LOAD_MOTOR_CMD_MAGIC 0x4C443508UL /* 定义 LOAD_MOTOR_CMD_MAGIC。 */
#define LOAD_MOTOR_CMD_SIGNATURE_KEY 0xA53C3508UL /* 定义 LOAD_MOTOR_CMD_SIGNATURE_KEY。 */
#define LOAD_MOTOR_CMD_STREAM_DEPTH 8U /* 定义 LOAD_MOTOR_CMD_STREAM_DEPTH。 */
#define LOAD_MOTOR_CMD_MAX_AGE_MS 100U /* 定义 LOAD_MOTOR_CMD_MAX_AGE_MS。 */
#define LOAD_MOTOR_DEFAULT_HOLD_MS 500U /* 定义 LOAD_MOTOR_DEFAULT_HOLD_MS。 */

/* Stream流的相关定义 */
static StaticStreamBuffer_t g_GripperCmdStreamBuffer; /* 保存 g_GripperCmdStreamBuffer。 */
static uint8_t g_GripperCmdStreamStorage[LOAD_MOTOR_CMD_STREAM_DEPTH * sizeof(LoadMotorCommand_t)]; /* 保存 g_GripperCmdStreamStorage。 */
static StreamBufferHandle_t g_GripperCmdStream = NULL; /* 初始化 g_GripperCmdStream。 */

/* LoadMotor的相关数值 */
static volatile uint32_t g_GripperCmdSeq = 0U; /* 初始化 g_GripperCmdSeq。 */
static LoadMotorOwner_e g_GripperActiveOwner = LOAD_MOTOR_OWNER_NONE; /* 初始化 g_GripperActiveOwner。 */
static uint8_t g_GripperActivePriority = 0U; /* 初始化 g_GripperActivePriority。 */
static uint32_t g_GripperActiveOwnerExpireMs = 0U; /* 初始化 g_GripperActiveOwnerExpireMs。 */

/* ------------------------------------------------------------------ */
static void Motor_TickOne(can_motor_cfg cfg, float sync_offset_deg); /* 声明 Motor_TickOne 接口。 */
static void Motor_HandleOffline(can_motor_cfg cfg, MotorRuntimeState_t *rt); /* 声明 Motor_HandleOffline 接口。 */
static void Motor_ResetTrap(MotorRuntimeState_t *rt, can_motor_cfg cfg, float seed_raw_deg); /* 声明 Motor_ResetTrap 接口。 */
// static void Motor_ResetState(MotorRuntimeState_t *rt, can_motor_cfg cfg);
static void LoadMotor_ProcessCommandStream(void); /* 声明 LoadMotor_ProcessCommandStream 接口。 */

/// @brief 初始化控制结构体
/// @param void
/// @note  这个只是整个电机控制结构体初始化用到的，不是电机信息初始化用到的
static SemaphoreHandle_t s_mtx[g_CanMotorNum]; /* 保存 s_mtx。 */
void MotorControl_Init(void) /* 实现 MotorControl_Init。 */
{
    uint8_t i; /* 保存 i。 */
    static StaticSemaphore_t s_mtx_buf[g_CanMotorNum]; /* 保存 s_mtx_buf。 */

    for (i = 0U; i < (uint8_t)g_CanMotorNum; i++) /* 遍历当前数据集合。 */
    {
        if (s_mtx[i] == NULL) /* 检查当前执行条件。 */
        {
            s_mtx[i] = xSemaphoreCreateMutexStatic(&s_mtx_buf[i]); // 创建各个电机的互斥量
        }
        memset(&s_runtime[i], 0, sizeof(s_runtime[i])); /* 调用 memset。 */
        s_runtime[i].target = Motor_GetTotalAngle((can_motor_cfg)(i + 1U)); // 根据当前位置进行初始化
        AngleMotor_ResetRuntime((can_motor_cfg)(i + 1U));                   /* 同步复位 angle_motor 判断/保护运行时 */
    }

    if (g_GripperCmdStream == NULL) /* 检查当前执行条件。 */
    {
        // 夹爪电机命令流
        g_GripperCmdStream = xStreamBufferCreateStatic(sizeof(g_GripperCmdStreamStorage), /* 传入下一项参数或数据。 */
                                                       sizeof(LoadMotorCommand_t), /* 传入下一项参数或数据。 */
                                                       g_GripperCmdStreamStorage, /* 传入下一项参数或数据。 */
                                                       &g_GripperCmdStreamBuffer); /* 完成本行操作。 */
    }

    /* 之前调试换弹的M3508电机用的 */
    // target_loc = 0.0f;
    // Load3508CurrentFilteredData = 0.0f; // 保存滤波后的夹爪电流
    // Load3508CurrentAbsFilteredData = 0.0f; // 保存滤波电流的绝对值
    // Load3508StillOverCurrentData = 0.0f; // 保存持续过流相关数据

    // 换弹夹爪电机命令信息流相关参数
    g_GripperActiveOwner = LOAD_MOTOR_OWNER_NONE; /* 更新 g_GripperActiveOwner。 */
    g_GripperActivePriority = 0U; /* 更新 g_GripperActivePriority。 */
    g_GripperActiveOwnerExpireMs = 0U; /* 更新 g_GripperActiveOwnerExpireMs。 */
    g_GripperCmdSeq = 0U; /* 更新 g_GripperCmdSeq。 */
}

/* ------------------------------------------------------------------
 * 通用 Setter / Getter
 * ------------------------------------------------------------------ */

/// @brief 设置电机目标值
/// @param cfg
/// @param target_deg
void Motor_SetTarget(can_motor_cfg cfg, float target_deg) /* 实现 Motor_SetTarget。 */
{
    // 拿状态和句柄
    MotorRuntimeState_t *rt = Motor_GetRuntime(cfg); /* 初始化 rt。 */
    SemaphoreHandle_t mtx = Motor_GetMutex(cfg); /* 初始化 mtx。 */
    bool same_target = false; /* 初始化 same_target。 */

    if (rt == NULL || !isfinite(target_deg)) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }
    if (mtx != NULL) /* 检查当前执行条件。 */
    {
        xSemaphoreTake(mtx, portMAX_DELAY); // 直接使用互斥量防止被其他线程操作
    }

    // 当前目标有效 && 新旧目标角度误差小于阈值
    same_target = isfinite(rt->target) && (fabsf(rt->target - target_deg) <= MOTOR_TARGET_EPS_DEG); /* 更新 same_target。 */
    if (same_target) /* 检查当前执行条件。 */
    {
        if (!rt->ctrl_enabled) /* 检查当前执行条件。 */
        {
            rt->target_set_while_disabled = 1U; /* 更新 target_set_while_disabled。 */
        }
        if (mtx != NULL) /* 检查当前执行条件。 */
        {
            xSemaphoreGive(mtx); /* 调用 xSemaphoreGive。 */
        }

        return; /* 结束当前函数。 */
    }

    rt->target = target_deg; /* 更新 target。 */
    rt->target_set_while_disabled = rt->ctrl_enabled ? 0U : 1U; // 电机处于禁用状态时，外部是否设置过新的目标位置
    rt->last_target_change_ms = HAL_GetTick(); /* 更新 last_target_change_ms。 */

    /* 换目标：复位 angle_motor 的过流计时与到位标志（防跨目标误判）*/
    /* 注意：不清故障闩锁——故障必须显式 AngleMotor_ClearFault()。 */
    /* 注意：不建议乱清除，遇到错误一般铁定都是死了 */
    AngleMotor_NotifyTargetChanged(cfg); /* 调用 AngleMotor_NotifyTargetChanged。 */

    /* 对于是否使用S形规划器进行分类 */
    if (rt->use_scurve) /* 检查当前执行条件。 */
    {
        Motor_ResetTrap(rt, cfg, Motor_GetPosRaw(cfg)); /* 调用 Motor_ResetTrap。 */
    }
    if (mtx != NULL) /* 检查当前执行条件。 */
    {
        xSemaphoreGive(mtx); /* 调用 xSemaphoreGive。 */
    }
}

/// @brief 获取电机目标值
/// @param cfg
/// @return
float Motor_GetTarget(can_motor_cfg cfg) /* 实现 Motor_GetTarget。 */
{
    MotorRuntimeState_t *rt = Motor_GetRuntime(cfg); /* 初始化 rt。 */
    SemaphoreHandle_t mtx = Motor_GetMutex(cfg); /* 初始化 mtx。 */
    float target; /* 保存 target。 */

    if (rt == NULL) /* 检查当前执行条件。 */
    {
        return 0.0f; /* 返回当前计算结果。 */
    }
    if (mtx == NULL) /* 检查当前执行条件。 */
    {
        return rt->target; /* 返回当前计算结果。 */
    }
    xSemaphoreTake(mtx, portMAX_DELAY); /* 调用 xSemaphoreTake。 */
    target = rt->target; /* 更新 target。 */
    xSemaphoreGive(mtx); /* 调用 xSemaphoreGive。 */
    return target; /* 返回当前计算结果。 */
}

/// @brief 使能电机控制
/// @param cfg
/// @param enable
void Motor_EnableControl(can_motor_cfg cfg, bool enable) /* 实现 Motor_EnableControl。 */
{
    MotorRuntimeState_t *rt = Motor_GetRuntime(cfg); /* 初始化 rt。 */
    MotorTypeDef *motor; /* 保存 motor。 */
    if (rt == NULL || (uint32_t)cfg < 1U || /* 检查当前执行条件。 */
        (uint32_t)cfg > (uint32_t)MotorManager.registered_count) /* 继续当前语句。 */
    {
        return; /* 结束当前函数。 */
    }
    motor = Motor_GetHandleFast(cfg); /* 更新 motor。 */

    // 电机控制使能，需要进行目标更新
    if (enable) /* 检查当前执行条件。 */
    {
        if (!Motor_IsOnline(cfg)) /* 离线电机不能直接开放控制。 */
        {
            if (motor->band != DM_MOTOR_BAND || motor->drive_enabled != 0U || /* 检查当前执行条件。 */
                DM_MotorEnable(cfg) == 0U || !Motor_IsOnline(cfg)) /* 继续更新 目标值。 */
            {
                return; /* 结束当前函数。 */
            }
        }
        if (rt->ctrl_enabled) /* 检查当前执行条件。 */
        {
            /* 软件控制已开放但 DM 驱动被单独失能时，允许重新补发使能。 */
            if (motor->band == DM_MOTOR_BAND && motor->drive_enabled == 0U) /* 检查当前执行条件。 */
            {
                (void)DM_MotorEnable(cfg); /* 调用 DM_MotorEnable。 */
            }
            return; /* 结束当前函数。 */
        }

        /* DM 电调有独立使能协议；发送失败则不能开放软件控制。 */
        if (motor->band == DM_MOTOR_BAND && motor->drive_enabled == 0U) /* 检查当前执行条件。 */
        {
            if (DM_MotorEnable(cfg) == 0U) /* 检查当前执行条件。 */
            {
                return; /* 结束当前函数。 */
            }
        }

        /* “安全启用”：仅在 disabled -> enabled 时清保护；
         * 若禁用期间已经设置过目标，则保留该目标。 */
        AngleMotor_ResetRuntime(cfg); /* 清 angle_motor 故障闩锁+过流计时+到位标志 */
        if (rt->target_set_while_disabled == 0U) /* 检查当前执行条件。 */
        {
            rt->target = Motor_GetTotalAngle(cfg); /* 更新 target。 */
        }
        rt->target_set_while_disabled = 0U; /* 更新 target_set_while_disabled。 */
        rt->ctrl_enabled = true; /* 更新 ctrl_enabled。 */
    }

    /* 未使能，不控制 */
    else /* 处理其余情况。 */
    {
        rt->ctrl_enabled = false; /* 更新 ctrl_enabled。 */
        rt->trap_initialized = false; /* 更新 trap_initialized。 */
        rt->target_set_while_disabled = 0U; /* 更新 target_set_while_disabled。 */
        if (motor->band == DM_MOTOR_BAND) /* 检查当前执行条件。 */
        {
            (void)DM_MotorDisable(cfg); /* 调用 DM_MotorDisable。 */
        }
        else /* 处理其余情况。 */
        {
            RmMotorSendCfg(cfg, 0); /* 调用 RmMotorSendCfg。 */
        }
    }
}

/// @brief 是否使用S形规划器
/// @param cfg
/// @return
void Motor_SetUseSCurve(can_motor_cfg cfg, bool enable) /* 实现 Motor_SetUseSCurve。 */
{
    MotorRuntimeState_t *rt = Motor_GetRuntime(cfg); /* 初始化 rt。 */
    if (rt == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }
    rt->use_scurve = enable; /* 更新 use_scurve。 */
    rt->trap_initialized = false; /* 更新 trap_initialized。 */
}

/// @brief 确认是否有拉簧拉力电机寄寄
/// @param
/// @return
bool Motor_IsAnyStoreProtected(void) /* 实现 Motor_IsAnyStoreProtected。 */
{
    /* 任一储能电机离线或保护锁存都进入安全返回。 */
    return !Motor_IsOnline(RM_3508_STORE_LEFT) || /* 继续组合表达式。 */
           !Motor_IsOnline(RM_3508_STORE_RIGHT) || /* 继续组合表达式。 */
           AngleMotor_IsFaulted(RM_3508_STORE_LEFT) || /* 开始调用 AngleMotor_IsFaulted。 */
           AngleMotor_IsFaulted(RM_3508_STORE_RIGHT); /* 调用 AngleMotor_IsFaulted。 */
}

static void Motor_HandleOffline(can_motor_cfg cfg, MotorRuntimeState_t *rt) /* 实现 Motor_HandleOffline。 */
{
    MotorTypeDef *motor; /* 当前离线电机。 */

    if (rt == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    motor = Motor_GetHandleFast(cfg); /* 获取电机运行状态。 */
    if (!rt->ctrl_enabled && motor->watchdog_lost_pending == 0U) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }
    motor->watchdog_lost_pending = 0U; /* 消费一次 LostCallback 事件。 */
    rt->ctrl_enabled = false;          /* 禁止继续计算输出。 */
    rt->trap_initialized = false;      /* 清轨迹规划状态。 */
    rt->target_set_while_disabled = 0U;/* 清离线期间目标标志。 */
    CASCADE_PID_Clear(&motor->cascade_pid); /* 清串级 PID 历史量。 */
    PID_Clear(&motor->inner_pid);            /* 清单环 PID 历史量。 */

    if (motor->MotorInf.band == DM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        if (motor->drive_enabled != 0U) /* 检查当前执行条件。 */
        {
            (void)DM_MotorDisable(cfg); /* DM 已使能时发送一次失能帧。 */
        }
    }
    else /* 处理其余情况。 */
    {
        RmMotorSendCfg(cfg, 0); /* RM 电机立即发送零电流。 */
    }
}

/* ------------------------------------------------------------------
 * S 型规划
 * ------------------------------------------------------------------ */
static void Motor_ResetTrap(MotorRuntimeState_t *rt, can_motor_cfg cfg, float seed_raw_deg) /* 实现 Motor_ResetTrap。 */
{
    MotorTypeDef *motor = Motor_GetHandleFast(cfg); /* 初始化 motor。 */
    const MotorTrapConfig_t *trap_cfg = &motor->trap_config; /* 初始化 trap_cfg。 */

    if (trap_cfg->registered == 0U) /* 检查当前执行条件。 */
    {
        rt->trap_initialized = false; /* 更新 trap_initialized。 */
        return; /* 结束当前函数。 */
    }

    Motor_TrapPos_Init(&rt->trap_profile, seed_raw_deg, /* 传入下一项参数或数据。 */
                       trap_cfg->vmax_deg_s, trap_cfg->amax_deg_s2); /* 完成本行操作。 */

    rt->trap_profile.brake_gain = trap_cfg->brake_gain; /* 更新 brake_gain。 */
    rt->trap_profile.arrive_zone = trap_cfg->arrive_zone; /* 更新 arrive_zone。 */
    rt->trap_profile.decel_zone = trap_cfg->decel_zone; /* 更新 decel_zone。 */
    Motor_TrapPos_SetJerk(&rt->trap_profile, trap_cfg->jmax_deg_s3); /* 调用 Motor_TrapPos_SetJerk。 */

    DWT_GetDeltaT(&rt->trap_cnt_last); /* 调用 DWT_GetDeltaT。 */
    rt->trap_initialized = true; /* 更新 trap_initialized。 */
}

/* ------------------------------------------------------------------
 * 单电机 tick（四层）：
 *  1) 规划器：trap/S 型生成参考位置 ref（未启用则直通目标）
 *  2~5) 控制器→状态判断器→限幅器→输出：统一交给 AngleMotor_Drive
 *
 * sync_offset_deg 专给储能同步 PID 使用，其他电机传 0。
 * ------------------------------------------------------------------ */
static void Motor_TickOne(can_motor_cfg cfg, float sync_offset_deg) /* 实现 Motor_TickOne。 */
{
    MotorRuntimeState_t *rt = Motor_GetRuntime(cfg); /* 初始化 rt。 */
    MotorTypeDef *motor = Motor_GetHandleFast(cfg); /* 初始化 motor。 */
    float target_relative_deg; /* 保存 target_relative_deg。 */
    float ref_relative_deg; /* 保存 ref_relative_deg。 */
    float target_raw_deg; /* 保存 target_raw_deg。 */
    float dt_s; /* 保存 dt_s。 */
    float cmd_pos; /* 保存 cmd_pos。 */

    if (rt == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    if (motor->watchdog_lost_pending != 0U) /* 丢失事件优先于在线恢复处理。 */
    {
        Motor_HandleOffline(cfg, rt); /* 调用 Motor_HandleOffline。 */
        return; /* 结束当前函数。 */
    }
    if (!Motor_IsOnline(cfg)) /* 检查当前执行条件。 */
    {
        Motor_HandleOffline(cfg, rt); /* 调用 Motor_HandleOffline。 */
        return; /* 结束当前函数。 */
    }
    if (!rt->ctrl_enabled) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    target_relative_deg = rt->target + sync_offset_deg; /* 更新 target_relative_deg。 */
    /* ---------------- 第 1 层：规划器 ----------------
     * 有 S 型规划则由 trap 生成参考位置，否则参考位置=目标位置(直通)。
     * RM 工作在编码器 raw 度域；S3519 经反馈适配后工作在逻辑度域。 */
    if (rt->use_scurve && motor->trap_config.registered != 0U) /* 检查当前执行条件。 */
    {
        target_raw_deg = target_relative_deg; /* 更新 target_raw_deg。 */
        if (motor->MotorInf.band == RM_MOTOR_BAND) /* 检查当前执行条件。 */
        {
            target_raw_deg += motor->motor_data.offset_ecd_angle; /* 更新 target_raw_deg。 */
        }

        // 未初始化规划器
        if (rt->trap_initialized == false) /* 检查当前执行条件。 */
        {
            Motor_ResetTrap(rt, cfg, Motor_GetPosRaw(cfg)); /* 调用 Motor_ResetTrap。 */
        }

        // 已经初始化规划器
        if (rt->trap_initialized != false) /* 检查当前执行条件。 */
        {
            // 更新目标
            rt->trap_profile.target_pos = target_raw_deg; /* 更新 target_pos。 */

            // 确认时间间隔
            dt_s = DWT_GetDeltaT(&rt->trap_cnt_last); /* 更新 dt_s。 */

            // 防止异常浮点数进入控制
            if (!isfinite(dt_s) || dt_s <= 0.0f) /* 检查当前执行条件。 */
            {
                dt_s = 0.001f; /* 更新 dt_s。 */
            }

            // 给控制周期设置上限
            // 如果任务因为阻塞、调试断点或调度延迟，导致 dt_s 突然变成 0.5s，这一步就会产生很大的位置或速度跳变。限制后就会截断控制到20ms的步进效果
            else if (dt_s > 0.02f) /* 继续判断下一条件。 */
            {
                dt_s = 0.02f; /* 更新 dt_s。 */
            }

            // 更新规划后的控制位置
            cmd_pos = Motor_TrapPos_Update(&rt->trap_profile, dt_s); /* 更新 cmd_pos。 */
            ref_relative_deg = cmd_pos; /* 更新 ref_relative_deg。 */
            if (motor->MotorInf.band == RM_MOTOR_BAND) /* 检查当前执行条件。 */
            {
                ref_relative_deg -= motor->motor_data.offset_ecd_angle; /* 更新 ref_relative_deg。 */
            }
        }

        else /* 处理其余情况。 */
        {
            ref_relative_deg = target_relative_deg; // 未注册规划器，直接把大幅度阶跃目标给出去
        }
    }

    else /* 处理其余情况。 */
    {
        ref_relative_deg = target_relative_deg; // 未注册规划器，直接把大幅度阶跃目标给出去
    }

    /* ---------------- 第 2~5 层：控制器→判断器→限幅器→输出 ----------------
     * 全部由 angle_motor 统一 tick 完成（数据驱动，无按电机的特殊分支）。 */
    if (AngleMotor_IsManaged(cfg)) /* 检查当前执行条件。 */
    {
        (void)AngleMotor_Drive(cfg, ref_relative_deg, target_relative_deg); /* 调用 AngleMotor_Drive。 */
        return; /* 结束当前函数。 */
    }

    /* 未纳入 angle_motor 的电机（理论上不会走到，YAW 由 StateSet 直控）：*/
    /* 退回旧串级 PID，保证兼容不炸。 */
    RmMotorPID_Calc(cfg, ref_relative_deg); /* 调用 RmMotorPID_Calc。 */
}

/* ------------------------------------------------------------------
 * 统一电机控制任务（2ms 周期）
 *   - 先跑储能左右同步 PID，得到 correction
 *   - 遍历所有已注册电机 tick
 *   - 储能两侧在遍历时带入 ±correction 偏置
 * ------------------------------------------------------------------ */
void MotorCtrlTask(void *argument) /* 实现 MotorCtrlTask。 */
{
    TickType_t xLastWake; /* 保存 xLastWake。 */
    (void)argument; /* 显式忽略参数 argument。 */

    /* 启动时先把夹爪目标拉到当前位置（原 3508 任务的初始 home 行为） */
    {
        float home_deg = Motor_GetTotalAngle(RM_3508_GRIPPER); /* 初始化 home_deg。 */
        MotorData = home_deg; /* 定义 MotorData 枚举项。 */
        (void)LoadMotor_SubmitTarget(LOAD_MOTOR_OWNER_HOME, /* 开始调用 LoadMotor_SubmitTarget。 */
                                     LOAD_MOTOR_PRIORITY_HOME, /* 定义 LOAD_MOTOR_PRIORITY_HOME 枚举项。 */
                                     home_deg, /* 传入下一项参数或数据。 */
                                     LOAD_MOTOR_DEFAULT_HOLD_MS); /* 完成本行操作。 */
    }

    xLastWake = xTaskGetTickCount(); /* 更新 xLastWake。 */
    for (;;) /* 遍历当前数据集合。 */
    {
        uint8_t i; /* 保存 i。 */
        float correction = 0.0f; /* 初始化 correction。 */
        MotorRuntimeState_t *left = Motor_GetRuntime(RM_3508_STORE_LEFT); /* 初始化 left。 */
        MotorRuntimeState_t *right = Motor_GetRuntime(RM_3508_STORE_RIGHT); /* 初始化 right。 */

        /* 先处理夹爪命令流（stream-buffer） */
        LoadMotor_ProcessCommandStream(); /* 调用 LoadMotor_ProcessCommandStream。 */

        /* 左右储能同步 PID：两侧都 enable 且未故障时才介入 */
        if (left != NULL && right != NULL && /* 检查当前执行条件。 */
            left->ctrl_enabled && right->ctrl_enabled && /* 继续组合表达式。 */
            !AngleMotor_IsFaulted(RM_3508_STORE_LEFT) && /* 继续组合表达式。 */
            !AngleMotor_IsFaulted(RM_3508_STORE_RIGHT)) /* 继续当前语句。 */
        {
            float left_pos = Motor_GetTotalAngle(RM_3508_STORE_LEFT); /* 初始化 left_pos。 */
            float right_pos = Motor_GetTotalAngle(RM_3508_STORE_RIGHT); /* 初始化 right_pos。 */
            correction = AngleMotor_UpdateStoreSync(left_pos, right_pos); /* 更新 correction。 */
            if (!isfinite(correction)) /* 检查当前执行条件。 */
            {
                correction = 0.0f; /* 更新 correction。 */
            }
        }

        for (i = 1U; i <= (uint8_t)MotorManager.registered_count && i <= (uint8_t)g_CanMotorNum; i++) /* 遍历当前数据集合。 */
        {
            can_motor_cfg cfg = (can_motor_cfg)i; /* 初始化 cfg。 */
            float sync_offset = 0.0f; /* 初始化 sync_offset。 */

            if (cfg == RM_3508_STORE_LEFT) /* 检查当前执行条件。 */
            {
                sync_offset = correction; /* 更新 sync_offset。 */
            }
            else if (cfg == RM_3508_STORE_RIGHT) /* 继续判断下一条件。 */
            {
                sync_offset = -correction; /* 更新 sync_offset。 */
            }

            Motor_TickOne(cfg, sync_offset); /* 调用 Motor_TickOne。 */
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(2)); /* 调用 vTaskDelayUntil。 */
    }
}

/* ------------------------------------------------------------------
 * LoadMotor_* 业务层封装（夹爪 M3508 专用）
 *   - 多 owner 优先级抢占
 *   - stream-buffer 命令流，带 magic+签名校验
 *   - 内部最终 Motor_SetTarget(RM_3508_GRIPPER, ...)
 * ------------------------------------------------------------------ */
static bool LoadMotor_TimeReached(uint32_t now_ms, uint32_t target_ms) /* 实现 LoadMotor_TimeReached。 */
{
    return ((int32_t)(now_ms - target_ms) >= 0); /* 返回当前计算结果。 */
}

static uint32_t LoadMotor_CalcSignature(const LoadMotorCommand_t *cmd) /* 实现 LoadMotor_CalcSignature。 */
{
    uint32_t target_bits = 0U; /* 初始化 target_bits。 */

    if (cmd == NULL) /* 检查当前执行条件。 */
    {
        return 0U; /* 返回状态值 0U。 */
    }
    memcpy(&target_bits, &cmd->target_pos_deg, sizeof(target_bits)); /* 调用 memcpy。 */
    return LOAD_MOTOR_CMD_SIGNATURE_KEY ^ /* 继续组合表达式。 */
           cmd->magic ^ /* 继续组合表达式。 */
           cmd->seq ^ /* 继续组合表达式。 */
           cmd->timestamp_ms ^ /* 继续组合表达式。 */
           cmd->max_age_ms ^ /* 继续组合表达式。 */
           cmd->owner_hold_ms ^ /* 继续组合表达式。 */
           ((uint32_t)cmd->owner << 24) ^ /* 继续组合表达式。 */
           ((uint32_t)cmd->priority << 16) ^ /* 继续组合表达式。 */
           target_bits; /* 完成本行操作。 */
}

static void LoadMotor_ProcessCommandStream(void) /* 实现 LoadMotor_ProcessCommandStream。 */
{
    LoadMotorCommand_t cmd; /* 保存 cmd。 */
    size_t received; /* 保存 received。 */
    uint32_t now_ms; /* 保存 now_ms。 */

    if (g_GripperCmdStream == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    while ((received = xStreamBufferReceive(g_GripperCmdStream, &cmd, sizeof(cmd), 0)) > 0U) /* 条件满足时继续执行。 */
    {
        bool accepted; /* 保存 accepted。 */
        uint32_t hold_ms; /* 保存 hold_ms。 */

        if (received != sizeof(cmd)) /* 检查当前执行条件。 */
        {
            (void)xStreamBufferReset(g_GripperCmdStream); /* 调用 xStreamBufferReset。 */
            break; /* 结束当前循环或分支。 */
        }
        now_ms = HAL_GetTick(); /* 更新 now_ms。 */
        if (cmd.magic != LOAD_MOTOR_CMD_MAGIC || /* 检查当前执行条件。 */
            cmd.signature != LoadMotor_CalcSignature(&cmd) || /* 继续组合表达式。 */
            !isfinite(cmd.target_pos_deg) || /* 继续组合表达式。 */
            (cmd.max_age_ms > 0U && /* 继续组合表达式。 */
             (uint32_t)(now_ms - cmd.timestamp_ms) > cmd.max_age_ms)) /* 继续当前语句。 */
        {
            continue; /* 跳过本轮剩余处理。 */
        }

        taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
        if (g_GripperActiveOwner != LOAD_MOTOR_OWNER_NONE && /* 检查当前执行条件。 */
            LoadMotor_TimeReached(now_ms, g_GripperActiveOwnerExpireMs)) /* 开始调用 LoadMotor_TimeReached。 */
        {
            g_GripperActiveOwner = LOAD_MOTOR_OWNER_NONE; /* 更新 g_GripperActiveOwner。 */
            g_GripperActivePriority = 0U; /* 更新 g_GripperActivePriority。 */
        }
        accepted = (g_GripperActiveOwner == LOAD_MOTOR_OWNER_NONE || /* 继续更新 accepted。 */
                    g_GripperActiveOwner == (LoadMotorOwner_e)cmd.owner || /* 继续更新 目标值。 */
                    cmd.priority >= g_GripperActivePriority); /* 更新 priority。 */
        taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
        if (!accepted) /* 检查当前执行条件。 */
        {
            continue; /* 跳过本轮剩余处理。 */
        }

        hold_ms = (cmd.owner_hold_ms == 0U) ? LOAD_MOTOR_DEFAULT_HOLD_MS : cmd.owner_hold_ms; /* 更新 hold_ms。 */
        Motor_SetTarget(RM_3508_GRIPPER, cmd.target_pos_deg); /* 调用 Motor_SetTarget。 */
        MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER); /* 定义 MotorData 枚举项。 */

        taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
        g_GripperActiveOwner = (LoadMotorOwner_e)cmd.owner; /* 更新 g_GripperActiveOwner。 */
        g_GripperActivePriority = cmd.priority; /* 更新 g_GripperActivePriority。 */
        g_GripperActiveOwnerExpireMs = now_ms + hold_ms; /* 更新 g_GripperActiveOwnerExpireMs。 */
        taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
    }
}

bool LoadMotor_SubmitTarget(LoadMotorOwner_e owner, /* 传入下一项参数或数据。 */
                            uint8_t priority, /* 传入下一项参数或数据。 */
                            float target_pos_deg, /* 传入下一项参数或数据。 */
                            uint32_t owner_hold_ms) /* 继续当前语句。 */
{
    LoadMotorCommand_t cmd = {0}; /* 初始化 cmd。 */
    SemaphoreHandle_t mtx; /* 保存 mtx。 */
    size_t sent; /* 保存 sent。 */
    uint32_t now_ms; /* 保存 now_ms。 */

    if (g_GripperCmdStream == NULL || !isfinite(target_pos_deg)) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }
    if (owner == LOAD_MOTOR_OWNER_NONE || priority == 0U) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }

    now_ms = HAL_GetTick(); /* 更新 now_ms。 */
    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    if (g_GripperActiveOwner != LOAD_MOTOR_OWNER_NONE && /* 检查当前执行条件。 */
        LoadMotor_TimeReached(now_ms, g_GripperActiveOwnerExpireMs)) /* 开始调用 LoadMotor_TimeReached。 */
    {
        g_GripperActiveOwner = LOAD_MOTOR_OWNER_NONE; /* 更新 g_GripperActiveOwner。 */
        g_GripperActivePriority = 0U; /* 更新 g_GripperActivePriority。 */
    }
    if (g_GripperActiveOwner != LOAD_MOTOR_OWNER_NONE && /* 检查当前执行条件。 */
        g_GripperActiveOwner != owner && /* 继续更新 目标值。 */
        priority < g_GripperActivePriority) /* 继续当前语句。 */
    {
        taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
        return false; /* 返回 false。 */
    }
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    cmd.magic = LOAD_MOTOR_CMD_MAGIC; /* 更新 magic。 */
    cmd.timestamp_ms = now_ms; /* 更新 timestamp_ms。 */
    cmd.max_age_ms = LOAD_MOTOR_CMD_MAX_AGE_MS; /* 更新 max_age_ms。 */
    cmd.owner_hold_ms = owner_hold_ms; /* 更新 owner_hold_ms。 */
    cmd.owner = (uint8_t)owner; /* 更新 owner。 */
    cmd.priority = priority; /* 更新 priority。 */
    cmd.target_pos_deg = target_pos_deg; /* 更新 target_pos_deg。 */

    mtx = Motor_GetMutex(RM_3508_GRIPPER); /* 更新 mtx。 */
    if (mtx != NULL) /* 检查当前执行条件。 */
    {
        xSemaphoreTake(mtx, portMAX_DELAY); /* 调用 xSemaphoreTake。 */
    }
    g_GripperCmdSeq++; /* 递增 g_GripperCmdSeq。 */
    cmd.seq = g_GripperCmdSeq; /* 更新 seq。 */
    cmd.signature = LoadMotor_CalcSignature(&cmd); /* 更新 signature。 */

    if (xStreamBufferSpacesAvailable(g_GripperCmdStream) < sizeof(cmd)) /* 检查当前执行条件。 */
    {
        (void)xStreamBufferReset(g_GripperCmdStream); /* 调用 xStreamBufferReset。 */
    }
    sent = xStreamBufferSend(g_GripperCmdStream, &cmd, sizeof(cmd), 0); /* 更新 sent。 */
    if (mtx != NULL) /* 检查当前执行条件。 */
    {
        xSemaphoreGive(mtx); /* 调用 xSemaphoreGive。 */
    }

    return (sent == sizeof(cmd)); /* 返回当前计算结果。 */
}

void LoadMotor_ReleaseOwner(LoadMotorOwner_e owner) /* 实现 LoadMotor_ReleaseOwner。 */
{
    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    if (g_GripperActiveOwner == owner) /* 检查当前执行条件。 */
    {
        g_GripperActiveOwner = LOAD_MOTOR_OWNER_NONE; /* 更新 g_GripperActiveOwner。 */
        g_GripperActivePriority = 0U; /* 更新 g_GripperActivePriority。 */
        g_GripperActiveOwnerExpireMs = 0U; /* 更新 g_GripperActiveOwnerExpireMs。 */
    }
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
}

/// @brief 获取运行时长
/// @param cfg 电机别名
/// @return 运行状态结构体
static inline MotorRuntimeState_t *Motor_GetRuntime(can_motor_cfg cfg) /* 实现 Motor_GetRuntime。 */
{
    if ((uint32_t)cfg < 1U || (uint32_t)cfg > (uint32_t)g_CanMotorNum) /* 检查当前执行条件。 */
    {
        return NULL; /* 返回当前计算结果。 */
    }
    return &s_runtime[(uint32_t)cfg - 1U]; /* 返回当前计算结果。 */
}

/// @brief 获取互斥量操作句柄
/// @param cfg 电机别名
/// @return 互斥量操作句柄
static inline SemaphoreHandle_t Motor_GetMutex(can_motor_cfg cfg) /* 实现 Motor_GetMutex。 */
{
    if ((uint32_t)cfg < 1U || (uint32_t)cfg > (uint32_t)g_CanMotorNum) /* 检查当前执行条件。 */
    {
        return NULL; /* 返回当前计算结果。 */
    }
    return s_mtx[(uint32_t)cfg - 1U]; /* 返回当前计算结果。 */
}

/// @brief 获取电机转动的角度
/// @param cfg 电机别名
/// @return 对应电机角度
static inline float Motor_GetPosRaw(can_motor_cfg cfg) /* 实现 Motor_GetPosRaw。 */
{
    MotorTypeDef *motor = Motor_GetHandleFast(cfg); /* 初始化 motor。 */
    if (motor->MotorInf.band == DM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        /* S3519 规划初值使用已适配的度制逻辑位置，不能读取 DM 的温度槽 [3]。 */
        return Motor_GetTotalAngle(cfg); /* 返回当前计算结果。 */
    }
    return motor->motor_data.solved_data[3]; /* 返回当前计算结果。 */
}
