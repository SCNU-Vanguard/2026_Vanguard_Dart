/**********************************************************
 * 文件名：angle_motor.c
 * 用途：位置控制类电机通用分层框架实现
 * 见 angle_motor.h 头部的分层说明。
 *********************************************************/
#include "angle_motor.h"
#include "config.h"
#include "RM_Motor.h"
#include "DM_Motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/*============================== 后端 compute/send 目标声明 ==============================*/
/* RM 串级：纯控制器（只算电流，不下发、不判到位、不限幅） */
extern float Rm_ComputeCascade(can_motor_cfg cfg, float ref_pos_deg); /* 声明 Rm_ComputeCascade 接口。 */
extern void RmMotorSendCfg(can_motor_cfg motor_cfg, int16_t TargetCurrent); /* 声明 RmMotorSendCfg 接口。 */

/* DM 位置速度：纯控制器（把 ref 转成下发位置，电机内部闭环） */
extern float Dm_ComputePosVel(can_motor_cfg cfg, float ref_pos_deg); /* 声明 Dm_ComputePosVel 接口。 */
extern void DmMotorSendCfg(can_motor_cfg motor_cfg, float TargetPos, float TargetVel, /* 传入下一项参数或数据。 */
                           float TargetTorque, DM_WorkMode workmode); /* 完成本行操作。 */
extern bool DM_Motor_Is3519StallProtected(void); /* 声明 DM_Motor_Is3519StallProtected 接口。 */

/*============================== 后端适配：send 包装 ==============================*/

static void AngleBackend_RmSend(can_motor_cfg cfg, float out) /* 实现 AngleBackend_RmSend。 */
{
    RmMotorSendCfg(cfg, (int16_t)out); /* 调用 RmMotorSendCfg。 */
}

static void AngleBackend_DmSend(can_motor_cfg cfg, float out) /* 实现 AngleBackend_DmSend。 */
{
    MotorTypeDef *motor = Motor_GetHandleFast(cfg); /* 初始化 motor。 */
    float target_rad = DegreeToRad(out); /* 初始化 target_rad。 */
    float speed_rad_s = DegreeToRad(fabsf(StoreSpeed)); /* 初始化 speed_rad_s。 */

    /* 上层和规划器共用 RM 的度制逻辑坐标；仅在 S3519 控制帧边界
     * 转成 rad/rad/s，并把反向安装恢复到电机物理坐标。 */
    if (motor->config.reverse != 0U) /* 检查当前执行条件。 */
    {
        target_rad = -target_rad; /* 更新 target_rad。 */
    }
    DmMotorSendCfg(cfg, target_rad, speed_rad_s, 0.0f, DM_LOCATION_SPEED); /* 调用 DmMotorSendCfg。 */
}

const AngleMotorBackend_t kAngleBackendRmCascade = { /* 初始化 kAngleBackendRmCascade。 */
    .compute = Rm_ComputeCascade, /* 配置 compute。 */
    .send = AngleBackend_RmSend, /* 配置 send。 */
    .use_current_limit = 1U, /* 配置 use_current_limit。 */
};

const AngleMotorBackend_t kAngleBackendDmPosVel = { /* 初始化 kAngleBackendDmPosVel。 */
    .compute = Dm_ComputePosVel, /* 配置 compute。 */
    .send = AngleBackend_DmSend, /* 配置 send。 */
    .use_current_limit = 0U, /* 配置 use_current_limit。 */
};

/*============================== 画像表 ==============================*/
/* 每个位置控制类电机一行。加/换电机只改这里。
 * 储能位 3508↔3519 换装：后端由 AngleMotor_Backend() 按 band 自动解析，
 * 画像无需改动（current_hard_limit 对 DM 后端自动忽略，不做电流限幅）。 */

/* 储能 3508 过流参数 */
#define ANGLE_OC_STORE /* 继续当前语句。 */ \
    {                                           \
        STORE3508_STILL_OVERCURRENT_LIMIT_A, /* 继续传入下一项。 */ \
        STORE3508_STILL_OVERCURRENT_CLEAR_A, /* 继续传入下一项。 */ \
        STORE3508_STILL_OVERCURRENT_CONFIRM_MS, /* 继续传入下一项。 */ \
        STORE3508_STALL_OVERCURRENT_LIMIT_A, /* 继续传入下一项。 */ \
        STORE3508_STALL_OVERCURRENT_CLEAR_A, /* 继续传入下一项。 */ \
        STORE3508_STALL_CONFIRM_MS, /* 继续传入下一项。 */ \
        STORE3508_STALL_SPEED_RPM, /* 继续传入下一项。 */ \
        STORE3508_STALL_POS_DELTA_DEG, /* 继续传入下一项。 */ \
        STORE3508_STALL_POS_SAMPLE_MS, /* 继续传入下一项。 */ \
        STORE3508_OVERCURRENT_TARGET_BLANK_MS, /* 继续传入下一项。 */ \
    }

/* 夹爪 3508 过流参数 */
#define ANGLE_OC_LOAD /* 继续当前语句。 */ \
    {                                         \
        LOAD3508_STILL_OVERCURRENT_LIMIT_A, /* 继续传入下一项。 */ \
        LOAD3508_STILL_OVERCURRENT_CLEAR_A, /* 继续传入下一项。 */ \
        LOAD3508_STILL_OVERCURRENT_RETURN_MS, /* 继续传入下一项。 */ \
        LOAD3508_STALL_OVERCURRENT_LIMIT_A, /* 继续传入下一项。 */ \
        LOAD3508_STALL_OVERCURRENT_CLEAR_A, /* 继续传入下一项。 */ \
        LOAD3508_STALL_OVERCURRENT_RETURN_MS, /* 继续传入下一项。 */ \
        LOAD3508_STALL_SPEED_RPM, /* 继续传入下一项。 */ \
        LOAD3508_STALL_POS_DELTA_DEG, /* 继续传入下一项。 */ \
        LOAD3508_STALL_POS_SAMPLE_MS, /* 继续传入下一项。 */ \
        LOAD3508_OVERCURRENT_TARGET_BLANK_MS, /* 继续传入下一项。 */ \
    }

/* 扳机 2006 专属过流参数 */
#define ANGLE_OC_TRIGGER /* 继续当前语句。 */ \
    {                                             \
        TRIGGER2006_STILL_OVERCURRENT_LIMIT_A, /* 继续传入下一项。 */ \
        TRIGGER2006_STILL_OVERCURRENT_CLEAR_A, /* 继续传入下一项。 */ \
        TRIGGER2006_STILL_OVERCURRENT_CONFIRM_MS, /* 继续传入下一项。 */ \
        TRIGGER2006_STALL_OVERCURRENT_LIMIT_A, /* 继续传入下一项。 */ \
        TRIGGER2006_STALL_OVERCURRENT_CLEAR_A, /* 继续传入下一项。 */ \
        TRIGGER2006_STALL_CONFIRM_MS, /* 继续传入下一项。 */ \
        TRIGGER2006_STALL_SPEED_RPM, /* 继续传入下一项。 */ \
        TRIGGER2006_STALL_POS_DELTA_DEG, /* 继续传入下一项。 */ \
        TRIGGER2006_STALL_POS_SAMPLE_MS, /* 继续传入下一项。 */ \
        TRIGGER2006_OVERCURRENT_TARGET_BLANK_MS, /* 继续传入下一项。 */ \
    }

/* 画像表：每行一个位置控制类电机。
 * 注意：.backend 字段仅作“是否纳管”标记；实际后端由 AngleMotor_Backend()
 * 按注册电机的 band 运行时解析——储能位 3508↔3519 换装零改代码。 */
static const AngleMotorProfile_t s_angle_profiles[g_CanMotorNum] = { /* 初始化 s_angle_profiles。 */
    [RM_3508_GRIPPER - 1] = { /* 继续更新 目标值。 */
        .backend = &kAngleBackendRmCascade, /* 配置 backend。 */
        .oc = ANGLE_OC_LOAD, /* 配置 oc。 */
        .enable_oc = 1U, /* 配置 enable_oc。 */
        .temp_limit_c = 0.0f, // 夹爪不判温度
        .pos_tolerance_deg = GRIPPER_POS_TOLERANCE_DEG, /* 配置 pos_tolerance_deg。 */
        .arrival = ANGLE_MOTOR_ARRIVAL_INFO, /* 配置 arrival。 */
        .current_hard_limit = M3508_CURRENT_HARD_LIMIT, /* 配置 current_hard_limit。 */
    },
    [RM_2006_TRIGGER - 1] = { /* 继续更新 目标值。 */
        .backend = &kAngleBackendRmCascade, /* 配置 backend。 */
        .oc = ANGLE_OC_TRIGGER, /* 配置 oc。 */
        .enable_oc = 1U, /* 配置 enable_oc。 */
        .temp_limit_c = 0.0f, /* 配置 temp_limit_c。 */
        .pos_tolerance_deg = TRIGGER_POS_TOLERANCE_DEG, /* 配置 pos_tolerance_deg。 */
        .arrival = ANGLE_MOTOR_ARRIVAL_INFO, /* 配置 arrival。 */
        .current_hard_limit = M2006_CURRENT_HARD_LIMIT, /* 配置 current_hard_limit。 */
    },
    [RM_3508_STORE_RIGHT - 1] = { /* 继续更新 目标值。 */
        .backend = &kAngleBackendRmCascade, /* 配置 backend。 */
        .oc = ANGLE_OC_STORE, /* 配置 oc。 */
        .enable_oc = 1U, /* 配置 enable_oc。 */
        .temp_limit_c = STORE3508_TEMP_LIMIT_C, /* 配置 temp_limit_c。 */
        .pos_tolerance_deg = STORE_POS_TOLERANCE_DEG, /* 配置 pos_tolerance_deg。 */
        .arrival = ANGLE_MOTOR_ARRIVAL_WARNING, // 顶住拉簧不撒手
        .current_hard_limit = M3508_CURRENT_HARD_LIMIT, /* 配置 current_hard_limit。 */
    },
    [RM_3508_STORE_LEFT - 1] = { /* 继续更新 目标值。 */
        .backend = &kAngleBackendRmCascade, /* 配置 backend。 */
        .oc = ANGLE_OC_STORE, /* 配置 oc。 */
        .enable_oc = 1U, /* 配置 enable_oc。 */
        .temp_limit_c = STORE3508_TEMP_LIMIT_C, /* 配置 temp_limit_c。 */
        .pos_tolerance_deg = STORE_POS_TOLERANCE_DEG, /* 配置 pos_tolerance_deg。 */
        .arrival = ANGLE_MOTOR_ARRIVAL_WARNING, /* 配置 arrival。 */
        .current_hard_limit = M3508_CURRENT_HARD_LIMIT, /* 配置 current_hard_limit。 */
    },
    /* DM_4310_YAW 不纳入本框架（MIT 力控，性质不同），backend 留 NULL。 */
};

/*============================== 运行时表 ==============================*/

static AngleMotorRuntime_t s_angle_rt[g_CanMotorNum]; /* 保存 s_angle_rt。 */

/// @brief 根据电机枚举编号 cfg，找到该电机在 angle_motor 框架中的运行时状态结构体
/// @param cfg
/// @return AngleMotorRuntime_t * 运行状态结构体指针
static inline AngleMotorRuntime_t *AngleMotor_GetRt(can_motor_cfg cfg) /* 实现 AngleMotor_GetRt。 */
{
    if ((uint32_t)cfg < 1U || (uint32_t)cfg > (uint32_t)g_CanMotorNum) /* 检查当前执行条件。 */
    {
        return NULL; /* 返回当前计算结果。 */
    }
    return &s_angle_rt[(uint32_t)cfg - 1U]; /* 返回当前计算结果。 */
}

/*============================== 画像/管理查询 ==============================*/

const AngleMotorProfile_t *AngleMotor_GetProfile(can_motor_cfg cfg) /* 实现 AngleMotor_GetProfile。 */
{
    if ((uint32_t)cfg < 1U || (uint32_t)cfg > (uint32_t)g_CanMotorNum) /* 检查当前执行条件。 */
    {
        return NULL; /* 返回当前计算结果。 */
    }
    const AngleMotorProfile_t *p = &s_angle_profiles[(uint32_t)cfg - 1U]; /* 初始化 p。 */
    return (p->backend != NULL) ? p : NULL; /* 返回当前计算结果。 */
}

bool AngleMotor_IsManaged(can_motor_cfg cfg) /* 实现 AngleMotor_IsManaged。 */
{
    return AngleMotor_GetProfile(cfg) != NULL; /* 返回当前计算结果。 */
}

/// @brief 按实际注册电机的 band 解析后端（RM 串级 / DM 位置速度）
/// @note  储能位 3508↔3519 共用枚举槽位，band 不同 → 换电机零改代码。
///        画像里的 backend 字段仅作“是否纳管”标记，实际后端以此为准。
static const AngleMotorBackend_t *AngleMotor_Backend(can_motor_cfg cfg) /* 实现 AngleMotor_Backend。 */
{
    MotorTypeDef *motor = Motor_GetHandleFast(cfg); /* 初始化 motor。 */
    if (motor->MotorInf.band == DM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        return &kAngleBackendDmPosVel; /* 返回当前计算结果。 */
    }
    return &kAngleBackendRmCascade; /* 返回当前计算结果。 */
}

/*============================== 状态判断器（纯函数，不碰 CAN） ==============================*/

AngleMotorDecision_t AngleMotor_Judge(can_motor_cfg cfg, uint32_t now_ms, /* 传入下一项参数或数据。 */
                                      float ref_pos_deg, float target_pos_deg) /* 继续当前语句。 */
{
    AngleMotorDecision_t dec = {ANGLE_MOTOR_NORMAL, false}; /* 初始化 dec。 */
    const AngleMotorProfile_t *prof = AngleMotor_GetProfile(cfg); /* 初始化 prof。 */
    AngleMotorRuntime_t *rt = AngleMotor_GetRt(cfg); /* 初始化 rt。 */
    MotorTypeDef *motor; /* 保存 motor。 */
    float current_a; /* 保存 current_a。 */
    float current_pos_deg; /* 保存 current_pos_deg。 */
    float speed_rpm; /* 保存 speed_rpm。 */
    float abs_current_a; /* 保存 abs_current_a。 */
    bool near_target; /* 保存 near_target。 */
    bool stall_like = false; /* 初始化 stall_like。 */

    (void)ref_pos_deg; /* 显式忽略参数 ref_pos_deg。 */

    if (prof == NULL || rt == NULL) /* 检查当前执行条件。 */
    {
        return dec; /* 返回当前计算结果。 */
    }
    motor = Motor_GetHandleFast(cfg); /* 更新 motor。 */

    /* 0) 已闩锁：直接 ERROR，直到显式清除 */
    if (rt->fault_latched) /* 检查当前执行条件。 */
    {
        dec.level = ANGLE_MOTOR_ERROR; /* 更新 level。 */
        return dec; /* 返回当前计算结果。 */
    }

    /* DM 后端（3519）：过流/温度保护走电机反馈里的 DM_Check3519StallInFeedback，
     * 这里只吸收其结论，避免重复实现。 */
    if (AngleMotor_Backend(cfg) == &kAngleBackendDmPosVel) /* 检查当前执行条件。 */
    {
        if (DM_Motor_Is3519StallProtected()) /* 检查当前执行条件。 */
        {
            rt->fault_latched = 1U; /* 更新 fault_latched。 */
            dec.level = ANGLE_MOTOR_ERROR; /* 更新 level。 */
            return dec; /* 返回当前计算结果。 */
        }
    }

    current_a = Motor_GetCurrent(cfg); /* 更新 current_a。 */
    current_pos_deg = Motor_GetTotalAngle(cfg); /* 更新 current_pos_deg。 */
    speed_rpm = Motor_GetSpeedRPM(cfg); /* 更新 speed_rpm。 */

    if (!isfinite(current_a) || !isfinite(current_pos_deg) || !isfinite(speed_rpm)) /* 检查当前执行条件。 */
    {
        rt->over_current_start_ms = 0U; /* 更新 over_current_start_ms。 */
        rt->stall_over_current_start_ms = 0U; /* 更新 stall_over_current_start_ms。 */
        /* 传感器异常，保守判 NORMAL，不误触发保护 */
        return dec; /* 返回当前计算结果。 */
    }

    /* 1) 温度保护（仅储能 3508，solved_data[5] 是温度） */
    if (prof->temp_limit_c > 0.0f && /* 检查当前执行条件。 */
        AngleMotor_Backend(cfg) == &kAngleBackendRmCascade) /* 继续更新 目标值。 */
    {
        float temperature_c = motor->motor_data.solved_data[5]; /* 初始化 temperature_c。 */
        if (isfinite(temperature_c) && temperature_c > prof->temp_limit_c) /* 检查当前执行条件。 */
        {
            rt->fault_latched = 1U; /* 更新 fault_latched。 */
            dec.level = ANGLE_MOTOR_ERROR; /* 更新 level。 */
            return dec; /* 返回当前计算结果。 */
        }
    }

    /* 2) 过流/堵转（RM 后端，enable_oc；DM 走上面的反馈保护） */
    if (prof->enable_oc && AngleMotor_Backend(cfg) == &kAngleBackendRmCascade) /* 检查当前执行条件。 */
    {
        const AngleMotorOcParams_t *oc = &prof->oc; /* 初始化 oc。 */

        /* Motor_GetCurrent() 已提供底层处理后的电流，判断层不再重复滤波。 */
        abs_current_a = fabsf(current_a); /* 更新 abs_current_a。 */
        near_target = (fabsf(target_pos_deg - current_pos_deg) <= MOTOR_DEAD_ZONE); /* 更新 near_target。 */
        /* 目标切换空白期由 AngleMotor_NotifyTargetChanged 复位计时实现 */

        if (rt->stall_sample_ms == 0U || /* 检查当前执行条件。 */
            (uint32_t)(now_ms - rt->stall_sample_ms) >= oc->stall_pos_sample_ms) /* 继续更新 目标值。 */
        {
            float pos_delta = fabsf(current_pos_deg - rt->stall_sample_pos_deg); /* 初始化 pos_delta。 */
            stall_like = (pos_delta <= oc->stall_pos_delta_deg); /* 更新 stall_like。 */
            rt->stall_sample_pos_deg = current_pos_deg; /* 更新 stall_sample_pos_deg。 */
            rt->stall_sample_ms = now_ms; /* 更新 stall_sample_ms。 */
        }
        else /* 处理其余情况。 */
        {
            stall_like = (fabsf(speed_rpm) <= oc->stall_speed_rpm); /* 更新 stall_like。 */
        }

        if (near_target) /* 检查当前执行条件。 */
        {
            rt->stall_over_current_start_ms = 0U; /* 更新 stall_over_current_start_ms。 */
            if (abs_current_a <= oc->still_clear_a) /* 检查当前执行条件。 */
            {
                rt->over_current_start_ms = 0U; /* 更新 over_current_start_ms。 */
            }
            else if (abs_current_a > oc->still_limit_a) /* 继续判断下一条件。 */
            {
                if (rt->over_current_start_ms == 0U) /* 检查当前执行条件。 */
                {
                    rt->over_current_start_ms = now_ms; /* 更新 over_current_start_ms。 */
                }
                else if ((uint32_t)(now_ms - rt->over_current_start_ms) >= oc->still_confirm_ms) /* 继续判断下一条件。 */
                {
                    rt->fault_latched = 1U; /* 更新 fault_latched。 */
                    dec.level = ANGLE_MOTOR_ERROR; /* 更新 level。 */
                    return dec; /* 返回当前计算结果。 */
                }
            }
        }
        else /* 处理其余情况。 */
        {
            rt->over_current_start_ms = 0U; /* 更新 over_current_start_ms。 */
            if (abs_current_a <= oc->stall_clear_a || !stall_like) /* 检查当前执行条件。 */
            {
                rt->stall_over_current_start_ms = 0U; /* 更新 stall_over_current_start_ms。 */
            }
            else if (abs_current_a > oc->stall_limit_a) /* 继续判断下一条件。 */
            {
                if (rt->stall_over_current_start_ms == 0U) /* 检查当前执行条件。 */
                {
                    rt->stall_over_current_start_ms = now_ms; /* 更新 stall_over_current_start_ms。 */
                }
                else if ((uint32_t)(now_ms - rt->stall_over_current_start_ms) >= oc->stall_confirm_ms) /* 继续判断下一条件。 */
                {
                    rt->fault_latched = 1U; /* 更新 fault_latched。 */
                    dec.level = ANGLE_MOTOR_ERROR; /* 更新 level。 */
                    return dec; /* 返回当前计算结果。 */
                }
            }
        }
    }

    /* 3) 到位判定 */
    if (fabsf(target_pos_deg - current_pos_deg) <= prof->pos_tolerance_deg) /* 检查当前执行条件。 */
    {
        dec.arrived = true; /* 更新 arrived。 */
        rt->arrived = 1U; /* 更新 arrived。 */
        dec.level = (prof->arrival == ANGLE_MOTOR_ARRIVAL_WARNING) /* 继续更新 level。 */
                        ? ANGLE_MOTOR_WARNING /* 继续当前语句。 */
                        : ANGLE_MOTOR_INFO; /* 完成本行操作。 */
    }
    else /* 处理其余情况。 */
    {
        rt->arrived = 0U; /* 更新 arrived。 */
        dec.level = ANGLE_MOTOR_NORMAL; /* 更新 level。 */
    }

    return dec; /* 返回当前计算结果。 */
}

/*============================== 限幅器 ==============================*/

float AngleMotor_LimitOutput(can_motor_cfg cfg, float raw) /* 实现 AngleMotor_LimitOutput。 */
{
    const AngleMotorProfile_t *prof = AngleMotor_GetProfile(cfg); /* 初始化 prof。 */
    MotorTypeDef *motor; /* 保存 motor。 */
    float out = raw; /* 初始化 out。 */
    float pos; /* 保存 pos。 */
    float lim; /* 保存 lim。 */

    if (prof == NULL) /* 检查当前执行条件。 */
    {
        return raw; /* 返回当前计算结果。 */
    }

    /* DM 位置速度后端：out 是度制位置指令，按与 RM 共用的逻辑位置范围
     * 直接夹紧；电流/力矩保护由 S3519 反馈保护完成。 */
    if (!AngleMotor_Backend(cfg)->use_current_limit) /* 检查当前执行条件。 */
    {
        motor = Motor_GetHandleFast(cfg); /* 更新 motor。 */
        if (isfinite(motor->config.position_min) && /* 检查当前执行条件。 */
            isfinite(motor->config.position_max)) /* 开始调用 isfinite。 */
        {
            float pmin = motor->config.position_min; /* 初始化 pmin。 */
            float pmax = motor->config.position_max; /* 初始化 pmax。 */
            if (pmin > pmax) /* 检查当前执行条件。 */
            {
                float t = pmin; /* 初始化 t。 */
                pmin = pmax; /* 更新 pmin。 */
                pmax = t; /* 更新 pmax。 */
            }
            if (out > pmax) /* 检查当前执行条件。 */
            {
                out = pmax; /* 更新 out。 */
            }
            else if (out < pmin) /* 继续判断下一条件。 */
            {
                out = pmin; /* 更新 out。 */
            }
        }
        return out; /* 返回当前计算结果。 */
    }

    motor = Motor_GetHandleFast(cfg); /* 更新 motor。 */

    /* 方向限位：到极限且输出继续往极限方向 → 阻止；反向放行。
     * 用注册的相对位置限位 config.position_min/max。 */
    pos = Motor_GetTotalAngle(cfg); /* 更新 pos。 */
    if (isfinite(motor->config.position_min) && isfinite(motor->config.position_max)) /* 检查当前执行条件。 */
    {
        float pmin = motor->config.position_min; /* 初始化 pmin。 */
        float pmax = motor->config.position_max; /* 初始化 pmax。 */
        if (pmin > pmax) /* 检查当前执行条件。 */
        {
            float t = pmin; /* 初始化 t。 */
            pmin = pmax; /* 更新 pmin。 */
            pmax = t; /* 更新 pmax。 */
        }
        if (pos >= pmax && out > 0.0f) /* 检查当前执行条件。 */
        {
            out = 0.0f; /* 更新 out。 */
        }
        else if (pos <= pmin && out < 0.0f) /* 继续判断下一条件。 */
        {
            out = 0.0f; /* 更新 out。 */
        }
    }

    /* ±current_hard_limit 硬限幅（执行器安全网） */
    lim = (float)prof->current_hard_limit; /* 更新 lim。 */
    if (lim > 0.0f) /* 检查当前执行条件。 */
    {
        if (out > lim) /* 检查当前执行条件。 */
        {
            out = lim; /* 更新 out。 */
        }
        else if (out < -lim) /* 继续判断下一条件。 */
        {
            out = -lim; /* 更新 out。 */
        }
    }

    return out; /* 返回当前计算结果。 */
}

/*============================== 完整 tick ==============================*/

AngleMotorStateLevel_e AngleMotor_Drive(can_motor_cfg cfg, /* 传入下一项参数或数据。 */
                                        float ref_pos_deg, /* 传入下一项参数或数据。 */
                                        float target_pos_deg) /* 继续当前语句。 */
{
    const AngleMotorProfile_t *prof = AngleMotor_GetProfile(cfg); /* 初始化 prof。 */
    AngleMotorRuntime_t *rt = AngleMotor_GetRt(cfg); /* 初始化 rt。 */
    const AngleMotorBackend_t *backend; /* 保存 backend。 */
    AngleMotorDecision_t dec; /* 保存 dec。 */
    float raw; /* 保存 raw。 */
    float out; /* 保存 out。 */

    if (prof == NULL || rt == NULL) /* 检查当前执行条件。 */
    {
        return ANGLE_MOTOR_NORMAL; /* 返回当前计算结果。 */
    }
    backend = AngleMotor_Backend(cfg); /* 按实际电机 band 解析（3508/3519 自适应） */

    /* Judge：先判分级（含 ERROR 闩锁、到位分级） */
    dec = AngleMotor_Judge(cfg, HAL_GetTick(), ref_pos_deg, target_pos_deg); /* 更新 dec。 */

    /* ERROR：输出 0，闩锁保持，不进控制器 */
    if (dec.level == ANGLE_MOTOR_ERROR) /* 检查当前执行条件。 */
    {
        if (backend == &kAngleBackendDmPosVel) /* 检查当前执行条件。 */
        {
            MotorTypeDef *m = Motor_GetHandleFast(cfg); /* 初始化 m。 */
            /* 对位置后端发送 0 会变成回零命令；故障时应失能 S3519。 */
            if (m->drive_enabled != 0U) /* 检查当前执行条件。 */
            {
                (void)DM_MotorDisable(cfg); /* 调用 DM_MotorDisable。 */
            }
        }
        else /* 处理其余情况。 */
        {
            backend->send(cfg, 0.0f); /* 完成本行操作。 */
        }
        return ANGLE_MOTOR_ERROR; /* 返回当前计算结果。 */
    }

    /* INFO 到位：轻载 → 输出 0 + 清积分（RM 串级清积分，DM 无积分） */
    if (dec.level == ANGLE_MOTOR_INFO) /* 检查当前执行条件。 */
    {
        if (backend == &kAngleBackendRmCascade) /* 检查当前执行条件。 */
        {
            MotorTypeDef *m = Motor_GetHandleFast(cfg); /* 初始化 m。 */
            CASCADE_PID_Clear_Integral(&m->cascade_pid); /* 调用 CASCADE_PID_Clear_Integral。 */
        }
        if (backend == &kAngleBackendDmPosVel) /* 检查当前执行条件。 */
        {
            backend->send(cfg, target_pos_deg); /* 完成本行操作。 */
        }
        else /* 处理其余情况。 */
        {
            backend->send(cfg, 0.0f); /* 完成本行操作。 */
        }
        return ANGLE_MOTOR_INFO; /* 返回当前计算结果。 */
    }

    /* NORMAL / WARNING：都走控制器闭环。WARNING 到位仍出力，顶住负载不撒手。 */
    raw = backend->compute(cfg, ref_pos_deg); /* 更新 raw。 */
    out = AngleMotor_LimitOutput(cfg, raw); /* 更新 out。 */
    backend->send(cfg, out); /* 完成本行操作。 */

    return dec.level; /* 返回当前计算结果。 */
}

/*============================== 故障/到位/运行时管理 ==============================*/

bool AngleMotor_IsFaulted(can_motor_cfg cfg) /* 实现 AngleMotor_IsFaulted。 */
{
    AngleMotorRuntime_t *rt = AngleMotor_GetRt(cfg); /* 初始化 rt。 */
    return !Motor_IsOnline(cfg) || ((rt != NULL) && (rt->fault_latched != 0U)); /* 返回当前计算结果。 */
}

void AngleMotor_ClearFault(can_motor_cfg cfg) /* 实现 AngleMotor_ClearFault。 */
{
    AngleMotorRuntime_t *rt = AngleMotor_GetRt(cfg); /* 初始化 rt。 */
    if (rt == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }
    if (AngleMotor_Backend(cfg) == &kAngleBackendDmPosVel) /* 检查当前执行条件。 */
    {
        DM_Motor_Clear3519StallProtection(); /* 调用 DM_Motor_Clear3519StallProtection。 */
    }
    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    rt->fault_latched = 0U; /* 更新 fault_latched。 */
    rt->over_current_start_ms = 0U; /* 更新 over_current_start_ms。 */
    rt->stall_over_current_start_ms = 0U; /* 更新 stall_over_current_start_ms。 */
    rt->stall_sample_ms = 0U; /* 更新 stall_sample_ms。 */
    rt->stall_sample_pos_deg = Motor_GetTotalAngle(cfg); /* 更新 stall_sample_pos_deg。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
}

void AngleMotor_NotifyTargetChanged(can_motor_cfg cfg) /* 实现 AngleMotor_NotifyTargetChanged。 */
{
    AngleMotorRuntime_t *rt = AngleMotor_GetRt(cfg); /* 初始化 rt。 */
    if (rt == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }
    /* 换目标：复位过流计时与堵转采样，避免跨目标误判；到位标志清 0。
     * 不清 fault_latched（故障必须显式 ClearFault）。 */
    rt->over_current_start_ms = 0U; /* 更新 over_current_start_ms。 */
    rt->stall_over_current_start_ms = 0U; /* 更新 stall_over_current_start_ms。 */
    rt->stall_sample_ms = 0U; /* 更新 stall_sample_ms。 */
    rt->stall_sample_pos_deg = Motor_GetTotalAngle(cfg); /* 更新 stall_sample_pos_deg。 */
    rt->arrived = 0U; /* 更新 arrived。 */
}

void AngleMotor_ResetRuntime(can_motor_cfg cfg) /* 实现 AngleMotor_ResetRuntime。 */
{
    AngleMotorRuntime_t *rt = AngleMotor_GetRt(cfg); /* 初始化 rt。 */
    if (rt == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }
    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    rt->over_current_start_ms = 0U; /* 更新 over_current_start_ms。 */
    rt->stall_over_current_start_ms = 0U; /* 更新 stall_over_current_start_ms。 */
    rt->stall_sample_ms = 0U; /* 更新 stall_sample_ms。 */
    rt->stall_sample_pos_deg = Motor_GetTotalAngle(cfg); /* 更新 stall_sample_pos_deg。 */
    rt->fault_latched = 0U; /* 更新 fault_latched。 */
    rt->arrived = 0U; /* 更新 arrived。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
}

/*============================== 左右蓄力同步 PID（通用命名封装） ==============================*/
/* 内部沿用 RM_Motor.c 里已标定好的同步 PID 实现，这里只提供后端无关的
 * 通用命名，使左右蓄力位无论装 3508 还是 DM 3519 备用都用同一入口。 */

void AngleMotor_InitStoreSyncPid(float kp, float ki, float kd, float kf, /* 传入下一项参数或数据。 */
                                 float max_out, float min_out, float max_iout) /* 继续当前语句。 */
{
    RM_Motor_InitStoreSyncPid(kp, ki, kd, kf, max_out, min_out, max_iout); /* 调用 RM_Motor_InitStoreSyncPid。 */
}

float AngleMotor_UpdateStoreSync(float left_pos_deg, float right_pos_deg) /* 实现 AngleMotor_UpdateStoreSync。 */
{
    return RM_Motor_UpdateStoreSync(left_pos_deg, right_pos_deg); /* 返回当前计算结果。 */
}
