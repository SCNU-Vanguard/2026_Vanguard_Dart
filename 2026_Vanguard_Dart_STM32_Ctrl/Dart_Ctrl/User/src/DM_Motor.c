/*****************************************************
 * DM电机（达妙电机）控制模块
 * 适配H35系列电机（DM3519、DM4310等）
 * --------------------------------------------------
 * DM电机说明：
 * 支持MIT模式、位置速度模式、速度模式、PVT模式
 * 目前主要使用MIT模式进行控制
 * --------------------------------------------------
 * 面向对象设计说明：
 * 使用 DM_MotorClass_t 作为电机"类"，包含：
 * - 电机默认参数（限幅参数、默认KP/KD/力矩）
 * - 虚函数表（初始化、解算、发送控制等）
 * 预定义 DM_J3519_Class、DM_J4310_Class 两个类实例
 ****************************************************/

#include "DM_Motor.h"
#include "CanMotor.h"
#include "config.h"
#include "bsp_dwt.h"
#include <math.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

// 直接访问电机管理器（减少函数调用开销）
extern MotorManager_t MotorManager; /* 声明外部变量 MotorManager。 */
static float s_dm_motor_pid_output = 0.0f; /* 初始化 s_dm_motor_pid_output。 */

// 注意：不再直接调用硬件函数，改用 Motor_GetHAL() 接口

#define CtrlMotorLen 8 /* 定义 CtrlMotorLen。 */
#define DM_SPEED_FILTER_COEF 0.0f // 达妙电机本身很准确
#define DM3519_STALL_TORQUE_LIMIT_NM 5.0f /* 定义 DM3519_STALL_TORQUE_LIMIT_NM。 */
#define DM3519_STALL_TORQUE_CLEAR_NM 4.0f /* 定义 DM3519_STALL_TORQUE_CLEAR_NM。 */
#define DM3519_STALL_CONFIRM_MS 2500U /* 定义 DM3519_STALL_CONFIRM_MS。 */
#define DM3519_COIL_TEMP_LIMIT_C 50.0f /* 定义 DM3519_COIL_TEMP_LIMIT_C。 */

// DM电机控制帧
static const uint8_t DM_MOTOR_ENABLE[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}; /* 初始化 DM_MOTOR_ENABLE。 */
static const uint8_t DM_MOTOR_DISABLE[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD}; /* 初始化 DM_MOTOR_DISABLE。 */

// DM电机配置存储（非static，供头文件内联函数使用）
DM_MotorConfig_t g_DM_Configs[4] = {0}; /* 初始化 g_DM_Configs。 */

typedef struct /* 开始定义数据类型。 */
{
    uint32_t over_current_start_ms; /* 保存 over_current_start_ms。 */
} DM3519_StallState_t; /* 结束 DM3519_StallState_t 类型定义。 */

static volatile uint8_t s_dm3519_stall_protected = 0U; /* 初始化 s_dm3519_stall_protected。 */
static DM3519_StallState_t s_dm3519_stall_state[2] = {0}; /* 初始化 s_dm3519_stall_state。 */

static inline float DM_ClampFloat(float value, float min, float max) /* 实现 DM_ClampFloat。 */
{
    if (value < min) /* 检查当前执行条件。 */
        return min; /* 返回当前计算结果。 */
    if (value > max) /* 检查当前执行条件。 */
        return max; /* 返回当前计算结果。 */
    return value; /* 返回当前计算结果。 */
}

static inline float DM_ApplyOutputLimit(MotorTypeDef *motor, float pid_output) /* 实现 DM_ApplyOutputLimit。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
        return 0.0f; /* 返回当前计算结果。 */

    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class; /* 初始化 cls。 */
    if (cls != NULL) /* 检查当前执行条件。 */
    {
        return DM_ClampFloat(pid_output, cls->torque_min, cls->torque_max); /* 返回当前计算结果。 */
    }

    return pid_output; /* 返回当前计算结果。 */
}

static inline void DM_HandleAngleLoopDirectionChange(MotorTypeDef *motor, float target_angle, float current_angle) /* 实现 DM_HandleAngleLoopDirectionChange。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    PID_t *outer_pid = &motor->cascade_pid.outer; /* 初始化 outer_pid。 */
    if (!outer_pid->initialized || outer_pid->calc_count == 0U) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    const float angle_error = target_angle - current_angle; /* 初始化 angle_error。 */
    const float current_target_delta = target_angle - outer_pid->target; /* 初始化 current_target_delta。 */
    const bool target_direction_reversed = (current_target_delta * outer_pid->feedforward) < 0.0f; /* 初始化 target_direction_reversed。 */
    const bool error_direction_reversed = (angle_error * outer_pid->last_error) < 0.0f; /* 初始化 error_direction_reversed。 */

    if (target_direction_reversed || error_direction_reversed) /* 检查当前执行条件。 */
    {
        PID_Clear_Integral(outer_pid); /* 调用 PID_Clear_Integral。 */
    }
}

static int DM_Get3519PairIndexByMotorCfg(can_motor_cfg motor_cfg) /* 实现 DM_Get3519PairIndexByMotorCfg。 */
{
    if (motor_cfg == DM_3519_STRENTH_LEFT) /* 检查当前执行条件。 */
        return 0; /* 返回状态值 0。 */
    if (motor_cfg == DM_3519_STRENTH_RIGHT) /* 检查当前执行条件。 */
        return 1; /* 返回状态值 1。 */
    return -1; /* 返回当前计算结果。 */
}

static int DM_Get3519PairIndex(const MotorTypeDef *motor) /* 实现 DM_Get3519PairIndex。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
        return -1; /* 返回当前计算结果。 */

    const uint8_t motor_index = (uint8_t)(motor - MotorManager.MotorList); /* 初始化 motor_index。 */
    const can_motor_cfg motor_cfg = (can_motor_cfg)(motor_index + 1U); /* 初始化 motor_cfg。 */
    return DM_Get3519PairIndexByMotorCfg(motor_cfg); /* 返回当前计算结果。 */
}

static void DM_Reset3519StallStateByIndex(int index, float current_pos) /* 实现 DM_Reset3519StallStateByIndex。 */
{
    (void)current_pos; /* 显式忽略参数 current_pos。 */
    if (index < 0 || index >= 2) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    s_dm3519_stall_state[index].over_current_start_ms = 0U; /* 更新 over_current_start_ms。 */
}

static void DM_Record3519TargetChange(can_motor_cfg motor_cfg, float target_pos) /* 实现 DM_Record3519TargetChange。 */
{
    (void)target_pos; /* 显式忽略参数 target_pos。 */
    DM_Reset3519StallStateByIndex(DM_Get3519PairIndexByMotorCfg(motor_cfg), /* 传入下一项参数或数据。 */
                                  MotorManager.MotorList[motor_cfg - 1].motor_data.solved_data[0]); /* 完成本行操作。 */
}

static void DM_Check3519StallInFeedback(MotorTypeDef *motor) /* 实现 DM_Check3519StallInFeedback。 */
{
    const int index = DM_Get3519PairIndex(motor); /* 初始化 index。 */
    if (index < 0 || motor == NULL) /* 检查当前执行条件。 */
        return; /* 无效句柄。 */

    MotorSolvedData_t *pData = &motor->motor_data; /* 初始化 pData。 */
    DM3519_StallState_t *state = &s_dm3519_stall_state[index]; /* 初始化 state。 */
    const float torque_nm = pData->solved_data[2]; /* 初始化 torque_nm。 */
    const float coil_temp_c = pData->solved_data[4]; /* 初始化 coil_temp_c。 */
    const uint32_t now_ms = HAL_GetTick(); /* 初始化 now_ms。 */

    if (!isfinite(torque_nm) || !isfinite(coil_temp_c)) /* 检查当前执行条件。 */
    {
        state->over_current_start_ms = 0U; /* 更新 over_current_start_ms。 */
        return; /* 结束当前函数。 */
    }

    if (coil_temp_c > DM3519_COIL_TEMP_LIMIT_C) /* 检查当前执行条件。 */
    {
        s_dm3519_stall_protected = 1U; /* 更新 s_dm3519_stall_protected。 */
        state->over_current_start_ms = 0U; /* 更新 over_current_start_ms。 */
        return; /* 结束当前函数。 */
    }

    if (fabsf(torque_nm) <= DM3519_STALL_TORQUE_CLEAR_NM) /* 检查当前执行条件。 */
    {
        state->over_current_start_ms = 0U; /* 更新 over_current_start_ms。 */
        return; /* 结束当前函数。 */
    }

    if (fabsf(torque_nm) >= DM3519_STALL_TORQUE_LIMIT_NM) /* 检查当前执行条件。 */
    {
        if (state->over_current_start_ms == 0U) /* 检查当前执行条件。 */
        {
            state->over_current_start_ms = now_ms; /* 更新 over_current_start_ms。 */
            return; /* 结束当前函数。 */
        }

        if ((uint32_t)(now_ms - state->over_current_start_ms) >= DM3519_STALL_CONFIRM_MS) /* 检查当前执行条件。 */
        {
            s_dm3519_stall_protected = 1U; /* 更新 s_dm3519_stall_protected。 */
        }
    }
    else /* 处理其余情况。 */
    {
        state->over_current_start_ms = 0U; /* 更新 over_current_start_ms。 */
    }
}

/*============================== DM电机ID配置表 ==============================*/
// 集中管理所有DM电机的CAN ID配置，修改ID只需改这里
// motor_id: 达妙上位机设置的ID, tx_id: 发送ID, rx_id: 接收ID, work_mode: 工作模式
static const DM_MotorIdConfig_t g_DM_IdTable[] = { /* 初始化 g_DM_IdTable。 */
    // 左侧蓄力电机 - J3519 (位置速度模式, ID=1)
    [DM_3519_STRENTH_LEFT] = {1, 0x108, 0x028, DM_LOCATION_SPEED}, /* 继续更新 目标值。 */
    // 右侧蓄力电机 - J3519 (位置速度模式, ID=2)
    [DM_3519_STRENTH_RIGHT] = {2, 0x109, 0x029, DM_LOCATION_SPEED}, /* 继续更新 目标值。 */
    // Yaw轴电机 - J4310 (MIT模式, ID=3)
    [DM_4310_YAW] = {3, 0x009, 0x019, DM_MIT}, /* 继续更新 目标值。 */
};

const DM_MotorIdConfig_t *DM_GetIdConfig(can_motor_cfg motor_cfg) /* 实现 DM_GetIdConfig。 */
{
    if (motor_cfg == DM_3519_STRENTH_RIGHT || /* 检查当前执行条件。 */
        motor_cfg == DM_3519_STRENTH_LEFT || /* 继续更新 目标值。 */
        motor_cfg == DM_4310_YAW) /* 继续更新 目标值。 */
    {
        return &g_DM_IdTable[motor_cfg]; /* 返回当前计算结果。 */
    }
    return NULL; /* 返回当前计算结果。 */
}

/*============================== 静态函数声明（私有方法） ==============================*/
// 达妙电机通用函数
void DM_Motor_RefreshData(can_motor_cfg motor_cfg); /* 声明 DM_Motor_RefreshData 接口。 */

// J3519专用函数
static void DM_J3519_InitInternal(MotorTypeDef *motor, uint8_t id); /* 声明 DM_J3519_InitInternal 接口。 */
static void DM_J3519_CalculateInternal(MotorTypeDef *motor); /* 声明 DM_J3519_CalculateInternal 接口。 */

// J4310专用函数
static void DM_J4310_InitInternal(MotorTypeDef *motor, uint8_t id); /* 声明 DM_J4310_InitInternal 接口。 */
static void DM_J4310_CalculateInternal(MotorTypeDef *motor); /* 声明 DM_J4310_CalculateInternal 接口。 */

// 通用发送控制
static uint8_t DM_Motor_SendControlInternal(MotorTypeDef *motor); /* 声明 DM_Motor_SendControlInternal 接口。 */

/*============================== 电机类静态实例定义 ==============================*/

/// @brief J3519电机类
const DM_MotorClass_t DM_J3519_Class = { /* 初始化 DM_J3519_Class。 */
    .name = "J3519", /* 配置 name。 */
    .model = DmS3519, /* 配置 model。 */
    // 限幅参数
    .kp_max = 500.0f, /* 配置 kp_max。 */
    .kp_min = 0.0f, /* 配置 kp_min。 */
    .kd_max = 5.0f, /* 配置 kd_max。 */
    .kd_min = 0.0f, /* 配置 kd_min。 */
    .pos_max = 1500.0f, /* 配置 pos_max。 */
    .pos_min = -1500.0f, /* 配置 pos_min。 */
    .vel_max = 40.0f, /* 配置 vel_max。 */
    .vel_min = -40.0f, /* 配置 vel_min。 */
    .torque_max = 10.0f, /* 配置 torque_max。 */
    .torque_min = -10.0f, /* 配置 torque_min。 */
    // 默认控制参数
    .default_kp = 0.0f, /* 配置 default_kp。 */
    .default_kd = 0.0f, /* 配置 default_kd。 */
    .default_torque_ff = 0.0f, /* 配置 default_torque_ff。 */
    // 虚函数表
    .init = DM_J3519_InitInternal, /* 配置 init。 */
    .calculate = DM_J3519_CalculateInternal, /* 配置 calculate。 */
    .refresh_data = DM_Motor_RefreshData, /* 配置 refresh_data。 */
    .WorkMode = DM_LOCATION_SPEED, /* 配置 WorkMode。 */
};

// 87 FF 7F F0 0B 33 37 FF -> 位置 10 rad, 速度 0 rad/s, kp = 1.4591, kd = 1.0f, 转矩0.0f
// 7F FF 7F F0 0B 33 37 FF -> 位置变为 0 rad
/// @brief J4310电机类
const DM_MotorClass_t DM_J4310_Class = { /* 初始化 DM_J4310_Class。 */
    .name = "J4310", /* 配置 name。 */
    .model = DmJ4310, /* 配置 model。 */
    // 限幅参数（根据J4310-2EC手册）
    .kp_max = 500.0f, /* 配置 kp_max。 */
    .kp_min = 0.0f, /* 配置 kp_min。 */
    .kd_max = 5.0f, /* 配置 kd_max。 */
    .kd_min = 0.0f, /* 配置 kd_min。 */
    .pos_max = 160.0f, /* 配置 pos_max。 */
    .pos_min = -160.0f, /* 配置 pos_min。 */
    .vel_max = 30.0f, /* 配置 vel_max。 */
    .vel_min = -30.0f, /* 配置 vel_min。 */
    .torque_max = 10.0f, /* 配置 torque_max。 */
    .torque_min = -10.0f, /* 配置 torque_min。 */
    // 默认控制参数
    .default_kp = 0.0f, /* 配置 default_kp。 */
    .default_kd = 0.0f, /* 配置 default_kd。 */
    .default_torque_ff = 0.2f, /* 配置 default_torque_ff。 */
    // 虚函数表
    .init = DM_J4310_InitInternal, /* 配置 init。 */
    .calculate = DM_J4310_CalculateInternal, /* 配置 calculate。 */
    .refresh_data = DM_Motor_RefreshData, /* 配置 refresh_data。 */
    .WorkMode = DM_MIT, /* 配置 WorkMode。 */
};

/*============================== 数据转换函数 ==============================*/

static inline float uint_to_float_generic(uint16_t x_int, float x_min, float x_max, int8_t bits) /* 实现 uint_to_float_generic。 */
{
    float span = x_max - x_min; /* 初始化 span。 */
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min; /* 返回当前计算结果。 */
}

static inline int16_t float_to_uint_generic(float x_float, float x_min, float x_max, int8_t bits) /* 实现 float_to_uint_generic。 */
{
    float span = x_max - x_min; /* 初始化 span。 */
    if (x_float < x_min) /* 检查当前执行条件。 */
        x_float = x_min; /* 更新 x_float。 */
    if (x_float > x_max) /* 检查当前执行条件。 */
        x_float = x_max; /* 更新 x_float。 */
    return (int16_t)((x_float - x_min) * ((float)((1 << bits) - 1)) / span); /* 返回当前计算结果。 */
}

/*============================== 获取数据位数辅助函数 ==============================*/

/// @brief 根据工作模式和数据类型获取对应的位数
static inline int DM_GetDataBits(DM_WorkMode mode, DM_DATA data_type) /* 实现 DM_GetDataBits。 */
{
    switch (mode) /* 按当前状态选择处理分支。 */
    {
    case DM_MIT: /* 处理 DM_MIT 分支。 */
        switch (data_type) /* 按当前状态选择处理分支。 */
        {
        case DM_POS: /* 处理 DM_POS 分支。 */
            return DM_MIT_POS_BIT; /* 返回当前计算结果。 */
        case DM_VEL: /* 处理 DM_VEL 分支。 */
            return DM_MIT_VEL_BIT; /* 返回当前计算结果。 */
        case DM_KP: /* 处理 DM_KP 分支。 */
            return DM_MIT_KP_BIT; /* 返回当前计算结果。 */
        case DM_KD: /* 处理 DM_KD 分支。 */
            return DM_MIT_KD_BIT; /* 返回当前计算结果。 */
        case DM_TORQUE: /* 处理 DM_TORQUE 分支。 */
            return DM_MIT_TORQUE_BIT; /* 返回当前计算结果。 */
        default: /* 处理默认分支。 */
            return 16; /* 返回当前计算结果。 */
        }
    case DM_LOCATION_SPEED: /* 处理 DM_LOCATION_SPEED 分支。 */
        switch (data_type) /* 按当前状态选择处理分支。 */
        {
        case DM_POS: /* 处理 DM_POS 分支。 */
            return DM_LS_POS_BIT; /* 返回当前计算结果。 */
        case DM_VEL: /* 处理 DM_VEL 分支。 */
            return DM_LS_VEL_BIT; /* 返回当前计算结果。 */
        default: /* 处理默认分支。 */
            return 32; /* 返回当前计算结果。 */
        }
    case DM_SPEED: /* 处理 DM_SPEED 分支。 */
        switch (data_type) /* 按当前状态选择处理分支。 */
        {
        case DM_VEL: /* 处理 DM_VEL 分支。 */
            return DM_SPD_VEL_BIT; /* 返回当前计算结果。 */
        default: /* 处理默认分支。 */
            return 32; /* 返回当前计算结果。 */
        }
    case DM_PVT: /* 处理 DM_PVT 分支。 */
        switch (data_type) /* 按当前状态选择处理分支。 */
        {
        case DM_POS: /* 处理 DM_POS 分支。 */
            return DM_PVT_POS_BIT; /* 返回当前计算结果。 */
        case DM_VEL: /* 处理 DM_VEL 分支。 */
            return DM_PVT_VEL_BIT; /* 返回当前计算结果。 */
        case DM_TORQUE: /* 处理 DM_TORQUE 分支。 */
            return DM_PVT_CURRENT_BIT; /* 返回当前计算结果。 */
        default: /* 处理默认分支。 */
            return 16; /* 返回当前计算结果。 */
        }
    default: /* 处理默认分支。 */
        return 16; /* 返回当前计算结果。 */
    }
}

/*============================== 多模式数据转换函数 ==============================*/

static float uint_to_float_config(float x_int, DM_DATA DataMode, DM_WorkMode mode, /* 传入下一项参数或数据。 */
                                  const DM_MotorConfig_t *cfg, const DM_MotorClass_t *cls) /* 继续当前语句。 */
{
    if (cfg == NULL || cls == NULL) /* 检查当前执行条件。 */
        return 0.0f; /* 返回当前计算结果。 */

    int bits = DM_GetDataBits(mode, DataMode); /* 初始化 bits。 */

    switch (DataMode) /* 按当前状态选择处理分支。 */
    {
    case DM_POS: /* 处理 DM_POS 分支。 */
        return uint_to_float_generic((int)x_int, cfg->pos_min, cfg->pos_max, bits); /* 返回当前计算结果。 */
    case DM_VEL: /* 处理 DM_VEL 分支。 */
        return uint_to_float_generic((int)x_int, cfg->vel_min, cfg->vel_max, bits); /* 返回当前计算结果。 */
    case DM_KD: /* 处理 DM_KD 分支。 */
        return uint_to_float_generic((int)x_int, cls->kd_min, cls->kd_max, bits); /* 返回当前计算结果。 */
    case DM_KP: /* 处理 DM_KP 分支。 */
        return uint_to_float_generic((int)x_int, cls->kp_min, cls->kp_max, bits); /* 返回当前计算结果。 */
    case DM_TORQUE: /* 处理 DM_TORQUE 分支。 */
        return uint_to_float_generic((int)x_int, cls->torque_min, cls->torque_max, bits); /* 返回当前计算结果。 */
    default: /* 处理默认分支。 */
        return 0.0f; /* 返回当前计算结果。 */
    }
}

static int16_t float_to_uint_config(float x_float, DM_DATA DataMode, DM_WorkMode mode, /* 传入下一项参数或数据。 */
                                    const DM_MotorConfig_t *cfg, const DM_MotorClass_t *cls) /* 继续当前语句。 */
{
    if (cfg == NULL || cls == NULL) /* 检查当前执行条件。 */
        return 0; /* 无效句柄。 */

    int bits = DM_GetDataBits(mode, DataMode); /* 初始化 bits。 */

    switch (DataMode) /* 按当前状态选择处理分支。 */
    {
    case DM_POS: /* 处理 DM_POS 分支。 */
        return float_to_uint_generic(x_float, cfg->pos_min, cfg->pos_max, bits); /* 返回当前计算结果。 */
    case DM_VEL: /* 处理 DM_VEL 分支。 */
        return float_to_uint_generic(x_float, cfg->vel_min, cfg->vel_max, bits); /* 返回当前计算结果。 */
    case DM_KD: /* 处理 DM_KD 分支。 */
        return float_to_uint_generic(x_float, cls->kd_min, cls->kd_max, bits); /* 返回当前计算结果。 */
    case DM_KP: /* 处理 DM_KP 分支。 */
        return float_to_uint_generic(x_float, cls->kp_min, cls->kp_max, bits); /* 返回当前计算结果。 */
    case DM_TORQUE: /* 处理 DM_TORQUE 分支。 */
        return float_to_uint_generic(x_float, cls->torque_min, cls->torque_max, bits); /* 返回当前计算结果。 */
    default: /* 处理默认分支。 */
        return 0; /* 返回状态值 0。 */
    }
}

/*============================== 面向对象接口实现 ==============================*/

/// @brief 使用指定的电机类创建电机实例
void DM_Motor_Create(MotorTypeDef *motor, const DM_MotorClass_t *motor_class, uint8_t id) /* 实现 DM_Motor_Create。 */
{
    if (motor == NULL || motor_class == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    if (id < 1 || id > 4) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    // 关联电机类
    motor->motor_class.dm_motor_class = motor_class; /* 更新 dm_motor_class。 */

    // 调用类的初始化函数
    if (motor_class->init != NULL) /* 检查当前执行条件。 */
    {
        motor_class->init(motor, id); /* 完成本行操作。 */
    }
}

/// @brief 调用电机的解算函数
void DM_Motor_Calculate(MotorTypeDef *motor) /* 实现 DM_Motor_Calculate。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    // 优先使用类的虚函数表
    if (motor->motor_class.dm_motor_class != NULL && motor->motor_class.dm_motor_class->calculate != NULL) /* 检查当前执行条件。 */
    {
        motor->motor_class.dm_motor_class->calculate(motor); /* 完成本行操作。 */
        return; /* 结束当前函数。 */
    }

    // 兼容旧接口：使用直接绑定的函数指针
    if (motor->calculate != NULL) /* 检查当前执行条件。 */
    {
        motor->calculate(motor); /* 完成本行操作。 */
    }
}

/// @brief 获取电机所属的类指针
const DM_MotorClass_t *DM_Motor_GetClass(MotorTypeDef *motor) /* 实现 DM_Motor_GetClass。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
        return NULL; /* 返回当前计算结果。 */
    if (motor->MotorInf.band != DM_MOTOR_BAND) /* 检查当前执行条件。 */
        return NULL; /* 返回当前计算结果。 */
    return motor->motor_class.dm_motor_class; /* 返回当前计算结果。 */
}

/*============================== 电机初始化函数 ==============================*/

/// @brief 初始化DM电机基础属性（使用电机类）
static void DM_Motor_InitWithClass(MotorTypeDef *motor, uint8_t id, /* 传入下一项参数或数据。 */
                                   const DM_MotorClass_t *motor_class) /* 继续当前语句。 */
{
    memset(motor, 0, sizeof(MotorTypeDef)); /* 调用 memset。 */

    // 关联电机类（推荐通过motor_class访问虚函数）
    motor->motor_class.dm_motor_class = motor_class; /* 更新 dm_motor_class。 */

    // 基本属性
    motor->MotorID = id; /* 更新 MotorID。 */
    motor->MotorInf.band = DM_MOTOR_BAND; /* 更新 band。 */
    motor->MotorInf.model = motor_class->model; /* 更新 model。 */

    // 默认配置
    motor->config.direction_bias = 0.0f; /* 更新 direction_bias。 */
    motor->config.position_tolerance = 50.0f; /* 更新 position_tolerance。 */
    motor->config.reverse = 0; /* 更新 reverse。 */

    // 绑定函数指针（已弃用，保留用于向后兼容，新代码请使用motor_class虚函数表）
    motor->SendMotorControl = DM_Motor_SendControlInternal; /* 更新 SendMotorControl。 */
    motor->calculate = motor_class->calculate; /* 更新 calculate。 */

    // CAN报文头（发送ID由CanRegisterMotorCfg从配置表统一设置）
    motor->g_TxHeader.StdId = 0; // 占位，由CanRegisterMotorCfg设置
    motor->g_TxHeader.IDE = CAN_ID_STD; /* 更新 IDE。 */
    motor->g_TxHeader.RTR = CAN_RTR_DATA; /* 更新 RTR。 */
    motor->g_TxHeader.DLC = CtrlMotorLen; /* 更新 DLC。 */

    // 初始化DM特有配置（使用类的默认参数）
    if (id > 0 && id <= 4) /* 检查当前执行条件。 */
    {
        g_DM_Configs[id - 1].kp = motor_class->default_kp; /* 更新 kp。 */
        g_DM_Configs[id - 1].kd = motor_class->default_kd; /* 更新 kd。 */
        g_DM_Configs[id - 1].torque_ff = motor_class->default_torque_ff; /* 更新 torque_ff。 */

        // 从类的默认值初始化位置和速度限幅参数
        g_DM_Configs[id - 1].pos_max = motor_class->pos_max; /* 更新 pos_max。 */
        g_DM_Configs[id - 1].pos_min = motor_class->pos_min; /* 更新 pos_min。 */
        g_DM_Configs[id - 1].vel_max = motor_class->vel_max; /* 更新 vel_max。 */
        g_DM_Configs[id - 1].vel_min = motor_class->vel_min; /* 更新 vel_min。 */
    }
}

static void DM_J3519_InitInternal(MotorTypeDef *motor, uint8_t id) /* 实现 DM_J3519_InitInternal。 */
{
    DM_Motor_InitWithClass(motor, id, &DM_J3519_Class); /* 调用 DM_Motor_InitWithClass。 */
}

static void DM_J4310_InitInternal(MotorTypeDef *motor, uint8_t id) /* 实现 DM_J4310_InitInternal。 */
{
    DM_Motor_InitWithClass(motor, id, &DM_J4310_Class); /* 调用 DM_Motor_InitWithClass。 */
}

/*============================== 兼容旧接口的初始化函数 ==============================*/

void DM_J3519_Init(MotorTypeDef *motor, uint8_t id) /* 实现 DM_J3519_Init。 */
{
    if (motor == NULL || id < 1) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    DM_Motor_Create(motor, &DM_J3519_Class, id); /* 调用 DM_Motor_Create。 */
}

void DM_J4310_Init(MotorTypeDef *motor, uint8_t id) /* 实现 DM_J4310_Init。 */
{
    if (motor == NULL || id < 1) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    DM_Motor_Create(motor, &DM_J4310_Class, id); /* 调用 DM_Motor_Create。 */
}

/*============================== 电机配置函数 ==============================*/

void DM_Motor_SetConfig(MotorTypeDef *motor, const DM_MotorConfig_t *config) /* 实现 DM_Motor_SetConfig。 */
{
    if (motor == NULL || config == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    if (motor->MotorInf.band != DM_MOTOR_BAND) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    uint8_t id = motor->MotorID; /* 初始化 id。 */
    if (id > 0 && id <= 4) /* 检查当前执行条件。 */
    {
        g_DM_Configs[id - 1] = *config; /* 更新 g_DM_Configs。 */
    }
}

DM_MotorConfig_t *DM_Motor_GetConfig(MotorTypeDef *motor) /* 实现 DM_Motor_GetConfig。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
        return NULL; /* 返回当前计算结果。 */
    if (motor->MotorInf.band != DM_MOTOR_BAND) /* 检查当前执行条件。 */
        return NULL; /* 返回当前计算结果。 */

    uint8_t id = motor->MotorID; /* 初始化 id。 */
    if (id > 0 && id <= 4) /* 检查当前执行条件。 */
    {
        return &g_DM_Configs[id - 1]; /* 返回当前计算结果。 */
    }
    return NULL; /* 返回当前计算结果。 */
}

void DM_Motor_SetCascadePID(MotorTypeDef *motor, /* 传入下一项参数或数据。 */
                            float outer_p, float outer_i, float outer_d, float outer_f, /* 传入下一项参数或数据。 */
                            float inner_p, float inner_i, float inner_d, float inner_f, /* 传入下一项参数或数据。 */
                            float outer_max_out, float outer_min_out, float outer_max_iout, /* 传入下一项参数或数据。 */
                            float inner_max_out, float inner_min_out, float inner_max_iout) /* 继续当前语句。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    motor->use_cascade = 1; /* 更新 use_cascade。 */
    CASCADE_PID_Init(&motor->cascade_pid, /* 传入下一项参数或数据。 */
                     outer_p, outer_i, outer_d, outer_f, /* 传入下一项参数或数据。 */
                     inner_p, inner_i, inner_d, inner_f, /* 传入下一项参数或数据。 */
                     outer_max_out, outer_min_out, outer_max_iout, /* 传入下一项参数或数据。 */
                     inner_max_out, inner_min_out, inner_max_iout); /* 完成本行操作。 */
    CASCADE_PID_Clear(&motor->cascade_pid); /* 调用 CASCADE_PID_Clear。 */
}

void DM_Motor_SetSpeedPID(MotorTypeDef *motor, PID_MODE_e mode, /* 传入下一项参数或数据。 */
                          float p, float i, float d, float f, /* 传入下一项参数或数据。 */
                          float max_out, float min_out, float max_iout) /* 继续当前语句。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    motor->use_cascade = 0; /* 更新 use_cascade。 */
    PID_Init(&motor->inner_pid, mode, p, i, d, f, max_out, min_out, max_iout); /* 调用 PID_Init。 */
    PID_Clear(&motor->inner_pid); /* 调用 PID_Clear。 */
}

/*============================== 单独配置参数函数 ==============================*/

void DM_Motor_SetKp(MotorTypeDef *motor, float kp) /* 实现 DM_Motor_SetKp。 */
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor); /* 初始化 cfg。 */
    if (cfg == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    // 获取电机类进行限幅
    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class; /* 初始化 cls。 */
    if (cls != NULL) /* 检查当前执行条件。 */
    {
        if (kp < cls->kp_min) /* 检查当前执行条件。 */
            kp = cls->kp_min; /* 更新 kp。 */
        if (kp > cls->kp_max) /* 检查当前执行条件。 */
            kp = cls->kp_max; /* 更新 kp。 */
    }

    cfg->kp = kp; /* 更新 kp。 */
}

void DM_Motor_SetKd(MotorTypeDef *motor, float kd) /* 实现 DM_Motor_SetKd。 */
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor); /* 初始化 cfg。 */
    if (cfg == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    // 获取电机类进行限幅
    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class; /* 初始化 cls。 */
    if (cls != NULL) /* 检查当前执行条件。 */
    {
        if (kd < cls->kd_min) /* 检查当前执行条件。 */
            kd = cls->kd_min; /* 更新 kd。 */
        if (kd > cls->kd_max) /* 检查当前执行条件。 */
            kd = cls->kd_max; /* 更新 kd。 */
    }

    cfg->kd = kd; /* 更新 kd。 */
}

void DM_Motor_SetTorqueFF(MotorTypeDef *motor, float torque_ff) /* 实现 DM_Motor_SetTorqueFF。 */
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor); /* 初始化 cfg。 */
    if (cfg == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    // 获取电机类进行限幅
    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class; /* 初始化 cls。 */
    if (cls != NULL) /* 检查当前执行条件。 */
    {
        if (torque_ff < cls->torque_min) /* 检查当前执行条件。 */
            torque_ff = cls->torque_min; /* 更新 torque_ff。 */
        if (torque_ff > cls->torque_max) /* 检查当前执行条件。 */
            torque_ff = cls->torque_max; /* 更新 torque_ff。 */
    }

    cfg->torque_ff = torque_ff; /* 更新 torque_ff。 */
}

void DM_Motor_SetPosLimits(MotorTypeDef *motor, float pos_min, float pos_max) /* 实现 DM_Motor_SetPosLimits。 */
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor); /* 初始化 cfg。 */
    if (cfg == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    cfg->pos_min = pos_min; /* 更新 pos_min。 */
    cfg->pos_max = pos_max; /* 更新 pos_max。 */
}

void DM_Motor_SetVelLimits(MotorTypeDef *motor, float vel_min, float vel_max) /* 实现 DM_Motor_SetVelLimits。 */
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor); /* 初始化 cfg。 */
    if (cfg == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    cfg->vel_min = vel_min; /* 更新 vel_min。 */
    cfg->vel_max = vel_max; /* 更新 vel_max。 */
}

void DM_Motor_SetMITParams(MotorTypeDef *motor, float kp, float kd, float torque_ff) /* 实现 DM_Motor_SetMITParams。 */
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor); /* 初始化 cfg。 */
    if (cfg == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    // 获取电机类进行限幅
    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class; /* 初始化 cls。 */
    if (cls != NULL) /* 检查当前执行条件。 */
    {
        // KP限幅
        if (kp < cls->kp_min) /* 检查当前执行条件。 */
            kp = cls->kp_min; /* 更新 kp。 */
        if (kp > cls->kp_max) /* 检查当前执行条件。 */
            kp = cls->kp_max; /* 更新 kp。 */

        // KD限幅
        if (kd < cls->kd_min) /* 检查当前执行条件。 */
            kd = cls->kd_min; /* 更新 kd。 */
        if (kd > cls->kd_max) /* 检查当前执行条件。 */
            kd = cls->kd_max; /* 更新 kd。 */

        // 力矩限幅
        if (torque_ff < cls->torque_min) /* 检查当前执行条件。 */
            torque_ff = cls->torque_min; /* 更新 torque_ff。 */
        if (torque_ff > cls->torque_max) /* 检查当前执行条件。 */
            torque_ff = cls->torque_max; /* 更新 torque_ff。 */
    }

    cfg->kp = kp; /* 更新 kp。 */
    cfg->kd = kd; /* 更新 kd。 */
    cfg->torque_ff = torque_ff; /* 更新 torque_ff。 */
}

float DM_Motor_GetKp(MotorTypeDef *motor) /* 实现 DM_Motor_GetKp。 */
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor); /* 初始化 cfg。 */
    if (cfg == NULL) /* 检查当前执行条件。 */
        return 0.0f; /* 返回当前计算结果。 */
    return cfg->kp; /* 返回当前计算结果。 */
}

float DM_Motor_GetKd(MotorTypeDef *motor) /* 实现 DM_Motor_GetKd。 */
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor); /* 初始化 cfg。 */
    if (cfg == NULL) /* 检查当前执行条件。 */
        return 0.0f; /* 返回当前计算结果。 */
    return cfg->kd; /* 返回当前计算结果。 */
}

float DM_Motor_GetTorqueFF(MotorTypeDef *motor) /* 实现 DM_Motor_GetTorqueFF。 */
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor); /* 初始化 cfg。 */
    if (cfg == NULL) /* 检查当前执行条件。 */
        return 0.0f; /* 返回当前计算结果。 */
    return cfg->torque_ff; /* 返回当前计算结果。 */
}

/*============================== 使能/失能函数 ==============================*/

static uint8_t DM_Motor_SendTracked(MotorTypeDef *motor, uint8_t *data) /* 实现 DM_Motor_SendTracked。 */
{
    /* 发送前必须抢到唯一回复窗口。 */
    if (motor == NULL || data == NULL || !CanMotor_DmReplyWaitBegin(motor)) /* 检查当前执行条件。 */
    {
        return 0U; /* 参数无效或上一帧仍未回复。 */
    }

    /* pending 已在发送前置位，避免极速反馈抢跑。 */
    if (Motor_GetHAL_Fast()->can_send(&motor->g_TxHeader, data)) /* 检查当前执行条件。 */
    {
        return 1U; /* 发送成功，等待反馈清标志。 */
    }

    CanMotor_DmReplyWaitCancel(motor); /* 入邮箱失败，立即撤销计时。 */
    return 0U;                         /* 报告发送失败。 */
}

uint8_t DM_Motor_Enable(MotorTypeDef *motor) /* 实现 DM_Motor_Enable。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
        return 0; /* 返回状态值 0。 */

    const MotorHAL_t *hal = Motor_GetHAL(); /* 获取可替换硬件接口。 */
    if (DM_Motor_SendTracked(motor, (uint8_t *)DM_MOTOR_ENABLE)) /* 检查当前执行条件。 */
    {
        motor->drive_enabled = 1U; /* 记录使能帧已发送。 */
        hal->delay_ms(1);          /* 给电调留出处理时间。 */
        return 1;                  /* 返回发送成功。 */
    }
    return 0; /* 发送失败或上一帧仍在等待。 */
}

uint8_t DM_Motor_Disable(MotorTypeDef *motor) /* 实现 DM_Motor_Disable。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
        return 0; /* 返回状态值 0。 */

    const MotorHAL_t *hal = Motor_GetHAL(); /* 获取可替换硬件接口。 */
    if (DM_Motor_SendTracked(motor, (uint8_t *)DM_MOTOR_DISABLE)) /* 检查当前执行条件。 */
    {
        motor->drive_enabled = 0U; /* 禁止后续控制输出。 */
        hal->delay_ms(1);          /* 给电调留出处理时间。 */
        return 1;                  /* 返回发送成功。 */
    }
    return 0; /* 发送失败或上一帧仍在等待。 */
}

uint8_t DM_MotorDisable(can_motor_cfg motor_cfg) /* 实现 DM_MotorDisable。 */
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1]; /* 获取目标电机。 */
    if (motor == NULL) /* 检查当前执行条件。 */
        return 0; /* 无效句柄。 */
    return DM_Motor_Disable(motor); /* 复用带回复跟踪的失能入口。 */
}

uint8_t DM_MotorEnable(can_motor_cfg motor_cfg) /* 实现 DM_MotorEnable。 */
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1]; /* 获取目标电机。 */
    if (motor == NULL) /* 检查当前执行条件。 */
        return 0; /* 无效句柄。 */

    const uint32_t enable_timeout_ms = 500U; /* 整体使能重试上限。 */
    uint32_t start_tick = HAL_GetTick();     /* 记录重试起点。 */

    /* 在总超时内重复执行一发一收。 */
    while (1) /* 持续执行当前任务。 */
    {
        if ((uint32_t)(HAL_GetTick() - start_tick) >= enable_timeout_ms) /* 检查当前执行条件。 */
        {
            return 0; /* 长时间未收到使能确认。 */
        }

        /* pending 未清时不会叠发。 */
        if (!DM_Motor_SendTracked(motor, (uint8_t *)DM_MOTOR_ENABLE)) /* 检查当前执行条件。 */
        {
            vTaskDelay(pdMS_TO_TICKS(3)); /* 等回复或等待下一轮。 */
            continue;                    /* 本轮不检查反馈状态。 */
        }

        vTaskDelay(pdMS_TO_TICKS(3)); /* 等待反馈中断更新数据。 */

        uint8_t state = (motor->ReceiveMotorData[0] >> 4) & 0x0F; /* 读取反馈使能状态。 */
        if (motor->dm_reply_pending == 0U && state == 0x01) /* 检查当前执行条件。 */
        {
            motor->drive_enabled = 1U; /* 新反馈确认使能成功。 */
            return 1;                  /* 完成使能流程。 */
        }
        /* 未确认使能则继续下一轮一发一收。 */
    }
}

/*============================== 刷新数据函数 ==============================*/
void DM_Motor_RefreshData(can_motor_cfg motor_cfg) /* 实现 DM_Motor_RefreshData。 */
{
    MotorTypeDef *st = &MotorManager.MotorList[motor_cfg - 1]; /* 获取目标电机。 */
    if (st == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    const MotorHAL_t *hal = Motor_GetHAL(); /* 初始化 hal。 */
    if (DM_Motor_SendTracked(st, (uint8_t *)DM_MOTOR_ENABLE)) /* 检查当前执行条件。 */
    {
        st->drive_enabled = 1U; /* 更新 drive_enabled。 */
        hal->delay_ms(1); /* 完成本行操作。 */
    }
}

/*============================== 发送控制函数 ==============================*/

// 记录上次发送结束时间，防止连续调用导致CAN帧丢失
static uint32_t s_DM_LastSendTick = 0; /* 全局 DM 最后发送时刻。 */

static uint8_t DM_Motor_SendControlInternal(MotorTypeDef *st) /* 实现 DM_Motor_SendControlInternal。 */
{
    if (st == NULL) /* 检查当前执行条件。 */
        return 0; /* 返回状态值 0。 */

    // 检查距离上次发送是否有至少1tick间隔，防止连续调用丢帧
    uint32_t current_tick = HAL_GetTick(); /* 读取当前毫秒时刻。 */
    if (current_tick - s_DM_LastSendTick < 1) /* 检查当前执行条件。 */
    {
        vTaskDelay(1); /* 两帧至少间隔 1 ms。 */
    }

    // 自动使能（合并条件判断）
    if (st->drive_enabled == 0U) /* 检查当前执行条件。 */
    {
        if (DM_Motor_Enable(st) == 0U) /* 检查当前执行条件。 */
        {
            return 0U; /* 使能帧未发送成功。 */
        }
    }

    // 使用Fast版本（内联，零开销）
    uint8_t result = DM_Motor_SendTracked(st, st->SendMotorData); /* 发送并等待唯一回复。 */

    // 更新上次发送结束时间
    s_DM_LastSendTick = HAL_GetTick(); /* 记录本次发送结束时间。 */

    return result; /* 返回控制帧发送结果。 */
}

/***********************************
 * 函数名: DM_MotorSetTxData
 * 作用:   用于设置发送数据
 * 参数:   motor_cfg 电机名称
 * 参数:   data      数据指针
 * 返回值: 无
 * todo:   当前还需要加上各个电机的模式选择,不同的模式调用不同的发送函数
 **********************************/
void DM_MotorSetTxData(can_motor_cfg motor_cfg, uint8_t *data) /* 实现 DM_MotorSetTxData。 */
{
    if (data == NULL) /* 检查当前执行条件。 */
    {
        Error_Handler(); /* 数据指针错误属于配置故障。 */
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1]; /* 初始化 motor。 */
    if (motor == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    memset(motor->SendMotorData, 0x00, CtrlMotorLen); /* 清旧控制数据。 */
    memcpy(motor->SendMotorData, data, CtrlMotorLen);  /* 保存本次控制帧。 */
    motor->SendMotorControl(motor);                    /* 进入带回复跟踪的发送入口。 */
}

/*============================== 电机解算函数 ==============================*/
// note:关节电机是可以直接读取并且不做减速比

/// @brief J3519电机解算（内部版本，优化：使用Fast版本获取配置）
static void DM_J3519_CalculateInternal(MotorTypeDef *motor) /* 实现 DM_J3519_CalculateInternal。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class; /* 初始化 cls。 */
    if (cls == NULL) /* 检查当前执行条件。 */
        cls = &DM_J3519_Class; /* 更新 cls。 */

    MotorSolvedData_t *pData = &motor->motor_data; /* 初始化 pData。 */
    uint8_t *rx = motor->ReceiveMotorData; /* 初始化 rx。 */

    // 解析原始数据
    uint16_t pos_raw = ((uint16_t)rx[1] << 8) | rx[2]; /* 初始化 pos_raw。 */
    uint16_t vel_raw = ((uint16_t)rx[3] << 4) | (rx[4] >> 4); /* 初始化 vel_raw。 */
    uint16_t tor_raw = ((rx[4] & 0x0F) << 8) | rx[5]; /* 初始化 tor_raw。 */

    // J3519: 使用电机类的固定范围进行解算（不受SetPosLimits/SetVelLimits影响）
    pData->solved_data[0] = uint_to_float_generic(pos_raw, cls->pos_min, cls->pos_max, 16); /* 更新 solved_data。 */

    // 速度滤波
    float vel_new = uint_to_float_generic(vel_raw, cls->vel_min, cls->vel_max, 12); /* 初始化 vel_new。 */
    if (!pData->filter_init) /* 检查当前执行条件。 */
    {
        pData->solved_data[1] = vel_new; /* 更新 solved_data。 */
        pData->filter_init = 1; /* 更新 filter_init。 */
    }
    else /* 处理其余情况。 */
    {
        pData->solved_data[1] = (DM_SPEED_FILTER_COEF * pData->last_speed + (1.0f - DM_SPEED_FILTER_COEF) * vel_new); /* 更新 solved_data。 */
    }
    pData->last_speed = pData->solved_data[1]; /* 更新 last_speed。 */

    // 力矩
    pData->solved_data[2] = uint_to_float_generic(tor_raw, cls->torque_min, cls->torque_max, 12); /* 更新 solved_data。 */

    // 温度
    pData->solved_data[3] = (float)rx[6]; /* 更新 solved_data。 */
    pData->solved_data[4] = (float)rx[7]; /* 更新 solved_data。 */

    /* 与 RM 储能电机保持同一逻辑方向：反馈先翻到业务坐标，
     * 控制发送边界再按相同配置翻回电机物理坐标。 */
    if (motor->config.reverse != 0U) /* 检查当前执行条件。 */
    {
        pData->solved_data[0] = -pData->solved_data[0]; /* 更新 solved_data。 */
        pData->solved_data[1] = -pData->solved_data[1]; /* 更新 solved_data。 */
        pData->solved_data[2] = -pData->solved_data[2]; /* 更新 solved_data。 */
    }

    pData->solved_data[5] = pData->solved_data[0] / 19.2f; // 转过的角度
    pData->solved_data[6] = pData->solved_data[1] / 19.2f; // 转过的速度

    DM_Check3519StallInFeedback(motor); /* 调用 DM_Check3519StallInFeedback。 */
}

/// @brief J4310电机解算（内部版本，优化：使用Fast版本获取配置）
static void DM_J4310_CalculateInternal(MotorTypeDef *motor) /* 实现 DM_J4310_CalculateInternal。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class; /* 初始化 cls。 */
    if (cls == NULL) /* 检查当前执行条件。 */
        cls = &DM_J4310_Class; /* 更新 cls。 */

    MotorSolvedData_t *pData = &motor->motor_data; /* 初始化 pData。 */
    uint8_t *rx = motor->ReceiveMotorData; /* 初始化 rx。 */

    // 解析原始数据
    uint16_t pos_raw = ((uint16_t)rx[1] << 8) | rx[2]; /* 初始化 pos_raw。 */
    uint16_t vel_raw = ((uint16_t)rx[3] << 4) | (rx[4] >> 4); /* 初始化 vel_raw。 */
    uint16_t tor_raw = ((rx[4] & 0x0F) << 8) | rx[5]; /* 初始化 tor_raw。 */

    // J4310: 使用电机类的固定范围进行解算（不受SetPosLimits/SetVelLimits影响）
    pData->solved_data[0] = uint_to_float_generic(pos_raw, cls->pos_min, cls->pos_max, 16); /* 更新 solved_data。 */

    // 速度滤波
    float vel_new = uint_to_float_generic(vel_raw, cls->vel_min, cls->vel_max, 12); /* 初始化 vel_new。 */
    if (!pData->filter_init) /* 检查当前执行条件。 */
    {
        pData->solved_data[1] = vel_new; /* 更新 solved_data。 */
        pData->filter_init = 1; /* 更新 filter_init。 */
    }
    else /* 处理其余情况。 */
    {
        pData->solved_data[1] = DM_SPEED_FILTER_COEF * pData->last_speed + (1.0f - DM_SPEED_FILTER_COEF) * vel_new; /* 更新 solved_data。 */
    }
    pData->last_speed = pData->solved_data[1]; /* 更新 last_speed。 */

    // 力矩
    pData->solved_data[2] = uint_to_float_generic(tor_raw, cls->torque_min, cls->torque_max, 12); /* 更新 solved_data。 */

    // 温度
    pData->solved_data[3] = (float)rx[6]; /* 更新 solved_data。 */
    pData->solved_data[4] = (float)rx[7]; /* 更新 solved_data。 */
}

void DM_MOTOR_CALCU(MotorTypeDef *motor) /* 实现 DM_MOTOR_CALCU。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    if (motor->calculate != NULL) /* 检查当前执行条件。 */
    {
        motor->calculate(motor); /* 完成本行操作。 */
        return; /* 结束当前函数。 */
    }
}

static void DM_SendMITPIDFrame(can_motor_cfg motor_cfg, float target_pos, float target_vel, float torque_ff) /* 实现 DM_SendMITPIDFrame。 */
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1]; /* 获取目标电机。 */
    if (motor == NULL || motor->MotorInf.band != DM_MOTOR_BAND) /* 检查当前执行条件。 */
        return; /* 无效句柄。 */

    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class; /* 初始化 cls。 */
    if (cls == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor); /* 初始化 cfg。 */
    if (cfg == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    uint8_t data[8] = {0x00}; /* 初始化 data。 */
    uint16_t KP_RESULT = float_to_uint_config(cfg->kp, DM_KP, DM_MIT, cfg, cls); /* 初始化 KP_RESULT。 */
    uint16_t KD_RESULT = float_to_uint_config(cfg->kd, DM_KD, DM_MIT, cfg, cls); /* 初始化 KD_RESULT。 */
    uint16_t Torque_ff = float_to_uint_config(torque_ff, DM_TORQUE, DM_MIT, cfg, cls); /* 初始化 Torque_ff。 */
    uint16_t Pos_des = float_to_uint_config(target_pos, DM_POS, DM_MIT, cfg, cls); /* 初始化 Pos_des。 */
    uint16_t Vel_des = float_to_uint_config(target_vel, DM_VEL, DM_MIT, cfg, cls); /* 初始化 Vel_des。 */

    data[0] = Pos_des >> 8; /* 更新 data。 */
    data[1] = (uint8_t)Pos_des; /* 更新 data。 */
    data[2] = Vel_des >> 4; /* 更新 data。 */
    data[3] = ((Vel_des & 0x000F) << 4) | ((KP_RESULT & 0x0F00) >> 8); /* 更新 data。 */
    data[4] = KP_RESULT; /* 更新 data。 */
    data[5] = KD_RESULT >> 4; /* 更新 data。 */
    data[6] = ((KD_RESULT & 0x000F) << 4) | ((Torque_ff & 0x0F00) >> 8); /* 更新 data。 */
    data[7] = Torque_ff; /* 更新 data。 */

    DM_MotorSetTxData(motor_cfg, data); /* 调用 DM_MotorSetTxData。 */
}

// 通用电机使用，可以是J4310也可以是J3519电机
void DmMotorSendCfg(can_motor_cfg motor_cfg, float TargetPos, float TargetVel, float TargetTorque, DM_WorkMode workmode) /* 实现 DmMotorSendCfg。 */
{
    // 获取电机结构体并使用其配置参数（使用接口函数，解耦）
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1]; /* 初始化 motor。 */
    if (motor == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    if (!Motor_IsOnline(motor_cfg)) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class; /* 初始化 cls。 */
    if (cls == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor); /* 初始化 cfg。 */
    if (cfg == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    uint8_t data[8] = {0x00}; /* 初始化 data。 */
    if (workmode == DM_MIT) /* 检查当前执行条件。 */
    {
        // 使用用户配置的参数进行转换（修复：使用cfg而非cls->default）
        uint16_t KP_RESULT = float_to_uint_config(cfg->kp, DM_KP, DM_MIT, cfg, cls); /* 初始化 KP_RESULT。 */
        uint16_t KD_RESULT = float_to_uint_config(cfg->kd, DM_KD, DM_MIT, cfg, cls); /* 初始化 KD_RESULT。 */
        uint16_t Torque_ff = float_to_uint_config(TargetTorque, DM_TORQUE, DM_MIT, cfg, cls); /* 初始化 Torque_ff。 */

        // 直接调PID电流
        uint16_t Pos_des = float_to_uint_config(0.0f, DM_POS, DM_MIT, cfg, cls); /* 初始化 Pos_des。 */
        uint16_t Vel_des = float_to_uint_config(0.0f, DM_VEL, DM_MIT, cfg, cls); /* 初始化 Vel_des。 */

        data[0] = Pos_des >> 8; /* 更新 data。 */
        data[1] = (uint8_t)Pos_des; /* 更新 data。 */
        data[2] = Vel_des >> 4; /* 更新 data。 */
        data[3] = ((Vel_des & 0x000F) << 4) | ((KP_RESULT & 0x0F00) >> 8); /* 更新 data。 */
        data[4] = KP_RESULT; /* 更新 data。 */
        data[5] = KD_RESULT >> 4; /* 更新 data。 */
        data[6] = ((KD_RESULT & 0x000F) << 4) | ((Torque_ff & 0x0F00) >> 8); /* 更新 data。 */
        data[7] = Torque_ff; /* 更新 data。 */
    }
    else if (workmode == DM_LOCATION_SPEED) /* 继续判断下一条件。 */
    {
        DM_Record3519TargetChange(motor_cfg, TargetPos); /* 调用 DM_Record3519TargetChange。 */

        // 位置速度模式：直接发送float原始字节（小端序）
        uint8_t *pbuf = (uint8_t *)&TargetPos; /* 初始化 pbuf。 */
        uint8_t *vbuf = (uint8_t *)&TargetVel; /* 初始化 vbuf。 */

        data[0] = pbuf[0]; /* 更新 data。 */
        data[1] = pbuf[1]; /* 更新 data。 */
        data[2] = pbuf[2]; /* 更新 data。 */
        data[3] = pbuf[3]; /* 更新 data。 */
        data[4] = vbuf[0]; /* 更新 data。 */
        data[5] = vbuf[1]; /* 更新 data。 */
        data[6] = vbuf[2]; /* 更新 data。 */
        data[7] = vbuf[3]; /* 更新 data。 */
    }
    else if (workmode == DM_SPEED) /* 继续判断下一条件。 */
    {
        // 速度模式：直接发送float原始字节（小端序）
        uint8_t *vbuf = (uint8_t *)&TargetVel; /* 初始化 vbuf。 */

        data[0] = vbuf[0]; /* 更新 data。 */
        data[1] = vbuf[1]; /* 更新 data。 */
        data[2] = vbuf[2]; /* 更新 data。 */
        data[3] = vbuf[3]; /* 更新 data。 */
        data[4] = 0; /* 更新 data。 */
        data[5] = 0; /* 更新 data。 */
        data[6] = 0; /* 更新 data。 */
        data[7] = 0; /* 更新 data。 */
    }
    else if (workmode == DM_PVT) /* 继续判断下一条件。 */
    {
        // 达妙官方似乎也不管这个模式了
        // PVT模式待实现 - 需要查阅官方文档确认数据格式
        // TODO: 实现PVT模式
        return; /* 结束当前函数。 */
    }
    DM_MotorSetTxData(motor_cfg, data); /* 调用 DM_MotorSetTxData。 */
}

void DmMotorPID_Calc(can_motor_cfg motor_cfg, float target) /* 实现 DmMotorPID_Calc。 */
{
    if (motor_cfg < DM_3519_STRENTH_LEFT || motor_cfg > DM_4310_YAW) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1]; /* 初始化 motor。 */
    if (motor == NULL || motor->MotorInf.band != DM_MOTOR_BAND) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    MotorSolvedData_t *pData = &motor->motor_data; /* 初始化 pData。 */
    DM_HandleAngleLoopDirectionChange(motor, target, pData->solved_data[0]); /* 调用 DM_HandleAngleLoopDirectionChange。 */

    const float target_speed = PID_Calculate(&motor->cascade_pid.outer, target, pData->solved_data[0]); /* 初始化 target_speed。 */
    s_dm_motor_pid_output = PID_Calculate(&motor->cascade_pid.inner, target_speed, pData->solved_data[1]); /* 更新 s_dm_motor_pid_output。 */
    // s_dm_motor_pid_output = PID_Calculate(&motor->inner_pid, target, pData->solved_data[1]); // 用于测试单环调参
    s_dm_motor_pid_output = DM_ApplyOutputLimit(motor, s_dm_motor_pid_output); /* 更新 s_dm_motor_pid_output。 */

    // 默认使用MIT模式
    DmMotorSendCfg(motor_cfg, 0.0f, 0.0f, s_dm_motor_pid_output, DM_MIT); /* 调用 DmMotorSendCfg。 */
}

/// @brief angle_motor 纯控制器后端：DM 位置速度模式（3519）
/// @param cfg 电机别名（储能位 3519）
/// @param ref_pos_deg 参考位置(相对零点，°)
/// @return 下发位置指令（原样返回 ref，实际位置速度闭环在电机内部完成）
/// @note  3519 是 DM_LOCATION_SPEED，MCU 侧无 PID：控制器层不做运算，
///        只把规划器给的参考位置透传给限幅/发送层。send 层补速度 StoreSpeed。
///        堵转/温度保护由 DM_Check3519StallInFeedback 在反馈解算里完成，
///        angle_motor 判断器只吸收 DM_Motor_Is3519StallProtected() 结论。
float Dm_ComputePosVel(can_motor_cfg cfg, float ref_pos_deg) /* 实现 Dm_ComputePosVel。 */
{
    (void)cfg; /* 显式忽略参数 cfg。 */
    return ref_pos_deg; /* 返回当前计算结果。 */
}

void DmMotorSpeedPID_Calc(can_motor_cfg motor_cfg, float target_speed_rpm) /* 实现 DmMotorSpeedPID_Calc。 */
{
    if (motor_cfg < DM_3519_STRENTH_LEFT || motor_cfg > DM_4310_YAW) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1]; /* 初始化 motor。 */
    if (motor == NULL || motor->MotorInf.band != DM_MOTOR_BAND) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    MotorSolvedData_t *pData = &motor->motor_data; /* 初始化 pData。 */

    s_dm_motor_pid_output = PID_Calculate(&motor->inner_pid, target_speed_rpm, pData->solved_data[1]); /* 更新 s_dm_motor_pid_output。 */
    s_dm_motor_pid_output = DM_ApplyOutputLimit(motor, s_dm_motor_pid_output); /* 更新 s_dm_motor_pid_output。 */

    // 默认使用MIT模式
    DmMotorSendCfg(motor_cfg, 0.0f, 0.0f, s_dm_motor_pid_output, DM_MIT); /* 调用 DmMotorSendCfg。 */
}

bool DM_Motor_Is3519StallProtected(void) /* 实现 DM_Motor_Is3519StallProtected。 */
{
    return (s_dm3519_stall_protected != 0U); /* 返回当前计算结果。 */
}

void DM_Motor_Clear3519StallProtection(void) /* 实现 DM_Motor_Clear3519StallProtection。 */
{
    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    s_dm3519_stall_protected = 0U; /* 更新 s_dm3519_stall_protected。 */
    DM_Reset3519StallStateByIndex(0, MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1].motor_data.solved_data[0]); /* 调用 DM_Reset3519StallStateByIndex。 */
    DM_Reset3519StallStateByIndex(1, MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1].motor_data.solved_data[0]); /* 调用 DM_Reset3519StallStateByIndex。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
}
