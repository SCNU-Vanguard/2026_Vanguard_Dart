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
extern MotorManager_t MotorManager;
static float s_dm_motor_pid_output = 0.0f;

// 注意：不再直接调用硬件函数，改用 Motor_GetHAL() 接口

#define CtrlMotorLen 8
#define DM_SPEED_FILTER_COEF 0.0f // 达妙电机本身很准确
#define DM3519_STALL_TORQUE_LIMIT_NM 5.0f
#define DM3519_STALL_TORQUE_CLEAR_NM 4.0f
#define DM3519_STALL_CONFIRM_MS 2500U
#define DM3519_COIL_TEMP_LIMIT_C 50.0f

// DM电机控制帧
static const uint8_t DM_MOTOR_ENABLE[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
static const uint8_t DM_MOTOR_DISABLE[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

// DM电机使能状态数组
static bool DM_ENABLE_ARR[g_DM_MOTOR_NUM] = {false};

// DM电机配置存储（非static，供头文件内联函数使用）
DM_MotorConfig_t g_DM_Configs[4] = {0};

typedef struct
{
    uint32_t over_current_start_ms;
} DM3519_StallState_t;

static volatile uint8_t s_dm3519_stall_protected = 0U;
static DM3519_StallState_t s_dm3519_stall_state[2] = {0};

static inline float DM_ClampFloat(float value, float min, float max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

static inline float DM_ApplyOutputLimit(MotorTypeDef *motor, float pid_output)
{
    if (motor == NULL)
        return 0.0f;

    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class;
    if (cls != NULL)
    {
        return DM_ClampFloat(pid_output, cls->torque_min, cls->torque_max);
    }

    return pid_output;
}

static inline void DM_HandleAngleLoopDirectionChange(MotorTypeDef *motor, float target_angle, float current_angle)
{
    if (motor == NULL)
    {
        return;
    }

    PID_t *outer_pid = &motor->cascade_pid.outer;
    if (!outer_pid->initialized || outer_pid->calc_count == 0U)
    {
        return;
    }

    const float angle_error = target_angle - current_angle;
    const float current_target_delta = target_angle - outer_pid->target;
    const bool target_direction_reversed = (current_target_delta * outer_pid->feedforward) < 0.0f;
    const bool error_direction_reversed = (angle_error * outer_pid->last_error) < 0.0f;

    if (target_direction_reversed || error_direction_reversed)
    {
        PID_Clear_Integral(outer_pid);
    }
}

static int DM_Get3519PairIndexByMotorCfg(can_motor_cfg motor_cfg)
{
    if (motor_cfg == DM_3519_STRENTH_LEFT)
        return 0;
    if (motor_cfg == DM_3519_STRENTH_RIGHT)
        return 1;
    return -1;
}

static int DM_Get3519PairIndex(const MotorTypeDef *motor)
{
    if (motor == NULL)
        return -1;

    const uint8_t motor_index = (uint8_t)(motor - MotorManager.MotorList);
    const can_motor_cfg motor_cfg = (can_motor_cfg)(motor_index + 1U);
    return DM_Get3519PairIndexByMotorCfg(motor_cfg);
}

static void DM_Reset3519StallStateByIndex(int index, float current_pos)
{
    (void)current_pos;
    if (index < 0 || index >= 2)
        return;

    s_dm3519_stall_state[index].over_current_start_ms = 0U;
}

static void DM_Record3519TargetChange(can_motor_cfg motor_cfg, float target_pos)
{
    (void)target_pos;
    DM_Reset3519StallStateByIndex(DM_Get3519PairIndexByMotorCfg(motor_cfg),
                                  MotorManager.MotorList[motor_cfg - 1].motor_data.solved_data[0]);
}

static void DM_Check3519StallInFeedback(MotorTypeDef *motor)
{
    const int index = DM_Get3519PairIndex(motor);
    if (index < 0 || motor == NULL)
        return;

    MotorSolvedData_t *pData = &motor->motor_data;
    DM3519_StallState_t *state = &s_dm3519_stall_state[index];
    const float torque_nm = pData->solved_data[2];
    const float coil_temp_c = pData->solved_data[4];
    const uint32_t now_ms = HAL_GetTick();

    if (!isfinite(torque_nm) || !isfinite(coil_temp_c))
    {
        state->over_current_start_ms = 0U;
        return;
    }

    if (coil_temp_c > DM3519_COIL_TEMP_LIMIT_C)
    {
        s_dm3519_stall_protected = 1U;
        state->over_current_start_ms = 0U;
        return;
    }

    if (fabsf(torque_nm) <= DM3519_STALL_TORQUE_CLEAR_NM)
    {
        state->over_current_start_ms = 0U;
        return;
    }

    if (fabsf(torque_nm) >= DM3519_STALL_TORQUE_LIMIT_NM)
    {
        if (state->over_current_start_ms == 0U)
        {
            state->over_current_start_ms = now_ms;
            return;
        }

        if ((uint32_t)(now_ms - state->over_current_start_ms) >= DM3519_STALL_CONFIRM_MS)
        {
            s_dm3519_stall_protected = 1U;
        }
    }
    else
    {
        state->over_current_start_ms = 0U;
    }
}

/*============================== DM电机ID配置表 ==============================*/
// 集中管理所有DM电机的CAN ID配置，修改ID只需改这里
// motor_id: 达妙上位机设置的ID, tx_id: 发送ID, rx_id: 接收ID, work_mode: 工作模式
static const DM_MotorIdConfig_t g_DM_IdTable[] = {
    // 左侧蓄力电机 - J3519 (位置速度模式, ID=1)
    [DM_3519_STRENTH_LEFT] = {1, 0x108, 0x028, DM_LOCATION_SPEED},
    // 右侧蓄力电机 - J3519 (位置速度模式, ID=2)
    [DM_3519_STRENTH_RIGHT] = {2, 0x109, 0x029, DM_LOCATION_SPEED},
    // Yaw轴电机 - J4310 (MIT模式, ID=3)
    [DM_4310_YAW] = {3, 0x003, 0x013, DM_MIT},
};

const DM_MotorIdConfig_t *DM_GetIdConfig(can_motor_cfg motor_cfg)
{
    if (motor_cfg >= DM_3519_STRENTH_LEFT && motor_cfg <= DM_4310_YAW)
    {
        return &g_DM_IdTable[motor_cfg];
    }
    return NULL;
}

/*============================== 静态函数声明（私有方法） ==============================*/
// 达妙电机通用函数
void DM_Motor_RefreshData(can_motor_cfg motor_cfg);

// J3519专用函数
static void DM_J3519_InitInternal(MotorTypeDef *motor, uint8_t id);
static void DM_J3519_CalculateInternal(MotorTypeDef *motor);

// J4310专用函数
static void DM_J4310_InitInternal(MotorTypeDef *motor, uint8_t id);
static void DM_J4310_CalculateInternal(MotorTypeDef *motor);

// 通用发送控制
static uint8_t DM_Motor_SendControlInternal(MotorTypeDef *motor);

/*============================== 电机类静态实例定义 ==============================*/

/// @brief J3519电机类
const DM_MotorClass_t DM_J3519_Class = {
    .name = "J3519",
    .model = DmS3519,
    // 限幅参数
    .kp_max = 500.0f,
    .kp_min = 0.0f,
    .kd_max = 5.0f,
    .kd_min = 0.0f,
    .pos_max = 1500.0f,
    .pos_min = -1500.0f,
    .vel_max = 40.0f,
    .vel_min = -40.0f,
    .torque_max = 10.0f,
    .torque_min = -10.0f,
    // 默认控制参数
    .default_kp = 0.0f,
    .default_kd = 0.0f,
    .default_torque_ff = 0.0f,
    // 虚函数表
    .init = DM_J3519_InitInternal,
    .calculate = DM_J3519_CalculateInternal,
    .refresh_data = DM_Motor_RefreshData,
    .WorkMode = DM_LOCATION_SPEED,
};

// 87 FF 7F F0 0B 33 37 FF -> 位置 10 rad, 速度 0 rad/s, kp = 1.4591, kd = 1.0f, 转矩0.0f
// 7F FF 7F F0 0B 33 37 FF -> 位置变为 0 rad
/// @brief J4310电机类
const DM_MotorClass_t DM_J4310_Class = {
    .name = "J4310",
    .model = DmJ4310,
    // 限幅参数（根据J4310-2EC手册）
    .kp_max = 500.0f,
    .kp_min = 0.0f,
    .kd_max = 5.0f,
    .kd_min = 0.0f,
    .pos_max = 160.0f,
    .pos_min = -160.0f,
    .vel_max = 30.0f,
    .vel_min = -30.0f,
    .torque_max = 10.0f,
    .torque_min = -10.0f,
    // 默认控制参数
    .default_kp = 0.0f,
    .default_kd = 0.0f,
    .default_torque_ff = 0.2f,
    // 虚函数表
    .init = DM_J4310_InitInternal,
    .calculate = DM_J4310_CalculateInternal,
    .refresh_data = DM_Motor_RefreshData,
    .WorkMode = DM_MIT,
};

/*============================== 数据转换函数 ==============================*/

static inline float uint_to_float_generic(uint16_t x_int, float x_min, float x_max, int8_t bits)
{
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

static inline int16_t float_to_uint_generic(float x_float, float x_min, float x_max, int8_t bits)
{
    float span = x_max - x_min;
    if (x_float < x_min)
        x_float = x_min;
    if (x_float > x_max)
        x_float = x_max;
    return (int16_t)((x_float - x_min) * ((float)((1 << bits) - 1)) / span);
}

/*============================== 获取数据位数辅助函数 ==============================*/

/// @brief 根据工作模式和数据类型获取对应的位数
static inline int DM_GetDataBits(DM_WorkMode mode, DM_DATA data_type)
{
    switch (mode)
    {
    case DM_MIT:
        switch (data_type)
        {
        case DM_POS:
            return DM_MIT_POS_BIT;
        case DM_VEL:
            return DM_MIT_VEL_BIT;
        case DM_KP:
            return DM_MIT_KP_BIT;
        case DM_KD:
            return DM_MIT_KD_BIT;
        case DM_TORQUE:
            return DM_MIT_TORQUE_BIT;
        default:
            return 16;
        }
    case DM_LOCATION_SPEED:
        switch (data_type)
        {
        case DM_POS:
            return DM_LS_POS_BIT;
        case DM_VEL:
            return DM_LS_VEL_BIT;
        default:
            return 32;
        }
    case DM_SPEED:
        switch (data_type)
        {
        case DM_VEL:
            return DM_SPD_VEL_BIT;
        default:
            return 32;
        }
    case DM_PVT:
        switch (data_type)
        {
        case DM_POS:
            return DM_PVT_POS_BIT;
        case DM_VEL:
            return DM_PVT_VEL_BIT;
        case DM_TORQUE:
            return DM_PVT_CURRENT_BIT; // PVT模式使用电流代替力矩
        default:
            return 16;
        }
    default:
        return 16;
    }
}

/*============================== 多模式数据转换函数 ==============================*/

/// @brief 使用电机配置参数进行uint到float转换（支持多种工作模式）
/// @param x_int 输入的整数值
/// @param DataMode 数据类型（位置/速度/KP/KD/力矩）
/// @param mode 工作模式（MIT/位置速度/速度/PVT）
/// @param cfg 电机配置参数（位置和速度限幅）
/// @param cls 电机类参数（KP/KD/力矩限幅）
static float uint_to_float_config(float x_int, DM_DATA DataMode, DM_WorkMode mode,
                                  const DM_MotorConfig_t *cfg, const DM_MotorClass_t *cls)
{
    if (cfg == NULL || cls == NULL)
        return 0.0f;

    int bits = DM_GetDataBits(mode, DataMode);

    switch (DataMode)
    {
    case DM_POS:
        return uint_to_float_generic((int)x_int, cfg->pos_min, cfg->pos_max, bits);
    case DM_VEL:
        return uint_to_float_generic((int)x_int, cfg->vel_min, cfg->vel_max, bits);
    case DM_KD:
        return uint_to_float_generic((int)x_int, cls->kd_min, cls->kd_max, bits);
    case DM_KP:
        return uint_to_float_generic((int)x_int, cls->kp_min, cls->kp_max, bits);
    case DM_TORQUE:
        return uint_to_float_generic((int)x_int, cls->torque_min, cls->torque_max, bits);
    default:
        return 0.0f;
    }
}

/// @brief 使用电机配置参数进行float到uint转换（支持多种工作模式）
/// @param x_float 输入的浮点值
/// @param DataMode 数据类型（位置/速度/KP/KD/力矩）
/// @param mode 工作模式（MIT/位置速度/速度/PVT）
/// @param cfg 电机配置参数（位置和速度限幅）
/// @param cls 电机类参数（KP/KD/力矩限幅）
static int16_t float_to_uint_config(float x_float, DM_DATA DataMode, DM_WorkMode mode,
                                    const DM_MotorConfig_t *cfg, const DM_MotorClass_t *cls)
{
    if (cfg == NULL || cls == NULL)
        return 0;

    int bits = DM_GetDataBits(mode, DataMode);

    switch (DataMode)
    {
    case DM_POS:
        return float_to_uint_generic(x_float, cfg->pos_min, cfg->pos_max, bits);
    case DM_VEL:
        return float_to_uint_generic(x_float, cfg->vel_min, cfg->vel_max, bits);
    case DM_KD:
        return float_to_uint_generic(x_float, cls->kd_min, cls->kd_max, bits);
    case DM_KP:
        return float_to_uint_generic(x_float, cls->kp_min, cls->kp_max, bits);
    case DM_TORQUE:
        return float_to_uint_generic(x_float, cls->torque_min, cls->torque_max, bits);
    default:
        return 0;
    }
}

/*============================== 面向对象接口实现 ==============================*/

/// @brief 使用指定的电机类创建电机实例
void DM_Motor_Create(MotorTypeDef *motor, const DM_MotorClass_t *motor_class, uint8_t id)
{
    if (motor == NULL || motor_class == NULL)
        return;
    if (id < 1 || id > 4)
        return;

    // 关联电机类
    motor->motor_class.dm_motor_class = motor_class;

    // 调用类的初始化函数
    if (motor_class->init != NULL)
    {
        motor_class->init(motor, id);
    }
}

/// @brief 调用电机的解算函数
void DM_Motor_Calculate(MotorTypeDef *motor)
{
    if (motor == NULL)
        return;

    // 优先使用类的虚函数表
    if (motor->motor_class.dm_motor_class != NULL && motor->motor_class.dm_motor_class->calculate != NULL)
    {
        motor->motor_class.dm_motor_class->calculate(motor);
        return;
    }

    // 兼容旧接口：使用直接绑定的函数指针
    if (motor->calculate != NULL)
    {
        motor->calculate(motor);
    }
}

/// @brief 获取电机所属的类指针
const DM_MotorClass_t *DM_Motor_GetClass(MotorTypeDef *motor)
{
    if (motor == NULL)
        return NULL;
    if (motor->MotorInf.band != DM_MOTOR_BAND)
        return NULL;
    return motor->motor_class.dm_motor_class;
}

/*============================== 电机初始化函数 ==============================*/

/// @brief 初始化DM电机基础属性（使用电机类）
static void DM_Motor_InitWithClass(MotorTypeDef *motor, uint8_t id,
                                   const DM_MotorClass_t *motor_class)
{
    memset(motor, 0, sizeof(MotorTypeDef));

    // 关联电机类（推荐通过motor_class访问虚函数）
    motor->motor_class.dm_motor_class = motor_class;

    // 基本属性
    motor->MotorID = id;
    motor->MotorInf.band = DM_MOTOR_BAND;
    motor->MotorInf.model = motor_class->model;

    // 默认配置
    motor->config.direction_bias = 0.0f;
    motor->config.position_tolerance = 50.0f;
    motor->config.reverse = 0;

    // 绑定函数指针（已弃用，保留用于向后兼容，新代码请使用motor_class虚函数表）
    motor->SendMotorControl = DM_Motor_SendControlInternal;
    motor->calculate = motor_class->calculate;

    // CAN报文头（发送ID由CanRegisterMotorCfg从配置表统一设置）
    motor->g_TxHeader.StdId = 0; // 占位，由CanRegisterMotorCfg设置
    motor->g_TxHeader.IDE = CAN_ID_STD;
    motor->g_TxHeader.RTR = CAN_RTR_DATA;
    motor->g_TxHeader.DLC = CtrlMotorLen;

    // 初始化DM特有配置（使用类的默认参数）
    if (id > 0 && id <= 4)
    {
        g_DM_Configs[id - 1].kp = motor_class->default_kp;
        g_DM_Configs[id - 1].kd = motor_class->default_kd;
        g_DM_Configs[id - 1].torque_ff = motor_class->default_torque_ff;

        // 从类的默认值初始化位置和速度限幅参数
        g_DM_Configs[id - 1].pos_max = motor_class->pos_max;
        g_DM_Configs[id - 1].pos_min = motor_class->pos_min;
        g_DM_Configs[id - 1].vel_max = motor_class->vel_max;
        g_DM_Configs[id - 1].vel_min = motor_class->vel_min;
    }
}

static void DM_J3519_InitInternal(MotorTypeDef *motor, uint8_t id)
{
    DM_Motor_InitWithClass(motor, id, &DM_J3519_Class);
}

static void DM_J4310_InitInternal(MotorTypeDef *motor, uint8_t id)
{
    DM_Motor_InitWithClass(motor, id, &DM_J4310_Class);
}

/*============================== 兼容旧接口的初始化函数 ==============================*/

void DM_J3519_Init(MotorTypeDef *motor, uint8_t id)
{
    if (motor == NULL || id < 1)
        return;
    DM_Motor_Create(motor, &DM_J3519_Class, id);
}

void DM_J4310_Init(MotorTypeDef *motor, uint8_t id)
{
    if (motor == NULL || id < 1)
        return;
    DM_Motor_Create(motor, &DM_J4310_Class, id);
}

/*============================== 电机配置函数 ==============================*/

void DM_Motor_SetConfig(MotorTypeDef *motor, const DM_MotorConfig_t *config)
{
    if (motor == NULL || config == NULL)
        return;
    if (motor->MotorInf.band != DM_MOTOR_BAND)
        return;

    uint8_t id = motor->MotorID;
    if (id > 0 && id <= 4)
    {
        g_DM_Configs[id - 1] = *config;
    }
}

DM_MotorConfig_t *DM_Motor_GetConfig(MotorTypeDef *motor)
{
    if (motor == NULL)
        return NULL;
    if (motor->MotorInf.band != DM_MOTOR_BAND)
        return NULL;

    uint8_t id = motor->MotorID;
    if (id > 0 && id <= 4)
    {
        return &g_DM_Configs[id - 1];
    }
    return NULL;
}

void DM_Motor_SetCascadePID(MotorTypeDef *motor,
                            float outer_p, float outer_i, float outer_d, float outer_f,
                            float inner_p, float inner_i, float inner_d, float inner_f,
                            float outer_max_out, float outer_min_out, float outer_max_iout,
                            float inner_max_out, float inner_min_out, float inner_max_iout)
{
    if (motor == NULL)
        return;

    motor->use_cascade = 1;
    CASCADE_PID_Init(&motor->cascade_pid,
                     outer_p, outer_i, outer_d, outer_f,
                     inner_p, inner_i, inner_d, inner_f,
                     outer_max_out, outer_min_out, outer_max_iout,
                     inner_max_out, inner_min_out, inner_max_iout);
    CASCADE_PID_Clear(&motor->cascade_pid);
}

void DM_Motor_SetSpeedPID(MotorTypeDef *motor, PID_MODE_e mode,
                          float p, float i, float d, float f,
                          float max_out, float min_out, float max_iout)
{
    if (motor == NULL)
        return;

    motor->use_cascade = 0;
    PID_Init(&motor->inner_pid, mode, p, i, d, f, max_out, min_out, max_iout);
    PID_Clear(&motor->inner_pid);
}

/*============================== 单独配置参数函数 ==============================*/

void DM_Motor_SetKp(MotorTypeDef *motor, float kp)
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor);
    if (cfg == NULL)
        return;

    // 获取电机类进行限幅
    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class;
    if (cls != NULL)
    {
        if (kp < cls->kp_min)
            kp = cls->kp_min;
        if (kp > cls->kp_max)
            kp = cls->kp_max;
    }

    cfg->kp = kp;
}

void DM_Motor_SetKd(MotorTypeDef *motor, float kd)
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor);
    if (cfg == NULL)
        return;

    // 获取电机类进行限幅
    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class;
    if (cls != NULL)
    {
        if (kd < cls->kd_min)
            kd = cls->kd_min;
        if (kd > cls->kd_max)
            kd = cls->kd_max;
    }

    cfg->kd = kd;
}

void DM_Motor_SetTorqueFF(MotorTypeDef *motor, float torque_ff)
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor);
    if (cfg == NULL)
        return;

    // 获取电机类进行限幅
    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class;
    if (cls != NULL)
    {
        if (torque_ff < cls->torque_min)
            torque_ff = cls->torque_min;
        if (torque_ff > cls->torque_max)
            torque_ff = cls->torque_max;
    }

    cfg->torque_ff = torque_ff;
}

void DM_Motor_SetPosLimits(MotorTypeDef *motor, float pos_min, float pos_max)
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor);
    if (cfg == NULL)
        return;

    cfg->pos_min = pos_min;
    cfg->pos_max = pos_max;
}

void DM_Motor_SetVelLimits(MotorTypeDef *motor, float vel_min, float vel_max)
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor);
    if (cfg == NULL)
        return;

    cfg->vel_min = vel_min;
    cfg->vel_max = vel_max;
}

void DM_Motor_SetMITParams(MotorTypeDef *motor, float kp, float kd, float torque_ff)
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor);
    if (cfg == NULL)
        return;

    // 获取电机类进行限幅
    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class;
    if (cls != NULL)
    {
        // KP限幅
        if (kp < cls->kp_min)
            kp = cls->kp_min;
        if (kp > cls->kp_max)
            kp = cls->kp_max;

        // KD限幅
        if (kd < cls->kd_min)
            kd = cls->kd_min;
        if (kd > cls->kd_max)
            kd = cls->kd_max;

        // 力矩限幅
        if (torque_ff < cls->torque_min)
            torque_ff = cls->torque_min;
        if (torque_ff > cls->torque_max)
            torque_ff = cls->torque_max;
    }

    cfg->kp = kp;
    cfg->kd = kd;
    cfg->torque_ff = torque_ff;
}

float DM_Motor_GetKp(MotorTypeDef *motor)
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor);
    if (cfg == NULL)
        return 0.0f;
    return cfg->kp;
}

float DM_Motor_GetKd(MotorTypeDef *motor)
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor);
    if (cfg == NULL)
        return 0.0f;
    return cfg->kd;
}

float DM_Motor_GetTorqueFF(MotorTypeDef *motor)
{
    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor);
    if (cfg == NULL)
        return 0.0f;
    return cfg->torque_ff;
}

/*============================== 使能/失能函数 ==============================*/

uint8_t DM_Motor_Enable(MotorTypeDef *motor)
{
    if (motor == NULL)
        return 0;

    const MotorHAL_t *hal = Motor_GetHAL();
    if (hal->can_send(&motor->g_TxHeader, (uint8_t *)DM_MOTOR_ENABLE))
    {
        if (motor->MotorID > 0 && motor->MotorID <= 4)
        {
            DM_ENABLE_ARR[motor->MotorID - 1] = true;
        }
        hal->delay_ms(1);
        return 1;
    }
    return 0;
}

uint8_t DM_Motor_Disable(MotorTypeDef *motor)
{
    if (motor == NULL)
        return 0;

    const MotorHAL_t *hal = Motor_GetHAL();
    if (hal->can_send(&motor->g_TxHeader, (uint8_t *)DM_MOTOR_DISABLE))
    {
        if (motor->MotorID > 0 && motor->MotorID <= 4)
        {
            DM_ENABLE_ARR[motor->MotorID - 1] = false;
        }
        hal->delay_ms(1);
        return 1;
    }
    return 0;
}

uint8_t DM_MotorDisable(can_motor_cfg motor_cfg)
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    if (motor == NULL)
        return 0;

    const MotorHAL_t *hal = Motor_GetHAL();
    if (hal->can_send(&(motor->g_TxHeader), (uint8_t *)DM_MOTOR_DISABLE))
    {
        if (motor->MotorID > 0 && motor->MotorID <= g_DM_MOTOR_NUM)
        {
            DM_ENABLE_ARR[motor->MotorID - 1] = false;
        }
        hal->delay_ms(1);
        return 1;
    }
    return 0;
}

uint8_t DM_MotorEnable(can_motor_cfg motor_cfg)
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    if (motor == NULL)
        return 0;

    const MotorHAL_t *hal = Motor_GetHAL();
    const uint32_t enable_timeout_ms = 500U;
    uint32_t start_tick = HAL_GetTick();

    while (1)
    {
        if ((uint32_t)(HAL_GetTick() - start_tick) >= enable_timeout_ms)
        {
            return 0;
        }

        // 发送使能命令
        if (!hal->can_send(&(motor->g_TxHeader), (uint8_t *)DM_MOTOR_ENABLE))
        {
            // CAN发送失败，继续重试
            vTaskDelay(pdMS_TO_TICKS(3));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(3));

        // 检查反馈帧的使能状态（byte0高4位为状态：0=失能，1=使能）
        uint8_t state = (motor->ReceiveMotorData[0] >> 4) & 0x0F;
        if (state == 0x01)
        {
            // 使能成功
            if (motor->MotorID > 0 && motor->MotorID <= g_DM_MOTOR_NUM)
            {
                DM_ENABLE_ARR[motor->MotorID - 1] = true;
            }
            return 1;
        }
        // 未使能，继续循环重试
    }
}

/*============================== 刷新数据函数 ==============================*/
void DM_Motor_RefreshData(can_motor_cfg motor_cfg)
{
    MotorTypeDef *st = &MotorManager.MotorList[motor_cfg - 1];
    if (st == NULL)
    {
        return;
    }

    const MotorHAL_t *hal = Motor_GetHAL();
    if (hal->can_send(&(st->g_TxHeader), (uint8_t *)DM_MOTOR_ENABLE))
    {
        if (st->MotorID > 0 && st->MotorID <= g_DM_MOTOR_NUM)
        {
            DM_ENABLE_ARR[st->MotorID - 1] = true;
        }
        hal->delay_ms(1);
    }
}

/*============================== 发送控制函数 ==============================*/

// 记录上次发送结束时间，防止连续调用导致CAN帧丢失
static uint32_t s_DM_LastSendTick = 0;

static uint8_t DM_Motor_SendControlInternal(MotorTypeDef *st)
{
    if (st == NULL)
        return 0;

    // 检查距离上次发送是否有至少1tick间隔，防止连续调用丢帧
    uint32_t current_tick = HAL_GetTick();
    if (current_tick - s_DM_LastSendTick < 1)
    {
        vTaskDelay(1);
    }

    // 自动使能（合并条件判断）
    if (st->MotorID > 0 && st->MotorID <= 4 && !DM_ENABLE_ARR[st->MotorID - 1])
    {
        DM_Motor_Enable(st);
    }

    // 使用Fast版本（内联，零开销）
    uint8_t result = Motor_GetHAL_Fast()->can_send(&st->g_TxHeader, st->SendMotorData) ? 1 : 0;

    // 更新上次发送结束时间
    s_DM_LastSendTick = HAL_GetTick();

    return result;
}

/***********************************
 * 函数名: DM_MotorSetTxData
 * 作用:   用于设置发送数据
 * 参数:   motor_cfg 电机名称
 * 参数:   data      数据指针
 * 返回值: 无
 * todo:   当前还需要加上各个电机的模式选择,不同的模式调用不同的发送函数
 **********************************/
void DM_MotorSetTxData(can_motor_cfg motor_cfg, uint8_t *data)
{
    if (data == NULL)
    {
        Error_Handler();
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    if (motor == NULL)
    {
        return;
    }

    memset(motor->SendMotorData, 0x00, CtrlMotorLen);
    memcpy(motor->SendMotorData, data, CtrlMotorLen);
    motor->SendMotorControl(motor); // 这里调用发送函数
}

/*============================== 电机解算函数 ==============================*/
// note:关节电机是可以直接读取并且不做减速比

/// @brief J3519电机解算（内部版本，优化：使用Fast版本获取配置）
static void DM_J3519_CalculateInternal(MotorTypeDef *motor)
{
    if (motor == NULL)
        return;

    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class;
    if (cls == NULL)
        cls = &DM_J3519_Class;

    MotorSolvedData_t *pData = &motor->motor_data;
    uint8_t *rx = motor->ReceiveMotorData;

    // 解析原始数据
    uint16_t pos_raw = ((uint16_t)rx[1] << 8) | rx[2];
    uint16_t vel_raw = ((uint16_t)rx[3] << 4) | (rx[4] >> 4);
    uint16_t tor_raw = ((rx[4] & 0x0F) << 8) | rx[5];

    // J3519: 使用电机类的固定范围进行解算（不受SetPosLimits/SetVelLimits影响）
    pData->solved_data[0] = uint_to_float_generic(pos_raw, cls->pos_min, cls->pos_max, 16);

    // 速度滤波
    float vel_new = uint_to_float_generic(vel_raw, cls->vel_min, cls->vel_max, 12);
    if (!pData->filter_init)
    {
        pData->solved_data[1] = vel_new;
        pData->filter_init = 1;
    }
    else
    {
        pData->solved_data[1] = (DM_SPEED_FILTER_COEF * pData->last_speed + (1.0f - DM_SPEED_FILTER_COEF) * vel_new);
    }
    pData->last_speed = pData->solved_data[1];

    // 力矩
    pData->solved_data[2] = uint_to_float_generic(tor_raw, cls->torque_min, cls->torque_max, 12);

    // 温度
    pData->solved_data[3] = (float)rx[6];
    pData->solved_data[4] = (float)rx[7];

    pData->solved_data[5] = pData->solved_data[0] / 19.2f; // 转过的角度
    pData->solved_data[6] = pData->solved_data[1] / 19.2f; // 转过的速度

    DM_Check3519StallInFeedback(motor);
}

/// @brief J4310电机解算（内部版本，优化：使用Fast版本获取配置）
static void DM_J4310_CalculateInternal(MotorTypeDef *motor)
{
    if (motor == NULL)
        return;

    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class;
    if (cls == NULL)
        cls = &DM_J4310_Class;

    MotorSolvedData_t *pData = &motor->motor_data;
    uint8_t *rx = motor->ReceiveMotorData;

    // 解析原始数据
    uint16_t pos_raw = ((uint16_t)rx[1] << 8) | rx[2];
    uint16_t vel_raw = ((uint16_t)rx[3] << 4) | (rx[4] >> 4);
    uint16_t tor_raw = ((rx[4] & 0x0F) << 8) | rx[5];

    // J4310: 使用电机类的固定范围进行解算（不受SetPosLimits/SetVelLimits影响）
    pData->solved_data[0] = uint_to_float_generic(pos_raw, cls->pos_min, cls->pos_max, 16);

    // 速度滤波
    float vel_new = uint_to_float_generic(vel_raw, cls->vel_min, cls->vel_max, 12);
    if (!pData->filter_init)
    {
        pData->solved_data[1] = vel_new;
        pData->filter_init = 1;
    }
    else
    {
        pData->solved_data[1] = DM_SPEED_FILTER_COEF * pData->last_speed + (1.0f - DM_SPEED_FILTER_COEF) * vel_new;
    }
    pData->last_speed = pData->solved_data[1];

    // 力矩
    pData->solved_data[2] = uint_to_float_generic(tor_raw, cls->torque_min, cls->torque_max, 12);

    // 温度
    pData->solved_data[3] = (float)rx[6];
    pData->solved_data[4] = (float)rx[7];
}

void DM_MOTOR_CALCU(MotorTypeDef *motor)
{
    if (motor == NULL)
        return;

    // 如果设置了虚函数，使用虚函数
    if (motor->calculate != NULL)
    {
        motor->calculate(motor);
        return;
    }

    // // 否则根据型号调用对应解算
    // switch (motor->MotorInf.model)
    // {
    // case DmS3519:
    //     DM_J3519_Calculate(motor);
    //     break;
    // case DmJ4310:
    //     DM_J4310_Calculate(motor);
    //     break;
    // default:
    //     break;
    // }
}

static void DM_SendMITPIDFrame(can_motor_cfg motor_cfg, float target_pos, float target_vel, float torque_ff)
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    if (motor == NULL || motor->MotorInf.band != DM_MOTOR_BAND)
        return;

    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class;
    if (cls == NULL)
        return;

    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor);
    if (cfg == NULL)
        return;

    uint8_t data[8] = {0x00};
    uint16_t KP_RESULT = float_to_uint_config(cfg->kp, DM_KP, DM_MIT, cfg, cls);
    uint16_t KD_RESULT = float_to_uint_config(cfg->kd, DM_KD, DM_MIT, cfg, cls);
    uint16_t Torque_ff = float_to_uint_config(torque_ff, DM_TORQUE, DM_MIT, cfg, cls);
    uint16_t Pos_des = float_to_uint_config(target_pos, DM_POS, DM_MIT, cfg, cls);
    uint16_t Vel_des = float_to_uint_config(target_vel, DM_VEL, DM_MIT, cfg, cls);

    data[0] = Pos_des >> 8;
    data[1] = (uint8_t)Pos_des;
    data[2] = Vel_des >> 4;
    data[3] = ((Vel_des & 0x000F) << 4) | ((KP_RESULT & 0x0F00) >> 8);
    data[4] = KP_RESULT;
    data[5] = KD_RESULT >> 4;
    data[6] = ((KD_RESULT & 0x000F) << 4) | ((Torque_ff & 0x0F00) >> 8);
    data[7] = Torque_ff;

    DM_MotorSetTxData(motor_cfg, data);
}

// 通用电机使用，可以是J4310也可以是J3519电机
void DmMotorSendCfg(can_motor_cfg motor_cfg, float TargetPos, float TargetVel, float TargetTorque, DM_WorkMode workmode)
{
    // 获取电机结构体并使用其配置参数（使用接口函数，解耦）
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    if (motor == NULL)
        return;
    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class;
    if (cls == NULL)
        return;

    DM_MotorConfig_t *cfg = DM_Motor_GetConfig(motor);
    if (cfg == NULL)
        return;

    uint8_t data[8] = {0x00};
    if (workmode == DM_MIT)
    {
        // 使用用户配置的参数进行转换（修复：使用cfg而非cls->default）
        uint16_t KP_RESULT = float_to_uint_config(cfg->kp, DM_KP, DM_MIT, cfg, cls);
        uint16_t KD_RESULT = float_to_uint_config(cfg->kd, DM_KD, DM_MIT, cfg, cls);
        uint16_t Torque_ff = float_to_uint_config(TargetTorque, DM_TORQUE, DM_MIT, cfg, cls);

        // 直接调PID电流
        uint16_t Pos_des = float_to_uint_config(0.0f, DM_POS, DM_MIT, cfg, cls);
        uint16_t Vel_des = float_to_uint_config(0.0f, DM_VEL, DM_MIT, cfg, cls);

        data[0] = Pos_des >> 8;
        data[1] = (uint8_t)Pos_des;
        data[2] = Vel_des >> 4;
        data[3] = ((Vel_des & 0x000F) << 4) | ((KP_RESULT & 0x0F00) >> 8);
        data[4] = KP_RESULT;
        data[5] = KD_RESULT >> 4;
        data[6] = ((KD_RESULT & 0x000F) << 4) | ((Torque_ff & 0x0F00) >> 8);
        data[7] = Torque_ff;
    }
    else if (workmode == DM_LOCATION_SPEED)
    {
        DM_Record3519TargetChange(motor_cfg, TargetPos);

        // 位置速度模式：直接发送float原始字节（小端序）
        uint8_t *pbuf = (uint8_t *)&TargetPos;
        uint8_t *vbuf = (uint8_t *)&TargetVel;

        data[0] = pbuf[0];
        data[1] = pbuf[1];
        data[2] = pbuf[2];
        data[3] = pbuf[3];
        data[4] = vbuf[0];
        data[5] = vbuf[1];
        data[6] = vbuf[2];
        data[7] = vbuf[3];
    }
    else if (workmode == DM_SPEED)
    {
        // 速度模式：直接发送float原始字节（小端序）
        uint8_t *vbuf = (uint8_t *)&TargetVel;

        data[0] = vbuf[0];
        data[1] = vbuf[1];
        data[2] = vbuf[2];
        data[3] = vbuf[3];
        data[4] = 0;
        data[5] = 0;
        data[6] = 0;
        data[7] = 0;
    }
    else if (workmode == DM_PVT)
    {
        // 达妙官方似乎也不管这个模式了
        // PVT模式待实现 - 需要查阅官方文档确认数据格式
        // TODO: 实现PVT模式
        return;
    }
    DM_MotorSetTxData(motor_cfg, data);
}

void DmMotorPID_Calc(can_motor_cfg motor_cfg, float target)
{
    if (motor_cfg < DM_3519_STRENTH_LEFT || motor_cfg > DM_4310_YAW)
        return;

    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    if (motor == NULL || motor->MotorInf.band != DM_MOTOR_BAND)
        return;

    MotorSolvedData_t *pData = &motor->motor_data;
    DM_HandleAngleLoopDirectionChange(motor, target, pData->solved_data[0]);

    const float target_speed = PID_Calculate(&motor->cascade_pid.outer, target, pData->solved_data[0]);
    s_dm_motor_pid_output = PID_Calculate(&motor->cascade_pid.inner, target_speed, pData->solved_data[1]);
    // s_dm_motor_pid_output = PID_Calculate(&motor->inner_pid, target, pData->solved_data[1]); // 用于测试单环调参
    s_dm_motor_pid_output = DM_ApplyOutputLimit(motor, s_dm_motor_pid_output);

    // 默认使用MIT模式
    DmMotorSendCfg(motor_cfg, 0.0f, 0.0f, s_dm_motor_pid_output, DM_MIT);
}

void DmMotorSpeedPID_Calc(can_motor_cfg motor_cfg, float target_speed_rpm)
{
    if (motor_cfg < DM_3519_STRENTH_LEFT || motor_cfg > DM_4310_YAW)
        return;

    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    if (motor == NULL || motor->MotorInf.band != DM_MOTOR_BAND)
        return;

    MotorSolvedData_t *pData = &motor->motor_data;

    s_dm_motor_pid_output = PID_Calculate(&motor->inner_pid, target_speed_rpm, pData->solved_data[1]);
    s_dm_motor_pid_output = DM_ApplyOutputLimit(motor, s_dm_motor_pid_output);

    // 默认使用MIT模式
    DmMotorSendCfg(motor_cfg, 0.0f, 0.0f, s_dm_motor_pid_output, DM_MIT);
}

bool DM_Motor_Is3519StallProtected(void)
{
    return (s_dm3519_stall_protected != 0U);
}

void DM_Motor_Clear3519StallProtection(void)
{
    taskENTER_CRITICAL();
    s_dm3519_stall_protected = 0U;
    DM_Reset3519StallStateByIndex(0, MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1].motor_data.solved_data[0]);
    DM_Reset3519StallStateByIndex(1, MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1].motor_data.solved_data[0]);
    taskEXIT_CRITICAL();
}
