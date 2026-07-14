/*****************************************************
 * RM电机控制模块
 * 适配 M3508、M2006、GM6020 等系列电机
 ****************************************************/

#include "RM_Motor.h"
#include "CanMotor.h"
#include "config.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

extern MotorManager_t MotorManager;
extern SemaphoreHandle_t g_xRmBufferMutexHandle; // 互斥访问

static float s_rm_motor_pid_output = 0.0f;
float g_RmDebugStoreLeftPosDeg = 0.0f;
float g_RmDebugStoreRightPosDeg = 0.0f;
float g_RmDebugStoreLeftPidOutput = 0.0f;
float g_RmDebugStoreRightPidOutput = 0.0f;
int16_t g_RmDebugStoreLeftFinalCurrent = 0;
int16_t g_RmDebugStoreRightFinalCurrent = 0;
uint8_t g_RmDebugStoreLeftLimitBlocked = 0U;
uint8_t g_RmDebugStoreRightLimitBlocked = 0U;
float g_testVariable = 0.0f;

/* 左右蓄力 3508 位置同步 PID：由 MotorRegister 初始化。*/
static PID_t s_rm_store_sync_pid;
static uint8_t s_rm_store_sync_pid_inited = 0U;
float g_RmStoreSyncErrorDeg = 0.0f;
float g_RmStoreSyncPidOutputDeg = 0.0f;

float g_RmDebugLoadCurrentFilt = 0.0f;
uint32_t g_RmDebugLoadOverCurMs = 0U;
uint32_t g_RmDebugLoadClearMs = 0U;
uint8_t g_RmDebugLoadStallTripped = 0U;

#define CtrlMotorLen 8
#define ECD_TO_DEGREE 0.04394531250f // (360.0f / 8192.0f)
#define RM_M2006_CURRENT_INV (1.0f / RM_M2006_CURRENT_RATIO)
#define RM_M3508_CURRENT_INV (1.0f / RM_M3508_CURRENT_RATIO)
#define RM_GM6020_CURRENT_INV (1.0f / RM_GM6020_CURRENT_RATIO)
#define RM_DEFAULT_POS_MIN (-10000.0f)
#define RM_DEFAULT_POS_MAX (10000.0f)
#define RM_DEFAULT_POS_TOLERANCE (3.0f)

static inline void RM_Motor_CalculateCommon(MotorTypeDef *motor, float current_inv);
static void RM_M2006_InitInternal(MotorTypeDef *motor, uint8_t id);
static void RM_M2006_CalculateInternal(MotorTypeDef *motor);
static void RM_M3508_InitInternal(MotorTypeDef *motor, uint8_t id);
static void RM_M3508_CalculateInternal(MotorTypeDef *motor);
static void RM_GM6020_InitInternal(MotorTypeDef *motor, uint8_t id);
static void RM_GM6020_CalculateInternal(MotorTypeDef *motor);
static uint8_t RM_Motor_SendControlLocked(MotorTypeDef *motor);
static inline int16_t RM_Motor_ApplyOutputLimit(can_motor_cfg motor_cfg, MotorTypeDef *motor, float pid_output);
static inline void RM_Motor_HandleAngleLoopDirectionChange(MotorTypeDef *motor, float target_angle, float current_angle);
static inline float RM_Motor_ClampFloat(float value, float min, float max);
static inline bool RM_Motor_IsSameTxGroup(const MotorTypeDef *lhs, const MotorTypeDef *rhs);

// M2006虚拟类
const RM_MotorClass_t RM_M2006_Class = {
    .name = "M2006",
    .model = RmM2006,
    .gear_ratio = RM_M2006_GEAR_RATIO,
    .max_current = RM_M2006_MAX_CURRENT,
    .current_ratio = RM_M2006_CURRENT_RATIO,
    .tx_base_addr = g_RM_MOTOR_BIAS_ADDR_2006,
    .id_min = 1,
    .id_max = 4,
    .init = RM_M2006_InitInternal,
    .calculate = RM_M2006_CalculateInternal,
};

// M3508虚拟类
const RM_MotorClass_t RM_M3508_Class = {
    .name = "M3508",
    .model = RmM3508,
    .gear_ratio = RM_M3508_GEAR_RATIO,
    .max_current = RM_M3508_MAX_CURRENT,
    .current_ratio = RM_M3508_CURRENT_RATIO,
    .tx_base_addr = g_RM_MOTOR_BIAS_ADDR_3508,
    .id_min = 1,
    .id_max = 4,
    .init = RM_M3508_InitInternal,
    .calculate = RM_M3508_CalculateInternal,
};

// GM6020虚拟类
const RM_MotorClass_t RM_GM6020_Class = {
    .name = "GM6020",
    .model = RmGM6020,
    .gear_ratio = RM_GM6020_GEAR_RATIO,
    .max_current = RM_GM6020_MAX_CURRENT,
    .current_ratio = RM_GM6020_CURRENT_RATIO,
    .tx_base_addr = g_RM_MOTOR_BIAS_ADDR_6020,
    .id_min = 1,
    .id_max = 7,
    .init = RM_GM6020_InitInternal,
    .calculate = RM_GM6020_CalculateInternal,
};

// 创建电机对应实体
void RM_Motor_Create(MotorTypeDef *motor, const RM_MotorClass_t *motor_class, uint8_t id)
{
    if (motor == NULL || motor_class == NULL)
    {
        return;
    }

    if (id < motor_class->id_min || id > motor_class->id_max)
    {
        return;
    }

    motor->motor_class.rm_motor_class = motor_class;
    if (motor_class->init != NULL)
    {
        motor_class->init(motor, id);
    }
}

// 电机解算回调函数API
void RM_Motor_Calculate(MotorTypeDef *motor)
{
    if (motor == NULL)
    {
        return;
    }

    if (motor->motor_class.rm_motor_class != NULL &&
        motor->motor_class.rm_motor_class->calculate != NULL)
    {
        motor->motor_class.rm_motor_class->calculate(motor);
    }
}

/// @brief 初始化RM电机(父类函数)
/// @param motor 电机对应结构体地址
/// @param id 电机ID
/// @param motor_class 对应RM电机虚拟类
static inline void RM_Motor_InitBase(MotorTypeDef *motor, uint8_t id, const RM_MotorClass_t *motor_class)
{
    memset(motor, 0, sizeof(MotorTypeDef));

    motor->motor_class.rm_motor_class = motor_class;
    motor->MotorID = id;
    motor->MotorInf.band = RM_MOTOR_BAND;
    motor->MotorInf.model = motor_class->model;
    motor->params.gear_ratio = motor_class->gear_ratio;
    motor->params.max_current = motor_class->max_current;
    motor->params.current_ratio = motor_class->current_ratio;

    // RM 电机默认配置：
    // position_min/max 用于位置环目标限位；
    // position_tolerance 用于位置到位死区；
    // direction_bias/reverse 用于安装方向修正。
    motor->config.direction_bias = 0.0f;
    motor->config.position_min = RM_DEFAULT_POS_MIN;
    motor->config.position_max = RM_DEFAULT_POS_MAX;
    motor->config.position_tolerance = RM_DEFAULT_POS_TOLERANCE;
    motor->config.reverse = 0;

    // S 型规划器默认不注册，只有在电机注册阶段显式配置后才启用。
    memset(&motor->trap_config, 0, sizeof(motor->trap_config));

    motor->calculate = motor_class->calculate;
    motor->g_TxHeader.IDE = CAN_ID_STD;
    motor->g_TxHeader.RTR = CAN_RTR_DATA;
    motor->g_TxHeader.DLC = CtrlMotorLen;
}

/// @brief M2006电机实例化初始化函数
/// @param motor 电机对应结构体地址
/// @param id 电机ID
static void RM_M2006_InitInternal(MotorTypeDef *motor, uint8_t id)
{
    RM_Motor_InitBase(motor, id, &RM_M2006_Class);
    motor->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_2006;
}

/// @brief M3508电机实例化初始化函数
/// @param motor 电机对应结构体地址
/// @param id 电机ID
static void RM_M3508_InitInternal(MotorTypeDef *motor, uint8_t id)
{
    RM_Motor_InitBase(motor, id, &RM_M3508_Class);
    motor->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_3508;
}

/// @brief GM6020电机实例化初始化函数
/// @param motor 电机对应结构体地址
/// @param id 电机ID
static void RM_GM6020_InitInternal(MotorTypeDef *motor, uint8_t id)
{
    RM_Motor_InitBase(motor, id, &RM_GM6020_Class);
    motor->g_TxHeader.StdId = (id <= 4U) ? (g_RM_MOTOR_BIAS_ADDR_6020 - 0x100U) : g_RM_MOTOR_BIAS_ADDR_6020;
}

/// @brief 创建M2006电机实例
/// @param motor 电机对应结构体地址
/// @param id 电机ID
void RM_M2006_Create(MotorTypeDef *motor, uint8_t id)
{
    if (motor != NULL && id >= 1U && id <= 4U)
    {
        RM_Motor_Create(motor, &RM_M2006_Class, id);
    }
}

/// @brief 创建M3508电机实例
/// @param motor 电机对应结构体地址
/// @param id 电机ID
void RM_M3508_Create(MotorTypeDef *motor, uint8_t id)
{
    if (motor != NULL && id >= 1U && id <= 4U)
    {
        RM_Motor_Create(motor, &RM_M3508_Class, id);
    }
}

/// @brief 创建GM6020电机实例
/// @param motor 电机对应结构体地址
/// @param id 电机ID
void RM_GM6020_Create(MotorTypeDef *motor, uint8_t id)
{
    if (motor != NULL && id >= 1U && id <= 7U)
    {
        RM_Motor_Create(motor, &RM_GM6020_Class, id);
    }
}

/// @brief 设置电机初始参数
/// @param motor 电机对应结构体地址
/// @param config 对应参数地址
void RM_Motor_SetConfig(MotorTypeDef *motor, const RM_MotorConfig_t *config)
{
    if (motor == NULL || config == NULL)
    {
        return;
    }

    // 上层注册的配置在这里统一写入底层通用配置结构，
    // 后续 PID、限位、死区等逻辑都只从 motor->config 读取。
    motor->config.direction_bias = config->direction_bias;         // 换向偏移补偿
    motor->config.position_min = config->position_min;             // 允许运动最小位置
    motor->config.position_max = config->position_max;             // 允许运动最大位置
    motor->config.position_tolerance = config->position_tolerance; // 位置环到位死区
    motor->config.reverse = config->reverse;                       // 正反向
}

/// @brief 设置RM电机串级PID参数
/// @param motor 电机对应结构体地址
/// @param outer_p 外环P
/// @param outer_i 外环I
/// @param outer_d 外环D
/// @param outer_f 外环F
/// @param inner_p 内环P
/// @param inner_i 内环I
/// @param inner_d 内环D
/// @param inner_f 内环F
/// @param outer_max_out 外环输出上限
/// @param outer_min_out 外环输出下限
/// @param outer_max_iout 外环积分输入上限
/// @param inner_max_out 内环输出上限
/// @param inner_min_out 内环输出下限
/// @param inner_max_iout 内环积分输入上限
/// @note  默认使用位置式PID
void RM_Motor_SetCascadePID(MotorTypeDef *motor,
                            float outer_p, float outer_i, float outer_d, float outer_f,
                            float inner_p, float inner_i, float inner_d, float inner_f,
                            float outer_max_out, float outer_min_out, float outer_max_iout,
                            float inner_max_out, float inner_min_out, float inner_max_iout)
{
    if (motor == NULL)
    {
        return;
    }

    motor->use_cascade = 1U;
    CASCADE_PID_Init(&motor->cascade_pid,
                     outer_p, outer_i, outer_d, outer_f,
                     inner_p, inner_i, inner_d, inner_f,
                     outer_max_out, outer_min_out, outer_max_iout,
                     inner_max_out, inner_min_out, inner_max_iout);
    CASCADE_PID_Clear(&motor->cascade_pid);
}

/// @brief 速度环PID设置
/// @param motor 电机对应结构体地址
/// @param mode PID模式
/// @param p KP
/// @param i KI
/// @param d KD
/// @param f KF
/// @param max_out 输出上限
/// @param min_out 输出下限
/// @param max_iout 积分输出上限
void RM_Motor_SetSpeedPID(MotorTypeDef *motor, PID_MODE_e mode,
                          float p, float i, float d, float f,
                          float max_out, float min_out, float max_iout)
{
    if (motor == NULL)
    {
        return;
    }

    motor->use_cascade = 0U;
    PID_Init(&motor->inner_pid, mode, p, i, d, f, max_out, min_out, max_iout);
    PID_Clear(&motor->inner_pid);
}

/// @brief 负责加锁和发送数据
/// @param motor 电机对应结构体地址
/// @return 发送成功与否
static uint8_t RM_Motor_SendControlLocked(MotorTypeDef *motor)
{
    uint8_t *send_buffer = MotorManager.RM_MOTOR_DATA_ARRAY;
    memset(send_buffer, 0x00, CtrlMotorLen);

    for (uint8_t i = 0; i < MotorManager.registered_count; i++)
    {
        MotorTypeDef *other = &MotorManager.MotorList[i];
        if (!RM_Motor_IsSameTxGroup(motor, other))
        {
            continue;
        }

        if (other->MotorID < 1U || other->MotorID > 4U)
        {
            continue;
        }

        uint8_t offset = (uint8_t)((other->MotorID - 1U) * 2U);
        send_buffer[offset] = other->SendMotorData[0];
        send_buffer[offset + 1U] = other->SendMotorData[1];
    }

    return Motor_GetHAL_Fast()->can_send(&motor->g_TxHeader, send_buffer);
}

/// @brief 设置发送数据
/// @param motor_cfg 电机别名
/// @param data 电机数据
static inline void RM_MotorSetTxData(can_motor_cfg motor_cfg, uint8_t *data)
{
    if (data == NULL)
    {
        return;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    if (motor->MotorInf.band != RM_MOTOR_BAND)
    {
        return;
    }

    if (xSemaphoreTake(g_xRmBufferMutexHandle, pdMS_TO_TICKS(2)) != pdTRUE)
    {
        return;
    }

    memset(motor->SendMotorData, 0x00, CtrlMotorLen);
    memcpy(motor->SendMotorData, data, CtrlMotorLen);
    (void)RM_Motor_SendControlLocked(motor);
    xSemaphoreGive(g_xRmBufferMutexHandle);
}

/// @brief 电机解算通用函数
/// @param motor 电机对应结构体地址
/// @param current_inv 电机对应的减速比
static inline void RM_Motor_CalculateCommon(MotorTypeDef *motor, float current_inv)
{
    // 暂存数据
    MotorSolvedData_t *data = &motor->motor_data;
    uint8_t *rx = motor->ReceiveMotorData;

    int16_t ecd = (int16_t)((((uint16_t)rx[0]) << 8) | rx[1]);
    int16_t speed_rpm = (int16_t)((((uint16_t)rx[2]) << 8) | rx[3]);
    int16_t current_raw = (int16_t)((((uint16_t)rx[4]) << 8) | rx[5]);

    // 确认是否初始化完成
    if (data->init_flag == 0U)
    {
        data->last_ecd = ecd;
        data->offset_ecd = ecd;
        data->offset_ecd_angle = ecd * ECD_TO_DEGREE;
        // offset_ecd_angle 要与 solved_data[3] 处在同一坐标系，否则
        // "solved_data[3] - offset_ecd_angle" 会跨坐标相减。
        if (motor->config.reverse != 0U)
        {
            data->offset_ecd_angle = -data->offset_ecd_angle;
        }
        data->init_flag = 1U;
    }

    int16_t err = (int16_t)(ecd - data->last_ecd);
    if (err > 4096)
    {
        data->total_round--;
        err -= 8192;
    }
    else if (err < -4096)
    {
        data->total_round++;
        err += 8192;
    }

    data->total_ecd += err;
    data->last_ecd = ecd;
    data->solved_data[0] = ecd * ECD_TO_DEGREE;

    if (data->filter_init == 0U)
    {
        data->solved_data[1] = (float)speed_rpm;
        data->filter_init = 1U;
    }
    else
    {
        data->solved_data[1] = (float)speed_rpm * 0.96f + data->last_speed * 0.04f;
    }

    data->solved_data[2] = current_raw * current_inv;
    data->solved_data[3] = (float)data->total_ecd * ECD_TO_DEGREE;
    data->solved_data[4] = (float)speed_rpm * 0.10472f;
    data->last_speed = data->solved_data[1];

    // reverse=1 时，把上层所见的反馈（角度、速度、电流）统一翻到逻辑坐标，
    // 后续 PID/限位/死区都按逻辑坐标工作；物理侧的反号只在 RmMotorSendCfg
    // 把 PID 输出电流再翻回来一次。
    if (motor->config.reverse != 0U)
    {
        data->solved_data[0] = -data->solved_data[0];
        data->solved_data[1] = -data->solved_data[1];
        data->solved_data[2] = -data->solved_data[2];
        data->solved_data[3] = -data->solved_data[3];
        data->solved_data[4] = -data->solved_data[4];
    }

    if (motor->MotorInf.model == RmM3508)
    {
        data->solved_data[5] = (float)rx[6];
        data->solved_data[6] = (float)rx[7];
    }
}

static void RM_M2006_CalculateInternal(MotorTypeDef *motor)
{
    if (motor != NULL)
    {
        RM_Motor_CalculateCommon(motor, RM_M2006_CURRENT_INV);
    }
}

static void RM_M3508_CalculateInternal(MotorTypeDef *motor)
{
    if (motor != NULL)
    {
        RM_Motor_CalculateCommon(motor, RM_M3508_CURRENT_INV);
    }
}

static void RM_GM6020_CalculateInternal(MotorTypeDef *motor)
{
    if (motor != NULL)
    {
        RM_Motor_CalculateCommon(motor, RM_GM6020_CURRENT_INV);
    }
}

void RM_MOTOR_CALCU(MotorTypeDef *motor)
{
    if (motor != NULL && motor->calculate != NULL)
    {
        motor->calculate(motor);
    }
}

void RM_Motor_Reset_Zero(MotorTypeDef *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->motor_data.total_round = 0;
    motor->motor_data.total_ecd = 0;
    motor->motor_data.offset_ecd = motor->motor_data.last_ecd;
    motor->motor_data.target_angle = motor->motor_data.solved_data[3];
    motor->motor_data.last_target = motor->motor_data.target_angle;
    motor->motor_data.pre_last_target = motor->motor_data.target_angle;
    motor->motor_data.target_init_flag = 0U;
}

void RM_Motor_Reset_All(void)
{
    for (uint8_t i = 0; i < g_RM_MOTOR_NUM; i++)
    {
        memset(&MotorManager.MotorList[i].motor_data, 0, sizeof(MotorSolvedData_t));
    }
}

/// @brief RM电机发送API
/// @param motor_cfg 电机别名
/// @param TargetCurrent 电机目标电流值
void RmMotorSendCfg(can_motor_cfg motor_cfg, int16_t TargetCurrent)
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    if (motor == NULL)
    {
        return;
    }

    if (motor->config.reverse != 0U)
    {
        TargetCurrent = (int16_t)(-TargetCurrent);
    }

    if (motor_cfg == RM_3508_STORE_LEFT)
    {
        g_RmDebugStoreLeftFinalCurrent = TargetCurrent;
    }
    else if (motor_cfg == RM_3508_STORE_RIGHT)
    {
        g_RmDebugStoreRightFinalCurrent = TargetCurrent;
    }
    if (TargetCurrent < 0)
    {
        TargetCurrent = (uint16_t)(~(-TargetCurrent));
    }

    uint8_t data[8] = {0};
    data[0] = (uint8_t)((uint16_t)TargetCurrent >> 8);
    data[1] = (uint8_t)TargetCurrent;
    RM_MotorSetTxData(motor_cfg, data);
}

/// @brief 这是一个虚假的限幅函数(目前作用只是四舍五入最后的发送数据)
/// @param motor_cfg 电机别名
/// @param motor 电机对应结构体地址
/// @param pid_output 电机PID输出
/// @return 四舍五入之后的数值
static inline int16_t RM_Motor_ApplyOutputLimit(can_motor_cfg motor_cfg, MotorTypeDef *motor, float pid_output)
{
    bool should_block = false;
    int16_t final_output = (int16_t)pid_output;
    float output_for_limit = pid_output;

    if (motor == NULL)
    {
        return final_output;
    }

    if (motor_cfg == RM_3508_STORE_LEFT)
    {
        float current_pos_deg = motor->motor_data.solved_data[3];
        g_RmDebugStoreLeftPosDeg = current_pos_deg;
        g_RmDebugStoreLeftPidOutput = pid_output;
        should_block = ((current_pos_deg >= STORE3508_LEFT_POS_MAX_DEG) && (output_for_limit > 0.0f)) ||
                       ((current_pos_deg <= STORE3508_LEFT_POS_MIN_DEG) && (output_for_limit < 0.0f));
        g_RmDebugStoreLeftLimitBlocked = (uint8_t)should_block;
    }
    else if (motor_cfg == RM_3508_STORE_RIGHT)
    {
        float current_pos_deg = motor->motor_data.solved_data[3];
        g_RmDebugStoreRightPosDeg = current_pos_deg;
        g_RmDebugStoreRightPidOutput = pid_output;
        should_block = ((current_pos_deg >= STORE3508_RIGHT_POS_MAX_DEG) && (output_for_limit > 0.0f)) ||
                       ((current_pos_deg <= STORE3508_RIGHT_POS_MIN_DEG) && (output_for_limit < 0.0f));
        g_RmDebugStoreRightLimitBlocked = (uint8_t)should_block;
    }
    else if (motor_cfg == RM_3508_GRIPPER)
    {
        // GRIPPER 过流/堵转保护的持久状态。g_RmDebug* 全局量同时作为示波器观测用途。
        // 当前 RmMotorPID_Calc 只在单一控制任务中周期触发，static + 临界区即可。
        static uint32_t s_last_tick = 0U;
        static uint8_t s_tick_init = 0U;
        static uint8_t s_tripped = 0U;

        uint32_t now = HAL_GetTick();
        float current_abs = fabsf(motor->motor_data.solved_data[2]);

        taskENTER_CRITICAL();
        uint32_t dt_ms = 0U;
        if (s_tick_init == 0U)
        {
            s_tick_init = 1U;
        }
        else
        {
            dt_ms = now - s_last_tick;
        }
        s_last_tick = now;

        g_RmDebugLoadCurrentFilt += LOAD3508_OVERCURRENT_FILTER_ALPHA * (current_abs - g_RmDebugLoadCurrentFilt);

        if (s_tripped == 0U)
        {
            if (g_RmDebugLoadCurrentFilt >= LOAD3508_STALL_OVERCURRENT_LIMIT_A)
            {
                g_RmDebugLoadOverCurMs += dt_ms;
                if (g_RmDebugLoadOverCurMs >= LOAD3508_STALL_OVERCURRENT_RETURN_MS)
                {
                    s_tripped = 1U;
                    g_RmDebugLoadClearMs = 0U;
                }
            }
            else
            {
                g_RmDebugLoadOverCurMs = 0U;
            }
        }
        else
        {
            if (g_RmDebugLoadCurrentFilt <= LOAD3508_STALL_OVERCURRENT_CLEAR_A)
            {
                g_RmDebugLoadClearMs += dt_ms;
                if (g_RmDebugLoadClearMs >= LOAD3508_STALL_OVERCURRENT_RETURN_MS)
                {
                    s_tripped = 0U;
                    g_RmDebugLoadOverCurMs = 0U;
                }
            }
            else
            {
                g_RmDebugLoadClearMs = 0U;
            }
        }

        g_RmDebugLoadStallTripped = s_tripped;
        uint8_t tripped_snapshot = s_tripped;
        taskEXIT_CRITICAL();

        if (tripped_snapshot != 0U)
        {
            should_block = true;
        }
    }

    if (should_block)
    {
        if (motor->use_cascade != 0U)
        {
            PID_Clear_Integral(&motor->cascade_pid.outer);
            PID_Clear_Integral(&motor->cascade_pid.inner);
        }
        else
        {
            PID_Clear_Integral(&motor->inner_pid);
        }
        final_output = 0;
    }

    return final_output;
}

/// @brief 处理角度环方向问题
/// @param motor 电机对应结构体
/// @param target_angle 电机目标角度
/// @param current_angle 电机当前角度
/// @note  主要用于处理换向时候的积分问题
static inline void RM_Motor_HandleAngleLoopDirectionChange(MotorTypeDef *motor, float target_angle, float current_angle)
{
#if RM_3508_CLEAR_ANGLE_I_ON_DIR_CHANGE
    if (motor == NULL)
    {
        return;
    }

    PID_t *outer_pid = &motor->cascade_pid.outer;
    if (!outer_pid->initialized || outer_pid->calc_count == 0U)
    {
        return;
    }

    float angle_error = target_angle - current_angle;
    float current_target_delta = target_angle - outer_pid->target;
    bool target_direction_reversed = (current_target_delta * outer_pid->feedforward) < 0.0f;
    bool error_direction_reversed = (angle_error * outer_pid->last_error) < 0.0f;
    if (target_direction_reversed || error_direction_reversed)
    {
        PID_Clear_Integral(outer_pid);
    }
#else
    (void)motor;
    (void)target_angle;
    (void)current_angle;
#endif
}

/// @brief 上下限限幅
/// @param value 当前需要限幅的数值
/// @param min 下限
/// @param max 上限
/// @return 限幅后的数值
static inline float RM_Motor_ClampFloat(float value, float min, float max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

/// @brief PID计算函数
/// @param motor_cfg 电机对应别名
/// @param target 电机目标值
/// @note 电机现在PID目标减小误差(以°为单位)
void RmMotorPID_Calc(can_motor_cfg motor_cfg, float target)
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    float current_pos_deg;
    if (motor == NULL)
    {
        return;
    }

    // 统一使用“相对零点角度”做位置环计算，
    // 这样目标值、限位、死区都在同一套坐标系下。
    if ((motor_cfg == RM_3508_STORE_LEFT) || (motor_cfg == RM_3508_STORE_RIGHT))
    {
        current_pos_deg = motor->motor_data.solved_data[3];
    }
    else
    {
        current_pos_deg = motor->motor_data.solved_data[3] - motor->motor_data.offset_ecd_angle;
    }

    if (motor->MotorInf.model == RmM3508)
    {
        // 先按注册的上下限约束目标位置，避免继续往机械极限方向推。
        float min_limit = motor->config.position_min;
        float max_limit = motor->config.position_max;
        if (min_limit > max_limit)
        {
            float tmp = min_limit;
            min_limit = max_limit;
            max_limit = tmp;
        }

        target = RM_Motor_ClampFloat(target, min_limit, max_limit);

        // position_tolerance 作为位置环到位死区：
        // 误差足够小时直接清积分并停输出，避免在小角度附近反复抖动。
        float tolerance = motor->config.position_tolerance;
        if (tolerance <= 0.0f)
        {
            tolerance = RM_DEFAULT_POS_TOLERANCE;
        }

        if (fabsf(target - current_pos_deg) <= tolerance)
        {
            if (motor->use_cascade != 0U)
            {
                PID_Clear_Integral(&motor->cascade_pid.outer);
                PID_Clear_Integral(&motor->cascade_pid.inner);
            }
            else
            {
                PID_Clear_Integral(&motor->inner_pid);
            }
            RmMotorSendCfg(motor_cfg, 0);
            return;
        }
    }

    RM_Motor_HandleAngleLoopDirectionChange(motor, target, current_pos_deg);
    MotorSolvedData_t *data = &motor->motor_data;
    if ((motor_cfg == RM_3508_STORE_LEFT) || (motor_cfg == RM_3508_STORE_RIGHT))
    {
        current_pos_deg = data->solved_data[3];
    }
    else
    {
        current_pos_deg = data->solved_data[3] - data->offset_ecd_angle; // update
    }
    if (motor->use_cascade != 0U)
    {
        s_rm_motor_pid_output = CASCADE_PID_Calculate(&motor->cascade_pid, target, current_pos_deg, data->solved_data[4]);
    }
    else
    {
        s_rm_motor_pid_output = PID_Calculate(&motor->inner_pid, target, data->solved_data[4]);
    }

    RmMotorSendCfg(motor_cfg, RM_Motor_ApplyOutputLimit(motor_cfg, motor, s_rm_motor_pid_output));
}

/// @brief 速度环PID调试
/// @param motor_cfg 电机对应别名
/// @param target_speed_rpm  目标转速
void RmMotorSpeedPID_Calc(can_motor_cfg motor_cfg, float target_speed_rpm)
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    if (motor == NULL)
    {
        return;
    }
    g_testVariable = motor->motor_data.solved_data[4];
    s_rm_motor_pid_output = PID_Calculate(&motor->inner_pid, target_speed_rpm, g_testVariable); // 这里调节速度环,但是得先看目标速度如何
    RmMotorSendCfg(motor_cfg, RM_Motor_ApplyOutputLimit(motor_cfg, motor, s_rm_motor_pid_output));
}

/*============================== 双侧蓄力 3508 位置同步 PID ==============================*/

void RM_Motor_InitStoreSyncPid(float kp, float ki, float kd, float kf,
                               float max_out, float min_out, float max_iout)
{
    PID_Init(&s_rm_store_sync_pid, PID_POSITION, kp, ki, kd, kf, max_out, min_out, max_iout);
    PID_Clear(&s_rm_store_sync_pid);
    g_RmStoreSyncErrorDeg = 0.0f;
    g_RmStoreSyncPidOutputDeg = 0.0f;
    s_rm_store_sync_pid_inited = 1U;
}

float RM_Motor_UpdateStoreSync(float left_pos_deg, float right_pos_deg)
{
    if (s_rm_store_sync_pid_inited == 0U)
    {
        g_RmStoreSyncErrorDeg = 0.0f;
        g_RmStoreSyncPidOutputDeg = 0.0f;
        return 0.0f;
    }

    g_RmStoreSyncErrorDeg = left_pos_deg - right_pos_deg;
    g_RmStoreSyncPidOutputDeg = PID_Calculate(&s_rm_store_sync_pid, 0.0f, g_RmStoreSyncErrorDeg);
    return g_RmStoreSyncPidOutputDeg;
}

/// @brief 检查打包帧
/// @param lhs
/// @param rhs
/// @return
static inline bool RM_Motor_IsSameTxGroup(const MotorTypeDef *lhs, const MotorTypeDef *rhs)
{
    if (lhs == NULL || rhs == NULL)
    {
        return false;
    }

    return lhs->MotorInf.band == RM_MOTOR_BAND &&
           rhs->MotorInf.band == RM_MOTOR_BAND &&
           lhs->model == rhs->model &&
           lhs->g_TxHeader.StdId == rhs->g_TxHeader.StdId;
}
