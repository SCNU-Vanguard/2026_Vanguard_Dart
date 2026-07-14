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

extern MotorManager_t MotorManager; /* 声明外部变量 MotorManager。 */
extern SemaphoreHandle_t g_xRmBufferMutexHandle; // 互斥访问

static float s_rm_motor_pid_output = 0.0f; /* 初始化 s_rm_motor_pid_output。 */
float g_RmDebugStoreLeftPosDeg = 0.0f; /* 初始化 g_RmDebugStoreLeftPosDeg。 */
float g_RmDebugStoreRightPosDeg = 0.0f; /* 初始化 g_RmDebugStoreRightPosDeg。 */
float g_RmDebugStoreLeftPidOutput = 0.0f; /* 初始化 g_RmDebugStoreLeftPidOutput。 */
float g_RmDebugStoreRightPidOutput = 0.0f; /* 初始化 g_RmDebugStoreRightPidOutput。 */
int16_t g_RmDebugStoreLeftFinalCurrent = 0; /* 初始化 g_RmDebugStoreLeftFinalCurrent。 */
int16_t g_RmDebugStoreRightFinalCurrent = 0; /* 初始化 g_RmDebugStoreRightFinalCurrent。 */
uint8_t g_RmDebugStoreLeftLimitBlocked = 0U; /* 初始化 g_RmDebugStoreLeftLimitBlocked。 */
uint8_t g_RmDebugStoreRightLimitBlocked = 0U; /* 初始化 g_RmDebugStoreRightLimitBlocked。 */
float g_testVariable = 0.0f; /* 初始化 g_testVariable。 */

/* 左右蓄力 3508 位置同步 PID：由 MotorRegister 初始化。*/
static PID_t s_rm_store_sync_pid; /* 保存 s_rm_store_sync_pid。 */
static uint8_t s_rm_store_sync_pid_inited = 0U; /* 初始化 s_rm_store_sync_pid_inited。 */
float g_RmStoreSyncErrorDeg = 0.0f; /* 初始化 g_RmStoreSyncErrorDeg。 */
float g_RmStoreSyncPidOutputDeg = 0.0f; /* 初始化 g_RmStoreSyncPidOutputDeg。 */

float g_RmDebugLoadCurrentFilt = 0.0f; /* 初始化 g_RmDebugLoadCurrentFilt。 */
uint32_t g_RmDebugLoadOverCurMs = 0U; /* 初始化 g_RmDebugLoadOverCurMs。 */
uint32_t g_RmDebugLoadClearMs = 0U; /* 初始化 g_RmDebugLoadClearMs。 */
uint8_t g_RmDebugLoadStallTripped = 0U; /* 初始化 g_RmDebugLoadStallTripped。 */

#define CtrlMotorLen 8 /* 定义 CtrlMotorLen。 */
#define ECD_TO_DEGREE 0.04394531250f // (360.0f / 8192.0f)
#define RM_M2006_CURRENT_INV (1.0f / RM_M2006_CURRENT_RATIO) /* 定义 RM_M2006_CURRENT_INV。 */
#define RM_M3508_CURRENT_INV (1.0f / RM_M3508_CURRENT_RATIO) /* 定义 RM_M3508_CURRENT_INV。 */
#define RM_GM6020_CURRENT_INV (1.0f / RM_GM6020_CURRENT_RATIO) /* 定义 RM_GM6020_CURRENT_INV。 */
#define RM_DEFAULT_POS_MIN (-10000.0f) /* 定义 RM_DEFAULT_POS_MIN。 */
#define RM_DEFAULT_POS_MAX (10000.0f) /* 定义 RM_DEFAULT_POS_MAX。 */
#define RM_DEFAULT_POS_TOLERANCE (3.0f) /* 定义 RM_DEFAULT_POS_TOLERANCE。 */

static inline void RM_Motor_CalculateCommon(MotorTypeDef *motor, float current_inv); /* 声明 RM_Motor_CalculateCommon 接口。 */
static void RM_M2006_InitInternal(MotorTypeDef *motor, uint8_t id); /* 声明 RM_M2006_InitInternal 接口。 */
static void RM_M2006_CalculateInternal(MotorTypeDef *motor); /* 声明 RM_M2006_CalculateInternal 接口。 */
static void RM_M3508_InitInternal(MotorTypeDef *motor, uint8_t id); /* 声明 RM_M3508_InitInternal 接口。 */
static void RM_M3508_CalculateInternal(MotorTypeDef *motor); /* 声明 RM_M3508_CalculateInternal 接口。 */
static void RM_GM6020_InitInternal(MotorTypeDef *motor, uint8_t id); /* 声明 RM_GM6020_InitInternal 接口。 */
static void RM_GM6020_CalculateInternal(MotorTypeDef *motor); /* 声明 RM_GM6020_CalculateInternal 接口。 */
static uint8_t RM_Motor_SendControlLocked(MotorTypeDef *motor); /* 声明 RM_Motor_SendControlLocked 接口。 */
static inline int16_t RM_Motor_ApplyOutputLimit(can_motor_cfg motor_cfg, MotorTypeDef *motor, float pid_output); /* 声明 RM_Motor_ApplyOutputLimit 接口。 */
static inline void RM_Motor_HandleAngleLoopDirectionChange(MotorTypeDef *motor, float target_angle, float current_angle); /* 声明 RM_Motor_HandleAngleLoopDirectionChange 接口。 */
static inline float RM_Motor_ClampFloat(float value, float min, float max); /* 声明 RM_Motor_ClampFloat 接口。 */
static inline bool RM_Motor_IsSameTxGroup(const MotorTypeDef *lhs, const MotorTypeDef *rhs); /* 声明 RM_Motor_IsSameTxGroup 接口。 */

// M2006虚拟类
const RM_MotorClass_t RM_M2006_Class = { /* 初始化 RM_M2006_Class。 */
    .name = "M2006", /* 配置 name。 */
    .model = RmM2006, /* 配置 model。 */
    .gear_ratio = RM_M2006_GEAR_RATIO, /* 配置 gear_ratio。 */
    .max_current = RM_M2006_MAX_CURRENT, /* 配置 max_current。 */
    .current_ratio = RM_M2006_CURRENT_RATIO, /* 配置 current_ratio。 */
    .tx_base_addr = g_RM_MOTOR_BIAS_ADDR_2006, /* 配置 tx_base_addr。 */
    .id_min = 1, /* 配置 id_min。 */
    .id_max = 4, /* 配置 id_max。 */
    .init = RM_M2006_InitInternal, /* 配置 init。 */
    .calculate = RM_M2006_CalculateInternal, /* 配置 calculate。 */
};

// M3508虚拟类
const RM_MotorClass_t RM_M3508_Class = { /* 初始化 RM_M3508_Class。 */
    .name = "M3508", /* 配置 name。 */
    .model = RmM3508, /* 配置 model。 */
    .gear_ratio = RM_M3508_GEAR_RATIO, /* 配置 gear_ratio。 */
    .max_current = RM_M3508_MAX_CURRENT, /* 配置 max_current。 */
    .current_ratio = RM_M3508_CURRENT_RATIO, /* 配置 current_ratio。 */
    .tx_base_addr = g_RM_MOTOR_BIAS_ADDR_3508, /* 配置 tx_base_addr。 */
    .id_min = 1, /* 配置 id_min。 */
    .id_max = 4, /* 配置 id_max。 */
    .init = RM_M3508_InitInternal, /* 配置 init。 */
    .calculate = RM_M3508_CalculateInternal, /* 配置 calculate。 */
};

// GM6020虚拟类
const RM_MotorClass_t RM_GM6020_Class = { /* 初始化 RM_GM6020_Class。 */
    .name = "GM6020", /* 配置 name。 */
    .model = RmGM6020, /* 配置 model。 */
    .gear_ratio = RM_GM6020_GEAR_RATIO, /* 配置 gear_ratio。 */
    .max_current = RM_GM6020_MAX_CURRENT, /* 配置 max_current。 */
    .current_ratio = RM_GM6020_CURRENT_RATIO, /* 配置 current_ratio。 */
    .tx_base_addr = g_RM_MOTOR_BIAS_ADDR_6020, /* 配置 tx_base_addr。 */
    .id_min = 1, /* 配置 id_min。 */
    .id_max = 7, /* 配置 id_max。 */
    .init = RM_GM6020_InitInternal, /* 配置 init。 */
    .calculate = RM_GM6020_CalculateInternal, /* 配置 calculate。 */
};

// 创建电机对应实体
void RM_Motor_Create(MotorTypeDef *motor, const RM_MotorClass_t *motor_class, uint8_t id) /* 实现 RM_Motor_Create。 */
{
    if (motor == NULL || motor_class == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    if (id < motor_class->id_min || id > motor_class->id_max) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    motor->motor_class.rm_motor_class = motor_class; /* 更新 rm_motor_class。 */
    if (motor_class->init != NULL) /* 检查当前执行条件。 */
    {
        motor_class->init(motor, id); /* 完成本行操作。 */
    }
}

// 电机解算回调函数API
void RM_Motor_Calculate(MotorTypeDef *motor) /* 实现 RM_Motor_Calculate。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    if (motor->motor_class.rm_motor_class != NULL && /* 检查当前执行条件。 */
        motor->motor_class.rm_motor_class->calculate != NULL) /* 继续更新 目标值。 */
    {
        motor->motor_class.rm_motor_class->calculate(motor); /* 完成本行操作。 */
    }
}

/// @brief 初始化RM电机(父类函数)
/// @param motor 电机对应结构体地址
/// @param id 电机ID
/// @param motor_class 对应RM电机虚拟类
static inline void RM_Motor_InitBase(MotorTypeDef *motor, uint8_t id, const RM_MotorClass_t *motor_class) /* 实现 RM_Motor_InitBase。 */
{
    memset(motor, 0, sizeof(MotorTypeDef)); /* 调用 memset。 */

    motor->motor_class.rm_motor_class = motor_class; /* 更新 rm_motor_class。 */
    motor->MotorID = id; /* 更新 MotorID。 */
    motor->MotorInf.band = RM_MOTOR_BAND; /* 更新 band。 */
    motor->MotorInf.model = motor_class->model; /* 更新 model。 */
    motor->params.gear_ratio = motor_class->gear_ratio; /* 更新 gear_ratio。 */
    motor->params.max_current = motor_class->max_current; /* 更新 max_current。 */
    motor->params.current_ratio = motor_class->current_ratio; /* 更新 current_ratio。 */

    // RM 电机默认配置：
    // position_min/max 用于位置环目标限位；
    // position_tolerance 用于位置到位死区；
    // direction_bias/reverse 用于安装方向修正。
    motor->config.direction_bias = 0.0f; /* 更新 direction_bias。 */
    motor->config.position_min = RM_DEFAULT_POS_MIN; /* 更新 position_min。 */
    motor->config.position_max = RM_DEFAULT_POS_MAX; /* 更新 position_max。 */
    motor->config.position_tolerance = RM_DEFAULT_POS_TOLERANCE; /* 更新 position_tolerance。 */
    motor->config.reverse = 0; /* 更新 reverse。 */

    // S 型规划器默认不注册，只有在电机注册阶段显式配置后才启用。
    memset(&motor->trap_config, 0, sizeof(motor->trap_config)); /* 调用 memset。 */

    motor->calculate = motor_class->calculate; /* 更新 calculate。 */
    motor->g_TxHeader.IDE = CAN_ID_STD; /* 更新 IDE。 */
    motor->g_TxHeader.RTR = CAN_RTR_DATA; /* 更新 RTR。 */
    motor->g_TxHeader.DLC = CtrlMotorLen; /* 更新 DLC。 */
}

/// @brief M2006电机实例化初始化函数
/// @param motor 电机对应结构体地址
/// @param id 电机ID
static void RM_M2006_InitInternal(MotorTypeDef *motor, uint8_t id) /* 实现 RM_M2006_InitInternal。 */
{
    RM_Motor_InitBase(motor, id, &RM_M2006_Class); /* 调用 RM_Motor_InitBase。 */
    motor->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_2006; /* 更新 StdId。 */
}

/// @brief M3508电机实例化初始化函数
/// @param motor 电机对应结构体地址
/// @param id 电机ID
static void RM_M3508_InitInternal(MotorTypeDef *motor, uint8_t id) /* 实现 RM_M3508_InitInternal。 */
{
    RM_Motor_InitBase(motor, id, &RM_M3508_Class); /* 调用 RM_Motor_InitBase。 */
    motor->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_3508; /* 更新 StdId。 */
}

/// @brief GM6020电机实例化初始化函数
/// @param motor 电机对应结构体地址
/// @param id 电机ID
static void RM_GM6020_InitInternal(MotorTypeDef *motor, uint8_t id) /* 实现 RM_GM6020_InitInternal。 */
{
    RM_Motor_InitBase(motor, id, &RM_GM6020_Class); /* 调用 RM_Motor_InitBase。 */
    motor->g_TxHeader.StdId = (id <= 4U) ? (g_RM_MOTOR_BIAS_ADDR_6020 - 0x100U) : g_RM_MOTOR_BIAS_ADDR_6020; /* 更新 StdId。 */
}

/// @brief 创建M2006电机实例
/// @param motor 电机对应结构体地址
/// @param id 电机ID
void RM_M2006_Create(MotorTypeDef *motor, uint8_t id) /* 实现 RM_M2006_Create。 */
{
    if (motor != NULL && id >= 1U && id <= 4U) /* 检查当前执行条件。 */
    {
        RM_Motor_Create(motor, &RM_M2006_Class, id); /* 调用 RM_Motor_Create。 */
    }
}

/// @brief 创建M3508电机实例
/// @param motor 电机对应结构体地址
/// @param id 电机ID
void RM_M3508_Create(MotorTypeDef *motor, uint8_t id) /* 实现 RM_M3508_Create。 */
{
    if (motor != NULL && id >= 1U && id <= 4U) /* 检查当前执行条件。 */
    {
        RM_Motor_Create(motor, &RM_M3508_Class, id); /* 调用 RM_Motor_Create。 */
    }
}

/// @brief 创建GM6020电机实例
/// @param motor 电机对应结构体地址
/// @param id 电机ID
void RM_GM6020_Create(MotorTypeDef *motor, uint8_t id) /* 实现 RM_GM6020_Create。 */
{
    if (motor != NULL && id >= 1U && id <= 7U) /* 检查当前执行条件。 */
    {
        RM_Motor_Create(motor, &RM_GM6020_Class, id); /* 调用 RM_Motor_Create。 */
    }
}

/// @brief 设置电机初始参数
/// @param motor 电机对应结构体地址
/// @param config 对应参数地址
void RM_Motor_SetConfig(MotorTypeDef *motor, const RM_MotorConfig_t *config) /* 实现 RM_Motor_SetConfig。 */
{
    if (motor == NULL || config == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
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
void RM_Motor_SetCascadePID(MotorTypeDef *motor, /* 传入下一项参数或数据。 */
                            float outer_p, float outer_i, float outer_d, float outer_f, /* 传入下一项参数或数据。 */
                            float inner_p, float inner_i, float inner_d, float inner_f, /* 传入下一项参数或数据。 */
                            float outer_max_out, float outer_min_out, float outer_max_iout, /* 传入下一项参数或数据。 */
                            float inner_max_out, float inner_min_out, float inner_max_iout) /* 继续当前语句。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    motor->use_cascade = 1U; /* 更新 use_cascade。 */
    CASCADE_PID_Init(&motor->cascade_pid, /* 传入下一项参数或数据。 */
                     outer_p, outer_i, outer_d, outer_f, /* 传入下一项参数或数据。 */
                     inner_p, inner_i, inner_d, inner_f, /* 传入下一项参数或数据。 */
                     outer_max_out, outer_min_out, outer_max_iout, /* 传入下一项参数或数据。 */
                     inner_max_out, inner_min_out, inner_max_iout); /* 完成本行操作。 */
    CASCADE_PID_Clear(&motor->cascade_pid); /* 调用 CASCADE_PID_Clear。 */
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
void RM_Motor_SetSpeedPID(MotorTypeDef *motor, PID_MODE_e mode, /* 传入下一项参数或数据。 */
                          float p, float i, float d, float f, /* 传入下一项参数或数据。 */
                          float max_out, float min_out, float max_iout) /* 继续当前语句。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    motor->use_cascade = 0U; /* 更新 use_cascade。 */
    PID_Init(&motor->inner_pid, mode, p, i, d, f, max_out, min_out, max_iout); /* 调用 PID_Init。 */
    PID_Clear(&motor->inner_pid); /* 调用 PID_Clear。 */
}

/// @brief 负责加锁和发送数据
/// @param motor 电机对应结构体地址
/// @return 发送成功与否
static uint8_t RM_Motor_SendControlLocked(MotorTypeDef *motor) /* 实现 RM_Motor_SendControlLocked。 */
{
    uint8_t *send_buffer = MotorManager.RM_MOTOR_DATA_ARRAY; /* 初始化 send_buffer。 */
    memset(send_buffer, 0x00, CtrlMotorLen); /* 调用 memset。 */

    for (uint8_t i = 0; i < MotorManager.registered_count; i++) /* 遍历当前数据集合。 */
    {
        MotorTypeDef *other = &MotorManager.MotorList[i]; /* 初始化 other。 */
        if (!RM_Motor_IsSameTxGroup(motor, other)) /* 检查当前执行条件。 */
        {
            continue; /* 跳过本轮剩余处理。 */
        }

        if (other->MotorID < 1U || other->MotorID > 4U) /* 检查当前执行条件。 */
        {
            continue; /* 跳过本轮剩余处理。 */
        }

        if (!Motor_IsOnline((can_motor_cfg)(i + 1U))) /* 离线槽位保持为零。 */
        {
            continue; /* 跳过本轮剩余处理。 */
        }

        uint8_t offset = (uint8_t)((other->MotorID - 1U) * 2U); /* 初始化 offset。 */
        send_buffer[offset] = other->SendMotorData[0]; /* 更新 send_buffer。 */
        send_buffer[offset + 1U] = other->SendMotorData[1]; /* 更新 send_buffer。 */
    }

    return Motor_GetHAL_Fast()->can_send(&motor->g_TxHeader, send_buffer); /* 返回当前计算结果。 */
}

/// @brief 设置发送数据
/// @param motor_cfg 电机别名
/// @param data 电机数据
static inline void RM_MotorSetTxData(can_motor_cfg motor_cfg, uint8_t *data) /* 实现 RM_MotorSetTxData。 */
{
    if (data == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1]; /* 初始化 motor。 */
    if (motor->MotorInf.band != RM_MOTOR_BAND) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    if (xSemaphoreTake(g_xRmBufferMutexHandle, pdMS_TO_TICKS(2)) != pdTRUE) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    memset(motor->SendMotorData, 0x00, CtrlMotorLen); /* 调用 memset。 */
    memcpy(motor->SendMotorData, data, CtrlMotorLen); /* 调用 memcpy。 */
    (void)RM_Motor_SendControlLocked(motor); /* 调用 RM_Motor_SendControlLocked。 */
    xSemaphoreGive(g_xRmBufferMutexHandle); /* 调用 xSemaphoreGive。 */
}

/// @brief 电机解算通用函数
/// @param motor 电机对应结构体地址
/// @param current_inv 电机对应的减速比
static inline void RM_Motor_CalculateCommon(MotorTypeDef *motor, float current_inv) /* 实现 RM_Motor_CalculateCommon。 */
{
    // 暂存数据
    MotorSolvedData_t *data = &motor->motor_data; /* 初始化 data。 */
    uint8_t *rx = motor->ReceiveMotorData; /* 初始化 rx。 */

    int16_t ecd = (int16_t)((((uint16_t)rx[0]) << 8) | rx[1]); /* 初始化 ecd。 */
    int16_t speed_rpm = (int16_t)((((uint16_t)rx[2]) << 8) | rx[3]); /* 初始化 speed_rpm。 */
    int16_t current_raw = (int16_t)((((uint16_t)rx[4]) << 8) | rx[5]); /* 初始化 current_raw。 */

    // 确认是否初始化完成
    if (data->init_flag == 0U) /* 检查当前执行条件。 */
    {
        data->last_ecd = ecd; /* 更新 last_ecd。 */
        data->offset_ecd = ecd; /* 更新 offset_ecd。 */
        data->offset_ecd_angle = ecd * ECD_TO_DEGREE; /* 更新 offset_ecd_angle。 */
        // offset_ecd_angle 要与 solved_data[3] 处在同一坐标系，否则
        // "solved_data[3] - offset_ecd_angle" 会跨坐标相减。
        if (motor->config.reverse != 0U) /* 检查当前执行条件。 */
        {
            data->offset_ecd_angle = -data->offset_ecd_angle; /* 更新 offset_ecd_angle。 */
        }
        data->init_flag = 1U; /* 更新 init_flag。 */
    }

    int16_t err = (int16_t)(ecd - data->last_ecd); /* 初始化 err。 */
    if (err > 4096) /* 检查当前执行条件。 */
    {
        data->total_round--; /* 完成本行操作。 */
        err -= 8192; /* 更新 err。 */
    }
    else if (err < -4096) /* 继续判断下一条件。 */
    {
        data->total_round++; /* 完成本行操作。 */
        err += 8192; /* 更新 err。 */
    }

    data->total_ecd += err; /* 更新 total_ecd。 */
    data->last_ecd = ecd; /* 更新 last_ecd。 */
    data->solved_data[0] = ecd * ECD_TO_DEGREE; /* 更新 solved_data。 */

    if (data->filter_init == 0U) /* 检查当前执行条件。 */
    {
        data->solved_data[1] = (float)speed_rpm; /* 更新 solved_data。 */
        data->filter_init = 1U; /* 更新 filter_init。 */
    }
    else /* 处理其余情况。 */
    {
        data->solved_data[1] = (float)speed_rpm * 0.96f + data->last_speed * 0.04f; /* 更新 solved_data。 */
    }

    data->solved_data[2] = current_raw * current_inv; /* 更新 solved_data。 */
    data->solved_data[3] = (float)data->total_ecd * ECD_TO_DEGREE; /* 更新 solved_data。 */
    data->solved_data[4] = (float)speed_rpm * 0.10472f; /* 更新 solved_data。 */
    data->last_speed = data->solved_data[1]; /* 更新 last_speed。 */

    // reverse=1 时，把上层所见的反馈（角度、速度、电流）统一翻到逻辑坐标，
    // 后续 PID/限位/死区都按逻辑坐标工作；物理侧的反号只在 RmMotorSendCfg
    // 把 PID 输出电流再翻回来一次。
    if (motor->config.reverse != 0U) /* 检查当前执行条件。 */
    {
        data->solved_data[0] = -data->solved_data[0]; /* 更新 solved_data。 */
        data->solved_data[1] = -data->solved_data[1]; /* 更新 solved_data。 */
        data->solved_data[2] = -data->solved_data[2]; /* 更新 solved_data。 */
        data->solved_data[3] = -data->solved_data[3]; /* 更新 solved_data。 */
        data->solved_data[4] = -data->solved_data[4]; /* 更新 solved_data。 */
    }

    if (motor->MotorInf.model == RmM3508) /* 检查当前执行条件。 */
    {
        data->solved_data[5] = (float)rx[6]; /* 更新 solved_data。 */
        data->solved_data[6] = (float)rx[7]; /* 更新 solved_data。 */
    }
}

static void RM_M2006_CalculateInternal(MotorTypeDef *motor) /* 实现 RM_M2006_CalculateInternal。 */
{
    if (motor != NULL) /* 检查当前执行条件。 */
    {
        RM_Motor_CalculateCommon(motor, RM_M2006_CURRENT_INV); /* 调用 RM_Motor_CalculateCommon。 */
    }
}

static void RM_M3508_CalculateInternal(MotorTypeDef *motor) /* 实现 RM_M3508_CalculateInternal。 */
{
    if (motor != NULL) /* 检查当前执行条件。 */
    {
        RM_Motor_CalculateCommon(motor, RM_M3508_CURRENT_INV); /* 调用 RM_Motor_CalculateCommon。 */
    }
}

static void RM_GM6020_CalculateInternal(MotorTypeDef *motor) /* 实现 RM_GM6020_CalculateInternal。 */
{
    if (motor != NULL) /* 检查当前执行条件。 */
    {
        RM_Motor_CalculateCommon(motor, RM_GM6020_CURRENT_INV); /* 调用 RM_Motor_CalculateCommon。 */
    }
}

void RM_MOTOR_CALCU(MotorTypeDef *motor) /* 实现 RM_MOTOR_CALCU。 */
{
    if (motor != NULL && motor->calculate != NULL) /* 检查当前执行条件。 */
    {
        motor->calculate(motor); /* 完成本行操作。 */
    }
}

void RM_Motor_Reset_Zero(MotorTypeDef *motor) /* 实现 RM_Motor_Reset_Zero。 */
{
    if (motor == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    motor->motor_data.total_round = 0; /* 更新 total_round。 */
    motor->motor_data.total_ecd = 0; /* 更新 total_ecd。 */
    motor->motor_data.offset_ecd = motor->motor_data.last_ecd; /* 更新 offset_ecd。 */
    motor->motor_data.target_angle = motor->motor_data.solved_data[3]; /* 更新 target_angle。 */
    motor->motor_data.last_target = motor->motor_data.target_angle; /* 更新 last_target。 */
    motor->motor_data.pre_last_target = motor->motor_data.target_angle; /* 更新 pre_last_target。 */
    motor->motor_data.target_init_flag = 0U; /* 更新 target_init_flag。 */
}

void RM_Motor_Reset_All(void) /* 实现 RM_Motor_Reset_All。 */
{
    for (uint8_t i = 0; i < g_RM_MOTOR_NUM; i++) /* 遍历当前数据集合。 */
    {
        memset(&MotorManager.MotorList[i].motor_data, 0, sizeof(MotorSolvedData_t)); /* 调用 memset。 */
    }
}

/// @brief RM电机发送API
/// @param motor_cfg 电机别名
/// @param TargetCurrent 电机目标电流值
void RmMotorSendCfg(can_motor_cfg motor_cfg, int16_t TargetCurrent) /* 实现 RmMotorSendCfg。 */
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1]; /* 初始化 motor。 */
    if (motor == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    if (!Motor_IsOnline(motor_cfg)) /* 离线时强制覆盖非零电流。 */
    {
        TargetCurrent = 0; /* 定义 TargetCurrent 枚举项。 */
    }

    /* 执行器电流硬限幅（安全网）：按型号 max_current 双向 clamp，
     * 防止上层 PID 输出溢出 CAN 电流量程导致未定义行为。 */
    {
        int16_t hard_limit = motor->max_current; /* 初始化 hard_limit。 */
        if (hard_limit > 0) /* 检查当前执行条件。 */
        {
            if (TargetCurrent > hard_limit) /* 检查当前执行条件。 */
            {
                TargetCurrent = hard_limit; /* 定义 TargetCurrent 枚举项。 */
            }
            else if (TargetCurrent < (int16_t)(-hard_limit)) /* 继续判断下一条件。 */
            {
                TargetCurrent = (int16_t)(-hard_limit); /* 定义 TargetCurrent 枚举项。 */
            }
        }
    }

    if (motor->config.reverse != 0U) /* 检查当前执行条件。 */
    {
        TargetCurrent = (int16_t)(-TargetCurrent); /* 定义 TargetCurrent 枚举项。 */
    }

    if (motor_cfg == RM_3508_STORE_LEFT) /* 检查当前执行条件。 */
    {
        g_RmDebugStoreLeftFinalCurrent = TargetCurrent; /* 更新 g_RmDebugStoreLeftFinalCurrent。 */
    }
    else if (motor_cfg == RM_3508_STORE_RIGHT) /* 继续判断下一条件。 */
    {
        g_RmDebugStoreRightFinalCurrent = TargetCurrent; /* 更新 g_RmDebugStoreRightFinalCurrent。 */
    }
    if (TargetCurrent < 0) /* 检查当前执行条件。 */
    {
        TargetCurrent = (uint16_t)(~(-TargetCurrent)); /* 定义 TargetCurrent 枚举项。 */
    }

    uint8_t data[8] = {0}; /* 初始化 data。 */
    data[0] = (uint8_t)((uint16_t)TargetCurrent >> 8); /* 更新 data。 */
    data[1] = (uint8_t)TargetCurrent; /* 更新 data。 */
    RM_MotorSetTxData(motor_cfg, data); /* 调用 RM_MotorSetTxData。 */
}

/// @brief 这是一个虚假的限幅函数(目前作用只是四舍五入最后的发送数据)
/// @param motor_cfg 电机别名
/// @param motor 电机对应结构体地址
/// @param pid_output 电机PID输出
/// @return 四舍五入之后的数值
static inline int16_t RM_Motor_ApplyOutputLimit(can_motor_cfg motor_cfg, MotorTypeDef *motor, float pid_output) /* 实现 RM_Motor_ApplyOutputLimit。 */
{
    bool should_block = false; /* 初始化 should_block。 */
    int16_t final_output = (int16_t)pid_output; /* 初始化 final_output。 */
    float output_for_limit = pid_output; /* 初始化 output_for_limit。 */

    if (motor == NULL) /* 检查当前执行条件。 */
    {
        return final_output; /* 返回当前计算结果。 */
    }

    if (motor_cfg == RM_3508_STORE_LEFT) /* 检查当前执行条件。 */
    {
        float current_pos_deg = motor->motor_data.solved_data[3]; /* 初始化 current_pos_deg。 */
        g_RmDebugStoreLeftPosDeg = current_pos_deg; /* 更新 g_RmDebugStoreLeftPosDeg。 */
        g_RmDebugStoreLeftPidOutput = pid_output; /* 更新 g_RmDebugStoreLeftPidOutput。 */
        should_block = ((current_pos_deg >= LimitStore) && (output_for_limit > 0.0f)) || /* 继续更新 should_block。 */
                       ((current_pos_deg <= LeftStoreTop) && (output_for_limit < 0.0f)); /* 更新 current_pos_deg。 */
        g_RmDebugStoreLeftLimitBlocked = (uint8_t)should_block; /* 更新 g_RmDebugStoreLeftLimitBlocked。 */
    }
    else if (motor_cfg == RM_3508_STORE_RIGHT) /* 继续判断下一条件。 */
    {
        float current_pos_deg = motor->motor_data.solved_data[3]; /* 初始化 current_pos_deg。 */
        g_RmDebugStoreRightPosDeg = current_pos_deg; /* 更新 g_RmDebugStoreRightPosDeg。 */
        g_RmDebugStoreRightPidOutput = pid_output; /* 更新 g_RmDebugStoreRightPidOutput。 */
        should_block = ((current_pos_deg >= LimitStore) && (output_for_limit > 0.0f)) || /* 继续更新 should_block。 */
                       ((current_pos_deg <= RightStoreTop) && (output_for_limit < 0.0f)); /* 更新 current_pos_deg。 */
        g_RmDebugStoreRightLimitBlocked = (uint8_t)should_block; /* 更新 g_RmDebugStoreRightLimitBlocked。 */
    }
    else if (motor_cfg == RM_3508_GRIPPER) /* 继续判断下一条件。 */
    {
        // GRIPPER 过流/堵转保护的持久状态。g_RmDebug* 全局量同时作为示波器观测用途。
        // 当前 RmMotorPID_Calc 只在单一控制任务中周期触发，static + 临界区即可。
        static uint32_t s_last_tick = 0U; /* 初始化 s_last_tick。 */
        static uint8_t s_tick_init = 0U; /* 初始化 s_tick_init。 */
        static uint8_t s_tripped = 0U; /* 初始化 s_tripped。 */

        uint32_t now = HAL_GetTick(); /* 初始化 now。 */
        float current_abs = fabsf(motor->motor_data.solved_data[2]); /* 初始化 current_abs。 */

        taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
        uint32_t dt_ms = 0U; /* 初始化 dt_ms。 */
        if (s_tick_init == 0U) /* 检查当前执行条件。 */
        {
            s_tick_init = 1U; /* 更新 s_tick_init。 */
        }
        else /* 处理其余情况。 */
        {
            dt_ms = now - s_last_tick; /* 更新 dt_ms。 */
        }
        s_last_tick = now; /* 更新 s_last_tick。 */

        g_RmDebugLoadCurrentFilt += LOAD3508_OVERCURRENT_FILTER_ALPHA * (current_abs - g_RmDebugLoadCurrentFilt); /* 更新 g_RmDebugLoadCurrentFilt。 */

        if (s_tripped == 0U) /* 检查当前执行条件。 */
        {
            if (g_RmDebugLoadCurrentFilt >= LOAD3508_STALL_OVERCURRENT_LIMIT_A) /* 检查当前执行条件。 */
            {
                g_RmDebugLoadOverCurMs += dt_ms; /* 更新 g_RmDebugLoadOverCurMs。 */
                if (g_RmDebugLoadOverCurMs >= LOAD3508_STALL_OVERCURRENT_RETURN_MS) /* 检查当前执行条件。 */
                {
                    s_tripped = 1U; /* 更新 s_tripped。 */
                    g_RmDebugLoadClearMs = 0U; /* 更新 g_RmDebugLoadClearMs。 */
                }
            }
            else /* 处理其余情况。 */
            {
                g_RmDebugLoadOverCurMs = 0U; /* 更新 g_RmDebugLoadOverCurMs。 */
            }
        }
        else /* 处理其余情况。 */
        {
            if (g_RmDebugLoadCurrentFilt <= LOAD3508_STALL_OVERCURRENT_CLEAR_A) /* 检查当前执行条件。 */
            {
                g_RmDebugLoadClearMs += dt_ms; /* 更新 g_RmDebugLoadClearMs。 */
                if (g_RmDebugLoadClearMs >= LOAD3508_STALL_OVERCURRENT_RETURN_MS) /* 检查当前执行条件。 */
                {
                    s_tripped = 0U; /* 更新 s_tripped。 */
                    g_RmDebugLoadOverCurMs = 0U; /* 更新 g_RmDebugLoadOverCurMs。 */
                }
            }
            else /* 处理其余情况。 */
            {
                g_RmDebugLoadClearMs = 0U; /* 更新 g_RmDebugLoadClearMs。 */
            }
        }

        g_RmDebugLoadStallTripped = s_tripped; /* 更新 g_RmDebugLoadStallTripped。 */
        uint8_t tripped_snapshot = s_tripped; /* 初始化 tripped_snapshot。 */
        taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

        if (tripped_snapshot != 0U) /* 检查当前执行条件。 */
        {
            should_block = true; /* 更新 should_block。 */
        }
    }

    if (should_block) /* 检查当前执行条件。 */
    {
        if (motor->use_cascade != 0U) /* 检查当前执行条件。 */
        {
            PID_Clear_Integral(&motor->cascade_pid.outer); /* 调用 PID_Clear_Integral。 */
            PID_Clear_Integral(&motor->cascade_pid.inner); /* 调用 PID_Clear_Integral。 */
        }
        else /* 处理其余情况。 */
        {
            PID_Clear_Integral(&motor->inner_pid); /* 调用 PID_Clear_Integral。 */
        }
        final_output = 0; /* 更新 final_output。 */
    }

    return final_output; /* 返回当前计算结果。 */
}

/// @brief 处理角度环方向问题
/// @param motor 电机对应结构体
/// @param target_angle 电机目标角度
/// @param current_angle 电机当前角度
/// @note  主要用于处理换向时候的积分问题
static inline void RM_Motor_HandleAngleLoopDirectionChange(MotorTypeDef *motor, float target_angle, float current_angle) /* 实现 RM_Motor_HandleAngleLoopDirectionChange。 */
{
#if RM_3508_CLEAR_ANGLE_I_ON_DIR_CHANGE /* 按 RM_3508_CLEAR_ANGLE_I_ON_DIR_CHANGE 选择编译分支。 */
    if (motor == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    PID_t *outer_pid = &motor->cascade_pid.outer; /* 初始化 outer_pid。 */
    if (!outer_pid->initialized || outer_pid->calc_count == 0U) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    float angle_error = target_angle - current_angle; /* 初始化 angle_error。 */
    float current_target_delta = target_angle - outer_pid->target; /* 初始化 current_target_delta。 */
    bool target_direction_reversed = (current_target_delta * outer_pid->feedforward) < 0.0f; /* 初始化 target_direction_reversed。 */
    bool error_direction_reversed = (angle_error * outer_pid->last_error) < 0.0f; /* 初始化 error_direction_reversed。 */
    if (target_direction_reversed || error_direction_reversed) /* 检查当前执行条件。 */
    {
        PID_Clear_Integral(outer_pid); /* 调用 PID_Clear_Integral。 */
    }
#else /* 切换到备用编译分支。 */
    (void)motor; /* 显式忽略参数 motor。 */
    (void)target_angle; /* 显式忽略参数 target_angle。 */
    (void)current_angle; /* 显式忽略参数 current_angle。 */
#endif /* 结束条件编译。 */
}

/// @brief 上下限限幅
/// @param value 当前需要限幅的数值
/// @param min 下限
/// @param max 上限
/// @return 限幅后的数值
static inline float RM_Motor_ClampFloat(float value, float min, float max) /* 实现 RM_Motor_ClampFloat。 */
{
    if (value < min) /* 检查当前执行条件。 */
    {
        return min; /* 返回当前计算结果。 */
    }
    if (value > max) /* 检查当前执行条件。 */
    {
        return max; /* 返回当前计算结果。 */
    }
    return value; /* 返回当前计算结果。 */
}

/// @brief PID计算函数
/// @param motor_cfg 电机对应别名
/// @param target 电机目标值
/// @note 电机现在PID目标减小误差(以°为单位)
void RmMotorPID_Calc(can_motor_cfg motor_cfg, float target) /* 实现 RmMotorPID_Calc。 */
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1]; /* 初始化 motor。 */
    float current_pos_deg; /* 保存 current_pos_deg。 */
    if (motor == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    // 统一使用“相对零点角度”做位置环计算，
    // 这样目标值、限位、死区都在同一套坐标系下。
    if ((motor_cfg == RM_3508_STORE_LEFT) || (motor_cfg == RM_3508_STORE_RIGHT)) /* 检查当前执行条件。 */
    {
        current_pos_deg = motor->motor_data.solved_data[3]; /* 更新 current_pos_deg。 */
    }
    else /* 处理其余情况。 */
    {
        current_pos_deg = motor->motor_data.solved_data[3] - motor->motor_data.offset_ecd_angle; /* 更新 current_pos_deg。 */
    }

    if (motor->MotorInf.model == RmM3508) /* 检查当前执行条件。 */
    {
        // 先按注册的上下限约束目标位置，避免继续往机械极限方向推。
        float min_limit = motor->config.position_min; /* 初始化 min_limit。 */
        float max_limit = motor->config.position_max; /* 初始化 max_limit。 */
        if (min_limit > max_limit) /* 检查当前执行条件。 */
        {
            float tmp = min_limit; /* 初始化 tmp。 */
            min_limit = max_limit; /* 更新 min_limit。 */
            max_limit = tmp; /* 更新 max_limit。 */
        }

        target = RM_Motor_ClampFloat(target, min_limit, max_limit); /* 更新 target。 */

        // position_tolerance 作为位置环到位死区：
        // 误差足够小时直接清积分并停输出，避免在小角度附近反复抖动。
        float tolerance = motor->config.position_tolerance; /* 初始化 tolerance。 */
        if (tolerance <= 0.0f) /* 检查当前执行条件。 */
        {
            tolerance = RM_DEFAULT_POS_TOLERANCE; /* 更新 tolerance。 */
        }

        if (fabsf(target - current_pos_deg) <= tolerance) /* 检查当前执行条件。 */
        {
            if (motor->use_cascade != 0U) /* 检查当前执行条件。 */
            {
                PID_Clear_Integral(&motor->cascade_pid.outer); /* 调用 PID_Clear_Integral。 */
                PID_Clear_Integral(&motor->cascade_pid.inner); /* 调用 PID_Clear_Integral。 */
            }
            else /* 处理其余情况。 */
            {
                PID_Clear_Integral(&motor->inner_pid); /* 调用 PID_Clear_Integral。 */
            }
            RmMotorSendCfg(motor_cfg, 0); /* 调用 RmMotorSendCfg。 */
            return; /* 结束当前函数。 */
        }
    }

    RM_Motor_HandleAngleLoopDirectionChange(motor, target, current_pos_deg); /* 调用 RM_Motor_HandleAngleLoopDirectionChange。 */
    MotorSolvedData_t *data = &motor->motor_data; /* 初始化 data。 */
    if ((motor_cfg == RM_3508_STORE_LEFT) || (motor_cfg == RM_3508_STORE_RIGHT)) /* 检查当前执行条件。 */
    {
        current_pos_deg = data->solved_data[3]; /* 更新 current_pos_deg。 */
    }
    else /* 处理其余情况。 */
    {
        current_pos_deg = data->solved_data[3] - data->offset_ecd_angle; // update
    }
    if (motor->use_cascade != 0U) /* 检查当前执行条件。 */
    {
        s_rm_motor_pid_output = CASCADE_PID_Calculate(&motor->cascade_pid, target, current_pos_deg, data->solved_data[4]); /* 更新 s_rm_motor_pid_output。 */
    }
    else /* 处理其余情况。 */
    {
        s_rm_motor_pid_output = PID_Calculate(&motor->inner_pid, target, data->solved_data[4]); /* 更新 s_rm_motor_pid_output。 */
    }

    RmMotorSendCfg(motor_cfg, RM_Motor_ApplyOutputLimit(motor_cfg, motor, s_rm_motor_pid_output)); /* 调用 RmMotorSendCfg。 */
}

/// @brief 纯控制器：由参考位置算出电流原始输出，无任何副作用
/// @param cfg 电机别名
/// @param ref_pos_deg 参考位置(相对零点，°)
/// @return 串级/单环 PID 电流原始输出（不限幅、不发送、不判到位）
/// @note  angle_motor 后端 kAngleBackendRmCascade.compute 指向本函数。
///        到位归零/方向限位/硬限幅/过流全部交给 angle_motor 的判断器+限幅器，
///        这里只做纯粹的控制律计算。目标已由上层规划器给定，不再做 clamp。
float Rm_ComputeCascade(can_motor_cfg cfg, float ref_pos_deg) /* 实现 Rm_ComputeCascade。 */
{
    MotorTypeDef *motor = &MotorManager.MotorList[cfg - 1]; /* 初始化 motor。 */
    MotorSolvedData_t *data; /* 保存 data。 */
    float current_pos_deg; /* 保存 current_pos_deg。 */

    if (motor == NULL) /* 检查当前执行条件。 */
    {
        return 0.0f; /* 返回当前计算结果。 */
    }
    data = &motor->motor_data; /* 更新 data。 */

    /* 统一使用“相对零点角度”做位置环计算。储能位直接用累计角，
     * 其他电机减去零点偏移，与 RmMotorPID_Calc 保持同一坐标系。 */
    if ((cfg == RM_3508_STORE_LEFT) || (cfg == RM_3508_STORE_RIGHT)) /* 检查当前执行条件。 */
    {
        current_pos_deg = data->solved_data[3]; /* 更新 current_pos_deg。 */
    }
    else /* 处理其余情况。 */
    {
        current_pos_deg = data->solved_data[3] - data->offset_ecd_angle; /* 更新 current_pos_deg。 */
    }

    RM_Motor_HandleAngleLoopDirectionChange(motor, ref_pos_deg, current_pos_deg); /* 调用 RM_Motor_HandleAngleLoopDirectionChange。 */

    if (motor->use_cascade != 0U) /* 检查当前执行条件。 */
    {
        return CASCADE_PID_Calculate(&motor->cascade_pid, ref_pos_deg, /* 传入下一项参数或数据。 */
                                     current_pos_deg, data->solved_data[4]); /* 完成本行操作。 */
    }
    return PID_Calculate(&motor->inner_pid, ref_pos_deg, data->solved_data[4]); /* 返回当前计算结果。 */
}

/// @brief 速度环PID调试
/// @param motor_cfg 电机对应别名
/// @param target_speed_rpm  目标转速
void RmMotorSpeedPID_Calc(can_motor_cfg motor_cfg, float target_speed_rpm) /* 实现 RmMotorSpeedPID_Calc。 */
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1]; /* 初始化 motor。 */
    if (motor == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }
    g_testVariable = motor->motor_data.solved_data[4]; /* 更新 g_testVariable。 */
    s_rm_motor_pid_output = PID_Calculate(&motor->inner_pid, target_speed_rpm, g_testVariable); // 这里调节速度环,但是得先看目标速度如何
    RmMotorSendCfg(motor_cfg, RM_Motor_ApplyOutputLimit(motor_cfg, motor, s_rm_motor_pid_output)); /* 调用 RmMotorSendCfg。 */
}

/*============================== 双侧蓄力 3508 位置同步 PID ==============================*/

void RM_Motor_InitStoreSyncPid(float kp, float ki, float kd, float kf, /* 传入下一项参数或数据。 */
                               float max_out, float min_out, float max_iout) /* 继续当前语句。 */
{
    PID_Init(&s_rm_store_sync_pid, PID_POSITION, kp, ki, kd, kf, max_out, min_out, max_iout); /* 调用 PID_Init。 */
    PID_Clear(&s_rm_store_sync_pid); /* 调用 PID_Clear。 */
    g_RmStoreSyncErrorDeg = 0.0f; /* 更新 g_RmStoreSyncErrorDeg。 */
    g_RmStoreSyncPidOutputDeg = 0.0f; /* 更新 g_RmStoreSyncPidOutputDeg。 */
    s_rm_store_sync_pid_inited = 1U; /* 更新 s_rm_store_sync_pid_inited。 */
}

float RM_Motor_UpdateStoreSync(float left_pos_deg, float right_pos_deg) /* 实现 RM_Motor_UpdateStoreSync。 */
{
    if (s_rm_store_sync_pid_inited == 0U) /* 检查当前执行条件。 */
    {
        g_RmStoreSyncErrorDeg = 0.0f; /* 更新 g_RmStoreSyncErrorDeg。 */
        g_RmStoreSyncPidOutputDeg = 0.0f; /* 更新 g_RmStoreSyncPidOutputDeg。 */
        return 0.0f; /* 返回当前计算结果。 */
    }

    g_RmStoreSyncErrorDeg = left_pos_deg - right_pos_deg; /* 更新 g_RmStoreSyncErrorDeg。 */
    g_RmStoreSyncPidOutputDeg = PID_Calculate(&s_rm_store_sync_pid, 0.0f, g_RmStoreSyncErrorDeg); /* 更新 g_RmStoreSyncPidOutputDeg。 */
    return g_RmStoreSyncPidOutputDeg; /* 返回当前计算结果。 */
}

/// @brief 检查打包帧
/// @param lhs
/// @param rhs
/// @return
static inline bool RM_Motor_IsSameTxGroup(const MotorTypeDef *lhs, const MotorTypeDef *rhs) /* 实现 RM_Motor_IsSameTxGroup。 */
{
    if (lhs == NULL || rhs == NULL) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }

    return lhs->MotorInf.band == RM_MOTOR_BAND && /* 继续组合表达式。 */
           rhs->MotorInf.band == RM_MOTOR_BAND && /* 继续组合表达式。 */
           lhs->model == rhs->model && /* 继续组合表达式。 */
           lhs->g_TxHeader.StdId == rhs->g_TxHeader.StdId; /* 更新 StdId。 */
}
