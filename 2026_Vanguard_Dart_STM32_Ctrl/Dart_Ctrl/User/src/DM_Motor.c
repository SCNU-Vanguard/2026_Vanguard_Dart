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
#include "bsp_dwt.h"
#include <stdbool.h>

// 直接访问电机管理器（减少函数调用开销）
extern MotorManager_t MotorManager;

// 注意：不再直接调用硬件函数，改用 Motor_GetHAL() 接口

#define CtrlMotorLen 8
#define DM_SPEED_FILTER_COEF 0.0f // 达妙电机本身很准确

// DM电机控制帧
static const uint8_t DM_MOTOR_ENABLE[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
static const uint8_t DM_MOTOR_DISABLE[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};

// DM电机使能状态数组
static bool DM_ENABLE_ARR[g_DM_MOTOR_NUM] = {false};

// DM电机配置存储（非static，供头文件内联函数使用）
DM_MotorConfig_t g_DM_Configs[4] = {0};

/*============================== 静态函数声明（私有方法） ==============================*/
// 达妙电机通用函数
static void DM_Motor_RefreshData(can_motor_cfg motor_cfg);

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
    .pos_max = 12.5f,
    .pos_min = -12.5f,
    .vel_max = 200.0f,
    .vel_min = -200.0f,
    .torque_max = 10.0f,
    .torque_min = -10.0f,
    // 默认控制参数
    .default_kp = 0.0f,
    .default_kd = 0.0f,
    .default_torque_ff = 0.2f,
    // 虚函数表
    .init = DM_J3519_InitInternal,
    .calculate = DM_J3519_CalculateInternal,
    .send_control = DM_Motor_SendControlInternal,
    .refresh_data = DM_Motor_RefreshData,
    .WorkMode = DM_LOCATION_SPEED,
};

/// @brief J4310电机类
const DM_MotorClass_t DM_J4310_Class = {
    .name = "J4310",
    .model = DmJ4310,
    // 限幅参数（根据J4310-2EC手册）
    .kp_max = 500.0f,
    .kp_min = 0.0f,
    .kd_max = 5.0f,
    .kd_min = 0.0f,
    .pos_max = 12.5f,
    .pos_min = -12.5f,
    .vel_max = 30.0f,  // 修正：J4310 MIT模式速度范围为 ±30 rad/s
    .vel_min = -30.0f, // 修正：符合手册规格
    .torque_max = 10.0f,
    .torque_min = -10.0f,
    // 默认控制参数
    .default_kp = 0.0f,
    .default_kd = 0.0f,
    .default_torque_ff = 0.2f,
    // 虚函数表
    .init = DM_J4310_InitInternal,
    .calculate = DM_J4310_CalculateInternal,
    .send_control = DM_Motor_SendControlInternal,
    .refresh_data = DM_Motor_RefreshData,
    .WorkMode = DM_MIT,
};

/*============================== 数据转换函数 ==============================*/

static inline float uint_to_float_generic(int16_t x_int, float x_min, float x_max, int8_t bits)
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

/// @brief 调用电机的发送控制函数
uint8_t DM_Motor_SendControl(MotorTypeDef *motor)
{
    if (motor == NULL)
        return 0;

    // 优先使用类的虚函数表
    if (motor->motor_class.dm_motor_class != NULL && motor->motor_class.dm_motor_class->send_control != NULL)
    {
        return motor->motor_class.dm_motor_class->send_control(motor);
    }

    // 兼容旧接口
    if (motor->SendMotorControl != NULL)
    {
        return motor->SendMotorControl(motor);
    }

    return 0;
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

    // CAN报文头
    motor->g_TxHeader.StdId = DM_MIT_TX_BIAS + id;
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

static uint8_t DM_MotorEnable(can_motor_cfg motor_cfg)
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    if (motor == NULL)
        return 0;

    const MotorHAL_t *hal = Motor_GetHAL();
    if (hal->can_send(&(motor->g_TxHeader), (uint8_t *)DM_MOTOR_ENABLE))
    {
        if (motor->MotorID > 0 && motor->MotorID <= g_DM_MOTOR_NUM)
        {
            DM_ENABLE_ARR[motor->MotorID - 1] = true;
        }
        hal->delay_ms(1);
        return 1;
    }
    return 0;
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

static uint8_t DM_Motor_SendControlInternal(MotorTypeDef *st)
{
    if (st == NULL)
        return 0;

    // 自动使能（合并条件判断）
    if (st->MotorID > 0 && st->MotorID <= 4 && !DM_ENABLE_ARR[st->MotorID - 1])
    {
        DM_Motor_Enable(st);
    }

    // 使用Fast版本（内联，零开销）
    return Motor_GetHAL_Fast()->can_send(&st->g_TxHeader, st->SendMotorData) ? 1 : 0;
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

    // 获取配置（使用Fast版本，内联无开销）
    DM_MotorConfig_t *cfg = DM_Motor_GetConfigFast(motor->MotorID);

    MotorSolvedData_t *pData = &motor->motor_data;
    uint8_t *rx = motor->ReceiveMotorData;

    // 解析原始数据
    uint16_t pos_raw = ((uint16_t)rx[1] << 8) | rx[2];
    uint16_t vel_raw = ((uint16_t)rx[3] << 4) | (rx[4] >> 4);
    uint16_t tor_raw = ((rx[4] & 0x0F) << 8) | rx[5];

    // J3519: 使用MIT协议范围转换
    pData->solved_data[0] = uint_to_float_generic(pos_raw, cfg->pos_min, cfg->pos_max, 16);

    // 速度滤波
    float vel_new = uint_to_float_generic(vel_raw, cfg->vel_min, cfg->vel_max, 12);
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
}

/// @brief J4310电机解算（内部版本，优化：使用Fast版本获取配置）
static void DM_J4310_CalculateInternal(MotorTypeDef *motor)
{
    if (motor == NULL)
        return;

    const DM_MotorClass_t *cls = motor->motor_class.dm_motor_class;
    if (cls == NULL)
        cls = &DM_J4310_Class;

    // 获取配置（使用Fast版本，内联无开销）
    DM_MotorConfig_t *cfg = DM_Motor_GetConfigFast(motor->MotorID);

    MotorSolvedData_t *pData = &motor->motor_data;
    uint8_t *rx = motor->ReceiveMotorData;

    // 解析原始数据
    uint16_t pos_raw = ((uint16_t)rx[1] << 8) | rx[2];
    uint16_t vel_raw = ((uint16_t)rx[3] << 4) | (rx[4] >> 4);
    uint16_t tor_raw = ((rx[4] & 0x0F) << 8) | rx[5];

    // J4310: 使用MIT协议范围转换
    pData->solved_data[0] = uint_to_float_generic(pos_raw, cfg->pos_min, cfg->pos_max, 16);

    // 速度滤波
    float vel_new = uint_to_float_generic(vel_raw, cfg->vel_min, cfg->vel_max, 12);
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

// 通用电机使用，可以是J4310也可以是J3519电机
void DmMotorSendCfg(can_motor_cfg motor_cfg, float TargetPos, float TargetVel, DM_WorkMode workmode)
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

    static uint8_t data[8] = {0x00};
    if (workmode == DM_MIT)
    {
        // 使用用户配置的参数进行转换（修复：使用cfg而非cls->default）
        uint16_t KP_RESULT = float_to_uint_config(cfg->kp, DM_KP, DM_MIT, cfg, cls);
        uint16_t KD_RESULT = float_to_uint_config(cfg->kd, DM_KD, DM_MIT, cfg, cls);
        uint16_t Torque_ff = float_to_uint_config(cfg->torque_ff, DM_TORQUE, DM_MIT, cfg, cls);
        uint16_t Pos_des = float_to_uint_config(TargetPos, DM_POS, DM_MIT, cfg, cls);
        uint16_t Vel_des = float_to_uint_config(TargetVel, DM_VEL, DM_MIT, cfg, cls);

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
    // 达妙电机无需PID（无功率控制情况下）,如果是有功率控制情况下,需要结合本身的系数将力矩转换成电流

    // DM_Motor_SetTorqueFF(); // 最后是对力矩进行设置
}
