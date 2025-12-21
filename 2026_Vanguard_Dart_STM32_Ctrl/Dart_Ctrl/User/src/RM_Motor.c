/*****************************************************
 * RM电机（大疆电机）控制模块
 * 适配M3508、M2006、GM6020等系列电机
 *
 * note:M3508 -> 19.2减速比
 *      M2006 -> 36.0减速比
 *      GM6020 -> 1.0减速比
 * --------------------------------------------------
 * RM电机说明：
 * 大疆电机的注册同一系列电调最多是4个
 * 同一CAN总线上挂载的大疆电调（只有一个系列电调最多是8个）
 * 如果是C610和C620都有，则二者均为4个即为最大值
 * --------------------------------------------------
 * 面向对象设计说明：
 * 使用 RM_MotorClass_t 作为电机"类"，包含：
 * - 电机默认参数（减速比、最大电流、电流转换系数）
 * - 虚函数表（初始化、解算、发送控制、PID计算等）
 * 预定义 RM_M2006_Class、RM_M3508_Class、RM_GM6020_Class 三个类实例
 ****************************************************/

#include "RM_Motor.h"
#include "CanMotor.h"
#include <stdbool.h>
#include "usart.h"
#include <stdio.h>

// 直接访问电机管理器（减少函数调用开销）
extern MotorManager_t MotorManager;

float output = 0.0f;

#define CtrlMotorLen 8     // 电机控制报文长度默认给8
#define RM_MOTOR_MAX_NUM 2 // 最大电机数量

/*============================== 预计算常量（优化除法为乘法） ==============================*/
#define ECD_TO_DEGREE (360.0f / 8192.0f)                       // 编码器转角度: 0.0439453125f
#define RM_M2006_CURRENT_INV (1.0f / RM_M2006_CURRENT_RATIO)   // M2006电流倒数
#define RM_M3508_CURRENT_INV (1.0f / RM_M3508_CURRENT_RATIO)   // M3508电流倒数
#define RM_GM6020_CURRENT_INV (1.0f / RM_GM6020_CURRENT_RATIO) // GM6020电流倒数

/*============================== 静态函数声明（私有方法） ==============================*/

// 通用解算函数（内部）- 参数为电流系数倒数（优化版本）
static void RM_Motor_CalculateCommon(MotorTypeDef *motor, float current_inv);

// M2006专用函数
static void RM_M2006_InitInternal(MotorTypeDef *motor, uint8_t id);
static void RM_M2006_CalculateInternal(MotorTypeDef *motor);

// M3508专用函数
static void RM_M3508_InitInternal(MotorTypeDef *motor, uint8_t id);
static void RM_M3508_CalculateInternal(MotorTypeDef *motor);

// GM6020专用函数
static void RM_GM6020_InitInternal(MotorTypeDef *motor, uint8_t id);
static void RM_GM6020_CalculateInternal(MotorTypeDef *motor);

// 通用发送控制和PID计算
static uint8_t RM_Motor_SendControlInternal(MotorTypeDef *motor);
static float RM_Motor_PIDCalcInternal(MotorTypeDef *motor, float target);

/*============================== 电机类静态实例定义 ==============================*/

/// @brief M2006电机类（C610电调）
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
    .send_control = RM_Motor_SendControlInternal,
    .pid_calc = RM_Motor_PIDCalcInternal,
};

/// @brief M3508电机类（C620电调）
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
    .send_control = RM_Motor_SendControlInternal,
    .pid_calc = RM_Motor_PIDCalcInternal,
};

/// @brief GM6020电机类
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
    .send_control = RM_Motor_SendControlInternal,
    .pid_calc = RM_Motor_PIDCalcInternal,
};

/*============================== 面向对象接口实现 ==============================*/

/// @brief 使用指定的电机类创建电机实例
void RM_Motor_Create(MotorTypeDef *motor, const RM_MotorClass_t *motor_class, uint8_t id)
{
    if (motor == NULL || motor_class == NULL)
        return;

    if (id < motor_class->id_min || id > motor_class->id_max)
        return;

    // 关联电机类
    motor->motor_class.rm_motor_class = motor_class;

    // 调用类的初始化函数
    if (motor_class->init != NULL)
    {
        motor_class->init(motor, id);
    }
}

/// @brief 调用电机的解算函数
void RM_Motor_Calculate(MotorTypeDef *motor)
{
    if (motor == NULL)
        return;

    // 优先使用类的虚函数表
    if (motor->motor_class.rm_motor_class != NULL && motor->motor_class.rm_motor_class->calculate != NULL)
    {
        motor->motor_class.rm_motor_class->calculate(motor);
        return;
    }

    // 兼容旧接口：使用直接绑定的函数指针
    if (motor->calculate != NULL)
    {
        motor->calculate(motor);
    }
}

/// @brief 调用电机的发送控制函数
uint8_t RM_Motor_SendControl(MotorTypeDef *motor)
{
    if (motor == NULL)
        return 0;

    // 优先使用类的虚函数表
    if (motor->motor_class.rm_motor_class != NULL && motor->motor_class.rm_motor_class->send_control != NULL)
    {
        return motor->motor_class.rm_motor_class->send_control(motor);
    }

    // 兼容旧接口
    if (motor->SendMotorControl != NULL)
    {
        return motor->SendMotorControl(motor);
    }

    return 0;
}

/// @brief 调用电机的PID计算函数
float RM_Motor_PIDCalc(MotorTypeDef *motor, float target)
{
    if (motor == NULL)
        return 0.0f;

    // 优先使用类的虚函数表
    if (motor->motor_class.rm_motor_class != NULL && motor->motor_class.rm_motor_class->pid_calc != NULL)
    {
        return motor->motor_class.rm_motor_class->pid_calc(motor, target);
    }

    // 默认实现
    return RM_Motor_PIDCalcInternal(motor, target);
}

/*============================== 电机初始化内部函数 ==============================*/

/// @brief 初始化RM电机基础属性（内部函数）
static void RM_Motor_InitBase(MotorTypeDef *motor, uint8_t id,
                              const RM_MotorClass_t *motor_class)
{
    memset(motor, 0, sizeof(MotorTypeDef));

    // 关联电机类（推荐通过motor_class访问虚函数）
    motor->motor_class.rm_motor_class = motor_class;

    // 基本属性
    motor->MotorID = id;
    motor->MotorInf.band = RM_MOTOR_BAND;
    motor->MotorInf.model = motor_class->model;

    // 从类中复制参数
    motor->params.gear_ratio = motor_class->gear_ratio;
    motor->params.max_current = motor_class->max_current;
    motor->params.current_ratio = motor_class->current_ratio;

    // 默认配置
    motor->config.direction_bias = 0.0f;
    motor->config.position_tolerance = 50.0f;
    motor->config.reverse = 0;

    // 绑定函数指针（已弃用，保留用于向后兼容，新代码请使用motor_class虚函数表）
    motor->SendMotorControl = RM_Motor_SendControlInternal;
    motor->calculate = motor_class->calculate;

    // CAN报文头
    motor->g_TxHeader.IDE = CAN_ID_STD;
    motor->g_TxHeader.RTR = CAN_RTR_DATA;
    motor->g_TxHeader.DLC = CtrlMotorLen;
}

static void RM_M2006_InitInternal(MotorTypeDef *motor, uint8_t id)
{
    RM_Motor_InitBase(motor, id, &RM_M2006_Class);
    motor->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_2006;
}

static void RM_M3508_InitInternal(MotorTypeDef *motor, uint8_t id)
{
    RM_Motor_InitBase(motor, id, &RM_M3508_Class);
    motor->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_3508;
}

static void RM_GM6020_InitInternal(MotorTypeDef *motor, uint8_t id)
{
    RM_Motor_InitBase(motor, id, &RM_GM6020_Class);

    // GM6020的ID 5-7使用不同地址
    if (id <= 4)
    {
        motor->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_6020 - 0x100; // 0x1FE
    }
    else
    {
        motor->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_6020; // 0x2FE
    }
}

/*============================== 兼容旧接口的初始化函数 ==============================*/

void RM_M2006_Init(MotorTypeDef *motor, uint8_t id)
{
    if (motor == NULL || id < 1 || id > 4)
        return;
    RM_Motor_Create(motor, &RM_M2006_Class, id);
}

void RM_M3508_Init(MotorTypeDef *motor, uint8_t id)
{
    if (motor == NULL || id < 1 || id > 4)
        return;
    RM_Motor_Create(motor, &RM_M3508_Class, id);
}

void RM_GM6020_Init(MotorTypeDef *motor, uint8_t id)
{
    if (motor == NULL || id < 1 || id > 7)
        return;
    RM_Motor_Create(motor, &RM_GM6020_Class, id);
}

/*============================== 电机配置函数 ==============================*/

void RM_Motor_SetConfig(MotorTypeDef *motor, const RM_MotorConfig_t *config)
{
    if (motor == NULL || config == NULL)
        return;

    motor->config.direction_bias = config->direction_bias;
    motor->config.position_tolerance = config->position_tolerance;
    motor->config.reverse = config->reverse;
}

void RM_Motor_SetCascadePID(MotorTypeDef *motor,
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

void RM_Motor_SetSpeedPID(MotorTypeDef *motor, PID_MODE_e mode,
                          float p, float i, float d, float f,
                          float max_out, float min_out, float max_iout)
{
    if (motor == NULL)
        return;

    motor->use_cascade = 0;
    PID_Init(&motor->inner_pid, mode, p, i, d, f, max_out, min_out, max_iout);
    PID_Clear(&motor->inner_pid);
}

/*============================== 发送电机数据专用函数 ==============================*/

static uint8_t RM_Motor_SendControlInternal(MotorTypeDef *st)
{
    if (st == NULL)
        return 0;

    // 获取发送缓冲区（直接访问，零开销）
    uint8_t *send_buffer = MotorManager.RM_MOTOR_DATA_ARRAY;

    // 将RM3508电机的数据进行拼接（使用Fast版本，无边界检查）
    if (st->MotorInf.model == RmM3508)
    {
        for (uint8_t a = st->MotorID - 1; a < g_RM_M3508_NUM; a++)
        {
            MotorTypeDef *motor_a = &MotorManager.MotorList[a];
            uint8_t offset = 2 * a - 2 * (st->MotorID - 1);
            send_buffer[offset] = motor_a->SendMotorData[0];
            send_buffer[offset + 1] = motor_a->SendMotorData[1];
        }
    }

    // 将RM2006电机的数据进行拼接
    if (st->MotorInf.model == RmM2006)
    {
        for (uint8_t a = st->MotorID - 1; a < g_RM_M2006_NUM; a++)
        {
            MotorTypeDef *motor_a = &MotorManager.MotorList[a];
            uint8_t offset = 2 * a - 2 * (st->MotorID - 1);
            send_buffer[offset] = motor_a->SendMotorData[0];
            send_buffer[offset + 1] = motor_a->SendMotorData[1];
        }
    }

    // 将RM6020电机的数据进行拼接
    if (st->MotorInf.model == RmGM6020)
    {
        for (uint8_t a = st->MotorID - 1; a < g_RM_GM6020_NUM; a++)
        {
            MotorTypeDef *motor_a = &MotorManager.MotorList[a];
            uint8_t offset = 2 * a - 2 * (st->MotorID - 1);
            send_buffer[offset] = motor_a->SendMotorData[0];
            send_buffer[offset + 1] = motor_a->SendMotorData[1];
        }
    }

    // 使用Fast版本发送（内联，零开销）
    return Motor_GetHAL_Fast()->can_send(&st->g_TxHeader, send_buffer) ? 1 : 0;
}

/// @brief 兼容旧接口
uint8_t RM_MotorSendControl(MotorTypeDef *st)
{
    return RM_Motor_SendControlInternal(st);
}

void RM_MotorSetTxData(can_motor_cfg motor_cfg, uint8_t *data)
{
    assert_param(data != NULL);
    if (data == NULL)
    {
        return;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    if (motor == NULL)
    {
        return;
    }

    memset(motor->SendMotorData, 0x00, CtrlMotorLen);
    memcpy(motor->SendMotorData, data, CtrlMotorLen);
    motor->SendMotorControl(motor);
}

/*============================== 电机数据解算函数 ==============================*/

/// @brief RM电机通用解算（内部函数，优化版本：使用乘法代替除法）
/// @param motor 电机结构体指针
/// @param current_inv 电流系数的倒数（预计算，避免运行时除法）
static void RM_Motor_CalculateCommon(MotorTypeDef *motor, float current_inv)
{
    MotorSolvedData_t *pData = &motor->motor_data;
    uint8_t *ReceiveData = motor->ReceiveMotorData;

    // 1. 数据解析
    int16_t ecd = (((uint16_t)ReceiveData[0]) << 8) | ReceiveData[1];
    int16_t speed_rpm = (int16_t)((((uint16_t)ReceiveData[2]) << 8) | ReceiveData[3]);
    int16_t current_raw = (int16_t)((((uint16_t)ReceiveData[4]) << 8) | ReceiveData[5]);

    // 2. 首次初始化
    if (pData->init_flag == 0)
    {
        pData->last_ecd = ecd;
        pData->offset_ecd = ecd;
        pData->init_flag = 1;
        pData->offset_ecd_angle = ecd * ECD_TO_DEGREE; // 乘法代替除法
    }

    // 3. 过零检测 & 多圈累计
    int16_t err = ecd - pData->last_ecd;
    pData->total_round += (err > 4096) ? -1 : (err < -4096) ? 1
                                                            : 0;
    pData->total_ecd += err + ((err > 4096) ? -8192 : (err < -4096) ? 8192
                                                                    : 0);
    pData->last_ecd = ecd;

    // 4. 数据转换（优化：使用预计算常量和乘法）
    // 单圈角度 (0~360°)
    pData->solved_data[0] = ecd * ECD_TO_DEGREE;

    // 速度 (rpm) - 低通滤波
    if (!pData->filter_init)
    {
        pData->solved_data[1] = (float)speed_rpm;
        pData->filter_init = 1;
    }
    else
    {
        pData->solved_data[1] = (float)speed_rpm * 0.9f + 0.1f * pData->last_speed;
    }

    // 电流 (A) - 乘法代替除法
    pData->solved_data[2] = current_raw * current_inv;

    // 累计角度 (°)
    pData->solved_data[3] = (pData->total_round * 360.0f + pData->solved_data[0]);

    // 速度 (rad/s)
    pData->solved_data[4] = speed_rpm * 0.10472f; // 2*PI/60 ≈ 0.10472

    pData->last_speed = pData->solved_data[1];
}

static void RM_M2006_CalculateInternal(MotorTypeDef *motor)
{
    if (motor == NULL)
        return;
    RM_Motor_CalculateCommon(motor, RM_M2006_CURRENT_INV); // 使用预计算倒数
}

static void RM_M3508_CalculateInternal(MotorTypeDef *motor)
{
    if (motor == NULL)
        return;
    RM_Motor_CalculateCommon(motor, RM_M3508_CURRENT_INV); // 使用预计算倒数
}

static void RM_GM6020_CalculateInternal(MotorTypeDef *motor)
{
    if (motor == NULL)
        return;
    RM_Motor_CalculateCommon(motor, RM_GM6020_CURRENT_INV); // 使用预计算倒数
}

/*============================== 兼容旧接口的解算函数 ==============================*/

// void RM_M2006_Calculate(MotorTypeDef *motor)
// {
//     RM_M2006_CalculateInternal(motor);
// }

// void RM_M3508_Calculate(MotorTypeDef *motor)
// {
//     RM_M3508_CalculateInternal(motor);
// }

// void RM_GM6020_Calculate(MotorTypeDef *motor)
// {
//     RM_GM6020_CalculateInternal(motor);
// }

/// @brief 通用RM电机解算（根据型号自动选择）
void RM_MOTOR_CALCU(MotorTypeDef *motor)
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
    // case RmM2006:
    //     RM_M2006_Calculate(motor);
    //     break;
    // case RmM3508:
    //     RM_M3508_Calculate(motor);
    //     break;
    // case RmGM6020:
    //     RM_GM6020_Calculate(motor);
    //     break;
    // default:
    //     break;
    // }
}

/*============================== 辅助函数 ==============================*/

void RM_Motor_Reset_Zero(MotorTypeDef *motor)
{
    if (motor != NULL)
    {
        motor->motor_data.total_round = 0;
        motor->motor_data.total_ecd = 0;
        motor->motor_data.offset_ecd = motor->motor_data.last_ecd;
        motor->motor_data.target_angle = motor->motor_data.solved_data[3];
        motor->motor_data.last_target = motor->motor_data.target_angle;
        motor->motor_data.pre_last_target = motor->motor_data.target_angle;
        motor->motor_data.target_init_flag = 0;
    }
}

void RM_Motor_Reset_All(void)
{
    for (uint8_t i = 0; i < RM_MOTOR_MAX_NUM; i++)
    {
        MotorTypeDef *motor = &MotorManager.MotorList[i];
        memset(&motor->motor_data, 0, sizeof(MotorSolvedData_t));
    }
}

/*============================== 暴露接口 ==============================*/

void RmMotorSendCfg(can_motor_cfg motor_cfg, int16_t TargetCurrent)
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    if (motor == NULL)
        return;

    // 考虑反向
    if (motor->config.reverse)
    {
        TargetCurrent = -TargetCurrent;
    }

    if (TargetCurrent < 0)
    {
        TargetCurrent = (uint16_t)(~(-TargetCurrent));
    }
    uint8_t data[8] = {0x00};
    data[0] = (uint8_t)(TargetCurrent >> 8);
    data[1] = (uint8_t)TargetCurrent;
    RM_MotorSetTxData(motor_cfg, data);
}

void RmTestMotorSingleRegister(void)
{
    // 直接访问电机句柄
    MotorTypeDef *motor = &MotorManager.MotorList[SingleMotorTest - 1];

    // 使用新的初始化方式
    RM_M2006_Init(motor, SingleMotorTest);
    Motor_SetRegisteredCount(1);

    // 配置PID (外环: 位置环, 内环: 速度环)
    RM_Motor_SetCascadePID(motor,
                           0.000091f, 0.0f, 0.0f, 0.002f, // 外环 P/I/D/F
                           27.91f, 0.08f, 0.0f, 1.0f,     // 内环 P/I/D/F
                           20673.0f, 0.0f, 5000.0f,       // 外环 max_out, min_out, max_iout
                           3000.0f, 0.0f, 1000.0f);       // 内环 max_out, min_out, max_iout
}

float RmMotorRemoveBias(can_motor_cfg motor_cfg, float Target, bool ChangeVel)
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];

    if (motor->MotorInf.band != RM_MOTOR_BAND)
    {
        return Target;
    }

    MotorSolvedData_t *pData = &motor->motor_data;

    while (!pData->filter_init)
        ;

    // 使用配置中的位置容限
    float tolerance = motor->config.position_tolerance;
    if (tolerance <= 0.0f)
        tolerance = 50.0f; // 默认值

    if (pData->target_init_flag == 0)
    {
        float new_target = pData->solved_data[3] + Target;
        pData->target_angle = new_target;
        pData->last_target = new_target;
        pData->pre_last_target = new_target;
        pData->target_init_flag = 1;
        return new_target;
    }

    float current_angle = pData->solved_data[3];
    float last_target_angle = pData->target_angle;

    if ((current_angle >= last_target_angle - tolerance) &&
        (current_angle <= last_target_angle + tolerance) && ChangeVel)
    {
        float new_target = pData->solved_data[3] + Target;

        // 使用配置中的换向补偿
        float bias = motor->config.direction_bias;
        if (bias != 0.0f && motor->cascade_pid.inner.calc_count)
        {
            if ((pData->last_target > new_target) && (pData->pre_last_target < pData->last_target))
            {
                new_target -= bias;
            }
            else if ((pData->last_target < new_target) && (pData->pre_last_target > pData->last_target))
            {
                new_target += bias;
            }
        }

        pData->pre_last_target = pData->last_target;
        pData->last_target = new_target;
        pData->target_angle = new_target;

        return new_target;
    }

    else
    {
        return pData->target_angle;
    }
}

// 该接口会直接发送数据到电机，但是新接口不会
void RmMotorPID_Calc(can_motor_cfg motor_cfg, float target)
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    if (motor == NULL)
        return;

    MotorSolvedData_t *pData = &motor->motor_data;

    output = CASCADE_PID_Calculate(&motor->cascade_pid, target, pData->solved_data[3], pData->solved_data[4]);
    // output = PID_Calculate(&motor->inner_pid, target, pData->solved_data[4]);
    RmMotorSendCfg(motor_cfg, output);
}

/*============================== PID计算内部函数(新接口) ==============================*/

/// @brief RM电机PID计算（内部实现）
static float RM_Motor_PIDCalcInternal(MotorTypeDef *motor, float target)
{
    if (motor == NULL)
        return 0.0f;

    MotorSolvedData_t *pData = &motor->motor_data;
    float out;

    if (motor->use_cascade)
    {
        out = CASCADE_PID_Calculate(&motor->cascade_pid, target, pData->solved_data[3], pData->solved_data[4]);
    }
    else
    {
        out = PID_Calculate(&motor->inner_pid, target, pData->solved_data[4]);
    }

    // 限幅
    if (out > motor->params.max_current)
        out = motor->params.max_current;
    if (out < -motor->params.max_current)
        out = -motor->params.max_current;

    // 考虑反向
    int16_t current = (int16_t)out;
    if (motor->config.reverse)
    {
        current = -current;
    }

    // 写入发送数据
    if (current < 0)
    {
        current = (uint16_t)(~(-current));
    }
    motor->SendMotorData[0] = (uint8_t)(current >> 8);
    motor->SendMotorData[1] = (uint8_t)current;

    return out;
}

/// @brief RM电机PID计算
float RM_Motor_PID_Calc(can_motor_cfg motor_cfg, float target)
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    return RM_Motor_PIDCalcInternal(motor, target);
}
