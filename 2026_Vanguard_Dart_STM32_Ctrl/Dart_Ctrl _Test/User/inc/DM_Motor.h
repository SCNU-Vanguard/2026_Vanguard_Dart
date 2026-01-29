#ifndef __DM_MOTOR_H_
#define __DM_MOTOR_H_

#include "main.h"
#include "bsp_can.h"
#include "CanMotor.h"
#include <stdlib.h>
#include <string.h>

/*============================== DM电机模式定义 ==============================*/

#define DM_TestUse 0U

/*============================== DM电机工作模式枚举 ==============================*/

typedef enum
{
    DM_MIT = 1,        // MIT模式
    DM_LOCATION_SPEED, // 位置速度模式
    DM_SPEED,          // 速度模式
    DM_PVT             // PVT模式
} DM_WorkMode;

/*============================== DM电机CAN地址定义（按模式） ==============================*/

// MIT模式地址
#define DM_MIT_TX_BIAS 0x000
#define DM_MIT_RX_BIAS 0x010

// 位置速度模式地址
#define DM_LS_TX_BIAS 0x100
#define DM_LS_RX_BIAS 0x020

// 速度模式地址
#define DM_SPD_TX_BIAS 0x200
#define DM_SPD_RX_BIAS 0x030

// PVT模式地址
#define DM_PVT_TX_BIAS 0x300
#define DM_PVT_RX_BIAS 0x040

/*============================== DM电机数据位数定义（所有模式） ==============================*/

// MIT模式数据位数（使用float_to_uint线性映射）
#define DM_MIT_POS_BIT 16
#define DM_MIT_VEL_BIT 12
#define DM_MIT_TORQUE_BIT 12
#define DM_MIT_KP_BIT 12
#define DM_MIT_KD_BIT 12

// 位置速度模式：直接发送float原始字节（IEEE 754格式），以下定义仅供参考
#define DM_LS_POS_BIT 32
#define DM_LS_VEL_BIT 32

// 速度模式：直接发送float原始字节（IEEE 754格式），以下定义仅供参考
#define DM_SPD_VEL_BIT 32

// PVT模式数据位数（待确认）
#define DM_PVT_POS_BIT 32
#define DM_PVT_VEL_BIT 16
#define DM_PVT_CURRENT_BIT 16

#define g_DM_MOTOR_NUM 3

/*============================== DM电机ID配置表（集中管理） ==============================*/

// DM电机ID配置结构体
typedef struct
{
    uint8_t motor_id;      // 达妙上位机设置的电机ID (0-15)
    uint16_t tx_id;        // 发送CAN ID
    uint16_t rx_id;        // 接收CAN ID
    DM_WorkMode work_mode; // 工作模式
} DM_MotorIdConfig_t;

// 获取DM电机ID配置（根据电机枚举）
const DM_MotorIdConfig_t *DM_GetIdConfig(can_motor_cfg motor_cfg);

/*============================== DM电机参数常量 ==============================*/

// 数据类型枚举
typedef enum
{
    DM_POS = 0,
    DM_VEL,
    DM_KP,
    DM_KD,
    DM_TORQUE,
} DM_DATA;

/*============================== 前向声明 ==============================*/
#pragma pack(push, 1)
struct _MotorTypeDef; // 前向声明电机结构体

/*============================== DM电机类定义（面向对象） ==============================*/

/// @brief DM电机类结构体 - 类似C++中的类，包含静态参数和虚函数表
typedef struct _DM_MotorClass
{
    // ==================== 电机类型标识 ====================
    const char *name; // 电机类型名称 (如 "J3519", "J4310")
    uint8_t model;    // 电机型号枚举值 (can_motor_model)

    /// @brief 达妙电机工作模式，根据工作模式进行设置偏移ID和接收ID
    DM_WorkMode WorkMode;

    // ==================== 限幅参数（只读，由型号决定）====================
    float kp_max;     // KP上限
    float kp_min;     // KP下限
    float kd_max;     // KD上限
    float kd_min;     // KD下限
    float pos_max;    // 位置上限(rad)
    float pos_min;    // 位置下限(rad)
    float vel_max;    // 速度上限(rad/s)
    float vel_min;    // 速度下限(rad/s)
    float torque_max; // 力矩上限(Nm)
    float torque_min; // 力矩下限(Nm)

    // ==================== 默认控制参数 ====================
    float default_kp;        // 默认KP
    float default_kd;        // 默认KD
    float default_torque_ff; // 默认前馈力矩(Nm)

    // ==================== 虚函数表 ====================
    /// @brief 初始化电机实例
    void (*init)(struct _MotorTypeDef *self, uint8_t id);

    /// @brief 解算电机反馈数据
    void (*calculate)(struct _MotorTypeDef *self);

    /// @brief 发送电机控制数据
    uint8_t (*send_control)(struct _MotorTypeDef *self);

    /// @brief 电机数据刷新函数(只刷新数据)
    void (*refresh_data)(can_motor_cfg motor_cfg);

} DM_MotorClass_t;
#pragma pack(pop)
/*============================== 预定义的电机类实例（类似C++静态类） ==============================*/

extern const DM_MotorClass_t DM_J3519_Class; // J3519电机类
extern const DM_MotorClass_t DM_J4310_Class; // J4310电机类

/*============================== DM电机配置结构 ==============================*/
#pragma pack(push, 1)
// DM电机配置（用户可调）
typedef struct
{
    float kp;        // KP
    float kd;        // KD
    float torque_ff; // 前馈力矩(Nm)

    // 位置和速度限幅参数（可在初始化后修改）
    float pos_max; // 位置上限(rad)
    float pos_min; // 位置下限(rad)
    float vel_max; // 速度上限(rad/s)
    float vel_min; // 速度下限(rad/s)
} DM_MotorConfig_t;
#pragma pack(pop)

// DM电机配置数组（供内联函数使用）
extern DM_MotorConfig_t g_DM_Configs[4];

/// @brief 获取DM电机配置（内联版本，零开销，无边界检查）
/// @param id 电机ID (1-4)，调用者需确保有效性
/// @return 配置指针
static inline DM_MotorConfig_t *DM_Motor_GetConfigFast(uint8_t id)
{
    return &g_DM_Configs[id - 1];
}

/*============================== 辅助函数 ==============================*/

/// @brief 获取KP差值
/// @param motor_class 电机类指针
/// @return KP差值 = kp_max - kp_min
static inline float DM_GetKpDifference(const DM_MotorClass_t *motor_class)
{
    return motor_class->kp_max - motor_class->kp_min;
}

/// @brief 获取KD差值
/// @param motor_class 电机类指针
/// @return KD差值 = kd_max - kd_min
static inline float DM_GetKdDifference(const DM_MotorClass_t *motor_class)
{
    return motor_class->kd_max - motor_class->kd_min;
}

/// @brief 获取位置差值
/// @param motor_class 电机类指针
/// @return 位置差值 = pos_max - pos_min
static inline float DM_GetPosDifference(const DM_MotorClass_t *motor_class)
{
    return motor_class->pos_max - motor_class->pos_min;
}

/// @brief 获取速度差值
/// @param motor_class 电机类指针
/// @return 速度差值 = vel_max - vel_min
static inline float DM_GetVelDifference(const DM_MotorClass_t *motor_class)
{
    return motor_class->vel_max - motor_class->vel_min;
}

/// @brief 获取力矩差值
/// @param motor_class 电机类指针
/// @return 力矩差值 = torque_max - torque_min
static inline float DM_GetTorqueDifference(const DM_MotorClass_t *motor_class)
{
    return motor_class->torque_max - motor_class->torque_min;
}

/*============================== 面向对象接口 ==============================*/

/// @brief 使用指定的电机类创建电机实例
/// @param motor 电机结构体指针
/// @param motor_class 电机类指针 (如 &DM_J3519_Class, &DM_J4310_Class)
/// @param id 电机ID (1-based)
/// @example DM_Motor_Create(&motor, &DM_J4310_Class, 1);
void DM_Motor_Create(MotorTypeDef *motor, const DM_MotorClass_t *motor_class, uint8_t id);

/// @brief 调用电机的解算函数（通过类虚函数表）
/// @param motor 电机结构体指针
/// @note 等同于 motor->motor_class->calculate(motor)
void DM_Motor_Calculate(MotorTypeDef *motor);

/// @brief 调用电机的发送控制函数（通过类虚函数表）
/// @param motor 电机结构体指针
/// @return 发送成功返回1，失败返回0
uint8_t DM_Motor_SendControl(MotorTypeDef *motor);

/// @brief 获取电机所属的类指针
/// @param motor 电机结构体指针
/// @return 电机类指针，失败返回NULL
const DM_MotorClass_t *DM_Motor_GetClass(MotorTypeDef *motor);

/*============================== 电机初始化函数（兼容旧接口） ==============================*/

/// @brief 初始化DM J3519电机实例
/// @param motor 电机结构体指针
/// @param id 电机ID (1-based)
void DM_J3519_Init(MotorTypeDef *motor, uint8_t id);

/// @brief 初始化DM J4310电机实例
/// @param motor 电机结构体指针
/// @param id 电机ID (1-based)
void DM_J4310_Init(MotorTypeDef *motor, uint8_t id);

/*============================== 电机配置函数 ==============================*/

/// @brief 设置DM电机配置
/// @param motor 电机结构体指针
/// @param config 配置结构体指针
void DM_Motor_SetConfig(MotorTypeDef *motor, const DM_MotorConfig_t *config);

/// @brief 获取DM电机配置指针
/// @param motor 电机结构体指针
/// @return 配置指针，失败返回NULL
DM_MotorConfig_t *DM_Motor_GetConfig(MotorTypeDef *motor);

/// @brief 设置DM电机KP参数
/// @param motor 电机结构体指针
/// @param kp KP值 (范围由电机类决定，通常0~500)
void DM_Motor_SetKp(MotorTypeDef *motor, float kp);

/// @brief 设置DM电机KD参数
/// @param motor 电机结构体指针
/// @param kd KD值 (范围由电机类决定，通常0~5)
void DM_Motor_SetKd(MotorTypeDef *motor, float kd);

/// @brief 设置DM电机前馈力矩
/// @param motor 电机结构体指针
/// @param torque_ff 前馈力矩值(Nm) (范围由电机类决定)
void DM_Motor_SetTorqueFF(MotorTypeDef *motor, float torque_ff);

/// @brief 设置DM电机位置限幅参数
/// @param motor 电机结构体指针
/// @param pos_min 位置下限(rad)
/// @param pos_max 位置上限(rad)
void DM_Motor_SetPosLimits(MotorTypeDef *motor, float pos_min, float pos_max);

/// @brief 设置DM电机速度限幅参数
/// @param motor 电机结构体指针
/// @param vel_min 速度下限(rad/s)
/// @param vel_max 速度上限(rad/s)
void DM_Motor_SetVelLimits(MotorTypeDef *motor, float vel_min, float vel_max);

/// @brief 一次性设置DM电机MIT控制参数
/// @param motor 电机结构体指针
/// @param kp KP值
/// @param kd KD值
/// @param torque_ff 前馈力矩(Nm)
void DM_Motor_SetMITParams(MotorTypeDef *motor, float kp, float kd, float torque_ff);

/// @brief 获取DM电机当前KP值
/// @param motor 电机结构体指针
/// @return KP值，失败返回0
float DM_Motor_GetKp(MotorTypeDef *motor);

/// @brief 获取DM电机当前KD值
/// @param motor 电机结构体指针
/// @return KD值，失败返回0
float DM_Motor_GetKd(MotorTypeDef *motor);

/// @brief 获取DM电机当前前馈力矩值
/// @param motor 电机结构体指针
/// @return 前馈力矩值(Nm)，失败返回0
float DM_Motor_GetTorqueFF(MotorTypeDef *motor);

/*============================== 原有函数声明 ==============================*/

/// @brief 用于失能达妙电机
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @return 1：发送成功，0：发送失败
uint8_t DM_MotorDisable(can_motor_cfg motor_cfg);

/// @brief 用于使能达妙电机
/// @param motor 电机结构体指针
/// @return 1：发送成功，0：发送失败
uint8_t DM_Motor_Enable(MotorTypeDef *motor);

/// @brief 用于失能达妙电机
/// @param motor 电机结构体指针
/// @return 1：发送成功，0：发送失败
uint8_t DM_Motor_Disable(MotorTypeDef *motor);

/// @brief 设置达妙电机发送的数据
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param data 数据所在数组的指针
void DM_MotorSetTxData(can_motor_cfg motor_cfg, uint8_t *data);

/// @brief DM电机的解算
/// @param motor 电机结构体指针
/// @note motor->motor_data.solved_data[0]: 位置(rad/°)
/// @note motor->motor_data.solved_data[1]: 速度(rad/s)
/// @note motor->motor_data.solved_data[2]: 力矩(N·m)
/// @note motor->motor_data.solved_data[3]: MOS温度(℃)
/// @note motor->motor_data.solved_data[4]: 转子温度(℃)
void DM_MOTOR_CALCU(MotorTypeDef *motor);

/// @brief 用于设置发送DM电机数据
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param TargetPos 目标位置
/// @param TargetVel 目标速度
/// @param workmode  达妙电机的工作模式
void DmMotorSendCfg(can_motor_cfg motor_cfg, float TargetPos, float TargetVel, DM_WorkMode workmode);

/// @brief DM电机MIT模式控制
/// @param motor 电机结构体指针
/// @param pos 目标位置(rad)
/// @param vel 目标速度(rad/s)
// void DM_Motor_MIT_Ctrl(MotorTypeDef *motor, float pos, float vel);

/// @brief DM电机输出
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param target 目标值
void DmMotorPID_Calc(can_motor_cfg motor_cfg, float target);

/// @brief 电机数据刷新函数(只刷新数据)
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
void DM_Motor_RefreshData(can_motor_cfg motor_cfg);

#endif
