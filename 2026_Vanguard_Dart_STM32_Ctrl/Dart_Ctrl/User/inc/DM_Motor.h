#ifndef __DM_MOTOR_H_ /* 按 __DM_MOTOR_H_ 选择编译分支。 */
#define __DM_MOTOR_H_ /* 定义 __DM_MOTOR_H_。 */

#include "main.h"
#include "bsp_can.h"
#include "CanMotor.h"
#include <stdlib.h>
#include <string.h>

/*============================== DM电机模式定义 ==============================*/

#define DM_TestUse 0U /* 定义 DM_TestUse。 */

/*============================== DM电机工作模式枚举 ==============================*/

typedef enum /* 开始定义数据类型。 */
{
    DM_MIT = 1,        // MIT模式
    DM_LOCATION_SPEED, // 位置速度模式
    DM_SPEED,          // 速度模式
    DM_PVT             // PVT模式
} DM_WorkMode; /* 结束 DM_WorkMode 类型定义。 */

/*============================== DM电机CAN地址定义（按模式） ==============================*/

// MIT模式地址
#define DM_MIT_TX_BIAS 0x000 /* 定义 DM_MIT_TX_BIAS。 */
#define DM_MIT_RX_BIAS 0x010 /* 定义 DM_MIT_RX_BIAS。 */

// 位置速度模式地址
#define DM_LS_TX_BIAS 0x100 /* 定义 DM_LS_TX_BIAS。 */
#define DM_LS_RX_BIAS 0x020 /* 定义 DM_LS_RX_BIAS。 */

// 速度模式地址
#define DM_SPD_TX_BIAS 0x200 /* 定义 DM_SPD_TX_BIAS。 */
#define DM_SPD_RX_BIAS 0x030 /* 定义 DM_SPD_RX_BIAS。 */

// PVT模式地址
#define DM_PVT_TX_BIAS 0x300 /* 定义 DM_PVT_TX_BIAS。 */
#define DM_PVT_RX_BIAS 0x040 /* 定义 DM_PVT_RX_BIAS。 */

/*============================== DM电机数据位数定义（所有模式） ==============================*/

// MIT模式数据位数（使用float_to_uint线性映射）
#define DM_MIT_POS_BIT 16 /* 定义 DM_MIT_POS_BIT。 */
#define DM_MIT_VEL_BIT 12 /* 定义 DM_MIT_VEL_BIT。 */
#define DM_MIT_TORQUE_BIT 12 /* 定义 DM_MIT_TORQUE_BIT。 */
#define DM_MIT_KP_BIT 12 /* 定义 DM_MIT_KP_BIT。 */
#define DM_MIT_KD_BIT 12 /* 定义 DM_MIT_KD_BIT。 */

// 位置速度模式：直接发送float原始字节（IEEE 754格式），以下定义仅供参考
#define DM_LS_POS_BIT 32 /* 定义 DM_LS_POS_BIT。 */
#define DM_LS_VEL_BIT 32 /* 定义 DM_LS_VEL_BIT。 */

// 速度模式：直接发送float原始字节（IEEE 754格式），以下定义仅供参考
#define DM_SPD_VEL_BIT 32 /* 定义 DM_SPD_VEL_BIT。 */

// PVT模式数据位数（待确认）
#define DM_PVT_POS_BIT 32 /* 定义 DM_PVT_POS_BIT。 */
#define DM_PVT_VEL_BIT 16 /* 定义 DM_PVT_VEL_BIT。 */
#define DM_PVT_CURRENT_BIT 16 /* 定义 DM_PVT_CURRENT_BIT。 */

#define g_DM_MOTOR_NUM 3 /* 定义 g_DM_MOTOR_NUM。 */

/*============================== DM电机ID配置表（集中管理） ==============================*/

// DM电机ID配置结构体
typedef struct /* 开始定义数据类型。 */
{
    uint8_t motor_id;      // 达妙上位机设置的电机ID (0-15)
    uint16_t tx_id;        // 发送CAN ID
    uint16_t rx_id;        // 接收CAN ID
    DM_WorkMode work_mode; // 工作模式
} DM_MotorIdConfig_t; /* 结束 DM_MotorIdConfig_t 类型定义。 */

// 获取DM电机ID配置（根据电机枚举）
const DM_MotorIdConfig_t *DM_GetIdConfig(can_motor_cfg motor_cfg); /* 声明 DM_GetIdConfig 接口。 */

/*============================== DM电机参数常量 ==============================*/

// 数据类型枚举
typedef enum /* 开始定义数据类型。 */
{
    DM_POS = 0, /* 定义 DM_POS 枚举项。 */
    DM_VEL, /* 定义 DM_VEL 枚举项。 */
    DM_KP, /* 定义 DM_KP 枚举项。 */
    DM_KD, /* 定义 DM_KD 枚举项。 */
    DM_TORQUE, /* 定义 DM_TORQUE 枚举项。 */
} DM_DATA; /* 结束 DM_DATA 类型定义。 */

/*============================== 前向声明 ==============================*/
#pragma pack(push, 1) /* 配置编译选项 pack(push, 1)。 */
struct _MotorTypeDef; // 前向声明电机结构体

/*============================== DM电机类定义（面向对象） ==============================*/

/// @brief DM电机类结构体 - 类似C++中的类，包含静态参数和虚函数表
typedef struct _DM_MotorClass /* 开始定义数据类型。 */
{
    // ==================== 电机类型标识 ====================
    const char *name; // 电机类型名称 (如 "J3519", "J4310")
    uint8_t model;    // 电机型号枚举值 (can_motor_model)

    /// @brief 达妙电机工作模式，根据工作模式进行设置偏移ID和接收ID
    DM_WorkMode WorkMode; /* 保存 WorkMode。 */

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
    void (*init)(struct _MotorTypeDef *self, uint8_t id); /* 调用 void。 */

    /// @brief 解算电机反馈数据
    void (*calculate)(struct _MotorTypeDef *self); /* 调用 void。 */

    /// @brief 发送电机控制数据
    uint8_t (*send_control)(struct _MotorTypeDef *self); /* 调用 uint8_t。 */

    /// @brief 电机数据刷新函数(只刷新数据)
    void (*refresh_data)(can_motor_cfg motor_cfg); /* 调用 void。 */

} DM_MotorClass_t; /* 结束 DM_MotorClass_t 类型定义。 */
#pragma pack(pop) /* 配置编译选项 pack(pop)。 */
/*============================== 预定义的电机类实例（类似C++静态类） ==============================*/

extern const DM_MotorClass_t DM_J3519_Class; // J3519电机类
extern const DM_MotorClass_t DM_J4310_Class; // J4310电机类

/*============================== DM电机配置结构 ==============================*/
#pragma pack(push, 1) /* 配置编译选项 pack(push, 1)。 */
// DM电机配置（用户可调）
typedef struct /* 开始定义数据类型。 */
{
    float kp;        // KP
    float kd;        // KD
    float torque_ff; // 前馈力矩(Nm)

    // 位置和速度限幅参数（可在初始化后修改）
    float pos_max; // 位置上限(rad)
    float pos_min; // 位置下限(rad)
    float vel_max; // 速度上限(rad/s)
    float vel_min; // 速度下限(rad/s)
} DM_MotorConfig_t; /* 结束 DM_MotorConfig_t 类型定义。 */
#pragma pack(pop) /* 配置编译选项 pack(pop)。 */

// DM电机配置数组（供内联函数使用）
extern DM_MotorConfig_t g_DM_Configs[4]; /* 声明外部变量 g_DM_Configs。 */

/// @brief 获取DM电机配置（内联版本，零开销，无边界检查）
/// @param id 电机ID (1-4)，调用者需确保有效性
/// @return 配置指针
static inline DM_MotorConfig_t *DM_Motor_GetConfigFast(uint8_t id) /* 实现 DM_Motor_GetConfigFast。 */
{
    return &g_DM_Configs[id - 1]; /* 返回当前计算结果。 */
}

/*============================== 辅助函数 ==============================*/

/// @brief 获取KP差值
/// @param motor_class 电机类指针
/// @return KP差值 = kp_max - kp_min
static inline float DM_GetKpDifference(const DM_MotorClass_t *motor_class) /* 实现 DM_GetKpDifference。 */
{
    return motor_class->kp_max - motor_class->kp_min; /* 返回当前计算结果。 */
}

/// @brief 获取KD差值
/// @param motor_class 电机类指针
/// @return KD差值 = kd_max - kd_min
static inline float DM_GetKdDifference(const DM_MotorClass_t *motor_class) /* 实现 DM_GetKdDifference。 */
{
    return motor_class->kd_max - motor_class->kd_min; /* 返回当前计算结果。 */
}

/// @brief 获取位置差值
/// @param motor_class 电机类指针
/// @return 位置差值 = pos_max - pos_min
static inline float DM_GetPosDifference(const DM_MotorClass_t *motor_class) /* 实现 DM_GetPosDifference。 */
{
    return motor_class->pos_max - motor_class->pos_min; /* 返回当前计算结果。 */
}

/// @brief 获取速度差值
/// @param motor_class 电机类指针
/// @return 速度差值 = vel_max - vel_min
static inline float DM_GetVelDifference(const DM_MotorClass_t *motor_class) /* 实现 DM_GetVelDifference。 */
{
    return motor_class->vel_max - motor_class->vel_min; /* 返回当前计算结果。 */
}

/// @brief 获取力矩差值
/// @param motor_class 电机类指针
/// @return 力矩差值 = torque_max - torque_min
static inline float DM_GetTorqueDifference(const DM_MotorClass_t *motor_class) /* 实现 DM_GetTorqueDifference。 */
{
    return motor_class->torque_max - motor_class->torque_min; /* 返回当前计算结果。 */
}

/*============================== 面向对象接口 ==============================*/

/// @brief 使用指定的电机类创建电机实例
/// @param motor 电机结构体指针
/// @param motor_class 电机类指针 (如 &DM_J3519_Class, &DM_J4310_Class)
/// @param id 电机ID (1-based)
/// @example DM_Motor_Create(&motor, &DM_J4310_Class, 1);
void DM_Motor_Create(MotorTypeDef *motor, const DM_MotorClass_t *motor_class, uint8_t id); /* 声明 DM_Motor_Create 接口。 */

/// @brief 调用电机的解算函数（通过类虚函数表）
/// @param motor 电机结构体指针
/// @note 等同于 motor->motor_class->calculate(motor)
void DM_Motor_Calculate(MotorTypeDef *motor); /* 声明 DM_Motor_Calculate 接口。 */

/// @brief 获取电机所属的类指针
/// @param motor 电机结构体指针
/// @return 电机类指针，失败返回NULL
const DM_MotorClass_t *DM_Motor_GetClass(MotorTypeDef *motor); /* 声明 DM_Motor_GetClass 接口。 */

/*============================== 电机初始化函数（兼容旧接口） ==============================*/

/// @brief 初始化DM J3519电机实例
/// @param motor 电机结构体指针
/// @param id 电机ID (1-based)
void DM_J3519_Init(MotorTypeDef *motor, uint8_t id); /* 声明 DM_J3519_Init 接口。 */

/// @brief 初始化DM J4310电机实例
/// @param motor 电机结构体指针
/// @param id 电机ID (1-based)
void DM_J4310_Init(MotorTypeDef *motor, uint8_t id); /* 声明 DM_J4310_Init 接口。 */

/*============================== 电机配置函数 ==============================*/

/// @brief 设置DM电机配置
/// @param motor 电机结构体指针
/// @param config 配置结构体指针
void DM_Motor_SetConfig(MotorTypeDef *motor, const DM_MotorConfig_t *config); /* 声明 DM_Motor_SetConfig 接口。 */

/// @brief 获取DM电机配置指针
/// @param motor 电机结构体指针
/// @return 配置指针，失败返回NULL
DM_MotorConfig_t *DM_Motor_GetConfig(MotorTypeDef *motor); /* 声明 DM_Motor_GetConfig 接口。 */

/// @brief 配置DM电机串级PID
/// @param motor 电机结构体指针
/// @param outer_p/i/d/f 外环PID参数
/// @param inner_p/i/d/f 内环PID参数
/// @param outer_max_out 外环输出上限
/// @param outer_min_out 外环输出下限
/// @param outer_max_iout 外环积分限幅
/// @param inner_max_out 内环输出上限
/// @param inner_min_out 内环输出下限
/// @param inner_max_iout 内环积分限幅
void DM_Motor_SetCascadePID(MotorTypeDef *motor, /* 传入下一项参数或数据。 */
                            float outer_p, float outer_i, float outer_d, float outer_f, /* 传入下一项参数或数据。 */
                            float inner_p, float inner_i, float inner_d, float inner_f, /* 传入下一项参数或数据。 */
                            float outer_max_out, float outer_min_out, float outer_max_iout, /* 传入下一项参数或数据。 */
                            float inner_max_out, float inner_min_out, float inner_max_iout); /* 完成本行操作。 */

/// @brief 配置DM电机单环速度PID
/// @param motor 电机结构体指针
/// @param mode PID模式 (PID_POSITION 或 PID_DELTA)
/// @param p/i/d/f PID参数
/// @param max_out 输出上限
/// @param min_out 输出下限
/// @param max_iout 积分限幅
void DM_Motor_SetSpeedPID(MotorTypeDef *motor, PID_MODE_e mode, /* 传入下一项参数或数据。 */
                          float p, float i, float d, float f, /* 传入下一项参数或数据。 */
                          float max_out, float min_out, float max_iout); /* 完成本行操作。 */

/// @brief 设置DM电机KP参数
/// @param motor 电机结构体指针
/// @param kp KP值 (范围由电机类决定，通常0~500)
void DM_Motor_SetKp(MotorTypeDef *motor, float kp); /* 声明 DM_Motor_SetKp 接口。 */

/// @brief 设置DM电机KD参数
/// @param motor 电机结构体指针
/// @param kd KD值 (范围由电机类决定，通常0~5)
void DM_Motor_SetKd(MotorTypeDef *motor, float kd); /* 声明 DM_Motor_SetKd 接口。 */

/// @brief 设置DM电机前馈力矩
/// @param motor 电机结构体指针
/// @param torque_ff 前馈力矩值(Nm) (范围由电机类决定)
void DM_Motor_SetTorqueFF(MotorTypeDef *motor, float torque_ff); /* 声明 DM_Motor_SetTorqueFF 接口。 */

/// @brief 设置DM电机位置限幅参数
/// @param motor 电机结构体指针
/// @param pos_min 位置下限(rad)
/// @param pos_max 位置上限(rad)
void DM_Motor_SetPosLimits(MotorTypeDef *motor, float pos_min, float pos_max); /* 声明 DM_Motor_SetPosLimits 接口。 */

/// @brief 设置DM电机速度限幅参数
/// @param motor 电机结构体指针
/// @param vel_min 速度下限(rad/s)
/// @param vel_max 速度上限(rad/s)
void DM_Motor_SetVelLimits(MotorTypeDef *motor, float vel_min, float vel_max); /* 声明 DM_Motor_SetVelLimits 接口。 */

/// @brief 一次性设置DM电机MIT控制参数
/// @param motor 电机结构体指针
/// @param kp KP值
/// @param kd KD值
/// @param torque_ff 前馈力矩(Nm)
void DM_Motor_SetMITParams(MotorTypeDef *motor, float kp, float kd, float torque_ff); /* 声明 DM_Motor_SetMITParams 接口。 */

/// @brief 获取DM电机当前KP值
/// @param motor 电机结构体指针
/// @return KP值，失败返回0
float DM_Motor_GetKp(MotorTypeDef *motor); /* 声明 DM_Motor_GetKp 接口。 */

/// @brief 获取DM电机当前KD值
/// @param motor 电机结构体指针
/// @return KD值，失败返回0
float DM_Motor_GetKd(MotorTypeDef *motor); /* 声明 DM_Motor_GetKd 接口。 */

/// @brief 获取DM电机当前前馈力矩值
/// @param motor 电机结构体指针
/// @return 前馈力矩值(Nm)，失败返回0
float DM_Motor_GetTorqueFF(MotorTypeDef *motor); /* 声明 DM_Motor_GetTorqueFF 接口。 */

/*============================== 原有函数声明 ==============================*/
/// @brief 用于使能达妙电机
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @return 1：发送成功，0：发送失败
uint8_t DM_MotorEnable(can_motor_cfg motor_cfg); /* 声明 DM_MotorEnable 接口。 */

/// @brief 用于失能达妙电机
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @return 1：发送成功，0：发送失败
uint8_t DM_MotorDisable(can_motor_cfg motor_cfg); /* 声明 DM_MotorDisable 接口。 */

/// @brief 用于使能达妙电机
/// @param motor 电机结构体指针
/// @return 1：发送成功，0：发送失败
uint8_t DM_Motor_Enable(MotorTypeDef *motor); /* 声明 DM_Motor_Enable 接口。 */

/// @brief 用于失能达妙电机
/// @param motor 电机结构体指针
/// @return 1：发送成功，0：发送失败
uint8_t DM_Motor_Disable(MotorTypeDef *motor); /* 声明 DM_Motor_Disable 接口。 */

/// @brief 设置达妙电机发送的数据
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param data 数据所在数组的指针
void DM_MotorSetTxData(can_motor_cfg motor_cfg, uint8_t *data); /* 声明 DM_MotorSetTxData 接口。 */

/// @brief DM电机的解算
/// @param motor 电机结构体指针
/// @note motor->motor_data.solved_data[0]: 位置(rad/°)
/// @note motor->motor_data.solved_data[1]: 速度(rad/s)
/// @note motor->motor_data.solved_data[2]: 力矩(N·m)
/// @note motor->motor_data.solved_data[3]: MOS温度(℃)
/// @note motor->motor_data.solved_data[4]: 转子温度(℃)
void DM_MOTOR_CALCU(MotorTypeDef *motor); /* 声明 DM_MOTOR_CALCU 接口。 */

/// @brief 用于设置发送DM电机数据
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param TargetPos 目标位置
/// @param TargetVel 目标速度
/// @param workmode  达妙电机的工作模式
void DmMotorSendCfg(can_motor_cfg motor_cfg, float TargetPos, float TargetVel, float TargetTorque, DM_WorkMode workmode); /* 声明 DmMotorSendCfg 接口。 */

/// @brief DM电机MIT模式控制
/// @param motor 电机结构体指针
/// @param pos 目标位置(rad)
/// @param vel 目标速度(rad/s)
// void DM_Motor_MIT_Ctrl(MotorTypeDef *motor, float pos, float vel);

/// @brief DM电机输出
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param target 目标值
void DmMotorPID_Calc(can_motor_cfg motor_cfg, float target); /* 声明 DmMotorPID_Calc 接口。 */

void DmMotorSpeedPID_Calc(can_motor_cfg motor_cfg, float target_speed_rpm); /* 声明 DmMotorSpeedPID_Calc 接口。 */

/* angle_motor 纯控制器后端：DM 位置速度模式，透传参考位置（电机内部闭环）。 */
float Dm_ComputePosVel(can_motor_cfg cfg, float ref_pos_deg); /* 声明 Dm_ComputePosVel 接口。 */

/// @brief 电机数据刷新函数(只刷新数据)
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
void DM_Motor_RefreshData(can_motor_cfg motor_cfg); /* 声明 DM_Motor_RefreshData 接口。 */

bool DM_Motor_Is3519StallProtected(void); /* 声明 DM_Motor_Is3519StallProtected 接口。 */
void DM_Motor_Clear3519StallProtection(void); /* 声明 DM_Motor_Clear3519StallProtection 接口。 */

#endif /* 结束条件编译。 */
