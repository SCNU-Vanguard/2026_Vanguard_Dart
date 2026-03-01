#ifndef __RM_MOTOR_H_
#define __RM_MOTOR_H_

#include "main.h"
#include "bsp_can.h"
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*============================== RM电机CAN地址定义 ==============================*/

#define g_RM_MOTOR_BIAS_ADDR 0x200
#define g_RM_MOTOR_BIAS_ADDR_3508 0x200
#define g_RM_MOTOR_BIAS_ADDR_2006 0x1FF
#define g_RM_MOTOR_BIAS_ADDR_6020 0x2FE

#define g_RM_MOTOR_NUM 2
#define g_RM_M3508_NUM 1
#define g_RM_M2006_NUM 1
#define g_RM_GM6020_NUM 0

#define RM_TestUse 0U

/*============================== RM电机参数常量 ==============================*/

// M2006电机参数 (C610电调)
#define RM_M2006_GEAR_RATIO 36.0f    // 减速比
#define RM_M2006_MAX_CURRENT 10000   // 最大电流
#define RM_M2006_CURRENT_RATIO 0.18f // 电流转换系数

// M3508电机参数 (C620电调)
#define RM_M3508_GEAR_RATIO 19.2f                 // 减速比
#define RM_M3508_MAX_CURRENT 16384                // 最大电流
#define RM_M3508_CURRENT_RATIO (16384.0f / 20.0f) // 16384对应20A

// GM6020电机参数
#define RM_GM6020_GEAR_RATIO 1.0f                 // 减速比
#define RM_GM6020_MAX_CURRENT 30000               // 最大电流（电压控制）
#define RM_GM6020_CURRENT_RATIO (16384.0f / 3.0f) // 16384对应3A

/*============================== 前向声明 ==============================*/
#pragma pack(push, 1)
struct _MotorTypeDef; // 前向声明电机结构体

/*============================== RM电机类定义（面向对象） ==============================*/

/// @brief RM电机类结构体 - 类似C++中的类，包含静态参数和虚函数表
typedef struct _RM_MotorClass
{
    // ==================== 电机类型标识 ====================
    const char *name; // 电机类型名称 (如 "M2006", "M3508", "GM6020")
    uint8_t model;    // 电机型号枚举值 (can_motor_model)

    // ==================== 默认参数（只读，由型号决定）====================
    float gear_ratio;      // 减速比
    int16_t max_current;   // 最大电流/电压
    float current_ratio;   // 电流转换系数
    uint16_t tx_base_addr; // CAN发送基地址
    uint8_t id_min;        // 最小ID
    uint8_t id_max;        // 最大ID

    // ==================== 虚函数表（多态支持）====================
    /// @brief 初始化电机实例
    void (*init)(struct _MotorTypeDef *self, uint8_t id);

    /// @brief 解算电机反馈数据
    void (*calculate)(struct _MotorTypeDef *self);

    /// @brief 发送电机控制数据
    uint8_t (*send_control)(struct _MotorTypeDef *self);

} RM_MotorClass_t;
#pragma pack(pop)
/*============================== 预定义的电机类实例（类似C++静态类） ==============================*/

extern const RM_MotorClass_t RM_M2006_Class;  // M2006电机类
extern const RM_MotorClass_t RM_M3508_Class;  // M3508电机类
extern const RM_MotorClass_t RM_GM6020_Class; // GM6020电机类

/*============================== RM电机配置结构 ==============================*/
#pragma pack(push, 1)
// RM电机配置（用户可调）
typedef struct
{
    float direction_bias;     // 换向偏移补偿(°)，用于补偿换向时的齿轮间隙
    float position_tolerance; // 位置误差容限(°)，用于判断是否到达目标
    uint8_t reverse;          // 是否反向: 0-正向, 1-反向
} RM_MotorConfig_t;
#pragma pack(pop)

// 需要在CanMotor.h之后包含，因为需要完整的MotorTypeDef定义
#include "CanMotor.h"

/*============================== 面向对象接口（推荐使用） ==============================*/

/// @brief 使用指定的电机类创建电机实例
/// @param motor 电机结构体指针
/// @param motor_class 电机类指针 (如 &RM_M2006_Class, &RM_M3508_Class, &RM_GM6020_Class)
/// @param id 电机ID
/// @example RM_Motor_Create(&motor, &RM_M2006_Class, 1);
void RM_Motor_Create(MotorTypeDef *motor, const RM_MotorClass_t *motor_class, uint8_t id);

/// @brief 调用电机的解算函数（通过类虚函数表）
/// @param motor 电机结构体指针
/// @note 等同于 motor->rm_motor_class->calculate(motor)
void RM_Motor_Calculate(MotorTypeDef *motor);

/// @brief 调用电机的发送控制函数（通过类虚函数表）
/// @param motor 电机结构体指针
/// @return 发送成功返回1，失败返回0
uint8_t RM_Motor_SendControl(MotorTypeDef *motor);

/// @brief 调用电机的PID计算函数（通过类虚函数表）
/// @param motor 电机结构体指针
/// @param target 目标值
/// @return PID输出值
float RM_Motor_PIDCalc(MotorTypeDef *motor, float target);

/*============================== 电机初始化函数（兼容旧接口） ==============================*/

/// @brief 初始化M2006电机实例
/// @param motor 电机结构体指针
/// @param id 电机ID (1-4)
void RM_M2006_Init(MotorTypeDef *motor, uint8_t id);

/// @brief 初始化M3508电机实例
/// @param motor 电机结构体指针
/// @param id 电机ID (1-4)
void RM_M3508_Init(MotorTypeDef *motor, uint8_t id);

/// @brief 初始化GM6020电机实例
/// @param motor 电机结构体指针
/// @param id 电机ID (1-7)
void RM_GM6020_Init(MotorTypeDef *motor, uint8_t id);

/*============================== 电机配置函数 ==============================*/

/// @brief 设置RM电机配置
/// @param motor 电机结构体指针
/// @param config 配置结构体指针
void RM_Motor_SetConfig(MotorTypeDef *motor, const RM_MotorConfig_t *config);

/// @brief 配置RM电机串级PID
/// @param motor 电机结构体指针
/// @param outer_p/i/d/f 外环PID参数
/// @param inner_p/i/d/f 内环PID参数
/// @param outer_max_out 外环输出上限
/// @param outer_min_out 外环输出下限
/// @param outer_max_iout 外环积分限幅
/// @param inner_max_out 内环输出上限
/// @param inner_min_out 内环输出下限
/// @param inner_max_iout 内环积分限幅
void RM_Motor_SetCascadePID(MotorTypeDef *motor,
                            float outer_p, float outer_i, float outer_d, float outer_f,
                            float inner_p, float inner_i, float inner_d, float inner_f,
                            float outer_max_out, float outer_min_out, float outer_max_iout,
                            float inner_max_out, float inner_min_out, float inner_max_iout);

/// @brief 配置RM电机单环速度PID
/// @param motor 电机结构体指针
/// @param mode PID模式 (PID_POSITION 或 PID_DELTA)
/// @param p/i/d/f PID参数
/// @param max_out 输出上限
/// @param min_out 输出下限
/// @param max_iout 积分限幅
void RM_Motor_SetSpeedPID(MotorTypeDef *motor, PID_MODE_e mode,
                          float p, float i, float d, float f,
                          float max_out, float min_out, float max_iout);

/*============================== 原有函数声明（保留兼容） ==============================*/

/// @brief 设置对应RM电机的发送数据
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param data 发送数据指针（必须8字节）
void RM_MotorSetTxData(can_motor_cfg motor_cfg, uint8_t *data);

/// @brief RM电机接收数据解算
/// @param motor 电机结构体指针
/// @note motor->motor_data.solved_data[0]: 单圈角度(°)
/// @note motor->motor_data.solved_data[1]: 速度(rpm)
/// @note motor->motor_data.solved_data[2]: 电流(A)
/// @note motor->motor_data.solved_data[3]: 累计角度(°) - 用于位置闭环
/// @note motor->motor_data.solved_data[4]: 速度(rad/s) - 弧度制速度
void RM_MOTOR_CALCU(MotorTypeDef *motor);

/// @brief 重置电机零点（当前位置设为零点）
/// @param motor 电机结构体指针
void RM_Motor_Reset_Zero(MotorTypeDef *motor);

/// @brief 重置所有电机状态（用于重新初始化）
void RM_Motor_Reset_All(void);

/// @brief 用于设置发送RM电机数据
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param TargetCurrent 电流大小
void RmMotorSendCfg(can_motor_cfg motor_cfg, int16_t TargetCurrent);

/// @brief 去除电机偏移对电机目标数值影响（绝对式）
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param Target 电机目标数值（相对于首次调用时位置的偏移）
/// @param ChangeVel 是否改变目标数值
/// @return 修正后的电机目标数值（绝对位置）
/// @note 只有到达上次目标位置后才允许更新新目标
float RmMotorRemoveBias(can_motor_cfg motor_cfg, float Target, bool ChangeVel);

/// @brief RM电机PID计算并发送
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param target 目标值，单环时候为速度，串级为位置
/// @retval 无
void RmMotorPID_Calc(can_motor_cfg motor_cfg, float target);

/// @brief RM电机PID计算（使用电机结构体）
/// @param motor 电机结构体指针
/// @param target 目标值
/// @return PID输出值
float RM_Motor_PID_Calc(can_motor_cfg motor_cfg, float target);

#endif
