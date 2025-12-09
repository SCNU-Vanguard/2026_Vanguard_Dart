#ifndef __RM_MOTOR_H_
#define __RM_MOTOR_H_

#include "main.h"
#include "bsp_can.h"
#include <stdlib.h>
#include <string.h>
#include "CanMotor.h"

/*-------------------------RM电机的电调的偏移ID----------------------------------------------*/
#define g_RM_MOTOR_BIAS_ADDR 0x200
#define g_RM_MOTOR_BIAS_ADDR_3508 0x200
#define g_RM_MOTOR_BIAS_ADDR_2006 0x1FF
#define g_RM_MOTOR_BIAS_ADDR_6020 0x2FE
#define g_RM_MOTOR_NUM 2
#define g_RM_M3508_NUM 1
#define g_RM_M2006_NUM 1
#define g_RM_GM6020_NUM 0

#define RM_TestUse 1U

// 8192为机械角度范围
// -16384->0->16384反馈的电流范围
// 力矩的反馈不知
// typedef struct __attribute__((packed))
// {
//     // 电流和反馈电流
//     volatile uint16_t LastCurrent;
//     volatile uint16_t NowCurrent;

//     // 力矩和反馈力矩
//     volatile uint16_t LastTorque;
//     volatile uint16_t NowTorque;

//     // 角度和上一次的角度
//     volatile float LastAngle;
//     volatile float NowAngle;
// } MotorFbData;

/*********************************************************函数声明***************************************************************/

/// @brief 大疆电机发送控制函数(RM电机使用电流控制)
/// @param st 电机结构体指针
/// @return 1->发送成功 | 0->发送失败
uint8_t RM_MotorSendControl(MotorTypeDef *st);

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

/// @brief 去除电机偏移对电机目标数值影响
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param Target 电机目标数值
/// @return 修正后的电机目标数值
float RmMotorRemoveBias(can_motor_cfg motor_cfg, float Target);

/// @brief 测试单个RM电机注册函数
/// @param 无
/// @note 仅供测试使用
/// @return 无
void RmTestMotorSingleRegister(void);

/// @brief RM电机输出
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param target 目标值，单环时候为速度，串级为位置
/// @retval 无
void RmMotorPID_Calc(can_motor_cfg motor_cfg, float target);

#endif
