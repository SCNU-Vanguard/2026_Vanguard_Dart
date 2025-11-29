#ifndef __RM_MOTOR_H_
#define __RM_MOTOR_H_

#include "main.h"
#include "bsp_can.h"
#include <stdlib.h>
#include <string.h>
#include "CanMotor.h"

/*-------------------------RM电机的电调的偏移ID----------------------------------------------*/
#define g_RM_MOTOR_BIAS_ADDR 0x200
#define g_RM_MOTOR_NUM 2

#define RM_TestUse 1U

// 8191为机械角度范围
// -16384->0->16384反馈的电流范围
// 力矩的反馈不知
typedef struct __attribute__((packed))
{
    // 电流和反馈电流
    volatile uint16_t LastCurrent;
    volatile uint16_t NowCurrent;

    // 力矩和反馈力矩
    volatile uint16_t LastTorque;
    volatile uint16_t NowTorque;

    // 角度和上一次的角度
    volatile float LastAngle;
    volatile float NowAngle;
} MotorFbData;

/*********************************************************函数声明***************************************************************/

/// @brief 大疆电机发送控制函数(RM电机使用电流控制)
/// @param st 电机结构体指针
/// @return 1->发送成功 | 0->发送失败
uint8_t RM_MotorSendControl(MotorTypeDef *st);

/// @brief 设置对应RM电机的发送数据
/// @param motor_id 第几个大疆电机
/// @param data 发送数据指针（必须8字节）
void RM_MotorSetTxData(uint8_t motor_id, int8_t *data);

/// @brief RM电机接收数据解算
/// @param motor_id_num 大疆电机的id号（无需管其他不同品牌的电机）
/// @param ReceiveData 接收到的数据数组
/// @param solved_data 解算后的数据数组（至少3个float）
void RM_MOTOR_CALCU(uint8_t motor_id_num, int8_t *ReceiveData, float *solved_data);

/// @brief 用于设置发送RM电机数据
/// @param motor_id RM电机ID
/// @param TargetCurrent 电流大小
void RmMotorSendCfg(uint8_t motor_id, int16_t TargetCurrent);

/// @brief 测试单个RM电机注册函数
/// @param 无
/// @note 仅供测试使用
/// @return 无
void RmTestMotorSingleRegister(void);

#endif
