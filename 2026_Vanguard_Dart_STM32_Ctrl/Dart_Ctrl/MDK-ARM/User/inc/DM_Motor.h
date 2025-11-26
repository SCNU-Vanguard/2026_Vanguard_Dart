#ifndef __DM_MOTOR_H_
#define __DM_MOTOR_H_

#include "main.h"
#include "bsp_can.h"
#include "CanMotor.h"
#include <stdlib.h>
#include <string.h>

/*-------------------------DM电机模式以及模式下的偏移-----------------------------------------*/
#define DM_MIT_MODE 1U
#define DM_LOCATION_SPEED_MODE 0U
#define DM_SPEED_MODE 0U
#define DM_PVT_MODE 0U

#define DM_TestUse 0U

// 如果你需要使用其他模式只能在这里手动更改数值，现在这个定义是只可以使用MIT模式
#if DM_MIT_MODE
#define g_DM_MOTOR_BIAS_ADDR_TXID 0x000 // 达妙电机默认使用MIT模式，具体是MIT模式的哪个细分模式还等待敲定
#define g_DM_MOTOR_BIAS_ADDR_RXID 0x010
typedef enum
{
    DM_POS_BIT = 16,    // 保留两位小数
    DM_VEL_BIT = 12,    // 保留两位小数
    DM_TORQUE_BIT = 12, // 保留两位小数
    DM_KP_BIT = 12,     // 直接整数就可以了，默认的话
    DM_KD_BIT = 12,     // 默认下是需要两位小数
} DM_DATA_BIT;
#endif

#if DM_LOCATION_SPEED_MODE
#define g_DM_MOTOR_BIAS_ADDR_TXID 0x100 // 达妙电机使用位置速度模式 note:This mode is equaling to Cascade PID but don't have current limitation
#define g_DM_MOTOR_BIAS_ADDR_RXID 0x020
typedef enum
{
    DM_POS_BIT = 32,
    DM_VEL_BIT = 32,
} DM_DATA_BIT;
#endif

#if DM_SPEED_MODE
#define g_DM_MOTOR_BIAS_ADDR_TXID 0x200 // 达妙电机使用速度模式 note:This mode is equaling to PID but don't have current limitation
#define g_DM_MOTOR_BIAS_ADDR_RXID 0x030
typedef enum
{
    DM_VEL_BIT = 32
} DM_DATA_BIT;
#endif

#if DM_PVT_MODE
#define g_DM_MOTOR_BIAS_ADDR_TXID 0x300 // 达妙电机使用PVT模式 note:This mode is equaling to Cascade PID but have current limitation
#define g_DM_MOTOR_BIAS_ADDR_RXID 0x040
typedef enum
{
    DM_POS_BIT = 32,
    DM_VEL_BIT = 16,
    DM_CURRENT_BIT = 16,
} DM_DATA_BIT;
#endif

#define g_DM_MOTOR_NUM 2

// 限幅,这里给的是电机默认的数值
#define g_DM_UPPER_LIMITATION_KP 500.0 // hex:1F4
#define g_DM_LOWER_LIMITATION_KP 0.0
#define g_DM_UPPER_LIMITATION_KD 5.0 // hex:5.00
#define g_DM_LOWER_LIMITATION_KD 0.0
#define g_DM_UPPER_LIMITATION_POS 12.5 // hex:C.80
#define g_DM_LOWER_LIMITATION_POS -12.5
#define g_DM_UPPER_LIMITATION_TORQUE 10.0 // A.00
#define g_DM_LOWER_LIMITATION_TORQUE -10.0
#define g_DM_UPPER_LIMITATION_VEL 200.0 // C8.00
#define g_DM_LOWER_LIMITATION_VEL -200.0

// 差值,这里给的是电机默认的数值,这里的DIFFERENCE = UPPER_LIMITAION - LOWERLIMITATION
#define g_DM_KP_DIFFERENCE 500
#define g_DM_KD_DIFFERENCE 0.00
#define g_DM_VEL_DIFFERENCE 400.00
#define g_DM_POS_DIFFERENCE 25.00
#define g_DM_TORQUE_DIFFERENCE 20.00

#define g_DM_KP 0.00
#define g_DM_KD 0.00
#define g_DM_Compensating_Torque 0.20 // 达妙电机前馈力矩,这里只是一个固定的数,具体数值得上机再测

// 电机的KP、KD和Tff(前馈力矩)应该都是一个固定的量
typedef enum
{
    DM_POS = 0,
    DM_VEL,
    DM_KP,
    DM_KD,
    DM_TORQUE,
} DM_DATA;

/*********************************************************函数声明***************************************************************/

/// @brief 用于失能达妙电机
/// @param DM_MOTOR_ID 达妙电机的电机号（无需管是否有大疆电机，只需要知道这是第几个达妙电机即可）
/// @return 1：发送成功，0：发送失败
uint8_t DM_MotorDisable(uint8_t DM_MOTOR_ID);

/// @brief 用于控制达妙电机
/// @param st 要控制的达妙电机的结构体
/// @return 1：发送成功，0：发送失败
uint8_t DM_MotorSendControl(MotorTypeDef *st);

/// @brief 设置达妙电机发送的数据
/// @param motor_id 达妙电机ID
/// @param data 数据所在数组的指针
void DM_MotorSetTxData(uint8_t motor_id, int8_t *data);

/// @brief DM电机的解算
/// @param motor_id_num DM电机的id号（无需管理其他品牌型号的电机）
/// @param ReceiveData 接收到的数据数组
/// @param solved_data 解算后的数据数组（至少5个float）
void DM_MOTOR_CALCU(uint8_t motor_id_num, int8_t *ReceiveData, float *solved_data);

/// @brief 用于设置发送DM电机数据
/// @param motor_id DM电机的电机号
/// @param TargetPos 目标位置
/// @param TargetVel 目标速度
/// @note 不知道是不是默认保留一位小数的，可能是范围导致？
void DmMotorSendCfg(uint8_t motor_id, float TargetPos, float TargetVel);

/// @brief 测试单个DM电机注册函数
/// @param 无
/// @note 仅供测试使用
/// @return 无
void DmTestMotorSingleRegister(void);

#endif
