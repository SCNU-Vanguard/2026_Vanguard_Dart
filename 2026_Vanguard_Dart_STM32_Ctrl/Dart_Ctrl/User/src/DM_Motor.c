/*****************************************************
 * DM电机（达妙电机）控制模块
 * 适配H35系列电机（DM3510、DM4310等）
 * --------------------------------------------------
 * DM电机说明：
 * 支持MIT模式、位置速度模式、速度模式、PVT模式
 * 目前主要使用MIT模式进行控制
 * todo:当前文件存在阻塞延时函数，替换成定时器中断解决延时或者vTaskDelay，现在先使用vTaskDelay
 ****************************************************/

#include "DM_Motor.h"
#include "RM_Motor.h"
#include "CanMotor.h"
#include "bsp_dwt.h"
#include <stdbool.h>
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

// 外部引用电机管理器
extern MotorManager_t MotorManager;
extern float dm_motor_solved_data[];

#define CtrlMotorLen 8 // 电机控制报文长度默认给8
#define SingleMotorTest 1

// DM电机控制标志和数据
const uint8_t DM_MOTOR_ENABLE[8] = {0xFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFC};  // DM电机使能控制数据帧
const uint8_t DM_MOTOR_DISABLE[8] = {0xFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFD}; // DM电机失能控制数据值
static int16_t KP_RESULT = 0;
static int16_t KD_RESULT = 0;
static int16_t Torque_ff = 0;
static bool DM_ENABLE_ARR[2] = {false};

/************************************************************************
 * @brief:       uint_to_float: 无符号整数转换为浮点数函数（通用版）
 * @param:       x_int: 待转换的无符号整数
 * @param:       x_min: 范围最小值
 * @param:       x_max: 范围最大值
 * @param:       bits:  位数
 * @retval:      浮点数结果
 ************************************************************************/
static inline float uint_to_float_generic(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

static float uint_to_float(float x_int, DM_DATA DataMode)
{
    switch (DataMode)
    {
    case DM_POS:
        return uint_to_float_generic((int)x_int, g_DM_LOWER_LIMITATION_POS, g_DM_UPPER_LIMITATION_POS, DM_POS_BIT);
    case DM_VEL:
        return uint_to_float_generic((int)x_int, g_DM_LOWER_LIMITATION_VEL, g_DM_UPPER_LIMITATION_VEL, DM_VEL_BIT);
    case DM_KD:
        return uint_to_float_generic((int)x_int, g_DM_LOWER_LIMITATION_KD, g_DM_UPPER_LIMITATION_KD, DM_KD_BIT);
    case DM_KP:
        return uint_to_float_generic((int)x_int, g_DM_LOWER_LIMITATION_KP, g_DM_UPPER_LIMITATION_KP, DM_KP_BIT);
    case DM_TORQUE:
        return uint_to_float_generic((int)x_int, g_DM_LOWER_LIMITATION_TORQUE, g_DM_UPPER_LIMITATION_TORQUE, DM_TORQUE_BIT);
    default:
        return 0.0f;
    }
}

/************************************************************************
 * @brief:       float_to_uint: 浮点数转换为无符号整数函数（通用版）
 * @param:       x_float: 待转换的浮点数
 * @param:       x_min: 范围最小值
 * @param:       x_max: 范围最大值
 * @param:       bits:  位数
 * @retval:      无符号整数结果
 ************************************************************************/
static inline int float_to_uint_generic(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    if (x_float < x_min)
        x_float = x_min;
    if (x_float > x_max)
        x_float = x_max;
    return (int)((x_float - x_min) * ((float)((1 << bits) - 1)) / span);
}

static int float_to_uint(float x_float, DM_DATA DataMode)
{
    switch (DataMode)
    {
    case DM_POS:
        return float_to_uint_generic(x_float, g_DM_LOWER_LIMITATION_POS, g_DM_UPPER_LIMITATION_POS, DM_POS_BIT);
    case DM_VEL:
        return float_to_uint_generic(x_float, g_DM_LOWER_LIMITATION_VEL, g_DM_UPPER_LIMITATION_VEL, DM_VEL_BIT);
    case DM_KD:
        return float_to_uint_generic(x_float, g_DM_LOWER_LIMITATION_KD, g_DM_UPPER_LIMITATION_KD, DM_KD_BIT);
    case DM_KP:
        return float_to_uint_generic(x_float, g_DM_LOWER_LIMITATION_KP, g_DM_UPPER_LIMITATION_KP, DM_KP_BIT);
    case DM_TORQUE:
        return float_to_uint_generic(x_float, g_DM_LOWER_LIMITATION_TORQUE, g_DM_UPPER_LIMITATION_TORQUE, DM_TORQUE_BIT);
    default:
        return 0;
    }
}

/**********************************************************发送电机数据专用函数**********************************************************************/

/// @brief 用于失能达妙电机
/// @param DM_MOTOR_ID 达妙电机的电机号（无需管是否有大疆电机，只需要知道这是第几个达妙电机即可）
/// @return 1：发送成功，0：发送失败
uint8_t DM_MotorDisable(uint8_t DM_MOTOR_ID)
{
#if DM_TestUse
    if (CAN_SendData(&hcan1, &(MotorManager.MotorList[DM_MOTOR_ID - 1].g_TxHeader), (uint8_t *)DM_MOTOR_DISABLE))
    {
        DM_ENABLE_ARR[DM_MOTOR_ID - 1] = false;
        // DWT_Delay_us(200); // 一个完整的8字节标准数据帧是108位，这里用一个200us的延时足以搞定
        vTaskDelay(1);
        return 1;
    }
#else
    if (CAN_SendData(&hcan1, &(MotorManager.MotorList[DM_MOTOR_ID + g_RM_MOTOR_NUM - 1].g_TxHeader), (uint8_t *)DM_MOTOR_DISABLE))
    {
        DM_ENABLE_ARR[DM_MOTOR_ID - 1] = false;
        // DWT_Delay_us(200);
        vTaskDelay(1);
        return 1;
    }
#endif
    return 0;
}

/// @brief 用于使能达妙电机
/// @param DM_MOTOR_ID 达妙电机的电机号（无需管是否有大疆电机，只需要知道这是第几个达妙电机即可）
/// @return 1：发送成功，0：发送失败
static uint8_t DM_MotorEnable(uint8_t DM_MOTOR_ID)
{
#if DM_TestUse
    if (CAN_SendData(&hcan1, &(MotorManager.MotorList[DM_MOTOR_ID - 1].g_TxHeader), (uint8_t *)DM_MOTOR_ENABLE))
    {
        DM_ENABLE_ARR[DM_MOTOR_ID - 1] = true;
        // DWT_Delay_us(200);
        vTaskDelay(1);
        return 1;
    }
#else
    if (CAN_SendData(&hcan1, &(MotorManager.MotorList[DM_MOTOR_ID + g_RM_MOTOR_NUM - 1].g_TxHeader), (uint8_t *)DM_MOTOR_ENABLE))
    {
        DM_ENABLE_ARR[DM_MOTOR_ID - 1] = true;
        // DWT_Delay_us(200);
        vTaskDelay(1);
        return 1;
    }
#endif
    return 0;
}

/// @brief 用于控制达妙电机
/// @param st 要控制的达妙电机的结构体
/// @return 1：发送成功，0：发送失败
/// @todo 目前发送逻辑有问题，需要在连续发送过程中添加一定的延时函数保证发送不会吞帧
uint8_t DM_MotorSendControl(MotorTypeDef *st)
{
    assert_param(st != NULL);
    if (st == NULL)
    {
        return 0; // 返回错误码
    }
    if (!DM_ENABLE_ARR[(st->MotorID) - 1])
    {
        DM_MotorEnable(st->MotorID);
    }
    if (CAN_SendData(&hcan1, &(st->g_TxHeader), (uint8_t *)st->SendMotorData))
    {
        // 发送成功
        return 1;
    }
    else
    {
        return 0;
    }
}

/// @brief 设置达妙电机发送的数据
/// @param motor_id 达妙电机ID
/// @param data 数据所在数组的指针
void DM_MotorSetTxData(uint8_t motor_id, uint8_t *data)
{
// MIT模式
#ifdef DM_MIT_MODE
    // 7F FF 7F F0 00 00 08 28 -> 位置0 速度0.0 KP为0, KD为0, 转矩0.20
    // 按道理来说这个KP和KD都是固定的，这个转矩是前馈的量，可能固定也可能PID？是固定的前馈
    assert_param(data != NULL);
    if (data == NULL)
    {
        Error_Handler(); // 返回错误码
    }

    // 这里是要整个都清理一遍
    memset(MotorManager.MotorList[motor_id - 1 + g_DM_MOTOR_BIAS_ADDR_TXID].SendMotorData, 0x00, CtrlMotorLen);
    memcpy(MotorManager.MotorList[motor_id - 1 + g_DM_MOTOR_BIAS_ADDR_TXID].SendMotorData, data, CtrlMotorLen);
    MotorManager.MotorList[motor_id - 1 + g_DM_MOTOR_BIAS_ADDR_TXID].SendMotorControl(&MotorManager.MotorList[motor_id - 1 + g_DM_MOTOR_BIAS_ADDR_TXID]); // 调用发送函数
#endif

// 位置速度模式
#ifdef DM_LOCATION_SPEED_MODE
// 有心者自己补充
#endif

// 速度模式
#ifdef DM_SPEED_MODE
// 有心者自己补充
#endif

// PVT模式
#ifdef DM_PVT_MODE
// 有心者自己补充
#endif
}

/**********************************************************电机数据接收解算专用函数******************************************************************/

/************************************************************************
 * 达妙电机反馈帧(ID是MasterID)
 * 8Byte:
 * ID + ERR<<4 | POS[15 :  8] | POS[7 :  0] | VEL[11 :  4] | VEL[3  :  0]  T[11 :  8] | T[7  :  0] | T_Mos | T_Rotor
 * POS电机位置信息
 * VEL电机速度信息
 * T电机扭矩信息
 * T_MOS表示驱动上MOS管上的均温，单位℃
 * T_ROTOR表示电机内部线圈的均温，单位℃
 ***********************************************************************/

/*====================  静态变量（速度滤波）  ====================*/
#define DM_SPEED_FILTER_COEF 0.8f    // 速度滤波系数（0~1，越大越平滑）
static float last_velocity[4] = {0}; // 上次速度值（最多4个DM电机）

/// @brief DM电机的解算（优化版）
/// @param motor_id_num DM电机的id号（0~3）
/// @param ReceiveData 接收到的数据数组
/// @param solved_data 解算后的数据数组（至少5个float）
/// @note solved_data[0]: 位置(rad), solved_data[1]: 速度(rad/s), solved_data[2]: 力矩(N·m)
///       solved_data[3]: MOS温度(℃), solved_data[4]: 转子温度(℃)
void DM_MOTOR_CALCU(uint8_t motor_id_num, uint8_t *ReceiveData, float *solved_data)
{
    // =============== 1. 数据解析（位操作压缩） ===============
    uint16_t pos_raw = ((uint16_t)ReceiveData[1] << 8) | ReceiveData[2];
    uint16_t vel_raw = ((uint16_t)ReceiveData[3] << 4) | (ReceiveData[4] >> 4);
    uint16_t tor_raw = ((ReceiveData[4] & 0x0F) << 8) | ReceiveData[5];

    // =============== 2. 数据转换（使用通用函数） ===============
#ifdef DM_USE_4310 // 4310电机使用MIT协议范围
    solved_data[0] = uint_to_float_generic(pos_raw, g_DM_LOWER_LIMITATION_POS, g_DM_UPPER_LIMITATION_POS, 16);
    float vel_new = uint_to_float_generic(vel_raw, g_DM_LOWER_LIMITATION_VEL, g_DM_UPPER_LIMITATION_VEL, 12);
    solved_data[2] = uint_to_float_generic(tor_raw, g_DM_LOWER_LIMITATION_TORQUE, g_DM_UPPER_LIMITATION_TORQUE, 12);
#else // 3519等电机
    solved_data[0] = pos_raw / 8192.0f * 360.0f; // 位置(°)
    float vel_new = (float)(int16_t)vel_raw;     // 速度原始值
    solved_data[2] = tor_raw / 16384.0f * 20.0f; // 力矩(N·m)
#endif

    // =============== 3. 速度滤波 ===============
    if (motor_id_num < 4)
    {
        solved_data[1] = DM_SPEED_FILTER_COEF * last_velocity[motor_id_num] + (1.0f - DM_SPEED_FILTER_COEF) * vel_new;
        last_velocity[motor_id_num] = solved_data[1];
    }
    else
    {
        solved_data[1] = vel_new;
    }

    // =============== 4. 温度 ===============
    solved_data[3] = (float)ReceiveData[6]; // MOS温度
    solved_data[4] = (float)ReceiveData[7]; // 转子温度
}

/**********************************************************暴露接口，下面是外部一般用于调用的函数******************************************************/

/****************************************************
 * 函数名： DmMotorSendCfg
 * 作用：用于设置发送DM电机数据
 * 参数：motor_id ：DM电机的电机号
 * 参数：TargetPos：目标位置 angle
 * 参数：TargetVel：目标速度 rad/s
 * 返回值：无
 ****************************************************/
void DmMotorSendCfg(uint8_t motor_id, float TargetPos, float TargetVel)
{
    KP_RESULT = float_to_uint(g_DM_KP, DM_KP);
    KD_RESULT = float_to_uint(g_DM_KD, DM_KD);
    Torque_ff = float_to_uint(g_DM_Compensating_Torque, DM_TORQUE); // float 转 uint
    int16_t Pos_des = float_to_uint((float)TargetPos, DM_POS);
    int16_t Vel_des = float_to_uint((float)TargetVel, DM_VEL);

    static uint8_t data[8] = {0x00};
    data[0] = Pos_des >> 8;
    data[1] = (uint8_t)Pos_des;
    data[2] = Vel_des >> 4;
    data[3] = ((Vel_des & 0x000F) << 4) | ((KP_RESULT & 0x0F00) >> 8); // 当更改速度限幅的时候可能要更改这里的移位逻辑
    data[4] = KP_RESULT;
    data[5] = KD_RESULT >> 4;
    data[6] = ((KD_RESULT & 0x000F) << 4) | ((Torque_ff & 0x0F00) >> 8);
    data[7] = Torque_ff;
    DM_MotorSetTxData(motor_id, data);
}

/**********************************************************电机初始化专用函数************************************************************************/

/// @brief  测试单个DM电机
/// @param  无
/// @note   仅供测试使用
/// @return 无
void DmTestMotorSingleRegister(void)
{
    // 按照需求更改参数
    MotorManager.MotorList[SingleMotorTest - 1].MotorID = SingleMotorTest;
    MotorManager.MotorList[SingleMotorTest - 1].MotorBand = DM_MOTOR_BAND;
    MotorManager.MotorList[SingleMotorTest - 1].SendMotorControl = DM_MotorSendControl;
    MotorManager.registered_count = 1;

    // CAN报文头配置在CanMotor.c中的CanRegisterMotorCfg函数完成
}

/// @brief RM电机输出
void DmMotorPID_Calc(void)
{
    // PID数据输出
}
