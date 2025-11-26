/*****************************************************
 * DM电机（达妙电机）控制模块
 * 适配H35系列电机（DM3510、DM4310等）
 * --------------------------------------------------
 * DM电机说明：
 * 支持MIT模式、位置速度模式、速度模式、PVT模式
 * 目前主要使用MIT模式进行控制
 ****************************************************/

#include "DM_Motor.h"
#include "RM_Motor.h"
#include "CanMotor.h"
#include "bsp_dwt.h"
#include <stdbool.h>

// 外部引用电机管理器
extern MotorManager_t MotorManager;

#define CtrlMotorLen 8 // 电机控制报文长度默认给8
#define SingleMotorTest 1

// DM电机控制标志和数据
static bool g_DM_CONTROL_FLAG = false;                                                // DM电机的使能允许控制标志位
const uint8_t DM_MOTOR_ENABLE[8] = {0xFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFC};  // DM电机使能控制数据帧
const uint8_t DM_MOTOR_DISABLE[8] = {0xFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFF, 0XFD}; // DM电机失能控制数据值
static int16_t KP_RESULT = 0;
static int16_t KD_RESULT = 0;
static int16_t Torque_ff = 0;
static bool DM_ENABLE_ARR[2] = {false};

/************************************************************************
 * @brief:       uint_to_float: 无符号整数转换为浮点数函数
 * @param:       x_int: 待转换的无符号整数
 * @param:       DataMode: 转换的数据字节类型
 * @retval:      浮点数结果(再转换为整数，方便发送)
 * @details:     将给定的无符号整数 x_int 在指定范围 [ x_min, x_max ] 内进行线性映射，映射结果为一个浮点数
 ************************************************************************/
static float uint_to_float(float x_int, DM_DATA DataMode)
{
    /* converts unsigned int to float, given range and number of bits */
    float Result = 0.0f;
    switch (DataMode)
    {
    case DM_POS:
        Result = ((float)x_int) * g_DM_POS_DIFFERENCE / ((float)((1 << DM_POS_BIT) - 1)) + g_DM_LOWER_LIMITATION_POS;
        break;
    case DM_VEL:
        Result = ((float)x_int) * g_DM_VEL_DIFFERENCE / ((float)((1 << DM_VEL_BIT) - 1)) + g_DM_LOWER_LIMITATION_VEL;
        break;
    case DM_KD:
        Result = ((float)x_int) * g_DM_KD_DIFFERENCE / ((float)((1 << DM_KD_BIT) - 1)) + g_DM_LOWER_LIMITATION_KD;
        break;
    case DM_KP:
        Result = ((float)x_int) * g_DM_KP_DIFFERENCE / ((float)((1 << DM_KP_BIT) - 1)) + g_DM_LOWER_LIMITATION_KP;
        break;
    case DM_TORQUE:
        Result = ((float)x_int) * g_DM_TORQUE_DIFFERENCE / ((float)((1 << DM_TORQUE_BIT) - 1)) + g_DM_LOWER_LIMITATION_TORQUE;
        break;
    default:
        break;
    }
    return Result;
}

/************************************************************************
 * @brief:       float_to_uint: 浮点数转换为无符号整数函数
 * @param:       x_int: 待转换的无符号整数
 * @param:       DataMode: 转换的数据字节类型
 * @retval:      无符号整数结果
 * @details:     将给定的浮点数 x 在指定范围 [ x_min, x_max ] 内进行线性映射，映射结果为一个无符号整数
 ************************************************************************/
static int float_to_uint(float x_float, DM_DATA DataMode)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    int16_t Result = 0.0f;
    switch (DataMode)
    {
    case DM_POS:
        Result = (int32_t)((x_float - g_DM_LOWER_LIMITATION_POS) * ((float)((1 << DM_POS_BIT) - 1)) / g_DM_POS_DIFFERENCE);
        break;
    case DM_VEL:
        Result = (int32_t)((x_float - g_DM_LOWER_LIMITATION_VEL) * ((float)((1 << DM_VEL_BIT) - 1)) / g_DM_VEL_DIFFERENCE);
        break;
    case DM_KD:
        Result = (int32_t)((x_float - g_DM_LOWER_LIMITATION_KD) * ((float)((1 << DM_KD_BIT) - 1)) / g_DM_KD_DIFFERENCE);
        break;
    case DM_KP:
        Result = (int32_t)((x_float - g_DM_LOWER_LIMITATION_KP) * ((float)((1 << DM_KP_BIT) - 1)) / g_DM_KP_DIFFERENCE);
        break;
    case DM_TORQUE:
        Result = (int32_t)((x_float - g_DM_LOWER_LIMITATION_TORQUE) * ((float)((1 << DM_TORQUE_BIT) - 1)) / g_DM_TORQUE_DIFFERENCE);
        break;
    default:
        break;
    }
    return Result;
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
        DWT_Delay_us(200); // 一个完整的8字节标准数据帧是108位，这里用一个200us的延时足以搞定
        return 1;
    }
#else
    if (CAN_SendData(&hcan1, &(MotorManager.MotorList[DM_MOTOR_ID + g_RM_MOTOR_NUM - 1].g_TxHeader), (uint8_t *)DM_MOTOR_DISABLE))
    {
        DM_ENABLE_ARR[DM_MOTOR_ID - 1] = false;
        DWT_Delay_us(200);
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
        DWT_Delay_us(200);
        return 1;
    }
#else
    if (CAN_SendData(&hcan1, &(MotorManager.MotorList[DM_MOTOR_ID + g_RM_MOTOR_NUM - 1].g_TxHeader), (uint8_t *)DM_MOTOR_ENABLE))
    {
        DM_ENABLE_ARR[DM_MOTOR_ID - 1] = true;
        DWT_Delay_us(200);
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
void DM_MotorSetTxData(uint8_t motor_id, int8_t *data)
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

/// @brief DM电机的解算
/// @param motor_id_num DM电机的id号（无需管理其他品牌型号的电机）
/// @param ReceiveData 接收到的数据数组
/// @param solved_data 解算后的数据数组（至少5个float）
/// @note solved_data[0]: 位置(°), solved_data[1]: 速度(rad/s), solved_data[2]: 力矩(N*m)
///       solved_data[3]: MOS温度(℃), solved_data[4]: 转子温度(℃)
void DM_MOTOR_CALCU(uint8_t motor_id_num, int8_t *ReceiveData, float *solved_data)
{
    // 对得到的数据进行移位
    int16_t DM_MOTOR_STATE_CODE = ((((uint16_t)ReceiveData[0]) - (motor_id_num + g_DM_MOTOR_BIAS_ADDR_RXID)) >> 4);

    int16_t DM_MOTOR_DATA_ROTARY_POSTION = (((uint16_t)ReceiveData[1]) << 8) | ((uint16_t)ReceiveData[2]);

    int16_t DM_MOTOR_DATA_ROTARY_SPEED = (((uint16_t)ReceiveData[3]) << 4) | ((uint16_t)ReceiveData[4] >> 4);

    int16_t DM_MOTOR_DATA_ROTARY_TORQUE = (((uint16_t)ReceiveData[4]) & 0x000F) | ((uint16_t)ReceiveData[5]);

    int16_t DM_MOTOR_DATA_TEMPRATURE_MOS = (uint16_t)ReceiveData[6];

    int16_t DM_MOTOR_DATA_TEMPRATURE_ROTOR = (uint16_t)ReceiveData[7];

    // 解算数据
    // 0：解算后的位置数据（实际上是角度）
    // 1：解算后的速度
    // 2：解算后的力矩
    // 3：解算后的MOS管平均温度数据
    // 4：解算后的线圈平均温度数据

    // 这个是4310的解算函数，这里飞镖也会用到
    // solved_data[0] = uint_to_float(DM_MOTOR_DATA_ROTARY_POSTION, DM_POS);
    // solved_data[1] = uint_to_float(DM_MOTOR_DATA_ROTARY_SPEED, DM_VEL);
    // solved_data[2] = uint_to_float(DM_MOTOR_DATA_ROTARY_TORQUE, DM_TORQUE);
    // solved_data[3] = (float)DM_MOTOR_DATA_TEMPRATURE_MOS;
    // solved_data[4] = (float)DM_MOTOR_DATA_TEMPRATURE_ROTOR;

    // 这个是3519的解算
    solved_data[0] = DM_MOTOR_DATA_ROTARY_POSTION / 8192.0f * 360.0f;           // 单位为°
    solved_data[1] = (float)DM_MOTOR_DATA_ROTARY_SPEED;                         // 单位为rad/s
    solved_data[2] = ((float)DM_MOTOR_DATA_ROTARY_TORQUE) / (16384.0f / 20.0f); // 单位为N*m
    solved_data[3] = (float)(DM_MOTOR_DATA_TEMPRATURE_MOS);                     // 单位为℃
    solved_data[4] = (float)(DM_MOTOR_DATA_TEMPRATURE_ROTOR);                   // 单位为℃
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

    static int8_t data[8] = {0x00};
    data[0] = Pos_des >> 8;
    data[1] = (int8_t)Pos_des;
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
