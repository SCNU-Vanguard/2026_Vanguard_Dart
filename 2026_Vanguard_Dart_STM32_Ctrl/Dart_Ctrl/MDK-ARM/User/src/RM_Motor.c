/*****************************************************
 * RM电机（大疆电机）控制模块
 * 适配M3508、M2006、M6020等系列电机
 * --------------------------------------------------
 * RM电机说明：
 * 大疆电机的注册同一系列电调最多是4个
 * 同一CAN总线上挂载的大疆电调（只有一个系列电调最多是8个）
 * 如果是C610和C620都有，则二者均为4个即为最大值
 ****************************************************/

#include "RM_Motor.h"
#include "CanMotor.h"
#include <stdbool.h>

// 外部引用电机管理器
extern MotorManager_t MotorManager;

#define CtrlMotorLen 8 // 电机控制报文长度默认给8
#define SingleMotorTest 1

/**********************************************************发送电机数据专用函数**********************************************************************/

/// @brief 大疆电机用这个(RM电机使用电流控制)
/// @param st 这个指针瞎几把给就好了，只要是RM电机的就行
/// @return 1->发送成功 | 0->发送失败
uint8_t RM_MotorSendControl(MotorTypeDef *st)
{
    assert_param(st != NULL);
    if (st == NULL)
    {
        return 0; // 或返回错误码
    }

    // 将RM电机的数据进行拼接
    for (uint8_t a = 0; a < g_RM_MOTOR_NUM; a++)
    {
        for (uint8_t b = 0; b < 2; b++)
        {
            MotorManager.RM_MOTOR_DATA_ARRAY[2 * a + b] = MotorManager.MotorList[a].SendMotorData[b];
        }
    }

    if (CAN_SendData(&hcan1, &(st->g_TxHeader), (uint8_t *)MotorManager.RM_MOTOR_DATA_ARRAY)) // 数据丢失
    {
        // 发送成功
        return 1;
    }
    else
    {
        return 0;
    }
}

/// @brief 设置对应电机的参数
/// @param motor_id 第几个大疆电机
/// @param data 发送数据指针（必须8字节），似乎可以封装一下发送缓冲区
/// @todo 分离一下，这里可以只设置发送函数，将发送任务放在其他地方（比如CAN通信发送任务）
void RM_MotorSetTxData(uint8_t motor_id, int8_t *data)
{
    assert_param(data != NULL);
    if (data == NULL)
    {
        return; // 返回错误码
    }
    memset(MotorManager.MotorList[motor_id - 1].SendMotorData, 0x00, CtrlMotorLen);               // 初始化
    memcpy(MotorManager.MotorList[motor_id - 1].SendMotorData, data, CtrlMotorLen);               // Copy数据
    MotorManager.MotorList[motor_id - 1].SendMotorControl(&MotorManager.MotorList[motor_id - 1]); // 调用发送函数
}

/**********************************************************电机数据接收解算专用函数******************************************************************/

/**************************************
 * RoboMaster电机返回数据
 * 长度 : 8字节（固定）
 * 数据说明:
 * note : 默认前一个是高字节，后一个是低字节
 * Data[0~1] : 电机转角高低字节
 * Data[2~3] : 电机转速高低字节
 * Data[4~5] : 电机力矩高低字节
 * Data[6~7] : 保留(C610电调)
 * Data[6] : 电机温度(C620电调)
 * Data[7] : 保留
 *************************************/

/// @brief RM电机接收数据解算
/// @param motor_id_num 大疆电机的id号（无需管其他品牌的电机）
/// @param ReceiveData 接收到的数据数组
/// @param solved_data 解算后的数据数组（至少3个float）
/// @note solved_data[0]: 角度(°), solved_data[1]: 速度(rpm), solved_data[2]: 电流(A)
void RM_MOTOR_CALCU(uint8_t motor_id_num, int8_t *ReceiveData, float *solved_data)
{
    // 对得到的数据进行移位
    int16_t RM_MOTOR_DATA_ROTARY_ANGLE = (((uint16_t)ReceiveData[0]) << 8) | ReceiveData[1];
    int16_t RM_MOTOR_DATA_ROTARY_SPEED = (((uint16_t)ReceiveData[2]) << 8) | ReceiveData[3];
    int16_t RM_MOTOR_DATA_ROTARY_TORQUE = (((uint16_t)ReceiveData[4]) << 8) | ReceiveData[5];

    // C620电调温度
    // int8_t C620Temp = ReceiveData[6];

    solved_data[0] = RM_MOTOR_DATA_ROTARY_ANGLE / 8190.00f * 360.00f; // 单位为°
    solved_data[1] = RM_MOTOR_DATA_ROTARY_SPEED;                      // 单位为rpm
    solved_data[2] = RM_MOTOR_DATA_ROTARY_TORQUE / 16384.0f * 20;     // 单位为A，实际上是转矩电流
    // solved_data[4] = C620Temp; // 单位为℃
}

/**********************************************************暴露接口，下面是外部一般用于调用的函数******************************************************/

/****************************************************
 * 函数名： RmMotorSendCfg
 * 作用：用于设置发送RM电机数据
 * 参数：motor_id：RM电机ID
 * 参数：TargetCurrent：电流大小
 * 返回值：无
 ****************************************************/
void RmMotorSendCfg(uint8_t motor_id, int16_t TargetCurrent)
{
    // 这里是建议只为8位数组，两位也行（前两位才是发送的电流的高低位）
    int8_t data[2] = {0};
    data[0] = TargetCurrent >> 8;
    data[1] = (int8_t)TargetCurrent;
    RM_MotorSetTxData(motor_id, data);
}

/**********************************************************电机初始化专用函数************************************************************************/

/// @brief  测试单个RM电机
/// @param  无
/// @note   仅供测试使用
/// @return 无
void RmTestMotorSingleRegister(void)
{
    // 按照需求更改参数
    MotorManager.MotorList[SingleMotorTest - 1].MotorID = SingleMotorTest;
    MotorManager.MotorList[SingleMotorTest - 1].MotorBand = RM_MOTOR_BAND;
    MotorManager.MotorList[SingleMotorTest - 1].SendMotorControl = RM_MotorSendControl;
    MotorManager.registered_count = 1;

    // CAN报文头配置在CanMotor.c中的CanRegisterMotorCfg函数完成
}
