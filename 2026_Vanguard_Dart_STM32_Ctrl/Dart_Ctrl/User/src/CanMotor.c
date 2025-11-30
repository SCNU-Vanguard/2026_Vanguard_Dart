/*****************************************************
 * 通用CAN电机管理层
 * 负责统一管理RM电机和DM电机
 * --------------------------------------------------
 * 功能说明：
 * - 电机注册和初始化
 * - CAN过滤器配置
 * - CAN接收中断回调处理
 * - 统一管理RM和DM电机的通信
 ****************************************************/

#include "CanMotor.h"
#include "RM_Motor.h"
#include "DM_Motor.h"
#include <stdbool.h>

// 电机管理表
MotorManager_t MotorManager = {0};
float dm_motor_solved_data[5] = {0.0f}; // 存储达妙电机解算后的数据
float rm_motor_solved_data[5] = {0.0f}; // 存储RM电机解算后的数据

/**********************************************************电机初始化专用函数************************************************************************/

/// @brief 注册电机的CAN通信信息
/// @param ptr  电机初始化指针
/// @note  包含报文头的初始化
/// @return 无
void CanRegisterMotorCfg(MotorTypeDef *ptr)
{
    assert_param(ptr != NULL);
    if (ptr == NULL)
    {
        return; // 或返回错误码
    }
    ptr->g_TxHeader.StdId = (ptr->MotorBand == RM_MOTOR_BAND) ? g_RM_MOTOR_BIAS_ADDR : g_DM_MOTOR_BIAS_ADDR_TXID + ptr->MotorID;
    ptr->g_TxHeader.IDE = CAN_ID_STD;   // 标准帧标识符
    ptr->g_TxHeader.RTR = CAN_RTR_DATA; // 数据帧
    ptr->g_TxHeader.DLC = CtrlMotorLen; // 数据长度
}

/// @brief  注册电机的信息
/// @param  无（按照已经配置的电机表注册）
/// @return 注册是否成功
/// @note   最后暴露的接口应该是更改电机发送数据和读取电机接收数据的接口，用户无需关心报文头等信息
/// @todo   达妙电机的PID可以不用调节，因为本身内置PID，但是这次调节的是前馈量，是为了速度响应更快，并且不影响期望位置和速度
void MotorRegister(void)
{
    // 手动申请接受头数组，似乎这里有点多余，也可以直接static一个接收头数组
    // malloc(sizeof(CAN_RxHeaderTypeDef) * g_CanMotorNum); // 申请g_CanMotorNum个接受头空间 g_CanMotorNum = 5

    // 注册电机应该包含电机的ID以及电机的发送地址和接收地址、发送数据存储地方
    // 注册RM电机
    // 发送之后需要memset()

    // 夹爪
    MotorManager.MotorList[RM_3508_GRIPPER - 1].MotorID = RM_3508_GRIPPER;
    MotorManager.MotorList[RM_3508_GRIPPER - 1].MotorBand = RM_MOTOR_BAND;
    MotorManager.MotorList[RM_3508_GRIPPER - 1].SendMotorControl = RM_MotorSendControl;
    MotorManager.MotorList[RM_3508_GRIPPER - 1].use_cascade = 1;
    // PID_Set_Coefficient(&MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid.inner, 0.0, 0.0, 0.0, 0.0); // 内环
    // PID_Set_Coefficient(&MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid.outer, 0.0, 0.0, 0.0, 0.0); // 外环
    // PID_Clear(&MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid.inner);                               // 初始化
    // PID_Clear(&MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid.outer);                               // 初始化
    // PID_Set_MaxOutput(&MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid.inner, 0.0f, 0.0f);
    // PID_Set_MaxOutput(&MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid.outer, 0.0f, 0.0f);
    CASCADE_PID_Init(&MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    // 扳机
    MotorManager.MotorList[RM_2006_TRIGGER - 1].MotorID = RM_2006_TRIGGER;
    MotorManager.MotorList[RM_2006_TRIGGER - 1].MotorBand = RM_MOTOR_BAND;
    MotorManager.MotorList[RM_2006_TRIGGER - 1].SendMotorControl = RM_MotorSendControl;
    // PID_Set_Coefficient(&MotorManager.MotorList[RM_2006_TRIGGER - 1].cascade_pid.inner, 0.0, 0.0, 0.0, 0.0); // 内环
    // PID_Set_Coefficient(&MotorManager.MotorList[RM_2006_TRIGGER - 1].cascade_pid.outer, 0.0, 0.0, 0.0, 0.0); // 外环
    // PID_Clear(&MotorManager.MotorList[RM_2006_TRIGGER - 1].cascade_pid.inner);                               // 初始化
    // PID_Clear(&MotorManager.MotorList[RM_2006_TRIGGER - 1].cascade_pid.outer);                               // 初始化
    // PID_Set_MaxOutput(&MotorManager.MotorList[RM_2006_TRIGGER - 1].cascade_pid.inner, 0.0f, 0.0f);
    // PID_Set_MaxOutput(&MotorManager.MotorList[RM_2006_TRIGGER - 1].cascade_pid.outer, 0.0f, 0.0f);
    CASCADE_PID_Init(&MotorManager.MotorList[RM_2006_TRIGGER - 1].cascade_pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    // 注册DM电机
    // 注意注册的DM电机发送和接收其实数据帧都不与RM电机冲突（在MIT模式、位置速度模式和PVT模式下，就完整通信帧而言）
    MotorManager.MotorList[DM_3510_STRENTH_LEFT - 1].MotorID = DM_3510_STRENTH_LEFT - g_RM_MOTOR_NUM;
    MotorManager.MotorList[DM_3510_STRENTH_LEFT - 1].MotorBand = DM_MOTOR_BAND;
    MotorManager.MotorList[DM_3510_STRENTH_LEFT - 1].SendMotorControl = DM_MotorSendControl;

    MotorManager.MotorList[DM_3510_STRENTH_RIGHT - 1].MotorID = DM_3510_STRENTH_RIGHT - g_RM_MOTOR_NUM;
    MotorManager.MotorList[DM_3510_STRENTH_RIGHT - 1].MotorBand = DM_MOTOR_BAND;
    MotorManager.MotorList[DM_3510_STRENTH_RIGHT - 1].SendMotorControl = DM_MotorSendControl;

    MotorManager.MotorList[DM_4310_YAW - 1].MotorID = DM_4310_YAW;
    MotorManager.MotorList[DM_4310_YAW - 1].MotorBand = DM_MOTOR_BAND;
    MotorManager.MotorList[DM_4310_YAW - 1].SendMotorControl = DM_MotorSendControl;

    MotorManager.registered_count = 5;

    for (uint8_t i = 0; i < MotorManager.registered_count; i++)
    {
        CanRegisterMotorCfg(&MotorManager.MotorList[i]);
    }

    // 用户需手动注册电机
    // 结构体存储发送的报文和接收的数据
}

/**********************************************************暴露接口,下面是外部一般用于调用的函数******************************************************/

/****************************************************
 * 函数名： GetPtrMotorManager
 * 作用：获取电机管理器指针
 * 参数：无
 * 返回值：电机管理器结构体
 ****************************************************/
MotorManager_t GetPtrMotorManager(void)
{
    return MotorManager;
}

/****************************************************
 * 函数名： MotorInit
 * 作用：用于初始化与电机控制相关的CAN以及注册电机
 * 参数：无
 * 返回值：无
 ****************************************************/
void MotorInit(void)
{
#if TestUse
#if DM_TestUse
    DmTestMotorSingleRegister();
    // 配置CAN报文头
    for (uint8_t i = 0; i < MotorManager.registered_count; i++)
    {
        CanRegisterMotorCfg(&MotorManager.MotorList[i]);
    }
#elif RM_TestUse
    RmTestMotorSingleRegister();
    // 配置CAN报文头
    for (uint8_t i = 0; i < MotorManager.registered_count; i++)
    {
        CanRegisterMotorCfg(&MotorManager.MotorList[i]);
    }
#endif
#else
    MotorRegister();
#endif
    CAN_Init(&hcan1, fifo0, 0, 0);
    CAN_Init(&hcan1, fifo1, 10, 0);
    HAL_Delay(5);
}

/****************************************************
 * 函数名： CanFliterCfg
 * 作用：CAN过滤器的再初始化，其实都是因为强迫症所以只过滤了这个
 * 参数：无
 * 备注：默认接收全部数据,调用该函数之后将只接收达妙MIT模式的反馈和RM电机的反馈帧，但是要对比一下ID
 * 返回值：无
 ****************************************************/
void CanFliterCfg(void)
{
    HAL_CAN_Stop(&hcan1);
    // 保留的位数都应该差不多，只有ID之分，其他就没有了
    // FIFO0用于接收RM电机的反馈帧
    // uint16_t PreserveResult = 0x0000;
    // uint8_t PB_TempArray0[6] = {1, 2, 3, 4, 5, 15};
    // PreserveBit(PB_TempArray0, 6, PreserveResult);
    uint16_t ID_MASK_ARR[2] = {0x000F, 0x000F}; // 当位数较少的时候直接初始化
    uint16_t ID_ARR[2] = {0x0200, 0x0010};
    ID_MASK_ARR[0] = 0x0000; // 0x1100;
    FliterIdCfg_Init(&hcan1, ID_ARR, ID_MASK_ARR, 0, fifo0);
    HAL_Delay(5);

    // FIFO1用于接收DM电机的反馈帧，达妙电机的反馈ID是加上一个两位的偏移
    uint8_t PB_TempArray1[1] = {10};
    // PreserveBit(PB_TempArray1, 1, PreserveResult);
    // ID_ARR[0] = 0x0010;
    ID_MASK_ARR[1] = 0x0000;
    FliterIdCfg_Init(&hcan1, ID_ARR, ID_MASK_ARR, 10, fifo1);
    HAL_CAN_Start(&hcan1);
    HAL_Delay(10);
}

/*****************************************************
 * 函数名：CAN_FIFO_CBKHANDLER
 * 作用：中断回调函数，循环接收消息并根据ID调用对应的电机数据处理函数
 * 参数：fifo_num：FIFO的对应号
 * 参数：hcan：处理时候的can句柄
 * 参数：FIFOmessageNum：要处理的消息数量
 * 返回值：无
 ****************************************************/
void CAN_FIFO_CBKHANDLER(uint32_t fifo_num, uint8_t FIFOmessageNum)
{
    static uint8_t MotorRxDataTempArray[8] = {0}; // 数据暂存
    CAN_RxHeaderTypeDef pRxHeader;
    uint8_t CAN_RX_DATA_COUNT = 0;
    bool ID_MATCHED = false;

    // 循环处理FIFO中的所有消息
    for (uint8_t a = 0; a < FIFOmessageNum; a++)
    {
        // 获取消息
        HAL_CAN_GetRxMessage(&hcan1, fifo_num, &pRxHeader, MotorRxDataTempArray);
        ID_MATCHED = false;

        // 遍历所有已注册的电机，查找匹配的ID
        for (uint8_t i = 0; i < MotorManager.registered_count; i++)
        {
            // 检查是否为RM电机的反馈帧
            if ((MotorManager.MotorList[i].MotorBand == RM_MOTOR_BAND) &&
                (pRxHeader.StdId == (g_RM_MOTOR_BIAS_ADDR + MotorManager.MotorList[i].MotorID)))
            {
                // 找到对应的RM电机，存储接收数据
                memcpy(MotorManager.MotorList[i].ReceiveMotorData, MotorRxDataTempArray, CtrlMotorLen);

                // 调用RM电机数据解算函数
                RM_MOTOR_CALCU(i, MotorManager.MotorList[i].ReceiveMotorData, rm_motor_solved_data);

                CAN_RX_DATA_COUNT++;
                ID_MATCHED = true;
                break; // 找到匹配的电机后跳出内层循环
            }
            // 检查是否为DM电机的反馈帧
            else if ((MotorManager.MotorList[i].MotorBand == DM_MOTOR_BAND) &&
                     (pRxHeader.StdId == (g_DM_MOTOR_BIAS_ADDR_RXID + MotorManager.MotorList[i].MotorID)))
            {
                // 找到对应的DM电机，存储接收数据
                memcpy(MotorManager.MotorList[i].ReceiveMotorData, MotorRxDataTempArray, CtrlMotorLen);

                // 调用DM电机数据解算函数
                DM_MOTOR_CALCU(i, MotorManager.MotorList[i].ReceiveMotorData, dm_motor_solved_data);

                CAN_RX_DATA_COUNT++;
                ID_MATCHED = true;
                break; // 找到匹配的电机后跳出内层循环
            }
        }

        // 清空暂存数据
        memset(MotorRxDataTempArray, 0x00, CtrlMotorLen);
    }

    // 如果没有成功处理任何消息，可能需要错误处理
    // 这里注释掉，因为可能会收到未注册的电机的消息
    if (CAN_RX_DATA_COUNT == 0)
    {
        Error_Handler();
    }
}
