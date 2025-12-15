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

    // 3508默认占用200帧头（反馈为0x201 - 0x204）
    // 2006默认占用1FF帧头 (反馈为0x205 - 0x208)
    // 6020默认使用电流环，占用1FE帧头（反馈为0x204 + id）
    if ((ptr->MotorInf.band == RM_MOTOR_BAND) && (ptr->MotorInf.model == RmM3508))
    {
        ptr->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_3508;
    }

    else if ((ptr->MotorInf.band == RM_MOTOR_BAND) && (ptr->MotorInf.model == RmM2006))
    {
        ptr->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_2006;
    }

    else if ((ptr->MotorInf.band == RM_MOTOR_BAND) && (ptr->MotorInf.model == RmGM6020))
    {
        ptr->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_6020;
    }
    else
    {
        ptr->g_TxHeader.StdId = g_DM_MOTOR_BIAS_ADDR_TXID + ptr->MotorID;
    }

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
    // 发送之后自己会memset()

    // 夹爪传动带结构
    MotorManager.MotorList[RM_3508_GRIPPER - 1].MotorID = RM_3508_GRIPPER;
    MotorManager.MotorList[RM_3508_GRIPPER - 1].MotorInf.band = RM_MOTOR_BAND;
    MotorManager.MotorList[RM_3508_GRIPPER - 1].MotorInf.model = RmM3508;
    MotorManager.MotorList[RM_3508_GRIPPER - 1].SendMotorControl = RM_MotorSendControl;
    MotorManager.MotorList[RM_3508_GRIPPER - 1].use_cascade = 1;
    float inner_p = 175.91f;
    float inner_i = 0.40f;
    float inner_d = 0.0f;
    float inner_f = 7.91f;
    float outer_p = 1.05f;
    float outer_i = 0.0f;
    float outer_d = 0.0f;
    float outer_f = 0.1f;
    CASCADE_PID_Init(&MotorManager.MotorList[SingleMotorTest - 1].cascade_pid, outer_p, outer_i, outer_d, outer_f, inner_p, inner_i, inner_d, inner_f, 20673.0f, -20673.0f, 0.0f, 1691.0f, 100.0f, 60.0f); // 等待换弹结构总测试
    CASCADE_PID_Clear(&MotorManager.MotorList[SingleMotorTest - 1].cascade_pid);

    // 扳机
    MotorManager.MotorList[RM_2006_TRIGGER - 1].MotorID = RM_2006_TRIGGER;
    MotorManager.MotorList[RM_2006_TRIGGER - 1].MotorInf.band = RM_MOTOR_BAND;
    MotorManager.MotorList[RM_2006_TRIGGER - 1].MotorInf.model = RmM2006;
    MotorManager.MotorList[RM_2006_TRIGGER - 1].SendMotorControl = RM_MotorSendControl;
    inner_p = 27.91f;
    inner_i = 0.08f;
    inner_d = 0.0f;
    inner_f = 1.0f;
    outer_p = 0.000091f;
    outer_i = 0.0f;
    outer_d = 0.0f;
    outer_f = 0.002f;
    CASCADE_PID_Init(&MotorManager.MotorList[RM_2006_TRIGGER - 1].cascade_pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    // 注册DM电机
    // 注意注册的DM电机发送和接收其实数据帧都不与RM电机冲突（在MIT模式、位置速度模式和PVT模式下，就完整通信帧而言）
    MotorManager.MotorList[DM_3510_STRENTH_LEFT - 1].MotorID = DM_3510_STRENTH_LEFT - g_RM_MOTOR_NUM;
    MotorManager.MotorList[DM_3510_STRENTH_LEFT - 1].MotorInf.band = DM_MOTOR_BAND;
    MotorManager.MotorList[DM_3510_STRENTH_LEFT - 1].SendMotorControl = DM_MotorSendControl;

    MotorManager.MotorList[DM_3510_STRENTH_RIGHT - 1].MotorID = DM_3510_STRENTH_RIGHT - g_RM_MOTOR_NUM;
    MotorManager.MotorList[DM_3510_STRENTH_RIGHT - 1].MotorInf.band = DM_MOTOR_BAND;
    MotorManager.MotorList[DM_3510_STRENTH_RIGHT - 1].SendMotorControl = DM_MotorSendControl;

    MotorManager.MotorList[DM_4310_YAW - 1].MotorID = DM_4310_YAW;
    MotorManager.MotorList[DM_4310_YAW - 1].MotorInf.band = DM_MOTOR_BAND;
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
#elif !TestUse
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
    // uint8_t PB_TempArray1[1] = {10};
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
    // bool ID_MATCHED = false;

    // 循环处理FIFO中的所有消息
    for (uint8_t a = 0; a < FIFOmessageNum; a++)
    {
        // 获取消息
        HAL_CAN_GetRxMessage(&hcan1, fifo_num, &pRxHeader, MotorRxDataTempArray);
        // ID_MATCHED = false;

        // 遍历所有已注册的电机，查找匹配的ID
        for (uint8_t i = 0; i < MotorManager.registered_count; i++)
        {
            // 检查是否为RM3508电机的反馈帧
            if ((MotorManager.MotorList[i].MotorInf.band == RM_MOTOR_BAND) &&
                (pRxHeader.StdId == (g_RM_MOTOR_BIAS_ADDR_3508 + MotorManager.MotorList[i].MotorID)))
            {
                // 找到对应的RM电机，存储接收数据
                memcpy(MotorManager.MotorList[i].ReceiveMotorData, MotorRxDataTempArray, CtrlMotorLen);

                // 调用RM电机数据解算函数，传递结构体中的数据指针
                RM_MOTOR_CALCU(&MotorManager.MotorList[i]);

                CAN_RX_DATA_COUNT++;
                // ID_MATCHED = true;
                break; // 找到匹配的电机后跳出内层循环
            }

            // 检查是否为RM2006电机的反馈帧
            if ((MotorManager.MotorList[i].MotorInf.band == RM_MOTOR_BAND) &&
                (pRxHeader.StdId == (g_RM_MOTOR_BIAS_ADDR + MotorManager.MotorList[i].MotorID + 4)))
            {
                // 找到对应的RM电机，存储接收数据
                memcpy(MotorManager.MotorList[i].ReceiveMotorData, MotorRxDataTempArray, CtrlMotorLen);

                // 调用RM电机数据解算函数，传递结构体中的数据指针
                RM_MOTOR_CALCU(&MotorManager.MotorList[i]);

                CAN_RX_DATA_COUNT++;
                // ID_MATCHED = true;
                break; // 找到匹配的电机后跳出内层循环
            }

            // 检查是否为RM6020电机的反馈帧
            // 注意，这里不能和2006直接使用，假如6020电机ID为1，同时也有一个2006电机ID也为1就会轧钢
            // 解决方案：将6020的ID设置到5、6、7, 并同时更改 g_RM_MOTOR_BIAS_ADDR_6020 为 0x2FE，下面的 4 改为 8
            if ((MotorManager.MotorList[i].MotorInf.band == RM_MOTOR_BAND) &&
                (pRxHeader.StdId == (g_RM_MOTOR_BIAS_ADDR + MotorManager.MotorList[i].MotorID + 8)))
            {
                // 找到对应的RM电机，存储接收数据
                memcpy(MotorManager.MotorList[i].ReceiveMotorData, MotorRxDataTempArray, CtrlMotorLen);

                // 调用RM电机数据解算函数，传递结构体中的数据指针
                RM_MOTOR_CALCU(&MotorManager.MotorList[i]);

                CAN_RX_DATA_COUNT++;
                // ID_MATCHED = true;
                break; // 找到匹配的电机后跳出内层循环
            }

            // 检查是否为DM电机的反馈帧(DM电机的反馈是几乎一样的)
            else if ((MotorManager.MotorList[i + g_RM_MOTOR_NUM].MotorInf.band == DM_MOTOR_BAND) &&
                     (pRxHeader.StdId == (g_DM_MOTOR_BIAS_ADDR_RXID + MotorManager.MotorList[i].MotorID)))
            {
                // 找到对应的DM电机，存储接收数据
                memcpy(MotorManager.MotorList[i].ReceiveMotorData, MotorRxDataTempArray, CtrlMotorLen);

                // 调用DM电机数据解算函数，传递结构体中的数据指针
                DM_MOTOR_CALCU(&MotorManager.MotorList[i]);

                CAN_RX_DATA_COUNT++;
                // ID_MATCHED = true;
                break; // 找到匹配的电机后跳出内层循环
            }
        }

        // 清空暂存数据
        memset(MotorRxDataTempArray, 0x00, CtrlMotorLen);
    }

    // 如果没有成功处理任何消息，可能需要错误处理
    // 这里注释掉，因为可能会收到未注册的电机的消息
    // if (CAN_RX_DATA_COUNT == 0)
    // {
    //     Error_Handler();
    // }
}

/**********************************************************电机数据读取接口******************************************************/

/****************************************************
 * 函数名： Motor_GetTotalAngle
 * 作用：获取电机累计角度（单位：度）
 * 参数：motor_id - 电机ID（can_motor_cfg枚举值）
 * 返回值：累计角度（度），失败返回0
 ****************************************************/
float Motor_GetTotalAngle(can_motor_cfg motor_id)
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count)
    {
        return 0.0f;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1];

    if (motor->MotorInf.band == RM_MOTOR_BAND)
    {
        // RM电机: solved_data[3] = 累计角度(°)
        return motor->motor_data.solved_data[3];
    }
    else if (motor->MotorInf.band == DM_MOTOR_BAND)
    {
        // DM电机: solved_data[0] = 位置(rad)，需要转换为度
        return RadToDegree(motor->motor_data.solved_data[0]);
    }

    return 0.0f;
}

/****************************************************
 * 函数名： Motor_GetTotalAngleRad
 * 作用：获取电机累计角度（单位：弧度）
 * 参数：motor_id - 电机ID（can_motor_cfg枚举值）
 * 返回值：累计角度（弧度），失败返回0
 ****************************************************/
float Motor_GetTotalAngleRad(can_motor_cfg motor_id)
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count)
    {
        return 0.0f;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1];

    if (motor->MotorInf.band == RM_MOTOR_BAND)
    {
        // RM电机: solved_data[3] = 累计角度(°)，需要转换为弧度
        return DegreeToRad(motor->motor_data.solved_data[3]);
    }
    else if (motor->MotorInf.band == DM_MOTOR_BAND)
    {
        // DM电机: solved_data[0] = 位置(rad)
        return motor->motor_data.solved_data[0];
    }

    return 0.0f;
}

/****************************************************
 * 函数名： Motor_GetSpeedRPM
 * 作用：获取电机速度（单位：rpm）
 * 参数：motor_id - 电机ID（can_motor_cfg枚举值）
 * 返回值：速度（rpm），失败返回0
 ****************************************************/
float Motor_GetSpeedRPM(can_motor_cfg motor_id)
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count)
    {
        return 0.0f;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1];

    if (motor->MotorInf.band == RM_MOTOR_BAND)
    {
        // RM电机: solved_data[1] = 速度(rpm)
        return motor->motor_data.solved_data[1];
    }
    else if (motor->MotorInf.band == DM_MOTOR_BAND)
    {
        // DM电机: solved_data[1] = 速度(rad/s)，需要转换为rpm
        // rpm = rad/s * 60 / (2*π) ≈ rad/s * 9.5493
        return motor->motor_data.solved_data[1] * 9.5493f;
    }

    return 0.0f;
}

/****************************************************
 * 函数名： Motor_GetSpeedRadS
 * 作用：获取电机速度（单位：rad/s）
 * 参数：motor_id - 电机ID（can_motor_cfg枚举值）
 * 返回值：速度（rad/s），失败返回0
 ****************************************************/
float Motor_GetSpeedRadS(can_motor_cfg motor_id)
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count)
    {
        return 0.0f;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1];

    if (motor->MotorInf.band == RM_MOTOR_BAND)
    {
        // RM电机: solved_data[4] = 速度(rad/s)
        return motor->motor_data.solved_data[4];
    }
    else if (motor->MotorInf.band == DM_MOTOR_BAND)
    {
        // DM电机: solved_data[1] = 速度(rad/s)
        return motor->motor_data.solved_data[1];
    }

    return 0.0f;
}

/****************************************************
 * 函数名： Motor_GetSingleAngle
 * 作用：获取电机单圈角度（单位：度）
 * 参数：motor_id - 电机ID（can_motor_cfg枚举值）
 * 返回值：单圈角度（度），失败返回0
 ****************************************************/
float Motor_GetSingleAngle(can_motor_cfg motor_id)
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count)
    {
        return 0.0f;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1];

    if (motor->MotorInf.band == RM_MOTOR_BAND)
    {
        // RM电机: solved_data[0] = 单圈角度(°)
        return motor->motor_data.solved_data[0];
    }
    else if (motor->MotorInf.band == DM_MOTOR_BAND)
    {
        // DM电机: 将位置转换为0-360度范围
        float rad = motor->motor_data.solved_data[0];
        float deg = RadToDegree(rad);
        // 归一化到0-360度
        while (deg < 0)
            deg += 360.0f;
        while (deg >= 360.0f)
            deg -= 360.0f;
        return deg;
    }

    return 0.0f;
}

/****************************************************
 * 函数名： Motor_GetCurrent
 * 作用：获取电机电流（单位：A）
 * 参数：motor_id - 电机ID（can_motor_cfg枚举值）
 * 返回值：电流（A），失败返回0
 * 备注：DM电机返回的是力矩(N·m)
 ****************************************************/
float Motor_GetCurrent(can_motor_cfg motor_id)
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count)
    {
        return 0.0f;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1];

    if (motor->MotorInf.band == RM_MOTOR_BAND)
    {
        // RM电机: solved_data[2] = 电流(A)
        return motor->motor_data.solved_data[2];
    }
    else if (motor->MotorInf.band == DM_MOTOR_BAND)
    {
        // DM电机: solved_data[2] = 力矩(N·m)
        return motor->motor_data.solved_data[2];
    }

    return 0.0f;
}

/****************************************************
 * 函数名： Motor_GetAllData
 * 作用：获取电机所有解算数据
 * 参数：motor_id - 电机ID（can_motor_cfg枚举值）
 *       data - 输出数据指针（需提供5个float空间）
 * 返回值：true-成功，false-失败
 * 备注：
 *   RM电机: [0]单圈角度(°), [1]速度(rpm), [2]电流(A), [3]累计角度(°), [4]速度(rad/s)
 *   DM电机: [0]位置(rad), [1]速度(rad/s), [2]力矩(N·m), [3]MOS温度(℃), [4]转子温度(℃)
 ****************************************************/
bool Motor_GetAllData(can_motor_cfg motor_id, float *data)
{
    if (motor_id < 1 || motor_id > MotorManager.registered_count || data == NULL)
    {
        return false;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[motor_id - 1];
    memcpy(data, motor->motor_data.solved_data, sizeof(float) * MOTOR_SOLVED_DATA_NUM);

    return true;
}
