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

/*============================== 硬件抽象层实现 ==============================*/

/// @brief 默认CAN发送函数
static uint8_t Motor_DefaultCanSend(CAN_TxHeaderTypeDef *hdr, uint8_t *data)
{
    return CAN_SendData(&hcan1, hdr, data);
}

/// @brief 默认延时函数（毫秒）
static void Motor_DefaultDelayMs(uint32_t ms)
{
    vTaskDelay(ms);
}

/// @brief 默认HAL实例
static const MotorHAL_t g_DefaultMotorHAL = {
    .can_send = Motor_DefaultCanSend,
    .delay_ms = Motor_DefaultDelayMs,
};

/// @brief 当前使用的HAL指针（非static，供头文件内联函数使用）
const MotorHAL_t *g_pMotorHAL = &g_DefaultMotorHAL;

const MotorHAL_t *Motor_GetHAL(void)
{
    return g_pMotorHAL;
}

void Motor_SetHAL(const MotorHAL_t *hal)
{
    if (hal != NULL)
    {
        g_pMotorHAL = hal;
    }
    else
    {
        g_pMotorHAL = &g_DefaultMotorHAL;
    }
}

/**********************************************************电机初始化专用函数************************************************************************/

/// @brief 注册电机的CAN通信信息
/// @param ptr  电机初始化指针
/// @note  包含报文头的初始化，并同步类参数到展平字段
/// @return 无
void CanRegisterMotorCfg(MotorTypeDef *ptr)
{
    if (ptr == NULL)
    {
        return;
    }

    // 1. 同步品牌和型号到快速访问区
    ptr->band = ptr->MotorInf.band;
    ptr->model = ptr->MotorInf.model;

    // 2. 根据品牌拷贝类参数到展平字段
    if (ptr->band == RM_MOTOR_BAND)
    {
        if (ptr->motor_class.rm_motor_class != NULL)
        {
            ptr->gear_ratio = ptr->motor_class.rm_motor_class->gear_ratio;
            ptr->current_ratio = ptr->motor_class.rm_motor_class->current_ratio;
            ptr->max_current = ptr->motor_class.rm_motor_class->max_current;

            // 旧接口兼容
            ptr->calculate = ptr->motor_class.rm_motor_class->calculate;
            ptr->SendMotorControl = ptr->motor_class.rm_motor_class->send_control;
        }

        // 配置发送ID (StdId)
        if (ptr->model == RmM3508)
            ptr->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_3508;
        else if (ptr->model == RmM2006)
            ptr->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_2006;
        else if (ptr->model == RmGM6020)
            ptr->g_TxHeader.StdId = g_RM_MOTOR_BIAS_ADDR_6020;
    }
    else if (ptr->band == DM_MOTOR_BAND)
    {
        if (ptr->motor_class.dm_motor_class != NULL)
        {
            // DM电机参数暂无统一减速比，可根据需要扩展
            ptr->max_current = (int16_t)ptr->motor_class.dm_motor_class->torque_max;

            // 旧接口兼容
            ptr->calculate = ptr->motor_class.dm_motor_class->calculate;
            ptr->SendMotorControl = ptr->motor_class.dm_motor_class->send_control;

            // 配置发送ID（从ID配置表获取）
            uint8_t idx = (uint8_t)(ptr - MotorManager.MotorList);
            can_motor_cfg motor_cfg = (can_motor_cfg)(idx + 1);
            const DM_MotorIdConfig_t *id_cfg = DM_GetIdConfig(motor_cfg);
            if (id_cfg != NULL)
            {
                ptr->g_TxHeader.StdId = id_cfg->tx_id;
            }
        }
    }

    // 同步快速访问ID
    ptr->can_id_tx = ptr->g_TxHeader.StdId;
    ptr->can_id_rx = ptr->CAN_Rid;

    // 通用CAN设置
    ptr->g_TxHeader.IDE = CAN_ID_STD;
    ptr->g_TxHeader.RTR = CAN_RTR_DATA;
    ptr->g_TxHeader.DLC = CtrlMotorLen;
}

/// @brief  注册电机的信息
/// @param  无（按照已经配置的电机表注册）
/// @return 注册是否成功
void MotorRegister(void)
{
    // ==================== RM电机注册 ====================
    // 夹爪传动带结构 - M3508电机
    RM_M3508_Init(&MotorManager.MotorList[RM_3508_GRIPPER - 1], RM_3508_GRIPPER);
    RM_Motor_SetCascadePID(&MotorManager.MotorList[RM_3508_GRIPPER - 1],
                           14.00f, 0.1f, 0.05f, 10.0f,
                           0.225f, 1.00f, 0.05f, 93.0f,
                           20000.0f, 0.0f, 500.0f,
                           8000.0f, 0.0f, 1500.0f);
    // RM_Motor_SetSpeedPID(&MotorManager.MotorList[RM_3508_GRIPPER - 1], PID_POSITION, 0.225f, 2.50f, 0.0f, 93.0f, 8000.0f, 0.0f, 1500.0f);
    MotorManager.MotorList[RM_3508_GRIPPER - 1].CAN_Rid = 0X001 | g_RM_MOTOR_BIAS_ADDR;

    // 扳机 - M2006电机
    RM_M2006_Init(&MotorManager.MotorList[RM_2006_TRIGGER - 1], RM_2006_TRIGGER);
    RM_Motor_SetCascadePID(&MotorManager.MotorList[RM_2006_TRIGGER - 1],
                           0.3f, 0.0f, 0.0f, 0.000f,
                           15.91f, 0.0f, 0.0f, 0.0f,
                           9000.0f, 0.0f, 0.0f,
                           6000.0f, 0.0f, 0.0f);
    // RM_Motor_SetSpeedPID(&MotorManager.MotorList[RM_2006_TRIGGER - 1], PID_POSITION, 15.91f, 0.0f, 0.0f, 0.0f, 15000.0f, 0.0f, 15000.0f);
    MotorManager.MotorList[RM_2006_TRIGGER - 1].CAN_Rid = RM_2006_TRIGGER + g_RM_MOTOR_BIAS_ADDR_2006 + 0x004;

    // ==================== DM电机注册 ====================
    // 左侧蓄力电机 - J3519
    DM_J3519_Init(&MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1],
                  DM_GetIdConfig(DM_3519_STRENTH_LEFT)->motor_id);
    DM_Motor_SetVelLimits(&MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1], -20.0f, 20.0f);
    DM_Motor_SetPosLimits(&MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1], -200.0f, 200.0f);
    MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1].CAN_Rid = DM_GetIdConfig(DM_3519_STRENTH_LEFT)->rx_id;

    // 右侧蓄力电机 - J3519
    DM_J3519_Init(&MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1],
                  DM_GetIdConfig(DM_3519_STRENTH_RIGHT)->motor_id);
    DM_Motor_SetVelLimits(&MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1], -20.0f, 20.0f);
    DM_Motor_SetPosLimits(&MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1], -200.0f, 200.0f);
    MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1].CAN_Rid = DM_GetIdConfig(DM_3519_STRENTH_RIGHT)->rx_id;

    // Yaw轴电机 - J4310
    DM_J4310_Init(&MotorManager.MotorList[DM_4310_YAW - 1],
                  DM_GetIdConfig(DM_4310_YAW)->motor_id);
    DM_Motor_SetVelLimits(&MotorManager.MotorList[DM_4310_YAW - 1], -30.0f, 30.0f);
    DM_Motor_SetPosLimits(&MotorManager.MotorList[DM_4310_YAW - 1], -160.0f, 160.0f);
    DM_Motor_SetMITParams(&MotorManager.MotorList[DM_4310_YAW - 1], 1.4591f, 1.0f, 0.0f);
    MotorManager.MotorList[DM_4310_YAW - 1].CAN_Rid = DM_GetIdConfig(DM_4310_YAW)->rx_id;

    // ==================== 完成注册 ====================
    MotorManager.registered_count = 5;

    // 最终确认配置并同步展平字段
    for (uint8_t i = 0; i < MotorManager.registered_count; i++)
    {
        CanRegisterMotorCfg(&MotorManager.MotorList[i]);
    }
}

/**********************************************************暴露接口,下面是外部一般用于调用的函数******************************************************/

/****************************************************
 * 函数名： GetPtrMotorManager
 * 作用：获取电机管理器指针
 * 返回值：电机管理器结构体
 ****************************************************/
MotorManager_t *GetPtrMotorManager(void)
{
    return &MotorManager;
}

/****************************************************
 * 函数名： MotorInit
 * 作用：用于初始化与电机控制相关的CAN以及注册电机
 ****************************************************/
void MotorInit(void)
{
#if TestUse
#if DM_TestUse
    DmTestMotorSingleRegister();
#elif RM_TestUse
    RmTestMotorSingleRegister();
#endif
    for (uint8_t i = 0; i < MotorManager.registered_count; i++)
        CanRegisterMotorCfg(&MotorManager.MotorList[i]);
#elif !TestUse
    MotorRegister();
#endif
    CAN_Init(&hcan1, fifo0, 0, 0);
    CAN_Init(&hcan1, fifo1, 10, 0);
    HAL_Delay(5);
}

/****************************************************
 * 函数名： CanFilterCfg
 * 作用：CAN过滤器的再初始化
 ****************************************************/
void CanFilterCfg(void)
{
    HAL_CAN_Stop(&hcan1);
    uint16_t ID_MASK_ARR[2] = {0x0000, 0x0000};
    uint16_t ID_ARR[2] = {0x0200, 0x0010};
    FliterIdCfg_Init(&hcan1, ID_ARR, ID_MASK_ARR, 0, fifo0);
    HAL_Delay(5);
    FliterIdCfg_Init(&hcan1, ID_ARR, ID_MASK_ARR, 10, fifo1);
    HAL_CAN_Start(&hcan1);
    HAL_Delay(10);
}

/*****************************************************
 * 函数名：CAN_FIFO_CBKHANDLER
 * 作用：中断回调函数，优化后的ID匹配和解算逻辑
 ****************************************************/
void CAN_FIFO_CBKHANDLER(uint32_t fifo_num, uint8_t FIFOmessageNum)
{
    static uint8_t MotorRxDataTempArray[8];
    CAN_RxHeaderTypeDef pRxHeader;

    for (uint8_t a = 0; a < FIFOmessageNum; a++)
    {
        HAL_CAN_GetRxMessage(&hcan1, fifo_num, &pRxHeader, MotorRxDataTempArray);

        // 优化后的遍历：直接匹配展平后的 can_id_rx
        for (uint8_t i = 0; i < MotorManager.registered_count; i++)
        {
            if (pRxHeader.StdId == MotorManager.MotorList[i].can_id_rx)
            {
                // 存储并调用虚函数/旧接口指针
                memcpy(MotorManager.MotorList[i].ReceiveMotorData, MotorRxDataTempArray, 8);
                if (MotorManager.MotorList[i].calculate != NULL)
                {
                    MotorManager.MotorList[i].calculate(&MotorManager.MotorList[i]);
                }
                break;
            }
        }
    }
}

/**********************************************************电机句柄获取接口******************************************************/

// Motor_GetHandle() 和 Motor_GetRmSendBuffer() 已改为 CanMotor.h 中的内联函数

/****************************************************
 * 函数名： Motor_SetRegisteredCount
 * 作用：设置已注册电机数量（仅供测试使用）
 * 参数：count - 电机数量
 ****************************************************/
void Motor_SetRegisteredCount(uint8_t count)
{
    if (count <= g_CanMotorNum)
    {
        MotorManager.registered_count = count;
    }
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
        return (motor->motor_data.solved_data[3] - motor->motor_data.offset_ecd_angle);
    }
    else if (motor->MotorInf.band == DM_MOTOR_BAND)
    {
        // DM电机: solved_data[0] = 位置(rad)，需要转换为度
        // 等待修改
        // return RadToDegree(motor->motor_data.solved_data[0] - motor->motor_data.offset_ecd);
        return motor->motor_data.solved_data[0];
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
