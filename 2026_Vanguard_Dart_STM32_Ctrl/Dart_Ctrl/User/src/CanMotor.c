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
#include "FreeRTOS.h"
#include "task.h"

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
/// @note   使用面向对象初始化方式，每种电机类型有专门的电机类
/// @note   电机ID、参数、发送函数、解算函数均由电机类自动设置
/// @example 新接口用法:
///          RM_Motor_Create(&motor, &RM_M2006_Class, 1);  // 使用类创建电机
///          RM_Motor_SetCascadePID(&motor, ...);          // 配置PID
void MotorRegister(void)
{
    // ==================== RM电机注册 ====================
    // 新接口用法示例：使用 RM_Motor_Create + 电机类
    // RM_Motor_Create(&MotorManager.MotorList[RM_3508_GRIPPER - 1], &RM_M3508_Class, RM_3508_GRIPPER);

    // 夹爪传动带结构 - M3508电机
    RM_M3508_Init(&MotorManager.MotorList[RM_3508_GRIPPER - 1], RM_3508_GRIPPER);
    RM_Motor_SetCascadePID(&MotorManager.MotorList[RM_3508_GRIPPER - 1],
                           0.06591f, 0.1f, 0.0f, 0.0001f, // 外环: P=1.05, I=0, D=0, F=0.1
                           178.91f, 0.40f, 0.0f, 7.95f,   // 内环: P=175.91, I=0.40, D=0, F=7.95
                           20673.0f, 0.0f, 2.0f,          // 外环: max_out, min_out, max_iout
                           1691.0f, 0.0f, 300.0f);        // 内环: max_out, min_out, max_iout

    // 扳机 - M2006电机
    RM_M2006_Init(&MotorManager.MotorList[RM_2006_TRIGGER - 1], RM_2006_TRIGGER);
    RM_Motor_SetCascadePID(&MotorManager.MotorList[RM_2006_TRIGGER - 1],
                           0.0001f, 0.0f, 0.0f, 0.002f, // 外环: P=0.0001, I=0, D=0, F=0.002
                           27.91f, 0.08f, 0.0f, 1.0f,   // 内环: P=27.91, I=0.08, D=0, F=1.0
                           20673.0f, 0.0f, 5000.0f,     // 外环: max_out, min_out, max_iout
                           5000.0f, 0.0f, 1000.0f);     // 内环: max_out, min_out, max_iout

    // ==================== DM电机注册 ====================

    // 左侧蓄力电机 - J3519
    DM_J3519_Init(&MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1],
                  DM_3519_STRENTH_LEFT - g_RM_MOTOR_NUM);
    // DM_Motor_SetVelLimits(); // 速度上下限20
    // DM_Motor_SetPosLimits(); // 位置上下限200
    // DM_Motor_SetMITParams(); // MIT参数不知道,3519禁用MIT模式
    // 位置速度模式（PID已经调好了，速度KP：0.5395794，速度KI：0.002，位置KP：54，位置KI：0）

    // 右侧蓄力电机 - J3519
    DM_J3519_Init(&MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1],
                  DM_3519_STRENTH_RIGHT - g_RM_MOTOR_NUM);
    // DM_Motor_SetVelLimits(); // 速度上下限20
    // DM_Motor_SetPosLimits(); // 位置上下限200
    // DM_Motor_SetMITParams(); // MIT参数不知道,3519和3519禁用MIT模式
    // 位置速度模式（PID已经调好了，速度KP：0.5395794，速度KI：0.002，位置KP：54，位置KI：0）

    // Yaw轴电机 - J4310
    DM_J4310_Init(&MotorManager.MotorList[DM_4310_YAW - 1], DM_4310_YAW);                 // 这个电机解算等待处理
    DM_Motor_SetVelLimits(&MotorManager.MotorList[DM_4310_YAW - 1], 30.0f, -30.0f);       // 速度上下限+-30
    DM_Motor_SetPosLimits(&MotorManager.MotorList[DM_4310_YAW - 1], 160.0f, -160.0f);     // 位置上下限+-160
    DM_Motor_SetMITParams(&MotorManager.MotorList[DM_4310_YAW - 1], 1.4561f, 1.0f, 0.0f); // Kp->1.4591f, Kd->1.0f,但是位置不给就对了

    // ==================== 完成注册 ====================
    MotorManager.registered_count = 5;

    // 配置CAN报文头（已在Init函数中完成大部分，这里做最终确认）
    for (uint8_t i = 0; i < MotorManager.registered_count; i++)
    {
        CanRegisterMotorCfg(&MotorManager.MotorList[i]);
    }
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
 * 优化：移除无用memset，减少中断处理时间
 ****************************************************/
void CAN_FIFO_CBKHANDLER(uint32_t fifo_num, uint8_t FIFOmessageNum)
{
    static uint8_t MotorRxDataTempArray[8]; // 数据暂存（无需初始化，会被覆盖）
    CAN_RxHeaderTypeDef pRxHeader;

    // 循环处理FIFO中的所有消息
    for (uint8_t a = 0; a < FIFOmessageNum; a++)
    {
        // 获取消息
        HAL_CAN_GetRxMessage(&hcan1, fifo_num, &pRxHeader, MotorRxDataTempArray);

        // 遍历所有已注册的电机，查找匹配的ID
        for (uint8_t i = 0; i < MotorManager.registered_count; i++)
        {
            // 检查是否为RM3508电机的反馈帧
            if ((MotorManager.MotorList[i].MotorInf.band == RM_MOTOR_BAND) &&
                (pRxHeader.StdId == (g_RM_MOTOR_BIAS_ADDR_3508 + MotorManager.MotorList[i].MotorID)))
            {
                // 找到对应的RM电机，存储接收数据并解算
                memcpy(MotorManager.MotorList[i].ReceiveMotorData, MotorRxDataTempArray, 8);
                MotorManager.MotorList[i].calculate(&MotorManager.MotorList[i]);
                break;
            }

            // 检查是否为RM2006电机的反馈帧
            if ((MotorManager.MotorList[i].MotorInf.band == RM_MOTOR_BAND) &&
                (pRxHeader.StdId == (g_RM_MOTOR_BIAS_ADDR + MotorManager.MotorList[i].MotorID + 3)))
            {
                memcpy(MotorManager.MotorList[i].ReceiveMotorData, MotorRxDataTempArray, 8);
                MotorManager.MotorList[i].calculate(&MotorManager.MotorList[i]);
                break;
            }

            // 检查是否为RM6020电机的反馈帧
            if ((MotorManager.MotorList[i].MotorInf.band == RM_MOTOR_BAND) &&
                (pRxHeader.StdId == (g_RM_MOTOR_BIAS_ADDR + MotorManager.MotorList[i].MotorID + 8)))
            {
                memcpy(MotorManager.MotorList[i].ReceiveMotorData, MotorRxDataTempArray, 8);
                MotorManager.MotorList[i].calculate(&MotorManager.MotorList[i]);
                break;
            }

            // 检查是否为DM电机的反馈帧
            else if ((MotorManager.MotorList[i + g_RM_MOTOR_NUM].MotorInf.band == DM_MOTOR_BAND) &&
                     (pRxHeader.StdId == (g_DM_MOTOR_BIAS_ADDR_RXID + MotorManager.MotorList[i].MotorID)))
            {
                memcpy(MotorManager.MotorList[i].ReceiveMotorData, MotorRxDataTempArray, 8);
                MotorManager.MotorList[i].calculate(&MotorManager.MotorList[i]);
                break;
            }
        }
        // 删除无用的 memset - 下次循环数据会被新消息覆盖
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
        return RadToDegree(motor->motor_data.solved_data[0] - motor->motor_data.offset_ecd);
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
