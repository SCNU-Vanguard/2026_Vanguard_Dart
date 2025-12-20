/*****************************************************
 * RM电机（大疆电机）控制模块
 * 适配M3508、M2006、M6020等系列电机
 *
 * note:M3508 -> 19.2减速比
 *      M2006 -> 36.0减速比
 *      等待添加6020控制帧
 *      等待添加6020特殊逻辑
 * --------------------------------------------------
 * RM电机说明：
 * 大疆电机的注册同一系列电调最多是4个
 * 同一CAN总线上挂载的大疆电调（只有一个系列电调最多是8个）
 * 如果是C610和C620都有，则二者均为4个即为最大值
 ****************************************************/

#include "RM_Motor.h"
#include "CanMotor.h"
#include <stdbool.h>
#include "usart.h"
#include <stdio.h>

// 外部引用电机管理器
extern MotorManager_t MotorManager;
float output = 0.0f;

#define CtrlMotorLen 8     // 电机控制报文长度默认给8
#define RM_MOTOR_MAX_NUM 2 // 最大电机数量

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

    // 将RM3508电机的数据进行拼接
    for (uint8_t a = (st->MotorID - 1); (a <= g_RM_M3508_NUM) && (st->MotorInf.model == RmM3508); a++)
    {
        for (uint8_t b = 0; b < 2; b++)
        {
            MotorManager.RM_MOTOR_DATA_ARRAY[2 * a + b - 2 * (st->MotorID - 1)] = MotorManager.MotorList[a].SendMotorData[b];
        }
    }

    // 将RM2006电机的数据进行拼接
    for (uint8_t a = (st->MotorID - 1); (a <= g_RM_M2006_NUM) && (st->MotorInf.model == RmM2006); a++)
    {
        for (uint8_t b = 0; b < 2; b++)
        {
            MotorManager.RM_MOTOR_DATA_ARRAY[2 * a + b - 2 * (st->MotorID - 1)] = MotorManager.MotorList[a].SendMotorData[b];
        }
    }

    // 将RM6020电机的数据进行拼接
    for (uint8_t a = (st->MotorID - 1); (a < g_RM_GM6020_NUM) && (st->MotorInf.model == RmGM6020); a++)
    {
        for (uint8_t b = 0; b < 2; b++)
        {
            MotorManager.RM_MOTOR_DATA_ARRAY[2 * a + b - 2 * (st->MotorID - 1)] = MotorManager.MotorList[a].SendMotorData[b];
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
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param data 发送数据指针（必须8字节），似乎可以封装一下发送缓冲区
/// @todo 分离一下，这里可以只设置发送函数，将发送任务放在其他地方（比如CAN通信发送任务）
void RM_MotorSetTxData(can_motor_cfg motor_cfg, uint8_t *data)
{
    assert_param(data != NULL);
    if (data == NULL)
    {
        return; // 返回错误码
    }
    memset(MotorManager.MotorList[motor_cfg - 1].SendMotorData, 0x00, CtrlMotorLen);                // 初始化
    memcpy(MotorManager.MotorList[motor_cfg - 1].SendMotorData, data, CtrlMotorLen);                // Copy数据
    MotorManager.MotorList[motor_cfg - 1].SendMotorControl(&MotorManager.MotorList[motor_cfg - 1]); // 调用发送函数
}

/**********************************************************电机数据接收解算专用函数******************************************************************/

/**************************************
 * RoboMaster电机返回数据
 * 长度 : 8字节（固定）
 * 数据说明:
 * note : 默认前一个是高字节，后一个是低字节
 * Data[0~1] : 电机转角高低字节 (0~8191)
 * Data[2~3] : 电机转速高低字节 (rpm)
 * Data[4~5] : 电机力矩高低字节
 * Data[6~7] : 保留(C610电调)
 * Data[6] : 电机温度(C620电调)
 * Data[7] : 保留
 *************************************/

/// @brief RM电机接收数据解算
/// @param motor 电机结构体指针
/// @note motor->motor_data.solved_data[0]: 单圈角度(°)
/// @note motor->motor_data.solved_data[1]: 速度(rpm)
/// @note motor->motor_data.solved_data[2]: 电流(A)
/// @note motor->motor_data.solved_data[3]: 累计角度(°) - 用于位置闭环
/// @note motor->motor_data.solved_data[4]: 速度(rad/s) - 弧度制速度
void RM_MOTOR_CALCU(MotorTypeDef *motor)
{
    assert_param(motor != NULL);
    if (motor == NULL)
    {
        return;
    }

    MotorSolvedData_t *pData = &motor->motor_data;
    uint8_t *ReceiveData = motor->ReceiveMotorData;

    // =============== 1. 数据解析 ===============
    int16_t ecd = (((uint16_t)ReceiveData[0]) << 8) | ReceiveData[1];
    int16_t speed_rpm = (int16_t)((((uint16_t)ReceiveData[2]) << 8) | ReceiveData[3]);
    int16_t current_raw = (int16_t)((((uint16_t)ReceiveData[4]) << 8) | ReceiveData[5]);
    // int8_t temperature = ReceiveData[6];  // 温度，按需使用

    // =============== 2. 首次初始化 ===============
    if (pData->init_flag == 0)
    {
        pData->last_ecd = ecd;
        pData->offset_ecd = ecd; // 首次位置作为零点
        pData->init_flag = 1;
        pData->offset_ecd_angle = ecd / 8192.0f * 360.0f; // 零点(角度表示)
    }

    // =============== 3. 过零检测 & 多圈累计 ===============
    int16_t err = ecd - pData->last_ecd;
    pData->total_round += (err > 4096) ? -1 : (err < -4096) ? 1
                                                            : 0;
    pData->total_ecd += err + ((err > 4096) ? -8192 : (err < -4096) ? 8192
                                                                    : 0);
    pData->last_ecd = ecd;

    // =============== 4. 数据转换输出 ===============
    // 单圈角度 (0~360°)
    pData->solved_data[0] = ecd / 8192.0f * 360.0f;

    // 速度 (rpm) - 使用对应电机的滤波变量
    if (!pData->filter_init)
    {
        pData->solved_data[1] = (float)speed_rpm;
        pData->filter_init = 1;
    }
    else
    {
        pData->solved_data[1] = (float)speed_rpm * 0.9f + 0.1f * pData->last_speed;
    }

    if (motor->MotorInf.model == RmM3508)
    {
        // 电流 (A) - M3508标准：16384对应20A
        pData->solved_data[2] = current_raw / 16384.0f * 20.0f;
    }
    else if (motor->MotorInf.model == RmM2006)
    {
        // 电流(A) - M2006标准: 根据手册的转换系数得来
        pData->solved_data[2] = current_raw / 0.18;
    }
    else if (motor->MotorInf.model == RmGM6020)
    {
        // 电流 (A) - M3508标准：16384对应3A
        pData->solved_data[2] = current_raw / 16384.0f * 3.0f;
    }

    // 累计角度 (°) - 可超过360°，用于位置闭环
    pData->solved_data[3] = pData->total_round * 360.0f + pData->solved_data[0];

    // 速度 (rad/s) - rpm转弧度/秒
    pData->solved_data[4] = speed_rpm * 0.10472f; // 2*PI/60 ≈ 0.10472

    pData->last_speed = pData->solved_data[1];
}

/*====================  辅助函数  ====================*/

/// @brief 重置电机零点（当前位置设为零点）
/// @param motor 电机结构体指针
void RM_Motor_Reset_Zero(MotorTypeDef *motor)
{
    if (motor != NULL)
    {
        motor->motor_data.total_round = 0;
        motor->motor_data.total_ecd = 0;
        motor->motor_data.offset_ecd = motor->motor_data.last_ecd;
    }
}

/// @brief 重置所有电机状态（用于重新初始化）
void RM_Motor_Reset_All(void)
{
    for (uint8_t i = 0; i < RM_MOTOR_MAX_NUM; i++)
    {
        memset(&MotorManager.MotorList[i].motor_data, 0, sizeof(MotorSolvedData_t));
    }
}

/**********************************************************暴露接口，下面是外部一般用于调用的函数******************************************************/

/****************************************************
 * 函数名： RmMotorSendCfg
 * 作用：用于设置发送RM电机数据
 * 参数：motor_cfg：电机配置枚举值 (can_motor_cfg)
 * 参数：TargetCurrent：电流大小
 * 返回值：无
 * 说明：大疆电机使用反码形式，负数需对绝对值取反
 ****************************************************/
void RmMotorSendCfg(can_motor_cfg motor_cfg, int16_t TargetCurrent)
{
    if (TargetCurrent < 0)
    {
        // 负数：对绝对值取反（反码形式）
        // 例如：-400 -> ~400 = ~0x0190 = 0xFE6F
        TargetCurrent = (uint16_t)(~(-TargetCurrent));
    }
    uint8_t data[8] = {0x00};
    data[0] = (uint8_t)(TargetCurrent >> 8); // 高字节
    data[1] = (uint8_t)TargetCurrent;        // 低字节
    RM_MotorSetTxData(motor_cfg, data);
}

/**********************************************************电机初始化专用函数************************************************************************/

/// @brief  测试单个RM电机
/// @param  无
/// @note   仅供测试使用
/// @return 无
void RmTestMotorSingleRegister(void)
{
    /* -------------------------------------------------------------------------------------------- */
    // 3508传动带
    // 按照需求更改参数
    // MotorManager.MotorList[SingleMotorTest - 1].MotorID = SingleMotorTest;
    // MotorManager.MotorList[SingleMotorTest - 1].MotorInf.band = RM_MOTOR_BAND;
    // MotorManager.MotorList[SingleMotorTest - 1].MotorInf.model = RmM3508;
    // MotorManager.MotorList[SingleMotorTest - 1].SendMotorControl = RM_MotorSendControl;
    // MotorManager.registered_count = 1;
    // MotorManager.MotorList[SingleMotorTest - 1].use_cascade = 1;
    // float inner_p = 175.91f; // 内环p
    // float inner_i = 0.40f;   // 内环i
    // float inner_d = 0.0f;    // 内环d
    // float inner_f = 7.91f;   // 内环f
    // float outer_p = 1.05f;   // 外环p
    // float outer_i = 0.0f;    // 外环i
    // float outer_d = 0.0f;    // 外环d
    // float outer_f = 0.1f;    // 外环f
    // // 外环最大限幅19000（全路程对应角度20673左右）
    // // PID_Init(&MotorManager.MotorList[SingleMotorTest - 1].speed_pid, PID_DELTA, p, i, d, f, 1691.0f, 100.0f, 60.0f); // 暂定最大1691
    // // PID_Clear(&MotorManager.MotorList[SingleMotorTest - 1].speed_pid);
    // CASCADE_PID_Init(&MotorManager.MotorList[SingleMotorTest - 1].cascade_pid, outer_p, outer_i, outer_d, outer_f, inner_p, inner_i, inner_d, inner_f, 20673.0f, -20673.0f, 0.0f, 1691.0f, 100.0f, 60.0f); // 等待换弹结构总测试
    // CASCADE_PID_Clear(&MotorManager.MotorList[SingleMotorTest - 1].cascade_pid);
    /* -------------------------------------------------------------------------------------------- */

    // 2006扳机
    // 按照需求更改参数
    MotorManager.MotorList[SingleMotorTest - 1].MotorID = SingleMotorTest;
    MotorManager.MotorList[SingleMotorTest - 1].MotorInf.band = RM_MOTOR_BAND;
    MotorManager.MotorList[SingleMotorTest - 1].MotorInf.model = RmM2006;
    MotorManager.MotorList[SingleMotorTest - 1].SendMotorControl = RM_MotorSendControl;
    MotorManager.registered_count = 1;
    MotorManager.MotorList[SingleMotorTest - 1].use_cascade = 1;
    float inner_p = 27.91f;
    float inner_i = 0.08f;
    float inner_d = 0.0f;
    float inner_f = 1.0f;
    float outer_p = 0.000091f;
    float outer_i = 0.0f;
    float outer_d = 0.0f;
    float outer_f = 0.002f;

    // PID_Init(&MotorManager.MotorList[SingleMotorTest - 1].speed_pid, PID_DELTA, inner_p, inner_i, inner_d, inner_f, 3000.0f, 0.0f, 100.0f);
    // PID_Clear(&MotorManager.MotorList[SingleMotorTest - 1].speed_pid);

    // 2006
    // CASCADE_PID_Init(&MotorManager.MotorList[SingleMotorTest - 1].cascade_pid, outer_p, outer_i, outer_d, outer_f, inner_p, inner_i, inner_d, inner_f, 20673.0f, -20673.0f, 0.0f, 3000.0f, 0.0f, 100.0f); // 等待上扳机2006电机测试
    // CASCADE_PID_Clear(&MotorManager.MotorList[SingleMotorTest - 1].cascade_pid);

    // 3508
    // CASCADE_PID_Init(&MotorManager.MotorList[SingleMotorTest - 1].cascade_pid, outer_p, outer_i, outer_d, outer_f, inner_p, inner_i, inner_d, inner_f, 20673.0f, -20673.0f, 0.0f, 1691.0f, 100.0f, 60.0f); // 等待换弹结构总测试,位置环的上下限是角度限制
    // CASCADE_PID_Clear(&MotorManager.MotorList[SingleMotorTest - 1].cascade_pid);

    // CAN报文头配置在CanMotor.c中的CanRegisterMotorCfg函数完成
}

/// @brief 去除电机偏移对电机目标数值影响
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param Target 电机目标数值（相对增量）
/// @return 修正后的电机目标数值（绝对位置）
/// @note target_angle 用于记录上次设置的目标位置，只有电机到达该位置后才允许更新新目标
float RmMotorRemoveBias(can_motor_cfg motor_cfg, float Target)
{
    // 根据枚举值获取对应的电机结构体指针
    // can_motor_cfg 从1开始，数组索引从0开始
    uint8_t idx = (uint8_t)motor_cfg - 1;
    if (idx >= g_CanMotorNum)
    {
        return Target;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[idx];

    // 检查是否为RM电机
    if (motor->MotorInf.band != RM_MOTOR_BAND)
    {
        return Target; // 不是RM电机，直接返回原值
    }

    MotorSolvedData_t *pData = &motor->motor_data;

    // 等待滤波器初始化完成（确保已接收到电机反馈数据）
    while (!pData->filter_init)
        ;

    // 首次调用：初始化目标值，将当前位置作为基准并累加Target
    if (pData->target_init_flag == 0)
    {
        // 首次：基于当前累计角度计算目标（solved_data[3]单位为度）
        float new_target = pData->solved_data[3] + Target;

        // 记录目标角度
        pData->target_angle = new_target;
        pData->last_target = new_target;
        pData->pre_last_target = new_target;
        pData->target_init_flag = 1;

        return new_target;
    }

    // 非首次调用：检查是否到达上次目标位置
    // 使用 int16_t 比较，允许一定误差
    int16_t current_angle = (int16_t)pData->solved_data[3]; // 当前累计角度（度）
    int16_t last_target_angle = (int16_t)pData->target_angle;

    // 检查当前位置是否到达上次目标（允许±50度误差）,并且目标发生改变
    // if ((current_angle >= last_target_angle - 50) && (current_angle <= last_target_angle + 50) && (last_target_angle != (int16_t)(pData->solved_data[3] + Target)))
    if ((current_angle >= last_target_angle - 50) && (current_angle <= last_target_angle + 50) && (last_target_angle >= (pData->solved_data[3] + Target - 50)) && (last_target_angle <= (pData->solved_data[3] + Target + 50)))
    {
        // 已到达目标位置，允许更新新目标
        float new_target = pData->solved_data[3] + Target;

        // 3508换弹结构需要换向补偿
        if (motor->MotorInf.model == RmM3508)
        {
            if (motor->cascade_pid.inner.calc_count)
            {
                if ((pData->last_target > new_target) && (pData->pre_last_target < pData->last_target))
                {
                    // 上次正转，这次反转
                    new_target -= 15.0f;
                }
                else if ((pData->last_target < new_target) && (pData->pre_last_target > pData->last_target))
                {
                    // 上次反转，这次正转
                    new_target += 15.0f;
                }
            }
        }

        // 更新历史记录
        pData->pre_last_target = pData->last_target;
        pData->last_target = new_target;
        pData->target_angle = new_target;

        return new_target;
    }
    else
    {
        // 未到达目标位置，返回上次的目标值，不允许更新
        return pData->target_angle;
    }
}

/// @brief 增量式位置偏移计算（相对于当前位置的增量）
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param delta 相对于当前位置的增量（度）
/// @return 计算后的绝对目标位置
/// @note 与 RmMotorRemoveBias 的区别：
///       - RmMotorRemoveBias: 绝对式，Target 相对于首次调用时的位置
///       - RmMotorRemoveBiasIncr: 增量式，delta 相对于上次目标位置
///       增量式不需要等待到达目标，每次调用都会累加到目标值上
float RmMotorRemoveBiasIncr(can_motor_cfg motor_cfg, float delta)
{
    // 根据枚举值获取对应的电机结构体指针
    uint8_t idx = (uint8_t)motor_cfg - 1;
    if (idx >= g_CanMotorNum)
    {
        return delta;
    }

    MotorTypeDef *motor = &MotorManager.MotorList[idx];

    // 检查是否为RM电机
    if (motor->MotorInf.band != RM_MOTOR_BAND)
    {
        return delta;
    }

    MotorSolvedData_t *pData = &motor->motor_data;

    // 等待滤波器初始化完成（确保已接收到电机反馈数据）
    while (!pData->filter_init)
        ;

    // 首次调用：初始化目标值为当前位置
    if (pData->target_init_flag == 0)
    {
        pData->target_angle = pData->solved_data[3]; // 记录当前位置作为初始目标
        pData->last_target = pData->target_angle;
        pData->pre_last_target = pData->target_angle;
        pData->target_init_flag = 1;
    }

    // 增量式：在上次目标位置基础上累加增量
    float new_target = pData->target_angle + delta;

    // 3508换弹结构需要换向补偿
    if (motor->MotorInf.model == RmM3508)
    {
        if (motor->cascade_pid.inner.calc_count)
        {
            if ((pData->last_target > new_target) && (pData->pre_last_target < pData->last_target))
            {
                // 上次正转，这次反转
                new_target -= 15.0f;
            }
            else if ((pData->last_target < new_target) && (pData->pre_last_target > pData->last_target))
            {
                // 上次反转，这次正转
                new_target += 15.0f;
            }
        }
    }

    // 更新历史记录
    pData->pre_last_target = pData->last_target;
    pData->last_target = new_target;
    pData->target_angle = new_target;

    return new_target;
}

/// @brief RM电机输出
/// @param motor_cfg 电机配置枚举值 (can_motor_cfg)
/// @param target 目标值，单环时候为速度，串级为位置
void RmMotorPID_Calc(can_motor_cfg motor_cfg, float target)
{
    MotorTypeDef *motor = &MotorManager.MotorList[motor_cfg - 1];
    MotorSolvedData_t *pData = &motor->motor_data;

    // PID数据输出
    output = CASCADE_PID_Calculate(&motor->cascade_pid, target, pData->solved_data[3], pData->solved_data[4]); // 目标角度，反馈角度，反馈速度
    // output = PID_Calculate(&motor->speed_pid, target, pData->solved_data[4]); // Rad/S
    RmMotorSendCfg(motor_cfg, output);
    // printf("%.1f,%.1f,%.1f\r\n", pData->solved_data[1], target, output);

    // HAL_UART_Transmit_IT(&huart3, );
}
