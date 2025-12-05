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
#include "usart.h"
#include <stdio.h>

// 外部引用电机管理器
extern MotorManager_t MotorManager;
extern float rm_motor_solved_data[]; // 存储RM电机解算后的数据
float output = 0.0f;

#define CtrlMotorLen 8 // 电机控制报文长度默认给8
#define SingleMotorTest 1

/*====================  静态变量定义（用于多圈累计）  ====================*/
#define RM_MOTOR_MAX_NUM 8 // 最大电机数量

static int16_t last_ecd[RM_MOTOR_MAX_NUM] = {0};    // 上次编码器值
static int32_t total_round[RM_MOTOR_MAX_NUM] = {0}; // 累计圈数
static int32_t total_ecd[RM_MOTOR_MAX_NUM] = {0};   // 累计编码器值
static uint8_t init_flag[RM_MOTOR_MAX_NUM] = {0};   // 初始化标志
static int16_t offset_ecd[RM_MOTOR_MAX_NUM] = {0};  // 零点偏移
static float lastspeed = 0.0f;
static uint8_t count = 0;

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
void RM_MotorSetTxData(uint8_t motor_id, uint8_t *data)
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
 * Data[0~1] : 电机转角高低字节 (0~8191)
 * Data[2~3] : 电机转速高低字节 (rpm)
 * Data[4~5] : 电机力矩高低字节
 * Data[6~7] : 保留(C610电调)
 * Data[6] : 电机温度(C620电调)
 * Data[7] : 保留
 *************************************/

/// @brief RM电机接收数据解算
/// @param motor_id_num 大疆电机的id号 (0~7，对应电调ID 1~8)
/// @param ReceiveData 接收到的数据数组 (8字节CAN数据)
/// @param solved_data 解算后的数据数组（至少5个float）
/// @note solved_data[0]: 单圈角度(°)
/// @note solved_data[1]: 速度(rpm)
/// @note solved_data[2]: 电流(A)
/// @note solved_data[3]: 累计角度(°) - 用于位置闭环
/// @note solved_data[4]: 速度(rad/s) - 弧度制速度
void RM_MOTOR_CALCU(uint8_t motor_id_num, uint8_t *ReceiveData, float *solved_data)
{
    // =============== 1. 数据解析 ===============
    int16_t ecd = (((uint16_t)ReceiveData[0]) << 8) | ReceiveData[1];
    int16_t speed_rpm = (int16_t)((((uint16_t)ReceiveData[2]) << 8) | ReceiveData[3]);
    int16_t current_raw = (int16_t)((((uint16_t)ReceiveData[4]) << 8) | ReceiveData[5]);
    // int8_t temperature = ReceiveData[6];  // 温度，按需使用

    // =============== 2. 首次初始化 ===============
    if (init_flag[motor_id_num] == 0)
    {
        last_ecd[motor_id_num] = ecd;
        offset_ecd[motor_id_num] = ecd; // 首次位置作为零点
        init_flag[motor_id_num] = 1;
    }

    // =============== 3. 过零检测 & 多圈累计 ===============
    int16_t err = ecd - last_ecd[motor_id_num];
    total_round[motor_id_num] += (err > 4096) ? -1 : (err < -4096) ? 1
                                                                   : 0;
    total_ecd[motor_id_num] += err + ((err > 4096) ? -8192 : (err < -4096) ? 8192
                                                                           : 0);
    last_ecd[motor_id_num] = ecd;

    // =============== 4. 数据转换输出 ===============
    // 单圈角度 (0~360°)
    solved_data[0] = ecd / 8192.0f * 360.0f;

    // 速度 (rpm)
    if (!count)
    {
        solved_data[1] = (float)speed_rpm;
        count = 1;
    }
    else
    {
        solved_data[1] = (float)speed_rpm * 0.9f + 0.1f * lastspeed;
    }

    // 电流 (A) - M3508标准：16384对应20A
    solved_data[2] = current_raw / 16384.0f * 20.0f;

    // 累计角度 (°) - 可超过360°，用于位置闭环
    solved_data[3] = total_round[motor_id_num] * 360.0f + solved_data[0];

    // 速度 (rad/s) - rpm转弧度/秒
    solved_data[4] = speed_rpm * 0.10472f; // 2*PI/60 ≈ 0.10472

    lastspeed = solved_data[1];
}

/*====================  辅助函数  ====================*/

/// @brief 重置电机零点（当前位置设为零点）
/// @param motor_id_num 电机ID (0~7)
void RM_Motor_Reset_Zero(uint8_t motor_id_num)
{
    if (motor_id_num < RM_MOTOR_MAX_NUM)
    {
        total_round[motor_id_num] = 0;
        total_ecd[motor_id_num] = 0;
        offset_ecd[motor_id_num] = last_ecd[motor_id_num];
    }
}

/// @brief 重置所有电机状态（用于重新初始化）
void RM_Motor_Reset_All(void)
{
    for (uint8_t i = 0; i < RM_MOTOR_MAX_NUM; i++)
    {
        last_ecd[i] = 0;
        total_round[i] = 0;
        total_ecd[i] = 0;
        init_flag[i] = 0;
        offset_ecd[i] = 0;
    }
}

/**********************************************************暴露接口，下面是外部一般用于调用的函数******************************************************/

/****************************************************
 * 函数名： RmMotorSendCfg
 * 作用：用于设置发送RM电机数据
 * 参数：motor_id：RM电机ID
 * 参数：TargetCurrent：电流大小
 * 返回值：无
 * 说明：大疆电机使用反码形式，负数需对绝对值取反
 ****************************************************/
void RmMotorSendCfg(uint8_t motor_id, int16_t TargetCurrent)
{
    if (TargetCurrent < 0)
    {
        // 负数：对绝对值取反（反码形式）
        // 例如：-400 -> ~400 = ~0x0190 = 0xFE6F
        TargetCurrent = (uint16_t)(~(-TargetCurrent));
    }
    uint8_t data[2] = {0};
    data[0] = (uint8_t)(TargetCurrent >> 8); // 高字节
    data[1] = (uint8_t)TargetCurrent;        // 低字节
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

    MotorManager.MotorList[SingleMotorTest - 1].use_cascade = 1;
    float p = 175.91f;                                                                                               // 内环p (p)
    float i = 0.40f;                                                                                                 // 内环i (i)
    float d = 0.0f;                                                                                                  // 内环d (d)
    float f = 7.91f;                                                                                                 // 内环f (f)
    PID_Init(&MotorManager.MotorList[SingleMotorTest - 1].speed_pid, PID_DELTA, p, i, d, f, 1691.0f, 100.0f, 60.0f); // 暂定最大1691
    PID_Clear(&MotorManager.MotorList[SingleMotorTest - 1].speed_pid);
    // CASCADE_PID_Init(&MotorManager.MotorList[SingleMotorTest - 1].cascade_pid, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    // CASCADE_PID_Clear(&MotorManager.MotorList[SingleMotorTest - 1].cascade_pid);

    // CAN报文头配置在CanMotor.c中的CanRegisterMotorCfg函数完成
}

/// @brief RM电机输出
/// @param target 目标值，单环时候为速度，串级为位置(暂定)，放弃串级
void RmMotorPID_Calc(float target)
{
    // char FeedString[33] = "\0";
    // PID数据输出

    // CASCADE_PID_Calculate(&MotorManager.MotorList[SingleMotorTest - 1].cascade_pid, 360.0f, rm_motor_solved_data[0], rm_motor_solved_data[1]); // 目标角度，反馈角度，反馈速度
    // sprintf((char *)FeedString, "targetAngle:.1%f, feedbackAngle:%.1f, feedbackSpeed:%.1f\r\n", 360.0f, rm_motor_solved_data[0], rm_motor_solved_data[1]);
    // printf(FeedString);

    output = PID_Calculate(&MotorManager.MotorList[SingleMotorTest - 1].speed_pid, target, rm_motor_solved_data[4]);
    RmMotorSendCfg(1, output);
    // printf("%.1f,%.1f,%.1f\r\n", rm_motor_solved_data[1], target, output);

    // HAL_UART_Transmit_IT(&huart3, );
}
