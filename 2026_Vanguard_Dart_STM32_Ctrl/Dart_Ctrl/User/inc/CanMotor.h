#ifndef __CAN_MOTOR_H_
#define __CAN_MOTOR_H_

// 通用CAN电机管理层
// 负责统一管理RM电机和DM电机

#include "main.h"
#include "bsp_can.h"
#include "PID.h"
#include <stdlib.h>
#include <string.h>

#define CtrlMotorLen 8 // 电机控制报文长度默认给8
#define TestUse 1U

// 角度转弧度宏：degree * π/180 ≈ degree * 0.01745329f
#define DegreeToRad(degree) ((degree) * 0.01745329f)

// 弧度转角度宏：radian * 180/π ≈ radian * 57.29578f
#define RadToDegree(radian) ((radian) * 57.29578f)

// 电机品牌类型
typedef enum
{
    RM_MOTOR_BAND = 0,
    DM_MOTOR_BAND,
    CubeMars_MOTOR_BAND
} can_motor_band;

// 电机型号
typedef enum
{
    RmM2006 = 1,
    RmM3508,
    RmGM6020,
    DmS3510,
    DmJ4310,
    CmG80
} can_motor_model;

typedef struct
{
    can_motor_band band;
    can_motor_model model;
} motor_inf;

// 电机解算数据存储结构体（用于存储反馈解算后的数据）
// RM电机: [0]单圈角度(°), [1]速度(rpm), [2]电流(A), [3]累计角度(°), [4]速度(rad/s)
// DM电机: [0]位置(rad/°), [1]速度(rad/s), [2]力矩(N·m), [3]MOS温度(℃), [4]转子温度(℃)
#define MOTOR_SOLVED_DATA_NUM 5
typedef struct
{
    float solved_data[MOTOR_SOLVED_DATA_NUM]; // 解算后的数据数组

    // 电机使用的多圈累计相关变量
    int16_t last_ecd;    // 上次编码器值
    int32_t total_round; // 累计圈数
    int32_t total_ecd;   // 累计编码器值
    int16_t offset_ecd;  // 零点偏移
    uint8_t init_flag;   // 初始化标志

    // 速度滤波相关
    float last_speed;    // 上次速度值（用于滤波）
    uint8_t filter_init; // 滤波器初始化标志

    // 控制相关变量
    float target_angle;    // 目标角度
    float last_target;     // 上次目标值
    float pre_last_target; // 上上次目标值
} MotorSolvedData_t;

// CAN线挂载的电机（包含RM和DM电机）
typedef enum
{
    SingleMotorTest = 1,
    RM_3508_GRIPPER = 1,
    RM_2006_TRIGGER,
    DM_3510_STRENTH_LEFT,
    DM_3510_STRENTH_RIGHT,
    DM_4310_YAW
} can_motor_cfg;

// 电机结构体定义
typedef struct _MotorTypeDef
{
    uint8_t MotorID;
    motor_inf MotorInf;
    uint8_t (*SendMotorControl)(struct _MotorTypeDef *st);
    uint8_t ReceiveMotorData[8];    // 电机接收数据存储
    uint8_t SendMotorData[8];       // 电机发送数据存储
    CAN_TxHeaderTypeDef g_TxHeader; // 电机发送报文头

    // 电机反馈数据解算存储（每个电机独立）
    MotorSolvedData_t motor_data; // 电机解算数据

    // PID控制器（可选择单环或串级）
    PID_t speed_pid;           // 速度环PID（单环控制时使用）
    CASCADE_PID_t cascade_pid; // 串级PID（位置-速度双环控制时使用）
    uint8_t use_cascade;       // 是否使用串级控制：0-单环，1-串级
} MotorTypeDef;

// 电机管理器结构体
typedef struct
{
    MotorTypeDef MotorList[g_CanMotorNum];
    // 记录当前已注册的数量
    uint8_t registered_count;
    uint8_t RM_MOTOR_DATA_ARRAY[8]; // 电机列表发送数据的数组，每次发送RM电机的控制数据发送的都是这个arr
} MotorManager_t;

extern MotorManager_t MotorManager;

/*********************************************************函数声明***************************************************************/

/// @brief 上电之后的电机注册、CAN初始化等等
/// @param 无
void MotorInit(void);

/// @brief 注册所有电机的信息
/// @param 无（按照已经配置的电机表注册）
/// @return 注册是否成功
void MotorRegister(void);

/// @brief 注册电机的CAN通信信息
/// @param ptr 电机初始化指针
/// @note 包含报文头的初始化
/// @return 无
void CanRegisterMotorCfg(MotorTypeDef *ptr);

/// @brief CAN过滤器的再初始化，其实都是因为强迫症所以只过滤了这个
/// @param 无
/// @note 默认接收全部数据,调用该函数之后将只接收达妙MIT模式的反馈和RM电机的反馈帧
///       但是要对比一下ID
void CanFliterCfg(void);

/// @brief CAN FIFO中断回调处理函数
/// @param fifo_num FIFO的对应号
/// @param hcan 处理时候的can句柄
/// @param FIFOmessageNum 要处理的消息数量
/// @return 无
void CAN_FIFO_CBKHANDLER(uint32_t fifo_num, uint8_t FIFOmessageNum);

/// @brief 获取电机管理器指针
/// @return 电机管理器结构体
MotorManager_t GetPtrMotorManager(void);

#endif
