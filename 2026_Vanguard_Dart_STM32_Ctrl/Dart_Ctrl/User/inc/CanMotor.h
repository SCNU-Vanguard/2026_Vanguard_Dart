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
    DM_MOTOR_BAND
} can_motor_band;

#define SingleMotorTest 1

// CAN线挂载的电机（包含RM和DM电机）
typedef enum
{
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
    uint8_t MotorBand;
    uint8_t (*SendMotorControl)(struct _MotorTypeDef *st);
    uint8_t ReceiveMotorData[8];    // 电机接收数据存储
    uint8_t SendMotorData[8];       // 电机发送数据存储
    CAN_TxHeaderTypeDef g_TxHeader; // 电机发送报文头

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
