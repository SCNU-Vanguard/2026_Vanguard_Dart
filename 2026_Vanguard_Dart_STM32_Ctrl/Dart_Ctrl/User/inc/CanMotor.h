#ifndef __CAN_MOTOR_H_
#define __CAN_MOTOR_H_

// 通用CAN电机管理层
// 负责统一管理RM电机和DM电机

#include "main.h"
#include "bsp_can.h"
#include "PID.h"
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define CtrlMotorLen 8 // 电机控制报文长度默认给8
#define TestUse 0U

// 角度转弧度宏：degree * π/180 ≈ degree * 0.01745329f
#define DegreeToRad(degree) ((degree) * 0.01745329f)

// 弧度转角度宏：radian * 180/π ≈ radian * 57.29578f
#define RadToDegree(radian) ((radian) * 57.29578f)

/*============================== 前向声明 ==============================*/

struct _RM_MotorClass; // RM电机类前向声明
struct _DM_MotorClass; // DM电机类前向声明（预留扩展）

/*============================== 硬件抽象层 (HAL) ==============================*/

/// @brief 电机硬件抽象层结构体
/// @note 用于解耦驱动层与具体硬件实现，便于移植和测试
typedef struct
{
    /// @brief CAN数据发送函数
    /// @param hdr CAN报文头指针
    /// @param data 数据指针
    /// @return 1-成功，0-失败
    uint8_t (*can_send)(CAN_TxHeaderTypeDef *hdr, uint8_t *data);

    /// @brief 延时函数（毫秒）
    /// @param ms 延时时间（毫秒）
    void (*delay_ms)(uint32_t ms);
} MotorHAL_t;

/// @brief 获取当前电机HAL接口指针
/// @return HAL接口指针
const MotorHAL_t *Motor_GetHAL(void);

/// @brief 设置电机HAL接口（可选，用于测试或自定义硬件）
/// @param hal HAL接口指针，传NULL恢复默认
void Motor_SetHAL(const MotorHAL_t *hal);

// HAL指针（供内联函数使用）
extern const MotorHAL_t *g_pMotorHAL;

/// @brief 获取当前电机HAL接口（内联版本，零开销）
/// @return HAL接口指针
static inline const MotorHAL_t *Motor_GetHAL_Fast(void)
{
    return g_pMotorHAL;
}

/*============================== 电机类型枚举 ==============================*/

// 电机品牌
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
    DmS3519,
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
    int16_t last_ecd;       // 上次编码器值
    int32_t total_round;    // 累计圈数
    int32_t total_ecd;      // 累计编码器值
    int16_t offset_ecd;     // 零点偏移
    uint8_t init_flag;      // 初始化标志
    float offset_ecd_angle; // 零点偏移角度

    // 速度滤波相关
    float last_speed;    // 上次速度值（用于滤波）
    uint8_t filter_init; // 滤波器初始化标志

    // 控制相关变量
    float target_angle;       // 目标角度
    float last_target;        // 上次目标值
    float pre_last_target;    // 上上次目标值
    uint8_t target_init_flag; // 目标值初始化标志（首次调用RmMotorRemoveBias时置1）
} MotorSolvedData_t;

// CAN线挂载的电机（包含RM和DM电机）
typedef enum
{
    SingleMotorTest = 1,
    RM_3508_GRIPPER = 1,
    RM_2006_TRIGGER,
    DM_3519_STRENTH_LEFT,
    DM_3519_STRENTH_RIGHT,
    DM_4310_YAW
} can_motor_cfg;

// 电机配置结构体（用户可调参数）
typedef struct
{
    float direction_bias;     // 换向偏移补偿(°)
    float position_tolerance; // 位置误差容限(°)
    uint8_t reverse;          // 是否反向: 0-正向, 1-反向
} MotorConfig_t;

// 电机参数结构体（只读，由型号决定）
typedef struct
{
    float gear_ratio;    // 减速比
    int16_t max_current; // 最大电流
    float current_ratio; // 电流转换系数
} MotorParams_t;

// 电机结构体定义
typedef struct _MotorTypeDef
{
    uint8_t MotorID;
    motor_inf MotorInf;
    uint8_t ReceiveMotorData[8];    // 电机接收数据存储
    uint8_t SendMotorData[8];       // 电机发送数据存储
    CAN_TxHeaderTypeDef g_TxHeader; // 电机发送报文头

    // 电机反馈数据解算存储（每个电机独立）
    MotorSolvedData_t motor_data; // 电机解算数据

    // 电机配置和参数（面向对象扩展）
    MotorConfig_t config; // 用户可调配置
    MotorParams_t params; // 电机参数（只读）

    // ==================== 面向对象扩展 ====================
    // 电机类指针（指向类型定义，包含虚函数表和默认参数）
    // 推荐使用：通过 motor_class 访问虚函数，如 motor->motor_class.rm_motor_class->calculate(motor)
    union
    {
        const struct _RM_MotorClass *rm_motor_class; // RM电机类指针
        const struct _DM_MotorClass *dm_motor_class; // DM电机类指针
    } motor_class;

    // ==================== 弃用的函数指针 ====================
    // 以下函数指针保留用于向后兼容，新代码请使用 motor_class 虚函数表
    // @deprecated 使用 RM_Motor_SendControl() 或 DM_Motor_SendControl() 代替
    uint8_t (*SendMotorControl)(struct _MotorTypeDef *st);
    // @deprecated 使用 RM_Motor_Calculate() 或 DM_Motor_Calculate() 代替
    void (*calculate)(struct _MotorTypeDef *self);

    // PID控制器（可选择单环或串级）
    PID_t inner_pid;           // 内环PID（单环控制时使用）
    CASCADE_PID_t cascade_pid; // 串级PID
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

/// @brief 获取电机句柄（内联版本，零开销）
/// @param motor_id 电机ID（can_motor_cfg枚举值）
/// @return 电机结构体指针，ID无效时返回NULL
static inline MotorTypeDef *Motor_GetHandle(can_motor_cfg motor_id)
{
    if (motor_id < 1 || motor_id > g_CanMotorNum)
        return NULL;
    return &MotorManager.MotorList[motor_id - 1];
}

/// @brief 获取电机句柄（无检查版本，最高性能）
/// @param motor_id 电机ID（调用者需确保有效性）
/// @return 电机结构体指针
static inline MotorTypeDef *Motor_GetHandleFast(can_motor_cfg motor_id)
{
    return &MotorManager.MotorList[motor_id - 1];
}

/// @brief 获取RM电机发送缓冲区（内联版本）
/// @return RM电机发送数据数组指针
static inline uint8_t *Motor_GetRmSendBuffer(void)
{
    return MotorManager.RM_MOTOR_DATA_ARRAY;
}

/// @brief 设置已注册电机数量（仅供测试使用）
/// @param count 电机数量
void Motor_SetRegisteredCount(uint8_t count);

/*********************************************************电机数据读取接口***************************************************************/

/// @brief 获取电机累计角度（单位：度）
/// @param motor_id 电机ID（can_motor_cfg枚举值）
/// @return 累计角度（度），失败返回0
float Motor_GetTotalAngle(can_motor_cfg motor_id);

/// @brief 获取电机累计角度（单位：弧度）
/// @param motor_id 电机ID（can_motor_cfg枚举值）
/// @return 累计角度（弧度），失败返回0
float Motor_GetTotalAngleRad(can_motor_cfg motor_id);

/// @brief 获取电机速度（单位：rpm）
/// @param motor_id 电机ID（can_motor_cfg枚举值）
/// @return 速度（rpm），失败返回0
float Motor_GetSpeedRPM(can_motor_cfg motor_id);

/// @brief 获取电机速度（单位：rad/s）
/// @param motor_id 电机ID（can_motor_cfg枚举值）
/// @return 速度（rad/s），失败返回0
float Motor_GetSpeedRadS(can_motor_cfg motor_id);

/// @brief 获取电机单圈角度（单位：度）
/// @param motor_id 电机ID（can_motor_cfg枚举值）
/// @return 单圈角度（度），失败返回0
float Motor_GetSingleAngle(can_motor_cfg motor_id);

/// @brief 获取电机电流（单位：A）
/// @param motor_id 电机ID（can_motor_cfg枚举值）
/// @return 电流（A），DM电机返回力矩(N·m)，失败返回0
float Motor_GetCurrent(can_motor_cfg motor_id);

/// @brief 获取电机所有解算数据
/// @param motor_id 电机ID（can_motor_cfg枚举值）
/// @param data 输出数据指针（需提供5个float空间）
/// @return true-成功，false-失败
/// @note RM电机: [0]单圈角度(°), [1]速度(rpm), [2]电流(A), [3]累计角度(°), [4]速度(rad/s)
///       DM电机: [0]位置(rad), [1]速度(rad/s), [2]力矩(N·m), [3]MOS温度(℃), [4]转子温度(℃)
bool Motor_GetAllData(can_motor_cfg motor_id, float *data);

#endif
