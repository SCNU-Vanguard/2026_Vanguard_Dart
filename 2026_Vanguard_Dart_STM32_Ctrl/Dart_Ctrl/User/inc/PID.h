#ifndef __PID_H_
#define __PID_H_

#include "main.h"
#include "bsp_dwt.h"

/*********************************************************配置宏***************************************************************/

// 微分先行选择：1=启用微分先行，0=普通微分
// 微分先行：D项对测量值变化进行微分，避免目标值突变时产生尖峰
// 普通微分：D项对误差变化进行微分
#define PID_DERIVATIVE_ON_MEASUREMENT 0U

/*********************************************************类型定义*************************************************************/

// PID计算模式枚举
typedef enum
{
    PID_POSITION = 0, // 位置式PID
    PID_DELTA = 1     // 增量式PID
} PID_MODE_e;

// 前馈PID结构体
// PID基本参数(包含前馈)
// 反馈值
// 输出值储存
typedef struct
{
    // 初始化标志位
    uint8_t initialized;       // 是否已初始化
    uint8_t calc_count;        // 计算次数计数器（用于首次计算判断）
    uint8_t direction_changed; // 换向标志位（用于换向补偿）

    // PID工作模式
    PID_MODE_e mode; // 位置式或增量式

    // PID基本参数(包含前馈)
    float KP; // 比例系数
    float KI; // 积分系数
    float KD; // 微分系数
    float KF; // 前馈系数

    // 限幅参数
    float max_output; // 输出上限（实际输出范围为 [-max_output, +max_output]）
    float max_iout;   // 积分限幅

    // 反馈值
    float target;       // 目标值
    float measure;      // 测量值（反馈值）
    float last_measure; // 上次测量值（微分先行用）
    float prev_measure; // 上上次测量值（增量式微分先行用）
    float error;        // 当前误差
    float last_error;   // 上次误差
    float prev_error;   // 上上次误差（增量式用）
    float sum_error;    // 误差积分

    // 前馈值
    float feedforward; // 前馈输入值
    float last_target; // 上一次目标值（用于换向检测）

    // 输出值储存
    float output;      // 当前输出值
    float last_output; // 上次输出值
    float delta_out;   // 增量输出（增量式用）

} PID_t;

// 串级PID结构体（基于前馈PID）
typedef struct
{
    PID_t outer; // 外环PID（位置环）
    PID_t inner; // 内环PID（速度环）

} CASCADE_PID_t;

/*********************************************************函数声明***************************************************************/

// PID初始化函数
void PID_Init(PID_t *pid, PID_MODE_e mode, float kp, float ki, float kd, float kf, float max_out, float max_iout);

// 串级PID初始化
void CASCADE_PID_Init(CASCADE_PID_t *cascade_pid,
                      float outer_kp, float outer_ki, float outer_kd, float outer_kf,
                      float inner_kp, float inner_ki, float inner_kd, float inner_kf,
                      float outer_max_out, float outer_max_iout,
                      float inner_max_out, float inner_max_iout);

// PID计算函数（统一接口，根据mode自动选择位置式或增量式）
float PID_Calculate(PID_t *pid, float target, float measure);

// 位置式PID计算
float PID_Position_Calc(PID_t *pid, float target, float measure);

// 增量式PID计算
float PID_Incremental_Calc(PID_t *pid, float target, float measure);

// 串级PID计算
float CASCADE_PID_Calculate(CASCADE_PID_t *cascade_pid,
                            float outer_target,
                            float outer_measure,
                            float inner_measure);

// PID清零函数（清除所有误差和输出）
void PID_Clear(PID_t *pid);

// PID积分项清零函数
void PID_Clear_Integral(PID_t *pid);

// 串级PID清零
void CASCADE_PID_Clear(CASCADE_PID_t *cascade_pid);

// 串级PID积分项清零
void CASCADE_PID_Clear_Integral(CASCADE_PID_t *cascade_pid);

// PID参数设置函数
void PID_Set_Coefficient(PID_t *pid, float kp, float ki, float kd, float kf);
void PID_Set_OutputLimit(PID_t *pid, float max_output, float max_iout);

// 检查PID是否已初始化
uint8_t PID_Is_Initialized(PID_t *pid);

#endif
