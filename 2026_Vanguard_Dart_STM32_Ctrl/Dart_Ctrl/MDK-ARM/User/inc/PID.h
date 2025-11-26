#ifndef __PID_H_
#define __PID_H_

#include "main.h"
#include "bsp_dwt.h"

// PID计算模式枚举
typedef enum
{
    PID_POSITION = 0,  // 位置式PID
    PID_DELTA = 1      // 增量式PID
} PID_MODE_e;

// PID配置参数结构体
typedef struct
{
    PID_MODE_e mode;   // PID模式
    float kp;          // 比例系数
    float ki;          // 积分系数
    float kd;          // 微分系数
    float kf;          // 前馈系数
    float max_out;     // 输出限幅
    float max_iout;    // 积分限幅
} PID_Config_t;

// 前馈PID结构体
// PID基本参数(包含前馈)
// 反馈值
// 输出值储存
typedef struct
{
    // PID基本参数(包含前馈)
    float KP;          // 比例系数
    float KI;          // 积分系数
    float KD;          // 微分系数
    float KF;          // 前馈系数
    
    // 反馈值
    float target;      // 目标值
    float measure;     // 测量值（反馈值）
    float error;       // 当前误差
    float last_error;  // 上次误差
    float prev_error;  // 上上次误差（增量式用）
    float sum_error;   // 误差积分
    
    // 前馈值
    float feedforward; // 前馈输入值
    
    // 输出值储存
    float output;      // 当前输出值
    float last_output; // 上次输出值
    float delta_out;   // 增量输出（增量式用）
    
    // 限幅参数
    float max_output;  // 输出限幅
    float max_iout;    // 积分限幅
    
    // PID工作模式
    PID_MODE_e mode;   // 位置式或增量式
    
} PID_t;

// 串级PID结构体（基于前馈PID）
typedef struct
{
    PID_t outer;       // 外环PID（位置环）
    PID_t inner;       // 内环PID（速度环）
    
} CASCADE_PID_t;

/*********************************************************函数声明***************************************************************/

// PID初始化函数（传统方式）
void PID_Init(PID_t *pid, PID_MODE_e mode, float kp, float ki, float kd, float kf, float max_out, float max_iout);

// PID初始化函数（使用配置结构体）
void PID_Init_Config(PID_t *pid, const PID_Config_t *config);

// 串级PID初始化（传统方式）
void CASCADE_PID_Init(CASCADE_PID_t *cascade_pid,
                      float outer_kp, float outer_ki, float outer_kd, float outer_kf,
                      float inner_kp, float inner_ki, float inner_kd, float inner_kf,
                      float outer_max_out, float outer_max_iout,
                      float inner_max_out, float inner_max_iout);

// 串级PID初始化（使用配置结构体 - 推荐）
void CASCADE_PID_Init_Config(CASCADE_PID_t *cascade_pid, 
                             const PID_Config_t *outer_config, 
                             const PID_Config_t *inner_config);

// PID计算函数（统一接口，根据mode自动选择位置式或增量式）
float PID_Calculate(PID_t *pid, float target, float measure, float feedforward);

// 位置式PID计算
float PID_Position_Calc(PID_t *pid, float target, float measure, float feedforward);

// 增量式PID计算
float PID_Incremental_Calc(PID_t *pid, float target, float measure, float feedforward);

// 串级PID计算
float CASCADE_PID_Calculate(CASCADE_PID_t *cascade_pid, 
                           float outer_target, 
                           float outer_measure, 
                           float inner_measure,
                           float outer_feedforward,
                           float inner_feedforward);

// PID清零函数（清除所有误差和输出）
void PID_Clear(PID_t *pid);

// PID积分项清零函数
void PID_Clear_Integral(PID_t *pid);

// 串级PID清零
void CASCADE_PID_Clear(CASCADE_PID_t *cascade_pid);

// 串级PID积分项清零
void CASCADE_PID_Clear_Integral(CASCADE_PID_t *cascade_pid);

// PID参数设置函数
void PID_Set_Param(PID_t *pid, float kp, float ki, float kd, float kf);
void PID_Set_MaxOutput(PID_t *pid, float max_output, float max_iout);

#endif
