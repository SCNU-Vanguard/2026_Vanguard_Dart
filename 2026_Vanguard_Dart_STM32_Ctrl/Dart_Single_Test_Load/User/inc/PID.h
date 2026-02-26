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

// 注意：MotorTypeDef 在 CanMotor.h 中被 pack(1)，
// PID_t/CASCADE_PID_t 作为其成员时必须保持相同对齐策略，
// 否则在某些编译器/指令下会出现未对齐浮点访问导致硬Fault。
#pragma pack(push, 1)
// 前馈PID结构体
// PID基本参数(包含前馈)
// 反馈值
// 输出值储存
typedef struct
{
    // 初始化标志位
    uint8_t initialized; // 是否已初始化
    uint8_t calc_count;  // 计算次数计数器（用于首次计算判断）

    // PID工作模式
    PID_MODE_e mode; // 位置式或增量式

    // PID基本参数(包含前馈)
    float KP; // 比例系数
    float KI; // 积分系数
    float KD; // 微分系数
    float KF; // 前馈系数

    // 限幅参数
    float max_output; // 输出上限（实际输出范围为 [-max_output, +max_output]）
    float min_output; // 最小输出幅值（按误差方向生效，用于克服静摩擦/死区）
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
#pragma pack(pop)

// 正弦波发生器
typedef struct
{
    float amplitude; // 峰值幅值
    float period;    // 周期(秒)
    float dt;        // 时间步长(秒)，对应任务调用间隔
    float angle_deg; // 当前角度 [0, 360)
    float output;    // 当前输出值
} SineWaveGen_t;

// 梯形波发生器（线性升降，到达幅值反向，min = -max）
typedef struct
{
    float amplitude;  // 峰值幅值（最小值 = -amplitude）
    float gradient;   // 每次调用的变化量（梯度）
    int8_t direction; // 当前方向: +1 上升, -1 下降
    float output;     // 当前输出值
} TrapWaveGen_t;

/*********************************************************函数声明***************************************************************/

// PID初始化函数
void PID_Init(PID_t *pid, PID_MODE_e mode, float kp, float ki, float kd, float kf, float max_out, float min_out, float max_iout);

/**
 * @brief 串级PID初始化函数
 * @param cascade_pid 串级PID结构体指针
 * @param outer_kp 外环比例系数
 * @param outer_ki 外环积分系数
 * @param outer_kd 外环微分系数
 * @param outer_kf 外环前馈系数
 * @param inner_kp 内环比例系数
 * @param inner_ki 内环积分系数
 * @param inner_kd 内环微分系数
 * @param inner_kf 内环前馈系数
 * @param outer_max_out 外环输出上限
 * @param outer_min_out 外环最小输出幅值（按误差方向生效）
 * @param outer_max_iout 外环积分限幅
 * @param inner_max_out 内环输出上限
 * @param inner_min_out 内环最小输出幅值（按误差方向生效）
 * @param inner_max_iout 内环积分限幅
 */
void CASCADE_PID_Init(CASCADE_PID_t *cascade_pid,
                      float outer_kp, float outer_ki, float outer_kd, float outer_kf,
                      float inner_kp, float inner_ki, float inner_kd, float inner_kf,
                      float outer_max_out, float outer_min_out, float outer_max_iout,
                      float inner_max_out, float inner_min_out, float inner_max_iout);

// PID计算函数（统一接口，根据mode自动选择位置式或增量式）
float PID_Calculate(PID_t *pid, float target, float measure);

// 位置式PID计算
float PID_Position_Calc(PID_t *pid, float target, float measure);

// 增量式PID计算
float PID_Incremental_Calc(PID_t *pid, float target, float measure);

/**
 * @brief 串级PID计算
 * @param cascade_pid 串级PID结构体指针
 * @param outer_target 外环目标值（位置）
 * @param outer_measure 外环测量值（位置）
 * @param inner_measure 内环测量值（速度）
 * @return 内环PID输出值
 */
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
void PID_Set_OutputLimit(PID_t *pid, float max_output, float min_output, float max_iout);

// 检查PID是否已初始化
uint8_t PID_Is_Initialized(PID_t *pid);

// 正弦波发生器初始化
void SineWave_Init(SineWaveGen_t *wave, float amplitude, float period, float dt);
// 正弦波计算（每次调用推进一步，返回当前输出）
float SineWave_Calc(SineWaveGen_t *wave);
// 运行时修改正弦波幅值和周期
void SineWave_Set(SineWaveGen_t *wave, float amplitude, float period);

// 梯形波发生器初始化
void TrapWave_Init(TrapWaveGen_t *trap, float amplitude, float gradient);
// 梯形波计算（每次调用推进一步，返回当前输出）
float TrapWave_Calc(TrapWaveGen_t *trap);
// 运行时修改梯形波幅值和梯度
void TrapWave_Set(TrapWaveGen_t *trap, float amplitude, float gradient);

/*********************************************************波形任务***************************************************************/

// 波形输出变量（供外部读取）
extern float sine_output;
extern float trap_output;

// 波形参数变量（供外部运行时调节）
extern SineWaveGen_t g_SineWave;
extern TrapWaveGen_t g_TrapWave;

// 正弦波FreeRTOS任务函数
void SineWaveTask(void *argument);
// 梯形波FreeRTOS任务函数
void TrapWaveTask(void *argument);

#endif
