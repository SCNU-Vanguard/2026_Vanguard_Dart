#ifndef __PID_H_ /* 按 __PID_H_ 选择编译分支。 */
#define __PID_H_ /* 定义 __PID_H_。 */

#include "main.h"
#include "bsp_dwt.h"

/*********************************************************配置宏***************************************************************/

// 微分先行选择：1=启用微分先行，0=普通微分
// 微分先行：D项对测量值变化进行微分，避免目标值突变时产生尖峰
// 普通微分：D项对误差变化进行微分
#define PID_DERIVATIVE_ON_MEASUREMENT 0U /* 定义 PID_DERIVATIVE_ON_MEASUREMENT。 */

/*********************************************************类型定义*************************************************************/

// PID计算模式枚举
typedef enum /* 开始定义数据类型。 */
{
    PID_POSITION = 0, // 位置式PID
    PID_DELTA = 1     // 增量式PID
} PID_MODE_e; /* 结束 PID_MODE_e 类型定义。 */

// 注意：MotorTypeDef 在 CanMotor.h 中被 pack(1)，
// PID_t/CASCADE_PID_t 作为其成员时必须保持相同对齐策略，
// 否则在某些编译器/指令下会出现未对齐浮点访问导致硬Fault。
#pragma pack(push, 1) /* 配置编译选项 pack(push, 1)。 */
// 前馈PID结构体
// PID基本参数(包含前馈)
// 反馈值
// 输出值储存
typedef struct /* 开始定义数据类型。 */
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

} PID_t; /* 结束 PID_t 类型定义。 */

// 串级PID结构体（基于前馈PID）
typedef struct /* 开始定义数据类型。 */
{
    PID_t outer; // 外环PID（位置环）
    PID_t inner; // 内环PID（速度环）

} CASCADE_PID_t; /* 结束 CASCADE_PID_t 类型定义。 */
#pragma pack(pop) /* 配置编译选项 pack(pop)。 */

// 正弦波发生器
typedef struct /* 开始定义数据类型。 */
{
    float amplitude; // 峰值幅值
    float period;    // 周期(秒)
    float dt;        // 时间步长(秒)，对应任务调用间隔
    float angle_deg; // 当前角度 [0, 360)
    float output;    // 当前输出值
} SineWaveGen_t; /* 结束 SineWaveGen_t 类型定义。 */

// 梯形波发生器（线性升降，到达幅值反向，min = -max）
typedef struct /* 开始定义数据类型。 */
{
    float amplitude;  // 峰值幅值（最小值 = -amplitude）
    float gradient;   // 每次调用的变化量（梯度）
    int8_t direction; // 当前方向: +1 上升, -1 下降
    float output;     // 当前输出值
} TrapWaveGen_t; /* 结束 TrapWaveGen_t 类型定义。 */

/*********************************************************函数声明***************************************************************/

// PID初始化函数
void PID_Init(PID_t *pid, PID_MODE_e mode, float kp, float ki, float kd, float kf, float max_out, float min_out, float max_iout); /* 声明 PID_Init 接口。 */

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
void CASCADE_PID_Init(CASCADE_PID_t *cascade_pid, /* 传入下一项参数或数据。 */
                      float outer_kp, float outer_ki, float outer_kd, float outer_kf, /* 传入下一项参数或数据。 */
                      float inner_kp, float inner_ki, float inner_kd, float inner_kf, /* 传入下一项参数或数据。 */
                      float outer_max_out, float outer_min_out, float outer_max_iout, /* 传入下一项参数或数据。 */
                      float inner_max_out, float inner_min_out, float inner_max_iout); /* 完成本行操作。 */

// PID计算函数（统一接口，根据mode自动选择位置式或增量式）
float PID_Calculate(PID_t *pid, float target, float measure); /* 声明 PID_Calculate 接口。 */

// 位置式PID计算
float PID_Position_Calc(PID_t *pid, float target, float measure); /* 声明 PID_Position_Calc 接口。 */

// 增量式PID计算
float PID_Incremental_Calc(PID_t *pid, float target, float measure); /* 声明 PID_Incremental_Calc 接口。 */

/**
 * @brief 串级PID计算
 * @param cascade_pid 串级PID结构体指针
 * @param outer_target 外环目标值（位置）
 * @param outer_measure 外环测量值（位置）
 * @param inner_measure 内环测量值（速度）
 * @return 内环PID输出值
 */
float CASCADE_PID_Calculate(CASCADE_PID_t *cascade_pid, /* 传入下一项参数或数据。 */
                            float outer_target, /* 传入下一项参数或数据。 */
                            float outer_measure, /* 传入下一项参数或数据。 */
                            float inner_measure); /* 完成本行操作。 */

// PID清零函数（清除所有误差和输出）
void PID_Clear(PID_t *pid); /* 声明 PID_Clear 接口。 */

// PID积分项清零函数
void PID_Clear_Integral(PID_t *pid); /* 声明 PID_Clear_Integral 接口。 */

// 串级PID清零
void CASCADE_PID_Clear(CASCADE_PID_t *cascade_pid); /* 声明 CASCADE_PID_Clear 接口。 */

// 串级PID积分项清零
void CASCADE_PID_Clear_Integral(CASCADE_PID_t *cascade_pid); /* 声明 CASCADE_PID_Clear_Integral 接口。 */

// PID参数设置函数
void PID_Set_Coefficient(PID_t *pid, float kp, float ki, float kd, float kf); /* 声明 PID_Set_Coefficient 接口。 */
void PID_Set_OutputLimit(PID_t *pid, float max_output, float min_output, float max_iout); /* 声明 PID_Set_OutputLimit 接口。 */

// 检查PID是否已初始化
uint8_t PID_Is_Initialized(PID_t *pid); /* 声明 PID_Is_Initialized 接口。 */

// 正弦波发生器初始化
void SineWave_Init(SineWaveGen_t *wave, float amplitude, float period, float dt); /* 声明 SineWave_Init 接口。 */
// 正弦波计算（每次调用推进一步，返回当前输出）
float SineWave_Calc(SineWaveGen_t *wave); /* 声明 SineWave_Calc 接口。 */
// 运行时修改正弦波幅值和周期
void SineWave_Set(SineWaveGen_t *wave, float amplitude, float period); /* 声明 SineWave_Set 接口。 */

// 梯形波发生器初始化
void TrapWave_Init(TrapWaveGen_t *trap, float amplitude, float gradient); /* 声明 TrapWave_Init 接口。 */
// 梯形波计算（每次调用推进一步，返回当前输出）
float TrapWave_Calc(TrapWaveGen_t *trap); /* 声明 TrapWave_Calc 接口。 */
// 运行时修改梯形波幅值和梯度
void TrapWave_Set(TrapWaveGen_t *trap, float amplitude, float gradient); /* 声明 TrapWave_Set 接口。 */

/*********************************************************波形任务***************************************************************/

// 波形输出变量（供外部读取）
extern float sine_output; /* 声明外部变量 sine_output。 */
extern float trap_output; /* 声明外部变量 trap_output。 */

// 波形参数变量（供外部运行时调节）
extern SineWaveGen_t g_SineWave; /* 声明外部变量 g_SineWave。 */
extern TrapWaveGen_t g_TrapWave; /* 声明外部变量 g_TrapWave。 */

// 正弦波FreeRTOS任务函数
void SineWaveTask(void *argument); /* 声明 SineWaveTask 接口。 */
// 梯形波FreeRTOS任务函数
void TrapWaveTask(void *argument); /* 声明 TrapWaveTask 接口。 */

#endif /* 结束条件编译。 */
