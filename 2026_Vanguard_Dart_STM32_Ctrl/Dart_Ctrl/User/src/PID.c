#include "PID.h"
#include "CanMotor.h"
#include <string.h>
#include <arm_math.h>
#include "cmsis_os.h"

/**
 * @brief PID初始化函数
 * @param pid PID结构体指针
 * @param mode PID模式（位置式或增量式）
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param kf 前馈系数
 * @param max_out 输出上限（实际输出范围为 [-max_out, +max_out]）
 * @param min_out 最小输出幅值（按误差方向生效，用于克服静摩擦/死区）
 * @param max_iout 积分限幅
 */
void PID_Init(PID_t *pid, PID_MODE_e mode, float kp, float ki, float kd, float kf, float max_out, float min_out, float max_iout) /* 实现 PID_Init。 */
{
    if (pid == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    // 清零结构体
    memset(pid, 0, sizeof(PID_t)); /* 调用 memset。 */

    // 设置参数
    pid->mode = mode; /* 更新 mode。 */
    pid->KP = kp; /* 更新 KP。 */
    pid->KI = ki; /* 更新 KI。 */
    pid->KD = kd; /* 更新 KD。 */
    pid->KF = kf; /* 更新 KF。 */
    pid->max_output = max_out; /* 更新 max_output。 */
    pid->min_output = min_out; /* 更新 min_output。 */
    pid->max_iout = max_iout; /* 更新 max_iout。 */

    // 设置初始化标志位
    pid->initialized = 1; /* 更新 initialized。 */
}

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
                      float inner_max_out, float inner_min_out, float inner_max_iout) /* 继续当前语句。 */
{
    if (cascade_pid == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    // 初始化外环（位置环，使用位置式PID，D项更稳定）
    PID_Init(&cascade_pid->outer, PID_POSITION, outer_kp, outer_ki, outer_kd, outer_kf, /* 传入下一项参数或数据。 */
             outer_max_out, outer_min_out, outer_max_iout); /* 完成本行操作。 */

    // 初始化内环（速度环，使用位置式PID，避免增量式P项对误差突变的过激响应）
    PID_Init(&cascade_pid->inner, PID_POSITION, inner_kp, inner_ki, inner_kd, inner_kf, /* 传入下一项参数或数据。 */
             inner_max_out, inner_min_out, inner_max_iout); /* 完成本行操作。 */
}

/**
 * @brief 限幅函数
 * @param value 待限幅的值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的值
 */
static inline float float_constrain(float value, float min, float max) /* 实现 float_constrain。 */
{
    if (value < min) /* 检查当前执行条件。 */
        return min; /* 返回当前计算结果。 */
    else if (value > max) /* 继续判断下一条件。 */
        return max; /* 返回当前计算结果。 */
    else /* 处理其余情况。 */
        return value; /* 返回当前计算结果。 */
}

/**
 * @brief 最小输出保护（按误差方向）
 * @param output 当前输出
 * @param min_output 最小输出幅值
 * @param error 当前误差
 * @return 处理后的输出
 * @note 仅当输出方向与纠偏方向一致时，才抬升到最小输出；不会强行翻转输出方向
 */
static inline float apply_min_output_by_error(float output, float min_output, float error) /* 实现 apply_min_output_by_error。 */
{
    if (min_output <= 0.0f) /* 检查当前执行条件。 */
        return output; /* 返回当前计算结果。 */

    if (error > 0.0f) /* 检查当前执行条件。 */
    {
        if (output > 0.0f && output < min_output) /* 检查当前执行条件。 */
            return min_output; /* 返回当前计算结果。 */
    }
    else if (error < 0.0f) /* 继续判断下一条件。 */
    {
        if (output < 0.0f && output > -min_output) /* 检查当前执行条件。 */
            return -min_output; /* 返回当前计算结果。 */
    }

    return output; /* 返回当前计算结果。 */
}

/**
 * @brief 位置式PID计算
 * @param pid PID结构体指针
 * @param target 目标值
 * @param measure 测量值
 * @return PID输出值
 * @note 第一次计算只使用P项，第二次及之后使用完整P、I、D、F
 */
float PID_Position_Calc(PID_t *pid, float target, float measure) /* 实现 PID_Position_Calc。 */
{
    if (pid == NULL || !pid->initialized) /* 检查当前执行条件。 */
        return 0.0f; /* 返回当前计算结果。 */

    // 更新目标值、测量值
    float last_target = pid->target; /* 初始化 last_target。 */

    pid->target = target; /* 更新 target。 */
    pid->measure = measure; /* 更新 measure。 */

    // 计算误差
    pid->error = target - measure; /* 更新 error。 */

    // 第一次计算：只使用P项
    if (pid->calc_count == 0) /* 检查当前执行条件。 */
    {
        float p_out = pid->KP * pid->error; /* 初始化 p_out。 */

        pid->output = p_out; /* 更新 output。 */

        // 输出限幅（正负对称限幅）
        pid->output = float_constrain(pid->output, -pid->max_output, pid->max_output); /* 更新 output。 */

        // 更新上次误差、测量值
        pid->last_error = pid->error; /* 更新 last_error。 */
        pid->last_measure = measure; /* 更新 last_measure。 */

        // 计算前馈值（目标值变化量）
        pid->feedforward = 0.0f; // 第一次无前馈

        // 增加计算次数
        pid->calc_count = 1; /* 更新 calc_count。 */

        return pid->output; /* 返回当前计算结果。 */
    }

    // 计算前馈值（目标值变化量）
    pid->feedforward = target - last_target; /* 更新 feedforward。 */

    // 第二次及之后：使用完整P、I、D、F
    // 积分项累加（带限幅）
    pid->sum_error += pid->error; /* 更新 sum_error。 */
    if (pid->KI != 0) /* 检查当前执行条件。 */
    {
        pid->sum_error = float_constrain(pid->sum_error, -pid->max_iout / pid->KI, pid->max_iout / pid->KI); /* 更新 sum_error。 */
    }

    // PID计算：P + I + D + F
    float p_out = pid->KP * pid->error; /* 初始化 p_out。 */
    float i_out = pid->KI * pid->sum_error; /* 初始化 i_out。 */

#if PID_DERIVATIVE_ON_MEASUREMENT /* 按 PID_DERIVATIVE_ON_MEASUREMENT 选择编译分支。 */
    // 微分先行：对测量值变化进行微分（避免目标突变产生尖峰）
    // D = Kd × (last_measure - measure)，测量值增加时产生负阻尼
    float d_out = pid->KD * (pid->last_measure - measure); /* 初始化 d_out。 */
#else /* 切换到备用编译分支。 */
    // 普通微分：对误差变化进行微分
    float d_out = pid->KD * (pid->error - pid->last_error); /* 初始化 d_out。 */
#endif /* 结束条件编译。 */

    float f_out = pid->KF * pid->feedforward; /* 初始化 f_out。 */

    // 积分输出限幅
    i_out = float_constrain(i_out, -pid->max_iout, pid->max_iout); /* 更新 i_out。 */

    // 总输出
    pid->output = p_out + i_out + d_out + f_out; /* 更新 output。 */

    // 输出限幅（正负对称限幅）
    pid->output = float_constrain(pid->output, -pid->max_output, pid->max_output); /* 更新 output。 */

    // 最小输出保护：按误差方向生效，避免阻止反向纠偏
    pid->output = apply_min_output_by_error(pid->output, pid->min_output, pid->error); /* 更新 output。 */

    // 更新上次误差、测量值
    pid->last_error = pid->error; /* 更新 last_error。 */
    pid->last_measure = measure; /* 更新 last_measure。 */

    return pid->output; /* 返回当前计算结果。 */
}

/**
 * @brief 增量式PID计算
 * @param pid PID结构体指针
 * @param target 目标值
 * @param measure 测量值
 * @return PID增量输出值（需要累加到总输出上）
 * @note 第一次计算只使用P项，第二次及之后使用完整P、I、D、F
 */
float PID_Incremental_Calc(PID_t *pid, float target, float measure) /* 实现 PID_Incremental_Calc。 */
{
    if (pid == NULL || !pid->initialized) /* 检查当前执行条件。 */
        return 0.0f; /* 返回当前计算结果。 */

    // 更新目标值、测量值
    float last_target = pid->target; /* 初始化 last_target。 */

    pid->target = target; /* 更新 target。 */
    pid->measure = measure; /* 更新 measure。 */

    // 计算误差
    pid->error = target - measure; /* 更新 error。 */

    // 第一次计算：只使用P项
    if (pid->calc_count == 0) /* 检查当前执行条件。 */
    {
        float p_out = pid->KP * pid->error; /* 初始化 p_out。 */

        pid->output = p_out; /* 更新 output。 */

        // 输出限幅（正负对称限幅）
        pid->output = float_constrain(pid->output, -pid->max_output, pid->max_output); /* 更新 output。 */

        // 更新历史误差、测量值
        pid->prev_error = pid->last_error; /* 更新 prev_error。 */
        pid->last_error = pid->error; /* 更新 last_error。 */
        pid->prev_measure = pid->last_measure; /* 更新 prev_measure。 */
        pid->last_measure = measure; /* 更新 last_measure。 */

        // 计算前馈值（目标值变化量）
        pid->feedforward = 0.0f; // 第一次无前馈

        // 增加计算次数
        pid->calc_count = 1; /* 更新 calc_count。 */

        return pid->output; /* 返回当前计算结果。 */
    }

    // 计算前馈值（目标值变化量）
    pid->feedforward = target - last_target; /* 更新 feedforward。 */

    // 第二次及之后：使用完整增量式P、I、D、F
    // Δu(k) = Kp[e(k) - e(k-1)] + Ki*e(k) + Kd[e(k) - 2e(k-1) + e(k-2)]
    float p_out = pid->KP * (pid->error - pid->last_error); /* 初始化 p_out。 */
    float i_out = pid->KI * pid->error; /* 初始化 i_out。 */

#if PID_DERIVATIVE_ON_MEASUREMENT /* 按 PID_DERIVATIVE_ON_MEASUREMENT 选择编译分支。 */
    // 微分先行：对测量值变化进行微分（避免目标突变产生尖峰）
    // 增量式微分先行：Δd = Kd × (2×last_measure - measure - prev_measure)
    float d_out = pid->KD * (2.0f * pid->last_measure - measure - pid->prev_measure); /* 初始化 d_out。 */
#else /* 切换到备用编译分支。 */
    float d_out = pid->KD * (pid->error - 2.0f * pid->last_error + pid->prev_error); /* 初始化 d_out。 */
#endif /* 结束条件编译。 */

    float f_out = pid->KF * pid->feedforward; /* 初始化 f_out。 */

    // 积分输出限幅
    i_out = float_constrain(i_out, -pid->max_iout, pid->max_iout); /* 更新 i_out。 */

    // 增量输出
    pid->delta_out = p_out + i_out + d_out + f_out; /* 更新 delta_out。 */

    // 累加到总输出
    pid->output += pid->delta_out; /* 更新 output。 */

    // 总输出限幅（正负对称限幅）
    pid->output = float_constrain(pid->output, -pid->max_output, pid->max_output); /* 更新 output。 */

    // 最小输出保护：按误差方向生效，避免阻止反向纠偏
    pid->output = apply_min_output_by_error(pid->output, pid->min_output, pid->error); /* 更新 output。 */

    // 更新历史误差、测量值
    pid->prev_error = pid->last_error; /* 更新 prev_error。 */
    pid->last_error = pid->error; /* 更新 last_error。 */
    pid->prev_measure = pid->last_measure; /* 更新 prev_measure。 */
    pid->last_measure = measure; /* 更新 last_measure。 */

    return pid->output; /* 返回当前计算结果。 */
}

/**
 * @brief PID计算函数（统一接口）
 * @param pid PID结构体指针
 * @param target 目标值
 * @param measure 测量值
 * @return PID输出值
 */
float PID_Calculate(PID_t *pid, float target, float measure) /* 实现 PID_Calculate。 */
{
    if (pid == NULL || !pid->initialized) /* 检查当前执行条件。 */
        return 0.0f; /* 返回当前计算结果。 */

    if (pid->mode == PID_POSITION) /* 检查当前执行条件。 */
    {
        return PID_Position_Calc(pid, target, measure); /* 返回当前计算结果。 */
    }
    else if (pid->mode == PID_DELTA) /* 继续判断下一条件。 */
    {
        return PID_Incremental_Calc(pid, target, measure); /* 返回当前计算结果。 */
    }

    return 0.0f; /* 返回当前计算结果。 */
}

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
                            float inner_measure) /* 继续当前语句。 */
{
    if (cascade_pid == NULL) /* 检查当前执行条件。 */
        return 0.0f; /* 返回当前计算结果。 */

    // 外环计算，输出作为内环的目标值
    float inner_target = PID_Calculate(&cascade_pid->outer, outer_target, outer_measure); /* 初始化 inner_target。 */

    // 内环计算，输出作为最终控制量
    float output = PID_Calculate(&cascade_pid->inner, inner_target, inner_measure); /* 初始化 output。 */

    return output; /* 返回当前计算结果。 */
}

/**
 * @brief PID清零函数
 * @param pid PID结构体指针
 */
void PID_Clear(PID_t *pid) /* 实现 PID_Clear。 */
{
    if (pid == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    pid->calc_count = 0; // 重置计算次数，下次计算将重新从首次开始
    pid->target = 0.0f; /* 更新 target。 */
    pid->measure = 0.0f; /* 更新 measure。 */
    pid->last_measure = 0.0f; // 重置上次测量值
    pid->prev_measure = 0.0f; // 重置上上次测量值
    pid->error = 0.0f; /* 更新 error。 */
    pid->last_error = 0.0f; /* 更新 last_error。 */
    pid->prev_error = 0.0f; /* 更新 prev_error。 */
    pid->sum_error = 0.0f; /* 更新 sum_error。 */
    pid->feedforward = 0.0f; /* 更新 feedforward。 */
    pid->output = 0.0f; /* 更新 output。 */
    pid->last_output = 0.0f; /* 更新 last_output。 */
    pid->delta_out = 0.0f; /* 更新 delta_out。 */
}

/**
 * @brief PID积分项清零函数
 * @param pid PID结构体指针
 */
void PID_Clear_Integral(PID_t *pid) /* 实现 PID_Clear_Integral。 */
{
    if (pid == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    pid->sum_error = 0.0f; /* 更新 sum_error。 */
}

/**
 * @brief 串级PID清零
 * @param cascade_pid 串级PID结构体指针
 */
void CASCADE_PID_Clear(CASCADE_PID_t *cascade_pid) /* 实现 CASCADE_PID_Clear。 */
{
    if (cascade_pid == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    PID_Clear(&cascade_pid->outer); /* 调用 PID_Clear。 */
    PID_Clear(&cascade_pid->inner); /* 调用 PID_Clear。 */
}

/**
 * @brief 串级PID积分项清零
 * @param cascade_pid 串级PID结构体指针
 */
void CASCADE_PID_Clear_Integral(CASCADE_PID_t *cascade_pid) /* 实现 CASCADE_PID_Clear_Integral。 */
{
    if (cascade_pid == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    PID_Clear_Integral(&cascade_pid->outer); /* 调用 PID_Clear_Integral。 */
    PID_Clear_Integral(&cascade_pid->inner); /* 调用 PID_Clear_Integral。 */
}

/**
 * @brief PID参数设置函数
 * @param pid PID结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param kf 前馈系数
 */
void PID_Set_Coefficient(PID_t *pid, float kp, float ki, float kd, float kf) /* 实现 PID_Set_Coefficient。 */
{
    if (pid == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    pid->KP = kp; /* 更新 KP。 */
    pid->KI = ki; /* 更新 KI。 */
    pid->KD = kd; /* 更新 KD。 */
    pid->KF = kf; /* 更新 KF。 */
}

/**
 * @brief PID输出限幅设置函数
 * @param pid PID结构体指针
 * @param max_output 输出上限（实际输出范围为 [-max_output, +max_output]）
 * @param min_output 最小输出幅值（按误差方向生效）
 * @param max_iout 积分限幅
 */
void PID_Set_OutputLimit(PID_t *pid, float max_output, float min_output, float max_iout) /* 实现 PID_Set_OutputLimit。 */
{
    if (pid == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    pid->max_output = max_output; /* 更新 max_output。 */
    pid->min_output = min_output; /* 更新 min_output。 */
    pid->max_iout = max_iout; /* 更新 max_iout。 */
}

/**
 * @brief 检查PID是否已初始化
 * @param pid PID结构体指针
 * @return 1:已初始化, 0:未初始化
 */
uint8_t PID_Is_Initialized(PID_t *pid) /* 实现 PID_Is_Initialized。 */
{
    if (pid == NULL) /* 检查当前执行条件。 */
        return 0; /* 返回状态值 0。 */

    return pid->initialized; /* 返回当前计算结果。 */
}

/* ============================================================
 *  波形发生器
 * ============================================================ */

void SineWave_Init(SineWaveGen_t *wave, float amplitude, float period, float dt) /* 实现 SineWave_Init。 */
{
    if (wave == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    wave->amplitude = amplitude; /* 更新 amplitude。 */
    wave->period = period; /* 更新 period。 */
    wave->dt = dt; /* 更新 dt。 */
    wave->angle_deg = 0.0f; /* 更新 angle_deg。 */
    wave->output = 0.0f; /* 更新 output。 */
}

float SineWave_Calc(SineWaveGen_t *wave) /* 实现 SineWave_Calc。 */
{
    if (wave == NULL) /* 检查当前执行条件。 */
        return 0.0f; /* 返回当前计算结果。 */
    float radians = wave->angle_deg * (PI / 180.0f); /* 初始化 radians。 */
    wave->output = wave->amplitude * arm_sin_f32(radians); /* 更新 output。 */
    wave->angle_deg += (360.0f / wave->period) * wave->dt; /* 更新 angle_deg。 */
    if (wave->angle_deg >= 360.0f) /* 检查当前执行条件。 */
        wave->angle_deg -= 360.0f; /* 更新 angle_deg。 */
    return wave->output; /* 返回当前计算结果。 */
}

void SineWave_Set(SineWaveGen_t *wave, float amplitude, float period) /* 实现 SineWave_Set。 */
{
    if (wave == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    wave->amplitude = amplitude; /* 更新 amplitude。 */
    wave->period = period; /* 更新 period。 */
}

void TrapWave_Init(TrapWaveGen_t *trap, float amplitude, float gradient) /* 实现 TrapWave_Init。 */
{
    if (trap == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    trap->amplitude = amplitude; /* 更新 amplitude。 */
    trap->gradient = gradient; /* 更新 gradient。 */
    trap->direction = 1; /* 更新 direction。 */
    trap->output = -amplitude; /* 更新 output。 */
}

float TrapWave_Calc(TrapWaveGen_t *trap) /* 实现 TrapWave_Calc。 */
{
    if (trap == NULL) /* 检查当前执行条件。 */
        return 0.0f; /* 返回当前计算结果。 */

    trap->output += trap->direction * trap->gradient; /* 更新 output。 */

    if (trap->output >= trap->amplitude) /* 检查当前执行条件。 */
    {
        trap->output = trap->amplitude; /* 更新 output。 */
        trap->direction = -1; /* 更新 direction。 */
    }
    else if (trap->output <= -trap->amplitude) /* 继续判断下一条件。 */
    {
        trap->output = -trap->amplitude; /* 更新 output。 */
        trap->direction = 1; /* 更新 direction。 */
    }

    return trap->output; /* 返回当前计算结果。 */
}

void TrapWave_Set(TrapWaveGen_t *trap, float amplitude, float gradient) /* 实现 TrapWave_Set。 */
{
    if (trap == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    trap->amplitude = amplitude; /* 更新 amplitude。 */
    trap->gradient = gradient; /* 更新 gradient。 */
}

/* ============================================================
 *  波形发生器 FreeRTOS 任务
 * ============================================================ */

float sine_output = 0.0f; /* 初始化 sine_output。 */
float trap_output = 0.0f; /* 初始化 trap_output。 */

SineWaveGen_t g_SineWave; /* 保存 g_SineWave。 */
TrapWaveGen_t g_TrapWave; /* 保存 g_TrapWave。 */

void SineWaveTask(void *argument) /* 实现 SineWaveTask。 */
{
    SineWave_Init(&g_SineWave, 13824.0f, 5.5f, 0.001f); /* 调用 SineWave_Init。 */
    while (1) /* 持续执行当前任务。 */
    {
        sine_output = SineWave_Calc(&g_SineWave); /* 更新 sine_output。 */
        osDelay(1); /* 调用 osDelay。 */
    }
}

void TrapWaveTask(void *argument) /* 实现 TrapWaveTask。 */
{
    TrapWave_Init(&g_TrapWave, 8800.0f, 1.6f); /* 调用 TrapWave_Init。 */
    while (1) /* 持续执行当前任务。 */
    {
        trap_output = TrapWave_Calc(&g_TrapWave); /* 更新 trap_output。 */
        osDelay(1); /* 调用 osDelay。 */
    }
}
