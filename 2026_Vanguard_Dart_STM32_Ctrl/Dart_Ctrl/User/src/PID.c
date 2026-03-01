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
void PID_Init(PID_t *pid, PID_MODE_e mode, float kp, float ki, float kd, float kf, float max_out, float min_out, float max_iout)
{
    if (pid == NULL)
        return;

    // 清零结构体
    memset(pid, 0, sizeof(PID_t));

    // 设置参数
    pid->mode = mode;
    pid->KP = kp;
    pid->KI = ki;
    pid->KD = kd;
    pid->KF = kf;
    pid->max_output = max_out;
    pid->min_output = min_out;
    pid->max_iout = max_iout;

    // 设置初始化标志位
    pid->initialized = 1;
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
void CASCADE_PID_Init(CASCADE_PID_t *cascade_pid,
                      float outer_kp, float outer_ki, float outer_kd, float outer_kf,
                      float inner_kp, float inner_ki, float inner_kd, float inner_kf,
                      float outer_max_out, float outer_min_out, float outer_max_iout,
                      float inner_max_out, float inner_min_out, float inner_max_iout)
{
    if (cascade_pid == NULL)
        return;

    // 初始化外环（位置环，使用位置式PID，D项更稳定）
    PID_Init(&cascade_pid->outer, PID_POSITION, outer_kp, outer_ki, outer_kd, outer_kf,
             outer_max_out, outer_min_out, outer_max_iout);

    // 初始化内环（速度环，使用位置式PID，避免增量式P项对误差突变的过激响应）
    PID_Init(&cascade_pid->inner, PID_POSITION, inner_kp, inner_ki, inner_kd, inner_kf,
             inner_max_out, inner_min_out, inner_max_iout);
}

/**
 * @brief 限幅函数
 * @param value 待限幅的值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的值
 */
static inline float float_constrain(float value, float min, float max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}

/**
 * @brief 最小输出保护（按误差方向）
 * @param output 当前输出
 * @param min_output 最小输出幅值
 * @param error 当前误差
 * @return 处理后的输出
 * @note 仅当输出方向与纠偏方向一致时，才抬升到最小输出；不会强行翻转输出方向
 */
static inline float apply_min_output_by_error(float output, float min_output, float error)
{
    if (min_output <= 0.0f)
        return output;

    if (error > 0.0f)
    {
        if (output > 0.0f && output < min_output)
            return min_output;
    }
    else if (error < 0.0f)
    {
        if (output < 0.0f && output > -min_output)
            return -min_output;
    }

    return output;
}

/**
 * @brief 位置式PID计算
 * @param pid PID结构体指针
 * @param target 目标值
 * @param measure 测量值
 * @return PID输出值
 * @note 第一次计算只使用P项，第二次及之后使用完整P、I、D、F
 */
float PID_Position_Calc(PID_t *pid, float target, float measure)
{
    if (pid == NULL || !pid->initialized)
        return 0.0f;

    // 更新目标值、测量值
    float last_target = pid->target;

    pid->target = target;
    pid->measure = measure;

    // 计算误差
    pid->error = target - measure;

    // 第一次计算：只使用P项
    if (pid->calc_count == 0)
    {
        float p_out = pid->KP * pid->error;

        pid->output = p_out;

        // 输出限幅（正负对称限幅）
        pid->output = float_constrain(pid->output, -pid->max_output, pid->max_output);

        // 更新上次误差、测量值
        pid->last_error = pid->error;
        pid->last_measure = measure;

        // 计算前馈值（目标值变化量）
        pid->feedforward = 0.0f; // 第一次无前馈

        // 增加计算次数
        pid->calc_count = 1;

        return pid->output;
    }

    // 计算前馈值（目标值变化量）
    pid->feedforward = target - last_target;

    // 第二次及之后：使用完整P、I、D、F
    // 积分项累加（带限幅）
    pid->sum_error += pid->error;
    if (pid->KI != 0)
    {
        pid->sum_error = float_constrain(pid->sum_error, -pid->max_iout / pid->KI, pid->max_iout / pid->KI);
    }

    // PID计算：P + I + D + F
    float p_out = pid->KP * pid->error;
    float i_out = pid->KI * pid->sum_error;

#if PID_DERIVATIVE_ON_MEASUREMENT
    // 微分先行：对测量值变化进行微分（避免目标突变产生尖峰）
    // D = Kd × (last_measure - measure)，测量值增加时产生负阻尼
    float d_out = pid->KD * (pid->last_measure - measure);
#else
    // 普通微分：对误差变化进行微分
    float d_out = pid->KD * (pid->error - pid->last_error);
#endif

    float f_out = pid->KF * pid->feedforward;

    // 积分输出限幅
    i_out = float_constrain(i_out, -pid->max_iout, pid->max_iout);

    // 总输出
    pid->output = p_out + i_out + d_out + f_out;

    // 输出限幅（正负对称限幅）
    pid->output = float_constrain(pid->output, -pid->max_output, pid->max_output);

    // 最小输出保护：按误差方向生效，避免阻止反向纠偏
    pid->output = apply_min_output_by_error(pid->output, pid->min_output, pid->error);

    // 更新上次误差、测量值
    pid->last_error = pid->error;
    pid->last_measure = measure;

    return pid->output;
}

/**
 * @brief 增量式PID计算
 * @param pid PID结构体指针
 * @param target 目标值
 * @param measure 测量值
 * @return PID增量输出值（需要累加到总输出上）
 * @note 第一次计算只使用P项，第二次及之后使用完整P、I、D、F
 */
float PID_Incremental_Calc(PID_t *pid, float target, float measure)
{
    if (pid == NULL || !pid->initialized)
        return 0.0f;

    // 更新目标值、测量值
    float last_target = pid->target;

    pid->target = target;
    pid->measure = measure;

    // 计算误差
    pid->error = target - measure;

    // 第一次计算：只使用P项
    if (pid->calc_count == 0)
    {
        float p_out = pid->KP * pid->error;

        pid->output = p_out;

        // 输出限幅（正负对称限幅）
        pid->output = float_constrain(pid->output, -pid->max_output, pid->max_output);

        // 更新历史误差、测量值
        pid->prev_error = pid->last_error;
        pid->last_error = pid->error;
        pid->prev_measure = pid->last_measure;
        pid->last_measure = measure;

        // 计算前馈值（目标值变化量）
        pid->feedforward = 0.0f; // 第一次无前馈

        // 增加计算次数
        pid->calc_count = 1;

        return pid->output;
    }

    // 计算前馈值（目标值变化量）
    pid->feedforward = target - last_target;

    // 第二次及之后：使用完整增量式P、I、D、F
    // Δu(k) = Kp[e(k) - e(k-1)] + Ki*e(k) + Kd[e(k) - 2e(k-1) + e(k-2)]
    float p_out = pid->KP * (pid->error - pid->last_error);
    float i_out = pid->KI * pid->error;

#if PID_DERIVATIVE_ON_MEASUREMENT
    // 微分先行：对测量值变化进行微分（避免目标突变产生尖峰）
    // 增量式微分先行：Δd = Kd × (2×last_measure - measure - prev_measure)
    float d_out = pid->KD * (2.0f * pid->last_measure - measure - pid->prev_measure);
#else
    float d_out = pid->KD * (pid->error - 2.0f * pid->last_error + pid->prev_error);
#endif

    float f_out = pid->KF * pid->feedforward;

    // 积分输出限幅
    i_out = float_constrain(i_out, -pid->max_iout, pid->max_iout);

    // 增量输出
    pid->delta_out = p_out + i_out + d_out + f_out;

    // 累加到总输出
    pid->output += pid->delta_out;

    // 总输出限幅（正负对称限幅）
    pid->output = float_constrain(pid->output, -pid->max_output, pid->max_output);

    // 最小输出保护：按误差方向生效，避免阻止反向纠偏
    pid->output = apply_min_output_by_error(pid->output, pid->min_output, pid->error);

    // 更新历史误差、测量值
    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;
    pid->prev_measure = pid->last_measure;
    pid->last_measure = measure;

    return pid->output;
}

/**
 * @brief PID计算函数（统一接口）
 * @param pid PID结构体指针
 * @param target 目标值
 * @param measure 测量值
 * @return PID输出值
 */
float PID_Calculate(PID_t *pid, float target, float measure)
{
    if (pid == NULL || !pid->initialized)
        return 0.0f;

    if (pid->mode == PID_POSITION)
    {
        return PID_Position_Calc(pid, target, measure);
    }
    else if (pid->mode == PID_DELTA)
    {
        return PID_Incremental_Calc(pid, target, measure);
    }

    return 0.0f;
}

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
                            float inner_measure)
{
    if (cascade_pid == NULL)
        return 0.0f;

    // 外环计算，输出作为内环的目标值
    float inner_target = PID_Calculate(&cascade_pid->outer, outer_target, outer_measure);

    // 内环计算，输出作为最终控制量
    float output = PID_Calculate(&cascade_pid->inner, inner_target, inner_measure);

    return output;
}

/**
 * @brief PID清零函数
 * @param pid PID结构体指针
 */
void PID_Clear(PID_t *pid)
{
    if (pid == NULL)
        return;

    pid->calc_count = 0; // 重置计算次数，下次计算将重新从首次开始
    pid->target = 0.0f;
    pid->measure = 0.0f;
    pid->last_measure = 0.0f; // 重置上次测量值
    pid->prev_measure = 0.0f; // 重置上上次测量值
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->sum_error = 0.0f;
    pid->feedforward = 0.0f;
    pid->output = 0.0f;
    pid->last_output = 0.0f;
    pid->delta_out = 0.0f;
}

/**
 * @brief PID积分项清零函数
 * @param pid PID结构体指针
 */
void PID_Clear_Integral(PID_t *pid)
{
    if (pid == NULL)
        return;

    pid->sum_error = 0.0f;
}

/**
 * @brief 串级PID清零
 * @param cascade_pid 串级PID结构体指针
 */
void CASCADE_PID_Clear(CASCADE_PID_t *cascade_pid)
{
    if (cascade_pid == NULL)
        return;

    PID_Clear(&cascade_pid->outer);
    PID_Clear(&cascade_pid->inner);
}

/**
 * @brief 串级PID积分项清零
 * @param cascade_pid 串级PID结构体指针
 */
void CASCADE_PID_Clear_Integral(CASCADE_PID_t *cascade_pid)
{
    if (cascade_pid == NULL)
        return;

    PID_Clear_Integral(&cascade_pid->outer);
    PID_Clear_Integral(&cascade_pid->inner);
}

/**
 * @brief PID参数设置函数
 * @param pid PID结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param kf 前馈系数
 */
void PID_Set_Coefficient(PID_t *pid, float kp, float ki, float kd, float kf)
{
    if (pid == NULL)
        return;

    pid->KP = kp;
    pid->KI = ki;
    pid->KD = kd;
    pid->KF = kf;
}

/**
 * @brief PID输出限幅设置函数
 * @param pid PID结构体指针
 * @param max_output 输出上限（实际输出范围为 [-max_output, +max_output]）
 * @param min_output 最小输出幅值（按误差方向生效）
 * @param max_iout 积分限幅
 */
void PID_Set_OutputLimit(PID_t *pid, float max_output, float min_output, float max_iout)
{
    if (pid == NULL)
        return;

    pid->max_output = max_output;
    pid->min_output = min_output;
    pid->max_iout = max_iout;
}

/**
 * @brief 检查PID是否已初始化
 * @param pid PID结构体指针
 * @return 1:已初始化, 0:未初始化
 */
uint8_t PID_Is_Initialized(PID_t *pid)
{
    if (pid == NULL)
        return 0;

    return pid->initialized;
}

/* ============================================================
 *  波形发生器
 * ============================================================ */

void SineWave_Init(SineWaveGen_t *wave, float amplitude, float period, float dt)
{
    if (wave == NULL)
        return;
    wave->amplitude = amplitude;
    wave->period = period;
    wave->dt = dt;
    wave->angle_deg = 0.0f;
    wave->output = 0.0f;
}

float SineWave_Calc(SineWaveGen_t *wave)
{
    if (wave == NULL)
        return 0.0f;
    float radians = wave->angle_deg * (PI / 180.0f);
    wave->output = wave->amplitude * arm_sin_f32(radians);
    wave->angle_deg += (360.0f / wave->period) * wave->dt;
    if (wave->angle_deg >= 360.0f)
        wave->angle_deg -= 360.0f;
    return wave->output;
}

void SineWave_Set(SineWaveGen_t *wave, float amplitude, float period)
{
    if (wave == NULL)
        return;
    wave->amplitude = amplitude;
    wave->period = period;
}

void TrapWave_Init(TrapWaveGen_t *trap, float amplitude, float gradient)
{
    if (trap == NULL)
        return;
    trap->amplitude = amplitude;
    trap->gradient = gradient;
    trap->direction = 1;
    trap->output = -amplitude;
}

float TrapWave_Calc(TrapWaveGen_t *trap)
{
    if (trap == NULL)
        return 0.0f;

    trap->output += trap->direction * trap->gradient;

    if (trap->output >= trap->amplitude)
    {
        trap->output = trap->amplitude;
        trap->direction = -1;
    }
    else if (trap->output <= -trap->amplitude)
    {
        trap->output = -trap->amplitude;
        trap->direction = 1;
    }

    return trap->output;
}

void TrapWave_Set(TrapWaveGen_t *trap, float amplitude, float gradient)
{
    if (trap == NULL)
        return;
    trap->amplitude = amplitude;
    trap->gradient = gradient;
}

/* ============================================================
 *  波形发生器 FreeRTOS 任务
 * ============================================================ */

float sine_output = 0.0f;
float trap_output = 0.0f;

SineWaveGen_t g_SineWave;
TrapWaveGen_t g_TrapWave;

void SineWaveTask(void *argument)
{
    SineWave_Init(&g_SineWave, 13824.0f, 5.5f, 0.001f);
    while (1)
    {
        sine_output = SineWave_Calc(&g_SineWave);
        osDelay(1);
    }
}

void TrapWaveTask(void *argument)
{
    TrapWave_Init(&g_TrapWave, 8800.0f, 1.6f);
    while (1)
    {
        trap_output = TrapWave_Calc(&g_TrapWave);
        osDelay(1);
    }
}
