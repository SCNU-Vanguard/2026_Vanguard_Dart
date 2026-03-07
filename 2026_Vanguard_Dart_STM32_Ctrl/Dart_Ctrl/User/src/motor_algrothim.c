#include "motor_algrothim.h"

volatile float g_MotorTrapPosLastOutput = 0.0f;

/* ------------------------------------------------------------------ */
/*  内部工具函数                                                        */
/* ------------------------------------------------------------------ */

static inline float _signf(float x) { return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f); }
static inline float _fabsf(float x) { return (x >= 0.0f) ? x : -x; }

static inline float _clampf(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

/* ------------------------------------------------------------------ */
/*  公开 API                                                            */
/* ------------------------------------------------------------------ */

void Motor_TrapPos_Init(MotorTrapPosProfile_t *profile,
                        float initial_pos, float vmax, float amax)
{
    if (!profile)
        return;

    profile->target_pos = initial_pos;
    profile->cmd_pos = initial_pos;
    profile->cmd_vel = 0.0f;
    profile->cmd_acc = 0.0f;
    profile->vmax = _fabsf(vmax);
    profile->amax = _fabsf(amax);
    profile->jmax = profile->amax * 10.0f; /* 默认宽松 jerk */
    profile->brake_gain = 1.0f;            /* 默认不提前制动 */
    profile->arrive_zone = 0.5f;           /* 默认0.5°死区锁定 */
    profile->decel_zone = 50.0f;           /* 默认50°线性减速区 */
    profile->initialized = 1U;
}

void Motor_TrapPos_SetJerk(MotorTrapPosProfile_t *profile, float jmax)
{
    if (!profile)
        return;
    profile->jmax = _fabsf(jmax);
}

void Motor_TrapPos_Reset(MotorTrapPosProfile_t *profile, float pos)
{
    if (!profile)
        return;
    profile->target_pos = pos;
    profile->cmd_pos = pos;
    profile->cmd_vel = 0.0f;
    profile->cmd_acc = 0.0f;
    profile->initialized = 1U;
}

void Motor_TrapPos_Resync(MotorTrapPosProfile_t *profile, float actual_pos)
{
    if (!profile)
        return;
    profile->cmd_pos = actual_pos;
    /* 保留 cmd_vel 和 cmd_acc，避免运动中切目标时速度阶跃 */
}

/**
 * @brief  梯形加减速位置规划（每控制周期调用一次）
 *
 * 算法核心：
 *   1. 计算剩余距离 error = target - cmd_pos
 *   2. 计算"恰好能停下"的最大速度 v_brake = sqrt(2 * amax * |error|)
 *   3. 期望速度 = sign(error) * min(vmax, v_brake)
 *   4. 用加速度限幅将 cmd_vel 平滑过渡到期望速度（梯形斜坡）
 *   5. 积分得到新的 cmd_pos
 *
 * @param  profile  规划器实例
 * @param  dt       控制周期 (s)，例如 0.001f
 * @return 本周期的期望位置 cmd_pos
 */
float Motor_TrapPos_Update(MotorTrapPosProfile_t *profile, float dt)
{
    if (!profile || !profile->initialized)
        return 0.0f;

    float error = profile->target_pos - profile->cmd_pos;
    float abs_error = _fabsf(error);

    /* --- 0. 到达死区：误差极小时强制锁零速，防止PID硬响应导致飞车 --- */
    if (abs_error <= profile->arrive_zone)
    {
        profile->cmd_pos = profile->target_pos;
        profile->cmd_vel = 0.0f;
        profile->cmd_acc = 0.0f;
        g_MotorTrapPosLastOutput = profile->cmd_pos;
        return profile->cmd_pos;
    }

    /* --- 1. 计算制动速度上限 --- */
    /* brake_gain < 1 时等效增大制动距离，让电机更早开始减速 */
    float v_brake = sqrtf(2.0f * profile->amax * abs_error * profile->brake_gain);

    /* --- 1.5 线性减速区：进入 decel_zone 后速度上限线性压低 --- */
    /* 在 decel_zone 边界速度 = vmax，在 arrive_zone 边界速度 ≈ 0 */
    float v_limit = profile->vmax;
    if (abs_error < profile->decel_zone)
    {
        float denom = profile->decel_zone - profile->arrive_zone;
        if (denom > 1e-6f)
        {
            float ratio = (abs_error - profile->arrive_zone) / denom;
            if (ratio < 0.0f) ratio = 0.0f;
            v_limit = profile->vmax * ratio;
        }
        else
        {
            v_limit = 0.0f; // decel_zone ≈ arrive_zone，直接零速
        }
    }

    /* 取 v_brake 和 v_limit 中较小者 */
    float v_cap = (v_brake < v_limit) ? v_brake : v_limit;

    /* --- 2. 期望速度（方向 × 速度上限） --- */
    float v_desired = _signf(error) * _clampf(v_cap, 0.0f, profile->vmax);

    /* --- 3. 加速度限幅：cmd_vel → v_desired --- */
    float dv_max = profile->amax * dt;
    float dv = v_desired - profile->cmd_vel;

    if (dv > dv_max)
        dv = dv_max;
    else if (dv < -dv_max)
        dv = -dv_max;

    profile->cmd_acc = (dt > 1e-6f) ? (dv / dt) : 0.0f; /* 记录当前加速度（供外部监控） */
    profile->cmd_vel += dv;

    /* --- 4. 积分位置 --- */
    profile->cmd_pos += profile->cmd_vel * dt;

    /* --- 5. 到达判断：防止过冲 --- */
    /* 若越过目标，直接钳位并清零速度 */
    if (_signf(profile->target_pos - profile->cmd_pos) != _signf(error) && abs_error > 1e-6f)
    {
        profile->cmd_pos = profile->target_pos;
        profile->cmd_vel = 0.0f;
        profile->cmd_acc = 0.0f;
    }

    g_MotorTrapPosLastOutput = profile->cmd_pos;
    return profile->cmd_pos;
}
