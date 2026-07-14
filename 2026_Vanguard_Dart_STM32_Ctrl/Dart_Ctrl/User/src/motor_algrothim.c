#include "motor_algrothim.h"
#include <math.h>

volatile float g_MotorTrapPosLastOutput = 0.0f; /* 初始化 g_MotorTrapPosLastOutput。 */

/* ------------------------------------------------------------------ */
/*  内部工具函数                                                       */
/* ------------------------------------------------------------------ */

static inline float motor_signf(float x) { return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f); } /* 继续当前语句。 */

/* ------------------------------------------------------------------ */
/*  公开 API                                                            */
/* ------------------------------------------------------------------ */

uint32_t CalcTrapMoveTimeoutMs(float start_pos, float target_pos, /* 传入下一项参数或数据。 */
                               float vmax, float amax, /* 传入下一项参数或数据。 */
                               uint32_t min_ms, uint32_t max_ms) /* 继续当前语句。 */
{
    float dist = fabsf(target_pos - start_pos); /* 初始化 dist。 */
    float abs_vmax = fabsf(vmax); /* 初始化 abs_vmax。 */
    float abs_amax = fabsf(amax); /* 初始化 abs_amax。 */
    float t_s = 0.0f; /* 初始化 t_s。 */
    uint32_t est_ms; /* 保存 est_ms。 */

    if (abs_vmax <= 1e-3f || abs_amax <= 1e-3f) /* 检查当前执行条件。 */
    {
        return min_ms; /* 返回当前计算结果。 */
    }

    {
        float t_acc = abs_vmax / abs_amax; /* 初始化 t_acc。 */
        float d_acc = 0.5f * abs_amax * t_acc * t_acc; /* 初始化 d_acc。 */

        if (dist <= 2.0f * d_acc) /* 检查当前执行条件。 */
        {
            /* 三角速度曲线 */
            t_s = 2.0f * sqrtf(dist / abs_amax); /* 更新 t_s。 */
        }
        else /* 处理其余情况。 */
        {
            /* 梯形速度曲线 */
            t_s = 2.0f * t_acc + (dist - 2.0f * d_acc) / abs_vmax; /* 更新 t_s。 */
        }
    }

    /* 估算时间增加安全裕量，避免长行程按理论值过早超时 */
    est_ms = (uint32_t)(t_s * 1000.0f * 1.6f + 200.0f); /* 更新 est_ms。 */
    if (est_ms < min_ms) /* 检查当前执行条件。 */
    {
        est_ms = min_ms; /* 更新 est_ms。 */
    }
    if (est_ms > max_ms) /* 检查当前执行条件。 */
    {
        est_ms = max_ms; /* 更新 est_ms。 */
    }
    return est_ms; /* 返回当前计算结果。 */
}

void Motor_TrapPos_Init(MotorTrapPosProfile_t *profile, /* 传入下一项参数或数据。 */
                        float initial_pos, float vmax, float amax) /* 继续当前语句。 */
{
    if (!profile) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    profile->target_pos = initial_pos; /* 更新 target_pos。 */
    profile->cmd_pos = initial_pos; /* 更新 cmd_pos。 */
    profile->cmd_vel = 0.0f; /* 更新 cmd_vel。 */
    profile->cmd_acc = 0.0f; /* 更新 cmd_acc。 */
    profile->vmax = fabsf(vmax); /* 更新 vmax。 */
    profile->amax = fabsf(amax); /* 更新 amax。 */
    profile->jmax = profile->amax * 10.0f; /* 默认宽松 jerk（>0 即启用 S 型） */
    profile->brake_gain = 1.0f;            /* 默认不提前制动 */
    profile->arrive_zone = 0.5f;           /* 默认0.5°死区锁定 */
    profile->decel_zone = 50.0f;           /* 默认50°线性减速区 */
    profile->initialized = 1U; /* 更新 initialized。 */
}

void Motor_TrapPos_SetJerk(MotorTrapPosProfile_t *profile, float jmax) /* 实现 Motor_TrapPos_SetJerk。 */
{
    if (!profile) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    profile->jmax = fabsf(jmax); /* 更新 jmax。 */
}

void Motor_TrapPos_Reset(MotorTrapPosProfile_t *profile, float pos) /* 实现 Motor_TrapPos_Reset。 */
{
    if (!profile) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    profile->target_pos = pos; /* 更新 target_pos。 */
    profile->cmd_pos = pos; /* 更新 cmd_pos。 */
    profile->cmd_vel = 0.0f; /* 更新 cmd_vel。 */
    profile->cmd_acc = 0.0f; /* 更新 cmd_acc。 */
    profile->initialized = 1U; /* 更新 initialized。 */
}

void Motor_TrapPos_Resync(MotorTrapPosProfile_t *profile, float actual_pos) /* 实现 Motor_TrapPos_Resync。 */
{
    if (!profile) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    profile->cmd_pos = actual_pos; /* 更新 cmd_pos。 */
    /* 保留 cmd_vel 和 cmd_acc，避免运动中切目标时速度阶跃 */
}

/**
 * @brief  S 型（jerk 受限）位置规划（每控制周期调用一次）
 *
 * 三层级联结构：
 *   外环（速度上限）：根据剩余距离反解“恰好能停下”的速度上限。
 *                     jmax>0 时使用三阶停止距离公式（含 jerk 圆角补偿），
 *                     保证提前足够距离减速、加速度能平滑归零而不过冲。
 *   中环（期望速度）：v_desired = sign(error) * min(v_brake, v_limit, vmax)
 *   内环（加速度限幅）：把 cmd_acc 用 jerk 限幅平滑拉向“达到 v_desired 所需的加速度”，
 *                       cmd_acc 本身被 amax 限幅 → 加速度梯形过渡、jerk 有界。
 *
 * jmax<=0 时内环退化为加速度直接限幅（等价原梯形规划），保证旧配置兼容。
 *
 * @param  profile  规划器实例
 * @param  dt       控制周期 (s)，例如 0.001f
 * @return 本周期的期望位置 cmd_pos
 */
float Motor_TrapPos_Update(MotorTrapPosProfile_t *profile, float dt) /* 实现 Motor_TrapPos_Update。 */
{
    if (!profile || !profile->initialized) /* 检查当前执行条件。 */
        return 0.0f; /* 返回当前计算结果。 */

    if (!(dt > 1e-6f)) /* 防御非法 dt（NaN 也会落入此分支）*/
    {
        g_MotorTrapPosLastOutput = profile->cmd_pos; /* 更新 g_MotorTrapPosLastOutput。 */
        return profile->cmd_pos; /* 返回当前计算结果。 */
    }

    float error = profile->target_pos - profile->cmd_pos; /* 初始化 error。 */
    float abs_error = fabsf(error); /* 初始化 abs_error。 */

    /* --- 0. 到达死区：误差与速度都极小时强制锁定，防止末端抖动/飞车 --- */
    /* 注意：必须等速度也基本归零才锁，否则高速冲入死区会被硬截断产生冲击 */
    if (abs_error <= profile->arrive_zone && fabsf(profile->cmd_vel) <= profile->arrive_zone) /* 检查当前执行条件。 */
    {
        profile->cmd_pos = profile->target_pos; /* 更新 cmd_pos。 */
        profile->cmd_vel = 0.0f; /* 更新 cmd_vel。 */
        profile->cmd_acc = 0.0f; /* 更新 cmd_acc。 */
        g_MotorTrapPosLastOutput = profile->cmd_pos; /* 更新 g_MotorTrapPosLastOutput。 */
        return profile->cmd_pos; /* 返回当前计算结果。 */
    }

    /* --- 1. 制动速度上限 --- */
    float v_brake; /* 保存 v_brake。 */
    if (profile->jmax > 1e-6f) /* 检查当前执行条件。 */
    {
        /* 三阶停止距离：d_stop(v) = v^2/(2a) + v*a/(2j)
         * 令 d_stop = abs_error*brake_gain，解二次方程取正根：
         *   A v^2 + B v - C = 0,  A=1/(2a), B=a/(2j), C=abs_error*brake_gain
         *   v = (-B + sqrt(B^2 + 4AC)) / (2A)
         * 含 jerk 圆角项，保证加速度能在到达前平滑降到 0。*/
        float A = 1.0f / (2.0f * profile->amax); /* 初始化 A。 */
        float B = profile->amax / (2.0f * profile->jmax); /* 初始化 B。 */
        float C = abs_error * profile->brake_gain; /* 初始化 C。 */
        v_brake = (-B + sqrtf(B * B + 4.0f * A * C)) / (2.0f * A); /* 更新 v_brake。 */
    }
    else /* 处理其余情况。 */
    {
        /* jmax<=0：退化为二阶刹车公式（原梯形行为）*/
        v_brake = sqrtf(2.0f * profile->amax * abs_error * profile->brake_gain); /* 更新 v_brake。 */
    }

    /* --- 1.5 线性减速区：进入 decel_zone 后速度上限线性压低，末端 snap 更平滑 --- */
    float v_limit = profile->vmax; /* 初始化 v_limit。 */
    if (abs_error < profile->decel_zone) /* 检查当前执行条件。 */
    {
        float denom = profile->decel_zone - profile->arrive_zone; /* 初始化 denom。 */
        if (denom > 1e-6f) /* 检查当前执行条件。 */
        {
            float ratio = (abs_error - profile->arrive_zone) / denom; /* 初始化 ratio。 */
            if (ratio < 0.0f) /* 检查当前执行条件。 */
                ratio = 0.0f; /* 更新 ratio。 */
            v_limit = profile->vmax * ratio; /* 更新 v_limit。 */
        }
        else /* 处理其余情况。 */
        {
            v_limit = 0.0f; /* 更新 v_limit。 */
        }
    }

    float v_cap = (v_brake < v_limit) ? v_brake : v_limit; /* 初始化 v_cap。 */

    /* --- 2. 期望速度（方向 × 速度上限） --- */
    float v_desired = motor_signf(error) * v_cap; /* 初始化 v_desired。 */

    /* --- 3. 加速度规划 --- */
    /* 期望加速度：本周期把 cmd_vel 拉到 v_desired 所需的加速度，先按 amax 限幅 */
    float a_des = (v_desired - profile->cmd_vel) / dt; /* 初始化 a_des。 */
    if (a_des > profile->amax) /* 检查当前执行条件。 */
        a_des = profile->amax; /* 更新 a_des。 */
    else if (a_des < -profile->amax) /* 继续判断下一条件。 */
        a_des = -profile->amax; /* 更新 a_des。 */

    if (profile->jmax > 1e-6f) /* 检查当前执行条件。 */
    {
        /* jerk 限幅：cmd_acc 平滑过渡到 a_des（加速度梯形 → S 型速度曲线）*/
        float da_max = profile->jmax * dt; /* 初始化 da_max。 */
        float da = a_des - profile->cmd_acc; /* 初始化 da。 */
        if (da > da_max) /* 检查当前执行条件。 */
            da = da_max; /* 更新 da。 */
        else if (da < -da_max) /* 继续判断下一条件。 */
            da = -da_max; /* 更新 da。 */
        profile->cmd_acc += da; /* 更新 cmd_acc。 */
    }
    else /* 处理其余情况。 */
    {
        /* 无 jerk 限制：加速度直接取期望值（等价原梯形）*/
        profile->cmd_acc = a_des; /* 更新 cmd_acc。 */
    }

    /* --- 4. 速度积分（再次按 vmax 限幅，防止 jerk 平滑期间累积超调）--- */
    profile->cmd_vel += profile->cmd_acc * dt; /* 更新 cmd_vel。 */
    if (profile->cmd_vel > profile->vmax) /* 检查当前执行条件。 */
        profile->cmd_vel = profile->vmax; /* 更新 cmd_vel。 */
    else if (profile->cmd_vel < -profile->vmax) /* 继续判断下一条件。 */
        profile->cmd_vel = -profile->vmax; /* 更新 cmd_vel。 */

    /* --- 5. 位置积分（前向欧拉）--- */
    profile->cmd_pos += profile->cmd_vel * dt; /* 更新 cmd_pos。 */

    /* --- 6. 过冲钳位：若越过目标，直接钳位并清零速度/加速度 --- */
    if (motor_signf(profile->target_pos - profile->cmd_pos) != motor_signf(error) && abs_error > 1e-6f) /* 检查当前执行条件。 */
    {
        profile->cmd_pos = profile->target_pos; /* 更新 cmd_pos。 */
        profile->cmd_vel = 0.0f; /* 更新 cmd_vel。 */
        profile->cmd_acc = 0.0f; /* 更新 cmd_acc。 */
    }

    g_MotorTrapPosLastOutput = profile->cmd_pos; /* 更新 g_MotorTrapPosLastOutput。 */
    return profile->cmd_pos; /* 返回当前计算结果。 */
}
