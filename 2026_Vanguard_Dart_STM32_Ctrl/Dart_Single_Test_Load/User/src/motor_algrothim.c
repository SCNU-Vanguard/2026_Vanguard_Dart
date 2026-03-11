#include "motor_algrothim.h"
#include <math.h>

static inline float Motor_Fabsf(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static inline float Motor_Maxf(float a, float b)
{
    return (a >= b) ? a : b;
}

static inline float Motor_Clampf(float x, float min_v, float max_v)
{
    if (x < min_v)
    {
        return min_v;
    }
    if (x > max_v)
    {
        return max_v;
    }
    return x;
}

void Motor_TrapPos_Init(MotorTrapPosProfile_t *profile, float initial_pos, float vmax, float amax)
{
    if (profile == 0)
    {
        return;
    }

    profile->target_pos = initial_pos;
    profile->cmd_pos = initial_pos;
    profile->cmd_vel = 0.0f;
    profile->cmd_acc = 0.0f;
    profile->vmax = Motor_Fabsf(vmax);
    profile->amax = Motor_Fabsf(amax);
    profile->jmax = profile->amax * 10.0f;
    profile->initialized = 1U;
}

void Motor_TrapPos_SetJerk(MotorTrapPosProfile_t *profile, float jmax)
{
    if (profile == 0)
    {
        return;
    }

    profile->jmax = Motor_Fabsf(jmax);
}

void Motor_TrapPos_Reset(MotorTrapPosProfile_t *profile, float pos)
{
    if (profile == 0)
    {
        return;
    }

    profile->target_pos = pos;
    profile->cmd_pos = pos;
    profile->cmd_vel = 0.0f;
    profile->cmd_acc = 0.0f;
    profile->initialized = 1U;
}

float Motor_TrapPos_Update(MotorTrapPosProfile_t *profile, float target_pos, float dt_s)
{
    if (profile == 0)
    {
        return target_pos;
    }

    if (!isfinite(target_pos) || !isfinite(dt_s) || dt_s <= 0.0f)
    {
        return profile->cmd_pos;
    }

    if (!profile->initialized)
    {
        Motor_TrapPos_Init(profile, target_pos, profile->vmax, profile->amax);
        return profile->cmd_pos;
    }

    if (profile->vmax <= 0.0f || profile->amax <= 0.0f)
    {
        profile->target_pos = target_pos;
        profile->cmd_pos = target_pos;
        profile->cmd_vel = 0.0f;
        profile->cmd_acc = 0.0f;
        return profile->cmd_pos;
    }

    profile->target_pos = target_pos;

    {
        float error = profile->target_pos - profile->cmd_pos;
        float abs_error = Motor_Fabsf(error);
        float abs_vel = Motor_Fabsf(profile->cmd_vel);
        float brake_dist = (abs_vel * abs_vel) / (2.0f * profile->amax);
        const float jerk_soft_zone = Motor_Maxf(1.5f * brake_dist, 20.0f);
        float desired_acc = 0.0f;

        if (abs_error <= 1e-4f && abs_vel <= 1e-3f)
        {
            profile->cmd_pos = profile->target_pos;
            profile->cmd_vel = 0.0f;
            profile->cmd_acc = 0.0f;
            return profile->cmd_pos;
        }

        if (abs_error <= brake_dist)
        {
            if (profile->cmd_vel > 0.0f)
            {
                desired_acc = -profile->amax;
            }
            else if (profile->cmd_vel < 0.0f)
            {
                desired_acc = profile->amax;
            }
        }
        else
        {
            desired_acc = (error >= 0.0f) ? profile->amax : -profile->amax;
        }

        if (profile->jmax > 0.0f && abs_error <= jerk_soft_zone)
        {
            float max_acc_delta = profile->jmax * dt_s;
            float acc_delta = desired_acc - profile->cmd_acc;
            acc_delta = Motor_Clampf(acc_delta, -max_acc_delta, max_acc_delta);
            profile->cmd_acc += acc_delta;
        }
        else
        {
            profile->cmd_acc = desired_acc;
        }
        profile->cmd_acc = Motor_Clampf(profile->cmd_acc, -profile->amax, profile->amax);

        profile->cmd_vel += profile->cmd_acc * dt_s;
        if (profile->cmd_vel > profile->vmax)
        {
            profile->cmd_vel = profile->vmax;
            if (profile->cmd_acc > 0.0f)
            {
                profile->cmd_acc = 0.0f;
            }
        }
        else if (profile->cmd_vel < -profile->vmax)
        {
            profile->cmd_vel = -profile->vmax;
            if (profile->cmd_acc < 0.0f)
            {
                profile->cmd_acc = 0.0f;
            }
        }

        {
            float prev_error = error;
            profile->cmd_pos += profile->cmd_vel * dt_s;
            error = profile->target_pos - profile->cmd_pos;

            if ((prev_error > 0.0f && error < 0.0f) || (prev_error < 0.0f && error > 0.0f))
            {
                profile->cmd_pos = profile->target_pos;
                profile->cmd_vel = 0.0f;
                profile->cmd_acc = 0.0f;
            }
        }
    }

    return profile->cmd_pos;
}
