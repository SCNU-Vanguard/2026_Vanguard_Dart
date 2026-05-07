#include "UserLib.h"
#include "config.h"
#include "main.h"
#include <math.h>

/***********************************
 * 函数名: IsInDeadzoneTimedF
 * 作用:   带调用方自定义超时的浮点死区判定
 *         常用于“允许超时按到位处理”的流程控制
 **********************************/
bool IsInDeadzoneTimedF(float value, float target, float zone, uint32_t timeout_ms, DeadzoneTimer_t *timer)
{
    float abs_zone;
    uint32_t now_tick;
    bool target_changed;

    if (timer == NULL)
    {
        return false;
    }

    if (!isfinite(value) || !isfinite(target) || !isfinite(zone))
    {
        timer->active = false;
        return false;
    }

    abs_zone = fabsf(zone);
    if (fabsf(value - target) <= abs_zone)
    {
        // 已真实进入死区，清掉本轮计时状态
        timer->active = false;
        timer->last_target = target;
        timer->last_zone = abs_zone;
        return true;
    }

    now_tick = HAL_GetTick();
    target_changed = (!timer->active) ||
                     (fabsf(target - timer->last_target) > (abs_zone + 1e-3f)) ||
                     (fabsf(abs_zone - timer->last_zone) > 1e-3f);
    if (target_changed)
    {
        // 新目标或新死区到来时，重新开始计时
        timer->start_tick = now_tick;
        timer->last_target = target;
        timer->last_zone = abs_zone;
        timer->active = true;
        return false;
    }

    if ((uint32_t)(now_tick - timer->start_tick) >= timeout_ms)
    {
        // 超时后按到位处理，避免上层流程永久阻塞
        timer->active = false;
        timer->last_target = target;
        timer->last_zone = abs_zone;
        return true;
    }

    return false;
}

/***********************************
 * 函数名: IsInDeadzoneF
 * 作用:   通用浮点死区判定
 *         是否允许超时放行由 enable_timeout_pass 决定
 **********************************/
bool IsInDeadzoneF(float value, float target, float zone, DeadzoneState_t *state, bool enable_timeout_pass)
{
    float abs_zone;
    bool in_deadzone;
    uint32_t now_tick;
    bool target_or_zone_changed;

    if (state == NULL)
    {
        return false;
    }

    if (!isfinite(value) || !isfinite(target) || !isfinite(zone))
    {
        state->active = false;
        return false;
    }

    abs_zone = fabsf(zone);
    in_deadzone = (fabsf(value - target) <= abs_zone);
    if (in_deadzone)
    {
        // 已真实进入死区，直接返回到位
        state->active = false;
        state->last_target = target;
        state->last_zone = abs_zone;
        return true;
    }

    now_tick = HAL_GetTick();
    target_or_zone_changed = (!state->active) ||
                             (fabsf(target - state->last_target) > (abs_zone + 1e-3f)) ||
                             (fabsf(abs_zone - state->last_zone) > 1e-3f);

    if (target_or_zone_changed)
    {
        // 目标变化后重新开始这一轮死区等待
        state->start_tick = now_tick;
        state->active = true;
        state->last_target = target;
        state->last_zone = abs_zone;
        return false;
    }

    if (enable_timeout_pass &&
        state->active &&
        ((uint32_t)(now_tick - state->start_tick) >= MOTOR_DEADZONE_TIMEOUT_MS))
    {
        // 仅在允许超时放行时，才把超时视为“到位”
        state->active = false;
        state->last_target = target;
        state->last_zone = abs_zone;
        return true;
    }

    return false;
}
