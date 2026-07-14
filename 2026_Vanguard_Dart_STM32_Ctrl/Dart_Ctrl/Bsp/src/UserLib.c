#include "UserLib.h"
#include "config.h"
#include "main.h"
#include <math.h>

/***********************************
 * 函数名: IsInDeadzoneTimedF
 * 作用:   带调用方自定义超时的浮点死区判定
 *         常用于“允许超时按到位处理”的流程控制
 **********************************/
bool IsInDeadzoneTimedF(float value, float target, float zone, uint32_t timeout_ms, DeadzoneTimer_t *timer) /* 实现 IsInDeadzoneTimedF。 */
{
    float abs_zone; /* 保存 abs_zone。 */
    uint32_t now_tick; /* 保存 now_tick。 */
    bool target_changed; /* 保存 target_changed。 */

    if (timer == NULL) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }

    if (!isfinite(value) || !isfinite(target) || !isfinite(zone)) /* 检查当前执行条件。 */
    {
        timer->active = false; /* 更新 active。 */
        return false; /* 返回 false。 */
    }

    abs_zone = fabsf(zone); /* 更新 abs_zone。 */
    if (fabsf(value - target) <= abs_zone) /* 检查当前执行条件。 */
    {
        // 已真实进入死区，清掉本轮计时状态
        timer->active = false; /* 更新 active。 */
        timer->last_target = target; /* 更新 last_target。 */
        timer->last_zone = abs_zone; /* 更新 last_zone。 */
        return true; /* 返回 true。 */
    }

    now_tick = HAL_GetTick(); /* 更新 now_tick。 */
    target_changed = (!timer->active) || /* 继续更新 target_changed。 */
                     (fabsf(target - timer->last_target) > (abs_zone + 1e-3f)) || /* 继续组合表达式。 */
                     (fabsf(abs_zone - timer->last_zone) > 1e-3f); /* 完成本行操作。 */
    if (target_changed) /* 检查当前执行条件。 */
    {
        // 新目标或新死区到来时，重新开始计时
        timer->start_tick = now_tick; /* 更新 start_tick。 */
        timer->last_target = target; /* 更新 last_target。 */
        timer->last_zone = abs_zone; /* 更新 last_zone。 */
        timer->active = true; /* 更新 active。 */
        return false; /* 返回 false。 */
    }

    if ((uint32_t)(now_tick - timer->start_tick) >= timeout_ms) /* 检查当前执行条件。 */
    {
        // 超时后按到位处理，避免上层流程永久阻塞
        timer->active = false; /* 更新 active。 */
        timer->last_target = target; /* 更新 last_target。 */
        timer->last_zone = abs_zone; /* 更新 last_zone。 */
        return true; /* 返回 true。 */
    }

    return false; /* 返回 false。 */
}

/***********************************
 * 函数名: IsInDeadzoneF
 * 作用:   通用浮点死区判定
 *         是否允许超时放行由 enable_timeout_pass 决定
 **********************************/
bool IsInDeadzoneF(float value, float target, float zone, DeadzoneState_t *state, bool enable_timeout_pass) /* 实现 IsInDeadzoneF。 */
{
    float abs_zone; /* 保存 abs_zone。 */
    bool in_deadzone; /* 保存 in_deadzone。 */
    uint32_t now_tick; /* 保存 now_tick。 */
    bool target_or_zone_changed; /* 保存 target_or_zone_changed。 */

    if (state == NULL) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }

    if (!isfinite(value) || !isfinite(target) || !isfinite(zone)) /* 检查当前执行条件。 */
    {
        state->active = false; /* 更新 active。 */
        return false; /* 返回 false。 */
    }

    abs_zone = fabsf(zone); /* 更新 abs_zone。 */
    in_deadzone = (fabsf(value - target) <= abs_zone); /* 更新 in_deadzone。 */
    if (in_deadzone) /* 检查当前执行条件。 */
    {
        // 已真实进入死区，直接返回到位
        state->active = false; /* 更新 active。 */
        state->last_target = target; /* 更新 last_target。 */
        state->last_zone = abs_zone; /* 更新 last_zone。 */
        return true; /* 返回 true。 */
    }

    now_tick = HAL_GetTick(); /* 更新 now_tick。 */
    target_or_zone_changed = (!state->active) || /* 继续更新 target_or_zone_changed。 */
                             (fabsf(target - state->last_target) > (abs_zone + 1e-3f)) || /* 继续组合表达式。 */
                             (fabsf(abs_zone - state->last_zone) > 1e-3f); /* 完成本行操作。 */

    if (target_or_zone_changed) /* 检查当前执行条件。 */
    {
        // 目标变化后重新开始这一轮死区等待
        state->start_tick = now_tick; /* 更新 start_tick。 */
        state->active = true; /* 更新 active。 */
        state->last_target = target; /* 更新 last_target。 */
        state->last_zone = abs_zone; /* 更新 last_zone。 */
        return false; /* 返回 false。 */
    }

    if (enable_timeout_pass && /* 检查当前执行条件。 */
        state->active && /* 继续组合表达式。 */
        ((uint32_t)(now_tick - state->start_tick) >= MOTOR_DEADZONE_TIMEOUT_MS)) /* 继续更新 目标值。 */
    {
        // 仅在允许超时放行时，才把超时视为“到位”
        state->active = false; /* 更新 active。 */
        state->last_target = target; /* 更新 last_target。 */
        state->last_zone = abs_zone; /* 更新 last_zone。 */
        return true; /* 返回 true。 */
    }

    return false; /* 返回 false。 */
}
