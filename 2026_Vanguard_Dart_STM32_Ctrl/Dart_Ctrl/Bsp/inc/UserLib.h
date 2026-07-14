#ifndef __USER_LIB_H_ /* 按 __USER_LIB_H_ 选择编译分支。 */
#define __USER_LIB_H_ /* 定义 __USER_LIB_H_。 */

#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"
#include "stdbool.h"

/**
 * @brief 带自定义超时的死区判定状态
 * @note  同一组 value/target 判定必须复用同一个状态对象，
 *        否则会导致超时计时反复被重置。
 */
typedef struct /* 开始定义数据类型。 */
{
    uint32_t start_tick; // 本轮判定开始时刻
    float last_target;   // 上一次参与判定的目标值
    float last_zone;     // 上一次参与判定的死区范围
    bool active;         // 当前是否正在计时
} DeadzoneTimer_t; /* 结束 DeadzoneTimer_t 类型定义。 */

/**
 * @brief 通用死区判定状态
 * @note  与 DeadzoneTimer_t 结构保持一致，语义上区分为“普通死区判定”场景。
 */
typedef struct /* 开始定义数据类型。 */
{
    uint32_t start_tick; // 本轮判定开始时刻
    float last_target;   // 上一次参与判定的目标值
    float last_zone;     // 上一次参与判定的死区范围
    bool active;         // 当前是否正在计时
} DeadzoneState_t; /* 结束 DeadzoneState_t 类型定义。 */

/**
 * @brief 浮点死区判定，支持调用方指定超时时间
 * @param value       当前测量值
 * @param target      目标值
 * @param zone        死区范围，函数内部按绝对值处理
 * @param timeout_ms  超时时间，超过后按“已到位”处理
 * @param timer       判定状态对象，不能为 NULL
 * @retval true  已进入死区，或已超时放行
 * @retval false 尚未到位，继续等待
 * @note  适合 3508 这类需要避免流程卡死的场景。
 */
bool IsInDeadzoneTimedF(float value, float target, float zone, uint32_t timeout_ms, DeadzoneTimer_t *timer); /* 声明 IsInDeadzoneTimedF 接口。 */

/**
 * @brief 浮点死区判定，超时策略固定为 MOTOR_DEADZONE_TIMEOUT_MS
 * @param value                当前测量值
 * @param target               目标值
 * @param zone                 死区范围，函数内部按绝对值处理
 * @param state                判定状态对象，不能为 NULL
 * @param enable_timeout_pass  true: 超时后放行；false: 必须真实进入死区才算到位
 * @retval true  已进入死区，或允许超时放行且已超时
 * @retval false 尚未到位，继续等待
 * @note  适合把“允许超时”和“必须硬到位”两类逻辑统一到一个接口里。
 */
bool IsInDeadzoneF(float value, float target, float zone, DeadzoneState_t *state, bool enable_timeout_pass); /* 声明 IsInDeadzoneF 接口。 */

#endif /* 结束条件编译。 */
