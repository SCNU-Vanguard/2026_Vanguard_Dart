#include "FireControl.h"
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

extern int8_t Channel[13];

static RefereeGateSnapshot_t g_referee_snapshot; // 状态机留存数据
static RC_FireIntent_t g_RC_fire_intent;
static FirePermission_t g_fire_permission;
static FireSetupBaseline_t g_setup_baseline; // 记录之前设定的状态

/**
 * @brief 在临界区内重算统一仲裁结果
 *
 * 输入：
 * - g_referee_snapshot：最新裁判快照
 * - g_RC_fire_intent：最新遥控器业务意图
 * - g_setup_baseline：上一次已经完成的设定基线
 *
 * 输出：
 * - g_fire_permission：统一仲裁后的最终结果
 *
 * 注意：
 * - 该函数本身不加锁，调用者必须已经进入临界区
 * - 所有 Update/Get 快捷接口最终都会经过这里刷新结果
 */
static void FireControl_RecalcLocked(void)
{
    bool fresh = false;
    bool need_reconfigure = false;

    if (g_referee_snapshot.valid)
    {
        fresh = true;
    }

    if (fresh)
    {
        need_reconfigure = (!g_setup_baseline.valid ||
                            g_setup_baseline.target_change_time != g_referee_snapshot.target_change_time ||
                            g_setup_baseline.target_select != g_RC_fire_intent.target_select ||
                            g_setup_baseline.range_select != g_RC_fire_intent.range_select);
    }

    g_fire_permission.can_setup = fresh;
    g_fire_permission.can_shoot = (fresh &&
                                   g_referee_snapshot.opening_status == REFEREE_DART_OPEN &&
                                   g_referee_snapshot.dart_remaining_time >= REFEREE_DART_FIRE_MIN_REMAIN_TIME_S &&
                                   g_RC_fire_intent.fire_enable);
    g_fire_permission.need_reconfigure = need_reconfigure;
    g_fire_permission.abort_current_shot = !g_fire_permission.can_shoot;
    g_fire_permission.referee_fresh = fresh;
    g_fire_permission.opening_status = g_referee_snapshot.opening_status;
    g_fire_permission.remain_time = g_referee_snapshot.remain_time;
    g_fire_permission.dart_remaining_time = g_referee_snapshot.dart_remaining_time;
    g_fire_permission.target_change_time = g_referee_snapshot.target_change_time;
    g_fire_permission.latest_launch_cmd_time = g_referee_snapshot.latest_launch_cmd_time;
    g_fire_permission.ref_seq = g_referee_snapshot.seq;
    g_fire_permission.intent_seq = g_RC_fire_intent.seq;
    g_fire_permission.target_select_end = g_RC_fire_intent.target_select; // 这里需要看逻辑,到时候还是需要一个拨杆

    /* 使用遥控器数据 */
    // if ()
    // {
    //     g_fire_permission.target_select_end = g_RC_fire_intent.target_select;
    // }
    // /* 使用裁判系统数据 */
    // else
    // {
    //     g_fire_permission.target_select_end = g_referee_snapshot.target_select_referee;
    // }
}

/**
 * @brief 初始化 FireControl 内部缓存
 *
 * 作用：
 * - 清空裁判业务快照 g_referee_snapshot
 * - 清空遥控器业务意图 g_RC_fire_intent
 * - 清空统一仲裁结果 g_fire_permission
 * - 清空“已应用设定基线” g_setup_baseline
 *
 * 调用时机：
 * - 系统初始化阶段调用一次即可
 * - 一般放在 TaskInitFunc() 中
 */
void FireControl_Init(void)
{
    taskENTER_CRITICAL();
    memset(&g_referee_snapshot, 0, sizeof(g_referee_snapshot));
    memset(&g_RC_fire_intent, 0, sizeof(g_RC_fire_intent));
    memset(&g_fire_permission, 0, sizeof(g_fire_permission));
    memset(&g_setup_baseline, 0, sizeof(g_setup_baseline));
    g_RC_fire_intent.target_select = FIRE_TARGET_OUTPOST;
    g_RC_fire_intent.range_select = FIRE_RANGE_DEFAULT;
    FireControl_RecalcLocked();
    taskEXIT_CRITICAL();
}

/**
 * @brief 根据最新的飞镖舱门数据刷新裁判快照
 * @param dart_status 最新 ext_dart_launch_status_t
 * @return true-门控关键字段变化；false-无变化
 *
 * 典型调用位置：
 * - RefereeTaskFunc 收到新的 ID_dart_launch_status 之后
 *
 * 刷新的字段包括：
 * - opening_status
 * - target_change_time
 * - latest_launch_cmd_time
 * - local_tick_ms
 * - seq
 *
 * 完成刷新后会立刻重新计算一次统一仲裁结果。
 */
bool FireControl_UpdateRefereeGate(const ext_dart_launch_status_t *dart_status)
{
    bool changed;

    if (dart_status == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    changed = (!g_referee_snapshot.valid ||
               g_referee_snapshot.opening_status != dart_status->dart_launch_opening_status ||
               g_referee_snapshot.target_change_time != dart_status->target_change_time ||
               g_referee_snapshot.latest_launch_cmd_time != dart_status->latest_launch_cmd_time);

    g_referee_snapshot.valid = true;
    g_referee_snapshot.opening_status = dart_status->dart_launch_opening_status;
    g_referee_snapshot.target_change_time = dart_status->target_change_time;
    g_referee_snapshot.latest_launch_cmd_time = dart_status->latest_launch_cmd_time;
    g_referee_snapshot.local_tick_ms = HAL_GetTick();
    g_referee_snapshot.seq++;
    FireControl_RecalcLocked();
    taskEXIT_CRITICAL();

    return changed;
}

bool FireControl_UpdateRefereeRemainTime(uint16_t remain_time)
{
    bool changed;

    taskENTER_CRITICAL();
    changed = (g_referee_snapshot.remain_time != remain_time);
    g_referee_snapshot.remain_time = remain_time;
    FireControl_RecalcLocked();
    taskEXIT_CRITICAL();

    return changed;
}

bool FireControl_UpdateDartInfo(const ext_dart_info_t *dart_info)
{
    bool changed;

    if (dart_info == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    changed = (g_referee_snapshot.dart_remaining_time != dart_info->dart_remaining_time);
    g_referee_snapshot.dart_remaining_time = dart_info->dart_remaining_time;
    g_referee_snapshot.target_select_referee = dart_info->referee_dart_info.dart_info_bia_bits.selected_target;
    FireControl_RecalcLocked();
    taskEXIT_CRITICAL();

    return changed;
}

/**
 * @brief 一次性更新完整遥控器业务意图
 * @param fire_enable 总发射使能
 * @param target_select 当前目标选择
 * @param range_select 当前射程/角度档位
 * @return true-意图有变化；false-三项与当前缓存一致
 *
 * 典型调用位置：
 * - ControlState 或遥控器解析任务
 *
 * 注意：
 * - 这里更新的是业务意图，不是 RawChannel/Channel 这种原始通道值
 * - 更新后会自动重新计算统一仲裁结果
 */
bool FireControl_UpdateIntent(bool fire_enable, uint8_t target_select, uint8_t range_select)
{
    bool changed;

    taskENTER_CRITICAL();
    changed = (g_RC_fire_intent.fire_enable != fire_enable ||
               g_RC_fire_intent.target_select != target_select ||
               g_RC_fire_intent.range_select != range_select);
    if (changed)
    {
        g_RC_fire_intent.fire_enable = fire_enable;
        g_RC_fire_intent.target_select = target_select;
        g_RC_fire_intent.range_select = range_select;
        g_RC_fire_intent.seq++;
    }
    FireControl_RecalcLocked();
    taskEXIT_CRITICAL();

    return changed;
}

bool FireControl_SetFireEnable(bool fire_enable)
{
    RC_FireIntent_t current;

    FireControl_GetIntent(&current);
    return FireControl_UpdateIntent(fire_enable, current.target_select, current.range_select);
}

/**
 * @brief 单独更新目标选择
 * @param target_select 当前目标选择
 * @return true-值有变化；false-未变化
 *
 * 用于只改目标、不改其他两项时的快捷入口。
 */
bool FireControl_SetTargetSelect(uint8_t target_select)
{
    RC_FireIntent_t current;

    FireControl_GetIntent(&current);
    return FireControl_UpdateIntent(current.fire_enable, target_select, current.range_select);
}

/**
 * @brief 单独更新射程/角度档位
 * @param range_select 当前射程/角度档位
 * @return true-值有变化；false-未变化
 *
 * 用于只改射程档位、不改其他两项时的快捷入口。
 */
bool FireControl_SetRangeSelect(uint8_t range_select)
{
    RC_FireIntent_t current;

    FireControl_GetIntent(&current);
    return FireControl_UpdateIntent(current.fire_enable, current.target_select, range_select);
}

/**
 * @brief 标记“当前设定已经成功应用到机构”
 *
 * 作用：
 * - 把当前 target_change_time / target_select / range_select
 *   写入 g_setup_baseline
 * - 让 FireControl_RecalcLocked() 后续可据此判断 need_reconfigure
 *
 * 典型调用位置：
 * - StateSetTaskFunc 完成 2006/4310 设定并确认到位之后
 */
void FireControl_MarkSetupApplied(void)
{
    taskENTER_CRITICAL();
    g_setup_baseline.valid = g_referee_snapshot.valid;
    g_setup_baseline.target_change_time = g_referee_snapshot.target_change_time;
    g_setup_baseline.target_select = g_RC_fire_intent.target_select;
    g_setup_baseline.range_select = g_RC_fire_intent.range_select;
    FireControl_RecalcLocked();
    taskEXIT_CRITICAL();
}

void FireControl_GetRefereeSnapshot(RefereeGateSnapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *snapshot = g_referee_snapshot;
    taskEXIT_CRITICAL();
}

/**
 * @brief 获取当前遥控器业务意图副本
 * @param intent 输出指针
 *
 * 返回的是副本，用于调试或只读查看，
 * 不应该在外部直接修改后假定能写回内部状态。
 */
void FireControl_GetIntent(RC_FireIntent_t *intent)
{
    if (intent == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    *intent = g_RC_fire_intent;
    taskEXIT_CRITICAL();
}

/**
 * @brief 获取当前统一仲裁结果副本
 * @param permission 输出指针
 *
 * 这是外部任务最常用的读取接口。
 * 典型用法：
 * - StateSetTaskFunc 读取 can_setup / need_reconfigure
 * - 发射前读取 can_shoot
 * - 发射中读取 abort_current_shot
 */
void FireControl_GetPermission(FirePermission_t *permission)
{
    if (permission == NULL)
    {
        return;
    }

    taskENTER_CRITICAL();
    FireControl_RecalcLocked();
    *permission = g_fire_permission;
    taskEXIT_CRITICAL();
}

/**
 * @brief 快捷读取：当前是否允许设定
 * @return true-允许；false-不允许
 */
bool FireControl_CanSetup(void)
{
    bool result;

    taskENTER_CRITICAL();
    FireControl_RecalcLocked();
    result = g_fire_permission.can_setup;
    taskEXIT_CRITICAL();

    return result;
}

/**
 * @brief 快捷读取：当前是否允许发射
 * @return true-允许；false-不允许
 */
bool FireControl_CanShoot(void)
{
    bool result;

    taskENTER_CRITICAL();
    FireControl_RecalcLocked();
    result = g_fire_permission.can_shoot;
    taskEXIT_CRITICAL();

    return result;
}

/**
 * @brief 快捷读取：当前是否应中止发射流程
 * @return true-应中止；false-可继续
 */
bool FireControl_ShouldAbortShot(void)
{
    bool result;

    taskENTER_CRITICAL();
    FireControl_RecalcLocked();
    result = g_fire_permission.abort_current_shot;
    taskEXIT_CRITICAL();

    return result;
}

/**
 * @brief 快捷读取：当前击打目标
 * @return FIRE_TARGET_BASE->true,击打基地
 * @note   默认返回的是 FIRE_TARGET_OUTPOST
 */
uint8_t FireControl_SelectTarget(void)
{
    uint8_t result;

    taskENTER_CRITICAL();
    FireControl_RecalcLocked();
    result = (g_fire_permission.target_select_end == FIRE_TARGET_BASE) ? FIRE_TARGET_BASE : FIRE_TARGET_OUTPOST;
    taskEXIT_CRITICAL();
    return result;
}
