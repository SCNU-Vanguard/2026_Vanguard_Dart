#include "FireControl.h"
#include "config.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

extern int8_t Channel[13]; /* 声明外部变量 Channel。 */

static RefereeGateSnapshot_t g_referee_snapshot; // 状态机留存数据
static RC_FireIntent_t g_RC_fire_intent; /* 保存 g_RC_fire_intent。 */
static FirePermission_t g_fire_permission; /* 保存 g_fire_permission。 */
static volatile uint8_t s_referee_watchdog_lost = 0U; /* 初始化 s_referee_watchdog_lost。 */
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
 *
 * 仲裁模式 (由 config.h 中 TRUST_SHOOT_DART_DATA 决定)：
 * - TRUST_REFEREE_DART_DATA        : 纯裁判, 忽略遥控器业务意图
 * - TRUST_RCandREFEREE_DART_DATA   : 目标来自裁判, 发射/重配综合裁判与遥控器
 * - TRUST_REMOTE_CONTROL_DART_DATA : 纯遥控器 (留白, 待补充)
 */
static void FireControl_RecalcLocked(void) /* 实现 FireControl_RecalcLocked。 */
{
    /* 历史 valid、LostCallback 标志和 1500 ms 看门狗必须同时正常。 */
    const bool fresh = (g_referee_snapshot.valid && s_referee_watchdog_lost == 0U && /* 开始计算 fresh。 */
                        SoftwareWatchdog_IsHealthy(SOFTWARE_WATCHDOG_REFEREE_GATE)); /* 调用 SoftwareWatchdog_IsHealthy。 */
    bool need_reconfigure = false; /* 初始化 need_reconfigure。 */

    g_fire_permission.referee_fresh = fresh; /* 更新 referee_fresh。 */
    g_fire_permission.can_setup = fresh; /* 更新 can_setup。 */
    g_fire_permission.opening_status = g_referee_snapshot.opening_status; /* 更新 opening_status。 */
    g_fire_permission.remain_time = g_referee_snapshot.remain_time; /* 更新 remain_time。 */
    g_fire_permission.dart_remaining_time = g_referee_snapshot.dart_remaining_time; /* 更新 dart_remaining_time。 */
    g_fire_permission.target_change_time = g_referee_snapshot.target_change_time; /* 更新 target_change_time。 */
    g_fire_permission.latest_launch_cmd_time = g_referee_snapshot.latest_launch_cmd_time; /* 更新 latest_launch_cmd_time。 */
    g_fire_permission.ref_seq = g_referee_snapshot.seq; /* 更新 ref_seq。 */
    g_fire_permission.intent_seq = g_RC_fire_intent.seq; /* 更新 intent_seq。 */

#if (TRUST_SHOOT_DART_DATA == TRUST_REFEREE_DART_DATA) /* 按 (TRUST_SHOOT_DART_DATA == TRUST_REFEREE_DART_DATA) 选择编译分支。 */
    /* 纯裁判系统模式：目标与发射门禁完全由裁判决定，遥控器业务意图不参与仲裁 */
    if (fresh) /* 检查当前执行条件。 */
    {
        need_reconfigure = (!g_setup_baseline.valid || /* 继续更新 need_reconfigure。 */
                            g_setup_baseline.target_change_time != g_referee_snapshot.target_change_time); /* 更新 target_change_time。 */
    }

    g_fire_permission.can_shoot = (fresh && /* 继续组合表达式。 */
                                   g_referee_snapshot.opening_status == REFEREE_DART_OPEN && /* 继续组合表达式。 */
                                   g_referee_snapshot.dart_remaining_time >= REFEREE_DART_FIRE_MIN_REMAIN_TIME_S); /* 更新 dart_remaining_time。 */
    g_fire_permission.target_select_end = g_referee_snapshot.target_select_referee; /* 更新 target_select_end。 */

#elif (TRUST_SHOOT_DART_DATA == TRUST_RCandREFEREE_DART_DATA) /* 继续检查 (TRUST_SHOOT_DART_DATA == TRUST_RCandREFEREE_DART_DATA)。 */
    /* 裁判 + 遥控器综合模式：目标仍以裁判为准，发射需遥控器使能，重配同时关注遥控器目标/射程 */
    if (fresh) /* 检查当前执行条件。 */
    {
        need_reconfigure = (!g_setup_baseline.valid || /* 继续更新 need_reconfigure。 */
                            g_setup_baseline.target_change_time != g_referee_snapshot.target_change_time || /* 继续组合表达式。 */
                            g_setup_baseline.target_select != g_RC_fire_intent.target_select || /* 继续组合表达式。 */
                            g_setup_baseline.range_select != g_RC_fire_intent.range_select); /* 更新 range_select。 */
    }

    g_fire_permission.can_shoot = (fresh && /* 继续组合表达式。 */
                                   g_referee_snapshot.opening_status == REFEREE_DART_OPEN && /* 继续组合表达式。 */
                                   g_referee_snapshot.dart_remaining_time >= REFEREE_DART_FIRE_MIN_REMAIN_TIME_S && /* 继续组合表达式。 */
                                   g_RC_fire_intent.fire_enable); /* 完成本行操作。 */
    g_fire_permission.target_select_end = g_referee_snapshot.target_select_referee; /* 更新 target_select_end。 */

#elif (TRUST_SHOOT_DART_DATA == TRUST_REMOTE_CONTROL_DART_DATA) /* 继续检查 (TRUST_SHOOT_DART_DATA == TRUST_REMOTE_CONTROL_DART_DATA)。 */
#error "TRUST_REMOTE_CONTROL_DART_DATA 模式尚未实现, 请在 FireControl_RecalcLocked 中补充纯遥控器仲裁逻辑" /* 报告无效编译配置。 */
#else /* 切换到备用编译分支。 */
#error "Unknown TRUST_SHOOT_DART_DATA value" /* 报告无效编译配置。 */
#endif /* 结束条件编译。 */

    g_fire_permission.need_reconfigure = need_reconfigure; /* 更新 need_reconfigure。 */
    g_fire_permission.abort_current_shot = !g_fire_permission.can_shoot; /* 更新 abort_current_shot。 */
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
void FireControl_Init(void) /* 实现 FireControl_Init。 */
{
    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    memset(&g_referee_snapshot, 0, sizeof(g_referee_snapshot)); /* 调用 memset。 */
    memset(&g_RC_fire_intent, 0, sizeof(g_RC_fire_intent)); /* 调用 memset。 */
    memset(&g_fire_permission, 0, sizeof(g_fire_permission)); /* 调用 memset。 */
    memset(&g_setup_baseline, 0, sizeof(g_setup_baseline)); /* 调用 memset。 */
    g_RC_fire_intent.target_select = FIRE_TARGET_OUTPOST; /* 更新 target_select。 */
    g_RC_fire_intent.range_select = FIRE_RANGE_DEFAULT; /* 更新 range_select。 */
    FireControl_RecalcLocked(); /* 调用 FireControl_RecalcLocked。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
}

void FireControl_WatchdogInit(void) /* 实现 FireControl_WatchdogInit。 */
{
    s_referee_watchdog_lost = 0U; /* 初始化为未丢失。 */
    (void)SoftwareWatchdog_Register(SOFTWARE_WATCHDOG_REFEREE_GATE, /* 开始调用 SoftwareWatchdog_Register。 */
                                    REFEREE_GATE_FRESH_TIMEOUT_MS, /* 定义 REFEREE_GATE_FRESH_TIMEOUT_MS 枚举项。 */
                                    FireControl_RefereeLostCallback); /* 完成本行操作。 */
}

void FireControl_RefereeLostCallback(SoftwareWatchdogId_e id) /* 实现 FireControl_RefereeLostCallback。 */
{
    if (id == SOFTWARE_WATCHDOG_REFEREE_GATE) /* 检查当前执行条件。 */
    {
        s_referee_watchdog_lost = 1U; /* 中断内只发布丢失标志。 */
    }
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
bool FireControl_UpdateRefereeGate(const ext_dart_launch_status_t *dart_status) /* 实现 FireControl_UpdateRefereeGate。 */
{
    bool changed; /* 保存 changed。 */

    if (dart_status == NULL) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    changed = (!g_referee_snapshot.valid || /* 继续更新 changed。 */
               g_referee_snapshot.opening_status != dart_status->dart_launch_opening_status || /* 继续组合表达式。 */
               g_referee_snapshot.target_change_time != dart_status->target_change_time || /* 继续组合表达式。 */
               g_referee_snapshot.latest_launch_cmd_time != dart_status->latest_launch_cmd_time); /* 更新 latest_launch_cmd_time。 */

    g_referee_snapshot.valid = true; /* 更新 valid。 */
    g_referee_snapshot.opening_status = dart_status->dart_launch_opening_status; /* 更新 opening_status。 */
    g_referee_snapshot.target_change_time = dart_status->target_change_time; /* 更新 target_change_time。 */
    g_referee_snapshot.latest_launch_cmd_time = dart_status->latest_launch_cmd_time; /* 更新 latest_launch_cmd_time。 */
    g_referee_snapshot.local_tick_ms = HAL_GetTick(); /* 更新 local_tick_ms。 */
    g_referee_snapshot.seq++; /* 完成本行操作。 */
    s_referee_watchdog_lost = 0U;                           /* 新门控帧恢复通信状态。 */
    SoftwareWatchdog_Feed(SOFTWARE_WATCHDOG_REFEREE_GATE); /* 重新开始 1500 ms 计时。 */
    FireControl_RecalcLocked(); /* 调用 FireControl_RecalcLocked。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    return changed; /* 返回当前计算结果。 */
}

bool FireControl_UpdateRefereeRemainTime(uint16_t remain_time) /* 实现 FireControl_UpdateRefereeRemainTime。 */
{
    bool changed; /* 保存 changed。 */

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    changed = (g_referee_snapshot.remain_time != remain_time); /* 更新 changed。 */
    g_referee_snapshot.remain_time = remain_time; /* 更新 remain_time。 */
    FireControl_RecalcLocked(); /* 调用 FireControl_RecalcLocked。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    return changed; /* 返回当前计算结果。 */
}

bool FireControl_UpdateDartInfo(const ext_dart_info_t *dart_info) /* 实现 FireControl_UpdateDartInfo。 */
{
    bool changed; /* 保存 changed。 */

    if (dart_info == NULL) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    changed = (g_referee_snapshot.dart_remaining_time != dart_info->dart_remaining_time); /* 更新 changed。 */
    g_referee_snapshot.dart_remaining_time = dart_info->dart_remaining_time; /* 更新 dart_remaining_time。 */
    g_referee_snapshot.target_select_referee = dart_info->referee_dart_info.dart_info_bia_bits.selected_target; /* 更新 target_select_referee。 */
    FireControl_RecalcLocked(); /* 调用 FireControl_RecalcLocked。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    return changed; /* 返回当前计算结果。 */
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
bool FireControl_UpdateIntent(bool fire_enable, uint8_t target_select, uint8_t range_select) /* 实现 FireControl_UpdateIntent。 */
{
    bool changed; /* 保存 changed。 */

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    changed = (g_RC_fire_intent.fire_enable != fire_enable || /* 继续更新 changed。 */
               g_RC_fire_intent.target_select != target_select || /* 继续组合表达式。 */
               g_RC_fire_intent.range_select != range_select); /* 更新 range_select。 */
    if (changed) /* 检查当前执行条件。 */
    {
        g_RC_fire_intent.fire_enable = fire_enable; /* 更新 fire_enable。 */
        g_RC_fire_intent.target_select = target_select; /* 更新 target_select。 */
        g_RC_fire_intent.range_select = range_select; /* 更新 range_select。 */
        g_RC_fire_intent.seq++; /* 完成本行操作。 */
    }
    FireControl_RecalcLocked(); /* 调用 FireControl_RecalcLocked。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    return changed; /* 返回当前计算结果。 */
}

bool FireControl_SetFireEnable(bool fire_enable) /* 实现 FireControl_SetFireEnable。 */
{
    RC_FireIntent_t current; /* 保存 current。 */

    FireControl_GetIntent(&current); /* 调用 FireControl_GetIntent。 */
    return FireControl_UpdateIntent(fire_enable, current.target_select, current.range_select); /* 返回当前计算结果。 */
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
void FireControl_MarkSetupApplied(void) /* 实现 FireControl_MarkSetupApplied。 */
{
    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    g_setup_baseline.valid = g_referee_snapshot.valid; /* 更新 valid。 */
    g_setup_baseline.target_change_time = g_referee_snapshot.target_change_time; /* 更新 target_change_time。 */
    g_setup_baseline.target_select = g_RC_fire_intent.target_select; /* 更新 target_select。 */
    g_setup_baseline.range_select = g_RC_fire_intent.range_select; /* 更新 range_select。 */
    FireControl_RecalcLocked(); /* 调用 FireControl_RecalcLocked。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
}

/**
 * @brief 获取当前遥控器业务意图副本
 * @param intent 输出指针
 *
 * 返回的是副本，用于调试或只读查看，
 * 不应该在外部直接修改后假定能写回内部状态。
 */
void FireControl_GetIntent(RC_FireIntent_t *intent) /* 实现 FireControl_GetIntent。 */
{
    if (intent == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    *intent = g_RC_fire_intent; /* 更新 intent。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
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
void FireControl_GetPermission(FirePermission_t *permission) /* 实现 FireControl_GetPermission。 */
{
    if (permission == NULL) /* 检查当前执行条件。 */
    {
        return; /* 结束当前函数。 */
    }

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    FireControl_RecalcLocked(); /* 调用 FireControl_RecalcLocked。 */
    *permission = g_fire_permission; /* 更新 permission。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
}

/**
 * @brief 快捷读取：当前是否允许设定
 * @return true-允许；false-不允许
 */
bool FireControl_CanSetup(void) /* 实现 FireControl_CanSetup。 */
{
    bool result; /* 保存 result。 */

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    FireControl_RecalcLocked(); /* 调用 FireControl_RecalcLocked。 */
    result = g_fire_permission.can_setup; /* 更新 result。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    return result; /* 返回当前计算结果。 */
}

/**
 * @brief 快捷读取：当前击打目标
 * @return FIRE_TARGET_BASE->true,击打基地
 * @note   默认返回的是 FIRE_TARGET_OUTPOST
 */
uint8_t FireControl_SelectTarget(void) /* 实现 FireControl_SelectTarget。 */
{
    uint8_t result; /* 保存 result。 */

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    FireControl_RecalcLocked(); /* 调用 FireControl_RecalcLocked。 */
    result = (g_fire_permission.target_select_end == FIRE_TARGET_BASE) ? FIRE_TARGET_BASE : FIRE_TARGET_OUTPOST; /* 更新 result。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
    return result; /* 返回当前计算结果。 */
}
