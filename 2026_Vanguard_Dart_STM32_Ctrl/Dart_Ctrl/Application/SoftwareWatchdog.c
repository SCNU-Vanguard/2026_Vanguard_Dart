#include "SoftwareWatchdog.h"
#include <stddef.h>
#include <string.h>

typedef struct /* 开始定义数据类型。 */
{
    volatile uint32_t feed_seq;                  /* 数据源每次喂狗都递增。 */
    uint32_t seen_feed_seq;                      /* 1 ms 中断已处理的序号。 */
    volatile uint32_t elapsed_ms;                /* 距离最近数据的毫秒数。 */
    uint32_t timeout_ms;                         /* 当前通道超时阈值。 */
    SoftwareWatchdogLostCallback_t lost_callback; /* 首次超时回调。 */
    volatile uint8_t enabled;                    /* 1 表示正在计时。 */
    volatile uint8_t has_fed;                    /* 1 表示收到过有效数据。 */
    volatile uint8_t timed_out;                  /* 1 表示已经超时。 */
} SoftwareWatchdogState_t; /* 结束 SoftwareWatchdogState_t 类型定义。 */

static SoftwareWatchdogState_t s_watchdogs[SOFTWARE_WATCHDOG_COUNT]; /* 全部通道状态。 */
static volatile uint8_t s_initialized = 0U;                          /* 模块初始化标志。 */

void SoftwareWatchdog_Init(void) /* 实现 SoftwareWatchdog_Init。 */
{
    s_initialized = 0U;                         /* 先阻止 1 ms 中断访问。 */
    memset(s_watchdogs, 0, sizeof(s_watchdogs)); /* 清空所有历史状态。 */
    s_initialized = 1U;                         /* 状态清理完成后开放访问。 */
}

bool SoftwareWatchdog_Register(SoftwareWatchdogId_e id, /* 传入下一项参数或数据。 */
                               uint32_t timeout_ms, /* 传入下一项参数或数据。 */
                               SoftwareWatchdogLostCallback_t lost_callback) /* 继续当前语句。 */
{
    SoftwareWatchdogState_t *watchdog; /* 当前注册通道。 */

    /* 拒绝未初始化、越界或零超时配置。 */
    if (s_initialized == 0U || (uint32_t)id >= (uint32_t)SOFTWARE_WATCHDOG_COUNT || timeout_ms == 0U) /* 检查当前执行条件。 */
    {
        return false; /* 参数无效。 */
    }

    watchdog = &s_watchdogs[id];               /* 取得目标通道。 */
    watchdog->enabled = 0U;                    /* 配置期间暂停计时。 */
    watchdog->feed_seq = 0U;                   /* 清喂狗序号。 */
    watchdog->seen_feed_seq = 0U;              /* 清已处理序号。 */
    watchdog->elapsed_ms = 0U;                 /* 从零开始计时。 */
    watchdog->timeout_ms = timeout_ms;          /* 保存超时阈值。 */
    watchdog->lost_callback = lost_callback;   /* 保存丢失回调。 */
    watchdog->has_fed = 0U;                    /* 尚未收到数据。 */
    watchdog->timed_out = 0U;                  /* 尚未发生超时。 */
    watchdog->enabled = 1U;                    /* 连续通道立即开始计时。 */
    return true;                               /* 注册成功。 */
}

void SoftwareWatchdog_Feed(SoftwareWatchdogId_e id) /* 实现 SoftwareWatchdog_Feed。 */
{
    SoftwareWatchdogState_t *watchdog; /* 当前喂狗通道。 */

    /* 无效通道直接忽略。 */
    if (s_initialized == 0U || (uint32_t)id >= (uint32_t)SOFTWARE_WATCHDOG_COUNT) /* 检查当前执行条件。 */
    {
        return; /* 模块或编号无效。 */
    }

    watchdog = &s_watchdogs[id]; /* 取得通道状态。 */
    if (watchdog->enabled == 0U) /* 检查当前执行条件。 */
    {
        return; /* 未启动的通道不接受喂狗。 */
    }

    watchdog->feed_seq++;       /* 发布一次新数据事件。 */
    watchdog->elapsed_ms = 0U;  /* 立即清超时计数。 */
    watchdog->has_fed = 1U;     /* 标记数据已经到达。 */
    watchdog->timed_out = 0U;   /* 允许链路恢复在线。 */
}

bool SoftwareWatchdog_Arm(SoftwareWatchdogId_e id) /* 实现 SoftwareWatchdog_Arm。 */
{
    SoftwareWatchdogState_t *watchdog; /* 当前一次性通道。 */

    /* 无效通道不能启动。 */
    if (s_initialized == 0U || (uint32_t)id >= (uint32_t)SOFTWARE_WATCHDOG_COUNT) /* 检查当前执行条件。 */
    {
        return false; /* 模块或编号无效。 */
    }

    watchdog = &s_watchdogs[id]; /* 取得通道状态。 */
    if (watchdog->timeout_ms == 0U) /* 检查当前执行条件。 */
    {
        return false; /* 通道尚未注册。 */
    }

    watchdog->enabled = 0U;                         /* 重装期间暂停中断计数。 */
    watchdog->seen_feed_seq = watchdog->feed_seq;   /* 忽略以前的喂狗事件。 */
    watchdog->elapsed_ms = 0U;                      /* 回复等待从零开始。 */
    watchdog->has_fed = 1U;                         /* Arm 本身就是有效起点。 */
    watchdog->timed_out = 0U;                       /* 清除上次超时。 */
    watchdog->enabled = 1U;                         /* 正式启动一次性等待。 */
    return true;                                    /* 启动成功。 */
}

void SoftwareWatchdog_Disarm(SoftwareWatchdogId_e id) /* 实现 SoftwareWatchdog_Disarm。 */
{
    SoftwareWatchdogState_t *watchdog; /* 当前一次性通道。 */

    /* 无效通道无需处理。 */
    if (s_initialized == 0U || (uint32_t)id >= (uint32_t)SOFTWARE_WATCHDOG_COUNT) /* 检查当前执行条件。 */
    {
        return; /* 模块或编号无效。 */
    }

    watchdog = &s_watchdogs[id]; /* 取得通道状态。 */
    watchdog->enabled = 0U;      /* 立即停止计时。 */
    watchdog->elapsed_ms = 0U;   /* 清除回复等待时间。 */
    watchdog->has_fed = 0U;      /* 回到未启动状态。 */
    watchdog->timed_out = 0U;    /* 清除一次性超时结果。 */
}

void SoftwareWatchdog_Tick1msFromISR(void) /* 实现 SoftwareWatchdog_Tick1msFromISR。 */
{
    uint32_t i; /* 通道遍历索引。 */

    if (s_initialized == 0U) /* 检查当前执行条件。 */
    {
        return; /* 初始化期间不处理计时。 */
    }

    /* 每毫秒扫描全部固定通道。 */
    for (i = 0U; i < (uint32_t)SOFTWARE_WATCHDOG_COUNT; i++) /* 遍历当前数据集合。 */
    {
        SoftwareWatchdogState_t *watchdog = &s_watchdogs[i]; /* 当前通道。 */
        uint32_t feed_seq;                                    /* 本次读取的喂狗序号。 */

        if (watchdog->enabled == 0U) /* 检查当前执行条件。 */
        {
            continue; /* 停用通道不计时。 */
        }

        feed_seq = watchdog->feed_seq; /* 原子读取数据源序号。 */
        if (feed_seq != watchdog->seen_feed_seq) /* 检查当前执行条件。 */
        {
            watchdog->seen_feed_seq = feed_seq; /* 消费最新喂狗事件。 */
            watchdog->elapsed_ms = 0U;          /* 新数据使计时归零。 */
            watchdog->has_fed = 1U;             /* 链路已有有效数据。 */
            watchdog->timed_out = 0U;           /* 链路恢复正常。 */
            continue;                           /* 本周期不再累加。 */
        }

        if (watchdog->elapsed_ms < watchdog->timeout_ms) /* 检查当前执行条件。 */
        {
            watchdog->elapsed_ms++; /* 尚未到阈值，累加 1 ms。 */
        }

        /* 只在首次到达阈值时触发回调。 */
        if (watchdog->elapsed_ms >= watchdog->timeout_ms && watchdog->timed_out == 0U) /* 检查当前执行条件。 */
        {
            watchdog->timed_out = 1U; /* 先锁存超时，防止重复回调。 */
            if (watchdog->lost_callback != NULL) /* 检查当前执行条件。 */
            {
                watchdog->lost_callback((SoftwareWatchdogId_e)i); /* 回调只允许置标志。 */
            }
        }
    }
}

bool SoftwareWatchdog_IsHealthy(SoftwareWatchdogId_e id) /* 实现 SoftwareWatchdog_IsHealthy。 */
{
    const SoftwareWatchdogState_t *watchdog; /* 只读通道指针。 */

    /* 无效通道按离线处理。 */
    if (s_initialized == 0U || (uint32_t)id >= (uint32_t)SOFTWARE_WATCHDOG_COUNT) /* 检查当前执行条件。 */
    {
        return false; /* 查询失败即不健康。 */
    }

    watchdog = &s_watchdogs[id]; /* 取得只读状态。 */
    /* 必须已启动、收到过数据且未超时。 */
    return (watchdog->enabled != 0U && watchdog->has_fed != 0U && watchdog->timed_out == 0U); /* 返回当前计算结果。 */
}

bool SoftwareWatchdog_IsTimedOut(SoftwareWatchdogId_e id) /* 实现 SoftwareWatchdog_IsTimedOut。 */
{
    /* 无效通道按超时处理。 */
    if (s_initialized == 0U || (uint32_t)id >= (uint32_t)SOFTWARE_WATCHDOG_COUNT) /* 检查当前执行条件。 */
    {
        return true; /* 查询失败时采用安全值。 */
    }
    /* 停用或已锁存超时都返回 true。 */
    return (s_watchdogs[id].enabled == 0U || s_watchdogs[id].timed_out != 0U); /* 返回当前计算结果。 */
}

SoftwareWatchdogId_e SoftwareWatchdog_MotorId(uint8_t motor_index_one_based) /* 实现 SoftwareWatchdog_MotorId。 */
{
    /* 当前工程最多管理 5 路 CAN 电机。 */
    if (motor_index_one_based < 1U || motor_index_one_based > 5U) /* 检查当前执行条件。 */
    {
        return SOFTWARE_WATCHDOG_COUNT; /* 返回无效哨兵值。 */
    }
    /* 电机 1..5 连续映射到对应看门狗编号。 */
    return (SoftwareWatchdogId_e)((uint32_t)SOFTWARE_WATCHDOG_CAN_MOTOR_1 + /* 继续组合表达式。 */
                                  (uint32_t)motor_index_one_based - 1U); /* 完成本行操作。 */
}
