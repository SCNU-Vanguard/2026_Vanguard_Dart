#ifndef __FIRE_CONTROL_H_
#define __FIRE_CONTROL_H_

#include "main.h"
#include "rm_referee_protocol.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    FIRE_TARGET_OUTPOST = 0U,
    FIRE_TARGET_BASE = 1U,
} FireTargetSelect_e;

typedef enum
{
    FIRE_RANGE_DEFAULT = 0U,
    FIRE_RANGE_1 = 1U,
    FIRE_RANGE_2 = 2U,
} FireRangeSelect_e;

/**
 * @brief 裁判系统门控业务快照
 *
 * 说明：
 * - 该结构体不是协议原始帧的直接映射，而是 FireControl 内部维护的业务层快照
 * - 它以 ext_dart_launch_status_t 为核心，同时补充了剩余时间、本地时间戳、更新序号等字段
 *
 * 典型用途：
 * - 判断裁判舱门数据是否有效、是否超时
 * - 给设定任务/发射任务提供稳定的只读快照
 */
typedef struct
{
    bool valid;                      // 是否已经收到过有效裁判帧
    uint8_t opening_status;          // 当前舱门状态：OPEN/CLOSE/MID
    uint16_t target_change_time;     // 裁判系统目标切换时间
    uint16_t latest_launch_cmd_time; // 裁判系统最近一次发射命令时间
    uint16_t remain_time;            // 比赛阶段剩余时间（来自 game_state）
    uint8_t dart_remaining_time;     // 飞镖发射剩余时间（来自 dart_info）
    uint32_t local_tick_ms;          // 本地接收到这组裁判数据时的 HAL_GetTick()
    uint32_t seq;                    // 本地快照更新序号，每次门控状态更新后自增
    uint8_t target_select_referee;   // 裁判系统选择的目标
} RefereeGateSnapshot_t;             // 裁判系统的数据信息

/**
 * @brief 遥控器业务意图
 *
 * 注意：
 * - 这里保存的不是 RawChannel/Channel 这样的底层原始通道值
 * - 这里保存的是经过遥控器解析层提炼后的业务输入
 *
 * 当前只保留三类意图：
 * - fire_enable  : 总发射使能
 * - target_select: 目标选择
 * - range_select : 射程/角度档位
 *
 * 更新方式：
 * - 推荐由 ControlState 或遥控器解析任务调用 FireControl_UpdateIntent() / SetXXX()
 */
typedef struct
{
    bool fire_enable;      // 遥控器总发射使能
    uint8_t target_select; // 当前目标选择
    uint8_t range_select;  // 当前射程/角度档位
    uint32_t seq;          // 意图更新序号，每次意图变化后自增
} RC_FireIntent_t;         // 遥控器的允许开火和手动调节指令

/**
 * @brief FireControl 统一仲裁输出结果
 *
 * 这是外部任务应该直接消费的结果结构体。
 * 它由以下三类数据综合计算得出：
 * - 裁判系统快照 RefereeGateSnapshot_t
 * - 遥控器意图 RC_FireIntent_t
 * - 内部已应用设定基线 FireSetupBaseline_t（在 FireControl.c 内部维护）
 *
 * 推荐用法：
 * - StateSetTaskFunc 关注 can_setup / need_reconfigure
 * - 发射前最终门禁关注 can_shoot
 * - 储能/发射中途关注 abort_current_shot
 */
typedef struct
{
    bool can_setup;                  // 当前是否允许重新设置 2006/4310
    bool can_shoot;                  // 当前是否允许真正发射
    bool need_reconfigure;           // 当前目标/射程是否需要重新设定
    bool abort_current_shot;         // 当前是否应中止发射流程；通常等价于 !can_shoot
    bool referee_fresh;              // 裁判数据是否未超时
    uint8_t opening_status;          // 当前舱门状态镜像
    uint16_t remain_time;            // 当前剩余时间镜像
    uint8_t dart_remaining_time;     // 当前飞镖发射剩余时间镜像
    uint16_t target_change_time;     // 当前目标切换时间镜像
    uint16_t latest_launch_cmd_time; // 当前最近发射命令时间镜像
    uint32_t ref_seq;                // 本次仲裁使用的裁判快照序号
    uint32_t intent_seq;             // 本次仲裁使用的遥控器意图序号
    uint8_t target_select_end;       // 当前最终目标选择
} FirePermission_t;                  // 最后输出的开火指令

typedef struct
{
    bool valid;                  // 数据帧是否合法
    uint16_t target_change_time; // 目标改变的时间点
    uint8_t target_select;       // 选手端选择对应目标
    uint8_t range_select;        // 遥控器射程档
} FireSetupBaseline_t;           // 上一次已经成功下发到机构的设定基线，用于判断是否需要重新设定

/**
 * @brief 初始化 FireControl 模块
 *
 * 用法：
 * - 系统启动时调用一次
 * - 建议放在 TaskInitFunc() 或其他系统初始化入口中
 *
 * 作用：
 * - 清空裁判快照
 * - 清空遥控器意图
 * - 清空内部已应用设定基线
 * - 生成一份初始仲裁结果
 */
void FireControl_Init(void);

/**
 * @brief 更新裁判系统的飞镖舱门状态快照
 * @param dart_status 最新的 ext_dart_launch_status_t 数据指针
 * @return true-本次核心字段发生变化；false-无变化或传入为空
 *
 * 用法：
 * - 建议仅在 RefereeTaskFunc 中调用
 * - 每收到一帧新的 ID_dart_launch_status 后调用一次
 *
 * 该函数会自动：
 * - 刷新本地裁判快照
 * - 更新本地接收时间戳和序号
 * - 重新计算统一仲裁结果
 */
bool FireControl_UpdateRefereeGate(const ext_dart_launch_status_t *dart_status);

/**
 * @brief 更新比赛剩余时间镜像
 * @param remain_time ext_game_state_t.stage_remain_time
 * @return true-剩余时间发生变化；false-未变化
 *
 * 用法：
 * - 在 RefereeTaskFunc 收到 ID_game_state 后调用
 * - 主要用于补充仲裁结果中的上下文，不直接决定开火
 */
bool FireControl_UpdateRefereeRemainTime(uint16_t remain_time);

/**
 * @brief 更新飞镖发射剩余时间镜像
 * @param dart_info 最新 ext_dart_info_t 数据指针
 * @return true-飞镖剩余时间发生变化；false-未变化或传入为空
 *
 * 用法：
 * - 在 RefereeTaskFunc 收到 ID_dart_info 后调用
 * - 当前主要用于发射最终门禁：剩余时间小于 1s 时禁止发射
 */
bool FireControl_UpdateDartInfo(const ext_dart_info_t *dart_info);

/**
 * @brief 一次性更新完整遥控器业务意图
 * @param fire_enable 总发射使能
 * @param target_select 当前目标选择
 * @param range_select 当前射程/角度档位
 * @return true-意图有变化；false-三项均未变化
 *
 * 用法：
 * - 推荐在 ControlState 或遥控器解析任务中调用
 * - 当 fire_enable / target_select / range_select 任一变化时调用
 */
bool FireControl_UpdateIntent(bool fire_enable, uint8_t target_select, uint8_t range_select);

/**
 * @brief 单独更新总发射使能
 * @param fire_enable 总发射使能
 * @return true-值发生变化；false-未变化
 */
bool FireControl_SetFireEnable(bool fire_enable);

/**
 * @brief 单独更新目标选择
 * @param target_select 目标选择
 * @return true-值发生变化；false-未变化
 */
bool FireControl_SetTargetSelect(uint8_t target_select);

/**
 * @brief 单独更新射程/角度档位
 * @param range_select 射程/角度档位
 * @return true-值发生变化；false-未变化
 */
bool FireControl_SetRangeSelect(uint8_t range_select);

/**
 * @brief 标记“当前设定已经成功下发到机构”
 *
 * 用法：
 * - 在 StateSetTaskFunc 成功完成一次 2006/4310 设定后调用
 *
 * 作用：
 * - 记录当前裁判 target_change_time
 * - 记录当前 target_select / range_select
 * - 让 need_reconfigure 在条件未变化时恢复为 false
 */
void FireControl_MarkSetupApplied(void);

/**
 * @brief 获取当前裁判快照副本
 * @param snapshot 输出指针
 *
 * 用法：
 * - 需要调试或读取最新裁判门控上下文时调用
 * - 返回的是副本，不要尝试直接修改内部状态
 */
void FireControl_GetRefereeSnapshot(RefereeGateSnapshot_t *snapshot);

/**
 * @brief 获取当前遥控器业务意图副本
 * @param intent 输出指针
 *
 * 用法：
 * - 调试遥控器业务输入，或在其他任务中读取当前目标/射程配置时调用
 */
void FireControl_GetIntent(RC_FireIntent_t *intent);

/**
 * @brief 获取当前统一仲裁结果副本
 * @param permission 输出指针
 *
 * 用法：
 * - StateSetTaskFunc 读取 can_setup / need_reconfigure
 * - 发射前读取 can_shoot
 * - 储能/发射过程中读取 abort_current_shot
 */
void FireControl_GetPermission(FirePermission_t *permission);

/**
 * @brief 快捷接口：当前是否允许设定
 * @return true-允许；false-不允许
 */
bool FireControl_CanSetup(void);

/**
 * @brief 快捷接口：当前是否允许发射
 * @return true-允许；false-不允许
 */
bool FireControl_CanShoot(void);

/**
 * @brief 快捷接口：当前是否应中止发射流程
 * @return true-应中止；false-可继续
 */
bool FireControl_ShouldAbortShot(void);

/**
 * @brief 快捷读取：当前击打目标
 * @return FIRE_TARGET_BASE->true,击打基地
 * @note   默认返回的是 FIRE_TARGET_OUTPOST
 */
uint8_t FireControl_SelectTarget(void);

#endif
