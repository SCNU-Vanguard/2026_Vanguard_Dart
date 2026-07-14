/**********************************************************
 * 文件名：angle_motor.h
 * 用途：位置控制类电机（2006 / 三个 3508 / 储能位 3519）的通用分层框架
 * 创建时间：2026
 *
 * 设计原则（零专用函数、数据驱动）：
 *   所有位置控制类电机走同一条 tick 路径，电机差异全部落在画像表
 *   s_angle_profiles[] 里。加/换电机 = 填一行画像，不新增函数。
 *
 * 分层数据流（每电机每 tick）：
 *   Planner    ref  = 规划(trap/S曲线 或 直通)         —— 由 MotorControlTask 提供
 *   Controller raw  = backend->compute(cfg, ref)        —— 纯计算，无副作用
 *   Judge      dec  = AngleMotor_Judge(cfg, ref)        —— 纯判断，只读传感器，不碰 CAN
 *   Apply      out  = 按 dec 处理 raw + 方向限位 + 硬限幅
 *   Output          = backend->send(cfg, out)
 *
 * 状态判断分级：
 *   NORMAL   正常，raw 原样透传
 *   INFO     到位(无/轻载)：out=0 + 清积分 + 置到位标志   —— 夹爪/扳机
 *   WARNING  到位(带载/拉簧)：置到位标志，out=raw 继续闭环 —— 储能，顶住拉簧不撒手
 *   ERROR    过流/堵转/超温：out=0 + 闩锁(需显式 Clear)    —— 全电机
 *********************************************************/
#ifndef __ANGLE_MOTOR_H_ /* 按 __ANGLE_MOTOR_H_ 选择编译分支。 */
#define __ANGLE_MOTOR_H_ /* 定义 __ANGLE_MOTOR_H_。 */

#include "CanMotor.h"
#include <stdbool.h>
#include <stdint.h>

/*============================== 状态分级 ==============================*/

typedef enum /* 开始定义数据类型。 */
{
    ANGLE_MOTOR_NORMAL = 0, // 正常：控制器输出原样透传
    ANGLE_MOTOR_INFO,       // 到位(轻载)：输出置 0 + 清积分 + 置到位
    ANGLE_MOTOR_WARNING,    // 到位(带载)：置到位，输出仍走闭环（不撒手）
    ANGLE_MOTOR_ERROR,      // 故障：过流/堵转/超温，输出置 0 并闩锁
} AngleMotorStateLevel_e; /* 结束 AngleMotorStateLevel_e 类型定义。 */

/*============================== 到位策略 ==============================*/

typedef enum /* 开始定义数据类型。 */
{
    ANGLE_MOTOR_ARRIVAL_INFO = 0, // 到位停输出（轻载）
    ANGLE_MOTOR_ARRIVAL_WARNING,  // 到位保持输出（带载/拉簧）
} AngleMotorArrivalPolicy_e; /* 结束 AngleMotorArrivalPolicy_e 类型定义。 */

/*============================== 过流/堵转参数 ==============================*/

/* 与原 MotorControlTask.c 的 MotorOverCurrentParams_t 字段一致，
 * 移到公共头，画像表和判断器共用。 */
typedef struct /* 开始定义数据类型。 */
{
    float still_limit_a;          // 到位/静止时过流触发阈值
    float still_clear_a;          // 到位/静止时过流清除阈值
    uint32_t still_confirm_ms;    // 静止过流确认时长
    float stall_limit_a;          // 运动中堵转过流触发阈值
    float stall_clear_a;          // 堵转过流清除阈值
    uint32_t stall_confirm_ms;    // 堵转确认时长
    float stall_speed_rpm;        // 堵转速度判据
    float stall_pos_delta_deg;    // 堵转位移判据
    uint32_t stall_pos_sample_ms; // 堵转位移采样间隔
    uint32_t target_blank_ms;     // 目标切换空白期（防起步电流误判）
} AngleMotorOcParams_t; /* 结束 AngleMotorOcParams_t 类型定义。 */

/*============================== 后端插件 ==============================*/

/* 后端把“如何算输出、如何发送”从统一 tick 里解耦：
 *   - RM 串级：compute = 串级 PID → 电流(int16 装进 float)，send = RmMotorSendCfg
 *   - DM 位置速度：compute = 直发位置(电机内部闭环)，send = DmMotorSendCfg(LOCATION_SPEED)
 * 加新电机类型只需再实现一个后端，不动 tick 主体。 */
typedef struct /* 开始定义数据类型。 */
{
    /// @brief 控制器：由参考位置算出“原始输出”，纯计算不下发
    /// @param cfg 电机别名
    /// @param ref_pos_deg 参考位置(相对零点，°)
    /// @return 原始输出（RM:电流值; DM:约定为参考位置，send 时直发）
    float (*compute)(can_motor_cfg cfg, float ref_pos_deg); /* 调用 float。 */

    /// @brief 输出：把限幅决策后的值下发到 CAN
    /// @param cfg 电机别名
    /// @param out 经状态判断+限幅后的输出值
    void (*send)(can_motor_cfg cfg, float out); /* 调用 void。 */

    /// @brief 该后端是否对输出做 ±current_hard_limit 硬限幅（DM 位置速度不适用）
    uint8_t use_current_limit; /* 保存 use_current_limit。 */
} AngleMotorBackend_t; /* 结束 AngleMotorBackend_t 类型定义。 */

/*============================== 电机画像 ==============================*/

typedef struct /* 开始定义数据类型。 */
{
    const AngleMotorBackend_t *backend; // RM 串级 / DM 位置速度
    AngleMotorOcParams_t oc;            // 过流/堵转参数（全 0 表示不判过流）
    uint8_t enable_oc;                  // 是否启用过流判断
    float temp_limit_c;                 // 温度上限(℃)，<=0 不判
    float pos_tolerance_deg;            // 到位死区(°)
    AngleMotorArrivalPolicy_e arrival;  // 到位策略：INFO / WARNING
    int16_t current_hard_limit;         // 电流硬限幅绝对值(RM 电机用)
} AngleMotorProfile_t; /* 结束 AngleMotorProfile_t 类型定义。 */

/*============================== 判断结果 ==============================*/

typedef struct /* 开始定义数据类型。 */
{
    AngleMotorStateLevel_e level; // 本 tick 判定的分级
    bool arrived;                 // 是否已到位
} AngleMotorDecision_t; /* 结束 AngleMotorDecision_t 类型定义。 */

/*============================== 运行时状态 ==============================*/

/* angle_motor 自有的判断/保护运行时（与 MotorControlTask 的规划/使能状态分离）。 */
typedef struct /* 开始定义数据类型。 */
{
    /* 过流/堵转 */
    /* 静止/接近目标时，持续过流检测的起始时刻，单位 ms。
     * 为 0 表示当前没有进行静止过流计时。
     * 电流超过 still_limit_a 时开始计时；
     * 电流下降到 still_clear_a 以下时清零。 */
    uint32_t over_current_start_ms; /* 保存 over_current_start_ms。 */

    /* 运动途中疑似堵转且过流的起始时刻，单位 ms。
     * 为 0 表示当前没有进行堵转过流计时。
     * 只有“大电流 + 低转速/小位移”同时满足时才持续计时。 */
    uint32_t stall_over_current_start_ms; /* 保存 stall_over_current_start_ms。 */

    /* 上一次堵转位置采样的系统时刻，单位 ms。
     * 与 stall_sample_pos_deg 配合，在指定采样周期内判断
     * 电机位置变化量是否过小。 */
    uint32_t stall_sample_ms; /* 保存 stall_sample_ms。 */

    /* 上一次堵转检测采样时的电机累计角度，单位 °。
     * 当前角度与该值之差小于 stall_pos_delta_deg 时，
     * 可认为电机在采样周期内基本没有移动。 */
    float stall_sample_pos_deg; /* 保存 stall_sample_pos_deg。 */

    /* 状态 */

    /* 故障闩锁：
     * 0 = 无闩锁故障；
     * 1 = 已发生过流、堵转、超温或底层电机保护。
     * 闩锁后电机输出保持为 0，需要显式调用
     * AngleMotor_ClearFault() 才能恢复。 */
    uint8_t fault_latched; /* 保存 fault_latched。 */

    /* 电机到位标志：
     * 0 = 尚未进入目标位置容差范围；
     * 1 = 已进入目标位置容差范围。
     * 设置新目标时由 AngleMotor_NotifyTargetChanged() 清零。 */
    uint8_t arrived; /* 保存 arrived。 */
} AngleMotorRuntime_t; /* 结束 AngleMotorRuntime_t 类型定义。 */

/*============================== 对外后端实例 ==============================*/

extern const AngleMotorBackend_t kAngleBackendRmCascade; // 3508 / 2006
extern const AngleMotorBackend_t kAngleBackendDmPosVel;  // 储能位 3519

/*============================== API ==============================*/

/// @brief 该电机是否由 angle_motor 框架管理（画像表里有 backend）
/// @param cfg 电机别名
/// @return true=纳入框架
bool AngleMotor_IsManaged(can_motor_cfg cfg); /* 声明 AngleMotor_IsManaged 接口。 */

/// @brief 获取电机画像（只读）
/// @param cfg 电机别名
/// @return 画像指针，未纳入返回 NULL
const AngleMotorProfile_t *AngleMotor_GetProfile(can_motor_cfg cfg); /* 声明 AngleMotor_GetProfile 接口。 */

/// @brief 状态判断器：纯函数，只读传感器 + 参考，不碰 CAN
/// @param cfg 电机别名
/// @param now_ms 当前时刻(ms)
/// @param ref_pos_deg 本 tick 参考位置(相对零点，°)
/// @param target_pos_deg 最终目标位置(相对零点，°，用于到位判定)
/// @return 分级 + 到位结果
AngleMotorDecision_t AngleMotor_Judge(can_motor_cfg cfg, uint32_t now_ms, /* 传入下一项参数或数据。 */
                                      float ref_pos_deg, float target_pos_deg); /* 完成本行操作。 */

/// @brief 限幅器：方向限位 + ±current_hard_limit 硬限幅 + 取整
/// @param cfg 电机别名
/// @param raw 控制器原始输出
/// @return 限幅后的输出（float，交给后端 send）
float AngleMotor_LimitOutput(can_motor_cfg cfg, float raw); /* 声明 AngleMotor_LimitOutput 接口。 */

/// @brief 完整单电机 tick（供 MotorControlTask 调用）：Judge→Apply→输出
///        规划(ref) 由调用方算好传入，本函数完成控制→判断→限幅→发送。
/// @param cfg 电机别名
/// @param ref_pos_deg 规划器输出的参考位置(相对零点，°)
/// @param target_pos_deg 最终目标(相对零点，°，到位判定用)
/// @return 本 tick 判定分级（供调用方联动，如失能）
AngleMotorStateLevel_e AngleMotor_Drive(can_motor_cfg cfg, /* 传入下一项参数或数据。 */
                                        float ref_pos_deg, /* 传入下一项参数或数据。 */
                                        float target_pos_deg); /* 完成本行操作。 */

/// @brief 查询是否处于故障闩锁
bool AngleMotor_IsFaulted(can_motor_cfg cfg); /* 声明 AngleMotor_IsFaulted 接口。 */

/// @brief 清除故障闩锁 + 复位过流计时（对应业务层 ClearProtection）
void AngleMotor_ClearFault(can_motor_cfg cfg); /* 声明 AngleMotor_ClearFault 接口。 */

/// @brief 目标切换时复位保护计时（供 Motor_SetTarget 调用，防跨目标误判）
void AngleMotor_NotifyTargetChanged(can_motor_cfg cfg); /* 声明 AngleMotor_NotifyTargetChanged 接口。 */

/// @brief 复位单个电机的 angle_motor 运行时（使能/初始化时用）
void AngleMotor_ResetRuntime(can_motor_cfg cfg); /* 声明 AngleMotor_ResetRuntime 接口。 */

/*============================== 左右蓄力位置同步 PID ==============================*/
/* 通用命名封装：左右蓄力电机（3508 或 DM 3519 备用）的位置同步。
 * correction 在“参考位置”层注入，与后端解耦——RM 走串级、DM 透传均适用。
 * 内部实现沿用 RM_Motor.c 里已标定好的同步 PID。 */

/// @brief 初始化左右蓄力同步 PID（由 MotorRegister 调用）
void AngleMotor_InitStoreSyncPid(float kp, float ki, float kd, float kf, /* 传入下一项参数或数据。 */
                                 float max_out, float min_out, float max_iout); /* 完成本行操作。 */

/// @brief 计算同步校正量：把 (left_pos - right_pos) 推向 0
/// @param left_pos_deg 左侧蓄力电机当前位置(°)
/// @param right_pos_deg 右侧蓄力电机当前位置(°)
/// @return 校正量(°)，上层按 Left+corr / Right-corr 分发
float AngleMotor_UpdateStoreSync(float left_pos_deg, float right_pos_deg); /* 声明 AngleMotor_UpdateStoreSync 接口。 */

#endif /* __ANGLE_MOTOR_H_ */
