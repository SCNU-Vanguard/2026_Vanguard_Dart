#ifndef __MOTOR_CONTROL_TASK_H_ /* 按 __MOTOR_CONTROL_TASK_H_ 选择编译分支。 */
#define __MOTOR_CONTROL_TASK_H_ /* 定义 __MOTOR_CONTROL_TASK_H_。 */

#include "bsp_can.h"
#include "CanMotor.h"
#include "RM_Motor.h"
#include "motor_algrothim.h"
#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------
 * 通用电机控制 API
 * ------------------------------------------------------------------
 * 统一针对 can_motor_cfg 编号的电机，屏蔽 3508/2006/6020/储能
 * 4 套重复 API。策略（S 型规划、同步 PID、过流保护）通过注册时
 * 写入 MotorTypeDef 的字段决定，业务层只看这 6 个函数即可。
 * ------------------------------------------------------------------ */
void MotorControl_Init(void); /* 声明 MotorControl_Init 接口。 */
void Motor_SetTarget(can_motor_cfg motor_cfg, float target_deg); /* 声明 Motor_SetTarget 接口。 */
float Motor_GetTarget(can_motor_cfg motor_cfg); /* 声明 Motor_GetTarget 接口。 */
void Motor_EnableControl(can_motor_cfg motor_cfg, bool enable); /* 声明 Motor_EnableControl 接口。 */
void Motor_SetUseSCurve(can_motor_cfg motor_cfg, bool enable); /* 声明 Motor_SetUseSCurve 接口。 */
bool Motor_IsAnyStoreProtected(void); /* 左右两侧储能任一保护 */

/* 单一电机控制任务，塌合原来 3508/2006/StoreSync/6020 四个 */
void MotorCtrlTask(void *argument); /* 声明 MotorCtrlTask 接口。 */

/* ------------------------------------------------------------------
 * LoadMotor_* 业务层封装（夹爪 M3508 专用）
 * ------------------------------------------------------------------
 * 多 owner（StateSet/Load/Home）抢占同一电机，附带优先级 +
 * stream-buffer 命令流 + 命令签名校验。内部最终转调 Motor_SetTarget。
 * ------------------------------------------------------------------ */
typedef enum /* 开始定义数据类型。 */
{
    LOAD_MOTOR_OWNER_NONE = 0, /* 定义 LOAD_MOTOR_OWNER_NONE 枚举项。 */
    LOAD_MOTOR_OWNER_STATE_SET, /* 定义 LOAD_MOTOR_OWNER_STATE_SET 枚举项。 */
    LOAD_MOTOR_OWNER_LOAD, /* 定义 LOAD_MOTOR_OWNER_LOAD 枚举项。 */
    LOAD_MOTOR_OWNER_HOME, /* 定义 LOAD_MOTOR_OWNER_HOME 枚举项。 */
} LoadMotorOwner_e; /* 结束 LoadMotorOwner_e 类型定义。 */

/* ------------------------------------------------------------------
 * 每个电机的运行时状态表
 * ------------------------------------------------------------------ */
typedef struct /* 开始定义数据类型。 */
{
    /* 目标/使能（规划层状态；过流/堵转/到位/闩锁归 angle_motor 运行时所有） */
    float target; /* 保存 target。 */
    bool ctrl_enabled; /* 保存 ctrl_enabled。 */
    bool use_scurve; /* 保存 use_scurve。 */
    uint8_t target_set_while_disabled; /* 保存 target_set_while_disabled。 */
    uint32_t last_target_change_ms; /* 保存 last_target_change_ms。 */

    /* S 型规划 */
    MotorTrapPosProfile_t trap_profile; /* 保存 trap_profile。 */
    uint32_t trap_cnt_last; /* 保存 trap_cnt_last。 */
    bool trap_initialized; // S 型/梯形位置轨迹规划器是否已经初始化
} MotorRuntimeState_t; /* 结束 MotorRuntimeState_t 类型定义。 */

/// @brief LoadMotor的Stream包
typedef struct /* 开始定义数据类型。 */
{
    uint32_t magic; /* 保存 magic。 */
    uint32_t signature; /* 保存 signature。 */
    uint32_t seq; /* 保存 seq。 */
    uint32_t timestamp_ms; /* 保存 timestamp_ms。 */
    uint32_t max_age_ms; /* 保存 max_age_ms。 */
    uint32_t owner_hold_ms; /* 保存 owner_hold_ms。 */
    uint8_t owner; /* 保存 owner。 */
    uint8_t priority; /* 保存 priority。 */
    uint16_t reserved; /* 保存 reserved。 */
    float target_pos_deg; /* 保存 target_pos_deg。 */
} LoadMotorCommand_t; /* 结束 LoadMotorCommand_t 类型定义。 */

#define LOAD_MOTOR_PRIORITY_STATE_SET 20U /* 定义 LOAD_MOTOR_PRIORITY_STATE_SET。 */
#define LOAD_MOTOR_PRIORITY_LOAD 80U /* 定义 LOAD_MOTOR_PRIORITY_LOAD。 */
#define LOAD_MOTOR_PRIORITY_HOME 90U /* 定义 LOAD_MOTOR_PRIORITY_HOME。 */

bool LoadMotor_SubmitTarget(LoadMotorOwner_e owner, /* 传入下一项参数或数据。 */
                            uint8_t priority, /* 传入下一项参数或数据。 */
                            float target_pos_deg, /* 传入下一项参数或数据。 */
                            uint32_t owner_hold_ms); /* 完成本行操作。 */
void LoadMotor_ReleaseOwner(LoadMotorOwner_e owner); /* 声明 LoadMotor_ReleaseOwner 接口。 */

#endif /* 结束条件编译。 */
