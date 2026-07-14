/**********************************************************
 * 文件名：config.h
 * 用途：存放一些关于整个飞镖系统工程参数的宏定义
 * 创建时间：12.15 2025
 * 创建人：邓金水
 * 数据全部经过测试得来
 * todo：等待准确的数据
 *********************************************************/
#ifndef __CONFIG_H_
#define __CONFIG_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <arm_math.h>

// 每发飞镖对应的 YAW 和 Trigger 微调偏置映射表
// 最终目标 = 当前目标类型基础值(Base/Outpost) + 当前发序偏置
// 索引: [0]=Dart4(第1发), [1]=Dart3(第2发), [2]=Dart2(第3发), [3]=Dart1(第4发)

#define TRIGGER_2006_SETUP_TIMEOUT_MS 2500U
#define STATE_SET_GRIPPER_SETUP_MAX_TIMEOUT_MS 12000U
#define STATE_SET_GRIPPER_OWNER_EXTRA_MS 500U
#define STATE_SET_SETUP_GATE_TIMEOUT_MS 1000U
#define STATE_SET_REQUEST_TIMEOUT_MS (TRIGGER_2006_SETUP_TIMEOUT_MS + STATE_SET_GRIPPER_SETUP_MAX_TIMEOUT_MS + STATE_SET_GRIPPER_OWNER_EXTRA_MS + STATE_SET_SETUP_GATE_TIMEOUT_MS)

/* ==================== BSP层：协议常量 ==================== */

// IBUS 协议参数
#define IA6B_CHANNEL 10                             // FS-I6通道数
#define IBUS_LENGTH 0x20                            // 协议头
#define IBUS_COMMAND 0x40                           // 协议头
#define IBUS_FRAME_LEN 32                           // IBUS固定帧长度（字节）
#define IBUS_DMA_BUFFER_LEN (IBUS_FRAME_LEN * 2)    // DMA接收缓冲区长度
#define IBUS_STREAM_BUFFER_LEN (IBUS_FRAME_LEN * 2) // IBUS流解析缓存长度

/* ==================== 控制层：通用系统参数 ==================== */

#define MOTOR_TIMEOUT_MS 5000           // 电机运动超时时间（毫秒）
#define MOTOR_DEAD_ZONE 3.0f            // 电机死区（用于位置判定）
#define MOTOR_DEADZONE_TIMEOUT_MS 3000U // 死区判定超时退出（毫秒）
#define TRIGGER_DEAD_ZONE 5.0f          // 扳机电机死区
#define SERVO_MOVE_TIME_MS 315          // 舵机转动时间（毫秒）
#define POWER_ON_DELAY_MS 100           // 上电延迟时间（毫秒）
#define RELOAD_BUFFER_MS 1000           // 换弹缓冲时间（毫秒）
#define LOAD_DEADZONE_TIMEOUT_MS 1500U  // 换弹电机缓冲时间

/* ==================== 控制层：遥控器配置 ==================== */

/*============================== 遥控器总开关控制 ==============================*/
/**
 * @brief 启用遥控器总开关功能
 * @note  设为1启用，设为0禁用（禁用时代码行为与原来一致）
 *
 * 功能说明：
 * - SWB=1（默认状态）：调试模式，只允许电机调试，不执行发射和换弹流程
 * - SWB=0（手动切换）：正常模式，执行正常的发射和换弹流程
 */
#define ENABLE_RC_MASTER_SWITCH 0

// 遥控器监控任务周期（毫秒）
#define RC_MONITOR_TASK_PERIOD_MS 10

/* ==================== 任务层：事件组位定义 ==================== */

#define EVENT_GIMBAL_READY (1 << 6)      // 0x40 - 云台就绪
#define EVENT_TRIGGER_READY (1 << 2)     // 0x04 - 扳机就绪
#define EVENT_TRIGGER_LOC_READY (1 << 3) // 0x08 - 扳机位置就绪
#define EVENT_ALL_READY 0x4C             // 0x4C - 全部就绪

// 裁判系统飞镖舱门状态的本地超时判定
#define REFEREE_GATE_FRESH_TIMEOUT_MS 2000U

/* ==================== 业务层：机械结构参数 ==================== */

// 换弹结构相关
#define ConveyorBeltLength 20673
#define SeperationAngle 500
#define PresetLoc (0.0f)
#define LOAD3508_DART_FIRST_STEP_DEG (1152.0f)
#define LOAD3508_DART_STEP_DEG (2304.0f)
#define LOAD3508_DART_STEP_DIR (-1.0f)
#define FirstServoLoc (PresetLoc + (LOAD3508_DART_STEP_DIR * LOAD3508_DART_FIRST_STEP_DEG))
#define SecondServoLoc (PresetLoc + (LOAD3508_DART_STEP_DIR * (LOAD3508_DART_FIRST_STEP_DEG + LOAD3508_DART_STEP_DEG)))
#define ThirdServoLoc (PresetLoc + (LOAD3508_DART_STEP_DIR * (LOAD3508_DART_FIRST_STEP_DEG + (2.0f * LOAD3508_DART_STEP_DEG))))
// LOAD3508 targets are computed from PresetLoc: first 1152, then +2304, +2304.
#define LOAD3508_STILL_OVERCURRENT_LIMIT_A (2.0f)
#define LOAD3508_STILL_OVERCURRENT_CLEAR_A (1.0f)
#define LOAD3508_STILL_OVERCURRENT_RETURN_MS 2500U
#define LOAD3508_STALL_OVERCURRENT_LIMIT_A (LOAD3508_STILL_OVERCURRENT_LIMIT_A)
#define LOAD3508_STALL_OVERCURRENT_CLEAR_A (LOAD3508_STILL_OVERCURRENT_CLEAR_A)
#define LOAD3508_STALL_OVERCURRENT_RETURN_MS 2500U
#define LOAD3508_STALL_SPEED_RPM (8.0f)
#define LOAD3508_STALL_POS_DELTA_DEG (1.0f)
#define LOAD3508_STALL_POS_SAMPLE_MS 100U
#define LOAD3508_OVERCURRENT_TARGET_BLANK_MS 800U
#define LOAD3508_OVERCURRENT_FILTER_ALPHA (0.08f)
// TODO: LOAD3508_OVERCURRENT_TARGET_BLANK_MS 当前未被 RM_Motor_ApplyOutputLimit 使用，
//       GRIPPER 过流保护走的是简化版（持续过流→计时→trip）。若需要“目标刚切换时给空白时间”，
//       需在 RM_Motor_ApplyOutputLimit 里补相应逻辑并读取本宏。

// 换弹夹爪 M3508 的相对零点位置限位。
#define LOAD3508_MIN (-10000.0f)
#define LOAD3508_MAX (0.0f)

// 储能 M3508 在“已接近目标位置”时的异常电流保护参数。
#define STORE3508_STILL_OVERCURRENT_LIMIT_A (2.5f)
#define STORE3508_STILL_OVERCURRENT_CLEAR_A (1.2f)
#define STORE3508_STILL_OVERCURRENT_CONFIRM_MS 1500U

// 储能 M3508 在“运动途中疑似堵转”时的异常电流保护参数。
#define STORE3508_STALL_OVERCURRENT_LIMIT_A (3.0f)
#define STORE3508_STALL_OVERCURRENT_CLEAR_A (1.5f)
#define STORE3508_STALL_CONFIRM_MS 1500U
#define STORE3508_STALL_SPEED_RPM (8.0f)
#define STORE3508_STALL_POS_DELTA_DEG (1.0f)
#define STORE3508_STALL_POS_SAMPLE_MS 100U
#define STORE3508_OVERCURRENT_TARGET_BLANK_MS 400U
#define STORE3508_OVERCURRENT_FILTER_ALPHA (0.08f)
#define STORE3508_TEMP_LIMIT_C (60.0f)
#define STORE3508_TRAP_VMAX_DEG_S (1800.0f)
#define STORE3508_TRAP_AMAX_DEG_S2 (9000.0f)
#define STORE3508_TRAP_JERK_FACTOR (1.0f)
#define STORE3508_TRAP_DISABLE_JERK 0
#define STORE3508_TRAP_BRAKE_GAIN (0.95f)
#define STORE3508_TRAP_ARRIVE_ZONE (1.0f)
#define STORE3508_TRAP_DECEL_ZONE (30.0f)

/* 双侧蓄力 3508 位置同步 PID 参数：ia6b_task 生成 base 目标后，
 * 由 RM_Motor_UpdateStoreSync 把 (left_pos - right_pos) 推向 0。*/
#define STORE3508_SYNC_PID_KP (2.0f)
#define STORE3508_SYNC_PID_KI (0.0f)
#define STORE3508_SYNC_PID_KD (0.10f)
#define STORE3508_SYNC_PID_KF (0.00f)
#define STORE3508_SYNC_PID_MAX_OUT (200.0f)
#define STORE3508_SYNC_PID_MIN_OUT (0.0f)
#define STORE3508_SYNC_PID_MAX_IOUT (20.0f)

// 储能电机相关
#define LeftStoreLoad (30245.0f)    // 换弹位置
#define RightStoreLoad (30245.0f)   // 换弹位置
#define LeftStoreBottom (48500.0f)  // 左侧滑台底部
#define RightStoreBottom (48500.0f) // 右侧滑台底部
#define LeftStoreTop (0.0f)         // 左侧滑台叉上滑顶部
#define RightStoreTop (0.0f)        // 右侧滑台叉上滑顶部
#define LeftSafe (1500.0f)          // 左侧滑台叉上滑顶部
#define RightSafe (1500.0f)         // 右侧滑台叉上滑顶部
#define LimitStore 49000.0f         // 电机的位置限制
#define StoreSpeed (6.0f)           // 储能电机移动速度

// 扳机射程相关
#define MG996R_store 2500 // 发射扳机待机状态
#define MG996R_shoot 1200 // 发射扳机发射状态

// 换弹电机斜坡
#define LOAD_TASK_TRAP_VMAX_DEG_S 10.0f  // (3500.0f)
#define LOAD_TASK_TRAP_AMAX_DEG_S2 50.0f // (24000.0f)
#define LOAD_TASK_TRAP_JERK_FACTOR (1.2f)
#define LOAD_TASK_TRAP_DISABLE_JERK 0
#define LOAD_TASK_TRAP_RESET_ON_TARGET_CHANGE 1
#define LOAD_TASK_TRAP_BRAKE_GAIN (0.95f) /* (0,1]，越小越早减速，1.0=理论极限制动 */
#define LOAD_TASK_TRAP_ARRIVE_ZONE (1.0f) /* 到达死区(°)，误差<此值强制锁零速 */
#define LOAD_TASK_TRAP_DECEL_ZONE (40.0f) /* 线性减速区(°)，进入后速度线性压低 */

// 云台转轴相关
// 每发飞镖对应的 YAW(RM6020) 和 Trigger(RM2006) 预设位置
// 索引: [0]=Dart4(第1发), [1]=Dart3(第2发), [2]=Dart2(第3发), [3]=Dart1(第4发)
#define SETUP_YAW_DART4 0.0f
#define SETUP_YAW_DART3 0.0f
#define SETUP_YAW_DART2 0.0f
#define SETUP_YAW_DART1 0.0f

#define SETUP_TRIGGER_DART4 (0.0f) // 3
#define SETUP_TRIGGER_DART3 (0.0f) // 2
#define SETUP_TRIGGER_DART2 (0.0f) // 1
#define SETUP_TRIGGER_DART1 (0.0f) // 0

// 6020 Yaw 到位等待开关：1=在任务层等待进入±deadzone；0=下发目标后直接继续流程
#define ENABLE_6020_YAW_WAIT 1U
#define YAW_6020_DEAD_ZONE 5.0f

// 调试开关：1=跳过“每发前6020/2006到位等待”，直接进入3519流程；0=按原流程等待
#define STORE_BYPASS_SETUP_WAIT 0U

// 裁判系统飞镖发射站闸门相关参数
#define REFEREE_DART_OPEN 0
#define REFEREE_DART_CLOSE 1
#define REFEREE_DART_MID 2
#define REFEREE_DART_FIRE_MIN_REMAIN_TIME_S 2U

// 前哨站和基地对应的Yaw角度
#define OutpostYawAngle 10.0f
#define BaseYawAngle -10.0f

// 前哨站和基地对应的扳机位置
#define OutpostTrigger 10.0f
#define BaseTrigger -10.0f

static const float g_SetupYaw[4] = {
    SETUP_YAW_DART4, SETUP_YAW_DART3, SETUP_YAW_DART2, SETUP_YAW_DART1};
static const float g_SetupTrigger[4] = {
    SETUP_TRIGGER_DART4, SETUP_TRIGGER_DART3, SETUP_TRIGGER_DART2, SETUP_TRIGGER_DART1};

#define STORE3508_LEFT_POS_MIN_DEG (0.0f)
// #define STORE3508_LEFT_POS_MAX_DEG (10000.0f)
// #define STORE3508_RIGHT_POS_MAX_DEG (10000.0f)
#define STORE3508_RIGHT_POS_MIN_DEG (0.0f)
#define STORE3508_LEFT_POS_MAX_DEG (LimitStore)
#define STORE3508_RIGHT_POS_MAX_DEG (LimitStore)

#endif
