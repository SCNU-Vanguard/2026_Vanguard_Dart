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
#define MOTOR_DEADZONE_TIMEOUT_MS 1000U // 死区判定超时退出（毫秒）
#define TRIGGER_DEAD_ZONE 5.0f          // 扳机电机死区
#define SERVO_MOVE_TIME_MS 315          // 舵机转动时间（毫秒）
#define POWER_ON_DELAY_MS 100           // 上电延迟时间（毫秒）
#define RELOAD_BUFFER_MS 1000           // 换弹缓冲时间（毫秒）
#define LOAD_DEADZONE_TIMEOUT_MS 2000U  // 换弹电机缓冲时间

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

/* ==================== 业务层：机械结构参数 ==================== */

// 换弹结构相关
#define ConveyorBeltLength 20673
#define SeperationAngle 250
#define DART_SERVO_ID1 0x01
#define DART_SERVO_ID2 0x02
#define DART_SERVO_ID3 0x03
#define DART_SERVO1_RAW_ZERO 0
#define DART_SERVO1_RAW_RELEASE 250
#define DART_SERVO2_RAW_ZERO 210
#define DART_SERVO2_RAW_RELEASE 390
#define DART_SERVO3_RAW_ZERO 80
#define DART_SERVO3_RAW_RELEASE 250
#define PresetLoc (6427.0f)
#define FirstServoLoc (1688.0f)
#define SecondServoLoc (-4658.0f)
#define ThirdServoLoc (-11240.0f)
// 当前单独测试工程标定值：
// Preset = 6427，First = 1688，Second = -4658，Third = -11240

// 储能电机相关
#define LeftStoreLoad (-835.0f)   // 换弹位置
#define RightStoreLoad (835.0f)   // 换弹位置
#define LeftStoreBottom (-930.0f) // 左侧滑台底部
#define RightStoreBottom (930.0f) // 右侧滑台底部
#define LeftStoreTop (0.0f)       // 左侧滑台叉上滑顶部
#define RightStoreTop (0.0f)      // 右侧滑台叉上滑顶部
#define LimitStore 930.0f         // 电机的位置限制
#define StoreSpeed (12.0f)        // 储能电机移动速度

// 扳机射程相关
#define MG996R_store 2500   // 发射扳机待机状态。
#define MG996R_shoot 1000   // 发射扳机发射状态
#define MG996R_initial 2500 // 飞镖支架初始状态，向前摆，方便安装飞镖体
#define MG996R_extend 1750  // 飞镖支架伸出状态
#define MG996R_shrink 900   // 飞镖支架收回状态，向后摆，让SG90的线距离C板短点
#define MG995_initial 2500  // 飞镖支架初始状态，向前摆，方便安装飞镖体
#define MG995_extend 2000   // 飞镖支架伸出状态
#define MG995_shrink 1500   // 飞镖支架收回状态，向后摆，让SG90的线距离C板短点

#define LOAD_TASK_TRAP_VMAX_DEG_S (3500.0f)
#define LOAD_TASK_TRAP_AMAX_DEG_S2 (24000.0f)
#define LOAD_TASK_TRAP_JERK_FACTOR (1.2f)
#define LOAD_TASK_TRAP_DISABLE_JERK 0
#define LOAD_TASK_TRAP_RESET_ON_TARGET_CHANGE 1
#define LOAD_TASK_TRAP_BRAKE_GAIN (0.95f) /* (0,1]，越小越早减速，1.0=理论极限制动 */
#define LOAD_TASK_TRAP_ARRIVE_ZONE (1.0f) /* 到达死区(°)，误差<此值强制锁零速 */
#define LOAD_TASK_TRAP_DECEL_ZONE (40.0f) /* 线性减速区(°)，进入后速度线性压低 */

// 云台转轴相关
// 每发飞镖对应的 YAW(DM4310) 和 Trigger(RM2006) 预设位置
// 索引: [0]=Dart4(第1发), [1]=Dart3(第2发), [2]=Dart2(第3发), [3]=Dart1(第4发)
#define SETUP_YAW_DART4 0.0f
#define SETUP_YAW_DART3 0.0f
#define SETUP_YAW_DART2 0.0f
#define SETUP_YAW_DART1 0.0f

#define SETUP_TRIGGER_DART4 (-100.0f)
#define SETUP_TRIGGER_DART3 (100.0f)
#define SETUP_TRIGGER_DART2 (200.0f)
#define SETUP_TRIGGER_DART1 (0.0f)

// 调试开关：1=跳过“每发前4310/2006到位等待”，直接进入3519流程；0=按原流程等待
#define STORE_BYPASS_SETUP_WAIT 0U

#endif
