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

#include <math.h>

// 通用系统参数
#define MOTOR_TIMEOUT_MS 5000  // 电机运动超时时间（毫秒）
#define MOTOR_DEAD_ZONE 1.0f   // 电机死区（用于位置判定）
#define TRIGGER_DEAD_ZONE 5.0f // 扳机电机死区
#define SERVO_MOVE_TIME_MS 310 // 舵机转动时间（毫秒）
#define POWER_ON_DELAY_MS 100  // 上电延迟时间（毫秒）
#define RELOAD_BUFFER_MS 1000  // 换弹缓冲时间（毫秒）

// 死区判定宏
#define IS_IN_DEADZONE(value, target, zone) \
    (((value) >= ((target) - (zone))) && ((value) <= ((target) + (zone))))

// 事件组位定义
#define EVENT_GIMBAL_READY (1 << 6)      // 0x40 - 云台就绪
#define EVENT_TRIGGER_READY (1 << 2)     // 0x04 - 扳机就绪
#define EVENT_TRIGGER_LOC_READY (1 << 3) // 0x08 - 扳机位置就绪
#define EVENT_ALL_READY 0x4C             // 0x4C - 全部就绪

// 换弹结构相关
#define ConveyorBeltLength 20673
#define SeperationAngle 375
#define FirstServoLoc -7547
#define SecondServoLoc -13200
#define ThirdServoLoc -18749
// 第三个舵机（距离7547）
// 第二个舵机（距离13200）<与第三个相距5653>
// 第一个舵机（距离18749）<与第二个相距5549>

// 储能电机相关（统一使用浮点数类型）
#define StoreLocPrimitive 0.0f // 两个电机的初始位置
#define LeftStoreLocBase 2.0f  // 基地位置力度-左边3519电机
#define RightStoreLocBase 3.0f // 基地位置力度-右边35199电机
#define LeftStoreOutpost 4.0f  // 前哨站力度-左边3519电机
#define RightStoreOutpost 5.0f // 前哨站力度-右边3519电机
#define LeftStoreTrigger 6.0f  // 靠近扳机位置-左边3519电机
#define RightStoreTrigger 7.0f // 靠近扳机位置-右边3519电机
#define LeftStoreBottom 0.0f  // 左侧电机下底（原0x10=16）
#define RightStoreBottom 17.0f // 右侧电机下底（原0x11=17）
#define LimitStore 910.0f      // 电机的位置限制

// 扳机射程相关
#define MG996R_store 2500   // 发射扳机待机状态。
#define MG996R_shoot 1000   // 发射扳机发射状态
#define MG996R_initial 2500 // 飞镖支架初始状态，向前摆，方便安装飞镖体
#define MG996R_extend 1750  // 飞镖支架伸出状态
#define MG996R_shrink 900   // 飞镖支架收回状态，向后摆，让SG90的线距离C板短点
#define MG995_initial 2500  // 飞镖支架初始状态，向前摆，方便安装飞镖体
#define MG995_extend 2000   // 飞镖支架伸出状态
#define MG995_shrink 1500   // 飞镖支架收回状态，向后摆，让SG90的线距离C板短点

// 云台转轴相关

// 遥控器相关
#define IA6B_CHANNEL 10                             // FS-I6通道数
#define IBUS_LENGTH 0x20                            // 协议头
#define IBUS_COMMAND 0x40                           // 协议头
#define IBUS_FRAME_LEN 32                           // IBUS固定帧长度（字节）
#define IBUS_DMA_BUFFER_LEN (IBUS_FRAME_LEN * 2)    // DMA接收缓冲区长度
#define IBUS_STREAM_BUFFER_LEN (IBUS_FRAME_LEN * 2) // IBUS流解析缓存长度

#endif
