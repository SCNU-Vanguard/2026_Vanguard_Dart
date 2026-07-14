#ifndef __SOFTWARE_WATCHDOG_H_ /* 按 __SOFTWARE_WATCHDOG_H_ 选择编译分支。 */
#define __SOFTWARE_WATCHDOG_H_ /* 定义 __SOFTWARE_WATCHDOG_H_。 */

#include <stdbool.h>
#include <stdint.h>

typedef enum /* 开始定义数据类型。 */
{
    SOFTWARE_WATCHDOG_REFEREE_GATE = 0, /* 裁判系统舱门数据。 */
    SOFTWARE_WATCHDOG_IBUS,             /* 遥控器 IBUS 数据。 */
    SOFTWARE_WATCHDOG_CAN_MOTOR_1,      /* 第 1 路 CAN 电机。 */
    SOFTWARE_WATCHDOG_CAN_MOTOR_2,      /* 第 2 路 CAN 电机。 */
    SOFTWARE_WATCHDOG_CAN_MOTOR_3,      /* 第 3 路 CAN 电机。 */
    SOFTWARE_WATCHDOG_CAN_MOTOR_4,      /* 第 4 路 CAN 电机。 */
    SOFTWARE_WATCHDOG_CAN_MOTOR_5,      /* 第 5 路 CAN 电机。 */
    SOFTWARE_WATCHDOG_COUNT             /* 看门狗通道总数。 */
} SoftwareWatchdogId_e; /* 结束 SoftwareWatchdogId_e 类型定义。 */

/* 超时回调在 TIM4 中断中执行，只允许置标志。 */
typedef void (*SoftwareWatchdogLostCallback_t)(SoftwareWatchdogId_e id); /* 声明 void 接口。 */

/* 清空全部软件看门狗。 */
void SoftwareWatchdog_Init(void); /* 声明 SoftwareWatchdog_Init 接口。 */
/* 注册连续监控通道，注册后立即开始计时。 */
bool SoftwareWatchdog_Register(SoftwareWatchdogId_e id, /* 传入下一项参数或数据。 */
                               uint32_t timeout_ms, /* 传入下一项参数或数据。 */
                               SoftwareWatchdogLostCallback_t lost_callback); /* 完成本行操作。 */
/* 连续监控通道收到新数据后喂狗。 */
void SoftwareWatchdog_Feed(SoftwareWatchdogId_e id); /* 声明 SoftwareWatchdog_Feed 接口。 */
/* 启动一次性回复等待，供 DM 电机使用。 */
bool SoftwareWatchdog_Arm(SoftwareWatchdogId_e id); /* 声明 SoftwareWatchdog_Arm 接口。 */
/* 停止一次性回复等待并清状态。 */
void SoftwareWatchdog_Disarm(SoftwareWatchdogId_e id); /* 声明 SoftwareWatchdog_Disarm 接口。 */
/* TIM4 每 1 ms 调用一次。 */
void SoftwareWatchdog_Tick1msFromISR(void); /* 声明 SoftwareWatchdog_Tick1msFromISR 接口。 */
/* 查询连续监控通道是否在线。 */
bool SoftwareWatchdog_IsHealthy(SoftwareWatchdogId_e id); /* 声明 SoftwareWatchdog_IsHealthy 接口。 */
/* 查询通道是否已经超时。 */
bool SoftwareWatchdog_IsTimedOut(SoftwareWatchdogId_e id); /* 声明 SoftwareWatchdog_IsTimedOut 接口。 */
/* 把电机序号转换为看门狗通道。 */
SoftwareWatchdogId_e SoftwareWatchdog_MotorId(uint8_t motor_index_one_based); /* 声明 SoftwareWatchdog_MotorId 接口。 */

#endif /* 结束条件编译。 */
