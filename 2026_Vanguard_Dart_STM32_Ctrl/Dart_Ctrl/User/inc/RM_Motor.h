#ifndef __RM_MOTOR_H_ /* 按 __RM_MOTOR_H_ 选择编译分支。 */
#define __RM_MOTOR_H_ /* 定义 __RM_MOTOR_H_。 */

#include "main.h"
#include "bsp_can.h"
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*============================== RM电机CAN地址定义 ==============================*/

#define g_RM_MOTOR_BIAS_ADDR 0x200 /* 定义 g_RM_MOTOR_BIAS_ADDR。 */
#define g_RM_MOTOR_BIAS_ADDR_3508 0x200 /* 定义 g_RM_MOTOR_BIAS_ADDR_3508。 */
#define g_RM_MOTOR_BIAS_ADDR_2006 0x1FF /* 定义 g_RM_MOTOR_BIAS_ADDR_2006。 */
#define g_RM_MOTOR_BIAS_ADDR_6020 0x2FE /* 定义 g_RM_MOTOR_BIAS_ADDR_6020。 */

#define g_RM_MOTOR_NUM 5 /* 定义 g_RM_MOTOR_NUM。 */
#define g_RM_M3508_NUM 3 /* 定义 g_RM_M3508_NUM。 */
#define g_RM_M2006_NUM 1 /* 定义 g_RM_M2006_NUM。 */
#define g_RM_GM6020_NUM 4 /* 定义 g_RM_GM6020_NUM。 */

#define RM_TestUse 0U /* 定义 RM_TestUse。 */

/*============================== RM电机参数常量 ==============================*/

#define RM_M2006_GEAR_RATIO 36.0f /* 定义 RM_M2006_GEAR_RATIO。 */
#define RM_M2006_MAX_CURRENT 10000 /* 定义 RM_M2006_MAX_CURRENT。 */
#define RM_M2006_CURRENT_RATIO 0.18f /* 定义 RM_M2006_CURRENT_RATIO。 */

#define RM_M3508_GEAR_RATIO 19.2f /* 定义 RM_M3508_GEAR_RATIO。 */
#define RM_M3508_MAX_CURRENT 16384 /* 定义 RM_M3508_MAX_CURRENT。 */
#define RM_M3508_CURRENT_RATIO 819.20f //(16384.0f / 20.0f)
#define RM_3508_CLEAR_ANGLE_I_ON_DIR_CHANGE 1U /* 定义 RM_3508_CLEAR_ANGLE_I_ON_DIR_CHANGE。 */

#define RM_GM6020_GEAR_RATIO 1.0f /* 定义 RM_GM6020_GEAR_RATIO。 */
#define RM_GM6020_MAX_CURRENT 16384 /* 定义 RM_GM6020_MAX_CURRENT。 */
#define RM_GM6020_CURRENT_RATIO 5461.330f // (16384.0f / 3.0f)

/*============================== 前向声明 ==============================*/

#pragma pack(push, 1) /* 配置编译选项 pack(push, 1)。 */
struct _MotorTypeDef; /* 保存 _MotorTypeDef。 */

/*============================== RM电机类定义（面向对象） ==============================*/

typedef struct _RM_MotorClass /* 开始定义数据类型。 */
{
    const char *name; /* 保存 name。 */
    uint8_t model; /* 保存 model。 */

    float gear_ratio; /* 保存 gear_ratio。 */
    int16_t max_current; /* 保存 max_current。 */
    float current_ratio; /* 保存 current_ratio。 */
    uint16_t tx_base_addr; /* 保存 tx_base_addr。 */
    uint8_t id_min; /* 保存 id_min。 */
    uint8_t id_max; /* 保存 id_max。 */

    void (*init)(struct _MotorTypeDef *self, uint8_t id); /* 调用 void。 */
    void (*calculate)(struct _MotorTypeDef *self); /* 调用 void。 */
} RM_MotorClass_t; /* 结束 RM_MotorClass_t 类型定义。 */
#pragma pack(pop) /* 配置编译选项 pack(pop)。 */

extern const RM_MotorClass_t RM_M2006_Class; /* 声明外部变量 RM_M2006_Class。 */
extern const RM_MotorClass_t RM_M3508_Class; /* 声明外部变量 RM_M3508_Class。 */
extern const RM_MotorClass_t RM_GM6020_Class; /* 声明外部变量 RM_GM6020_Class。 */

/*============================== RM电机配置结构 ==============================*/

#pragma pack(push, 1) /* 配置编译选项 pack(push, 1)。 */
typedef struct /* 开始定义数据类型。 */
{
    float direction_bias; /* 保存 direction_bias。 */
    float position_min; /* 保存 position_min。 */
    float position_max; /* 保存 position_max。 */
    float position_tolerance; /* 保存 position_tolerance。 */
    uint8_t reverse; /* 保存 reverse。 */
} RM_MotorConfig_t; /* 结束 RM_MotorConfig_t 类型定义。 */
#pragma pack(pop) /* 配置编译选项 pack(pop)。 */

#include "CanMotor.h"

/*============================== 面向对象接口 ==============================*/

void RM_Motor_Create(MotorTypeDef *motor, const RM_MotorClass_t *motor_class, uint8_t id); /* 声明 RM_Motor_Create 接口。 */
void RM_Motor_Calculate(MotorTypeDef *motor); /* 声明 RM_Motor_Calculate 接口。 */

/*============================== 初始化函数 ==============================*/

void RM_M2006_Create(MotorTypeDef *motor, uint8_t id); /* 声明 RM_M2006_Create 接口。 */
void RM_M3508_Create(MotorTypeDef *motor, uint8_t id); /* 声明 RM_M3508_Create 接口。 */
void RM_GM6020_Create(MotorTypeDef *motor, uint8_t id); /* 声明 RM_GM6020_Create 接口。 */

/*============================== 配置函数 ==============================*/

void RM_Motor_SetConfig(MotorTypeDef *motor, const RM_MotorConfig_t *config); /* 声明 RM_Motor_SetConfig 接口。 */
void RM_Motor_SetCascadePID(MotorTypeDef *motor, /* 传入下一项参数或数据。 */
                            float outer_p, float outer_i, float outer_d, float outer_f, /* 传入下一项参数或数据。 */
                            float inner_p, float inner_i, float inner_d, float inner_f, /* 传入下一项参数或数据。 */
                            float outer_max_out, float outer_min_out, float outer_max_iout, /* 传入下一项参数或数据。 */
                            float inner_max_out, float inner_min_out, float inner_max_iout); /* 完成本行操作。 */
void RM_Motor_SetSpeedPID(MotorTypeDef *motor, PID_MODE_e mode, /* 传入下一项参数或数据。 */
                          float p, float i, float d, float f, /* 传入下一项参数或数据。 */
                          float max_out, float min_out, float max_iout); /* 完成本行操作。 */

/*============================== 兼容旧接口 ==============================*/

void RM_MOTOR_CALCU(MotorTypeDef *motor); /* 声明 RM_MOTOR_CALCU 接口。 */
void RM_Motor_Reset_Zero(MotorTypeDef *motor); /* 声明 RM_Motor_Reset_Zero 接口。 */
void RM_Motor_Reset_All(void); /* 声明 RM_Motor_Reset_All 接口。 */
void RmMotorSendCfg(can_motor_cfg motor_cfg, int16_t TargetCurrent); /* 声明 RmMotorSendCfg 接口。 */
void RmMotorPID_Calc(can_motor_cfg motor_cfg, float target); /* 声明 RmMotorPID_Calc 接口。 */
void RmMotorSpeedPID_Calc(can_motor_cfg motor_cfg, float target_speed_rpm); /* 声明 RmMotorSpeedPID_Calc 接口。 */

/* angle_motor 纯控制器后端：由参考位置算电流原始输出，无副作用。 */
float Rm_ComputeCascade(can_motor_cfg cfg, float ref_pos_deg); /* 声明 Rm_ComputeCascade 接口。 */

/*============================== 双侧蓄力 3508 位置同步 PID ==============================*/

/* 初始化左右蓄力 3508 的位置同步 PID（由 CanMotor.c:MotorRegister 调用）。 */
void RM_Motor_InitStoreSyncPid(float kp, float ki, float kd, float kf, /* 传入下一项参数或数据。 */
                               float max_out, float min_out, float max_iout); /* 完成本行操作。 */

/* 传入左右蓄力 3508 的当前位置真实值（累计角 deg, solved_data[3]），
 * 内部以 (left - right) 为 sync_error 跑 PID，目标 0。
 * 返回 correction，调用方按 LeftFinal=LeftTarget+correction、RightFinal=RightTarget-correction 分发。
 */
float RM_Motor_UpdateStoreSync(float left_pos_deg, float right_pos_deg); /* 声明 RM_Motor_UpdateStoreSync 接口。 */

/* 调试观测：最近一次的 sync error 与 PID 输出。 */
extern float g_RmStoreSyncErrorDeg; /* 声明外部变量 g_RmStoreSyncErrorDeg。 */
extern float g_RmStoreSyncPidOutputDeg; /* 声明外部变量 g_RmStoreSyncPidOutputDeg。 */

#endif /* 结束条件编译。 */
