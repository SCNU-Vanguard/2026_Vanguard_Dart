#ifndef __RM_MOTOR_H_
#define __RM_MOTOR_H_

#include "main.h"
#include "bsp_can.h"
#include <stdlib.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*============================== RM电机CAN地址定义 ==============================*/

#define g_RM_MOTOR_BIAS_ADDR 0x200
#define g_RM_MOTOR_BIAS_ADDR_3508 0x200
#define g_RM_MOTOR_BIAS_ADDR_2006 0x1FF
#define g_RM_MOTOR_BIAS_ADDR_6020 0x2FE

#define g_RM_MOTOR_NUM 5
#define g_RM_M3508_NUM 3
#define g_RM_M2006_NUM 1
#define g_RM_GM6020_NUM 4

#define RM_TestUse 0U

/*============================== RM电机参数常量 ==============================*/

#define RM_M2006_GEAR_RATIO 36.0f
#define RM_M2006_MAX_CURRENT 10000
#define RM_M2006_CURRENT_RATIO 0.18f

#define RM_M3508_GEAR_RATIO 19.2f
#define RM_M3508_MAX_CURRENT 16384
#define RM_M3508_CURRENT_RATIO 819.20f //(16384.0f / 20.0f)
#define RM_3508_CLEAR_ANGLE_I_ON_DIR_CHANGE 1U

#define RM_GM6020_GEAR_RATIO 1.0f
#define RM_GM6020_MAX_CURRENT 16384
#define RM_GM6020_CURRENT_RATIO 5461.330f // (16384.0f / 3.0f)

/*============================== 前向声明 ==============================*/

#pragma pack(push, 1)
struct _MotorTypeDef;

/*============================== RM电机类定义（面向对象） ==============================*/

typedef struct _RM_MotorClass
{
    const char *name;
    uint8_t model;

    float gear_ratio;
    int16_t max_current;
    float current_ratio;
    uint16_t tx_base_addr;
    uint8_t id_min;
    uint8_t id_max;

    void (*init)(struct _MotorTypeDef *self, uint8_t id);
    void (*calculate)(struct _MotorTypeDef *self);
} RM_MotorClass_t;
#pragma pack(pop)

extern const RM_MotorClass_t RM_M2006_Class;
extern const RM_MotorClass_t RM_M3508_Class;
extern const RM_MotorClass_t RM_GM6020_Class;

/*============================== RM电机配置结构 ==============================*/

#pragma pack(push, 1)
typedef struct
{
    float direction_bias;
    float position_min;
    float position_max;
    float position_tolerance;
    uint8_t reverse;
} RM_MotorConfig_t;
#pragma pack(pop)

#include "CanMotor.h"

/*============================== 面向对象接口 ==============================*/

void RM_Motor_Create(MotorTypeDef *motor, const RM_MotorClass_t *motor_class, uint8_t id);
void RM_Motor_Calculate(MotorTypeDef *motor);

/*============================== 初始化函数 ==============================*/

void RM_M2006_Create(MotorTypeDef *motor, uint8_t id);
void RM_M3508_Create(MotorTypeDef *motor, uint8_t id);
void RM_GM6020_Create(MotorTypeDef *motor, uint8_t id);

/*============================== 配置函数 ==============================*/

void RM_Motor_SetConfig(MotorTypeDef *motor, const RM_MotorConfig_t *config);
void RM_Motor_SetCascadePID(MotorTypeDef *motor,
                            float outer_p, float outer_i, float outer_d, float outer_f,
                            float inner_p, float inner_i, float inner_d, float inner_f,
                            float outer_max_out, float outer_min_out, float outer_max_iout,
                            float inner_max_out, float inner_min_out, float inner_max_iout);
void RM_Motor_SetSpeedPID(MotorTypeDef *motor, PID_MODE_e mode,
                          float p, float i, float d, float f,
                          float max_out, float min_out, float max_iout);

/*============================== 双侧蓄力 3508 同步 PID ==============================*/

/* 初始化左右蓄力 3508 的位置同步 PID（由 CanMotor.c:MotorRegister 调用）。 */
void RM_Motor_InitStoreSyncPid(float kp, float ki, float kd, float kf,
                               float max_out, float min_out, float max_iout);

/* 传入左右蓄力 3508 的当前位置真实值（accumulated deg，即 solved_data[3]），
 * 内部以 (left - right) 为 sync_error 跑 PID，目标 0。
 * 返回的 correction 建议由调用方按 ±correction/2 分发到左右目标。
 */
float RM_Motor_UpdateStoreSync(float left_pos_deg, float right_pos_deg);

/* 调试观测：最近一次的 sync error 与 PID 输出。 */
extern float g_RmStoreSyncErrorDeg;
extern float g_RmStoreSyncPidOutputDeg;

/*============================== 兼容旧接口 ==============================*/

void RM_MOTOR_CALCU(MotorTypeDef *motor);
void RM_Motor_Reset_Zero(MotorTypeDef *motor);
void RM_Motor_Reset_All(void);
void RmMotorSendCfg(can_motor_cfg motor_cfg, int16_t TargetCurrent);
void RmMotorPID_Calc(can_motor_cfg motor_cfg, float target);
void RmMotorSpeedPID_Calc(can_motor_cfg motor_cfg, float target_speed_rpm);

extern float g_RmDebugStoreLeftPosDeg;
extern float g_RmDebugStoreRightPosDeg;
extern float g_RmDebugStoreLeftPidOutput;
extern float g_RmDebugStoreRightPidOutput;
extern int16_t g_RmDebugStoreLeftFinalCurrent;
extern int16_t g_RmDebugStoreRightFinalCurrent;
extern uint8_t g_RmDebugStoreLeftLimitBlocked;
extern uint8_t g_RmDebugStoreRightLimitBlocked;

#endif
