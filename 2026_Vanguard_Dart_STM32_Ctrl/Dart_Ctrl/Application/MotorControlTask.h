#ifndef __MOTOR_CONTROL_TASK_H_
#define __MOTOR_CONTROL_TASK_H_

#include "bsp_can.h"
#include "CanMotor.h"
#include "RM_Motor.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    LOAD_MOTOR_OWNER_NONE = 0,
    LOAD_MOTOR_OWNER_STATE_SET,
    LOAD_MOTOR_OWNER_LOAD,
    LOAD_MOTOR_OWNER_HOME,
} LoadMotorOwner_e;

#define LOAD_MOTOR_PRIORITY_STATE_SET 20U
#define LOAD_MOTOR_PRIORITY_LOAD 80U
#define LOAD_MOTOR_PRIORITY_HOME 90U

void MotorControl_Init(void);

bool StoreMotor_Enable(can_motor_cfg motor_cfg);
void StoreMotor_Disable(can_motor_cfg motor_cfg);
void StoreMotor_EnableControl(can_motor_cfg motor_cfg, bool enable);
void StoreMotor_SetTarget(can_motor_cfg motor_cfg, float target);
float StoreMotor_GetTarget(can_motor_cfg motor_cfg);
void StoreMotor_RefreshData(can_motor_cfg motor_cfg);
void StoreMotor_SetUseSCurve(can_motor_cfg motor_cfg, bool enable);
bool StoreMotor_IsProtected(can_motor_cfg motor_cfg);
bool StoreMotor_IsAnyProtected(void);
void StoreMotor_ClearProtection(void);

void Motor3508_SetTarget(float target);
float Motor3508_GetTarget(void);
void Motor3508_EnableControl(bool enable);
bool LoadMotor_IsOverCurrentProtected(void);
void LoadMotor_ClearOverCurrentProtection(void);
bool LoadMotor_SubmitTarget(LoadMotorOwner_e owner, uint8_t priority, float target_pos_deg, uint32_t owner_hold_ms);
void LoadMotor_ReleaseOwner(LoadMotorOwner_e owner);

void Motor2006_SetTarget(float target);
float Motor2006_GetTarget(void);
void Motor2006_EnableControl(bool enable);

void Motor6020_SetTarget(float target);
float Motor6020_GetTarget(void);
void Motor6020_EnableControl(bool enable);

void Motor3508CtrlTask(void *argument);
void Motor2006CtrlTask(void *argument);
void MotorLeftStore3508CtrlTask(void *argument);
void MotorRightStore3508CtrlTask(void *argument);
void Motor6020CtrlTask(void *argument);

#endif
