#ifndef __MOTOR_ALGROTHIM_H_
#define __MOTOR_ALGROTHIM_H_

#include <stdint.h>

typedef struct
{
    float target_pos;
    float cmd_pos;
    float cmd_vel;
    float cmd_acc;
    float vmax;
    float amax;
    float jmax;
    uint8_t initialized;
} MotorTrapPosProfile_t;

void Motor_TrapPos_Init(MotorTrapPosProfile_t *profile, float initial_pos, float vmax, float amax);
void Motor_TrapPos_SetJerk(MotorTrapPosProfile_t *profile, float jmax);
void Motor_TrapPos_Reset(MotorTrapPosProfile_t *profile, float pos);
float Motor_TrapPos_Update(MotorTrapPosProfile_t *profile, float target_pos, float dt_s);

#endif
