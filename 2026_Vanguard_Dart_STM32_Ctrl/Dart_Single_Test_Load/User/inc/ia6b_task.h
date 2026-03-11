#ifndef __IA6B_TASK_H_
#define __IA6B_TASK_H_

#include "main.h"
#include "CanMotor.h"
#include "RM_Motor.h"
#include "IA6B.h"
#include "bsp_dwt.h"
#include <stdint.h>

#define LOAD3508_STICK_CENTER 1500
#define LOAD3508_STICK_RANGE 500.0f
#define LOAD3508_STICK_DEADZONE 0.15f
#define LOAD3508_MIN_SPEED_RPM (-LOAD3508_MAX_SPEED_RPM)
#define LOAD3508_MAX_SPEED_RPM 3000.0f
#define LOAD3508_SPEED_STEP_MIN 5.0f
#define LOAD3508_SPEED_STEP_MAX 80.0f
#define LOAD3508_SPEED_STEP_SENSITIVITY 25.0f
#define IBUS_LOST_TIMEOUT_MS 200U

#define ENABLE_RC_DEBUG_3508 0U
#define ENABLE_RC_DEBUG_2006 1U
#define ENABLE_DEFAULTTASK_RC_DEBUG ((ENABLE_RC_DEBUG_3508) || (ENABLE_RC_DEBUG_2006))
#define RC_INPUT_USE_UKF 1U

// 3508输出模式：0-速度环，1-串级位置环
#define LOAD3508_OUTPUT_SPEED_LOOP 0U
#define LOAD3508_OUTPUT_CASCADE_POS 1U
#define LOAD3508_OUTPUT_MODE LOAD3508_OUTPUT_CASCADE_POS

#define LOAD3508_POS_STEP_CMD_MAX_DEG_PER_FRAME 30.0f
#define LOAD3508_POS_STEP_MIN 1.0f
#define LOAD3508_POS_STEP_MAX 15.0f
#define LOAD3508_POS_STEP_SENSITIVITY 1.0f
#define LOAD3508_POS_TARGET_MIN_DEG -9000.0f
#define LOAD3508_POS_TARGET_MAX_DEG 6000.0f
#define LOAD3508_POS_TEST_START_DEG 0.0f
#define LOAD3508_POS_TEST_END_DEG 6000.0f

#define RC_INPUT_UKF_ALPHA 0.2f
#define RC_INPUT_UKF_BETA 2.0f
#define RC_INPUT_UKF_KAPPA 0.0f
#define RC_INPUT_UKF_Q 50.0f
#define RC_INPUT_UKF_R 400.0f

#define LOAD2006_STEP_RAW_CHANNEL_INDEX 2U
#define LOAD2006_DIR_RAW_CHANNEL_INDEX 5U
#define LOAD2006_MIN_SPEED_RPM -10000.0f
#define LOAD2006_MAX_SPEED_RPM 10000.0f
#define LOAD2006_SPEED_STEP_MIN 200.0f
#define LOAD2006_SPEED_STEP_MAX 1000.0f
#define LOAD2006_SPEED_STEP_SENSITIVITY 25.0f
#define LOAD2006_STEP_RAW_IDLE_MAX 1350
#define LOAD2006_STEP_RAW_MAX 2000
#define LOAD2006_DIR_INC_RAW_MIN 1400
#define LOAD2006_DIR_INC_RAW_MAX 1600
#define LOAD2006_DIR_DEC_RAW_MIN 1850
#define LOAD2006_POS_TARGET_MIN_DEG -250000.0f
#define LOAD2006_POS_TARGET_MAX_DEG 250000.0f

extern float RmMotorTargetSpeedData;
extern float RmMotorTargetPosData;
extern float RcLoad3508DesiredSpeedRpm;
extern float RcLoad3508DesiredPosStepDeg;
extern float RcLoad3508UkfSpeedRpm;
extern float RcLoad3508StepSpeedRpm;
extern float RcLoad3508StepValuePerFrame;
extern float RcLoad3508StepPosStepDeg;
extern float RcLoad3508PosStepValuePerFrame;
extern float RcLoad3508UkfCovP;
extern int16_t RcLoad3508Raw;
extern int8_t RcSWBState;
extern uint8_t RcLoad3508PosTargetInitialized;

extern float RmMotor2006TargetPosData;
extern float RmMotor2006SpeedData;
extern float RcLoad2006DesiredPosStepDeg;
extern float RcLoad2006PosStepValuePerFrame;
extern int16_t RcLoad2006Raw;

extern float RmMotorPosTargetTestCurrentMs;
extern float RmMotorPosTargetTestLastMs;
extern float RmMotorPosTargetTestStartTimestampMs;
extern float RmMotorPosTargetTestPrevDeg;
extern uint32_t RmMotorPosTargetTestRoundCount;
extern uint8_t RmMotorPosTargetTestState;

void IA6BTask_Init(void);
void IA6BTask_ProcessAndControl(void);

#endif
