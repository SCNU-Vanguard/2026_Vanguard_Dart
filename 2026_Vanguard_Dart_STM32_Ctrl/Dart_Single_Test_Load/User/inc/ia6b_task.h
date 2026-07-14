#ifndef __IA6B_TASK_H_
#define __IA6B_TASK_H_

#include "main.h"
#include "config.h"
#include "CanMotor.h"
#include "RM_Motor.h"
#include "IA6B.h"
#include "bsp_dwt.h"
#include <stdint.h>

#define RC_DEBUG_CENTER_STICK_RAW_CHANNEL_INDEX 3U

#define LOAD3508_RAW_CHANNEL_INDEX RC_DEBUG_CENTER_STICK_RAW_CHANNEL_INDEX
#define LOAD3508_STICK_CENTER 1500
#define LOAD3508_STICK_RANGE 500.0f
#define LOAD3508_STICK_DEADZONE 0.15f
#define LOAD3508_MIN_SPEED_RPM (-LOAD3508_MAX_SPEED_RPM)
#define LOAD3508_MAX_SPEED_RPM 350.0f
#define LOAD3508_SPEED_STEP_MIN 0.0f
#define LOAD3508_SPEED_STEP_MAX 0.0f
#define LOAD3508_SPEED_STEP_SENSITIVITY 20.0f
#define IBUS_LOST_TIMEOUT_MS 200U

#define ENABLE_RC_DEBUG_3508_STORE_LEFT 1U
#define ENABLE_RC_DEBUG_3508_STORE_RIGHT 0U
#define ENABLE_RC_DEBUG_3508_STORE_BOTH 0U
#if ((ENABLE_RC_DEBUG_3508_STORE_LEFT + ENABLE_RC_DEBUG_3508_STORE_RIGHT + ENABLE_RC_DEBUG_3508_STORE_BOTH) > 1U)
#error "Only one 3508 RC debug mode can be enabled at a time."
#endif
#define ENABLE_RC_DEBUG_3508 ((ENABLE_RC_DEBUG_3508_STORE_LEFT) || (ENABLE_RC_DEBUG_3508_STORE_RIGHT) || (ENABLE_RC_DEBUG_3508_STORE_BOTH))

#define ENABLE_RC_DEBUG_2006 0U
#define ENABLE_RC_DEBUG_6020 0U
#define ENABLE_DEFAULTTASK_RC_DEBUG ((ENABLE_RC_DEBUG_3508) || (ENABLE_RC_DEBUG_2006) || (ENABLE_RC_DEBUG_6020))
#define RC_INPUT_USE_UKF 1U

/* 3508 output mode:
 * 0: tune speed-loop PID first
 * 1: tune cascade position-loop PID after the speed loop is stable
 */
#define LOAD3508_OUTPUT_SPEED_LOOP 0U
#define LOAD3508_OUTPUT_CASCADE_POS 1U
#define LOAD3508_OUTPUT_MODE LOAD3508_OUTPUT_CASCADE_POS

#define LOAD3508_POS_STEP_CMD_MAX_DEG_PER_FRAME 10.0f
#define LOAD3508_POS_STEP_MIN 2.0f
#define LOAD3508_POS_STEP_MAX 8.0f
#define LOAD3508_POS_STEP_SENSITIVITY 0.4f
#define LOAD3508_POS_TARGET_MIN_DEG -9000.0f
#define LOAD3508_POS_TARGET_MAX_DEG 6000.0f
/* 双电机模式下，base 目标按正向行程表示；左侧实际目标取镜像负值，右侧实际目标取正值。 */
#define LOAD3508_BOTH_TARGET_MIN_DEG 0.0f
#define LOAD3508_BOTH_TARGET_MAX_DEG LimitStore
#define LOAD3508_POS_TEST_START_DEG 0.0f
#define LOAD3508_POS_TEST_END_DEG 6000.0f

/* 同步 PID：控制 left + right -> 0，使两侧镜像位置一致（参数定义在 config.h: STORE3508_SYNC_PID_*）。 */

#define RC_INPUT_UKF_ALPHA 0.2f
#define RC_INPUT_UKF_BETA 2.0f
#define RC_INPUT_UKF_KAPPA 0.0f
#define RC_INPUT_UKF_Q 10.0f
#define RC_INPUT_UKF_R 2000.0f

#define LOAD2006_USE_CENTER_STICK 0U
#define LOAD2006_CENTER_RAW_CHANNEL_INDEX RC_DEBUG_CENTER_STICK_RAW_CHANNEL_INDEX
#define LOAD2006_CENTER_STICK_CENTER LOAD3508_STICK_CENTER
#define LOAD2006_CENTER_STICK_RANGE LOAD3508_STICK_RANGE
#define LOAD2006_CENTER_STICK_DEADZONE LOAD3508_STICK_DEADZONE
#define LOAD2006_CENTER_POS_STEP_MAX_DEG_PER_FRAME LOAD2006_SPEED_STEP_MAX
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

#define LOAD6020_RAW_CHANNEL_INDEX RC_DEBUG_CENTER_STICK_RAW_CHANNEL_INDEX
#define LOAD6020_STICK_CENTER 1500
#define LOAD6020_STICK_RANGE 500.0f
#define LOAD6020_STICK_DEADZONE 0.15f
#define LOAD6020_MIN_SPEED_RPM (-LOAD6020_MAX_SPEED_RPM)
#define LOAD6020_MAX_SPEED_RPM 330.0f
#define LOAD6020_SPEED_STEP_MIN 1.0f
#define LOAD6020_SPEED_STEP_MAX 10.0f
#define LOAD6020_SPEED_STEP_SENSITIVITY 10.0f
#define LOAD6020_OUTPUT_SPEED_LOOP 0U
#define LOAD6020_OUTPUT_CASCADE_POS 1U
#define LOAD6020_OUTPUT_MODE LOAD6020_OUTPUT_CASCADE_POS
#define LOAD6020_POS_STEP_CMD_MAX_DEG_PER_FRAME 2.0f
#define LOAD6020_POS_STEP_MIN 0.1f
#define LOAD6020_POS_STEP_MAX 1.0f
#define LOAD6020_POS_STEP_SENSITIVITY 2.0f
#define LOAD6020_POS_TARGET_MIN_DEG -2000.0f
#define LOAD6020_POS_TARGET_MAX_DEG 4000.0f

extern float g_RcDebug3508TargetSpeedRpm;
extern float g_RcDebug3508TargetPosDeg;
extern float g_RcDebug3508DesiredSpeedRpm;
extern float g_RcDebug3508DesiredPosStepDeg;
extern float g_RcDebug3508PreviewDesiredPosStepDeg;
extern float g_RcDebug3508PreviewStickNorm;
extern float g_RcDebug3508UkfSpeedRpm;
extern float g_RcDebug3508StepSpeedRpm;
extern float g_RcDebug3508SpeedStepPerFrame;
extern float g_RcDebug3508StepPosDeg;
extern float g_RcDebug3508PreviewStepPosDeg;
extern float g_RcDebug3508PosStepPerFrame;
extern float g_RcDebug3508PreviewPosStepPerFrame;
extern int8_t g_RcDebug3508PreviewSwbState;
extern float g_RcDebug3508UkfCovP;
extern int16_t g_RcDebug3508Raw;
extern int8_t g_RcDebugSwbState;
extern uint8_t g_RcDebug3508PosTargetInitialized;

extern float g_RcDebug3508LeftTargetPosDeg;
extern float g_RcDebug3508RightTargetPosDeg;
extern float g_RcDebug3508LeftPosDeg;
extern float g_RcDebug3508RightPosDeg;
extern float g_RcDebug3508LeftSpeedRpm;
extern float g_RcDebug3508RightSpeedRpm;
extern float g_RcDebug3508SyncErrorDeg;
extern float g_RcDebug3508SyncPidOutputDeg;

extern float g_RcDebug2006TargetPosDeg;
extern float g_RcDebug2006SpeedRpm;
extern float g_RcDebug2006DesiredPosStepDeg;
extern float g_RcDebug2006PosStepPerFrame;
extern int16_t g_RcDebug2006Raw;

extern float g_RcDebug6020TargetSpeedRpm;
extern float g_RcDebug6020TargetPosDeg;
extern float g_RcDebug6020SpeedRpm;
extern float g_RcDebug6020DesiredSpeedRpm;
extern float g_RcDebug6020DesiredPosStepDeg;
extern float g_RcDebug6020StepSpeedRpm;
extern float g_RcDebug6020SpeedStepPerFrame;
extern float g_RcDebug6020StepPosDeg;
extern float g_RcDebug6020PosStepPerFrame;
extern int16_t g_RcDebug6020Raw;
extern uint8_t g_RcDebug6020PosTargetInitialized;

extern float RmMotorPosTargetTestCurrentMs;
extern float RmMotorPosTargetTestLastMs;
extern float RmMotorPosTargetTestStartTimestampMs;
extern float RmMotorPosTargetTestPrevDeg;
extern uint32_t RmMotorPosTargetTestRoundCount;
extern uint8_t RmMotorPosTargetTestState;

void IA6BTask_Init(void);
void IA6BTask_ProcessAndControl(void);

#endif
