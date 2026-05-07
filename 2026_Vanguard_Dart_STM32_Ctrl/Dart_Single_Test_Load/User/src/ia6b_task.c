#include "ia6b_task.h"
#include <math.h>
#include <stdbool.h>

typedef struct
{
    float x;
    float P;
    float q;
    float r;
    float alpha;
    float beta;
    float kappa;
    uint8_t initialized;
} UKF1D_t;

typedef struct
{
    can_motor_cfg motor_id;
    float min_pos_deg;
    float max_pos_deg;
} RcDebug3508MotorInfo_t;

float g_RcDebug3508TargetSpeedRpm = 0.0f;
float g_RcDebug3508TargetPosDeg = 0.0f;
float g_RcDebug3508LeftTargetPosDeg = 0.0f;
float g_RcDebug3508RightTargetPosDeg = 0.0f;
float g_RcDebug3508LeftPosDeg = 0.0f;
float g_RcDebug3508RightPosDeg = 0.0f;
float g_RcDebug3508LeftSpeedRpm = 0.0f;
float g_RcDebug3508RightSpeedRpm = 0.0f;
float g_RcDebug3508SyncErrorDeg = 0.0f;
float g_RcDebug3508SyncPidOutputDeg = 0.0f;
float g_RcDebug2006TargetPosDeg = 0.0f;
float g_RcDebug2006SpeedRpm = 0.0f;
float g_RcDebug3508DesiredSpeedRpm = 0.0f;
float g_RcDebug3508DesiredPosStepDeg = 0.0f;
float g_RcDebug3508UkfSpeedRpm = 0.0f;
float g_RcDebug3508StepSpeedRpm = 0.0f;
float g_RcDebug3508SpeedStepPerFrame = LOAD3508_SPEED_STEP_MIN;
float g_RcDebug3508StepPosDeg = 0.0f;
float g_RcDebug3508PosStepPerFrame = LOAD3508_POS_STEP_MIN;
float g_RcDebug3508UkfCovP = 0.0f;
int16_t g_RcDebug3508Raw = 0;
int8_t g_RcDebugSwbState = 0;
uint8_t g_RcDebug3508PosTargetInitialized = 0U;
float g_RcDebug2006DesiredPosStepDeg = 0.0f;
float g_RcDebug2006PosStepPerFrame = LOAD2006_SPEED_STEP_MIN;
int16_t g_RcDebug2006Raw = 0;
float g_RcDebug6020TargetSpeedRpm = 0.0f;
float g_RcDebug6020TargetPosDeg = 0.0f;
float g_RcDebug6020SpeedRpm = 0.0f;
float g_RcDebug6020DesiredSpeedRpm = 0.0f;
float g_RcDebug6020DesiredPosStepDeg = 0.0f;
float g_RcDebug6020StepSpeedRpm = 0.0f;
float g_RcDebug6020SpeedStepPerFrame = LOAD6020_SPEED_STEP_MIN;
float g_RcDebug6020StepPosDeg = 0.0f;
float g_RcDebug6020PosStepPerFrame = LOAD6020_POS_STEP_MIN;
int16_t g_RcDebug6020Raw = 0;
uint8_t g_RcDebug6020PosTargetInitialized = 0U;

float RmMotorPosTargetTestCurrentMs = 0.0f;
float RmMotorPosTargetTestLastMs = 0.0f;
float RmMotorPosTargetTestStartTimestampMs = 0.0f;
float RmMotorPosTargetTestPrevDeg = 0.0f;
uint32_t RmMotorPosTargetTestRoundCount = 0U;
uint8_t RmMotorPosTargetTestState = 0U;

extern float RmMotorAngleData;
extern float RmMotorSpeedData;

static uint32_t g_last_ibus_tick = 0U;
static float g_prev_desired_speed_rpm_3508 = 0.0f;
static float g_prev_desired_pos_step_deg = 0.0f;
static float g_prev_desired_speed_rpm_6020 = 0.0f;
static float g_prev_desired_pos_step_deg_6020 = 0.0f;
static PID_t g_RcDebug3508SyncPid;

#if RC_INPUT_USE_UKF
static UKF1D_t g_RcInputUkf = {0};
#endif

static const RcDebug3508MotorInfo_t g_RcDebug3508PrimaryMotorInfo =
#if ENABLE_RC_DEBUG_3508_STORE_RIGHT
    {RM_3508_STORE_RIGHT, RightStoreTop, LimitStore};
#else
    {RM_3508_STORE_LEFT, -LimitStore, LeftStoreTop};
#endif

static float NormalizeCenterStick(int16_t raw);
static float NormalizeCenterStickWithConfig(int16_t raw, int16_t center, float range, float deadzone);
static float ClampFloat(float value, float min_value, float max_value);
static float StepToward(float current, float target, float step);
static float ComputeAdaptiveStep(float desired_value, float prev_desired_value, float tracking_error,
                                 float value_limit_abs, float step_min, float step_max, float step_sensitivity);
static float UpdateSpeedTargetWithStep(float desired_speed_rpm, float *step_speed_rpm, float *step_value_per_frame,
                                       float *prev_desired_speed_rpm, float min_speed_rpm, float max_speed_rpm,
                                       float step_min, float step_max, float step_sensitivity, bool ibus_updated);
static void UpdatePosTargetDurationTest(float target_pos_deg);
static uint8_t IsMotorHandleReady(can_motor_cfg motor_id);
static float GetMotorRelativePosDeg(can_motor_cfg motor_id);
static float GetMotorRelativeSpeedRpm(can_motor_cfg motor_id);
static void Initialize3508PosTargetsIfNeeded(void);
static void Update3508CascadeTargets(float stick_norm, bool ibus_updated);
#if RC_INPUT_USE_UKF
static void UKF1D_Reset(UKF1D_t *ukf, float init_x);
static float UKF1D_Update(UKF1D_t *ukf, float z);
#endif
static void RC3508ResetStates(void);
static void RC2006ResetStates(void);
static void RC6020ResetStates(void);

void IA6BTask_Init(void)
{
    g_last_ibus_tick = 0U;
    g_prev_desired_speed_rpm_3508 = 0.0f;
    g_prev_desired_pos_step_deg = 0.0f;
    g_prev_desired_speed_rpm_6020 = 0.0f;
    g_prev_desired_pos_step_deg_6020 = 0.0f;
    g_RcDebug3508PosTargetInitialized = 0U;
    g_RcDebug6020PosTargetInitialized = 0U;

    g_RcDebug3508SpeedStepPerFrame = LOAD3508_SPEED_STEP_MIN;
    g_RcDebug3508PosStepPerFrame = LOAD3508_POS_STEP_MIN;
    g_RcDebug2006PosStepPerFrame = LOAD2006_SPEED_STEP_MIN;
    g_RcDebug6020SpeedStepPerFrame = LOAD6020_SPEED_STEP_MIN;
    g_RcDebug6020PosStepPerFrame = LOAD6020_POS_STEP_MIN;
    g_RcDebug2006TargetPosDeg = 0.0f;
    PID_Init(&g_RcDebug3508SyncPid, PID_POSITION,
             LOAD3508_SYNC_PID_KP, LOAD3508_SYNC_PID_KI, LOAD3508_SYNC_PID_KD, LOAD3508_SYNC_PID_KF,
             LOAD3508_SYNC_PID_MAX_OUT, LOAD3508_SYNC_PID_MIN_OUT, LOAD3508_SYNC_PID_MAX_IOUT);
    PID_Clear(&g_RcDebug3508SyncPid);

#if ENABLE_RC_DEBUG_3508
    g_RcDebug3508TargetPosDeg = 0.0f;
    g_RcDebug3508LeftTargetPosDeg = 0.0f;
    g_RcDebug3508RightTargetPosDeg = 0.0f;
#endif

    RC3508ResetStates();
    RC2006ResetStates();
    RC6020ResetStates();
}

void IA6BTask_ProcessAndControl(void)
{
#if ENABLE_DEFAULTTASK_RC_DEBUG
    bool ibus_updated = false;
    bool rc_online = false;
    bool rc_enabled_3508 = false;
    bool rc_enabled_2006 = false;
    bool rc_enabled_6020 = false;

    if (IA6B_ProcessIbusPacket(BSP_UART6))
    {
        ibus_updated = true;
        g_last_ibus_tick = HAL_GetTick();
    }

    rc_online = (g_last_ibus_tick != 0U) &&
                ((HAL_GetTick() - g_last_ibus_tick) <= IBUS_LOST_TIMEOUT_MS);

    g_RcDebugSwbState = Channel[4];
    rc_enabled_3508 = rc_online && (g_RcDebugSwbState == 0);
    rc_enabled_2006 = rc_online;
    rc_enabled_6020 = rc_online;

#if ENABLE_RC_DEBUG_3508
    g_RcDebug3508DesiredSpeedRpm = 0.0f;
    g_RcDebug3508DesiredPosStepDeg = 0.0f;
    g_RcDebug3508UkfSpeedRpm = 0.0f;
    g_RcDebug3508Raw = RawChannel[LOAD3508_RAW_CHANNEL_INDEX];
    g_RcDebug3508LeftPosDeg = GetMotorRelativePosDeg(RM_3508_STORE_LEFT);
    g_RcDebug3508RightPosDeg = GetMotorRelativePosDeg(RM_3508_STORE_RIGHT);
    g_RcDebug3508LeftSpeedRpm = GetMotorRelativeSpeedRpm(RM_3508_STORE_LEFT);
    g_RcDebug3508RightSpeedRpm = GetMotorRelativeSpeedRpm(RM_3508_STORE_RIGHT);
    g_RcDebug3508SyncErrorDeg = g_RcDebug3508LeftPosDeg + g_RcDebug3508RightPosDeg;
    g_RcDebug3508SyncPidOutputDeg = 0.0f;
#if RC_INPUT_USE_UKF
    g_RcDebug3508UkfCovP = g_RcInputUkf.P;
#else
    g_RcDebug3508UkfCovP = 0.0f;
#endif

    if (rc_enabled_3508)
    {
        float stick_norm = 0.0f;
        float filtered_speed_rpm = 0.0f;
        if (g_RcDebug3508Raw >= 900 && g_RcDebug3508Raw <= 2100)
        {
            stick_norm = NormalizeCenterStick(g_RcDebug3508Raw);
            g_RcDebug3508DesiredSpeedRpm = stick_norm * LOAD3508_MAX_SPEED_RPM;
            g_RcDebug3508DesiredSpeedRpm = ClampFloat(g_RcDebug3508DesiredSpeedRpm, LOAD3508_MIN_SPEED_RPM, LOAD3508_MAX_SPEED_RPM);
        }

#if RC_INPUT_USE_UKF
        g_RcDebug3508UkfSpeedRpm = UKF1D_Update(&g_RcInputUkf, g_RcDebug3508DesiredSpeedRpm);
        g_RcDebug3508UkfCovP = g_RcInputUkf.P;
        filtered_speed_rpm = g_RcDebug3508UkfSpeedRpm;
#else
        g_RcDebug3508UkfSpeedRpm = g_RcDebug3508DesiredSpeedRpm;
        g_RcDebug3508UkfCovP = 0.0f;
        filtered_speed_rpm = g_RcDebug3508UkfSpeedRpm;
#endif

        filtered_speed_rpm = ClampFloat(filtered_speed_rpm, LOAD3508_MIN_SPEED_RPM, LOAD3508_MAX_SPEED_RPM);
        g_RcDebug3508TargetSpeedRpm = UpdateSpeedTargetWithStep(filtered_speed_rpm,
                                                                &g_RcDebug3508StepSpeedRpm,
                                                                &g_RcDebug3508SpeedStepPerFrame,
                                                                &g_prev_desired_speed_rpm_3508,
                                                                LOAD3508_MIN_SPEED_RPM,
                                                                LOAD3508_MAX_SPEED_RPM,
                                                                LOAD3508_SPEED_STEP_MIN,
                                                                LOAD3508_SPEED_STEP_MAX,
                                                                LOAD3508_SPEED_STEP_SENSITIVITY,
                                                                ibus_updated);

#if LOAD3508_OUTPUT_MODE == LOAD3508_OUTPUT_CASCADE_POS
        Update3508CascadeTargets(stick_norm, ibus_updated);
#endif
        if (ibus_updated)
        {
            g_prev_desired_pos_step_deg = g_RcDebug3508DesiredPosStepDeg;
        }
    }
    else
    {
        RC3508ResetStates();
    }
#endif

#if ENABLE_RC_DEBUG_2006
    g_RcDebug2006DesiredPosStepDeg = 0.0f;
#if LOAD2006_USE_CENTER_STICK
    g_RcDebug2006Raw = RawChannel[LOAD2006_CENTER_RAW_CHANNEL_INDEX];
    if (rc_enabled_2006)
    {
        float stick_norm = 0.0f;
        if (g_RcDebug2006Raw >= 900 && g_RcDebug2006Raw <= 2100)
        {
            stick_norm = NormalizeCenterStickWithConfig(g_RcDebug2006Raw,
                                                        LOAD2006_CENTER_STICK_CENTER,
                                                        LOAD2006_CENTER_STICK_RANGE,
                                                        LOAD2006_CENTER_STICK_DEADZONE);
        }

        g_RcDebug2006DesiredPosStepDeg = stick_norm * LOAD2006_CENTER_POS_STEP_MAX_DEG_PER_FRAME;
        g_RcDebug2006PosStepPerFrame = g_RcDebug2006DesiredPosStepDeg;
        g_RcDebug2006TargetPosDeg += g_RcDebug2006DesiredPosStepDeg;
        g_RcDebug2006TargetPosDeg = ClampFloat(g_RcDebug2006TargetPosDeg,
                                               LOAD2006_POS_TARGET_MIN_DEG,
                                               LOAD2006_POS_TARGET_MAX_DEG);
    }
    else
    {
        RC2006ResetStates();
    }
#else
    g_RcDebug2006Raw = RawChannel[LOAD2006_STEP_RAW_CHANNEL_INDEX];
    if (rc_enabled_2006)
    {
        int16_t rc_dir_raw = RawChannel[LOAD2006_DIR_RAW_CHANNEL_INDEX];

        if (g_RcDebug2006Raw > LOAD2006_STEP_RAW_IDLE_MAX)
        {
            float ratio = (float)(g_RcDebug2006Raw - LOAD2006_STEP_RAW_IDLE_MAX) /
                          (float)(LOAD2006_STEP_RAW_MAX - LOAD2006_STEP_RAW_IDLE_MAX);
            ratio = ClampFloat(ratio, 0.0f, 1.0f);
            g_RcDebug2006PosStepPerFrame = LOAD2006_SPEED_STEP_MIN +
                                           (LOAD2006_SPEED_STEP_MAX - LOAD2006_SPEED_STEP_MIN) * ratio;
        }
        else
        {
            g_RcDebug2006PosStepPerFrame = 0.0f;
        }

        if ((rc_dir_raw >= LOAD2006_DIR_INC_RAW_MIN) && (rc_dir_raw <= LOAD2006_DIR_INC_RAW_MAX))
        {
            g_RcDebug2006DesiredPosStepDeg = g_RcDebug2006PosStepPerFrame;
        }
        else if (rc_dir_raw >= LOAD2006_DIR_DEC_RAW_MIN)
        {
            g_RcDebug2006DesiredPosStepDeg = -g_RcDebug2006PosStepPerFrame;
        }

        g_RcDebug2006TargetPosDeg += g_RcDebug2006DesiredPosStepDeg;
        g_RcDebug2006TargetPosDeg = ClampFloat(g_RcDebug2006TargetPosDeg,
                                               LOAD2006_POS_TARGET_MIN_DEG,
                                               LOAD2006_POS_TARGET_MAX_DEG);
    }
    else
    {
        RC2006ResetStates();
    }
#endif
#endif

#if ENABLE_RC_DEBUG_6020
    g_RcDebug6020DesiredSpeedRpm = 0.0f;
    g_RcDebug6020DesiredPosStepDeg = 0.0f;
    g_RcDebug6020Raw = RawChannel[LOAD6020_RAW_CHANNEL_INDEX];
    if (rc_enabled_6020)
    {
        float stick_norm = 0.0f;
        if (g_RcDebug6020Raw >= 900 && g_RcDebug6020Raw <= 2100)
        {
            stick_norm = NormalizeCenterStickWithConfig(g_RcDebug6020Raw,
                                                        LOAD6020_STICK_CENTER,
                                                        LOAD6020_STICK_RANGE,
                                                        LOAD6020_STICK_DEADZONE);
            g_RcDebug6020DesiredSpeedRpm = stick_norm * LOAD6020_MAX_SPEED_RPM;
            g_RcDebug6020DesiredSpeedRpm = ClampFloat(g_RcDebug6020DesiredSpeedRpm,
                                                      LOAD6020_MIN_SPEED_RPM,
                                                      LOAD6020_MAX_SPEED_RPM);
        }

#if LOAD6020_OUTPUT_MODE == LOAD6020_OUTPUT_CASCADE_POS
        if (!g_RcDebug6020PosTargetInitialized)
        {
            MotorTypeDef *motor = Motor_GetHandle(RM_6020_YAW);
            if (motor != NULL)
            {
                g_RcDebug6020TargetPosDeg = motor->motor_data.solved_data[3];
                g_RcDebug6020PosTargetInitialized = 1U;
            }
        }
        if (g_RcDebug6020PosTargetInitialized)
        {
            g_RcDebug6020DesiredPosStepDeg = stick_norm * LOAD6020_POS_STEP_CMD_MAX_DEG_PER_FRAME;
            if (ibus_updated)
            {
                float tracking_error_pos = g_RcDebug6020DesiredPosStepDeg - g_RcDebug6020StepPosDeg;
                if (tracking_error_pos < 0.0f)
                {
                    tracking_error_pos = -tracking_error_pos;
                }
                g_RcDebug6020PosStepPerFrame = ComputeAdaptiveStep(g_RcDebug6020DesiredPosStepDeg,
                                                                   g_prev_desired_pos_step_deg_6020,
                                                                   tracking_error_pos,
                                                                   LOAD6020_POS_STEP_CMD_MAX_DEG_PER_FRAME,
                                                                   LOAD6020_POS_STEP_MIN,
                                                                   LOAD6020_POS_STEP_MAX,
                                                                   LOAD6020_POS_STEP_SENSITIVITY);
            }

            g_RcDebug6020StepPosDeg = StepToward(g_RcDebug6020StepPosDeg,
                                                 g_RcDebug6020DesiredPosStepDeg,
                                                 g_RcDebug6020PosStepPerFrame);
            if (g_RcDebug6020StepPosDeg < -LOAD6020_POS_STEP_CMD_MAX_DEG_PER_FRAME)
            {
                g_RcDebug6020StepPosDeg = -LOAD6020_POS_STEP_CMD_MAX_DEG_PER_FRAME;
            }
            else if (g_RcDebug6020StepPosDeg > LOAD6020_POS_STEP_CMD_MAX_DEG_PER_FRAME)
            {
                g_RcDebug6020StepPosDeg = LOAD6020_POS_STEP_CMD_MAX_DEG_PER_FRAME;
            }

            g_RcDebug6020TargetPosDeg += g_RcDebug6020StepPosDeg;
            g_RcDebug6020TargetPosDeg = ClampFloat(g_RcDebug6020TargetPosDeg,
                                                   LOAD6020_POS_TARGET_MIN_DEG,
                                                   LOAD6020_POS_TARGET_MAX_DEG);
        }
        if (ibus_updated)
        {
            g_prev_desired_pos_step_deg_6020 = g_RcDebug6020DesiredPosStepDeg;
        }
#else
        g_RcDebug6020TargetSpeedRpm = UpdateSpeedTargetWithStep(g_RcDebug6020DesiredSpeedRpm,
                                                                &g_RcDebug6020StepSpeedRpm,
                                                                &g_RcDebug6020SpeedStepPerFrame,
                                                                &g_prev_desired_speed_rpm_6020,
                                                                LOAD6020_MIN_SPEED_RPM,
                                                                LOAD6020_MAX_SPEED_RPM,
                                                                LOAD6020_SPEED_STEP_MIN,
                                                                LOAD6020_SPEED_STEP_MAX,
                                                                LOAD6020_SPEED_STEP_SENSITIVITY,
                                                                ibus_updated);
#endif
    }
    else
    {
        RC6020ResetStates();
    }
#endif

#if ENABLE_RC_DEBUG_3508
#if LOAD3508_OUTPUT_MODE == LOAD3508_OUTPUT_CASCADE_POS
    UpdatePosTargetDurationTest(g_RcDebug3508TargetPosDeg);
#endif
#endif

#else
    // 保持兼容：即使外层误调用，本函数也不输出控制
    RC3508ResetStates();
    RC2006ResetStates();
    RC6020ResetStates();
#endif
}

static void RC3508ResetStates(void)
{
#if RC_INPUT_USE_UKF
    UKF1D_Reset(&g_RcInputUkf, 0.0f);
#endif
    g_RcDebug3508StepSpeedRpm = 0.0f;
    g_RcDebug3508SpeedStepPerFrame = LOAD3508_SPEED_STEP_MIN;
    g_RcDebug3508DesiredPosStepDeg = 0.0f;
    g_RcDebug3508StepPosDeg = 0.0f;
    g_RcDebug3508PosStepPerFrame = LOAD3508_POS_STEP_MIN;
    g_RcDebug3508TargetSpeedRpm = 0.0f;
    g_RcDebug3508UkfSpeedRpm = 0.0f;
#if RC_INPUT_USE_UKF
    g_RcDebug3508UkfCovP = g_RcInputUkf.P;
#else
    g_RcDebug3508UkfCovP = 0.0f;
#endif
    g_prev_desired_speed_rpm_3508 = 0.0f;
    g_prev_desired_pos_step_deg = 0.0f;
    g_RcDebug3508PosTargetInitialized = 0U;
    g_RcDebug3508TargetPosDeg = 0.0f;
    g_RcDebug3508LeftTargetPosDeg = 0.0f;
    g_RcDebug3508RightTargetPosDeg = 0.0f;
    g_RcDebug3508SyncErrorDeg = 0.0f;
    g_RcDebug3508SyncPidOutputDeg = 0.0f;
    PID_Clear(&g_RcDebug3508SyncPid);
}

static void RC2006ResetStates(void)
{
    g_RcDebug2006DesiredPosStepDeg = 0.0f;
    g_RcDebug2006PosStepPerFrame = 0.0f;
}

static void RC6020ResetStates(void)
{
    g_RcDebug6020DesiredSpeedRpm = 0.0f;
    g_RcDebug6020StepSpeedRpm = 0.0f;
    g_RcDebug6020SpeedStepPerFrame = LOAD6020_SPEED_STEP_MIN;
    g_RcDebug6020TargetSpeedRpm = 0.0f;
    g_RcDebug6020DesiredPosStepDeg = 0.0f;
    g_RcDebug6020StepPosDeg = 0.0f;
    g_RcDebug6020PosStepPerFrame = LOAD6020_POS_STEP_MIN;
    g_prev_desired_speed_rpm_6020 = 0.0f;
    g_prev_desired_pos_step_deg_6020 = 0.0f;
    g_RcDebug6020PosTargetInitialized = 0U;
}

static float NormalizeCenterStick(int16_t raw)
{
    return NormalizeCenterStickWithConfig(raw,
                                          LOAD3508_STICK_CENTER,
                                          LOAD3508_STICK_RANGE,
                                          LOAD3508_STICK_DEADZONE);
}

static uint8_t IsMotorHandleReady(can_motor_cfg motor_id)
{
    return (Motor_GetHandle(motor_id) != NULL) ? 1U : 0U;
}

static float GetMotorRelativePosDeg(can_motor_cfg motor_id)
{
    MotorTypeDef *motor = Motor_GetHandle(motor_id);
    if (motor == NULL)
    {
        return 0.0f;
    }
    return motor->motor_data.solved_data[3] - motor->motor_data.offset_ecd_angle;
}

static float GetMotorRelativeSpeedRpm(can_motor_cfg motor_id)
{
    MotorTypeDef *motor = Motor_GetHandle(motor_id);
    if (motor == NULL)
    {
        return 0.0f;
    }
    return motor->motor_data.solved_data[1];
}

static void Initialize3508PosTargetsIfNeeded(void)
{
    if (g_RcDebug3508PosTargetInitialized != 0U)
    {
        return;
    }

#if ENABLE_RC_DEBUG_3508_STORE_BOTH
    if ((IsMotorHandleReady(RM_3508_STORE_LEFT) == 0U) ||
        (IsMotorHandleReady(RM_3508_STORE_RIGHT) == 0U))
    {
        return;
    }

    g_RcDebug3508LeftTargetPosDeg = GetMotorRelativePosDeg(RM_3508_STORE_LEFT);
    g_RcDebug3508RightTargetPosDeg = GetMotorRelativePosDeg(RM_3508_STORE_RIGHT);
    g_RcDebug3508TargetPosDeg = 0.5f * ((-g_RcDebug3508LeftTargetPosDeg) + g_RcDebug3508RightTargetPosDeg);
#else
    if (IsMotorHandleReady(g_RcDebug3508PrimaryMotorInfo.motor_id) == 0U)
    {
        return;
    }

    g_RcDebug3508TargetPosDeg = GetMotorRelativePosDeg(g_RcDebug3508PrimaryMotorInfo.motor_id);
#if ENABLE_RC_DEBUG_3508_STORE_LEFT
    g_RcDebug3508LeftTargetPosDeg = g_RcDebug3508TargetPosDeg;
    g_RcDebug3508RightTargetPosDeg = 0.0f;
#elif ENABLE_RC_DEBUG_3508_STORE_RIGHT
    g_RcDebug3508RightTargetPosDeg = g_RcDebug3508TargetPosDeg;
    g_RcDebug3508LeftTargetPosDeg = 0.0f;
#endif
#endif

    g_RcDebug3508PosTargetInitialized = 1U;
}

static void Update3508CascadeTargets(float stick_norm, bool ibus_updated)
{
    Initialize3508PosTargetsIfNeeded();
    if (g_RcDebug3508PosTargetInitialized == 0U)
    {
        return;
    }

    g_RcDebug3508DesiredPosStepDeg = stick_norm * LOAD3508_POS_STEP_CMD_MAX_DEG_PER_FRAME;
    if (ibus_updated)
    {
        float tracking_error_pos = g_RcDebug3508DesiredPosStepDeg - g_RcDebug3508StepPosDeg;
        if (tracking_error_pos < 0.0f)
        {
            tracking_error_pos = -tracking_error_pos;
        }
        g_RcDebug3508PosStepPerFrame = ComputeAdaptiveStep(g_RcDebug3508DesiredPosStepDeg,
                                                           g_prev_desired_pos_step_deg,
                                                           tracking_error_pos,
                                                           LOAD3508_POS_STEP_CMD_MAX_DEG_PER_FRAME,
                                                           LOAD3508_POS_STEP_MIN,
                                                           LOAD3508_POS_STEP_MAX,
                                                           LOAD3508_POS_STEP_SENSITIVITY);
    }

    g_RcDebug3508StepPosDeg = StepToward(g_RcDebug3508StepPosDeg,
                                         g_RcDebug3508DesiredPosStepDeg,
                                         g_RcDebug3508PosStepPerFrame);
    g_RcDebug3508StepPosDeg = ClampFloat(g_RcDebug3508StepPosDeg,
                                         -LOAD3508_POS_STEP_CMD_MAX_DEG_PER_FRAME,
                                         LOAD3508_POS_STEP_CMD_MAX_DEG_PER_FRAME);

#if ENABLE_RC_DEBUG_3508_STORE_BOTH
    g_RcDebug3508TargetPosDeg += g_RcDebug3508StepPosDeg;
    g_RcDebug3508TargetPosDeg = ClampFloat(g_RcDebug3508TargetPosDeg,
                                           LOAD3508_BOTH_TARGET_MIN_DEG,
                                           LOAD3508_BOTH_TARGET_MAX_DEG);

    g_RcDebug3508SyncErrorDeg = g_RcDebug3508LeftPosDeg + g_RcDebug3508RightPosDeg;
    g_RcDebug3508SyncPidOutputDeg = PID_Calculate(&g_RcDebug3508SyncPid, 0.0f, g_RcDebug3508SyncErrorDeg);
    g_RcDebug3508SyncPidOutputDeg = ClampFloat(g_RcDebug3508SyncPidOutputDeg,
                                               -LOAD3508_SYNC_PID_MAX_OUT,
                                               LOAD3508_SYNC_PID_MAX_OUT);

    /* 左侧目标为镜像负值，右侧目标为正值；同步 PID 输出在此基础上做差分补偿。 */
    g_RcDebug3508LeftTargetPosDeg = ClampFloat(-g_RcDebug3508TargetPosDeg + g_RcDebug3508SyncPidOutputDeg,
                                               -LimitStore,
                                               LeftStoreTop);
    g_RcDebug3508RightTargetPosDeg = ClampFloat(g_RcDebug3508TargetPosDeg - g_RcDebug3508SyncPidOutputDeg,
                                                RightStoreTop,
                                                LimitStore);
#elif ENABLE_RC_DEBUG_3508_STORE_LEFT
    g_RcDebug3508TargetPosDeg += g_RcDebug3508StepPosDeg;
    g_RcDebug3508TargetPosDeg = ClampFloat(g_RcDebug3508TargetPosDeg,
                                           g_RcDebug3508PrimaryMotorInfo.min_pos_deg,
                                           g_RcDebug3508PrimaryMotorInfo.max_pos_deg);
    g_RcDebug3508LeftTargetPosDeg = g_RcDebug3508TargetPosDeg;
#elif ENABLE_RC_DEBUG_3508_STORE_RIGHT
    g_RcDebug3508TargetPosDeg += g_RcDebug3508StepPosDeg;
    g_RcDebug3508TargetPosDeg = ClampFloat(g_RcDebug3508TargetPosDeg,
                                           g_RcDebug3508PrimaryMotorInfo.min_pos_deg,
                                           g_RcDebug3508PrimaryMotorInfo.max_pos_deg);
    g_RcDebug3508RightTargetPosDeg = g_RcDebug3508TargetPosDeg;
#endif
}

static float NormalizeCenterStickWithConfig(int16_t raw, int16_t center, float range, float deadzone)
{
    float norm = ((float)raw - (float)center) / range;

    if (norm > 1.0f)
    {
        norm = 1.0f;
    }
    else if (norm < -1.0f)
    {
        norm = -1.0f;
    }

    if ((norm > -deadzone) && (norm < deadzone))
    {
        norm = 0.0f;
    }

    return norm;
}

static float ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static float StepToward(float current, float target, float step)
{
    if (step <= 0.0f)
    {
        return target;
    }

    if (current < (target - step))
    {
        return current + step;
    }
    if (current > (target + step))
    {
        return current - step;
    }
    return target;
}

static float UpdateSpeedTargetWithStep(float desired_speed_rpm, float *step_speed_rpm, float *step_value_per_frame,
                                       float *prev_desired_speed_rpm, float min_speed_rpm, float max_speed_rpm,
                                       float step_min, float step_max, float step_sensitivity, bool ibus_updated)
{
    float limit_abs = max_speed_rpm;
    if (-min_speed_rpm > limit_abs)
    {
        limit_abs = -min_speed_rpm;
    }

    desired_speed_rpm = ClampFloat(desired_speed_rpm, min_speed_rpm, max_speed_rpm);

    if (ibus_updated)
    {
        float tracking_error = desired_speed_rpm - *step_speed_rpm;
        if (tracking_error < 0.0f)
        {
            tracking_error = -tracking_error;
        }

        *step_value_per_frame = ComputeAdaptiveStep(desired_speed_rpm,
                                                    *prev_desired_speed_rpm,
                                                    tracking_error,
                                                    limit_abs,
                                                    step_min,
                                                    step_max,
                                                    step_sensitivity);
        *prev_desired_speed_rpm = desired_speed_rpm;
    }

    *step_speed_rpm = StepToward(*step_speed_rpm, desired_speed_rpm, *step_value_per_frame);
    *step_speed_rpm = ClampFloat(*step_speed_rpm, min_speed_rpm, max_speed_rpm);

    return *step_speed_rpm;
}

static void UpdatePosTargetDurationTest(float target_pos_deg)
{
    if (!isfinite(target_pos_deg))
    {
        return;
    }

    float now_ms = DWT_GetTimeline_ms();

    if (RmMotorPosTargetTestState == 0U)
    {
        RmMotorPosTargetTestCurrentMs = 0.0f;
        if ((RmMotorPosTargetTestPrevDeg <= LOAD3508_POS_TEST_START_DEG) &&
            (target_pos_deg > LOAD3508_POS_TEST_START_DEG))
        {
            RmMotorPosTargetTestStartTimestampMs = now_ms;
            RmMotorPosTargetTestCurrentMs = 0.0f;
            RmMotorPosTargetTestState = 1U;
        }
    }
    else if (RmMotorPosTargetTestState == 1U)
    {
        RmMotorPosTargetTestCurrentMs = now_ms - RmMotorPosTargetTestStartTimestampMs;
        if (target_pos_deg >= LOAD3508_POS_TEST_END_DEG)
        {
            RmMotorPosTargetTestLastMs = RmMotorPosTargetTestCurrentMs;
            RmMotorPosTargetTestRoundCount++;
            RmMotorPosTargetTestState = 2U;
        }
    }
    else
    {
        RmMotorPosTargetTestCurrentMs = RmMotorPosTargetTestLastMs;
        if (target_pos_deg <= LOAD3508_POS_TEST_START_DEG)
        {
            RmMotorPosTargetTestState = 0U;
        }
    }

    RmMotorPosTargetTestPrevDeg = target_pos_deg;
}

static float ComputeAdaptiveStep(float desired_value, float prev_desired_value, float tracking_error,
                                 float value_limit_abs, float step_min, float step_max, float step_sensitivity)
{
    float input_delta = desired_value - prev_desired_value;
    if (input_delta < 0.0f)
    {
        input_delta = -input_delta;
    }

    float blended_delta = input_delta + 0.35f * tracking_error;
    float ratio = 0.0f;
    if (value_limit_abs > 1e-6f)
    {
        ratio = (blended_delta * step_sensitivity) / value_limit_abs;
    }
    if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }
    else if (ratio < 0.0f)
    {
        ratio = 0.0f;
    }

    return step_min + (step_max - step_min) * ratio;
}

#if RC_INPUT_USE_UKF
static void UKF1D_Reset(UKF1D_t *ukf, float init_x)
{
    if (ukf == NULL)
    {
        return;
    }

    ukf->x = init_x;
    ukf->P = 1.0f;
    ukf->q = RC_INPUT_UKF_Q;
    ukf->r = RC_INPUT_UKF_R;
    ukf->alpha = RC_INPUT_UKF_ALPHA;
    ukf->beta = RC_INPUT_UKF_BETA;
    ukf->kappa = RC_INPUT_UKF_KAPPA;
    ukf->initialized = 1U;
}

static float UKF1D_Update(UKF1D_t *ukf, float z)
{
    if (ukf == NULL)
    {
        return z;
    }

    if (ukf->initialized == 0U)
    {
        UKF1D_Reset(ukf, z);
    }

    const float n = 1.0f;
    float lambda = ukf->alpha * ukf->alpha * (n + ukf->kappa) - n;
    float c = n + lambda;
    if (c < 1e-6f)
    {
        c = 1e-6f;
    }

    float wm0 = lambda / c;
    float wc0 = wm0 + (1.0f - ukf->alpha * ukf->alpha + ukf->beta);
    float wi = 1.0f / (2.0f * c);

    float p_safe = ukf->P;
    if (p_safe < 1e-6f)
    {
        p_safe = 1e-6f;
    }

    float sqrt_cp = sqrtf(c * p_safe);
    float x_sigma0 = ukf->x;
    float x_sigma1 = ukf->x + sqrt_cp;
    float x_sigma2 = ukf->x - sqrt_cp;

    float x_pred = wm0 * x_sigma0 + wi * x_sigma1 + wi * x_sigma2;
    float p_pred = wc0 * (x_sigma0 - x_pred) * (x_sigma0 - x_pred) +
                   wi * (x_sigma1 - x_pred) * (x_sigma1 - x_pred) +
                   wi * (x_sigma2 - x_pred) * (x_sigma2 - x_pred) + ukf->q;

    if (p_pred < 1e-6f)
    {
        p_pred = 1e-6f;
    }

    float z_pred = x_pred;
    float s = p_pred + ukf->r;
    if (s < 1e-6f)
    {
        s = 1e-6f;
    }

    float pxz = p_pred;
    float k = pxz / s;

    ukf->x = x_pred + k * (z - z_pred);
    ukf->P = p_pred - k * s * k;
    if (ukf->P < 1e-6f)
    {
        ukf->P = 1e-6f;
    }

    return ukf->x;
}
#endif
