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

float RmMotorTargetSpeedData = 0.0f;
float RmMotorTargetPosData = 0.0f;
float RmMotor2006TargetPosData = 0.0f;
float RmMotor2006SpeedData = 0.0f;
float RcLoad3508DesiredSpeedRpm = 0.0f;
float RcLoad3508DesiredPosStepDeg = 0.0f;
float RcLoad3508UkfSpeedRpm = 0.0f;
float RcLoad3508StepSpeedRpm = 0.0f;
float RcLoad3508StepValuePerFrame = LOAD3508_SPEED_STEP_MIN;
float RcLoad3508StepPosStepDeg = 0.0f;
float RcLoad3508PosStepValuePerFrame = LOAD3508_POS_STEP_MIN;
float RcLoad3508UkfCovP = 0.0f;
int16_t RcLoad3508Raw = 0;
int8_t RcSWBState = 0;
uint8_t RcLoad3508PosTargetInitialized = 0U;
float RcLoad2006DesiredPosStepDeg = 0.0f;
float RcLoad2006PosStepValuePerFrame = LOAD2006_SPEED_STEP_MIN;
int16_t RcLoad2006Raw = 0;

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
static uint8_t g_offset_applied = 0U;

#if RC_INPUT_USE_UKF
static UKF1D_t g_RcInputUkf = {0};
#endif

static float NormalizeCenterStick(int16_t raw);
static float ClampFloat(float value, float min_value, float max_value);
static float StepToward(float current, float target, float step);
static float ComputeAdaptiveStep(float desired_value, float prev_desired_value, float tracking_error,
                                 float value_limit_abs, float step_min, float step_max, float step_sensitivity);
static float UpdateSpeedTargetWithStep(float desired_speed_rpm, float *step_speed_rpm, float *step_value_per_frame,
                                       float *prev_desired_speed_rpm, float min_speed_rpm, float max_speed_rpm,
                                       float step_min, float step_max, float step_sensitivity, bool ibus_updated);
static void UpdatePosTargetDurationTest(float target_pos_deg);
#if RC_INPUT_USE_UKF
static void UKF1D_Reset(UKF1D_t *ukf, float init_x);
static float UKF1D_Update(UKF1D_t *ukf, float z);
#endif
static void RC3508ResetStates(void);
static void RC2006ResetStates(void);

void IA6BTask_Init(void)
{
    g_last_ibus_tick = 0U;
    g_prev_desired_speed_rpm_3508 = 0.0f;
    g_prev_desired_pos_step_deg = 0.0f;
    RcLoad3508PosTargetInitialized = 0U;

    RcLoad3508StepValuePerFrame = LOAD3508_SPEED_STEP_MIN;
    RcLoad3508PosStepValuePerFrame = LOAD3508_POS_STEP_MIN;
    RcLoad2006PosStepValuePerFrame = LOAD2006_SPEED_STEP_MIN;
    RmMotor2006TargetPosData = 0.0f;

#if ENABLE_RC_DEBUG_3508
    if (g_offset_applied == 0U)
    {
        float m_offset_angle = MotorManager.MotorList[RM_3508_GRIPPER - 1].motor_data.offset_ecd_angle;
        RmMotorTargetPosData += m_offset_angle;
        g_offset_applied = 1U;
    }
#endif

    RC3508ResetStates();
    RC2006ResetStates();
}

void IA6BTask_ProcessAndControl(void)
{
#if ENABLE_DEFAULTTASK_RC_DEBUG
    bool ibus_updated = false;
    bool rc_online = false;
    bool rc_enabled_3508 = false;
    bool rc_enabled_2006 = false;

    if (IA6B_ProcessIbusPacket(BSP_UART6))
    {
        ibus_updated = true;
        g_last_ibus_tick = HAL_GetTick();
    }

    rc_online = (g_last_ibus_tick != 0U) &&
                ((HAL_GetTick() - g_last_ibus_tick) <= IBUS_LOST_TIMEOUT_MS);

    RcSWBState = Channel[4];
    rc_enabled_3508 = rc_online && (RcSWBState == 0);
    rc_enabled_2006 = rc_online;

#if ENABLE_RC_DEBUG_3508
    RcLoad3508DesiredSpeedRpm = 0.0f;
    RcLoad3508DesiredPosStepDeg = 0.0f;
    RcLoad3508UkfSpeedRpm = 0.0f;
    RcLoad3508Raw = RawChannel[3];
#if RC_INPUT_USE_UKF
    RcLoad3508UkfCovP = g_RcInputUkf.P;
#else
    RcLoad3508UkfCovP = 0.0f;
#endif

    if (rc_enabled_3508)
    {
        float stick_norm = 0.0f;
        float filtered_speed_rpm = 0.0f;
        if (RcLoad3508Raw >= 900 && RcLoad3508Raw <= 2100)
        {
            stick_norm = NormalizeCenterStick(RcLoad3508Raw);
            RcLoad3508DesiredSpeedRpm = stick_norm * LOAD3508_MAX_SPEED_RPM;
            RcLoad3508DesiredSpeedRpm = ClampFloat(RcLoad3508DesiredSpeedRpm, LOAD3508_MIN_SPEED_RPM, LOAD3508_MAX_SPEED_RPM);
        }

#if RC_INPUT_USE_UKF
        RcLoad3508UkfSpeedRpm = UKF1D_Update(&g_RcInputUkf, RcLoad3508DesiredSpeedRpm);
        RcLoad3508UkfCovP = g_RcInputUkf.P;
        filtered_speed_rpm = RcLoad3508UkfSpeedRpm;
#else
        RcLoad3508UkfSpeedRpm = RcLoad3508DesiredSpeedRpm;
        RcLoad3508UkfCovP = 0.0f;
        filtered_speed_rpm = RcLoad3508UkfSpeedRpm;
#endif

        filtered_speed_rpm = ClampFloat(filtered_speed_rpm, LOAD3508_MIN_SPEED_RPM, LOAD3508_MAX_SPEED_RPM);
        RmMotorTargetSpeedData = UpdateSpeedTargetWithStep(filtered_speed_rpm,
                                                           &RcLoad3508StepSpeedRpm,
                                                           &RcLoad3508StepValuePerFrame,
                                                           &g_prev_desired_speed_rpm_3508,
                                                           LOAD3508_MIN_SPEED_RPM,
                                                           LOAD3508_MAX_SPEED_RPM,
                                                           LOAD3508_SPEED_STEP_MIN,
                                                           LOAD3508_SPEED_STEP_MAX,
                                                           LOAD3508_SPEED_STEP_SENSITIVITY,
                                                           ibus_updated);

#if LOAD3508_OUTPUT_MODE == LOAD3508_OUTPUT_CASCADE_POS
        if (!RcLoad3508PosTargetInitialized)
        {
            MotorTypeDef *motor = Motor_GetHandle(RM_3508_GRIPPER);
            if (motor != NULL)
            {
                RmMotorTargetPosData = motor->motor_data.solved_data[3];
                RcLoad3508PosTargetInitialized = 1U;
            }
        }
        if (RcLoad3508PosTargetInitialized)
        {
            RcLoad3508DesiredPosStepDeg = stick_norm * LOAD3508_POS_STEP_CMD_MAX_DEG_PER_FRAME;
            if (ibus_updated)
            {
                float tracking_error_pos = RcLoad3508DesiredPosStepDeg - RcLoad3508StepPosStepDeg;
                if (tracking_error_pos < 0.0f)
                {
                    tracking_error_pos = -tracking_error_pos;
                }
                RcLoad3508PosStepValuePerFrame = ComputeAdaptiveStep(RcLoad3508DesiredPosStepDeg,
                                                                     g_prev_desired_pos_step_deg,
                                                                     tracking_error_pos,
                                                                     LOAD3508_POS_STEP_CMD_MAX_DEG_PER_FRAME,
                                                                     LOAD3508_POS_STEP_MIN,
                                                                     LOAD3508_POS_STEP_MAX,
                                                                     LOAD3508_POS_STEP_SENSITIVITY);
            }

            RcLoad3508StepPosStepDeg = StepToward(RcLoad3508StepPosStepDeg,
                                                  RcLoad3508DesiredPosStepDeg,
                                                  RcLoad3508PosStepValuePerFrame);
            if (RcLoad3508StepPosStepDeg < -LOAD3508_POS_STEP_CMD_MAX_DEG_PER_FRAME)
            {
                RcLoad3508StepPosStepDeg = -LOAD3508_POS_STEP_CMD_MAX_DEG_PER_FRAME;
            }
            else if (RcLoad3508StepPosStepDeg > LOAD3508_POS_STEP_CMD_MAX_DEG_PER_FRAME)
            {
                RcLoad3508StepPosStepDeg = LOAD3508_POS_STEP_CMD_MAX_DEG_PER_FRAME;
            }

            RmMotorTargetPosData += RcLoad3508StepPosStepDeg;
            if (RmMotorTargetPosData < LOAD3508_POS_TARGET_MIN_DEG)
            {
                RmMotorTargetPosData = LOAD3508_POS_TARGET_MIN_DEG;
            }
            else if (RmMotorTargetPosData > LOAD3508_POS_TARGET_MAX_DEG)
            {
                RmMotorTargetPosData = LOAD3508_POS_TARGET_MAX_DEG;
            }
        }
#endif
        if (ibus_updated)
        {
            g_prev_desired_pos_step_deg = RcLoad3508DesiredPosStepDeg;
        }
    }
    else
    {
        RC3508ResetStates();
    }
#endif

#if ENABLE_RC_DEBUG_2006
    RcLoad2006DesiredPosStepDeg = 0.0f;
    RcLoad2006Raw = RawChannel[LOAD2006_STEP_RAW_CHANNEL_INDEX];
    if (rc_enabled_2006)
    {
        int16_t rc_dir_raw = RawChannel[LOAD2006_DIR_RAW_CHANNEL_INDEX];

        if (RcLoad2006Raw > LOAD2006_STEP_RAW_IDLE_MAX)
        {
            float ratio = (float)(RcLoad2006Raw - LOAD2006_STEP_RAW_IDLE_MAX) /
                          (float)(LOAD2006_STEP_RAW_MAX - LOAD2006_STEP_RAW_IDLE_MAX);
            ratio = ClampFloat(ratio, 0.0f, 1.0f);
            RcLoad2006PosStepValuePerFrame = LOAD2006_SPEED_STEP_MIN +
                                             (LOAD2006_SPEED_STEP_MAX - LOAD2006_SPEED_STEP_MIN) * ratio;
        }
        else
        {
            RcLoad2006PosStepValuePerFrame = 0.0f;
        }

        if ((rc_dir_raw >= LOAD2006_DIR_INC_RAW_MIN) && (rc_dir_raw <= LOAD2006_DIR_INC_RAW_MAX))
        {
            RcLoad2006DesiredPosStepDeg = RcLoad2006PosStepValuePerFrame;
        }
        else if (rc_dir_raw >= LOAD2006_DIR_DEC_RAW_MIN)
        {
            RcLoad2006DesiredPosStepDeg = -RcLoad2006PosStepValuePerFrame;
        }

        RmMotor2006TargetPosData += RcLoad2006DesiredPosStepDeg;
        RmMotor2006TargetPosData = ClampFloat(RmMotor2006TargetPosData,
                                              LOAD2006_POS_TARGET_MIN_DEG,
                                              LOAD2006_POS_TARGET_MAX_DEG);
    }
    else
    {
        RC2006ResetStates();
    }
#endif

#if ENABLE_RC_DEBUG_3508
#if LOAD3508_OUTPUT_MODE == LOAD3508_OUTPUT_CASCADE_POS
    UpdatePosTargetDurationTest(RmMotorTargetPosData);
#endif
#endif

#else
    // 保持兼容：即使外层误调用，本函数也不输出控制
    RC3508ResetStates();
    RC2006ResetStates();
#endif
}

static void RC3508ResetStates(void)
{
#if RC_INPUT_USE_UKF
    UKF1D_Reset(&g_RcInputUkf, 0.0f);
#endif
    RcLoad3508StepSpeedRpm = 0.0f;
    RcLoad3508StepValuePerFrame = LOAD3508_SPEED_STEP_MIN;
    RcLoad3508DesiredPosStepDeg = 0.0f;
    RcLoad3508StepPosStepDeg = 0.0f;
    RcLoad3508PosStepValuePerFrame = LOAD3508_POS_STEP_MIN;
    RmMotorTargetSpeedData = 0.0f;
    RcLoad3508UkfSpeedRpm = 0.0f;
#if RC_INPUT_USE_UKF
    RcLoad3508UkfCovP = g_RcInputUkf.P;
#else
    RcLoad3508UkfCovP = 0.0f;
#endif
    g_prev_desired_speed_rpm_3508 = 0.0f;
    g_prev_desired_pos_step_deg = 0.0f;
    RcLoad3508PosTargetInitialized = 0U;
}

static void RC2006ResetStates(void)
{
    RcLoad2006DesiredPosStepDeg = 0.0f;
    RcLoad2006PosStepValuePerFrame = 0.0f;
}

static float NormalizeCenterStick(int16_t raw)
{
    float norm = ((float)raw - (float)LOAD3508_STICK_CENTER) / LOAD3508_STICK_RANGE;

    if (norm > 1.0f)
    {
        norm = 1.0f;
    }
    else if (norm < -1.0f)
    {
        norm = -1.0f;
    }

    if ((norm > -LOAD3508_STICK_DEADZONE) && (norm < LOAD3508_STICK_DEADZONE))
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
