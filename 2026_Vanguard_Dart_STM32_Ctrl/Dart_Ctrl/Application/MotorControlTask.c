#include "./MotorControlTask.h"
#include "../User/inc/motor_algrothim.h"
#include "bsp_dwt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stream_buffer.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

extern MotorManager_t MotorManager;
float MotorData = 0.0f;
float gripper_offset = 0.0f;
float trigger_offset = 0.0f;
float target_loc = 0.0f;
float Load3508CurrentFilteredData = 0.0f;
float Load3508CurrentAbsFilteredData = 0.0f;
float Load3508StillOverCurrentData = 0.0f;
volatile uint8_t Load3508OverCurrentProtected = 0U;

static float g_GripperTarget = 0.0f;
static float g_GripperPrevTarget = 0.0f;
static float g_TriggerTarget = 0.0f;
static float g_Yaw6020Target = 0.0f;

static StaticSemaphore_t g_x3508MtxBuf;
static SemaphoreHandle_t g_x3508Mtx = NULL;

static StaticSemaphore_t g_x2006MtxBuf;
static SemaphoreHandle_t g_x2006Mtx = NULL;

static StaticSemaphore_t g_x6020MtxBuf;
static SemaphoreHandle_t g_x6020Mtx = NULL;

#define LOAD_MOTOR_CMD_MAGIC 0x4C443508UL
#define LOAD_MOTOR_CMD_SIGNATURE_KEY 0xA53C3508UL
#define LOAD_MOTOR_CMD_STREAM_DEPTH 8U
#define LOAD_MOTOR_CMD_MAX_AGE_MS 100U
#define LOAD_MOTOR_DEFAULT_HOLD_MS 500U

typedef struct
{
    uint32_t magic;
    uint32_t signature;
    uint32_t seq;
    uint32_t timestamp_ms;
    uint32_t max_age_ms;
    uint32_t owner_hold_ms;
    uint8_t owner;
    uint8_t priority;
    uint16_t reserved;
    float target_pos_deg;
} LoadMotorCommand_t;

static StaticStreamBuffer_t g_x3508CmdStreamBuffer;
static uint8_t g_uc3508CmdStreamStorage[LOAD_MOTOR_CMD_STREAM_DEPTH * sizeof(LoadMotorCommand_t)];
static StreamBufferHandle_t g_x3508CmdStream = NULL;
static volatile uint32_t g_ul3508CmdSeq = 0U;
static LoadMotorOwner_e g_Active3508Owner = LOAD_MOTOR_OWNER_NONE;
static uint8_t g_Active3508Priority = 0U;
static uint32_t g_Active3508OwnerExpireMs = 0U;

static MotorTrapPosProfile_t g_LoadTrapProfile = {0};
static uint32_t g_LoadTrapCntLast = 0U;
static float g_GripperCurrentFilteredA = 0.0f;
static uint8_t g_GripperCurrentFilterReady = 0U;
static uint32_t g_GripperLastTargetChangeMs = 0U;
static uint32_t g_GripperOverCurrentStartMs = 0U;
static uint32_t g_GripperStallOverCurrentStartMs = 0U;
static uint32_t g_GripperStallSampleMs = 0U;
static float g_GripperStallSamplePosDeg = 0.0f;
static bool g_GripperOverCurrentReturning = false;

typedef struct
{
    can_motor_cfg motor_cfg;
    float target;
    float filtered_current_a;
    uint8_t filter_ready;
    uint32_t last_target_change_ms;
    uint32_t over_current_start_ms;
    uint32_t stall_over_current_start_ms;
    uint32_t stall_sample_ms;
    float stall_sample_pos_deg;
    bool ctrl_enabled;
    bool use_scurve;
    bool protected_flag;
    MotorTrapPosProfile_t trap_profile;
    uint32_t trap_cnt_last;
    uint8_t trap_initialized;
} StoreMotorCtrlState_t;

static StaticSemaphore_t g_xStore3508MtxBuf;
static SemaphoreHandle_t g_xStore3508Mtx = NULL;
static StoreMotorCtrlState_t g_LeftStoreMotor = {.motor_cfg = RM_3508_STORE_LEFT};
static StoreMotorCtrlState_t g_RightStoreMotor = {.motor_cfg = RM_3508_STORE_RIGHT};

static volatile bool g_b3508CtrlEnabled = false;
static volatile bool g_b2006CtrlEnabled = false;
static volatile bool g_b6020CtrlEnabled = false;

static void Motor3508_SetTargetAndRemember(float target);
static void Motor3508_CheckOverCurrent(uint32_t now_ms, float target_pos_deg);
static StoreMotorCtrlState_t *StoreMotor_GetState(can_motor_cfg motor_cfg);
static void StoreMotor_ClearProtectionState(StoreMotorCtrlState_t *state);
static void StoreMotor_ResetState(StoreMotorCtrlState_t *state);
static void StoreMotor_ResetTrap(StoreMotorCtrlState_t *state, float current_pos_deg);
static float StoreMotor_GetRawAngle(const StoreMotorCtrlState_t *state);
static float StoreMotor_GetRelativeAngle(const StoreMotorCtrlState_t *state);
static void StoreMotor_CheckProtection(StoreMotorCtrlState_t *state, uint32_t now_ms, float target_pos_deg);
static void StoreMotor_RunControl(StoreMotorCtrlState_t *state);

#if 0
static inline float LoadMotor_GetRawAngleInternal(void)
{
    return Motor_GetTotalAngle(RM_3508_GRIPPER);
    return Motor_GetTotalAngle(RM_3508_GRIPPER);
    // 业务层统一使用相对零点角度，避免夹爪 3508 的目标坐标系不一致。
    return Motor_GetTotalAngle(RM_3508_GRIPPER);
    // 对业务层统一使用相对零点角度，避免和 RM 位置环坐标系不一致
    return Motor_GetTotalAngle(RM_3508_GRIPPER);
}

#endif
#if 0
static inline float LoadMotor_GetRawAngleInternal(void)
{
    // 业务层统一读取夹爪 M3508 的相对零点角度，避免目标坐标系不一致。
    return Motor_GetTotalAngle(RM_3508_GRIPPER);
}

#endif
static inline float LoadMotor_GetRawAngleInternal(void)
{
    return Motor_GetTotalAngle(RM_3508_GRIPPER);
    // 业务层统一读取夹爪 M3508 的相对零点角度，避免目标坐标系不一致。
    return Motor_GetTotalAngle(RM_3508_GRIPPER);
}

static bool LoadMotor_TimeReached(uint32_t now_ms, uint32_t target_ms)
{
    return ((int32_t)(now_ms - target_ms) >= 0);
}

static uint32_t LoadMotor_FloatBits(float value)
{
    uint32_t bits = 0U;
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

static uint32_t LoadMotor_CalcSignature(const LoadMotorCommand_t *cmd)
{
    if (cmd == NULL)
    {
        return 0U;
    }

    return LOAD_MOTOR_CMD_SIGNATURE_KEY ^
           cmd->magic ^
           cmd->seq ^
           cmd->timestamp_ms ^
           cmd->max_age_ms ^
           cmd->owner_hold_ms ^
           ((uint32_t)cmd->owner << 24) ^
           ((uint32_t)cmd->priority << 16) ^
           LoadMotor_FloatBits(cmd->target_pos_deg);
}

static bool LoadMotor_CommandIsValid(const LoadMotorCommand_t *cmd, uint32_t now_ms)
{
    if (cmd == NULL ||
        cmd->magic != LOAD_MOTOR_CMD_MAGIC ||
        cmd->signature != LoadMotor_CalcSignature(cmd) ||
        !isfinite(cmd->target_pos_deg))
    {
        return false;
    }

    if (cmd->max_age_ms > 0U &&
        (uint32_t)(now_ms - cmd->timestamp_ms) > cmd->max_age_ms)
    {
        return false;
    }

    return true;
}

static bool LoadMotor_ShouldAcceptCommand(const LoadMotorCommand_t *cmd, uint32_t now_ms)
{
    bool accepted;

    if (cmd == NULL)
    {
        return false;
    }

    taskENTER_CRITICAL();
    if (g_Active3508Owner != LOAD_MOTOR_OWNER_NONE &&
        LoadMotor_TimeReached(now_ms, g_Active3508OwnerExpireMs))
    {
        g_Active3508Owner = LOAD_MOTOR_OWNER_NONE;
        g_Active3508Priority = 0U;
    }

    accepted = (g_Active3508Owner == LOAD_MOTOR_OWNER_NONE ||
                g_Active3508Owner == (LoadMotorOwner_e)cmd->owner ||
                cmd->priority >= g_Active3508Priority);
    taskEXIT_CRITICAL();

    return accepted;
}

static void LoadMotor_AcceptCommand(const LoadMotorCommand_t *cmd, uint32_t now_ms)
{
    uint32_t hold_ms;
    MotorTypeDef *motor = Motor_GetHandleFast(RM_3508_GRIPPER);
    const MotorTrapConfig_t *trap_cfg = &motor->trap_config;

    if (cmd == NULL)
    {
        return;
    }

    hold_ms = (cmd->owner_hold_ms == 0U) ? LOAD_MOTOR_DEFAULT_HOLD_MS : cmd->owner_hold_ms;
    Motor3508_SetTargetAndRemember(cmd->target_pos_deg);
    if (trap_cfg->registered != 0U && trap_cfg->resync_on_target_change != 0U)
    {
        MotorData = LoadMotor_GetRawAngleInternal();
        Motor_TrapPos_Resync(&g_LoadTrapProfile, MotorData);
    }
    taskENTER_CRITICAL();
    g_Active3508Owner = (LoadMotorOwner_e)cmd->owner;
    g_Active3508Priority = cmd->priority;
    g_Active3508OwnerExpireMs = now_ms + hold_ms;
    taskEXIT_CRITICAL();
}

static void LoadMotor_ProcessCommandStream(void)
{
    LoadMotorCommand_t cmd;
    size_t received;
    uint32_t now_ms = HAL_GetTick();

    if (g_x3508CmdStream == NULL)
    {
        return;
    }

    while ((received = xStreamBufferReceive(g_x3508CmdStream, &cmd, sizeof(cmd), 0)) > 0U)
    {
        if (received != sizeof(cmd))
        {
            (void)xStreamBufferReset(g_x3508CmdStream);
            break;
        }

        now_ms = HAL_GetTick();
        if (LoadMotor_CommandIsValid(&cmd, now_ms) &&
            LoadMotor_ShouldAcceptCommand(&cmd, now_ms))
        {
            LoadMotor_AcceptCommand(&cmd, now_ms);
        }
    }
}

static inline void LoadMotor_RunTrapTo(float target_pos_deg)
{
    MotorTypeDef *motor = Motor_GetHandleFast(RM_3508_GRIPPER);
    const MotorTrapConfig_t *trap_cfg = &motor->trap_config;
    float dt_s;
    float cmd_pos;

    if (trap_cfg->registered == 0U)
    {
        // 没给夹爪注册 S 型规划时，直接走原有位置 PID。
        RmMotorPID_Calc(RM_3508_GRIPPER, target_pos_deg);
        return;
    }

    g_LoadTrapProfile.target_pos = target_pos_deg;
    dt_s = DWT_GetDeltaT(&g_LoadTrapCntLast);
    if (!isfinite(dt_s) || dt_s <= 0.0f)
    {
        dt_s = 0.001f;
    }
    else if (dt_s > 0.02f)
    {
        dt_s = 0.02f;
    }

    cmd_pos = Motor_TrapPos_Update(&g_LoadTrapProfile, dt_s);
    RmMotorPID_Calc(RM_3508_GRIPPER, cmd_pos);
}

void MotorControl_Init(void)
{
    if (g_x3508Mtx == NULL)
    {
        g_x3508Mtx = xSemaphoreCreateMutexStatic(&g_x3508MtxBuf);
    }
    if (g_x2006Mtx == NULL)
    {
        g_x2006Mtx = xSemaphoreCreateMutexStatic(&g_x2006MtxBuf);
    }
    if (g_x6020Mtx == NULL)
    {
        g_x6020Mtx = xSemaphoreCreateMutexStatic(&g_x6020MtxBuf);
    }
    if (g_xStore3508Mtx == NULL)
    {
        g_xStore3508Mtx = xSemaphoreCreateMutexStatic(&g_xStore3508MtxBuf);
    }
    if (g_x3508CmdStream == NULL)
    {
        g_x3508CmdStream = xStreamBufferCreateStatic(sizeof(g_uc3508CmdStreamStorage),
                                                     sizeof(LoadMotorCommand_t),
                                                     g_uc3508CmdStreamStorage,
                                                     &g_x3508CmdStreamBuffer);
    }

    g_GripperTarget = 0.0f;
    g_GripperPrevTarget = 0.0f;
    g_TriggerTarget = 0.0f;
    g_Yaw6020Target = 0.0f;
    target_loc = 0.0f;
    Load3508CurrentFilteredData = 0.0f;
    Load3508CurrentAbsFilteredData = 0.0f;
    Load3508StillOverCurrentData = 0.0f;
    Load3508OverCurrentProtected = 0U;
    g_GripperCurrentFilteredA = 0.0f;
    g_GripperCurrentFilterReady = 0U;
    g_GripperLastTargetChangeMs = 0U;
    g_GripperOverCurrentStartMs = 0U;
    g_GripperStallOverCurrentStartMs = 0U;
    g_GripperStallSampleMs = 0U;
    g_GripperStallSamplePosDeg = 0.0f;
    g_GripperOverCurrentReturning = false;
    g_b3508CtrlEnabled = false;
    g_b2006CtrlEnabled = false;
    g_b6020CtrlEnabled = false;
    g_Active3508Owner = LOAD_MOTOR_OWNER_NONE;
    g_Active3508Priority = 0U;
    g_Active3508OwnerExpireMs = 0U;
    StoreMotor_ResetState(&g_LeftStoreMotor);
    StoreMotor_ResetState(&g_RightStoreMotor);
}

void Motor3508_SetTarget(float target)
{
    if (g_x3508Mtx != NULL)
    {
        xSemaphoreTake(g_x3508Mtx, portMAX_DELAY);
        g_GripperTarget = target;
        xSemaphoreGive(g_x3508Mtx);
    }
    else
    {
        g_GripperTarget = target;
    }
    target_loc = target;
    g_GripperLastTargetChangeMs = HAL_GetTick();
}

static void Motor3508_SetTargetAndRemember(float target)
{
    if (g_x3508Mtx != NULL)
    {
        xSemaphoreTake(g_x3508Mtx, portMAX_DELAY);
        g_GripperPrevTarget = g_GripperTarget;
        g_GripperTarget = target;
        xSemaphoreGive(g_x3508Mtx);
    }
    else
    {
        g_GripperPrevTarget = g_GripperTarget;
        g_GripperTarget = target;
    }
    target_loc = target;
    g_GripperLastTargetChangeMs = HAL_GetTick();
    g_GripperOverCurrentReturning = false;
    g_GripperOverCurrentStartMs = 0U;
    g_GripperStallOverCurrentStartMs = 0U;
    g_GripperStallSampleMs = 0U;
    g_GripperStallSamplePosDeg = LoadMotor_GetRawAngleInternal();
    Load3508OverCurrentProtected = 0U;
}

float Motor3508_GetTarget(void)
{
    float target;

    if (g_x3508Mtx != NULL)
    {
        xSemaphoreTake(g_x3508Mtx, portMAX_DELAY);
        target = g_GripperTarget;
        xSemaphoreGive(g_x3508Mtx);
        return target;
    }

    return g_GripperTarget;
}

void Motor3508_EnableControl(bool enable)
{
    g_b3508CtrlEnabled = enable;
}

bool LoadMotor_IsOverCurrentProtected(void)
{
    return (Load3508OverCurrentProtected != 0U);
}

void LoadMotor_ClearOverCurrentProtection(void)
{
    taskENTER_CRITICAL();
    Load3508OverCurrentProtected = 0U;
    g_GripperOverCurrentReturning = false;
    g_GripperOverCurrentStartMs = 0U;
    g_GripperStallOverCurrentStartMs = 0U;
    g_GripperStallSampleMs = 0U;
    g_GripperStallSamplePosDeg = LoadMotor_GetRawAngleInternal();
    taskEXIT_CRITICAL();
}

static void Motor3508_CheckOverCurrent(uint32_t now_ms, float target_pos_deg)
{
    float current_a = Motor_GetCurrent(RM_3508_GRIPPER);
    float abs_filtered_current_a;
    float current_pos_deg;
    float pos_error_deg;
    float speed_rpm;
    bool near_target;
    bool stall_like = false;

    if (!isfinite(current_a))
    {
        g_GripperOverCurrentStartMs = 0U;
        g_GripperStallOverCurrentStartMs = 0U;
        return;
    }

    if (g_GripperCurrentFilterReady == 0U)
    {
        g_GripperCurrentFilteredA = current_a;
        g_GripperCurrentFilterReady = 1U;
        g_GripperStallSamplePosDeg = LoadMotor_GetRawAngleInternal();
        g_GripperStallSampleMs = now_ms;
        return;
    }

    g_GripperCurrentFilteredA +=
        (current_a - g_GripperCurrentFilteredA) * LOAD3508_OVERCURRENT_FILTER_ALPHA;
    abs_filtered_current_a = fabsf(g_GripperCurrentFilteredA);
    current_pos_deg = LoadMotor_GetRawAngleInternal();
    speed_rpm = Motor_GetSpeedRPM(RM_3508_GRIPPER);

    Load3508CurrentFilteredData = g_GripperCurrentFilteredA;
    Load3508CurrentAbsFilteredData = abs_filtered_current_a;
    Load3508StillOverCurrentData = abs_filtered_current_a;

    pos_error_deg = fabsf(target_pos_deg - current_pos_deg);
    near_target = (pos_error_deg <= MOTOR_DEAD_ZONE);

    if ((uint32_t)(now_ms - g_GripperLastTargetChangeMs) < LOAD3508_OVERCURRENT_TARGET_BLANK_MS)
    {
        g_GripperOverCurrentStartMs = 0U;
        g_GripperStallOverCurrentStartMs = 0U;
        g_GripperStallSamplePosDeg = current_pos_deg;
        g_GripperStallSampleMs = now_ms;
        return;
    }

    if (g_GripperStallSampleMs == 0U ||
        (uint32_t)(now_ms - g_GripperStallSampleMs) >= LOAD3508_STALL_POS_SAMPLE_MS)
    {
        float pos_delta_deg = fabsf(current_pos_deg - g_GripperStallSamplePosDeg);
        stall_like = (pos_delta_deg <= LOAD3508_STALL_POS_DELTA_DEG);
        g_GripperStallSamplePosDeg = current_pos_deg;
        g_GripperStallSampleMs = now_ms;
    }
    else
    {
        stall_like = (fabsf(speed_rpm) <= LOAD3508_STALL_SPEED_RPM);
    }

    if (near_target)
    {
        g_GripperStallOverCurrentStartMs = 0U;
    }
    else
    {
        g_GripperOverCurrentStartMs = 0U;
    }

    if (near_target && abs_filtered_current_a <= LOAD3508_STILL_OVERCURRENT_CLEAR_A)
    {
        g_GripperOverCurrentStartMs = 0U;
        return;
    }

    if (!near_target && abs_filtered_current_a <= LOAD3508_STALL_OVERCURRENT_CLEAR_A)
    {
        g_GripperStallOverCurrentStartMs = 0U;
        return;
    }

    if (near_target && abs_filtered_current_a > LOAD3508_STILL_OVERCURRENT_LIMIT_A)
    {
        if (g_GripperOverCurrentStartMs == 0U)
        {
            g_GripperOverCurrentStartMs = now_ms;
            return;
        }

        if (!g_GripperOverCurrentReturning &&
            (uint32_t)(now_ms - g_GripperOverCurrentStartMs) >= LOAD3508_STILL_OVERCURRENT_RETURN_MS)
        {
            Motor_TrapPos_Resync(&g_LoadTrapProfile, current_pos_deg);
            g_GripperOverCurrentReturning = true;
            Load3508OverCurrentProtected = 1U;
            g_GripperOverCurrentStartMs = 0U;
            g_GripperStallOverCurrentStartMs = 0U;
        }
    }

    if (!near_target && stall_like && abs_filtered_current_a > LOAD3508_STALL_OVERCURRENT_LIMIT_A)
    {
        if (g_GripperStallOverCurrentStartMs == 0U)
        {
            g_GripperStallOverCurrentStartMs = now_ms;
            return;
        }

        if (!g_GripperOverCurrentReturning &&
            (uint32_t)(now_ms - g_GripperStallOverCurrentStartMs) >= LOAD3508_STALL_OVERCURRENT_RETURN_MS)
        {
            Motor_TrapPos_Resync(&g_LoadTrapProfile, current_pos_deg);
            g_GripperOverCurrentReturning = true;
            Load3508OverCurrentProtected = 1U;
            g_GripperOverCurrentStartMs = 0U;
            g_GripperStallOverCurrentStartMs = 0U;
        }
    }
    else if (!near_target)
    {
        g_GripperStallOverCurrentStartMs = 0U;
    }
}

bool LoadMotor_SubmitTarget(LoadMotorOwner_e owner, uint8_t priority, float target_pos_deg, uint32_t owner_hold_ms)
{
    LoadMotorCommand_t cmd = {0};
    size_t sent;
    uint32_t now_ms;

    if (g_x3508CmdStream == NULL || !isfinite(target_pos_deg))
    {
        return false;
    }

    if (owner == LOAD_MOTOR_OWNER_NONE || priority == 0U)
    {
        return false;
    }

    now_ms = HAL_GetTick();
    taskENTER_CRITICAL();
    if (g_Active3508Owner != LOAD_MOTOR_OWNER_NONE &&
        LoadMotor_TimeReached(now_ms, g_Active3508OwnerExpireMs))
    {
        g_Active3508Owner = LOAD_MOTOR_OWNER_NONE;
        g_Active3508Priority = 0U;
    }
    if (g_Active3508Owner != LOAD_MOTOR_OWNER_NONE &&
        g_Active3508Owner != owner &&
        priority < g_Active3508Priority)
    {
        taskEXIT_CRITICAL();
        return false;
    }
    taskEXIT_CRITICAL();

    cmd.magic = LOAD_MOTOR_CMD_MAGIC;
    cmd.timestamp_ms = now_ms;
    cmd.max_age_ms = LOAD_MOTOR_CMD_MAX_AGE_MS;
    cmd.owner_hold_ms = owner_hold_ms;
    cmd.owner = (uint8_t)owner;
    cmd.priority = priority;
    cmd.target_pos_deg = target_pos_deg;

    if (g_x3508Mtx != NULL)
    {
        xSemaphoreTake(g_x3508Mtx, portMAX_DELAY);
    }
    g_ul3508CmdSeq++;
    cmd.seq = g_ul3508CmdSeq;
    cmd.signature = LoadMotor_CalcSignature(&cmd);

    if (xStreamBufferSpacesAvailable(g_x3508CmdStream) < sizeof(cmd))
    {
        (void)xStreamBufferReset(g_x3508CmdStream);
    }
    sent = xStreamBufferSend(g_x3508CmdStream, &cmd, sizeof(cmd), 0);
    if (g_x3508Mtx != NULL)
    {
        xSemaphoreGive(g_x3508Mtx);
    }

    return (sent == sizeof(cmd));
}

void LoadMotor_ReleaseOwner(LoadMotorOwner_e owner)
{
    taskENTER_CRITICAL();
    if (g_Active3508Owner == owner)
    {
        g_Active3508Owner = LOAD_MOTOR_OWNER_NONE;
        g_Active3508Priority = 0U;
        g_Active3508OwnerExpireMs = 0U;
    }
    taskEXIT_CRITICAL();
}

void Motor2006_SetTarget(float target)
{
    if (g_x2006Mtx != NULL)
    {
        xSemaphoreTake(g_x2006Mtx, portMAX_DELAY);
        g_TriggerTarget = target;
        xSemaphoreGive(g_x2006Mtx);
    }
    else
    {
        g_TriggerTarget = target;
    }
}

float Motor2006_GetTarget(void)
{
    float target;

    if (g_x2006Mtx != NULL)
    {
        xSemaphoreTake(g_x2006Mtx, portMAX_DELAY);
        target = g_TriggerTarget;
        xSemaphoreGive(g_x2006Mtx);
        return target;
    }

    return g_TriggerTarget;
}

void Motor2006_EnableControl(bool enable)
{
    g_b2006CtrlEnabled = enable;
}

void Motor6020_SetTarget(float target)
{
    if (g_x6020Mtx != NULL)
    {
        xSemaphoreTake(g_x6020Mtx, portMAX_DELAY);
        g_Yaw6020Target = target;
        xSemaphoreGive(g_x6020Mtx);
    }
    else
    {
        g_Yaw6020Target = target;
    }
}

float Motor6020_GetTarget(void)
{
    float target;

    if (g_x6020Mtx != NULL)
    {
        xSemaphoreTake(g_x6020Mtx, portMAX_DELAY);
        target = g_Yaw6020Target;
        xSemaphoreGive(g_x6020Mtx);
        return target;
    }

    return g_Yaw6020Target;
}

void Motor6020_EnableControl(bool enable)
{
    g_b6020CtrlEnabled = enable;
    if (!enable)
    {
        RmMotorSendCfg(RM_6020_YAW, 0);
    }
}

/**
 * @brief 根据电机配置枚举获取对应的储能电机控制状态
 * @param motor_cfg 储能电机枚举
 * @return 控制状态指针，非储能电机返回 NULL
 */
static StoreMotorCtrlState_t *StoreMotor_GetState(can_motor_cfg motor_cfg)
{
    if (motor_cfg == RM_3508_STORE_LEFT || motor_cfg == DM_3519_STRENTH_LEFT)
    {
        return &g_LeftStoreMotor;
    }
    if (motor_cfg == RM_3508_STORE_RIGHT || motor_cfg == DM_3519_STRENTH_RIGHT)
    {
        return &g_RightStoreMotor;
    }
    return NULL;
}

/**
 * @brief 获取储能电机当前原始总角度（带零点偏置）
 * @param state 储能电机控制状态
 * @return 原始总角度，单位为度
 */
static float StoreMotor_GetRawAngle(const StoreMotorCtrlState_t *state)
{
    if (state == NULL)
    {
        return 0.0f;
    }

    // 原始总角度保留编码器零点偏移，主要给 S 型规划器做轨迹同步使用。
    MotorTypeDef *motor = Motor_GetHandleFast(state->motor_cfg);
    return motor->motor_data.solved_data[3];
}

/**
 * @brief 获取储能电机当前相对角度（业务层统一使用该坐标系）
 * @param state 储能电机控制状态
 * @return 相对零点角度，单位为度
 */
static float StoreMotor_GetRelativeAngle(const StoreMotorCtrlState_t *state)
{
    if (state == NULL)
    {
        return 0.0f;
    }

    // 业务层统一使用相对零点角度，目标值、限位和保护都基于这套坐标系。
    return Motor_GetTotalAngle(state->motor_cfg);
}

/**
 * @brief 重新初始化单个储能电机的 S 型规划器
 * @param state 储能电机控制状态
 * @param current_pos_deg 当前原始角度
 */
static void StoreMotor_ResetTrap(StoreMotorCtrlState_t *state, float current_pos_deg)
{
    MotorTypeDef *motor;
    const MotorTrapConfig_t *trap_cfg;

    if (state == NULL)
    {
        return;
    }

    // 这里不再直接读 STORE3508_TRAP_* 宏，而是统一读取电机注册时写入的规划参数。
    motor = Motor_GetHandleFast(state->motor_cfg);
    trap_cfg = &motor->trap_config;
    if (trap_cfg->registered == 0U)
    {
        state->trap_initialized = 0U;
        return;
    }

    // 每个储能电机各自维护一套 S 型规划器，互不影响。
    Motor_TrapPos_Init(&state->trap_profile,
                       current_pos_deg,
                       trap_cfg->vmax_deg_s,
                       trap_cfg->amax_deg_s2);
    state->trap_profile.brake_gain = trap_cfg->brake_gain;
    state->trap_profile.arrive_zone = trap_cfg->arrive_zone;
    state->trap_profile.decel_zone = trap_cfg->decel_zone;
    Motor_TrapPos_SetJerk(&state->trap_profile, trap_cfg->jmax_deg_s3);
    DWT_GetDeltaT(&state->trap_cnt_last);
    state->trap_initialized = 1U;
}

/**
 * @brief 重置储能电机控制/保护状态
 * @param state 储能电机控制状态
 */
static void StoreMotor_ClearProtectionState(StoreMotorCtrlState_t *state)
{
    if (state == NULL)
    {
        return;
    }

    // 这里只清当前电机自己的保护状态，不连带另一侧储能电机。
    state->protected_flag = false;
    state->over_current_start_ms = 0U;
    state->stall_over_current_start_ms = 0U;
    state->stall_sample_ms = 0U;
    state->filter_ready = 0U;
}

static void StoreMotor_ResetState(StoreMotorCtrlState_t *state)
{
    if (state == NULL)
    {
        return;
    }

    // 初始化目标时先锁定当前位置，避免任务启动瞬间发生大步进。
    state->target = Motor_GetTotalAngle(state->motor_cfg);
    state->filtered_current_a = 0.0f;
    state->filter_ready = 0U;
    state->last_target_change_ms = 0U;
    state->over_current_start_ms = 0U;
    state->stall_over_current_start_ms = 0U;
    state->stall_sample_ms = 0U;
    state->stall_sample_pos_deg = StoreMotor_GetRelativeAngle(state);
    state->ctrl_enabled = false;
    state->use_scurve = false;
    state->protected_flag = false;
    memset(&state->trap_profile, 0, sizeof(state->trap_profile));
    state->trap_cnt_last = 0U;
    state->trap_initialized = 0U;
}

/**
 * @brief 检查单个储能 M3508 的独立异常保护
 * @param state 储能电机控制状态
 * @param now_ms 当前时刻
 * @param target_pos_deg 当前目标相对角度
 */
static void StoreMotor_CheckProtection(StoreMotorCtrlState_t *state, uint32_t now_ms, float target_pos_deg)
{
    float current_a;
    float abs_filtered_current_a;
    float current_pos_deg;
    float pos_error_deg;
    float speed_rpm;
    float temperature_c;
    bool near_target;
    bool stall_like = false;

    if (state == NULL)
    {
        return;
    }

    current_a = Motor_GetCurrent(state->motor_cfg);
    current_pos_deg = StoreMotor_GetRelativeAngle(state);
    speed_rpm = Motor_GetSpeedRPM(state->motor_cfg);
    temperature_c = Motor_GetHandleFast(state->motor_cfg)->motor_data.solved_data[5];

    if (!isfinite(current_a) || !isfinite(current_pos_deg) || !isfinite(speed_rpm))
    {
        // 反馈异常时不立即误判保护，先清本轮计时。
        state->over_current_start_ms = 0U;
        state->stall_over_current_start_ms = 0U;
        return;
    }

    if (isfinite(temperature_c) && temperature_c > STORE3508_TEMP_LIMIT_C)
    {
        // 储能 3508 额外带温度保护，温度超限直接锁保护。
        state->protected_flag = true;
        return;
    }

    if (state->filter_ready == 0U)
    {
        state->filtered_current_a = current_a;
        state->filter_ready = 1U;
        state->stall_sample_pos_deg = current_pos_deg;
        state->stall_sample_ms = now_ms;
        return;
    }

    state->filtered_current_a +=
        (current_a - state->filtered_current_a) * STORE3508_OVERCURRENT_FILTER_ALPHA;
    abs_filtered_current_a = fabsf(state->filtered_current_a);
    pos_error_deg = fabsf(target_pos_deg - current_pos_deg);
    near_target = (pos_error_deg <= MOTOR_DEAD_ZONE);

    // 目标刚切换时给一个空白时间，避免起步电流造成误判。
    if ((uint32_t)(now_ms - state->last_target_change_ms) < STORE3508_OVERCURRENT_TARGET_BLANK_MS)
    {
        state->over_current_start_ms = 0U;
        state->stall_over_current_start_ms = 0U;
        state->stall_sample_pos_deg = current_pos_deg;
        state->stall_sample_ms = now_ms;
        return;
    }

    if (state->stall_sample_ms == 0U ||
        (uint32_t)(now_ms - state->stall_sample_ms) >= STORE3508_STALL_POS_SAMPLE_MS)
    {
        float pos_delta_deg = fabsf(current_pos_deg - state->stall_sample_pos_deg);
        stall_like = (pos_delta_deg <= STORE3508_STALL_POS_DELTA_DEG);
        state->stall_sample_pos_deg = current_pos_deg;
        state->stall_sample_ms = now_ms;
    }
    else
    {
        stall_like = (fabsf(speed_rpm) <= STORE3508_STALL_SPEED_RPM);
    }

    if (near_target)
    {
        // 已经接近目标：
        // 此时持续大电流更像是“到位后顶死机构”，按 still 规则判断。
        state->stall_over_current_start_ms = 0U;
        if (abs_filtered_current_a <= STORE3508_STILL_OVERCURRENT_CLEAR_A)
        {
            state->over_current_start_ms = 0U;
            return;
        }

        if (abs_filtered_current_a > STORE3508_STILL_OVERCURRENT_LIMIT_A)
        {
            if (state->over_current_start_ms == 0U)
            {
                state->over_current_start_ms = now_ms;
                return;
            }

            if ((uint32_t)(now_ms - state->over_current_start_ms) >= STORE3508_STILL_OVERCURRENT_CONFIRM_MS)
            {
                state->protected_flag = true;
            }
        }
    }
    else
    {
        // 尚未到目标：
        // 此时高电流且位置/速度变化很小，更像是运动过程堵转，按 stall 规则判断。
        state->over_current_start_ms = 0U;
        if (abs_filtered_current_a <= STORE3508_STALL_OVERCURRENT_CLEAR_A || !stall_like)
        {
            state->stall_over_current_start_ms = 0U;
            return;
        }

        if (abs_filtered_current_a > STORE3508_STALL_OVERCURRENT_LIMIT_A)
        {
            if (state->stall_over_current_start_ms == 0U)
            {
                state->stall_over_current_start_ms = now_ms;
                return;
            }

            if ((uint32_t)(now_ms - state->stall_over_current_start_ms) >= STORE3508_STALL_CONFIRM_MS)
            {
                state->protected_flag = true;
            }
        }
    }
}

/**
 * @brief 执行单个储能电机控制
 * @param state 储能电机控制状态
 */
static void StoreMotor_RunControl(StoreMotorCtrlState_t *state)
{
    float target_relative_deg;
    float target_raw_deg;

    if (state == NULL || !state->ctrl_enabled)
    {
        return;
    }

    target_relative_deg = state->target;
    StoreMotor_CheckProtection(state, HAL_GetTick(), target_relative_deg);
    if (state->protected_flag)
    {
        // 一旦当前电机进入保护，立即断输出，但不影响另一侧储能电机继续工作。
        RmMotorSendCfg(state->motor_cfg, 0);
        return;
    }

    if (!state->use_scurve)
    {
        // 不启用 S 型规划时，目标直接送位置环。
        RmMotorPID_Calc(state->motor_cfg, target_relative_deg);
        return;
    }

    target_raw_deg = target_relative_deg + Motor_GetHandleFast(state->motor_cfg)->motor_data.offset_ecd_angle;
    if (state->trap_initialized == 0U)
    {
        StoreMotor_ResetTrap(state, StoreMotor_GetRawAngle(state));
        if (state->trap_initialized == 0U)
        {
            // 没有为该电机注册 S 型规划参数时，自动回退为普通位置 PID，避免使用未初始化规划器。
            RmMotorPID_Calc(state->motor_cfg, target_relative_deg);
            return;
        }
    }
    state->trap_profile.target_pos = target_raw_deg;

    {
        float dt_s = DWT_GetDeltaT(&state->trap_cnt_last);
        float cmd_pos;

        if (!isfinite(dt_s) || dt_s <= 0.0f)
        {
            dt_s = 0.001f;
        }
        else if (dt_s > 0.02f)
        {
            dt_s = 0.02f;
        }

        // 启用 S 型规划时，先生成平滑位置，再交给底层 PID 去跟踪。
        cmd_pos = Motor_TrapPos_Update(&state->trap_profile, dt_s);
        RmMotorPID_Calc(state->motor_cfg,
                        cmd_pos - Motor_GetHandleFast(state->motor_cfg)->motor_data.offset_ecd_angle);
    }
}

bool StoreMotor_Enable(can_motor_cfg motor_cfg)
{
    StoreMotorCtrlState_t *state = StoreMotor_GetState(motor_cfg);
    if (state == NULL)
    {
        return false;
    }

    StoreMotor_ClearProtectionState(state);
    StoreMotor_SetTarget(motor_cfg, Motor_GetTotalAngle(state->motor_cfg));
    return true;
}

void StoreMotor_Disable(can_motor_cfg motor_cfg)
{
    StoreMotorCtrlState_t *state = StoreMotor_GetState(motor_cfg);
    if (state == NULL)
    {
        return;
    }

    state->ctrl_enabled = false;
    RmMotorSendCfg(state->motor_cfg, 0);
}

void StoreMotor_EnableControl(can_motor_cfg motor_cfg, bool enable)
{
    StoreMotorCtrlState_t *state = StoreMotor_GetState(motor_cfg);
    if (state == NULL)
    {
        return;
    }

    state->ctrl_enabled = enable;
    if (!enable)
    {
        state->trap_initialized = 0U;
        RmMotorSendCfg(state->motor_cfg, 0);
    }
}

void StoreMotor_SetTarget(can_motor_cfg motor_cfg, float target)
{
    StoreMotorCtrlState_t *state = StoreMotor_GetState(motor_cfg);
    if (state == NULL || !isfinite(target))
    {
        return;
    }

    if (g_xStore3508Mtx != NULL)
    {
        xSemaphoreTake(g_xStore3508Mtx, portMAX_DELAY);
    }
    state->target = target;
    state->last_target_change_ms = HAL_GetTick();
    state->over_current_start_ms = 0U;
    state->stall_over_current_start_ms = 0U;
    state->stall_sample_ms = 0U;
    state->stall_sample_pos_deg = StoreMotor_GetRelativeAngle(state);
    if (state->use_scurve)
    {
        StoreMotor_ResetTrap(state, StoreMotor_GetRawAngle(state));
    }
    if (g_xStore3508Mtx != NULL)
    {
        xSemaphoreGive(g_xStore3508Mtx);
    }
}

float StoreMotor_GetTarget(can_motor_cfg motor_cfg)
{
    float target = 0.0f;
    StoreMotorCtrlState_t *state = StoreMotor_GetState(motor_cfg);
    if (state == NULL)
    {
        return 0.0f;
    }

    if (g_xStore3508Mtx != NULL)
    {
        xSemaphoreTake(g_xStore3508Mtx, portMAX_DELAY);
        target = state->target;
        xSemaphoreGive(g_xStore3508Mtx);
        return target;
    }

    return state->target;
}

void StoreMotor_RefreshData(can_motor_cfg motor_cfg)
{
    StoreMotorCtrlState_t *state = StoreMotor_GetState(motor_cfg);
    if (state == NULL)
    {
        return;
    }

    MotorTypeDef *motor = Motor_GetHandleFast(state->motor_cfg);
    if (motor->calculate != NULL)
    {
        motor->calculate(motor);
    }
}

void StoreMotor_SetUseSCurve(can_motor_cfg motor_cfg, bool enable)
{
    StoreMotorCtrlState_t *state = StoreMotor_GetState(motor_cfg);
    if (state == NULL)
    {
        return;
    }

    state->use_scurve = enable;
    state->trap_initialized = 0U;
}

bool StoreMotor_IsProtected(can_motor_cfg motor_cfg)
{
    StoreMotorCtrlState_t *state = StoreMotor_GetState(motor_cfg);
    if (state == NULL)
    {
        return false;
    }

    return state->protected_flag;
}

bool StoreMotor_IsAnyProtected(void)
{
    return g_LeftStoreMotor.protected_flag || g_RightStoreMotor.protected_flag;
}

void StoreMotor_ClearProtection(void)
{
    StoreMotor_ClearProtectionState(&g_LeftStoreMotor);
    StoreMotor_ClearProtectionState(&g_RightStoreMotor);
}

/***********************************
 * 函数名: Motor3508CtrlTask
 * 作用:   3508电机独立控制任务，固定2ms周期
 *         只负责梯形规划 + PID，不做任何状态逻辑
 **********************************/
void Motor3508CtrlTask(void *argument)
{
    TickType_t xLastWake;
    MotorTypeDef *motor = Motor_GetHandleFast(RM_3508_GRIPPER);
    const MotorTrapConfig_t *trap_cfg = &motor->trap_config;

    (void)argument;

    MotorData = LoadMotor_GetRawAngleInternal();
    if (trap_cfg->registered != 0U)
    {
        // 夹爪 3508 的 S 型规划参数同样由 MotorRegister() 统一注册，这里只做一次运行时初始化。
        Motor_TrapPos_Init(&g_LoadTrapProfile, MotorData,
                           trap_cfg->vmax_deg_s, trap_cfg->amax_deg_s2);
        g_LoadTrapProfile.brake_gain = trap_cfg->brake_gain;
        g_LoadTrapProfile.arrive_zone = trap_cfg->arrive_zone;
        g_LoadTrapProfile.decel_zone = trap_cfg->decel_zone;
        Motor_TrapPos_SetJerk(&g_LoadTrapProfile, trap_cfg->jmax_deg_s3);
    }
    (void)LoadMotor_SubmitTarget(LOAD_MOTOR_OWNER_HOME,
                                 LOAD_MOTOR_PRIORITY_HOME,
                                 MotorData,
                                 LOAD_MOTOR_DEFAULT_HOLD_MS);
    DWT_GetDeltaT(&g_LoadTrapCntLast);

    xLastWake = xTaskGetTickCount();
    for (;;)
    {
        LoadMotor_ProcessCommandStream();
        if (g_b3508CtrlEnabled)
        {
            float target_pos_deg = Motor3508_GetTarget();
            Motor3508_CheckOverCurrent(HAL_GetTick(), target_pos_deg);
            LoadMotor_RunTrapTo(Motor3508_GetTarget());
        }
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(2));
    }
}

/***********************************
 * 函数名: Motor2006CtrlTask
 * 作用:   2006扳机电机独立控制任务，固定2ms周期
 *         只负责PID位置跟踪
 **********************************/
void Motor2006CtrlTask(void *argument)
{
    TickType_t xLastWake;

    (void)argument;

    xLastWake = xTaskGetTickCount();
    for (;;)
    {
        if (g_b2006CtrlEnabled)
        {
            RmMotorPID_Calc(RM_2006_TRIGGER, Motor2006_GetTarget());
        }
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(2));
    }
}

/***********************************
 * 函数名: MotorLeftStore3508CtrlTask
 * 作用:   左侧储能 M3508 独立控制任务，固定 2ms 周期
 **********************************/
void MotorLeftStore3508CtrlTask(void *argument)
{
    TickType_t xLastWake;
    StoreMotorCtrlState_t *state = &g_LeftStoreMotor;

    (void)argument;

    StoreMotor_ResetState(state);
    StoreMotor_SetTarget(state->motor_cfg, Motor_GetTotalAngle(state->motor_cfg));

    xLastWake = xTaskGetTickCount();
    for (;;)
    {
        StoreMotor_RunControl(state);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(2));
    }
}

/***********************************
 * 函数名: MotorRightStore3508CtrlTask
 * 作用:   右侧储能 M3508 独立控制任务，固定 2ms 周期
 **********************************/
void MotorRightStore3508CtrlTask(void *argument)
{
    TickType_t xLastWake;
    StoreMotorCtrlState_t *state = &g_RightStoreMotor;

    (void)argument;

    StoreMotor_ResetState(state);
    StoreMotor_SetTarget(state->motor_cfg, Motor_GetTotalAngle(state->motor_cfg));

    xLastWake = xTaskGetTickCount();
    for (;;)
    {
        StoreMotor_RunControl(state);
        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(2));
    }
}

/***********************************
 * 函数名: Motor6020CtrlTask
 * 作用:   控制yaw角的6020电机独立控制任务，固定2ms周期
 *         逻辑与2006类似，位置环持续跟踪
 **********************************/
void Motor6020CtrlTask(void *argument)
{
    TickType_t xLastWake;

    (void)argument;

    xLastWake = xTaskGetTickCount();
    for (;;)
    {
        if (g_b6020CtrlEnabled)
        {
            MotorTypeDef *motor = &MotorManager.MotorList[RM_6020_YAW - 1];
            float target_yaw = Motor6020_GetTarget();
            RmMotorPID_Calc(RM_6020_YAW, target_yaw);
        }

        vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(2));
    }
}
