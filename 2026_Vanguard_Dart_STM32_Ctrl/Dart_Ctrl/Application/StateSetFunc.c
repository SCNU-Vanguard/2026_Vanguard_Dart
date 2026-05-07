#include "StateSetFunc.h"
#include "UserTask.h"

static volatile uint32_t g_ulStateSetRequestSeq = 0U;
static volatile uint32_t g_ulStateSetDoneSeq = 0U;
static volatile bool g_bStateSetLastSuccess = false;

extern QueueHandle_t g_xStateSetRequestQueueHandle;
extern SemaphoreHandle_t g_xStateSetDoneSemHandle;
extern volatile bool g_bStoreMotorSafeReturnPending;
extern volatile uint8_t LoadNumState;
extern EventGroupHandle_t g_pxStateSetEventGroupHandle;

extern float gripper_offset;
extern float trigger_offset;
// 复用 LoadTask 的换弹步进角度计算，保证状态设置和换弹逻辑一致。
extern float Load3508_GetReloadOffset(uint8_t step_count);

/// @brief 生成飞镖微调的具体数据
/// @param dart_num 飞镖序号
/// @param request 请求信息
/// @return true -> 成功
bool StateSet_BuildRequest(uint8_t dart_num, StateSetRequest_t *request)
{
    FirePermission_t permission = {0};
    uint8_t user_fire_target = FIRE_TARGET_OUTPOST;
    uint8_t setup_idx;
    float base_yaw;
    float base_trigger;

    if (request == NULL)
    {
        return false;
    }

    FireControl_GetPermission(&permission);
    if (!permission.can_setup)
    {
        return false;
    }

    if (dart_num < 1U || dart_num > 4U)
    {
        dart_num = 4U;
    }

    user_fire_target = FireControl_SelectTarget();

    // 对于每发进行目标位置微调
    setup_idx = (uint8_t)(4U - dart_num);
    base_yaw = (user_fire_target == FIRE_TARGET_BASE) ? BaseYawAngle : OutpostYawAngle;
    base_trigger = (user_fire_target == FIRE_TARGET_BASE) ? BaseTrigger : OutpostTrigger;

    taskENTER_CRITICAL();
    g_ulStateSetRequestSeq++;
    request->seq = g_ulStateSetRequestSeq;
    taskEXIT_CRITICAL();

    request->ref_seq = permission.ref_seq;
    request->intent_seq = permission.intent_seq;
    request->dart_num = dart_num;
    request->yaw_target = base_yaw + g_SetupYaw[setup_idx];
    request->trigger_target = base_trigger + g_SetupTrigger[setup_idx];

    return true;
}

/// @brief 发出请求
/// @param request 请求的具体信息
/// @return true -> 成功
bool StateSet_SubmitRequest(const StateSetRequest_t *request)
{
    if (request == NULL || g_xStateSetRequestQueueHandle == NULL)
    {
        return false;
    }

    xQueueOverwrite(g_xStateSetRequestQueueHandle, request);
    return true;
}

static void StateSet_FinishRequest(uint32_t seq, bool success)
{
    taskENTER_CRITICAL();
    g_ulStateSetDoneSeq = seq;
    g_bStateSetLastSuccess = success;
    taskEXIT_CRITICAL();

    if (g_xStateSetDoneSemHandle != NULL)
    {
        xSemaphoreGive(g_xStateSetDoneSemHandle);
    }
}

/// @brief 发送请求并进行调整yaw和trigger
/// @param dart_num 当前是第几发飞镖
/// @param timeout_ticks 超时时间
/// @return true -> 成功
/// @note  通过互斥量保护和队列传递消息
bool StateSet_RequestAndWait(uint8_t dart_num, TickType_t timeout_ticks)
{
    StateSetRequest_t request = {0};

    if (!StateSet_BuildRequest(dart_num, &request))
    {
        return false;
    }

    if (g_xStateSetDoneSemHandle != NULL)
    {
        xSemaphoreTake(g_xStateSetDoneSemHandle, 0);
    }

    if (!StateSet_SubmitRequest(&request))
    {
        return false;
    }

    while (g_ulStateSetDoneSeq < request.seq)
    {
        if (g_xStateSetDoneSemHandle == NULL ||
            xSemaphoreTake(g_xStateSetDoneSemHandle, timeout_ticks) != pdTRUE)
        {
            return false;
        }
    }

    return g_bStateSetLastSuccess;
}

void StateSetTaskMainLoopFunc(void)
{

    static bool state_ready_once = false;
    StateSetRequest_t task_request = {0};

    float g_2006preseting_distance = 0.0f;
    float preseting_yaw = 0.0f;
    float g_gripper_preset = 0.0f;

    // 使能两侧储能 M3508 电机
    while (!StoreMotor_Enable(RM_3508_STORE_LEFT))
    {
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    while (!StoreMotor_Enable(RM_3508_STORE_RIGHT))
    {
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    StoreMotor_ClearProtection();
    g_bStoreMotorSafeReturnPending = false;
    uint8_t DiagYaw4310EnableOk = DM_MotorEnable(DM_4310_YAW);
    while (1)
    {
        // 检查是否有裁判系统或者微调唤醒state_set,检查是否允许设置,如果不允许设置将直接等待
        if (g_xStateSetRequestQueueHandle == NULL ||
            xQueueReceive(g_xStateSetRequestQueueHandle, &task_request, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        uint32_t setup_gate_start = HAL_GetTick();
        bool setup_allowed = true;
        while (!FireControl_CanSetup())
        {
            if ((uint32_t)(HAL_GetTick() - setup_gate_start) >= STATE_SET_SETUP_GATE_TIMEOUT_MS)
            {
                setup_allowed = false;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (!setup_allowed)
        {
            StateSet_FinishRequest(task_request.seq, false);
            continue;
        }

        // 3508与2006获取最开始的上电零偏
        gripper_offset = 0.0f;
        trigger_offset = 0.0f;

        g_gripper_preset = PresetLoc + gripper_offset;
        preseting_yaw = task_request.yaw_target;
        g_2006preseting_distance = task_request.trigger_target + trigger_offset;

        // 2006预设位置
        Motor2006_SetTarget(g_2006preseting_distance);
        Motor2006_EnableControl(true);
        DeadzoneTimer_t trigger_preset_timer = {0};
        while (!IsInDeadzoneTimedF(Motor_GetTotalAngle(RM_2006_TRIGGER),
                                   g_2006preseting_distance,
                                   TRIGGER_DEAD_ZONE,
                                   TRIGGER_2006_SETUP_TIMEOUT_MS,
                                   &trigger_preset_timer))
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // 6020 Yaw 归位
        //         Motor6020_SetTarget(g_6020preseting_yaw);
        //         Motor6020_EnableControl(true);
        // #if ENABLE_6020_YAW_WAIT
        //         DeadzoneState_t yaw_preset_deadzone = {0};
        //         while (!IsInDeadzoneF(Motor_GetTotalAngle(RM_6020_YAW), g_6020preseting_yaw, YAW_6020_DEAD_ZONE, &yaw_preset_deadzone, true))
        //         {
        //             vTaskDelay(pdMS_TO_TICKS(10));
        //         }
        // #endif

        // 3508 初始对位：设目标，使能控制任务，等到位
        Motor3508_EnableControl(true);
        uint8_t loaded_count = 0U;
        taskENTER_CRITICAL();
        loaded_count = LoadNumState;
        taskEXIT_CRITICAL();
        float gripper_target = g_gripper_preset;
        if (state_ready_once)
        {
            gripper_target += Load3508_GetReloadOffset(loaded_count);
        }
        DeadzoneTimer_t preset_timer = {0};
        uint32_t preset_timeout_ms = CalcTrapMoveTimeoutMs(
            Motor_GetTotalAngle(RM_3508_GRIPPER),
            gripper_target,
            LOAD_TASK_TRAP_VMAX_DEG_S,
            LOAD_TASK_TRAP_AMAX_DEG_S2,
            LOAD_DEADZONE_TIMEOUT_MS,
            STATE_SET_GRIPPER_SETUP_MAX_TIMEOUT_MS);
        if (!LoadMotor_SubmitTarget(LOAD_MOTOR_OWNER_STATE_SET,
                                    LOAD_MOTOR_PRIORITY_STATE_SET,
                                    gripper_target,
                                    preset_timeout_ms + STATE_SET_GRIPPER_OWNER_EXTRA_MS))
        {
            StateSet_FinishRequest(task_request.seq, false);
            continue;
        }
        while (!IsInDeadzoneTimedF(Motor_GetTotalAngle(RM_3508_GRIPPER), gripper_target, MOTOR_DEAD_ZONE, preset_timeout_ms, &preset_timer))
        {
            if (LoadMotor_IsOverCurrentProtected())
            {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (LoadMotor_IsOverCurrentProtected())
        {
            LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_STATE_SET);
            StateSet_FinishRequest(task_request.seq, false);
            continue;
        }
        LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_STATE_SET);

        // 4310 Yaw 调整(不需要额外的任务)
        if (!StateSet_ApplyYawPreset(preseting_yaw))
        {
            StateSet_FinishRequest(task_request.seq, false);
            continue;
        }

        // 事件组唤醒所有其他的任务
        if (!state_ready_once)
        {
            xEventGroupSetBits(g_pxStateSetEventGroupHandle, EVENT_ALL_READY);
        }

        // 置成功设定标志位
        FireControl_MarkSetupApplied();
        StateSet_FinishRequest(task_request.seq, true);
        state_ready_once = true;

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

#define STATE_SET_DM_YAW_DEAD_ZONE 1.0f
#define STATE_SET_DM_YAW_REISSUE_MS 20U
#define STATE_SET_DM_YAW_TIMEOUT_MS 3000U

static bool g_bStateSetDirectReadyOnce = false;

static bool StateSet_ApplyDmYawPreset(float target_yaw)
{
    float degree = 0.0f;
    uint32_t preset_yaw_reissue_tick = HAL_GetTick();
    uint32_t preset_yaw_start_tick = preset_yaw_reissue_tick;
    DeadzoneState_t yaw_preset_deadzone = {0};

    DM_MotorEnable(DM_4310_YAW);
    DmMotorPID_Calc(DM_4310_YAW, target_yaw);
    while (1)
    {
        degree = Motor_GetTotalAngle(DM_4310_YAW);
        if (IsInDeadzoneF(degree, target_yaw, STATE_SET_DM_YAW_DEAD_ZONE, &yaw_preset_deadzone, false))
        {
            return true;
        }

        if ((uint32_t)(HAL_GetTick() - preset_yaw_start_tick) >= STATE_SET_DM_YAW_TIMEOUT_MS)
        {
            return false;
        }

        DM_Motor_RefreshData(DM_4310_YAW);
        if ((uint32_t)(HAL_GetTick() - preset_yaw_reissue_tick) >= STATE_SET_DM_YAW_REISSUE_MS)
        {
            DmMotorPID_Calc(DM_4310_YAW, target_yaw);
            preset_yaw_reissue_tick = HAL_GetTick();
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

bool StateSet_ApplyYawPreset(float target_yaw)
{
    if (target_yaw < DM_J4310_Class.pos_min)
    {
        target_yaw = DM_J4310_Class.pos_min;
    }
    else if (target_yaw > DM_J4310_Class.pos_max)
    {
        target_yaw = DM_J4310_Class.pos_max;
    }

    return StateSet_ApplyDmYawPreset(target_yaw);
}

static bool StateSet_RunPresetSequence(const StateSetRequest_t *request)
{
    float gripper_target;
    uint8_t loaded_count = 0U;
    uint32_t preset_timeout_ms;

    if (request == NULL)
    {
        return false;
    }

    gripper_offset = 0.0f;
    trigger_offset = 0.0f;

    Motor2006_SetTarget(request->trigger_target + trigger_offset);
    Motor2006_EnableControl(true);
    DeadzoneTimer_t trigger_preset_timer = {0};
    while (!IsInDeadzoneTimedF(Motor_GetTotalAngle(RM_2006_TRIGGER),
                               request->trigger_target + trigger_offset,
                               TRIGGER_DEAD_ZONE,
                               TRIGGER_2006_SETUP_TIMEOUT_MS,
                               &trigger_preset_timer))
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    Motor3508_EnableControl(true);
    taskENTER_CRITICAL();
    loaded_count = LoadNumState;
    taskEXIT_CRITICAL();
    gripper_target = PresetLoc + gripper_offset;
    if (g_bStateSetDirectReadyOnce)
    {
        gripper_target += Load3508_GetReloadOffset(loaded_count);
    }

    preset_timeout_ms = CalcTrapMoveTimeoutMs(
        Motor_GetTotalAngle(RM_3508_GRIPPER),
        gripper_target,
        LOAD_TASK_TRAP_VMAX_DEG_S,
        LOAD_TASK_TRAP_AMAX_DEG_S2,
        LOAD_DEADZONE_TIMEOUT_MS,
        STATE_SET_GRIPPER_SETUP_MAX_TIMEOUT_MS);

    if (!LoadMotor_SubmitTarget(LOAD_MOTOR_OWNER_STATE_SET,
                                LOAD_MOTOR_PRIORITY_STATE_SET,
                                gripper_target,
                                preset_timeout_ms + STATE_SET_GRIPPER_OWNER_EXTRA_MS))
    {
        return false;
    }

    {
        DeadzoneTimer_t preset_timer = {0};
        while (!IsInDeadzoneTimedF(Motor_GetTotalAngle(RM_3508_GRIPPER),
                                    gripper_target,
                                    MOTOR_DEAD_ZONE,
                                    preset_timeout_ms,
                                    &preset_timer))
        {
            if (LoadMotor_IsOverCurrentProtected())
            {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    if (LoadMotor_IsOverCurrentProtected())
    {
        LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_STATE_SET);
        return false;
    }

    LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_STATE_SET);

    if (!StateSet_ApplyDmYawPreset(request->yaw_target))
    {
        return false;
    }

    if (!g_bStateSetDirectReadyOnce)
    {
        xEventGroupSetBits(g_pxStateSetEventGroupHandle, EVENT_ALL_READY);
        g_bStateSetDirectReadyOnce = true;
    }

    FireControl_MarkSetupApplied();
    return true;
}

bool StateSet_ApplyRequest(const StateSetRequest_t *request)
{
    StateSetRequest_t request_local = {0};

    if (request == NULL)
    {
        return false;
    }

    request_local = *request;
    if (request_local.yaw_target < DM_J4310_Class.pos_min)
    {
        request_local.yaw_target = DM_J4310_Class.pos_min;
    }
    else if (request_local.yaw_target > DM_J4310_Class.pos_max)
    {
        request_local.yaw_target = DM_J4310_Class.pos_max;
    }

    return StateSet_RunPresetSequence(&request_local);
}

bool StateSet_ApplyPreset(uint8_t dart_num)
{
    StateSetRequest_t request = {0};

    if (!StateSet_BuildRequest(dart_num, &request))
    {
        return false;
    }

    if (request.yaw_target < DM_J4310_Class.pos_min)
    {
        request.yaw_target = DM_J4310_Class.pos_min;
    }
    else if (request.yaw_target > DM_J4310_Class.pos_max)
    {
        request.yaw_target = DM_J4310_Class.pos_max;
    }

    return StateSet_RunPresetSequence(&request);
}
