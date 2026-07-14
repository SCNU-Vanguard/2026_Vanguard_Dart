#include "StateSetFunc.h"
#include "UserTask.h"
#include "angle_motor.h"

static volatile uint32_t g_ulStateSetRequestSeq = 0U; /* 初始化 g_ulStateSetRequestSeq。 */
static volatile uint32_t g_ulStateSetDoneSeq = 0U; /* 初始化 g_ulStateSetDoneSeq。 */
static volatile bool g_bStateSetLastSuccess = false; /* 初始化 g_bStateSetLastSuccess。 */

extern QueueHandle_t g_xStateSetRequestQueueHandle; /* 声明外部变量 g_xStateSetRequestQueueHandle。 */
extern SemaphoreHandle_t g_xStateSetDoneSemHandle; /* 声明外部变量 g_xStateSetDoneSemHandle。 */
extern volatile bool g_bStoreMotorSafeReturnPending; /* 声明外部变量 g_bStoreMotorSafeReturnPending。 */
extern volatile uint8_t LoadNumState; /* 声明外部变量 LoadNumState。 */
extern EventGroupHandle_t g_pxStateSetEventGroupHandle; /* 声明外部变量 g_pxStateSetEventGroupHandle。 */

extern float gripper_offset; /* 声明外部变量 gripper_offset。 */
extern float trigger_offset; /* 声明外部变量 trigger_offset。 */
// 复用 LoadTask 的换弹步进角度计算，保证状态设置和换弹逻辑一致。
extern float Load3508_GetReloadOffset(uint8_t step_count); /* 声明 Load3508_GetReloadOffset 接口。 */

/// @brief 生成飞镖微调的具体数据
/// @param dart_num 飞镖序号
/// @param request 请求信息
/// @return true -> 成功
bool StateSet_BuildRequest(uint8_t dart_num, StateSetRequest_t *request) /* 实现 StateSet_BuildRequest。 */
{
    FirePermission_t permission = {0}; /* 初始化 permission。 */
    uint8_t user_fire_target = FIRE_TARGET_OUTPOST; /* 初始化 user_fire_target。 */
    uint8_t setup_idx; /* 保存 setup_idx。 */
    float base_yaw; /* 保存 base_yaw。 */
    float base_trigger; /* 保存 base_trigger。 */

    if (request == NULL) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }

    FireControl_GetPermission(&permission); /* 调用 FireControl_GetPermission。 */
    if (!permission.can_setup) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }

    if (dart_num < 1U || dart_num > 4U) /* 检查当前执行条件。 */
    {
        dart_num = 4U; /* 更新 dart_num。 */
    }

    user_fire_target = FireControl_SelectTarget(); /* 更新 user_fire_target。 */

    // 对于每发进行目标位置微调
    setup_idx = (uint8_t)(4U - dart_num); /* 更新 setup_idx。 */
    base_yaw = (user_fire_target == FIRE_TARGET_BASE) ? BaseYawAngle : OutpostYawAngle; /* 更新 base_yaw。 */
    base_trigger = (user_fire_target == FIRE_TARGET_BASE) ? BaseTrigger : OutpostTrigger; /* 更新 base_trigger。 */

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    g_ulStateSetRequestSeq++; /* 递增 g_ulStateSetRequestSeq。 */
    request->seq = g_ulStateSetRequestSeq; /* 更新 seq。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    request->ref_seq = permission.ref_seq; /* 更新 ref_seq。 */
    request->intent_seq = permission.intent_seq; /* 更新 intent_seq。 */
    request->dart_num = dart_num; /* 更新 dart_num。 */
    request->yaw_target = base_yaw + g_SetupYaw[setup_idx]; /* 更新 yaw_target。 */
    request->trigger_target = base_trigger + g_SetupTrigger[setup_idx]; /* 更新 trigger_target。 */

    return true; /* 返回 true。 */
}

/// @brief 发出请求
/// @param request 请求的具体信息
/// @return true -> 成功
bool StateSet_SubmitRequest(const StateSetRequest_t *request) /* 实现 StateSet_SubmitRequest。 */
{
    if (request == NULL || g_xStateSetRequestQueueHandle == NULL) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }

    xQueueOverwrite(g_xStateSetRequestQueueHandle, request); /* 调用 xQueueOverwrite。 */
    return true; /* 返回 true。 */
}

static void StateSet_FinishRequest(uint32_t seq, bool success) /* 实现 StateSet_FinishRequest。 */
{
    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    g_ulStateSetDoneSeq = seq; /* 更新 g_ulStateSetDoneSeq。 */
    g_bStateSetLastSuccess = success; /* 更新 g_bStateSetLastSuccess。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    if (g_xStateSetDoneSemHandle != NULL) /* 检查当前执行条件。 */
    {
        xSemaphoreGive(g_xStateSetDoneSemHandle); /* 调用 xSemaphoreGive。 */
    }
}

/// @brief 发送请求并进行调整yaw和trigger
/// @param dart_num 当前是第几发飞镖
/// @param timeout_ticks 超时时间
/// @return true -> 成功
/// @note  通过互斥量保护和队列传递消息
bool StateSet_RequestAndWait(uint8_t dart_num, TickType_t timeout_ticks) /* 实现 StateSet_RequestAndWait。 */
{
    StateSetRequest_t request = {0}; /* 初始化 request。 */

    if (!StateSet_BuildRequest(dart_num, &request)) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }

    if (g_xStateSetDoneSemHandle != NULL) /* 检查当前执行条件。 */
    {
        xSemaphoreTake(g_xStateSetDoneSemHandle, 0); /* 调用 xSemaphoreTake。 */
    }

    if (!StateSet_SubmitRequest(&request)) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }

    while (g_ulStateSetDoneSeq < request.seq) /* 条件满足时继续执行。 */
    {
        if (g_xStateSetDoneSemHandle == NULL || /* 检查当前执行条件。 */
            xSemaphoreTake(g_xStateSetDoneSemHandle, timeout_ticks) != pdTRUE) /* 继续更新 目标值。 */
        {
            return false; /* 返回 false。 */
        }
    }

    return g_bStateSetLastSuccess; /* 返回当前计算结果。 */
}

void StateSetTaskMainLoopFunc(void) /* 实现 StateSetTaskMainLoopFunc。 */
{

    static bool state_ready_once = false; /* 初始化 state_ready_once。 */
    StateSetRequest_t task_request = {0}; /* 初始化 task_request。 */

    float g_2006preseting_distance = 0.0f; /* 初始化 g_2006preseting_distance。 */
    float g_gripper_preset = 0.0f; /* 初始化 g_gripper_preset。 */

    // 使能两侧储能 M3508 电机（Motor_EnableControl 自带 ClearProtection + 锁当前位置）
    Motor_EnableControl(RM_3508_STORE_LEFT, true); /* 调用 Motor_EnableControl。 */
    vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
    Motor_EnableControl(RM_3508_STORE_RIGHT, true); /* 调用 Motor_EnableControl。 */
    vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
    AngleMotor_ClearFault(RM_3508_STORE_LEFT); /* 调用 AngleMotor_ClearFault。 */
    AngleMotor_ClearFault(RM_3508_STORE_RIGHT); /* 调用 AngleMotor_ClearFault。 */
    g_bStoreMotorSafeReturnPending = false; /* 更新 g_bStoreMotorSafeReturnPending。 */
    (void)DM_MotorEnable(DM_4310_YAW); /* 调用 DM_MotorEnable。 */
    while (1) /* 持续执行当前任务。 */
    {
        // 检查是否有裁判系统或者微调唤醒state_set,检查是否允许设置,如果不允许设置将直接等待
        if (g_xStateSetRequestQueueHandle == NULL || /* 检查当前执行条件。 */
            xQueueReceive(g_xStateSetRequestQueueHandle, &task_request, portMAX_DELAY) != pdTRUE) /* 继续更新 目标值。 */
        {
            continue; /* 跳过本轮剩余处理。 */
        }

        uint32_t setup_gate_start = HAL_GetTick(); /* 初始化 setup_gate_start。 */
        bool setup_allowed = true; /* 初始化 setup_allowed。 */
        while (!FireControl_CanSetup()) /* 条件满足时继续执行。 */
        {
            if ((uint32_t)(HAL_GetTick() - setup_gate_start) >= STATE_SET_SETUP_GATE_TIMEOUT_MS) /* 检查当前执行条件。 */
            {
                setup_allowed = false; /* 更新 setup_allowed。 */
                break; /* 结束当前循环或分支。 */
            }
            vTaskDelay(pdMS_TO_TICKS(100)); /* 调用 vTaskDelay。 */
        }
        if (!setup_allowed) /* 检查当前执行条件。 */
        {
            StateSet_FinishRequest(task_request.seq, false); /* 调用 StateSet_FinishRequest。 */
            continue; /* 跳过本轮剩余处理。 */
        }

        // 3508与2006获取最开始的上电零偏
        gripper_offset = 0.0f; /* 更新 gripper_offset。 */
        trigger_offset = 0.0f; /* 更新 trigger_offset。 */

        g_gripper_preset = PresetLoc + gripper_offset; /* 更新 g_gripper_preset。 */
        g_2006preseting_distance = trigger_offset; // task_request.trigger_target + trigger_offset;

        // 2006预设位置
        Motor_SetTarget(RM_2006_TRIGGER, g_2006preseting_distance); /* 调用 Motor_SetTarget。 */
        Motor_EnableControl(RM_2006_TRIGGER, true); /* 调用 Motor_EnableControl。 */
        DeadzoneTimer_t trigger_preset_timer = {0}; /* 初始化 trigger_preset_timer。 */
        while (!IsInDeadzoneTimedF(Motor_GetTotalAngle(RM_2006_TRIGGER), /* 条件满足时继续执行。 */
                                   g_2006preseting_distance, /* 传入下一项参数或数据。 */
                                   TRIGGER_DEAD_ZONE, /* 定义 TRIGGER_DEAD_ZONE 枚举项。 */
                                   TRIGGER_2006_SETUP_TIMEOUT_MS, /* 定义 TRIGGER_2006_SETUP_TIMEOUT_MS 枚举项。 */
                                   &trigger_preset_timer)) /* 继续当前语句。 */
        {
            vTaskDelay(pdMS_TO_TICKS(10)); /* 调用 vTaskDelay。 */
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
        Motor_EnableControl(RM_3508_GRIPPER, true); /* 调用 Motor_EnableControl。 */
        uint8_t loaded_count = 0U; /* 初始化 loaded_count。 */
        taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
        loaded_count = LoadNumState; /* 更新 loaded_count。 */
        taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
        float gripper_target = g_gripper_preset; /* 初始化 gripper_target。 */
        if (state_ready_once) /* 检查当前执行条件。 */
        {
            gripper_target += Load3508_GetReloadOffset(loaded_count); /* 更新 gripper_target。 */
        }
        DeadzoneTimer_t preset_timer = {0}; /* 初始化 preset_timer。 */
        uint32_t preset_timeout_ms = CalcTrapMoveTimeoutMs( /* 开始计算 preset_timeout_ms。 */
            Motor_GetTotalAngle(RM_3508_GRIPPER), /* 传入下一项参数或数据。 */
            gripper_target, /* 传入下一项参数或数据。 */
            LOAD_TASK_TRAP_VMAX_DEG_S, /* 定义 LOAD_TASK_TRAP_VMAX_DEG_S 枚举项。 */
            LOAD_TASK_TRAP_AMAX_DEG_S2, /* 定义 LOAD_TASK_TRAP_AMAX_DEG_S2 枚举项。 */
            LOAD_DEADZONE_TIMEOUT_MS, /* 定义 LOAD_DEADZONE_TIMEOUT_MS 枚举项。 */
            STATE_SET_GRIPPER_SETUP_MAX_TIMEOUT_MS); /* 完成本行操作。 */
        if (!LoadMotor_SubmitTarget(LOAD_MOTOR_OWNER_STATE_SET, /* 检查当前执行条件。 */
                                    LOAD_MOTOR_PRIORITY_STATE_SET, /* 定义 LOAD_MOTOR_PRIORITY_STATE_SET 枚举项。 */
                                    gripper_target, /* 传入下一项参数或数据。 */
                                    preset_timeout_ms + STATE_SET_GRIPPER_OWNER_EXTRA_MS)) /* 继续当前语句。 */
        {
            StateSet_FinishRequest(task_request.seq, false); /* 调用 StateSet_FinishRequest。 */
            continue; /* 跳过本轮剩余处理。 */
        }
        while (!IsInDeadzoneTimedF(Motor_GetTotalAngle(RM_3508_GRIPPER), gripper_target, MOTOR_DEAD_ZONE, preset_timeout_ms, &preset_timer)) /* 条件满足时继续执行。 */
        {
            if (AngleMotor_IsFaulted(RM_3508_GRIPPER)) /* 检查当前执行条件。 */
            {
                break; /* 结束当前循环或分支。 */
            }
            vTaskDelay(pdMS_TO_TICKS(10)); /* 调用 vTaskDelay。 */
        }
        if (AngleMotor_IsFaulted(RM_3508_GRIPPER)) /* 检查当前执行条件。 */
        {
            LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_STATE_SET); /* 调用 LoadMotor_ReleaseOwner。 */
            StateSet_FinishRequest(task_request.seq, false); /* 调用 StateSet_FinishRequest。 */
            continue; /* 跳过本轮剩余处理。 */
        }
        LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_STATE_SET); /* 调用 LoadMotor_ReleaseOwner。 */

        // 4310 Yaw 调整(不需要额外的任务)
        // if (!StateSet_ApplyYawPreset(preseting_yaw))
        // {
        //     StateSet_FinishRequest(task_request.seq, false);
        //     continue;
        // }

        // 事件组唤醒所有其他的任务
        if (!state_ready_once) /* 检查当前执行条件。 */
        {
            xEventGroupSetBits(g_pxStateSetEventGroupHandle, EVENT_ALL_READY); /* 调用 xEventGroupSetBits。 */
        }

        // 置成功设定标志位
        FireControl_MarkSetupApplied(); /* 调用 FireControl_MarkSetupApplied。 */
        StateSet_FinishRequest(task_request.seq, true); /* 调用 StateSet_FinishRequest。 */
        state_ready_once = true; /* 更新 state_ready_once。 */

        vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
    }
}

#define STATE_SET_DM_YAW_DEAD_ZONE 1.0f /* 定义 STATE_SET_DM_YAW_DEAD_ZONE。 */
#define STATE_SET_DM_YAW_REISSUE_MS 20U /* 定义 STATE_SET_DM_YAW_REISSUE_MS。 */
#define STATE_SET_DM_YAW_TIMEOUT_MS 3000U /* 定义 STATE_SET_DM_YAW_TIMEOUT_MS。 */

bool StateSet_ApplyYawPreset(float target_yaw) /* 实现 StateSet_ApplyYawPreset。 */
{
    float degree = 0.0f; /* 初始化 degree。 */
    uint32_t preset_yaw_reissue_tick = HAL_GetTick(); /* 初始化 preset_yaw_reissue_tick。 */
    uint32_t preset_yaw_start_tick = preset_yaw_reissue_tick; /* 初始化 preset_yaw_start_tick。 */
    DeadzoneState_t yaw_preset_deadzone = {0}; /* 初始化 yaw_preset_deadzone。 */

    if (target_yaw < DM_J4310_Class.pos_min) /* 检查当前执行条件。 */
    {
        target_yaw = DM_J4310_Class.pos_min; /* 更新 target_yaw。 */
    }
    else if (target_yaw > DM_J4310_Class.pos_max) /* 继续判断下一条件。 */
    {
        target_yaw = DM_J4310_Class.pos_max; /* 更新 target_yaw。 */
    }

    DM_MotorEnable(DM_4310_YAW); /* 调用 DM_MotorEnable。 */
    DmMotorPID_Calc(DM_4310_YAW, target_yaw); /* 调用 DmMotorPID_Calc。 */
    while (1) /* 持续执行当前任务。 */
    {
        degree = Motor_GetTotalAngle(DM_4310_YAW); /* 更新 degree。 */
        if (IsInDeadzoneF(degree, target_yaw, STATE_SET_DM_YAW_DEAD_ZONE, &yaw_preset_deadzone, false)) /* 检查当前执行条件。 */
        {
            return true; /* 返回 true。 */
        }

        if ((uint32_t)(HAL_GetTick() - preset_yaw_start_tick) >= STATE_SET_DM_YAW_TIMEOUT_MS) /* 检查当前执行条件。 */
        {
            return false; /* 返回 false。 */
        }

        DM_Motor_RefreshData(DM_4310_YAW); /* 调用 DM_Motor_RefreshData。 */
        if ((uint32_t)(HAL_GetTick() - preset_yaw_reissue_tick) >= STATE_SET_DM_YAW_REISSUE_MS) /* 检查当前执行条件。 */
        {
            DmMotorPID_Calc(DM_4310_YAW, target_yaw); /* 调用 DmMotorPID_Calc。 */
            preset_yaw_reissue_tick = HAL_GetTick(); /* 更新 preset_yaw_reissue_tick。 */
        }

        vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
    }
}
