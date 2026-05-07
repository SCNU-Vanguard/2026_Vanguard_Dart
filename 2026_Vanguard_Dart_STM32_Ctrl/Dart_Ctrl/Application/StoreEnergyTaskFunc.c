#include "StoreEnergyTaskFunc.h"

extern MotorManager_t MotorManager;

// 储能任务消息
extern StreamBufferHandle_t xLoadStreamBuf;

// Store <-> Load 双向同步信号量
extern StaticSemaphore_t g_xStore2LoadSemBuffer;
extern SemaphoreHandle_t g_xStore2LoadSemHandle; // Store通知Load开始
extern StaticSemaphore_t g_xLoad2StoreSemBuffer;
extern SemaphoreHandle_t g_xLoad2StoreSemHandle; // Load通知Store完成

// Store -> Load: 储能电机到位通知信号量（换弹时舵机需等储能机构到位才释放）
extern StaticSemaphore_t g_xStoreMotorArrivedSemBuffer;
extern SemaphoreHandle_t g_xStoreMotorArrivedSemHandle;

extern volatile uint32_t g_ulLoadRequestSeq;

extern volatile uint8_t g_ucLastReleasedServoGroup;

extern volatile uint8_t g_ucActiveDartNum;

// 这几项接口分别由 LoadTask / StateSet 模块实现，这里只做跨模块调用声明。
extern uint16_t LoadRequest_CalcChecksum(const LoadRequest_t *request);
extern void LoadAck_Set(uint32_t seq, LoadResult_e result);
extern LoadAck_t LoadAck_Get(void);
extern bool StateSet_RequestAndWait(uint8_t dart_num, TickType_t timeout_ticks);

extern volatile bool g_bStoreMotorSafeReturnPending;

static volatile bool g_bLoadCycleActive = false;
static volatile bool g_bLoadCycleAbort = false;
static volatile bool g_bLauncherOccupied = false;
static volatile bool g_bLauncherReloadedDart = false;

StoreEnergyFlowState_t flow_state = STORE_FLOW_WAIT_SETUP;
StoreEnergyFlowState_t safe_return_next_state = STORE_FLOW_READY;

static bool StoreEnergy_SendLoadRequest(uint8_t dart_num, LoadRequest_t *request_out)
{
    LoadRequest_t request = {0};
    size_t sent;

    if (xLoadStreamBuf == NULL ||
        g_xStore2LoadSemHandle == NULL ||
        dart_num < 1U ||
        dart_num > 4U)
    {
        return false;
    }

    taskENTER_CRITICAL();
    g_ulLoadRequestSeq++;
    request.seq = g_ulLoadRequestSeq;
    taskEXIT_CRITICAL();

    request.magic = LOAD_REQUEST_MAGIC;
    request.timestamp_ms = HAL_GetTick();
    request.dart_num = dart_num;
    request.priority = LOAD_REQUEST_PRIORITY_NORMAL;
    request.checksum = LoadRequest_CalcChecksum(&request);

    (void)xSemaphoreTake(g_xStore2LoadSemHandle, 0);
    (void)xSemaphoreTake(g_xLoad2StoreSemHandle, 0);
    (void)xStreamBufferReset(xLoadStreamBuf);
    LoadAck_Set(request.seq, LOAD_RESULT_NONE);

    sent = xStreamBufferSend(xLoadStreamBuf, &request, sizeof(request), 0);
    if (sent != sizeof(request))
    {
        LoadAck_Set(request.seq, LOAD_RESULT_FAILED);
        return false;
    }

    if (request_out != NULL)
    {
        *request_out = request;
    }
    xSemaphoreGive(g_xStore2LoadSemHandle);
    return true;
}

/**
 * @brief 控制左右储能 M3508 同步移动到指定目标
 * @note  用于裁判系统禁止发射时的保守回退动作
 */
static void StoreEnergy_MoveStoreMotorPairToTargets(float left_target, float right_target)
{
    uint32_t reissue_tick = HAL_GetTick();
    DeadzoneState_t left_deadzone = {0};
    DeadzoneState_t right_deadzone = {0};

    // 这里统一使用新的储能 M3508 兼容层，默认关闭 S 型规划
    StoreMotor_SetUseSCurve(RM_3508_STORE_LEFT, false);
    StoreMotor_SetUseSCurve(RM_3508_STORE_RIGHT, false);
    StoreMotor_SetTarget(RM_3508_STORE_LEFT, left_target);
    StoreMotor_SetTarget(RM_3508_STORE_RIGHT, right_target);
    StoreMotor_EnableControl(RM_3508_STORE_LEFT, true);
    StoreMotor_EnableControl(RM_3508_STORE_RIGHT, true);
    vTaskDelay(pdMS_TO_TICKS(1));
    vTaskDelay(pdMS_TO_TICKS(1));

    while (1)
    {
        StoreMotor_RefreshData(RM_3508_STORE_LEFT);
        vTaskDelay(pdMS_TO_TICKS(1));
        StoreMotor_RefreshData(RM_3508_STORE_RIGHT);
        vTaskDelay(pdMS_TO_TICKS(1));

        if ((uint32_t)(HAL_GetTick() - reissue_tick) >= 20U)
        {
            StoreMotor_SetTarget(RM_3508_STORE_LEFT, left_target);
            vTaskDelay(pdMS_TO_TICKS(1));
            StoreMotor_SetTarget(RM_3508_STORE_RIGHT, right_target);
            vTaskDelay(pdMS_TO_TICKS(1));
            reissue_tick = HAL_GetTick();
        }

        if (IsInDeadzoneF(Motor_GetTotalAngle(RM_3508_STORE_LEFT), left_target, MOTOR_DEAD_ZONE, &left_deadzone, false) &&
            IsInDeadzoneF(Motor_GetTotalAngle(RM_3508_STORE_RIGHT), right_target, MOTOR_DEAD_ZONE, &right_deadzone, false))
        {
            break;
        }

        if (StoreMotor_IsAnyProtected())
        {
            g_bStoreMotorSafeReturnPending = true;
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

/// @brief 设置发射机构飞镖状态
/// @param occupied true->发射机构上已有飞镖
/// @param reloaded true->当前飞镖来自弹夹换弹
static void StoreEnergy_SetLauncherDartState(bool occupied, bool reloaded)
{
    taskENTER_CRITICAL();
    g_bLauncherOccupied = occupied;
    g_bLauncherReloadedDart = occupied && reloaded;
    taskEXIT_CRITICAL();
}

/// @brief 确认发射机构上是否已有飞镖
/// @return true->发射机构上已有飞镖
static bool StoreEnergy_HasLauncherDart(void)
{
    bool occupied;

    taskENTER_CRITICAL();
    occupied = g_bLauncherOccupied;
    taskEXIT_CRITICAL();

    return occupied;
}

/**
 * @brief 禁止发射时的安全退出：先释放扳机，再让储能 M3508 上移
 * @note  不扣弹、不清发射位占用，避免下次允许发射时重复换弹肘击
 */
static void StoreEnergy_ParkLauncherDartSafely(void)
{
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_shoot);
    vTaskDelay(pdMS_TO_TICKS(1500));
    StoreEnergy_MoveStoreMotorPairToTargets(LeftStoreTop, RightStoreTop);
}

/**
 * @brief 将两侧储能 M3508 拉回零位（Top/0位）
 * @note  用于裁判系统禁止发射时的保守回退动作
 */
static void StoreEnergy_ReturnStoreMotorToZero(void)
{
    StoreEnergy_MoveStoreMotorPairToTargets(LeftStoreTop, RightStoreTop);
}

static void StoreEnergy_ReturnLoad3508Home(void)
{
    float home_pos = 0.0f;
    DeadzoneTimer_t home_timer = {0};
    uint32_t home_timeout_ms = CalcTrapMoveTimeoutMs(
        Motor_GetTotalAngle(RM_3508_GRIPPER),
        home_pos,
        LOAD_TASK_TRAP_VMAX_DEG_S,
        LOAD_TASK_TRAP_AMAX_DEG_S2,
        LOAD_DEADZONE_TIMEOUT_MS,
        STATE_SET_GRIPPER_SETUP_MAX_TIMEOUT_MS);

    LoadMotor_ClearOverCurrentProtection();
    Motor3508_EnableControl(true);
    if (!LoadMotor_SubmitTarget(LOAD_MOTOR_OWNER_HOME,
                                LOAD_MOTOR_PRIORITY_HOME,
                                home_pos,
                                home_timeout_ms + STATE_SET_GRIPPER_OWNER_EXTRA_MS))
    {
        Motor3508_EnableControl(false);
        return;
    }

    while (!IsInDeadzoneTimedF(Motor_GetTotalAngle(RM_3508_GRIPPER),
                               home_pos,
                               MOTOR_DEAD_ZONE,
                               home_timeout_ms,
                               &home_timer))
    {
        if (LoadMotor_IsOverCurrentProtected())
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_HOME);
    Motor3508_EnableControl(false);
}

/// @brief 确认当前储能 M3508 是否仍由 Store 流程控制
/// @param active true代表储能流程正常控制
/// @param abort_pending true代表本轮流程已请求中断
void StoreEnergy_SetLoadCycleState(bool active, bool abort_pending)
{
    taskENTER_CRITICAL();
    g_bLoadCycleActive = active;
    g_bLoadCycleAbort = abort_pending;
    taskEXIT_CRITICAL();
}

/// @brief 外部有没有要求立刻中止这轮换弹联动
/// @param 无
/// @return true->打断当前流程，直接安全回退
bool StoreEnergy_IsLoadAbortRequested(void)
{
    bool abort_requested;

    taskENTER_CRITICAL();
    abort_requested = g_bLoadCycleAbort;
    taskEXIT_CRITICAL();

    return abort_requested;
}

/// @brief 终止换弹同时归还控制权
/// @param  无
static void StoreEnergy_RequestLoadAbort(void)
{
    bool need_release = false;

    taskENTER_CRITICAL();
    if (g_bLoadCycleActive)
    {
        g_bLoadCycleAbort = true;
        need_release = true;
    }
    taskEXIT_CRITICAL();

    if (need_release && g_xStoreMotorArrivedSemHandle != NULL)
    {
        xSemaphoreGive(g_xStoreMotorArrivedSemHandle);
    }
}

/// @brief 收回控制权，同时让储能 M3508 回零
/// @param  无
static void StoreEnergy_HandleSafeReturn(void)
{
    uint32_t wait_start_tick = HAL_GetTick();

    StoreEnergy_RequestLoadAbort();
    while (g_bLoadCycleActive &&
           (uint32_t)(HAL_GetTick() - wait_start_tick) < LOAD_DEADZONE_TIMEOUT_MS)
    {
        if (g_xLoad2StoreSemHandle != NULL &&
            xSemaphoreTake(g_xLoad2StoreSemHandle, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            break;
        }
    }
    if (g_bStoreMotorSafeReturnPending)
    {
        StoreMotor_ClearProtection();
        StoreEnergy_MoveStoreMotorPairToTargets(LeftSafe, RightSafe);
        g_bStoreMotorSafeReturnPending = false;
    }
    else
    {
        StoreEnergy_ReturnStoreMotorToZero();
    }
    Servo_MoveAllToZero(SERVO_MOVE_TIME_MS);
    taskENTER_CRITICAL();
    g_ucLastReleasedServoGroup = 0U;
    taskEXIT_CRITICAL();
    vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS));
    StoreEnergy_ReturnLoad3508Home();
}

void StoreEnergyTaskMainLoopFuc(void)
{
    uint8_t Dart = 4;
    vTaskDelay(pdMS_TO_TICKS(POWER_ON_DELAY_MS));
    float left_pos = 0.0f, right_pos = 0.0f;
    FirePermission_t permission = {0};
    while (1)
    {
        // 获取电机控制权（与ControlTask互斥）
        xSemaphoreTake(g_xMotorCtrlSemHandle, portMAX_DELAY);

        // 检查是否被手动覆盖，如果是则释放控制权并等待
        if (StoreMotor_IsAnyProtected())
        {
            g_bStoreMotorSafeReturnPending = true;
            safe_return_next_state = STORE_FLOW_READY;
            flow_state = STORE_FLOW_SAFE_RETURN;
        }
        else if (LoadMotor_IsOverCurrentProtected())
        {
            safe_return_next_state = STORE_FLOW_SAFE_RETURN;
            flow_state = STORE_FLOW_SAFE_RETURN;
        }
        else if (ControlState_IsManualOverride())
        {
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        if (flow_state != STORE_FLOW_SAFE_RETURN && Dart == 0U)
        {
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            vTaskSuspend(StoreEnergyTaskHandle);
            continue;
        }

        switch (flow_state)
        {
        case STORE_FLOW_WAIT_SETUP:
        {
            g_ucActiveDartNum = Dart;

            // 微调位置,这里微调位置无法对应基地具体的靶子
            // 注意这里每次case进入都要保证无论是否允许发射状态下电机控制权的及时释放
            if (!StateSet_RequestAndWait(Dart, pdMS_TO_TICKS(STATE_SET_REQUEST_TIMEOUT_MS)))
            {
                if (LoadMotor_IsOverCurrentProtected())
                {
                    safe_return_next_state = STORE_FLOW_SAFE_RETURN;
                    flow_state = STORE_FLOW_SAFE_RETURN;
                    break;
                }
                xSemaphoreGive(g_xMotorCtrlSemHandle);
                vTaskDelay(pdMS_TO_TICKS(20));
                break;
            }

            flow_state = STORE_FLOW_READY;
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            break;
        }

        case STORE_FLOW_READY:
        {
            // 过渡
            FireControl_GetPermission(&permission);
            if (!permission.can_shoot)
            {
                xSemaphoreGive(g_xMotorCtrlSemHandle);
                vTaskDelay(pdMS_TO_TICKS(20));
                break;
            }

            if (StoreEnergy_HasLauncherDart())
            {
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_store);
                vTaskDelay(pdMS_TO_TICKS(1500));
                flow_state = STORE_FLOW_WAIT_SHOOT_PERMISSION;
            }
            else
            {
                flow_state = STORE_FLOW_STORING;
            }
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            break;
        }

        case STORE_FLOW_STORING:
        {
            uint32_t load_reissue_tick = HAL_GetTick();
            DeadzoneState_t load_left_deadzone = {0};
            DeadzoneState_t load_right_deadzone = {0};
            LoadRequest_t load_request = {0};
            LoadResult_e load_result = LOAD_RESULT_NONE;

            FireControl_GetPermission(&permission);
            if (permission.abort_current_shot)
            {
                safe_return_next_state = STORE_FLOW_READY;
                flow_state = STORE_FLOW_SAFE_RETURN;
                break;
            }

            // 清空上一轮可能残留的储能到位信号，避免本轮一开始就误判完成。
            (void)xSemaphoreTake(g_xStoreMotorArrivedSemHandle, 0);
            StoreEnergy_SetLoadCycleState(true, false);

            // 左右储能 M3508 默认直接走位置 PID，不启用 S 型规划。
            StoreMotor_SetUseSCurve(RM_3508_STORE_LEFT, false);
            StoreMotor_SetUseSCurve(RM_3508_STORE_RIGHT, false);
            StoreMotor_SetTarget(RM_3508_STORE_LEFT, LeftStoreLoad);
            StoreMotor_SetTarget(RM_3508_STORE_RIGHT, RightStoreLoad);
            StoreMotor_EnableControl(RM_3508_STORE_LEFT, true);
            StoreMotor_EnableControl(RM_3508_STORE_RIGHT, true);
            vTaskDelay(pdMS_TO_TICKS(1));
            vTaskDelay(pdMS_TO_TICKS(1));

            // 发出当前Dart的num,从而确认应该运动到什么地方
            if (!StoreEnergy_SendLoadRequest(Dart, &load_request))
            {
                safe_return_next_state = STORE_FLOW_READY;
                flow_state = STORE_FLOW_SAFE_RETURN;
                break;
            }

            // 死区检查、遥控允许发射检查、裁判系统发射检查
            while (1)
            {
                if (ControlState_IsManualOverride())
                {
                    safe_return_next_state = STORE_FLOW_READY;
                    flow_state = STORE_FLOW_SAFE_RETURN;
                    break;
                }

                FireControl_GetPermission(&permission);
                if (permission.abort_current_shot)
                {
                    safe_return_next_state = STORE_FLOW_READY;
                    flow_state = STORE_FLOW_SAFE_RETURN;
                    break;
                }

                if (StoreMotor_IsAnyProtected())
                {
                    g_bStoreMotorSafeReturnPending = true;
                    safe_return_next_state = STORE_FLOW_READY;
                    flow_state = STORE_FLOW_SAFE_RETURN;
                    break;
                }

                StoreMotor_RefreshData(RM_3508_STORE_LEFT);
                vTaskDelay(pdMS_TO_TICKS(3));
                StoreMotor_RefreshData(RM_3508_STORE_RIGHT);
                vTaskDelay(pdMS_TO_TICKS(3));
                left_pos = Motor_GetTotalAngle(RM_3508_STORE_LEFT);
                right_pos = Motor_GetTotalAngle(RM_3508_STORE_RIGHT);
                if ((uint32_t)(HAL_GetTick() - load_reissue_tick) >= 20U)
                {
                    StoreMotor_SetTarget(RM_3508_STORE_LEFT, LeftStoreLoad);
                    vTaskDelay(pdMS_TO_TICKS(3));
                    StoreMotor_SetTarget(RM_3508_STORE_RIGHT, RightStoreLoad);
                    vTaskDelay(pdMS_TO_TICKS(3));
                    load_reissue_tick = HAL_GetTick();
                }
                if (IsInDeadzoneF(left_pos, LeftStoreLoad, MOTOR_DEAD_ZONE, &load_left_deadzone, false) &&
                    IsInDeadzoneF(right_pos, RightStoreLoad, MOTOR_DEAD_ZONE, &load_right_deadzone, false))
                {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(2));
            }

            if (flow_state == STORE_FLOW_SAFE_RETURN)
            {
                break;
            }

            // 储能电机到位之后等待换弹电机收尾，同时检查裁判系统和遥控器控制
            xSemaphoreGive(g_xStoreMotorArrivedSemHandle);
            uint32_t load_wait_start_tick = HAL_GetTick();
            while (load_result == LOAD_RESULT_NONE || load_result == LOAD_RESULT_ACCEPTED)
            {
                if (xSemaphoreTake(g_xLoad2StoreSemHandle, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    LoadAck_t ack = LoadAck_Get();
                    if (ack.seq == load_request.seq)
                    {
                        load_result = ack.result;
                        break;
                    }
                    continue;
                }

                if ((uint32_t)(HAL_GetTick() - load_wait_start_tick) >= LOAD_REQUEST_DONE_TIMEOUT_MS)
                {
                    load_result = LOAD_RESULT_FAILED;
                    break;
                }

                if (StoreMotor_IsAnyProtected())
                {
                    g_bStoreMotorSafeReturnPending = true;
                    load_result = LOAD_RESULT_ABORTED;
                    break;
                }

                if (LoadMotor_IsOverCurrentProtected())
                {
                    load_result = LOAD_RESULT_ABORTED;
                    break;
                }

                if (ControlState_IsManualOverride())
                {
                    safe_return_next_state = STORE_FLOW_READY;
                    flow_state = STORE_FLOW_SAFE_RETURN;
                    break;
                }

                FireControl_GetPermission(&permission);
                if (permission.abort_current_shot)
                {
                    safe_return_next_state = STORE_FLOW_READY;
                    flow_state = STORE_FLOW_SAFE_RETURN;
                    break;
                }
            }

            if (flow_state == STORE_FLOW_SAFE_RETURN)
            {
                break;
            }

            if (load_result == LOAD_RESULT_ABORTED)
            {
                safe_return_next_state = STORE_FLOW_SAFE_RETURN;
                flow_state = STORE_FLOW_SAFE_RETURN;
                break;
            }

            if (load_result != LOAD_RESULT_DONE)
            {
                safe_return_next_state = STORE_FLOW_READY;
                flow_state = STORE_FLOW_SAFE_RETURN;
                break;
            }

            StoreEnergy_SetLoadCycleState(false, false);
            StoreEnergy_SetLauncherDartState(true, Dart < 4U);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_store);
            vTaskDelay(pdMS_TO_TICKS(1500));
            flow_state = STORE_FLOW_WAIT_SHOOT_PERMISSION;
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            break;
        }

        case STORE_FLOW_WAIT_SHOOT_PERMISSION:
        {
            // 过渡
            FireControl_GetPermission(&permission);
            if (!permission.can_shoot)
            {
                if (StoreEnergy_HasLauncherDart())
                {
                    StoreEnergy_ParkLauncherDartSafely();
                    flow_state = STORE_FLOW_WAIT_SETUP;
                    xSemaphoreGive(g_xMotorCtrlSemHandle);
                    break;
                }
                safe_return_next_state = STORE_FLOW_READY;
                flow_state = STORE_FLOW_SAFE_RETURN;
                break;
            }
            flow_state = STORE_FLOW_FIRING;
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            break;
        }

        case STORE_FLOW_FIRING:
        {
            uint32_t top_reissue_tick = HAL_GetTick();
            DeadzoneState_t top_left_deadzone = {0};
            DeadzoneState_t top_right_deadzone = {0};

            FireControl_GetPermission(&permission);
            if (permission.abort_current_shot)
            {
                if (StoreEnergy_HasLauncherDart())
                {
                    StoreEnergy_ParkLauncherDartSafely();
                    flow_state = STORE_FLOW_WAIT_SETUP;
                    xSemaphoreGive(g_xMotorCtrlSemHandle);
                    break;
                }
                safe_return_next_state = STORE_FLOW_READY;
                flow_state = STORE_FLOW_SAFE_RETURN;
                break;
            }

            StoreMotor_SetUseSCurve(RM_3508_STORE_LEFT, false);
            StoreMotor_SetUseSCurve(RM_3508_STORE_RIGHT, false);
            StoreMotor_SetTarget(RM_3508_STORE_LEFT, LeftStoreTop);
            StoreMotor_SetTarget(RM_3508_STORE_RIGHT, RightStoreTop);
            StoreMotor_EnableControl(RM_3508_STORE_LEFT, true);
            StoreMotor_EnableControl(RM_3508_STORE_RIGHT, true);
            vTaskDelay(pdMS_TO_TICKS(1));
            vTaskDelay(pdMS_TO_TICKS(1));

            while (1)
            {
                if (ControlState_IsManualOverride())
                {
                    safe_return_next_state = STORE_FLOW_READY;
                    flow_state = STORE_FLOW_SAFE_RETURN;
                    break;
                }

                FireControl_GetPermission(&permission);
                if (permission.abort_current_shot)
                {
                    if (StoreEnergy_HasLauncherDart())
                    {
                        StoreEnergy_ParkLauncherDartSafely();
                        flow_state = STORE_FLOW_WAIT_SETUP;
                        break;
                    }
                    safe_return_next_state = STORE_FLOW_READY;
                    flow_state = STORE_FLOW_SAFE_RETURN;
                    break;
                }

                if (StoreMotor_IsAnyProtected())
                {
                    g_bStoreMotorSafeReturnPending = true;
                    safe_return_next_state = STORE_FLOW_READY;
                    flow_state = STORE_FLOW_SAFE_RETURN;
                    break;
                }

                StoreMotor_RefreshData(RM_3508_STORE_LEFT);
                StoreMotor_RefreshData(RM_3508_STORE_RIGHT);
                left_pos = Motor_GetTotalAngle(RM_3508_STORE_LEFT);
                right_pos = Motor_GetTotalAngle(RM_3508_STORE_RIGHT);
                if ((uint32_t)(HAL_GetTick() - top_reissue_tick) >= 20U)
                {
                    StoreMotor_SetTarget(RM_3508_STORE_LEFT, LeftStoreTop);
                    vTaskDelay(pdMS_TO_TICKS(1));
                    StoreMotor_SetTarget(RM_3508_STORE_RIGHT, RightStoreTop);
                    vTaskDelay(pdMS_TO_TICKS(1));
                    top_reissue_tick = HAL_GetTick();
                }

                if (IsInDeadzoneF(left_pos, LeftStoreTop, MOTOR_DEAD_ZONE, &top_left_deadzone, false) &&
                    IsInDeadzoneF(right_pos, RightStoreTop, MOTOR_DEAD_ZONE, &top_right_deadzone, false))
                {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(2));
            }

            if (flow_state == STORE_FLOW_SAFE_RETURN)
            {
                break;
            }
            if (flow_state == STORE_FLOW_WAIT_SETUP)
            {
                xSemaphoreGive(g_xMotorCtrlSemHandle);
                break;
            }

            FireControl_GetPermission(&permission);
            if (!permission.can_shoot)
            {
                if (StoreEnergy_HasLauncherDart())
                {
                    StoreEnergy_ParkLauncherDartSafely();
                    flow_state = STORE_FLOW_WAIT_SETUP;
                    xSemaphoreGive(g_xMotorCtrlSemHandle);
                    break;
                }
                safe_return_next_state = STORE_FLOW_READY;
                flow_state = STORE_FLOW_SAFE_RETURN;
                break;
            }

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_shoot);
            vTaskDelay(pdMS_TO_TICKS(1500));
            StoreEnergy_SetLauncherDartState(false, false);
            flow_state = STORE_FLOW_WAIT_SETUP;
            Dart--;

            ControlState_StartDebugWindow(DEBUG_WINDOW_MS);
            xSemaphoreTake(g_xDebugFinishedSemHandle, 0);
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            xSemaphoreTake(g_xDebugFinishedSemHandle, portMAX_DELAY);
            break;
        }

        case STORE_FLOW_SAFE_RETURN:
        {
            break;
        }

        default:
            flow_state = STORE_FLOW_WAIT_SETUP;
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            break;
        }

        if (flow_state == STORE_FLOW_SAFE_RETURN)
        {
            StoreEnergy_HandleSafeReturn();
            flow_state = safe_return_next_state;
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            if (safe_return_next_state == STORE_FLOW_SAFE_RETURN)
            {
                vTaskSuspend(StoreEnergyTaskHandle);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (Dart == 0)
        {
            vTaskDelay(750);
            HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
            // 重新获取控制权进行收尾
            xSemaphoreTake(g_xMotorCtrlSemHandle, portMAX_DELAY);
            // 3508回到原始位置（offset_ecd_angle）
            StoreEnergy_ReturnStoreMotorToZero();
            Servo_MoveAllToZero(SERVO_MOVE_TIME_MS);
            taskENTER_CRITICAL();
            g_ucLastReleasedServoGroup = 0U;
            taskEXIT_CRITICAL();
            vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS));
            StoreEnergy_ReturnLoad3508Home();

            StoreMotor_Disable(RM_3508_STORE_LEFT);
            vTaskDelay(pdMS_TO_TICKS(7));
            StoreMotor_Disable(RM_3508_STORE_RIGHT);
            vTaskDelay(pdMS_TO_TICKS(7));
            // 6020 yaw axis is not connected now.
            // Motor6020_SetTarget(0.0f);
            // Motor6020_EnableControl(true);
            // #if ENABLE_6020_YAW_WAIT
            // DeadzoneState_t yaw_zero_deadzone = {0};
            // while (!IsInDeadzoneF(Motor_GetTotalAngle(RM_6020_YAW), 0.0f, YAW_6020_DEAD_ZONE, &yaw_zero_deadzone, true))
            // {
            //     vTaskDelay(pdMS_TO_TICKS(2));
            // }
            // #endif
            // Motor6020_EnableControl(false);
            Motor2006_SetTarget(0.0f);
            Motor2006_EnableControl(true);
            DeadzoneState_t trigger_preset_deadzone = {0};
            while (!IsInDeadzoneF(Motor_GetTotalAngle(RM_2006_TRIGGER), 0.0f, MOTOR_DEAD_ZONE, &trigger_preset_deadzone, true))
            {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            vTaskSuspend(StoreEnergyTaskHandle);
        }
    }
}
