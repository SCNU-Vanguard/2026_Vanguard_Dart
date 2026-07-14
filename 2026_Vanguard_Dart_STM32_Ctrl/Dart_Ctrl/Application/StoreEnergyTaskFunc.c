#include "StoreEnergyTaskFunc.h"
#include "angle_motor.h"

extern MotorManager_t MotorManager; /* 声明外部变量 MotorManager。 */

// 储能任务消息
extern StreamBufferHandle_t xLoadStreamBuf; /* 声明外部变量 xLoadStreamBuf。 */

// Store <-> Load 双向同步信号量
extern StaticSemaphore_t g_xStore2LoadSemBuffer; /* 声明外部变量 g_xStore2LoadSemBuffer。 */
extern SemaphoreHandle_t g_xStore2LoadSemHandle; // Store通知Load开始
extern StaticSemaphore_t g_xLoad2StoreSemBuffer; /* 声明外部变量 g_xLoad2StoreSemBuffer。 */
extern SemaphoreHandle_t g_xLoad2StoreSemHandle; // Load通知Store完成

// Store -> Load: 储能电机到位通知信号量（换弹时舵机需等储能机构到位才释放）
extern StaticSemaphore_t g_xStoreMotorArrivedSemBuffer; /* 声明外部变量 g_xStoreMotorArrivedSemBuffer。 */
extern SemaphoreHandle_t g_xStoreMotorArrivedSemHandle; /* 声明外部变量 g_xStoreMotorArrivedSemHandle。 */

extern volatile uint32_t g_ulLoadRequestSeq; /* 声明外部变量 g_ulLoadRequestSeq。 */

extern volatile uint8_t g_ucLastReleasedServoGroup; /* 声明外部变量 g_ucLastReleasedServoGroup。 */

extern volatile uint8_t g_ucActiveDartNum; /* 声明外部变量 g_ucActiveDartNum。 */

// 这几项接口分别由 LoadTask / StateSet 模块实现，这里只做跨模块调用声明。
extern uint16_t LoadRequest_CalcChecksum(const LoadRequest_t *request); /* 声明 LoadRequest_CalcChecksum 接口。 */
extern void LoadAck_Set(uint32_t seq, LoadResult_e result); /* 声明 LoadAck_Set 接口。 */
extern LoadAck_t LoadAck_Get(void); /* 声明 LoadAck_Get 接口。 */
extern bool StateSet_RequestAndWait(uint8_t dart_num, TickType_t timeout_ticks); /* 声明 StateSet_RequestAndWait 接口。 */

extern volatile bool g_bStoreMotorSafeReturnPending; /* 声明外部变量 g_bStoreMotorSafeReturnPending。 */

static volatile bool g_bLoadCycleActive = false; /* 初始化 g_bLoadCycleActive。 */
static volatile bool g_bLoadCycleAbort = false; /* 初始化 g_bLoadCycleAbort。 */
static volatile bool g_bLauncherOccupied = false; /* 初始化 g_bLauncherOccupied。 */
static volatile bool g_bLauncherReloadedDart = false; /* 初始化 g_bLauncherReloadedDart。 */

StoreEnergyFlowState_t flow_state = STORE_FLOW_WAIT_SETUP; /* 初始化 flow_state。 */
StoreEnergyFlowState_t safe_return_next_state = STORE_FLOW_READY; /* 初始化 safe_return_next_state。 */

static bool StoreEnergy_SendLoadRequest(uint8_t dart_num, LoadRequest_t *request_out) /* 实现 StoreEnergy_SendLoadRequest。 */
{
    LoadRequest_t request = {0}; /* 初始化 request。 */
    size_t sent; /* 保存 sent。 */

    if (xLoadStreamBuf == NULL || /* 检查当前执行条件。 */
        g_xStore2LoadSemHandle == NULL || /* 继续更新 目标值。 */
        dart_num < 1U || /* 继续组合表达式。 */
        dart_num > 4U) /* 继续当前语句。 */
    {
        return false; /* 返回 false。 */
    }

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    g_ulLoadRequestSeq++; /* 递增 g_ulLoadRequestSeq。 */
    request.seq = g_ulLoadRequestSeq; /* 更新 seq。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    request.magic = LOAD_REQUEST_MAGIC; /* 更新 magic。 */
    request.timestamp_ms = HAL_GetTick(); /* 更新 timestamp_ms。 */
    request.dart_num = dart_num; /* 更新 dart_num。 */
    request.priority = LOAD_REQUEST_PRIORITY_NORMAL; /* 更新 priority。 */
    request.checksum = LoadRequest_CalcChecksum(&request); /* 更新 checksum。 */

    (void)xSemaphoreTake(g_xStore2LoadSemHandle, 0); /* 调用 xSemaphoreTake。 */
    (void)xSemaphoreTake(g_xLoad2StoreSemHandle, 0); /* 调用 xSemaphoreTake。 */
    (void)xStreamBufferReset(xLoadStreamBuf); /* 调用 xStreamBufferReset。 */
    LoadAck_Set(request.seq, LOAD_RESULT_NONE); /* 调用 LoadAck_Set。 */

    sent = xStreamBufferSend(xLoadStreamBuf, &request, sizeof(request), 0); /* 更新 sent。 */
    if (sent != sizeof(request)) /* 检查当前执行条件。 */
    {
        LoadAck_Set(request.seq, LOAD_RESULT_FAILED); /* 调用 LoadAck_Set。 */
        return false; /* 返回 false。 */
    }

    if (request_out != NULL) /* 检查当前执行条件。 */
    {
        *request_out = request; /* 更新 request_out。 */
    }
    xSemaphoreGive(g_xStore2LoadSemHandle); /* 调用 xSemaphoreGive。 */
    return true; /* 返回 true。 */
}

/**
 * @brief 控制左右储能 M3508 同步移动到指定目标
 * @note  用于裁判系统禁止发射时的保守回退动作
 */
static void StoreEnergy_MoveStoreMotorPairToTargets(float left_target, float right_target) /* 实现 StoreEnergy_MoveStoreMotorPairToTargets。 */
{
    uint32_t reissue_tick = HAL_GetTick(); /* 初始化 reissue_tick。 */
    DeadzoneState_t left_deadzone = {0}; /* 初始化 left_deadzone。 */
    DeadzoneState_t right_deadzone = {0}; /* 初始化 right_deadzone。 */

    // 这里统一使用新的储能 M3508 兼容层，默认关闭 S 型规划
    Motor_SetUseSCurve(RM_3508_STORE_LEFT, false); /* 调用 Motor_SetUseSCurve。 */
    Motor_SetUseSCurve(RM_3508_STORE_RIGHT, false); /* 调用 Motor_SetUseSCurve。 */
    Motor_SetTarget(RM_3508_STORE_LEFT, left_target); /* 调用 Motor_SetTarget。 */
    Motor_SetTarget(RM_3508_STORE_RIGHT, right_target); /* 调用 Motor_SetTarget。 */
    Motor_EnableControl(RM_3508_STORE_LEFT, true); /* 调用 Motor_EnableControl。 */
    Motor_EnableControl(RM_3508_STORE_RIGHT, true); /* 调用 Motor_EnableControl。 */
    vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */

    while (1) /* 持续执行当前任务。 */
    {
        vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
        if ((uint32_t)(HAL_GetTick() - reissue_tick) >= 20U) /* 检查当前执行条件。 */
        {
            Motor_SetTarget(RM_3508_STORE_LEFT, left_target); /* 调用 Motor_SetTarget。 */
            vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
            Motor_SetTarget(RM_3508_STORE_RIGHT, right_target); /* 调用 Motor_SetTarget。 */
            vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
            reissue_tick = HAL_GetTick(); /* 更新 reissue_tick。 */
        }

        if (IsInDeadzoneF(Motor_GetTotalAngle(RM_3508_STORE_LEFT), left_target, MOTOR_DEAD_ZONE, &left_deadzone, false) && /* 检查当前执行条件。 */
            IsInDeadzoneF(Motor_GetTotalAngle(RM_3508_STORE_RIGHT), right_target, MOTOR_DEAD_ZONE, &right_deadzone, false)) /* 开始调用 IsInDeadzoneF。 */
        {
            break; /* 结束当前循环或分支。 */
        }

        if (Motor_IsAnyStoreProtected()) /* 检查当前执行条件。 */
        {
            g_bStoreMotorSafeReturnPending = true; /* 更新 g_bStoreMotorSafeReturnPending。 */
            break; /* 结束当前循环或分支。 */
        }
    }
}

/// @brief 设置发射机构飞镖状态
/// @param occupied true->发射机构上已有飞镖
/// @param reloaded true->当前飞镖来自弹夹换弹
static void StoreEnergy_SetLauncherDartState(bool occupied, bool reloaded) /* 实现 StoreEnergy_SetLauncherDartState。 */
{
    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    g_bLauncherOccupied = occupied; /* 更新 g_bLauncherOccupied。 */
    g_bLauncherReloadedDart = occupied && reloaded; /* 更新 g_bLauncherReloadedDart。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
}

/// @brief 确认发射机构上是否已有飞镖
/// @return true->发射机构上已有飞镖
static bool StoreEnergy_HasLauncherDart(void) /* 实现 StoreEnergy_HasLauncherDart。 */
{
    bool occupied; /* 保存 occupied。 */

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    occupied = g_bLauncherOccupied; /* 更新 occupied。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    return occupied; /* 返回当前计算结果。 */
}

/**
 * @brief 禁止发射时的安全退出：先释放扳机，再让储能 M3508 上移
 * @note  不扣弹、不清发射位占用，避免下次允许发射时重复换弹肘击
 */
static void StoreEnergy_ParkLauncherDartSafely(void) /* 实现 StoreEnergy_ParkLauncherDartSafely。 */
{
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_shoot); /* 调用 __HAL_TIM_SET_COMPARE。 */
    vTaskDelay(pdMS_TO_TICKS(1500)); /* 调用 vTaskDelay。 */
    StoreEnergy_MoveStoreMotorPairToTargets(LeftStoreTop, RightStoreTop); /* 调用 StoreEnergy_MoveStoreMotorPairToTargets。 */
}

/**
 * @brief 将两侧储能 M3508 拉回零位（Top/0位）
 * @note  用于裁判系统禁止发射时的保守回退动作
 */
static void StoreEnergy_ReturnStoreMotorToZero(void) /* 实现 StoreEnergy_ReturnStoreMotorToZero。 */
{
    StoreEnergy_MoveStoreMotorPairToTargets(LeftStoreTop, RightStoreTop); /* 调用 StoreEnergy_MoveStoreMotorPairToTargets。 */
}

static void StoreEnergy_ReturnLoad3508Home(void) /* 实现 StoreEnergy_ReturnLoad3508Home。 */
{
    float home_pos = 0.0f; /* 初始化 home_pos。 */
    DeadzoneTimer_t home_timer = {0}; /* 初始化 home_timer。 */
    uint32_t home_timeout_ms = CalcTrapMoveTimeoutMs( /* 开始计算 home_timeout_ms。 */
        Motor_GetTotalAngle(RM_3508_GRIPPER), /* 传入下一项参数或数据。 */
        home_pos, /* 传入下一项参数或数据。 */
        LOAD_TASK_TRAP_VMAX_DEG_S, /* 定义 LOAD_TASK_TRAP_VMAX_DEG_S 枚举项。 */
        LOAD_TASK_TRAP_AMAX_DEG_S2, /* 定义 LOAD_TASK_TRAP_AMAX_DEG_S2 枚举项。 */
        LOAD_DEADZONE_TIMEOUT_MS, /* 定义 LOAD_DEADZONE_TIMEOUT_MS 枚举项。 */
        STATE_SET_GRIPPER_SETUP_MAX_TIMEOUT_MS); /* 完成本行操作。 */

    AngleMotor_ClearFault(RM_3508_GRIPPER); /* 调用 AngleMotor_ClearFault。 */
    Motor_EnableControl(RM_3508_GRIPPER, true); /* 调用 Motor_EnableControl。 */
    if (!LoadMotor_SubmitTarget(LOAD_MOTOR_OWNER_HOME, /* 检查当前执行条件。 */
                                LOAD_MOTOR_PRIORITY_HOME, /* 定义 LOAD_MOTOR_PRIORITY_HOME 枚举项。 */
                                home_pos, /* 传入下一项参数或数据。 */
                                home_timeout_ms + STATE_SET_GRIPPER_OWNER_EXTRA_MS)) /* 继续当前语句。 */
    {
        Motor_EnableControl(RM_3508_GRIPPER, false); /* 调用 Motor_EnableControl。 */
        return; /* 结束当前函数。 */
    }

    while (!IsInDeadzoneTimedF(Motor_GetTotalAngle(RM_3508_GRIPPER), /* 条件满足时继续执行。 */
                               home_pos, /* 传入下一项参数或数据。 */
                               MOTOR_DEAD_ZONE, /* 定义 MOTOR_DEAD_ZONE 枚举项。 */
                               home_timeout_ms, /* 传入下一项参数或数据。 */
                               &home_timer)) /* 继续当前语句。 */
    {
        if (AngleMotor_IsFaulted(RM_3508_GRIPPER)) /* 检查当前执行条件。 */
        {
            break; /* 结束当前循环或分支。 */
        }
        vTaskDelay(pdMS_TO_TICKS(2)); /* 调用 vTaskDelay。 */
    }
    LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_HOME); /* 调用 LoadMotor_ReleaseOwner。 */
    Motor_EnableControl(RM_3508_GRIPPER, false); /* 调用 Motor_EnableControl。 */
}

/// @brief 确认当前储能 M3508 是否仍由 Store 流程控制
/// @param active true代表储能流程正常控制
/// @param abort_pending true代表本轮流程已请求中断
void StoreEnergy_SetLoadCycleState(bool active, bool abort_pending) /* 实现 StoreEnergy_SetLoadCycleState。 */
{
    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    g_bLoadCycleActive = active; /* 更新 g_bLoadCycleActive。 */
    g_bLoadCycleAbort = abort_pending; /* 更新 g_bLoadCycleAbort。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
}

/// @brief 外部有没有要求立刻中止这轮换弹联动
/// @param 无
/// @return true->打断当前流程，直接安全回退
bool StoreEnergy_IsLoadAbortRequested(void) /* 实现 StoreEnergy_IsLoadAbortRequested。 */
{
    bool abort_requested; /* 保存 abort_requested。 */

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    abort_requested = g_bLoadCycleAbort; /* 更新 abort_requested。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    return abort_requested; /* 返回当前计算结果。 */
}

/// @brief 终止换弹同时归还控制权
/// @param  无
static void StoreEnergy_RequestLoadAbort(void) /* 实现 StoreEnergy_RequestLoadAbort。 */
{
    bool need_release = false; /* 初始化 need_release。 */

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    if (g_bLoadCycleActive) /* 检查当前执行条件。 */
    {
        g_bLoadCycleAbort = true; /* 更新 g_bLoadCycleAbort。 */
        need_release = true; /* 更新 need_release。 */
    }
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    if (need_release && g_xStoreMotorArrivedSemHandle != NULL) /* 检查当前执行条件。 */
    {
        xSemaphoreGive(g_xStoreMotorArrivedSemHandle); /* 调用 xSemaphoreGive。 */
    }
}

/// @brief 收回控制权，同时让储能 M3508 回零
/// @param  无
static void StoreEnergy_HandleSafeReturn(void) /* 实现 StoreEnergy_HandleSafeReturn。 */
{
    uint32_t wait_start_tick = HAL_GetTick(); /* 初始化 wait_start_tick。 */

    StoreEnergy_RequestLoadAbort(); /* 调用 StoreEnergy_RequestLoadAbort。 */
    while (g_bLoadCycleActive && /* 条件满足时继续执行。 */
           (uint32_t)(HAL_GetTick() - wait_start_tick) < LOAD_DEADZONE_TIMEOUT_MS) /* 继续当前语句。 */
    {
        if (g_xLoad2StoreSemHandle != NULL && /* 检查当前执行条件。 */
            xSemaphoreTake(g_xLoad2StoreSemHandle, pdMS_TO_TICKS(10)) == pdTRUE) /* 继续更新 目标值。 */
        {
            break; /* 结束当前循环或分支。 */
        }
    }
    if (g_bStoreMotorSafeReturnPending) /* 检查当前执行条件。 */
    {
        AngleMotor_ClearFault(RM_3508_STORE_LEFT); /* 调用 AngleMotor_ClearFault。 */
        AngleMotor_ClearFault(RM_3508_STORE_RIGHT); /* 调用 AngleMotor_ClearFault。 */
        StoreEnergy_MoveStoreMotorPairToTargets(LeftSafe, RightSafe); /* 调用 StoreEnergy_MoveStoreMotorPairToTargets。 */
        g_bStoreMotorSafeReturnPending = false; /* 更新 g_bStoreMotorSafeReturnPending。 */
    }
    else /* 处理其余情况。 */
    {
        StoreEnergy_ReturnStoreMotorToZero(); /* 调用 StoreEnergy_ReturnStoreMotorToZero。 */
    }
    Servo_MoveAllToZero(SERVO_MOVE_TIME_MS); /* 调用 Servo_MoveAllToZero。 */
    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    g_ucLastReleasedServoGroup = 0U; /* 更新 g_ucLastReleasedServoGroup。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
    vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS)); /* 调用 vTaskDelay。 */
    StoreEnergy_ReturnLoad3508Home(); /* 调用 StoreEnergy_ReturnLoad3508Home。 */
}

void StoreEnergyTaskMainLoopFuc(void) /* 实现 StoreEnergyTaskMainLoopFuc。 */
{
    uint8_t Dart = 4; /* 初始化 Dart。 */
    vTaskDelay(pdMS_TO_TICKS(POWER_ON_DELAY_MS)); /* 调用 vTaskDelay。 */
    float left_pos = 0.0f, right_pos = 0.0f; /* 初始化 left_pos。 */
    FirePermission_t permission = {0}; /* 初始化 permission。 */
    while (1) /* 持续执行当前任务。 */
    {
        // 获取电机控制权（与ControlTask互斥）
        xSemaphoreTake(g_xMotorCtrlSemHandle, portMAX_DELAY); /* 调用 xSemaphoreTake。 */

        // 检查是否被手动覆盖，如果是则释放控制权并等待
        if (Motor_IsAnyStoreProtected()) /* 检查当前执行条件。 */
        {
            g_bStoreMotorSafeReturnPending = true; /* 更新 g_bStoreMotorSafeReturnPending。 */
            safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
            flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
        }
        else if (AngleMotor_IsFaulted(RM_3508_GRIPPER)) /* 继续判断下一条件。 */
        {
            safe_return_next_state = STORE_FLOW_SAFE_RETURN; /* 更新 safe_return_next_state。 */
            flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
        }
        else if (ControlState_IsManualOverride()) /* 继续判断下一条件。 */
        {
            xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
            vTaskDelay(pdMS_TO_TICKS(50)); /* 调用 vTaskDelay。 */
            continue; /* 跳过本轮剩余处理。 */
        }
        if (flow_state != STORE_FLOW_SAFE_RETURN && Dart == 0U) /* 检查当前执行条件。 */
        {
            xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
            vTaskSuspend(StoreEnergyTaskHandle); /* 调用 vTaskSuspend。 */
            continue; /* 跳过本轮剩余处理。 */
        }

        switch (flow_state) /* 按当前状态选择处理分支。 */
        {
        case STORE_FLOW_WAIT_SETUP: /* 处理 STORE_FLOW_WAIT_SETUP 分支。 */
        {
            g_ucActiveDartNum = Dart; /* 更新 g_ucActiveDartNum。 */
            flow_state = STORE_FLOW_READY; /* 更新 flow_state。 */
            xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
            break; /* 结束当前循环或分支。 */
        }

        case STORE_FLOW_READY: /* 处理 STORE_FLOW_READY 分支。 */
        {
            // 过渡
            FireControl_GetPermission(&permission); /* 调用 FireControl_GetPermission。 */
            if (!permission.can_shoot) /* 检查当前执行条件。 */
            {
                xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
                vTaskDelay(pdMS_TO_TICKS(20)); /* 调用 vTaskDelay。 */
                break; /* 结束当前循环或分支。 */
            }

            if (StoreEnergy_HasLauncherDart()) /* 检查当前执行条件。 */
            {
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_store); /* 调用 __HAL_TIM_SET_COMPARE。 */
                vTaskDelay(pdMS_TO_TICKS(1500)); /* 调用 vTaskDelay。 */
                flow_state = STORE_FLOW_WAIT_SHOOT_PERMISSION; /* 更新 flow_state。 */
            }
            else /* 处理其余情况。 */
            {
                flow_state = STORE_FLOW_STORING; /* 更新 flow_state。 */
            }
            xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
            break; /* 结束当前循环或分支。 */
        }

        case STORE_FLOW_STORING: /* 处理 STORE_FLOW_STORING 分支。 */
        {
            uint32_t load_reissue_tick = HAL_GetTick(); /* 初始化 load_reissue_tick。 */
            DeadzoneState_t load_left_deadzone = {0}; /* 初始化 load_left_deadzone。 */
            DeadzoneState_t load_right_deadzone = {0}; /* 初始化 load_right_deadzone。 */
            LoadRequest_t load_request = {0}; /* 初始化 load_request。 */
            LoadResult_e load_result = LOAD_RESULT_NONE; /* 初始化 load_result。 */

            FireControl_GetPermission(&permission); /* 调用 FireControl_GetPermission。 */
            if (permission.abort_current_shot) /* 检查当前执行条件。 */
            {
                safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
                flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                break; /* 结束当前循环或分支。 */
            }

            // 清空上一轮可能残留的储能到位信号，避免本轮一开始就误判完成。
            (void)xSemaphoreTake(g_xStoreMotorArrivedSemHandle, 0); /* 调用 xSemaphoreTake。 */
            StoreEnergy_SetLoadCycleState(true, false); /* 调用 StoreEnergy_SetLoadCycleState。 */

            // 左右储能 M3508 默认直接走位置 PID，不启用 S 型规划。
            Motor_SetUseSCurve(RM_3508_STORE_LEFT, false); /* 调用 Motor_SetUseSCurve。 */
            Motor_SetUseSCurve(RM_3508_STORE_RIGHT, false); /* 调用 Motor_SetUseSCurve。 */
            Motor_SetTarget(RM_3508_STORE_LEFT, LeftStoreLoad);   // PID不够用
            Motor_SetTarget(RM_3508_STORE_RIGHT, RightStoreLoad); // PID不够用
            Motor_EnableControl(RM_3508_STORE_LEFT, true); /* 调用 Motor_EnableControl。 */
            Motor_EnableControl(RM_3508_STORE_RIGHT, true); /* 调用 Motor_EnableControl。 */
            vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
            vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */

            // 发出当前Dart的num,从而确认应该运动到什么地方
            if (!StoreEnergy_SendLoadRequest(Dart, &load_request)) /* 检查当前执行条件。 */
            {
                safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
                flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                break; /* 结束当前循环或分支。 */
            }

            // 死区检查、遥控允许发射检查、裁判系统发射检查
            while (1) /* 持续执行当前任务。 */
            {
                if (ControlState_IsManualOverride()) /* 手动接管时停止自动储能流程。 */
                {
                    safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
                    flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                    break; /* 结束当前循环或分支。 */
                }

                FireControl_GetPermission(&permission); /* 刷新裁判看门狗仲裁结果。 */
                if (permission.abort_current_shot)      /* 裁判超时或门禁关闭。 */
                {
                    safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
                    flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                    break; /* 结束当前循环或分支。 */
                }

                if (Motor_IsAnyStoreProtected()) /* CAN 离线或电机保护触发。 */
                {
                    g_bStoreMotorSafeReturnPending = true; /* 更新 g_bStoreMotorSafeReturnPending。 */
                    safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
                    flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                    break; /* 结束当前循环或分支。 */
                }

                vTaskDelay(pdMS_TO_TICKS(3)); /* 调用 vTaskDelay。 */
                left_pos = Motor_GetTotalAngle(RM_3508_STORE_LEFT); /* 更新 left_pos。 */
                right_pos = Motor_GetTotalAngle(RM_3508_STORE_RIGHT); /* 更新 right_pos。 */
                if ((uint32_t)(HAL_GetTick() - load_reissue_tick) >= 20U) /* 检查当前执行条件。 */
                {
                    Motor_SetTarget(RM_3508_STORE_LEFT, LeftStoreLoad); /* 调用 Motor_SetTarget。 */
                    vTaskDelay(pdMS_TO_TICKS(3)); /* 调用 vTaskDelay。 */
                    Motor_SetTarget(RM_3508_STORE_RIGHT, RightStoreLoad); /* 调用 Motor_SetTarget。 */
                    vTaskDelay(pdMS_TO_TICKS(3)); /* 调用 vTaskDelay。 */
                    load_reissue_tick = HAL_GetTick(); /* 更新 load_reissue_tick。 */
                }
                if (IsInDeadzoneF(left_pos, LeftStoreLoad, MOTOR_DEAD_ZONE, &load_left_deadzone, false) && /* 检查当前执行条件。 */
                    IsInDeadzoneF(right_pos, RightStoreLoad, MOTOR_DEAD_ZONE, &load_right_deadzone, false)) /* 开始调用 IsInDeadzoneF。 */
                {
                    break; /* 结束当前循环或分支。 */
                }
                vTaskDelay(pdMS_TO_TICKS(2)); /* 调用 vTaskDelay。 */
            }
            // vTaskDelay(2500);

            if (flow_state == STORE_FLOW_SAFE_RETURN) /* 检查当前执行条件。 */
            {
                break; /* 结束当前循环或分支。 */
            }

            // 储能电机到位之后等待换弹电机收尾，同时检查裁判系统和遥控器控制
            xSemaphoreGive(g_xStoreMotorArrivedSemHandle); /* 调用 xSemaphoreGive。 */
            uint32_t load_wait_start_tick = HAL_GetTick(); /* 初始化 load_wait_start_tick。 */
            while (load_result == LOAD_RESULT_NONE || load_result == LOAD_RESULT_ACCEPTED) /* 条件满足时继续执行。 */
            {
                if (xSemaphoreTake(g_xLoad2StoreSemHandle, pdMS_TO_TICKS(10)) == pdTRUE) /* 检查当前执行条件。 */
                {
                    LoadAck_t ack = LoadAck_Get(); /* 初始化 ack。 */
                    if (ack.seq == load_request.seq) /* 检查当前执行条件。 */
                    {
                        load_result = ack.result; /* 更新 load_result。 */
                        break; /* 结束当前循环或分支。 */
                    }
                    continue; /* 跳过本轮剩余处理。 */
                }

                if ((uint32_t)(HAL_GetTick() - load_wait_start_tick) >= LOAD_REQUEST_DONE_TIMEOUT_MS) /* 检查当前执行条件。 */
                {
                    load_result = LOAD_RESULT_FAILED; /* 更新 load_result。 */
                    break; /* 结束当前循环或分支。 */
                }

                if (Motor_IsAnyStoreProtected()) /* 检查当前执行条件。 */
                {
                    g_bStoreMotorSafeReturnPending = true; /* 更新 g_bStoreMotorSafeReturnPending。 */
                    load_result = LOAD_RESULT_ABORTED; /* 更新 load_result。 */
                    break; /* 结束当前循环或分支。 */
                }

                if (AngleMotor_IsFaulted(RM_3508_GRIPPER)) /* 检查当前执行条件。 */
                {
                    load_result = LOAD_RESULT_ABORTED; /* 更新 load_result。 */
                    break; /* 结束当前循环或分支。 */
                }

                if (ControlState_IsManualOverride()) /* 等待换弹时仍响应手动接管。 */
                {
                    load_result = LOAD_RESULT_ABORTED; /* 更新 load_result。 */
                    safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
                    flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                    break; /* 结束当前循环或分支。 */
                }

                FireControl_GetPermission(&permission); /* 等待期间持续刷新裁判门禁。 */
                if (permission.abort_current_shot)      /* 裁判超时立即安全返回。 */
                {
                    load_result = LOAD_RESULT_ABORTED; /* 更新 load_result。 */
                    safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
                    flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                    break; /* 结束当前循环或分支。 */
                }
            }

            if (flow_state == STORE_FLOW_SAFE_RETURN) /* 检查当前执行条件。 */
            {
                break; /* 结束当前循环或分支。 */
            }

            if (load_result == LOAD_RESULT_ABORTED) /* 检查当前执行条件。 */
            {
                safe_return_next_state = STORE_FLOW_SAFE_RETURN; /* 更新 safe_return_next_state。 */
                flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                break; /* 结束当前循环或分支。 */
            }

            if (load_result != LOAD_RESULT_DONE) /* 检查当前执行条件。 */
            {
                safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
                flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                break; /* 结束当前循环或分支。 */
            }

            StoreEnergy_SetLoadCycleState(false, false); /* 调用 StoreEnergy_SetLoadCycleState。 */
            StoreEnergy_SetLauncherDartState(true, Dart < 4U); /* 调用 StoreEnergy_SetLauncherDartState。 */
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_shoot); /* 调用 __HAL_TIM_SET_COMPARE。 */
            vTaskDelay(pdMS_TO_TICKS(1500)); /* 调用 vTaskDelay。 */
            flow_state = STORE_FLOW_WAIT_SHOOT_PERMISSION; /* 更新 flow_state。 */
            xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
            break; /* 结束当前循环或分支。 */
        }

        case STORE_FLOW_WAIT_SHOOT_PERMISSION: /* 处理 STORE_FLOW_WAIT_SHOOT_PERMISSION 分支。 */
        {
            // 过渡
            FireControl_GetPermission(&permission); /* 调用 FireControl_GetPermission。 */
            if (!permission.can_shoot) /* 检查当前执行条件。 */
            {
                if (StoreEnergy_HasLauncherDart()) /* 检查当前执行条件。 */
                {
                    StoreEnergy_ParkLauncherDartSafely(); /* 调用 StoreEnergy_ParkLauncherDartSafely。 */
                    flow_state = STORE_FLOW_WAIT_SETUP; /* 更新 flow_state。 */
                    xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
                    break; /* 结束当前循环或分支。 */
                }
                safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
                flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                break; /* 结束当前循环或分支。 */
            }
            flow_state = STORE_FLOW_FIRING; /* 更新 flow_state。 */
            xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
            break; /* 结束当前循环或分支。 */
        }

        case STORE_FLOW_FIRING: /* 处理 STORE_FLOW_FIRING 分支。 */
        {
            uint32_t top_reissue_tick = HAL_GetTick(); /* 初始化 top_reissue_tick。 */
            DeadzoneState_t top_left_deadzone = {0}; /* 初始化 top_left_deadzone。 */
            DeadzoneState_t top_right_deadzone = {0}; /* 初始化 top_right_deadzone。 */

            FireControl_GetPermission(&permission); /* 调用 FireControl_GetPermission。 */
            if (permission.abort_current_shot) /* 检查当前执行条件。 */
            {
                if (StoreEnergy_HasLauncherDart()) /* 检查当前执行条件。 */
                {
                    StoreEnergy_ParkLauncherDartSafely(); /* 调用 StoreEnergy_ParkLauncherDartSafely。 */
                    flow_state = STORE_FLOW_WAIT_SETUP; /* 更新 flow_state。 */
                    xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
                    break; /* 结束当前循环或分支。 */
                }
                safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
                flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                break; /* 结束当前循环或分支。 */
            }

            Motor_SetUseSCurve(RM_3508_STORE_LEFT, false); /* 调用 Motor_SetUseSCurve。 */
            Motor_SetUseSCurve(RM_3508_STORE_RIGHT, false); /* 调用 Motor_SetUseSCurve。 */
            Motor_SetTarget(RM_3508_STORE_LEFT, LeftStoreTop); /* 调用 Motor_SetTarget。 */
            Motor_SetTarget(RM_3508_STORE_RIGHT, RightStoreTop); /* 调用 Motor_SetTarget。 */
            Motor_EnableControl(RM_3508_STORE_LEFT, true); /* 调用 Motor_EnableControl。 */
            Motor_EnableControl(RM_3508_STORE_RIGHT, true); /* 调用 Motor_EnableControl。 */
            vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */

            while (1) /* 持续执行当前任务。 */
            {
                if (ControlState_IsManualOverride()) /* 检查当前执行条件。 */
                {
                    safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
                    flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                    break; /* 结束当前循环或分支。 */
                }

                FireControl_GetPermission(&permission); /* 调用 FireControl_GetPermission。 */
                if (permission.abort_current_shot) /* 检查当前执行条件。 */
                {
                    if (StoreEnergy_HasLauncherDart()) /* 检查当前执行条件。 */
                    {
                        StoreEnergy_ParkLauncherDartSafely(); /* 调用 StoreEnergy_ParkLauncherDartSafely。 */
                        flow_state = STORE_FLOW_WAIT_SETUP; /* 更新 flow_state。 */
                        break; /* 结束当前循环或分支。 */
                    }
                    safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
                    flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                    break; /* 结束当前循环或分支。 */
                }

                if (Motor_IsAnyStoreProtected()) /* 检查当前执行条件。 */
                {
                    g_bStoreMotorSafeReturnPending = true; /* 更新 g_bStoreMotorSafeReturnPending。 */
                    safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
                    flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                    break; /* 结束当前循环或分支。 */
                }

                left_pos = Motor_GetTotalAngle(RM_3508_STORE_LEFT); /* 更新 left_pos。 */
                right_pos = Motor_GetTotalAngle(RM_3508_STORE_RIGHT); /* 更新 right_pos。 */
                if ((uint32_t)(HAL_GetTick() - top_reissue_tick) >= 20U) /* 检查当前执行条件。 */
                {
                    Motor_SetTarget(RM_3508_STORE_LEFT, LeftStoreTop); /* 调用 Motor_SetTarget。 */
                    vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
                    Motor_SetTarget(RM_3508_STORE_RIGHT, RightStoreTop); /* 调用 Motor_SetTarget。 */
                    vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
                    top_reissue_tick = HAL_GetTick(); /* 更新 top_reissue_tick。 */
                }

                if (IsInDeadzoneF(left_pos, LeftStoreTop, MOTOR_DEAD_ZONE, &top_left_deadzone, false) && /* 检查当前执行条件。 */
                    IsInDeadzoneF(right_pos, RightStoreTop, MOTOR_DEAD_ZONE, &top_right_deadzone, false)) /* 开始调用 IsInDeadzoneF。 */
                {
                    break; /* 结束当前循环或分支。 */
                }
                vTaskDelay(pdMS_TO_TICKS(2)); /* 调用 vTaskDelay。 */
            }

            if (flow_state == STORE_FLOW_SAFE_RETURN) /* 检查当前执行条件。 */
            {
                break; /* 结束当前循环或分支。 */
            }
            if (flow_state == STORE_FLOW_WAIT_SETUP) /* 检查当前执行条件。 */
            {
                xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
                break; /* 结束当前循环或分支。 */
            }

            FireControl_GetPermission(&permission); /* 调用 FireControl_GetPermission。 */
            if (!permission.can_shoot) /* 检查当前执行条件。 */
            {
                if (StoreEnergy_HasLauncherDart()) /* 检查当前执行条件。 */
                {
                    StoreEnergy_ParkLauncherDartSafely(); /* 调用 StoreEnergy_ParkLauncherDartSafely。 */
                    flow_state = STORE_FLOW_WAIT_SETUP; /* 更新 flow_state。 */
                    xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
                    break; /* 结束当前循环或分支。 */
                }
                safe_return_next_state = STORE_FLOW_READY; /* 更新 safe_return_next_state。 */
                flow_state = STORE_FLOW_SAFE_RETURN; /* 更新 flow_state。 */
                break; /* 结束当前循环或分支。 */
            }

            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_store); /* 调用 __HAL_TIM_SET_COMPARE。 */
            vTaskDelay(pdMS_TO_TICKS(1500)); /* 调用 vTaskDelay。 */
            StoreEnergy_SetLauncherDartState(false, false); /* 调用 StoreEnergy_SetLauncherDartState。 */
            flow_state = STORE_FLOW_WAIT_SETUP; /* 更新 flow_state。 */
            Dart--; /* 递减 Dart。 */

            ControlState_StartDebugWindow(DEBUG_WINDOW_MS); /* 调用 ControlState_StartDebugWindow。 */
            xSemaphoreTake(g_xDebugFinishedSemHandle, 0); /* 调用 xSemaphoreTake。 */
            xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
            xSemaphoreTake(g_xDebugFinishedSemHandle, portMAX_DELAY); /* 调用 xSemaphoreTake。 */
            break; /* 结束当前循环或分支。 */
        }

        case STORE_FLOW_SAFE_RETURN: /* 处理 STORE_FLOW_SAFE_RETURN 分支。 */
        {
            break; /* 结束当前循环或分支。 */
        }

        default: /* 处理默认分支。 */
            flow_state = STORE_FLOW_WAIT_SETUP; /* 更新 flow_state。 */
            xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
            break; /* 结束当前循环或分支。 */
        }

        if (flow_state == STORE_FLOW_SAFE_RETURN) /* 检查当前执行条件。 */
        {
            StoreEnergy_HandleSafeReturn(); /* 调用 StoreEnergy_HandleSafeReturn。 */
            flow_state = safe_return_next_state; /* 更新 flow_state。 */
            xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
            if (safe_return_next_state == STORE_FLOW_SAFE_RETURN) /* 检查当前执行条件。 */
            {
                vTaskSuspend(StoreEnergyTaskHandle); /* 调用 vTaskSuspend。 */
            }
            vTaskDelay(pdMS_TO_TICKS(20)); /* 调用 vTaskDelay。 */
            continue; /* 跳过本轮剩余处理。 */
        }

        if (Dart == 0) /* 检查当前执行条件。 */
        {
            vTaskDelay(750); /* 调用 vTaskDelay。 */
            HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET); /* 调用 HAL_GPIO_WritePin。 */
            // 重新获取控制权进行收尾
            xSemaphoreTake(g_xMotorCtrlSemHandle, portMAX_DELAY); /* 调用 xSemaphoreTake。 */
            // 3508回到原始位置（offset_ecd_angle）
            StoreEnergy_ReturnStoreMotorToZero(); /* 调用 StoreEnergy_ReturnStoreMotorToZero。 */
            Servo_MoveAllToZero(SERVO_MOVE_TIME_MS); /* 调用 Servo_MoveAllToZero。 */
            taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
            g_ucLastReleasedServoGroup = 0U; /* 更新 g_ucLastReleasedServoGroup。 */
            taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
            vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS)); /* 调用 vTaskDelay。 */
            StoreEnergy_ReturnLoad3508Home(); /* 调用 StoreEnergy_ReturnLoad3508Home。 */

            Motor_EnableControl(RM_3508_STORE_LEFT, false); /* 调用 Motor_EnableControl。 */
            vTaskDelay(pdMS_TO_TICKS(7)); /* 调用 vTaskDelay。 */
            Motor_EnableControl(RM_3508_STORE_RIGHT, false); /* 调用 Motor_EnableControl。 */
            vTaskDelay(pdMS_TO_TICKS(7)); /* 调用 vTaskDelay。 */
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
            Motor_SetTarget(RM_2006_TRIGGER, 0.0f); /* 调用 Motor_SetTarget。 */
            Motor_EnableControl(RM_2006_TRIGGER, true); /* 调用 Motor_EnableControl。 */
            DeadzoneState_t trigger_preset_deadzone = {0}; /* 初始化 trigger_preset_deadzone。 */
            while (!IsInDeadzoneF(Motor_GetTotalAngle(RM_2006_TRIGGER), 0.0f, MOTOR_DEAD_ZONE, &trigger_preset_deadzone, true)) /* 条件满足时继续执行。 */
            {
                vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
            }
            xSemaphoreGive(g_xMotorCtrlSemHandle); /* 调用 xSemaphoreGive。 */
            vTaskSuspend(StoreEnergyTaskHandle); /* 调用 vTaskSuspend。 */
        }
    }
}
