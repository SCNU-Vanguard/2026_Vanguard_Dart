#include "LoadTaskFunc.h"
#include "angle_motor.h"

extern MotorManager_t MotorManager; /* 声明外部变量 MotorManager。 */
extern float MotorData; /* 声明外部变量 MotorData。 */
extern volatile bool g_bStoreMotorSafeReturnPending; /* 声明外部变量 g_bStoreMotorSafeReturnPending。 */
volatile uint8_t g_ucLastReleasedServoGroup = 0U; /* 初始化 g_ucLastReleasedServoGroup。 */
volatile uint8_t LoadNumState = 0; /* 初始化 LoadNumState。 */

static volatile float g_LoadTrapCmdPosData = 0.0f; /* 初始化 g_LoadTrapCmdPosData。 */
static volatile float g_LoadTrapCmdVelData = 0.0f; /* 初始化 g_LoadTrapCmdVelData。 */
static volatile float g_LoadTrapCmdAccData = 0.0f; /* 初始化 g_LoadTrapCmdAccData。 */
static volatile float g_LoadTrapDtData = 0.0f; /* 初始化 g_LoadTrapDtData。 */
volatile uint32_t g_ulLoadRequestSeq = 0U; /* 初始化 g_ulLoadRequestSeq。 */
static volatile LoadAck_t g_LoadAck = {0U, LOAD_RESULT_NONE}; /* 初始化 g_LoadAck。 */

// 文件内前置声明：避免先调用、后定义导致的隐式声明报错。
static bool LoadRequest_IsValid(const LoadRequest_t *request); /* 声明 LoadRequest_IsValid 接口。 */
static uint8_t LoadDart_GetServoGroup(uint8_t dart_num); /* 声明 LoadDart_GetServoGroup 接口。 */
// 跨任务状态接口：实现位于 StoreEnergyTaskFunc.c。
extern bool StoreEnergy_IsLoadAbortRequested(void); /* 声明 StoreEnergy_IsLoadAbortRequested 接口。 */
extern void StoreEnergy_SetLoadCycleState(bool active, bool abort_pending); /* 声明 StoreEnergy_SetLoadCycleState 接口。 */

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

// 换弹结构电机队列
extern QueueHandle_t g_xLoad3508QueueHandler; /* 声明外部变量 g_xLoad3508QueueHandler。 */

static bool LoadTask_ReceiveRequest(LoadRequest_t *request, TickType_t timeout_ticks) /* 实现 LoadTask_ReceiveRequest。 */
{
    size_t received; /* 保存 received。 */

    if (request == NULL || xLoadStreamBuf == NULL) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }

    memset(request, 0, sizeof(*request)); /* 调用 memset。 */
    received = xStreamBufferReceive(xLoadStreamBuf, request, sizeof(*request), timeout_ticks); /* 更新 received。 */
    if (received != sizeof(*request)) /* 检查当前执行条件。 */
    {
        (void)xStreamBufferReset(xLoadStreamBuf); /* 调用 xStreamBufferReset。 */
        return false; /* 返回 false。 */
    }

    return LoadRequest_IsValid(request); /* 返回当前计算结果。 */
}

uint16_t LoadRequest_CalcChecksum(const LoadRequest_t *request) /* 实现 LoadRequest_CalcChecksum。 */
{
    uint32_t checksum; /* 保存 checksum。 */

    if (request == NULL) /* 检查当前执行条件。 */
    {
        return 0U; /* 返回状态值 0U。 */
    }

    checksum = request->magic ^ /* 继续更新 checksum。 */
               request->seq ^ /* 继续组合表达式。 */
               request->timestamp_ms ^ /* 继续组合表达式。 */
               ((uint32_t)request->dart_num << 24) ^ /* 继续组合表达式。 */
               ((uint32_t)request->priority << 16); /* 完成本行操作。 */
    return (uint16_t)((checksum & 0xFFFFU) ^ (checksum >> 16)); /* 返回当前计算结果。 */
}

static bool LoadRequest_IsValid(const LoadRequest_t *request) /* 实现 LoadRequest_IsValid。 */
{
    if (request == NULL || /* 检查当前执行条件。 */
        request->magic != LOAD_REQUEST_MAGIC || /* 继续组合表达式。 */
        request->checksum != LoadRequest_CalcChecksum(request) || /* 继续组合表达式。 */
        request->dart_num < 1U || /* 继续组合表达式。 */
        request->dart_num > 4U) /* 继续当前语句。 */
    {
        return false; /* 返回 false。 */
    }

    if ((uint32_t)(HAL_GetTick() - request->timestamp_ms) > LOAD_REQUEST_MAX_AGE_MS) /* 检查当前执行条件。 */
    {
        return false; /* 返回 false。 */
    }

    return true; /* 返回 true。 */
}

void LoadAck_Set(uint32_t seq, LoadResult_e result) /* 实现 LoadAck_Set。 */
{
    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    g_LoadAck.seq = seq; /* 更新 seq。 */
    g_LoadAck.result = result; /* 更新 result。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
}

LoadAck_t LoadAck_Get(void) /* 实现 LoadAck_Get。 */
{
    LoadAck_t ack; /* 保存 ack。 */

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    ack.seq = g_LoadAck.seq; /* 更新 seq。 */
    ack.result = g_LoadAck.result; /* 更新 result。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    return ack; /* 返回当前计算结果。 */
}

/***********************************
 * @brief 换弹到位后的通用处理：通知舵机、清积分、回零、通知储能
 * @param dart_num 当前飞镖编号
 * @param m_offset_angle 电机偏移角度
 **********************************/
static inline void LoadDart_ReturnToZero(uint8_t dart_num, float m_offset_angle, uint32_t request_seq) /* 实现 LoadDart_ReturnToZero。 */
{
    (void)m_offset_angle; /* 显式忽略参数 m_offset_angle。 */

    // 等待两侧储能电机到位后再释放舵机，避免夹爪先到位但储能机构未到位。
    xSemaphoreTake(g_xStoreMotorArrivedSemHandle, portMAX_DELAY); /* 调用 xSemaphoreTake。 */
    if (StoreEnergy_IsLoadAbortRequested() || AngleMotor_IsFaulted(RM_3508_GRIPPER)) /* 检查当前执行条件。 */
    {
        StoreEnergy_SetLoadCycleState(false, false); /* 调用 StoreEnergy_SetLoadCycleState。 */
        LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD); /* 调用 LoadMotor_ReleaseOwner。 */
        LoadAck_Set(request_seq, LOAD_RESULT_ABORTED); /* 调用 LoadAck_Set。 */
        xSemaphoreGive(g_xLoad2StoreSemHandle); /* 调用 xSemaphoreGive。 */
        return; /* 结束当前函数。 */
    }
    uint8_t servo_group = LoadDart_GetServoGroup(dart_num); /* 初始化 servo_group。 */
    if (servo_group == 0U || /* 检查当前执行条件。 */
        xQueueSend(g_xLoad3508QueueHandler, (const void *)&servo_group, 0) != pdTRUE) /* 继续更新 目标值。 */
    {
        StoreEnergy_SetLoadCycleState(false, false); /* 调用 StoreEnergy_SetLoadCycleState。 */
        LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD); /* 调用 LoadMotor_ReleaseOwner。 */
        LoadAck_Set(request_seq, LOAD_RESULT_FAILED); /* 调用 LoadAck_Set。 */
        xSemaphoreGive(g_xLoad2StoreSemHandle); /* 调用 xSemaphoreGive。 */
        return; /* 结束当前函数。 */
    }
    vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS)); /* 调用 vTaskDelay。 */
    if (StoreEnergy_IsLoadAbortRequested() || AngleMotor_IsFaulted(RM_3508_GRIPPER)) /* 检查当前执行条件。 */
    {
        StoreEnergy_SetLoadCycleState(false, false); /* 调用 StoreEnergy_SetLoadCycleState。 */
        LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD); /* 调用 LoadMotor_ReleaseOwner。 */
        LoadAck_Set(request_seq, LOAD_RESULT_ABORTED); /* 调用 LoadAck_Set。 */
        xSemaphoreGive(g_xLoad2StoreSemHandle); /* 调用 xSemaphoreGive。 */
        return; /* 结束当前函数。 */
    }

    // Bias空间
    // 双储能电机同步运动时，默认关闭 S 型规划，直接走 PID。
    Motor_SetUseSCurve(RM_3508_STORE_LEFT, false); /* 调用 Motor_SetUseSCurve。 */
    Motor_SetUseSCurve(RM_3508_STORE_RIGHT, false); /* 调用 Motor_SetUseSCurve。 */
    Motor_SetTarget(RM_3508_STORE_LEFT, (float)(LeftStoreLoad + 10.0f)); /* 调用 Motor_SetTarget。 */
    Motor_SetTarget(RM_3508_STORE_RIGHT, (float)(RightStoreLoad - 10.0f)); /* 调用 Motor_SetTarget。 */
    Motor_EnableControl(RM_3508_STORE_LEFT, true); /* 调用 Motor_EnableControl。 */
    Motor_EnableControl(RM_3508_STORE_RIGHT, true); /* 调用 Motor_EnableControl。 */
    vTaskDelay(pdMS_TO_TICKS(3)); /* 调用 vTaskDelay。 */
    vTaskDelay(pdMS_TO_TICKS(3)); /* 调用 vTaskDelay。 */

    uint32_t bottom_reissue_tick = HAL_GetTick(); /* 初始化 bottom_reissue_tick。 */
    DeadzoneTimer_t bottom_left_timer_1 = {0}; /* 初始化 bottom_left_timer_1。 */
    DeadzoneTimer_t bottom_right_timer_1 = {0}; /* 初始化 bottom_right_timer_1。 */
    while (1) /* 持续执行当前任务。 */
    {
        float left_pos = Motor_GetTotalAngle(RM_3508_STORE_LEFT); /* 初始化 left_pos。 */
        float right_pos = Motor_GetTotalAngle(RM_3508_STORE_RIGHT); /* 初始化 right_pos。 */

        if ((uint32_t)(HAL_GetTick() - bottom_reissue_tick) >= 20U) /* 检查当前执行条件。 */
        {
            Motor_SetTarget(RM_3508_STORE_LEFT, (float)(LeftStoreLoad + 350.0f)); /* 调用 Motor_SetTarget。 */
            vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
            Motor_SetTarget(RM_3508_STORE_RIGHT, (float)(RightStoreLoad - 350.0f)); /* 调用 Motor_SetTarget。 */
            vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
            bottom_reissue_tick = HAL_GetTick(); /* 更新 bottom_reissue_tick。 */
        }

        if (IsInDeadzoneTimedF(left_pos, (float)(LeftStoreLoad + 350.0f), /* 检查当前执行条件。 */
                               MOTOR_DEAD_ZONE, MOTOR_DEADZONE_TIMEOUT_MS, /* 传入下一项参数或数据。 */
                               &bottom_left_timer_1) && /* 继续组合表达式。 */
            IsInDeadzoneTimedF(right_pos, (float)(RightStoreLoad - 350.0f), /* 传入下一项参数或数据。 */
                               MOTOR_DEAD_ZONE, MOTOR_DEADZONE_TIMEOUT_MS, /* 传入下一项参数或数据。 */
                               &bottom_right_timer_1)) /* 继续当前语句。 */
        {
            break; /* 结束当前循环或分支。 */
        }
        if (Motor_IsAnyStoreProtected()) /* 检查当前执行条件。 */
        {
            g_bStoreMotorSafeReturnPending = true; /* 更新 g_bStoreMotorSafeReturnPending。 */
            StoreEnergy_SetLoadCycleState(false, false); /* 调用 StoreEnergy_SetLoadCycleState。 */
            LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD); /* 调用 LoadMotor_ReleaseOwner。 */
            LoadAck_Set(request_seq, LOAD_RESULT_ABORTED); /* 调用 LoadAck_Set。 */
            xSemaphoreGive(g_xLoad2StoreSemHandle); /* 调用 xSemaphoreGive。 */
            return; /* 结束当前函数。 */
        }
        if (StoreEnergy_IsLoadAbortRequested() || AngleMotor_IsFaulted(RM_3508_GRIPPER)) /* 检查当前执行条件。 */
        {
            StoreEnergy_SetLoadCycleState(false, false); /* 调用 StoreEnergy_SetLoadCycleState。 */
            LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD); /* 调用 LoadMotor_ReleaseOwner。 */
            LoadAck_Set(request_seq, LOAD_RESULT_ABORTED); /* 调用 LoadAck_Set。 */
            xSemaphoreGive(g_xLoad2StoreSemHandle); /* 调用 xSemaphoreGive。 */
            return; /* 结束当前函数。 */
        }
        vTaskDelay(pdMS_TO_TICKS(2)); /* 调用 vTaskDelay。 */
    }

    // 新换弹结构无需回0
    Motor_SetTarget(RM_3508_STORE_LEFT, LeftStoreBottom); /* 调用 Motor_SetTarget。 */
    vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
    Motor_SetTarget(RM_3508_STORE_RIGHT, RightStoreBottom); /* 调用 Motor_SetTarget。 */
    vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
    bottom_reissue_tick = HAL_GetTick(); /* 更新 bottom_reissue_tick。 */
    DeadzoneTimer_t bottom_left_timer_2 = {0}; /* 初始化 bottom_left_timer_2。 */
    DeadzoneTimer_t bottom_right_timer_2 = {0}; /* 初始化 bottom_right_timer_2。 */
    while (1) /* 持续执行当前任务。 */
    {
        float left_pos = Motor_GetTotalAngle(RM_3508_STORE_LEFT); /* 初始化 left_pos。 */
        float right_pos = Motor_GetTotalAngle(RM_3508_STORE_RIGHT); /* 初始化 right_pos。 */

        if ((uint32_t)(HAL_GetTick() - bottom_reissue_tick) >= 20U) /* 检查当前执行条件。 */
        {
            Motor_SetTarget(RM_3508_STORE_LEFT, LeftStoreBottom); /* 调用 Motor_SetTarget。 */
            vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
            Motor_SetTarget(RM_3508_STORE_RIGHT, RightStoreBottom); /* 调用 Motor_SetTarget。 */
            vTaskDelay(pdMS_TO_TICKS(1)); /* 调用 vTaskDelay。 */
            bottom_reissue_tick = HAL_GetTick(); /* 更新 bottom_reissue_tick。 */
        }

        if (IsInDeadzoneTimedF(left_pos, LeftStoreBottom, /* 检查当前执行条件。 */
                               MOTOR_DEAD_ZONE, MOTOR_DEADZONE_TIMEOUT_MS, /* 传入下一项参数或数据。 */
                               &bottom_left_timer_2) && /* 继续组合表达式。 */
            IsInDeadzoneTimedF(right_pos, RightStoreBottom, /* 传入下一项参数或数据。 */
                               MOTOR_DEAD_ZONE, MOTOR_DEADZONE_TIMEOUT_MS, /* 传入下一项参数或数据。 */
                               &bottom_right_timer_2)) /* 继续当前语句。 */
        {
            break; /* 结束当前循环或分支。 */
        }
        if (Motor_IsAnyStoreProtected()) /* 检查当前执行条件。 */
        {
            g_bStoreMotorSafeReturnPending = true; /* 更新 g_bStoreMotorSafeReturnPending。 */
            StoreEnergy_SetLoadCycleState(false, false); /* 调用 StoreEnergy_SetLoadCycleState。 */
            LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD); /* 调用 LoadMotor_ReleaseOwner。 */
            LoadAck_Set(request_seq, LOAD_RESULT_ABORTED); /* 调用 LoadAck_Set。 */
            xSemaphoreGive(g_xLoad2StoreSemHandle); /* 调用 xSemaphoreGive。 */
            return; /* 结束当前函数。 */
        }
        if (StoreEnergy_IsLoadAbortRequested() || AngleMotor_IsFaulted(RM_3508_GRIPPER)) /* 检查当前执行条件。 */
        {
            StoreEnergy_SetLoadCycleState(false, false); /* 调用 StoreEnergy_SetLoadCycleState。 */
            LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD); /* 调用 LoadMotor_ReleaseOwner。 */
            LoadAck_Set(request_seq, LOAD_RESULT_ABORTED); /* 调用 LoadAck_Set。 */
            xSemaphoreGive(g_xLoad2StoreSemHandle); /* 调用 xSemaphoreGive。 */
            return; /* 结束当前函数。 */
        }
        vTaskDelay(pdMS_TO_TICKS(2)); /* 调用 vTaskDelay。 */
    }

    StoreEnergy_SetLoadCycleState(false, false); /* 调用 StoreEnergy_SetLoadCycleState。 */
    LoadAck_Set(request_seq, LOAD_RESULT_DONE); /* 调用 LoadAck_Set。 */
    xSemaphoreGive(g_xLoad2StoreSemHandle); /* 调用 xSemaphoreGive。 */
    LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD); /* 调用 LoadMotor_ReleaseOwner。 */
    if (dart_num == 1) /* 检查当前执行条件。 */
        vTaskDelay(pdMS_TO_TICKS(200)); /* 调用 vTaskDelay。 */
}

float Load3508_GetReloadOffset(uint8_t step_count) /* 实现 Load3508_GetReloadOffset。 */
{
    if (step_count == 0U) /* 检查当前执行条件。 */
    {
        return 0.0f; /* 返回当前计算结果。 */
    }

    if (step_count > 3U) /* 检查当前执行条件。 */
    {
        step_count = 3U; /* 更新 step_count。 */
    }

    return LOAD3508_DART_STEP_DIR * /* 继续当前语句。 */
           (LOAD3508_DART_FIRST_STEP_DEG + ((float)(step_count - 1U) * LOAD3508_DART_STEP_DEG)); /* 完成本行操作。 */
}

static uint8_t LoadDart_GetServoGroup(uint8_t dart_num) /* 实现 LoadDart_GetServoGroup。 */
{
    if (dart_num < 1U || dart_num > 3U) /* 检查当前执行条件。 */
    {
        return 0U; /* 返回状态值 0U。 */
    }

    return (uint8_t)(4U - dart_num); /* 返回当前计算结果。 */
}

void LoadMotorTaskMainLoopFunc(void) /* 实现 LoadMotorTaskMainLoopFunc。 */
{
    bool MutexTake = false; /* 初始化 MutexTake。 */
    float load_3508_offset_angle = 0.0f; /* 初始化 load_3508_offset_angle。 */
    while (1) /* 持续执行当前任务。 */
    {
        xSemaphoreTake(g_xStore2LoadSemHandle, portMAX_DELAY); /* 调用 xSemaphoreTake。 */
        LoadRequest_t load_request = {0}; /* 初始化 load_request。 */
        if (!LoadTask_ReceiveRequest(&load_request, pdMS_TO_TICKS(50))) /* 检查当前执行条件。 */
        {
            LoadAck_Set(load_request.seq, LOAD_RESULT_FAILED); /* 调用 LoadAck_Set。 */
            xSemaphoreGive(g_xLoad2StoreSemHandle); /* 调用 xSemaphoreGive。 */
            continue; /* 跳过本轮剩余处理。 */
        }
        LoadAck_Set(load_request.seq, LOAD_RESULT_ACCEPTED); /* 调用 LoadAck_Set。 */
        vTaskDelay(pdMS_TO_TICKS(500)); // 本来是175的,但是这个时候换弹过快
        float cur_target = Motor_GetTarget(RM_3508_GRIPPER); /* 初始化 cur_target。 */
        switch (load_request.dart_num) /* 按当前状态选择处理分支。 */
        {
        case 4: /* 处理 4 分支。 */
            cur_target = PresetLoc + load_3508_offset_angle; /* 更新 cur_target。 */
            MutexTake = false; /* 定义 MutexTake 枚举项。 */
            break; /* 结束当前循环或分支。 */
        case 3: /* 处理 3 分支。 */
            cur_target = PresetLoc + load_3508_offset_angle + Load3508_GetReloadOffset(1U); /* 更新 cur_target。 */
            MutexTake = true; /* 定义 MutexTake 枚举项。 */
            break; /* 结束当前循环或分支。 */
        case 2: /* 处理 2 分支。 */
            cur_target = PresetLoc + load_3508_offset_angle + Load3508_GetReloadOffset(2U); /* 更新 cur_target。 */
            MutexTake = true; /* 定义 MutexTake 枚举项。 */
            break; /* 结束当前循环或分支。 */
        case 1: /* 处理 1 分支。 */
            cur_target = PresetLoc + load_3508_offset_angle + Load3508_GetReloadOffset(3U); /* 更新 cur_target。 */
            MutexTake = true; /* 定义 MutexTake 枚举项。 */
            break; /* 结束当前循环或分支。 */
        default: /* 处理默认分支。 */
            LoadAck_Set(load_request.seq, LOAD_RESULT_FAILED); /* 调用 LoadAck_Set。 */
            xSemaphoreGive(g_xLoad2StoreSemHandle); /* 调用 xSemaphoreGive。 */
            continue; /* 跳过本轮剩余处理。 */
        }
        uint8_t last_released_servo_group = 0U; /* 初始化 last_released_servo_group。 */
        taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
        last_released_servo_group = g_ucLastReleasedServoGroup; /* 更新 last_released_servo_group。 */
        taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
        if (MutexTake && last_released_servo_group >= 1U && last_released_servo_group <= 3U) /* 检查当前执行条件。 */
        {
            (void)Servo_MoveDartGroupToZero(last_released_servo_group, SERVO_MOVE_TIME_MS); /* 调用 Servo_MoveDartGroupToZero。 */
            vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS)); /* 调用 vTaskDelay。 */
            taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
            if (g_ucLastReleasedServoGroup == last_released_servo_group) /* 检查当前执行条件。 */
            {
                g_ucLastReleasedServoGroup = 0U; /* 更新 g_ucLastReleasedServoGroup。 */
            }
            taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
        }
        DeadzoneTimer_t reach_timer = {0}; /* 初始化 reach_timer。 */
        uint32_t reach_timeout_ms = CalcTrapMoveTimeoutMs( /* 开始计算 reach_timeout_ms。 */
            Motor_GetTotalAngle(RM_3508_GRIPPER), /* 传入下一项参数或数据。 */
            cur_target, /* 传入下一项参数或数据。 */
            LOAD_TASK_TRAP_VMAX_DEG_S, /* 定义 LOAD_TASK_TRAP_VMAX_DEG_S 枚举项。 */
            LOAD_TASK_TRAP_AMAX_DEG_S2, /* 定义 LOAD_TASK_TRAP_AMAX_DEG_S2 枚举项。 */
            LOAD_DEADZONE_TIMEOUT_MS, /* 定义 LOAD_DEADZONE_TIMEOUT_MS 枚举项。 */
            12000U); /* 完成本行操作。 */
        if (!LoadMotor_SubmitTarget(LOAD_MOTOR_OWNER_LOAD, /* 检查当前执行条件。 */
                                    LOAD_MOTOR_PRIORITY_LOAD, /* 定义 LOAD_MOTOR_PRIORITY_LOAD 枚举项。 */
                                    cur_target, /* 传入下一项参数或数据。 */
                                    reach_timeout_ms + 1000U)) /* 继续当前语句。 */
        {
            LoadAck_Set(load_request.seq, LOAD_RESULT_FAILED); /* 调用 LoadAck_Set。 */
            LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD); /* 调用 LoadMotor_ReleaseOwner。 */
            xSemaphoreGive(g_xLoad2StoreSemHandle); /* 调用 xSemaphoreGive。 */
            continue; /* 跳过本轮剩余处理。 */
        }
        if (!MutexTake) /* 检查当前执行条件。 */
        {
            LoadAck_Set(load_request.seq, LOAD_RESULT_DONE); /* 调用 LoadAck_Set。 */
            xSemaphoreGive(g_xLoad2StoreSemHandle); /* 调用 xSemaphoreGive。 */
            LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD); /* 调用 LoadMotor_ReleaseOwner。 */
        }

        // 这里再加一个互斥量进行保证
        while (MutexTake) /* 条件满足时继续执行。 */
        {
            MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER); /* 定义 MotorData 枚举项。 */
            if (AngleMotor_IsFaulted(RM_3508_GRIPPER)) /* 检查当前执行条件。 */
            {
                MutexTake = false; /* 定义 MutexTake 枚举项。 */
                Motor_EnableControl(RM_3508_GRIPPER, false); /* 调用 Motor_EnableControl。 */
                StoreEnergy_SetLoadCycleState(false, false); /* 调用 StoreEnergy_SetLoadCycleState。 */
                LoadAck_Set(load_request.seq, LOAD_RESULT_ABORTED); /* 调用 LoadAck_Set。 */
                LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD); /* 调用 LoadMotor_ReleaseOwner。 */
                xSemaphoreGive(g_xLoad2StoreSemHandle); /* 调用 xSemaphoreGive。 */
                break; /* 结束当前循环或分支。 */
            }
            if (IsInDeadzoneTimedF(MotorData, cur_target, /* 检查当前执行条件。 */
                                   MOTOR_DEAD_ZONE, reach_timeout_ms, /* 传入下一项参数或数据。 */
                                   &reach_timer)) /* 继续当前语句。 */
            {
                MutexTake = false; /* 定义 MutexTake 枚举项。 */
                LoadDart_ReturnToZero(load_request.dart_num, load_3508_offset_angle, load_request.seq); // 现在无需回零
            }
            vTaskDelay(pdMS_TO_TICKS(2)); /* 调用 vTaskDelay。 */
        }
    }
}

void LoadServoTaskMainLoopFunc(void) /* 实现 LoadServoTaskMainLoopFunc。 */
{
    uint8_t fQueueDartNum = 0; // 目标为0最初
    while (1) /* 持续执行当前任务。 */
    {
        xQueueReceive(g_xLoad3508QueueHandler, &fQueueDartNum, portMAX_DELAY); /* 调用 xQueueReceive。 */
        if (fQueueDartNum >= 1 && fQueueDartNum <= 3) /* 检查当前执行条件。 */
        {
            if (Servo_ReleaseDartGroup(fQueueDartNum, SERVO_MOVE_TIME_MS)) /* 检查当前执行条件。 */
            {
                vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS)); /* 调用 vTaskDelay。 */
                // 换弹标志位
                taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
                LoadNumState++; /* 递增 LoadNumState。 */
                g_ucLastReleasedServoGroup = fQueueDartNum; /* 更新 g_ucLastReleasedServoGroup。 */
                taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
            }
            vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS)); /* 调用 vTaskDelay。 */
        }
    }
}
