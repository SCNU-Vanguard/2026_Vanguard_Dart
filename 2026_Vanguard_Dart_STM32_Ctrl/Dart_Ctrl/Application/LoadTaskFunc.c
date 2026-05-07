#include "LoadTaskFunc.h"

extern MotorManager_t MotorManager;
extern float MotorData;
extern volatile bool g_bStoreMotorSafeReturnPending;
volatile uint8_t g_ucLastReleasedServoGroup = 0U;
volatile uint8_t LoadNumState = 0;

static volatile float g_LoadTrapCmdPosData = 0.0f;
static volatile float g_LoadTrapCmdVelData = 0.0f;
static volatile float g_LoadTrapCmdAccData = 0.0f;
static volatile float g_LoadTrapDtData = 0.0f;
volatile uint32_t g_ulLoadRequestSeq = 0U;
static volatile LoadAck_t g_LoadAck = {0U, LOAD_RESULT_NONE};

// 文件内前置声明：避免先调用、后定义导致的隐式声明报错。
static bool LoadRequest_IsValid(const LoadRequest_t *request);
static uint8_t LoadDart_GetServoGroup(uint8_t dart_num);
// 跨任务状态接口：实现位于 StoreEnergyTaskFunc.c。
extern bool StoreEnergy_IsLoadAbortRequested(void);
extern void StoreEnergy_SetLoadCycleState(bool active, bool abort_pending);

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

// 换弹结构电机队列
extern QueueHandle_t g_xLoad3508QueueHandler;

static bool LoadTask_ReceiveRequest(LoadRequest_t *request, TickType_t timeout_ticks)
{
    size_t received;

    if (request == NULL || xLoadStreamBuf == NULL)
    {
        return false;
    }

    memset(request, 0, sizeof(*request));
    received = xStreamBufferReceive(xLoadStreamBuf, request, sizeof(*request), timeout_ticks);
    if (received != sizeof(*request))
    {
        (void)xStreamBufferReset(xLoadStreamBuf);
        return false;
    }

    return LoadRequest_IsValid(request);
}

uint16_t LoadRequest_CalcChecksum(const LoadRequest_t *request)
{
    uint32_t checksum;

    if (request == NULL)
    {
        return 0U;
    }

    checksum = request->magic ^
               request->seq ^
               request->timestamp_ms ^
               ((uint32_t)request->dart_num << 24) ^
               ((uint32_t)request->priority << 16);
    return (uint16_t)((checksum & 0xFFFFU) ^ (checksum >> 16));
}

static bool LoadRequest_IsValid(const LoadRequest_t *request)
{
    if (request == NULL ||
        request->magic != LOAD_REQUEST_MAGIC ||
        request->checksum != LoadRequest_CalcChecksum(request) ||
        request->dart_num < 1U ||
        request->dart_num > 4U)
    {
        return false;
    }

    if ((uint32_t)(HAL_GetTick() - request->timestamp_ms) > LOAD_REQUEST_MAX_AGE_MS)
    {
        return false;
    }

    return true;
}

void LoadAck_Set(uint32_t seq, LoadResult_e result)
{
    taskENTER_CRITICAL();
    g_LoadAck.seq = seq;
    g_LoadAck.result = result;
    taskEXIT_CRITICAL();
}

LoadAck_t LoadAck_Get(void)
{
    LoadAck_t ack;

    taskENTER_CRITICAL();
    ack.seq = g_LoadAck.seq;
    ack.result = g_LoadAck.result;
    taskEXIT_CRITICAL();

    return ack;
}

/***********************************
 * @brief 换弹到位后的通用处理：通知舵机、清积分、回零、通知储能
 * @param dart_num 当前飞镖编号
 * @param m_offset_angle 电机偏移角度
 **********************************/
static inline void LoadDart_ReturnToZero(uint8_t dart_num, float m_offset_angle, uint32_t request_seq)
{
    (void)m_offset_angle;

    // 等待两侧储能电机到位后再释放舵机，避免夹爪先到位但储能机构未到位。
    xSemaphoreTake(g_xStoreMotorArrivedSemHandle, portMAX_DELAY);
    if (StoreEnergy_IsLoadAbortRequested() || LoadMotor_IsOverCurrentProtected())
    {
        StoreEnergy_SetLoadCycleState(false, false);
        LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD);
        LoadAck_Set(request_seq, LOAD_RESULT_ABORTED);
        xSemaphoreGive(g_xLoad2StoreSemHandle);
        return;
    }
    uint8_t servo_group = LoadDart_GetServoGroup(dart_num);
    if (servo_group == 0U ||
        xQueueSend(g_xLoad3508QueueHandler, (const void *)&servo_group, 0) != pdTRUE)
    {
        StoreEnergy_SetLoadCycleState(false, false);
        LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD);
        LoadAck_Set(request_seq, LOAD_RESULT_FAILED);
        xSemaphoreGive(g_xLoad2StoreSemHandle);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS));
    if (StoreEnergy_IsLoadAbortRequested() || LoadMotor_IsOverCurrentProtected())
    {
        StoreEnergy_SetLoadCycleState(false, false);
        LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD);
        LoadAck_Set(request_seq, LOAD_RESULT_ABORTED);
        xSemaphoreGive(g_xLoad2StoreSemHandle);
        return;
    }

    // Bias空间
    // 双储能电机同步运动时，默认关闭 S 型规划，直接走 PID。
    StoreMotor_SetUseSCurve(RM_3508_STORE_LEFT, false);
    StoreMotor_SetUseSCurve(RM_3508_STORE_RIGHT, false);
    StoreMotor_SetTarget(RM_3508_STORE_LEFT, (float)(LeftStoreLoad + 10.0f));
    StoreMotor_SetTarget(RM_3508_STORE_RIGHT, (float)(RightStoreLoad - 10.0f));
    StoreMotor_EnableControl(RM_3508_STORE_LEFT, true);
    StoreMotor_EnableControl(RM_3508_STORE_RIGHT, true);
    vTaskDelay(pdMS_TO_TICKS(3));
    vTaskDelay(pdMS_TO_TICKS(3));

    uint32_t bottom_reissue_tick = HAL_GetTick();
    DeadzoneState_t bottom_left_deadzone_1 = {0};
    DeadzoneState_t bottom_right_deadzone_1 = {0};
    while (1)
    {
        StoreMotor_RefreshData(RM_3508_STORE_LEFT);
        StoreMotor_RefreshData(RM_3508_STORE_RIGHT);
        float left_pos = Motor_GetTotalAngle(RM_3508_STORE_LEFT);
        float right_pos = Motor_GetTotalAngle(RM_3508_STORE_RIGHT);

        if ((uint32_t)(HAL_GetTick() - bottom_reissue_tick) >= 20U)
        {
            StoreMotor_SetTarget(RM_3508_STORE_LEFT, (float)(LeftStoreLoad + 350.0f));
            vTaskDelay(pdMS_TO_TICKS(1));
            StoreMotor_SetTarget(RM_3508_STORE_RIGHT, (float)(RightStoreLoad - 350.0f));
            vTaskDelay(pdMS_TO_TICKS(1));
            bottom_reissue_tick = HAL_GetTick();
        }

        if (IsInDeadzoneF(left_pos, (float)(LeftStoreLoad + 350.0f), MOTOR_DEAD_ZONE, &bottom_left_deadzone_1, false) &&
            IsInDeadzoneF(right_pos, (float)(RightStoreLoad - 350.0f), MOTOR_DEAD_ZONE, &bottom_right_deadzone_1, false))
        {
            break;
        }
        if (StoreMotor_IsAnyProtected())
        {
            g_bStoreMotorSafeReturnPending = true;
            StoreEnergy_SetLoadCycleState(false, false);
            LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD);
            LoadAck_Set(request_seq, LOAD_RESULT_ABORTED);
            xSemaphoreGive(g_xLoad2StoreSemHandle);
            return;
        }
        if (StoreEnergy_IsLoadAbortRequested() || LoadMotor_IsOverCurrentProtected())
        {
            StoreEnergy_SetLoadCycleState(false, false);
            LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD);
            LoadAck_Set(request_seq, LOAD_RESULT_ABORTED);
            xSemaphoreGive(g_xLoad2StoreSemHandle);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    // 新换弹结构无需回0
    StoreMotor_SetTarget(RM_3508_STORE_LEFT, LeftStoreBottom);
    vTaskDelay(pdMS_TO_TICKS(1));
    StoreMotor_SetTarget(RM_3508_STORE_RIGHT, RightStoreBottom);
    vTaskDelay(pdMS_TO_TICKS(1));
    bottom_reissue_tick = HAL_GetTick();
    DeadzoneState_t bottom_left_deadzone_2 = {0};
    DeadzoneState_t bottom_right_deadzone_2 = {0};
    while (1)
    {
        StoreMotor_RefreshData(RM_3508_STORE_LEFT);
        StoreMotor_RefreshData(RM_3508_STORE_RIGHT);
        float left_pos = Motor_GetTotalAngle(RM_3508_STORE_LEFT);
        float right_pos = Motor_GetTotalAngle(RM_3508_STORE_RIGHT);

        if ((uint32_t)(HAL_GetTick() - bottom_reissue_tick) >= 20U)
        {
            StoreMotor_SetTarget(RM_3508_STORE_LEFT, LeftStoreBottom);
            vTaskDelay(pdMS_TO_TICKS(1));
            StoreMotor_SetTarget(RM_3508_STORE_RIGHT, RightStoreBottom);
            vTaskDelay(pdMS_TO_TICKS(1));
            bottom_reissue_tick = HAL_GetTick();
        }

        if (IsInDeadzoneF(left_pos, LeftStoreBottom, MOTOR_DEAD_ZONE, &bottom_left_deadzone_2, false) &&
            IsInDeadzoneF(right_pos, RightStoreBottom, MOTOR_DEAD_ZONE, &bottom_right_deadzone_2, false))
        {
            break;
        }
        if (StoreMotor_IsAnyProtected())
        {
            g_bStoreMotorSafeReturnPending = true;
            StoreEnergy_SetLoadCycleState(false, false);
            LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD);
            LoadAck_Set(request_seq, LOAD_RESULT_ABORTED);
            xSemaphoreGive(g_xLoad2StoreSemHandle);
            return;
        }
        if (StoreEnergy_IsLoadAbortRequested() || LoadMotor_IsOverCurrentProtected())
        {
            StoreEnergy_SetLoadCycleState(false, false);
            LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD);
            LoadAck_Set(request_seq, LOAD_RESULT_ABORTED);
            xSemaphoreGive(g_xLoad2StoreSemHandle);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    StoreEnergy_SetLoadCycleState(false, false);
    LoadAck_Set(request_seq, LOAD_RESULT_DONE);
    xSemaphoreGive(g_xLoad2StoreSemHandle);
    LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD);
    if (dart_num == 1)
        vTaskDelay(pdMS_TO_TICKS(200));
}

float Load3508_GetReloadOffset(uint8_t step_count)
{
    if (step_count == 0U)
    {
        return 0.0f;
    }

    if (step_count > 3U)
    {
        step_count = 3U;
    }

    return LOAD3508_DART_STEP_DIR *
           (LOAD3508_DART_FIRST_STEP_DEG + ((float)(step_count - 1U) * LOAD3508_DART_STEP_DEG));
}

static uint8_t LoadDart_GetServoGroup(uint8_t dart_num)
{
    if (dart_num < 1U || dart_num > 3U)
    {
        return 0U;
    }

    return (uint8_t)(4U - dart_num);
}

void LoadMotorTaskMainLoopFunc(void)
{
    bool MutexTake = false;
    float load_3508_offset_angle = 0.0f;
    while (1)
    {
        xSemaphoreTake(g_xStore2LoadSemHandle, portMAX_DELAY);
        LoadRequest_t load_request = {0};
        if (!LoadTask_ReceiveRequest(&load_request, pdMS_TO_TICKS(50)))
        {
            LoadAck_Set(load_request.seq, LOAD_RESULT_FAILED);
            xSemaphoreGive(g_xLoad2StoreSemHandle);
            continue;
        }
        LoadAck_Set(load_request.seq, LOAD_RESULT_ACCEPTED);
        vTaskDelay(pdMS_TO_TICKS(500)); // 本来是175的,但是这个时候换弹过快
        float cur_target = Motor3508_GetTarget();
        switch (load_request.dart_num)
        {
        case 4:
            cur_target = PresetLoc + load_3508_offset_angle;
            MutexTake = false;
            break;
        case 3:
            cur_target = PresetLoc + load_3508_offset_angle + Load3508_GetReloadOffset(1U);
            MutexTake = true;
            break;
        case 2:
            cur_target = PresetLoc + load_3508_offset_angle + Load3508_GetReloadOffset(2U);
            MutexTake = true;
            break;
        case 1:
            cur_target = PresetLoc + load_3508_offset_angle + Load3508_GetReloadOffset(3U);
            MutexTake = true;
            break;
        default:
            LoadAck_Set(load_request.seq, LOAD_RESULT_FAILED);
            xSemaphoreGive(g_xLoad2StoreSemHandle);
            continue;
        }
        uint8_t last_released_servo_group = 0U;
        taskENTER_CRITICAL();
        last_released_servo_group = g_ucLastReleasedServoGroup;
        taskEXIT_CRITICAL();
        if (MutexTake && last_released_servo_group >= 1U && last_released_servo_group <= 3U)
        {
            (void)Servo_MoveDartGroupToZero(last_released_servo_group, SERVO_MOVE_TIME_MS);
            vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS));
            taskENTER_CRITICAL();
            if (g_ucLastReleasedServoGroup == last_released_servo_group)
            {
                g_ucLastReleasedServoGroup = 0U;
            }
            taskEXIT_CRITICAL();
        }
        DeadzoneTimer_t reach_timer = {0};
        uint32_t reach_timeout_ms = CalcTrapMoveTimeoutMs(
            Motor_GetTotalAngle(RM_3508_GRIPPER),
            cur_target,
            LOAD_TASK_TRAP_VMAX_DEG_S,
            LOAD_TASK_TRAP_AMAX_DEG_S2,
            LOAD_DEADZONE_TIMEOUT_MS,
            12000U);
        if (!LoadMotor_SubmitTarget(LOAD_MOTOR_OWNER_LOAD,
                                    LOAD_MOTOR_PRIORITY_LOAD,
                                    cur_target,
                                    reach_timeout_ms + 1000U))
        {
            LoadAck_Set(load_request.seq, LOAD_RESULT_FAILED);
            LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD);
            xSemaphoreGive(g_xLoad2StoreSemHandle);
            continue;
        }
        if (!MutexTake)
        {
            LoadAck_Set(load_request.seq, LOAD_RESULT_DONE);
            xSemaphoreGive(g_xLoad2StoreSemHandle);
            LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD);
        }

        // 这里再加一个互斥量进行保证
        while (MutexTake)
        {
            MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
            if (LoadMotor_IsOverCurrentProtected())
            {
                MutexTake = false;
                Motor3508_EnableControl(false);
                StoreEnergy_SetLoadCycleState(false, false);
                LoadAck_Set(load_request.seq, LOAD_RESULT_ABORTED);
                LoadMotor_ReleaseOwner(LOAD_MOTOR_OWNER_LOAD);
                xSemaphoreGive(g_xLoad2StoreSemHandle);
                break;
            }
            if (IsInDeadzoneTimedF(MotorData, cur_target,
                                   MOTOR_DEAD_ZONE, reach_timeout_ms,
                                   &reach_timer))
            {
                MutexTake = false;
                LoadDart_ReturnToZero(load_request.dart_num, load_3508_offset_angle, load_request.seq); // 现在无需回零
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
}

void LoadServoTaskMainLoopFunc(void)
{
    uint8_t fQueueDartNum = 0; // 目标为0最初
    while (1)
    {
        xQueueReceive(g_xLoad3508QueueHandler, &fQueueDartNum, portMAX_DELAY);
        if (fQueueDartNum >= 1 && fQueueDartNum <= 3)
        {
            if (Servo_ReleaseDartGroup(fQueueDartNum, SERVO_MOVE_TIME_MS))
            {
                vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS));
                // 换弹标志位
                taskENTER_CRITICAL();
                LoadNumState++;
                g_ucLastReleasedServoGroup = fQueueDartNum;
                taskEXIT_CRITICAL();
            }
            vTaskDelay(pdMS_TO_TICKS(SERVO_MOVE_TIME_MS));
        }
    }
}
