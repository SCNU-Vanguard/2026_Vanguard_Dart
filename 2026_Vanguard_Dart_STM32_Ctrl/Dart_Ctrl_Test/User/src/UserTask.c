/************************************************************************************************************************
 * 文件：UserTask.c
 * 用途：测试工程 4 发 RM3508 储能循环
 *
 *   - SWB=1 允许 4 发循环执行，SWB=0 立即冻结电机 (不再下发新目标)
 *   - 每一发为: 储能到 CHARGE 位 -> 扳机释放 -> 储能回到 RELEASE 位 -> inter-shot 停顿
 *   - 4 发打完后锁在 BATCH_DONE，SWB 需从 1->0->1 重新触发下一轮
 ***********************************************************************************************************************/
#include "UserTask.h"
#include "PID.h"
#include "ControlState.h"
#include "config.h"

extern MotorManager_t MotorManager;

static StaticSemaphore_t g_xStoreSemaphore;
SemaphoreHandle_t g_xStoreSemaphoreHandle;

static StreamBufferHandle_t xLoadStreamBuf;

static StaticEventGroup_t g_pxStateSetEventGroupBuffer;
static EventGroupHandle_t g_pxStateSetEventGroupHandeler;

void Module_Init(void)
{
    BSP_POWER_DeInit();
    DWT_Init(180);
    MotorInit();
    CanFilterCfg();
    BSP_UART_Init();
    UART_SetProtocolType(BSP_UART6, PROTOCOL_IBUS);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    HAL_TIM_Base_Start(&htim8);
    HAL_Delay(100);
}

void RTOS_ModuleInit(void)
{
    g_xStoreSemaphoreHandle = xSemaphoreCreateCountingStatic(5, sizeof(uint8_t), &g_xStoreSemaphore);
    xLoadStreamBuf = xStreamBufferCreate(1, 1);
    g_pxStateSetEventGroupHandeler = xEventGroupCreateStatic(&g_pxStateSetEventGroupBuffer);
}

osThreadId_t StoreEnergyTaskHandle;
const osThreadAttr_t StoreEnergyTask_attributes = {
    .name = "StoreEnergyTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};

void StoreEnergyTaskFunc(void *argument);

void TaskInitFunc(void)
{
    RTOS_ModuleInit();
    ControlState_Init();

    StoreEnergyTaskHandle = osThreadNew(StoreEnergyTaskFunc, NULL, &StoreEnergyTask_attributes);
    ControlState_CreateTasks();
}

/*============================== 4 发状态机 ==============================*/

typedef enum
{
    SHOT_STATE_IDLE = 0,
    SHOT_STATE_CHARGE,
    SHOT_STATE_RELEASE,
    SHOT_STATE_INTER,
    SHOT_STATE_BATCH_DONE,
} ShotState_e;

static bool Store_SwbAllow(void)
{
    return g_ControlInput.data_valid && (g_ControlInput.swb == 1);
}

static void Store_DriveMotors(float left_target, float right_target)
{
    RmMotorPID_Calc(RM_3508_STORE_LEFT, left_target);
    RmMotorPID_Calc(RM_3508_STORE_RIGHT, right_target);
}

static bool Store_ReachedBoth(float left_target, float right_target)
{
    float lp = Motor_GetTotalAngle(RM_3508_STORE_LEFT);
    float rp = Motor_GetTotalAngle(RM_3508_STORE_RIGHT);
    return IS_IN_DEADZONE(lp, left_target, MOTOR_DEAD_ZONE) &&
           IS_IN_DEADZONE(rp, right_target, MOTOR_DEAD_ZONE);
}

void StoreEnergyTaskFunc(void *argument)
{
    vTaskDelay(POWER_ON_DELAY_MS);

    ShotState_e state = SHOT_STATE_IDLE;
    uint8_t shot_idx = 0U;
    uint32_t phase_start = HAL_GetTick();
    int8_t last_swb = 1;
    bool batch_armed = false;

    for (;;)
    {
        bool allow = Store_SwbAllow();

        if (!allow)
        {
            state = SHOT_STATE_IDLE;
            shot_idx = 0U;
            batch_armed = true;
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_store);
            last_swb = (int8_t)g_ControlInput.swb;
            vTaskDelay(pdMS_TO_TICKS(TEST_STORE_CTRL_PERIOD_MS));
            continue;
        }

        uint32_t now = HAL_GetTick();

        switch (state)
        {
        case SHOT_STATE_IDLE:
        {
            if (!batch_armed)
            {
                last_swb = (int8_t)g_ControlInput.swb;
                break;
            }
            shot_idx = 0U;
            batch_armed = false;
            state = SHOT_STATE_CHARGE;
            phase_start = now;
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_store);
            break;
        }

        case SHOT_STATE_CHARGE:
        {
            Store_DriveMotors(TEST_STORE_LEFT_CHARGE_DEG, TEST_STORE_RIGHT_CHARGE_DEG);
            bool reached = Store_ReachedBoth(TEST_STORE_LEFT_CHARGE_DEG, TEST_STORE_RIGHT_CHARGE_DEG);
            bool min_hold = (now - phase_start) >= TEST_STORE_CHARGE_MS;
            bool timeout = (now - phase_start) >= (TEST_STORE_CHARGE_MS + MOTOR_TIMEOUT_MS);
            if ((reached && min_hold) || timeout)
            {
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_shoot);
                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                state = SHOT_STATE_RELEASE;
                phase_start = now;
            }
            break;
        }

        case SHOT_STATE_RELEASE:
        {
            Store_DriveMotors(TEST_STORE_LEFT_RELEASE_DEG, TEST_STORE_RIGHT_RELEASE_DEG);
            bool reached = Store_ReachedBoth(TEST_STORE_LEFT_RELEASE_DEG, TEST_STORE_RIGHT_RELEASE_DEG);
            bool min_hold = (now - phase_start) >= TEST_STORE_RELEASE_MS;
            bool timeout = (now - phase_start) >= (TEST_STORE_RELEASE_MS + MOTOR_TIMEOUT_MS);
            if ((reached && min_hold) || timeout)
            {
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_store);
                shot_idx++;
                if (shot_idx >= TEST_SHOT_COUNT)
                {
                    state = SHOT_STATE_BATCH_DONE;
                    batch_armed = false;
                }
                else
                {
                    state = SHOT_STATE_INTER;
                }
                phase_start = now;
            }
            break;
        }

        case SHOT_STATE_INTER:
        {
            Store_DriveMotors(TEST_STORE_LEFT_RELEASE_DEG, TEST_STORE_RIGHT_RELEASE_DEG);
            if ((now - phase_start) >= TEST_INTER_SHOT_MS)
            {
                state = SHOT_STATE_CHARGE;
                phase_start = now;
            }
            break;
        }

        case SHOT_STATE_BATCH_DONE:
        default:
            break;
        }

        last_swb = (int8_t)g_ControlInput.swb;
        vTaskDelay(pdMS_TO_TICKS(TEST_STORE_CTRL_PERIOD_MS));
    }
}
