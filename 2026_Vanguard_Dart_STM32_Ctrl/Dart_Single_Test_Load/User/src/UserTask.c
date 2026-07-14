// /************************************************************************************************************************
//  * 文件：UserTask.c
//  * 用途：换弹结构测试工程 - 独立测试换弹机构
//  * 创建者：邓金水
//  * 修改：重构为独立测试工程，简化目标位置控制逻辑
//  *
//  * 换弹结构说明：
//  * - 3508电机控制传送带移动
//  * - 3个舵机（0x01, 0x02, 0x03）控制飞镖分离
//  *
//  * 位置参数（累计角度，上电位置为0）：
//  * - 第三个舵机位置：-7547
//  * - 第二个舵机位置：-13200（与第三个相距5653）
//  * - 第一个舵机位置：-18749（与第二个相距5549）
//  ***********************************************************************************************************************/
#include "UserTask.h"
#include "PID.h"

/************************全局或静态作用域*********************/
static bool des_yes = false;
// #if TestUse
float RmMotorAngleData = 0.0f;
float RmMotorSpeedData = 0.0f;
float target_loc = 0.0f;
static DeadzoneState_t g_LoadReturnZeroDeadzoneState = {0};
static DeadzoneState_t g_LoadPresetDeadzoneState = {0};
static DeadzoneState_t g_LoadTargetDeadzoneState = {0};
// #endif

static inline void LoadMotor_SetFinalTarget(float new_target_pos_deg)
{
    target_loc = new_target_pos_deg;
}

static inline void LoadMotor_RunTarget(float target_pos_deg)
{
    RmMotorPID_Calc(RM_3508_GRIPPER, target_pos_deg);
}

static inline void LoadMotor_HoldTargetMs(float hold_target_pos_deg, uint32_t hold_ms)
{
    uint32_t start_tick = HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - start_tick) < hold_ms)
    {
        RmMotorAngleData = Motor_GetTotalAngle(RM_3508_GRIPPER);
        LoadMotor_RunTarget(hold_target_pos_deg);
        vTaskDelay(1);
    }
}

static uint8_t g_ucReloadQueueStorage[4 * sizeof(uint8_t)];
static StaticQueue_t g_xReloadQueue;
static QueueHandle_t g_xLoad3508QueueHandler;

// 测试状态
static volatile uint8_t g_ucTestDartNum = 3; // 从第3发开始测试
/*---------------------------------------------------------------------------------------*/

// // 模块驱动初始化
void Module_Init(void)
{
    BSP_POWER_DeInit(); // 失能无绿灯，亮红灯
    DWT_Init(180);
    MotorInit();
    CanFilterCfg();
    BSP_UART_Init();
    ServoInit();
    Servo_RegisterDartProfiles();
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    HAL_TIM_Base_Start(&htim8);
    HAL_Delay(100);
}

/// @brief 创建RTOS的通信量
/// @param  无
void RTOS_ModuleInit(void)
{
    g_xLoad3508QueueHandler = xQueueCreateStatic(4, sizeof(uint8_t), g_ucReloadQueueStorage, &g_xReloadQueue);
}

/***********************************************TASK******************************************************/

// 换弹任务
osThreadId_t LoadTaskHandle;
const osThreadAttr_t LoadTask_attributes = {
    .name = "LoadTask",
    .stack_size = 160 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal7,
};

void LoadTaskFunc(void *argument);
void LoadMotorTaskFunc(void *argument);

/***********************************
 * 函数名: TaskInitFunc
 * 作用:   任务初始化创建
 * 参数:   无
 **********************************/
void TaskInitFunc(void)
{
    RTOS_ModuleInit();
    LoadTaskHandle = osThreadNew(LoadTaskFunc, NULL, &LoadTask_attributes);
}

/**
 * @brief 换弹到位后的通用处理：通知舵机、清积分、回零、通知储能
 * @param dart_num 当前飞镖编号
 */
static inline void LoadDart_ReturnToZero(uint8_t dart_num)
{
    xQueueSend(g_xLoad3508QueueHandler, (const void *)&dart_num, 0);
    des_yes = false;
    // vTaskDelay(500);                                                          // 这里是上电后的绝对位置
    while (!des_yes) // 先移动到一个安全角度防止肘击
    {
        des_yes = IsInDeadzoneF(Motor_GetTotalAngle(RM_3508_GRIPPER), PresetLoc, MOTOR_DEAD_ZONE, &g_LoadReturnZeroDeadzoneState, false);
        LoadMotor_RunTarget(PresetLoc);
        osDelay(1);
    }
    des_yes = false;
    switch (dart_num)
    {
    case 3:
        HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
        break;
    case 2:
        HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
        break;
    case 1:
        HAL_GPIO_TogglePin(LED5_GPIO_Port, LED5_Pin);
        break;
    default:
        break;
    }
}

/***********************************
 * 函数名: LoadTaskFunc
 * 作用:   换弹测试任务（传送带电机控制以及电机目标设定）
 * 参数:   无
 * 说明:   独立测试模式，使用累计角度作为绝对目标位置
 **********************************/
void LoadTaskFunc(void *argument)
{
    uint8_t dart_num = 0;
    float pre_loc = 0.0f;

    // 初始化舵机到机械零点
    Servo_MoveAllToZero(300);
    vTaskDelay(500); // 等待舵机到位

    TaskHandle_t Load3508TaskHandle = NULL;
    xTaskCreate(LoadMotorTaskFunc, "LoadMotor", 64 * 4, NULL, osPriorityBelowNormal7, &Load3508TaskHandle);

    pre_loc = -6427.0f;
    RmMotorAngleData = Motor_GetTotalAngle(RM_3508_GRIPPER);

    while (1)
    {
        // 独立测试模式：等待一段时间后自动开始测试
        RmMotorAngleData = Motor_GetTotalAngle(RM_3508_GRIPPER);
        while (!des_yes)
        {
            des_yes = IsInDeadzoneF(RmMotorAngleData, pre_loc, MOTOR_DEAD_ZONE, &g_LoadPresetDeadzoneState, false);
            LoadMotor_RunTarget(pre_loc);
            RmMotorAngleData = Motor_GetTotalAngle(RM_3508_GRIPPER);
            vTaskDelay(1);
        }
        des_yes = false;
        dart_num = g_ucTestDartNum;
        LoadMotor_SetFinalTarget(FirstServoLoc);
        LoadMotor_HoldTargetMs(target_loc, 2500);
        while (1)
        {
            RmMotorAngleData = Motor_GetTotalAngle(RM_3508_GRIPPER);
            LoadMotor_RunTarget(target_loc);

            if (dart_num != 0)
            {
                des_yes = IsInDeadzoneF(RmMotorAngleData, target_loc, MOTOR_DEAD_ZONE, &g_LoadTargetDeadzoneState, false);
                if (des_yes)
                {
                    LoadDart_ReturnToZero(dart_num);
                    LoadMotor_HoldTargetMs(PresetLoc, 2500);
                    dart_num--;
                    if (dart_num == 0)
                    {
                        LoadMotor_SetFinalTarget(PresetLoc);
                        des_yes = false;
                        LoadMotor_HoldTargetMs(target_loc, 2500);
                        break;
                    }
                    else if (dart_num == 2)
                    {
                        LoadMotor_SetFinalTarget(SecondServoLoc);
                        des_yes = false;
                        LoadMotor_HoldTargetMs(target_loc, 2500);
                    }
                    else if (dart_num == 1)
                    {
                        LoadMotor_SetFinalTarget(ThirdServoLoc);
                        des_yes = false;
                        LoadMotor_HoldTargetMs(target_loc, 2500);
                    }
                }
            }
            HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
            vTaskDelay(1); // 控制循环频率
        }
    }
}

/***********************************
 * 函数名: LoadMotorTaskFunc
 * 作用:   换弹任务（舵机释放）
 * 参数:   无
 **********************************/
void LoadMotorTaskFunc(void *argument)
{
    uint8_t fQueueDartNum = 0;
    vTaskDelay(100); // 启动延迟

    while (1)
    {
        xQueueReceive(g_xLoad3508QueueHandler, &fQueueDartNum, portMAX_DELAY);

        if (fQueueDartNum >= 1 && fQueueDartNum <= 3)
        {
            (void)Servo_ReleaseDartGroup(fQueueDartNum, SERVO_MOVE_TIME_MS);
            HAL_Delay(SERVO_MOVE_TIME_MS);
            (void)Servo_MoveDartGroupToZero(fQueueDartNum, SERVO_MOVE_TIME_MS);
            vTaskDelay(SERVO_MOVE_TIME_MS);
        }
    }
}
