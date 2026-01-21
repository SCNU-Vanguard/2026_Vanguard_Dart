/************************************************************************************************************************
 * 文件：UserTask.c
 * 用途：换弹结构测试工程 - 独立测试换弹机构
 * 创建者：邓金水
 * 修改：重构为独立测试工程，简化目标位置控制逻辑
 *
 * 换弹结构说明：
 * - 3508电机控制传送带移动
 * - 3个舵机（0x01, 0x02, 0x03）控制飞镖分离
 *
 * 位置参数（累计角度，上电位置为0）：
 * - 第三个舵机位置：-7547
 * - 第二个舵机位置：-13200（与第三个相距5653）
 * - 第一个舵机位置：-18749（与第二个相距5549）
 ***********************************************************************************************************************/
#include "UserTask.h"
#include "PID.h"

/************************全局或静态作用域*********************/
static float MotorData = 0.0f;
static float GripperTarget = 0.0f;

// 换弹结构电机队列
static uint8_t g_ucReloadQueueStorage[4 * sizeof(uint8_t)];
static StaticQueue_t g_xReloadQueue;
static QueueHandle_t g_xLoad3508QueueHandler;

// 测试状态
static volatile uint8_t g_ucTestDartNum = 3; // 从第3发开始测试
/*---------------------------------------------------------------------------------------*/

// 模块驱动初始化
void Module_Init(void)
{
    BSP_POWER_DeInit(); // 失能无绿灯，亮红灯
    DWT_Init(180);
    MotorInit();
    CanFilterCfg();
    BSP_UART_Init();
    ServoInit();
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

/***********************************
 * 函数名: LoadTaskFunc
 * 作用:   换弹测试任务（传送带电机控制以及电机目标设定）
 * 参数:   无
 * 说明:   独立测试模式，使用累计角度作为绝对目标位置
 **********************************/
void LoadTaskFunc(void *argument)
{
    uint8_t servo_ids[3] = {0x01, 0x02, 0x03};
    uint16_t servo_angles[3] = {0x0000, 0x0000, 0x0000};
    bool MutexTake = false;
    uint8_t dart_num = 0;

    // 初始化舵机到0°位置
    ServoControlMulti(3, servo_ids, servo_angles, 300);
    vTaskDelay(500); // 等待舵机到位

    // 创建舵机控制子任务
    TaskHandle_t Load3508TaskHandle = NULL;
    xTaskCreate(LoadMotorTaskFunc, "LoadMotor", 64 * 4, NULL, osPriorityBelowNormal7, &Load3508TaskHandle);

    // 获取电机偏移角度，用于目标值补偿
    float offset_angle = MotorManager.MotorList[RM_3508_GRIPPER - 1].motor_data.offset_ecd_angle;
    
    // 初始目标为当前位置（上电零点 + 偏移）
    GripperTarget = offset_angle;

    while (1)
    {
        // 独立测试模式：等待一段时间后自动开始测试
        vTaskDelay(2000); // 2秒后开始测试

        dart_num = g_ucTestDartNum;

        switch (dart_num)
        {
        case 4:
            // 测试完成，回到初始位置
            GripperTarget = offset_angle;
            MutexTake = false;
            g_ucTestDartNum = 3; // 重置为第3发，准备下一轮测试
            vTaskDelay(3000);    // 等待3秒后重新开始
            continue;
        case 3:
            GripperTarget = FirstServoLoc + offset_angle;
            MutexTake = true;
            break;
        case 2:
            GripperTarget = SecondServoLoc + offset_angle;
            MutexTake = true;
            break;
        case 1:
            GripperTarget = ThirdServoLoc + offset_angle;
            MutexTake = true;
            break;
        default:
            g_ucTestDartNum = 3;
            continue;
        }

        while (MutexTake)
        {
            MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
            RmMotorPID_Calc(RM_3508_GRIPPER, GripperTarget);

            // 检查是否到达第3发位置
            if (IS_IN_DEADZONE(MotorData, FirstServoLoc, MOTOR_DEAD_ZONE) && (dart_num == 3))
            {
                xQueueSend(g_xLoad3508QueueHandler, (const void *)&dart_num, 0);
                vTaskDelay(SERVO_MOVE_TIME_MS);

                // 回到零点
                // 换向前清除积分
                CASCADE_PID_Clear_Integral(&MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid);
                GripperTarget = offset_angle;
                MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
                while (!IS_IN_DEADZONE(MotorData, 0.0f, MOTOR_DEAD_ZONE))
                {
                    RmMotorPID_Calc(RM_3508_GRIPPER, GripperTarget);
                    MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
                    osDelay(2);
                }
                MutexTake = false;
                HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
                g_ucTestDartNum = 2; // 进入下一发测试
            }

            // 检查是否到达第2发位置
            if (IS_IN_DEADZONE(MotorData, SecondServoLoc, MOTOR_DEAD_ZONE) && (dart_num == 2))
            {
                xQueueSend(g_xLoad3508QueueHandler, (const void *)&dart_num, 0);
                vTaskDelay(SERVO_MOVE_TIME_MS);

                // 回到零点
                // 换向前清除积分
                CASCADE_PID_Clear_Integral(&MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid);
                GripperTarget = offset_angle;
                MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
                while (!IS_IN_DEADZONE(MotorData, 0.0f, MOTOR_DEAD_ZONE))
                {
                    RmMotorPID_Calc(RM_3508_GRIPPER, GripperTarget);
                    MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
                    osDelay(2);
                }
                MutexTake = false;
                HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
                g_ucTestDartNum = 1; // 进入下一发测试
            }

            // 检查是否到达第1发位置
            if (IS_IN_DEADZONE(MotorData, ThirdServoLoc, MOTOR_DEAD_ZONE) && (dart_num == 1))
            {
                xQueueSend(g_xLoad3508QueueHandler, (const void *)&dart_num, 0);
                vTaskDelay(SERVO_MOVE_TIME_MS);

                // 回到零点
                // 换向前清除积分
                CASCADE_PID_Clear_Integral(&MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid);
                GripperTarget = offset_angle;
                MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
                while (!IS_IN_DEADZONE(MotorData, 0.0f, MOTOR_DEAD_ZONE))
                {
                    RmMotorPID_Calc(RM_3508_GRIPPER, GripperTarget);
                    MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
                    osDelay(2);
                }
                MutexTake = false;
                HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
                g_ucTestDartNum = 4; // 测试完成
                vTaskDelay(200);
            }

            osDelay(1); // 控制循环频率
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

        if (fQueueDartNum == 3)
        {
            ServoControlPos(0x03, SeperationAngle, SERVO_MOVE_TIME_MS);
            vTaskDelay(SERVO_MOVE_TIME_MS + 5);
            ServoControlPos(0x03, 0x0000, SERVO_MOVE_TIME_MS);
            vTaskDelay(SERVO_MOVE_TIME_MS + 5);
        }
        else if (fQueueDartNum == 2)
        {
            ServoControlPos(0x02, SeperationAngle, SERVO_MOVE_TIME_MS);
            vTaskDelay(SERVO_MOVE_TIME_MS + 5);
            ServoControlPos(0x02, 0x0000, SERVO_MOVE_TIME_MS);
            vTaskDelay(SERVO_MOVE_TIME_MS + 5);
        }
        else if (fQueueDartNum == 1)
        {
            ServoControlPos(0x01, SeperationAngle, SERVO_MOVE_TIME_MS);
            vTaskDelay(SERVO_MOVE_TIME_MS + 5);
            ServoControlPos(0x01, 0x0000, SERVO_MOVE_TIME_MS);
            vTaskDelay(SERVO_MOVE_TIME_MS + 5);
        }
    }
}
