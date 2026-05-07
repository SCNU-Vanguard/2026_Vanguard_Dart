/************************************************************************************************************************
 * 文件：UserTask.c
 * 用途：用户任务定义，所有的任务都在此
 * 创建者：邓金水
 * 创建日期：忘了
 * 目标：15s内完成4次发射过程
 * 流程：视觉/遥控器(可调射程)纠正方向->第一发飞镖->滑台下滑(同时扳机要放下来)->到达地点(扳机绷紧)->根据所需射程调节电机->释放扳机
 *
 * 框图：
 *
 *
 *                                                                        \===========================================\
 *                                                                         \                    0x201                  \
 *                                                                          \                  RM-M3508                 \
 *                                                                           \                                           \
 *                                                                            \      0x01          0x02          0x03     \
 *                                                                             \  SerialServo   SerialServo   SerialServo  \
 *                                                                              \                                           \
 *                                                                               \                                           \
 *                                                                              |=============================================|
 *                                                                              |                                             |
 *                                                                              |                                             |
 *                                                                              |                    0x03                     |
 * 遥控器(FS-I6X)----------------------------------------------->FX-16AB接收机--------------------DM-J4310-YAW                  |
 *                                                                              |                                             |
 *                                                                              |     0x01                          0x02      |
 *                                                                              |(DM-S3519-Left)              (DM-S3519-Right)|
 *                                                                              |                                             |
 *                                                                              |                   0x205                     |
 *                                                                              |               RM-M2006-Trigger              |
 *                                                                              |                                             |
 *                                                                              |                  TriggerServo               |
 *                                                                              |=============================================|
 *
 * 现在的毛胚房版本是没有加上视觉和上位机的,但是要用的话那么,首先要注释掉串口通信部份的代码,也就说全靠自己
 * 无视觉方案:
 * 上电、初始化后根据预设方案调节扳机位置和Yaw角度,让储能电机开始工作,拉到指定位置之后电机停止,之后等待换弹
 * 第一发可以不用换弹,直接拉到舵机位置,之后扳机处于位置之后电机不用动,扳机释放飞镖(电机处于失能状态),这里还
 * 要等待测试达妙电机的具体逻辑
 ***********************************************************************************************************************/
#include "UserTask.h"
#include "PID.h"
#include "ControlState.h"

// 19271（上限）<长度>
// 第三个（距离7547）
// 第二个（距离13200）<与第三个相距5653>
// 第一个（距离18749）<与第二个相距5549>

// 任务：
// 状态设置任务(云台Yaw轴角度和扳机位置调节任务)
// 换弹任务
// 扳机任务 (包含调节扳机松紧任务)
// 存储拉簧动能任务
// yaw调整任务

/************************全局或静态作用域*********************/
extern MotorManager_t MotorManager;
static float MotorData = 0.0f;
static __IO float GripperTarget = 0.0f; // 位置不动

// 储能任务信号量
static StaticSemaphore_t g_xStoreSemaphore;
SemaphoreHandle_t g_xStoreSemaphoreHandle;

// 储能任务消息
static StreamBufferHandle_t xLoadStreamBuf;

// StateSet任务的事件组
static StaticEventGroup_t g_pxStateSetEventGroupBuffer;
static EventGroupHandle_t g_pxStateSetEventGroupHandeler;

/*---------------------------------------------------------------------------------------*/

// 模块驱动初始化
void Module_Init(void)
{
    BSP_POWER_DeInit(); // 失能无绿灯，亮红灯
    DWT_Init(180);
    MotorInit();
    CanFilterCfg();
    BSP_UART_Init();
    // while (!ServoInit())
    //     ; // 这个地方有一个回调,需要进行数据读取
    // ServoInit();
    UART_SetProtocolType(BSP_UART6, PROTOCOL_IBUS);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    HAL_TIM_Base_Start(&htim8);
    HAL_Delay(100);
}

/// @brief 创建RTOS的通信量
/// @param  无
void RTOS_ModuleInit(void)
{
    // SEMAPHORE（信号量）：信号量与dart_num共同决定飞镖换弹的状态
    g_xStoreSemaphoreHandle = xSemaphoreCreateCountingStatic(5, sizeof(uint8_t), &g_xStoreSemaphore);

    // 使用stream流传输,也可以改成使用Message传输
    xLoadStreamBuf = xStreamBufferCreate(1, 1); // 1字节触发

    // EVENTGROUPS（事件组）：用于确认云台电机和射程电机正常设置,只有这两个正常设置之后才可以进行下面的流程
    g_pxStateSetEventGroupHandeler = xEventGroupCreateStatic(&g_pxStateSetEventGroupBuffer);
}

/***********************************************TASK******************************************************/

// 拉簧储能任务
osThreadId_t StoreEnergyTaskHandle;
const osThreadAttr_t StoreEnergyTask_attributes = {
    .name = "StoreEnergyTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};

// 换弹任务
osThreadId_t LoadTaskHandle;
const osThreadAttr_t LoadTask_attributes = {
    .name = "LoadTask",
    .stack_size = 160 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal7,
};

// 飞镖状态设置(设置Yaw和射程)
osThreadId_t StateSetTaskHandle;
const osThreadAttr_t StateSetTask_attributes = {
    .name = "StateSetTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void StoreEnergyTaskFunc(void *argument);
void StateSetTaskFunc(void *argument);

/***********************************
 * 函数名: TaskInitFunc
 * 作用:   任务初始化创建
 * 参数:   无
 **********************************/
void TaskInitFunc(void)
{
    RTOS_ModuleInit();
    ControlState_Init();

    // 任务初始化
    StoreEnergyTaskHandle = osThreadNew(StoreEnergyTaskFunc, NULL, &StoreEnergyTask_attributes);
    StateSetTaskHandle = osThreadNew(StateSetTaskFunc, NULL, &StateSetTask_attributes);
    ControlState_CreateTasks();
}

/***********************************
 * 函数名: StoreEnergyTaskFunc
 * 作用:   拉簧储能任务(控制左右3519电机转动)
 * 参数:   无
 *
 * 状态机流程:
 * case 0x00: 无换弹，直接进入下一步
 * case 0x01: 上移到扳机位置
 * case 0x02: 滑台叉上移 + 发射，然后继续下一轮
 **********************************/
void StoreEnergyTaskFunc(void *argument)
{
    // 等待事件组：云台和扳机都就绪后才开始
    xEventGroupWaitBits(g_pxStateSetEventGroupHandeler, EVENT_ALL_READY, pdFALSE, pdTRUE, portMAX_DELAY);
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    vTaskDelay(1);
    DM_MotorEnable(DM_3519_STRENTH_LEFT, CAN_TX_RETRY_ENABLE);
    DM_MotorEnable(DM_3519_STRENTH_RIGHT, CAN_TX_RETRY_ENABLE);

    uint8_t StoreState = 0x00;
    vTaskDelay(POWER_ON_DELAY_MS);
    float left_pos = 0.0f, right_pos = 0.0f;

    while (1)
    {
        // ===== 获取电机控制权（与调试任务互斥）=====
        // 如果信号量未初始化或调试任务正在运行，会等待
        if (g_xMotorCtrlSemHandle == NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue; // 等待信号量初始化
        }
        if (xSemaphoreTake(g_xMotorCtrlSemHandle, pdMS_TO_TICKS(50)) != pdTRUE)
            continue;

        // 再次检查是否处于手动调试模式
        // 如果刚获取到信号量但调试模式已激活，立即释放让给调试任务
        if (ControlState_IsManualOverride())
        {
            xSemaphoreGive(g_xMotorCtrlSemHandle);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue; // 重新尝试获取
        }

        switch (StoreState)
        {
        case 0x00:
        {
            // ========== 换弹等待状态（已跳过）==========
            StoreState++;
            break;
        }

        case 0x01:
        {
            // ========== 移到扳机位置 ==========
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_shoot);
            vTaskDelay(500);
            DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreTrigger, 5.0f, DM_LOCATION_SPEED, CAN_TX_RETRY_ENABLE);
            vTaskDelay(1);
            DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreTrigger, 5.0f, DM_LOCATION_SPEED, CAN_TX_RETRY_ENABLE);
            vTaskDelay(1);
            while (1)
            {
                // 检查调试模式，如果激活则释放信号量等待
                if (ControlState_IsManualOverride())
                {
                    xSemaphoreGive(g_xMotorCtrlSemHandle);
                    // 等待调试模式结束
                    while (ControlState_IsManualOverride())
                    {
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                    // 等待调试任务释放信号量（给100ms余量）
                    vTaskDelay(pdMS_TO_TICKS(100));
                    // 重新获取信号量（使用超时避免死锁）
                    while (xSemaphoreTake(g_xMotorCtrlSemHandle, pdMS_TO_TICKS(100)) != pdTRUE)
                    {
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    // 重新发送目标位置
                    DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreTrigger, 5.0f, DM_LOCATION_SPEED, CAN_TX_RETRY_ENABLE);
                    DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreTrigger, 5.0f, DM_LOCATION_SPEED, CAN_TX_RETRY_ENABLE);
                }
                DM_Motor_RefreshData(DM_3519_STRENTH_LEFT, CAN_TX_RETRY_ENABLE);
                vTaskDelay(1);
                DM_Motor_RefreshData(DM_3519_STRENTH_RIGHT, CAN_TX_RETRY_ENABLE);
                vTaskDelay(1);
                left_pos = Motor_GetTotalAngle(DM_3519_STRENTH_LEFT);
                right_pos = Motor_GetTotalAngle(DM_3519_STRENTH_RIGHT);

                if (IS_IN_DEADZONE(left_pos, LeftStoreTrigger, MOTOR_DEAD_ZONE) &&
                    IS_IN_DEADZONE(right_pos, RightStoreTrigger, MOTOR_DEAD_ZONE))
                {
                    break;
                }
                vTaskDelay(3);
            }

            StoreState++;
            break;
        }

        case 0x02:
        {
            // ========== 滑台叉上移 + 发射 ==========
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_store);
            vTaskDelay(1307);
            DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreTop, 5.0f, DM_LOCATION_SPEED, CAN_TX_RETRY_ENABLE);
            DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreTop, 5.0f, DM_LOCATION_SPEED, CAN_TX_RETRY_ENABLE);
            while (1)
            {
                // 检查调试模式
                if (ControlState_IsManualOverride())
                {
                    xSemaphoreGive(g_xMotorCtrlSemHandle);
                    while (ControlState_IsManualOverride())
                    {
                        vTaskDelay(pdMS_TO_TICKS(50));
                    }
                    // 等待调试任务释放信号量（给100ms余量）
                    vTaskDelay(pdMS_TO_TICKS(100));
                    // 重新获取信号量（使用超时避免死锁）
                    while (xSemaphoreTake(g_xMotorCtrlSemHandle, pdMS_TO_TICKS(100)) != pdTRUE)
                    {
                        vTaskDelay(pdMS_TO_TICKS(10));
                    }
                    DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreTop, 5.0f, DM_LOCATION_SPEED, CAN_TX_RETRY_ENABLE);
                    DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreTop, 5.0f, DM_LOCATION_SPEED, CAN_TX_RETRY_ENABLE);
                }

                DM_Motor_RefreshData(DM_3519_STRENTH_LEFT, CAN_TX_RETRY_ENABLE);
                DM_Motor_RefreshData(DM_3519_STRENTH_RIGHT, CAN_TX_RETRY_ENABLE);
                left_pos = MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1].motor_data.solved_data[0];
                right_pos = MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1].motor_data.solved_data[0];

                if (IS_IN_DEADZONE(left_pos, LeftStoreTop, MOTOR_DEAD_ZONE) &&
                    IS_IN_DEADZONE(right_pos, RightStoreTop, MOTOR_DEAD_ZONE))
                {
                    break;
                }
                vTaskDelay(3);
            }

            // 发射
            HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_shoot);
            vTaskDelay(1307);

            // 无换弹测试：发射后直接回到下一轮
            StoreState = 0x00;
            break;
        }
        default:
            break;
        }

        // 释放信号量，允许其他任务或调试任务获取
        xSemaphoreGive(g_xMotorCtrlSemHandle);

        vTaskDelay(7); // 短暂让出CPU
    }
}

/************************************
 * 函数名：StateSetTaskFunc
 * 作用：  飞镖状态设置
 * 参数：  无
 * // TODO：  等待链接上位机和测试无上位机版本
 ***********************************/
void StateSetTaskFunc(void *argument)
{
    // 接收上位机 / 遥控器 传递的数据
    vTaskDelay(1);
    float degree = 0.0f;
    float preseting_distance = MotorManager.MotorList[1].motor_data.offset_ecd_angle; // pay attention to this params, its unit is degree not rad!!!!
    float preseting_yaw = 0.0f;                                                       // this param is limited at (-160.0f, 160.0f)
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    while (1)
    {
        // while (!IS_IN_DEADZONE(Motor_GetTotalAngle(RM_2006_TRIGGER), preseting_distance, 2.0f))
        // {
        //     RmMotorPID_Calc(RM_2006_TRIGGER, preseting_distance, CAN_TX_RETRY_DISABLE);
        //     vTaskDelay(1);
        // }
        // DmMotorSendCfg(DM_4310_YAW, preseting_yaw, 0.0f, DM_MIT, CAN_TX_RETRY_ENABLE); // 调节Yaw轴位置
        // while (degree = Motor_GetTotalAngle(DM_4310_YAW), !IS_IN_DEADZONE(degree, preseting_yaw, 1.0f))
        // {
        //     DM_Motor_RefreshData(DM_4310_YAW, CAN_TX_RETRY_ENABLE);
        //     vTaskDelay(1);
        // }
        // 事件组唤醒所有其他的任务
        xEventGroupSetBits(g_pxStateSetEventGroupHandeler, EVENT_ALL_READY);
        vTaskDelay(1);
        vTaskSuspend(NULL);
    }
}
