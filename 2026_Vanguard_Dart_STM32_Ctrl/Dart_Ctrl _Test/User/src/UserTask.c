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
 * 遥控器(FS-I6)----------------------------------------------->FX-16AB接收机--------------------DM-J4310-YAW                  |
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
static ServoPacket_t g_xServoData;
static IbusPacket_t g_xIbusData;
extern MotorManager_t MotorManager;
static float MotorData = 0.0f;
static __IO float GripperTarget = 0.0f; // 位置不动

// Store <-> Shoot 双向同步信号量
static StaticSemaphore_t g_xStore2ShootSemBuffer;
static SemaphoreHandle_t g_xStore2ShootSemHandle; // Store通知Shoot开始
static StaticSemaphore_t g_xShoot2StoreSemBuffer;
static SemaphoreHandle_t g_xShoot2StoreSemHandle; // Shoot通知Store完成

// 储能任务信号量
static StaticSemaphore_t g_xStoreSemaphore;
SemaphoreHandle_t g_xStoreSemaphoreHandle;

// 储能任务消息
static StreamBufferHandle_t xLoadStreamBuf;

// 换弹结构电机队列
static uint8_t g_ucReloadQueueStorage[4 * sizeof(uint8_t)];
static StaticQueue_t g_xReloadQueue;
static QueueHandle_t g_xLoad3508QueueHandler;

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
    // QUEUE（队列）：1.融合到云台调节当中，保证云台调节正常<队列集>; 2.融入到换弹结构当中,确保换弹结构正常
    g_xLoad3508QueueHandler = xQueueCreateStatic(4, sizeof(uint8_t), g_ucReloadQueueStorage, &g_xReloadQueue); // 这个队列用于换弹结构当中

    // SEMAPHORE（信号量）：信号量与dart_num共同决定飞镖换弹的状态
    g_xStoreSemaphoreHandle = xSemaphoreCreateCountingStatic(5, sizeof(uint8_t), &g_xStoreSemaphore);

    // 二值信号量：用于任务间双向同步（创建后初始状态为"空"）
    g_xStore2ShootSemHandle = xSemaphoreCreateBinaryStatic(&g_xStore2ShootSemBuffer);
    g_xShoot2StoreSemHandle = xSemaphoreCreateBinaryStatic(&g_xShoot2StoreSemBuffer);

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
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};

// 换弹任务
osThreadId_t LoadTaskHandle;
const osThreadAttr_t LoadTask_attributes = {
    .name = "LoadTask",
    .stack_size = 160 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal7,
};

// 扳机调节与发射任务(等待任务通知)
osThreadId_t ShootTaskHandle;
const osThreadAttr_t ShootTask_attributes = {
    .name = "ShootTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

// 飞镖状态设置(设置Yaw和射程)
osThreadId_t StateSetTaskHandle;
const osThreadAttr_t StateSetTask_attributes = {
    .name = "StateSetTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void StoreEnergyTaskFunc(void *argument);
void ShootTaskFunc(void *argument);
void StateSetTaskFunc(void *argument);

/***********************************
 * 函数名: TaskInitFunc
 * 作用:   任务初始化创建
 * 参数:   无
 **********************************/
void TaskInitFunc(void)
{
    RTOS_ModuleInit();

    // 控制状态模块初始化
    ControlState_Init();

    // 任务初始化
    StoreEnergyTaskHandle = osThreadNew(StoreEnergyTaskFunc, NULL, &StoreEnergyTask_attributes);
    ShootTaskHandle = osThreadNew(ShootTaskFunc, NULL, &ShootTask_attributes);
    StateSetTaskHandle = osThreadNew(StateSetTaskFunc, NULL, &StateSetTask_attributes);

    // 创建遥控器控制任务
    ControlState_CreateTasks();
}

/***********************************
 * 函数名: StoreEnergyTaskFunc
 * 作用:   拉簧储能任务(控制左右3519电机转动)
 * 参数:   无
 **********************************/
void StoreEnergyTaskFunc(void *argument)
{
    // 等待事件组：云台和扳机都就绪后才开始
    xEventGroupWaitBits(g_pxStateSetEventGroupHandeler, EVENT_ALL_READY, pdTRUE, pdTRUE, portMAX_DELAY);
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    // 接收计数型信号量之后才可以正常,这里更新第一次目标值（下拉至换弹位置 -> 上拉至扳机位置 -> 释放（同时回到零点）-> 下一次循环）
    HAL_Delay(10);
    DM_Motor_Enable(&MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1]);
    vTaskDelay(10);
    DM_Motor_Enable(&MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1]);
    vTaskDelay(10);
    uint8_t StoreState = 0x00;
    int8_t Dart = 4;
    vTaskDelay(POWER_ON_DELAY_MS); // 上电保护延迟
    float left_pos = 0.0f, right_pos = 0.0f;

    // J3519电机运动从顶部到底部
    // TODO:当前运动有问题，有一边不运动
    while (1)
    {
        // 检查是否处于手动调试模式，如果是则暂停状态机
        // 在调试窗口期间，让遥控器接管电机控制
        while (ControlState_IsManualOverride())
        {
            vTaskDelay(pdMS_TO_TICKS(50)); // 等待手动模式结束
        }
        // 由StoreState决定循环
        switch (StoreState)
        {
        case 0x00:
        {
            // 发射滑台下移到达换弹位置(即为底部位置)
            // 循环到达发射位置,释放计数量/信号量/Stream流
            // 等待对方执行完任务,这可以直接读DartNum进行确定
            DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreBottom, 5.0f, DM_LOCATION_SPEED);
            DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreBottom, 5.0f, DM_LOCATION_SPEED);

            bool temp = true;

            while (temp)
            {
                // 检测手动模式，退出内层循环让外层处理
                if (ControlState_IsManualOverride())
                    break;

                DM_Motor_RefreshData(DM_3519_STRENTH_LEFT);
                DM_Motor_RefreshData(DM_3519_STRENTH_RIGHT);
                left_pos = Motor_GetTotalAngle(DM_3519_STRENTH_LEFT);
                right_pos = Motor_GetTotalAngle(DM_3519_STRENTH_RIGHT);

                // 检查是否都到达目标位置（死区判定）
                if (IS_IN_DEADZONE(left_pos, LeftStoreBottom, MOTOR_DEAD_ZONE) &&
                    IS_IN_DEADZONE(right_pos, RightStoreBottom, MOTOR_DEAD_ZONE))
                {
                    temp = false; // 到达目标
                }
                // TODO: 添加错误处理（日志、报警等）
            }
            StoreState++;
            break;
        }

        case 0x01:
        {
            // 发射滑台上移到达扳机位置
            // 释放一个信号量或者任务通知量进行通信
            DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreTrigger, 5.0f, DM_LOCATION_SPEED);
            DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreTrigger, 5.0f, DM_LOCATION_SPEED);

            // 等待电机到达目标位置
            while (1)
            {
                // 检测手动模式，退出内层循环让外层处理
                if (ControlState_IsManualOverride())
                    break;

                left_pos = Motor_GetTotalAngle(DM_3519_STRENTH_LEFT);
                right_pos = Motor_GetTotalAngle(DM_3519_STRENTH_RIGHT);

                // 检查是否都到达目标位置（死区判定）
                if (IS_IN_DEADZONE(left_pos, LeftStoreTrigger, MOTOR_DEAD_ZONE) &&
                    IS_IN_DEADZONE(right_pos, RightStoreTrigger, MOTOR_DEAD_ZONE))
                {
                    break; // 到达目标
                }

                // TODO: 添加错误处理（日志、报警等）
            }

            StoreState++;
            break;
        }

        case 0x02:
        {
            // 发射滑台叉上移
            // 释放一个信号量或者任务通知量进行通信
            DM_MotorDisable(DM_3519_STRENTH_LEFT);
            DM_MotorDisable(DM_3519_STRENTH_RIGHT);
            // DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreTop, 5.0f, DM_LOCATION_SPEED);
            // DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreTop, 5.0f, DM_LOCATION_SPEED);

            // 等待电机到达目标位置
            while (1)
            {
                // 检测手动模式，退出内层循环让外层处理
                if (ControlState_IsManualOverride())
                    break;

                left_pos = MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1].motor_data.solved_data[0];
                right_pos = MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1].motor_data.solved_data[0];

                // 检查是否都到达目标位置（死区判定）
                if (IS_IN_DEADZONE(left_pos, LeftStoreTrigger, MOTOR_DEAD_ZONE) &&
                    IS_IN_DEADZONE(right_pos, RightStoreTrigger, MOTOR_DEAD_ZONE))
                {
                    break; // 到达目标
                }

                // TODO: 添加错误处理（日志、报警等）
            }
            // 通知Shoot任务开始发射
            xSemaphoreGive(g_xStore2ShootSemHandle); // 通知Shoot开始
            // TODO: 添加错误处理（日志、报警等），完善整体流程,单发测试标定参数之前这个地方直接死等
            xSemaphoreTake(g_xShoot2StoreSemHandle, portMAX_DELAY); // 等待Shoot完成
            Dart--;
            StoreState = 0;
            break;
        }
        default:
            break;
        }

        if (Dart == 0)
        {
            // 4发打完，失能所有电机，实际上这里需要归中
            // DM_Motor_Disable(&MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1]);
            // DM_Motor_Disable(&MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1]);
            // DM_Motor_Disable(&MotorManager.MotorList[DM_4310_YAW - 1]);
            // vTaskSuspend(StoreEnergyTaskHandle);
					   Dart = 4;
        }

        // 根据反馈数据进行确定是否放飞镖
        // xSemaphoreCreateCounting
        // xSemaphoreGive
        // xSemaphoreTake
    }
}

/***********************************
 * 函数名: ShootTaskFunc
 * 作用:   发射任务（舵机释放以及电机目标设定）
 * 参数:   无
 * // TODO:   增加code注释,为之后的更新作准备
 * 备注:   MG996R舵机直接转动的范围是( 500 ~ 2500 )
 **********************************/
void ShootTaskFunc(void *argument)
{
    xEventGroupWaitBits(g_pxStateSetEventGroupHandeler, EVENT_ALL_READY, pdTRUE, pdTRUE, portMAX_DELAY);
    vTaskDelay(10);
    while (1)
    {
        // 等待Store通知开始发射
        xSemaphoreTake(g_xStore2ShootSemHandle, portMAX_DELAY);
        HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_shoot);
        // delay一会再次释放
        vTaskDelay(2000);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_store); // 这个地方设置为未释放状态
        vTaskDelay(2000);
        xSemaphoreGive(g_xShoot2StoreSemHandle); // 通知Store完成

        // 开启20s调试窗口，允许遥控器手动调试
        ControlState_StartDebugWindow(DEBUG_WINDOW_MS);
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
    // preset the target at the base
    float temp = -5000.0f;
    float degree = 0.0f;
    float preseting_distance = RmMotorRemoveBias(RM_2006_TRIGGER, temp, true); // pay attention to this params, its unit is degree not rad!!!!
    float preseting_yaw = 0.0f;                                                // this param is limited at (-160.0f, 160.0f)
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    while (1)
    {
        // 调节位置,并等待位置调节成功,确保所有的任务处于阻塞态,防止其他任务有所影响,但是要确保所有电机停转并且失能

        while (!IS_IN_DEADZONE(Motor_GetTotalAngle(RM_2006_TRIGGER), temp, 2.0f))
        {
            RmMotorPID_Calc(RM_2006_TRIGGER, preseting_distance);
            vTaskDelay(1);
        }
        DmMotorSendCfg(DM_4310_YAW, preseting_yaw, 0.0f, DM_MIT); // 调节Yaw轴位置
        while (degree = Motor_GetTotalAngle(DM_4310_YAW), !IS_IN_DEADZONE(degree, preseting_yaw, 1.0f))
        {
            DM_Motor_RefreshData(DM_4310_YAW);
            vTaskDelay(1);
        }
        // 事件组唤醒所有其他的任务
        xEventGroupSetBits(g_pxStateSetEventGroupHandeler, EVENT_ALL_READY);
        // DM_MotorDisable(DM_4310_YAW);
        // DM_MotorDisable(DM_3519_STRENTH_LEFT);
        // DM_MotorDisable(DM_3519_STRENTH_RIGHT);
        vTaskDelay(1);

        // 最后进入阻塞态,等待遥控器解算唤醒 / 串口接收唤醒
        vTaskSuspend(NULL);
    }
}
