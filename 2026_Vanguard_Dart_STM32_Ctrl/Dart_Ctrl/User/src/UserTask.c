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
 * 现在的毛胚房版本是没有加上视觉和上位机的,但是要用的话那么,首先要注释掉串口通信部份的代码,也就是说
 * 无视觉方案:
 * 云台不动或者固定动一个角度/扳机移动之后直接释放任务通知,储能开始,储能到底换弹之地,发信号量,开始换弹？或者通过信号量改变数据,储能电机上升到扳机位置,释放飞镖
 ***********************************************************************************************************************/
#include "UserTask.h"

// 19271（上限）<长度>
// 第三个（距离7547）
// 第二个（距离13200）<与第三个相距5653>
// 第一个（距离18749）<与第二个相距5549>

// 任务：
// 换弹任务
// 扳机任务 (包含调节扳机松紧任务)
// 通信任务 (can, uart)
// 存储拉簧动能任务
// yaw调整任务

/************************全局或静态作用域*********************/
static DartPacket_t g_xDartData;
static ServoPacket_t g_xServoData;
static __IO uint8_t DartNum = 4;
extern MotorManager_t MotorManager;
static float MotorData = 0.0f;
static __IO float GripperTarget = 0.0f; // 位置不动

// 储能任务互斥量
static StaticSemaphore_t g_xLoadMutexBuffer;
static SemaphoreHandle_t g_xLoadMutexHandle;

// 储能任务信号量
static StaticSemaphore_t g_xStoreSemaphore;
SemaphoreHandle_t g_xStoreSemaphoreHandle;

// 换弹结构电机队列
static uint8_t g_ucReloadQueueStorage[4 * sizeof(uint8_t)];
static StaticQueue_t g_xReloadQueue;
static QueueHandle_t g_xLoad3508QueueHandler;

// Uart信号量
static StaticSemaphore_t g_xUartSemaphore;
SemaphoreHandle_t g_xUartSemaphoreHandle;

// Uart任务的Stream流传输
static StreamBufferHandle_t xGimbalStreamBuf;
static StreamBufferHandle_t xTriggerStreamBuf;

// Load任务的事件组
static StaticEventGroup_t g_pxLoadEventGroupBuffer;
static EventGroupHandle_t g_pxLoadEventGroupHandeler;
/*---------------------------------------------------------------------------------------*/
// 模块驱动初始化
void Module_Init(void)
{
    BSP_POWER_DeInit(); // 失能无绿灯，亮红灯
    DWT_Init(180);
    MotorInit();
    CanFliterCfg();
    BSP_UART_Init();
    // while (!ServoInit())
    //     ; // 这个地方有一个回调,需要进行数据读取
    ServoInit();
    // HAL_TIMEx_PWM_Start();

    // 云台遥控器初始化
}

/// @brief 创建RTOS的通信量
/// @param  无
void RTOS_ModuleInit(void)
{
    // 任务通知 ：给串口通信确认是操控什么类型的（2006 / 4310 / HX06L）

    // 软件定时器：定时向上位机回复确认当前状态正常，否则加一些安全措施，Tick大概30s触发一次

    // QUEUE（队列）：1.融合到云台调节当中，保证云台调节正常<队列集>; 2.融入到换弹结构当中,确保换弹结构正常
    g_xLoad3508QueueHandler = xQueueCreateStatic(4, sizeof(uint8_t), g_ucReloadQueueStorage, &g_xReloadQueue); // 这个队列用于换弹结构当中

    // SEMAPHORE（信号量）：信号量与DartNum共同决定飞镖换弹的状态
    // g_xUartSemaphoreHandle = xSemaphoreCreateCountingStatic(3, 0, &g_xUartSemaphore); // 初始值为0，上限为2，接收时候释放
    g_xStoreSemaphoreHandle = xSemaphoreCreateCountingStatic(5, sizeof(uint8_t), &g_xStoreSemaphore);

    // MUTEX（互斥量）：调控云台方式只有一种，不允许上位机和遥控器同时调控
    // g_xGimbalMutexHandle = xSemaphoreCreateMutexStatic(&g_xGimbalMutexBuffer);
    g_xLoadMutexHandle = xSemaphoreCreateMutexStatic(&g_xLoadMutexBuffer);

    // 使用stream流传输,也可以改成使用Message传输
    xGimbalStreamBuf = xStreamBufferCreate(1, 1);  // 1字节触发,总共空间1字节
    xTriggerStreamBuf = xStreamBufferCreate(1, 1); // 1字节触发

    // EVENTGROUPS（事件组）：用于确认云台电机和射程电机正常设置,只有这两个正常设置之后才可以进行下面的流程
    g_pxLoadEventGroupHandeler = xEventGroupCreateStatic(&g_pxLoadEventGroupBuffer);
}

/***********************************************TASK******************************************************/
// 云台调节任务(yaw调整任务),默认优先度较低,继承中断函数优先级
osThreadId_t GimbalTaskHandle;
const osThreadAttr_t GimbalTask_attributes = {
    .name = "GimbalTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

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

// CAN通信任务(似乎已经直接存在于电机调控过程)
// osThreadId_t CanModuleTaskHandle;
// const osThreadAttr_t CanModuleTask_attributes = {
//     .name = "CanModuleTaskHandle",
//     .stack_size = 128 * 4,
//     .priority = (osPriority_t)osPriorityNormal,
// };

// UART通信任务 (上位机 / 总线舵机   共同使用)
osThreadId_t UartModuleTaskHandle;
const osThreadAttr_t UartModuleTask_attributes = {
    .name = "UartModuleTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void GimbalTaskFunc(void *argument);
void StoreEnergyTaskFunc(void *argument);
void LoadTaskFunc(void *argument);
void ShootTaskFunc(void *argument);
void UartModuleTaskFunc(void *argument);
void LoadMotorTaskFunc(void *argument);

/***********************************
 * 函数名: TaskInitFunc
 * 作用:   任务初始化创建
 * 参数:   无
 **********************************/
void TaskInitFunc(void)
{
    RTOS_ModuleInit();

    // 任务初始化
    GimbalTaskHandle = osThreadNew(GimbalTaskFunc, NULL, &GimbalTask_attributes);
    StoreEnergyTaskHandle = osThreadNew(StoreEnergyTaskFunc, NULL, &StoreEnergyTask_attributes);
    LoadTaskHandle = osThreadNew(LoadTaskFunc, NULL, &LoadTask_attributes);
    ShootTaskHandle = osThreadNew(ShootTaskFunc, NULL, &ShootTask_attributes);
    UartModuleTaskHandle = osThreadNew(UartModuleTaskFunc, NULL, &UartModuleTask_attributes);
}

/**********************************
 * 函数名：GimbalTaskFunc
 * 作用：  调节飞镖云台
 * 参数：  无
 *********************************/
void GimbalTaskFunc(void *argument)
{
    // 87 FF 7F F0 0B 33 37 FF -> 位置 10 rad, 速度 0 rad/s, kp = 1.4591, kd = 1.0f, 转矩0.0f
    // 7F FF 7F F0 0B 33 37 FF -> 位置变为 0 rad

    float GimablTarget = 0.0f;

    while (1)
    {
        // 根据上位机和遥控器的命令对4310进行旋转
        xStreamBufferReceive(xGimbalStreamBuf, &GimablTarget, sizeof(float), portMAX_DELAY);
        DM_Motor_Enable(&MotorManager.MotorList[DM_4310_YAW - 1]);
        DmMotorSendCfg(DM_4310_YAW, GimablTarget, 0.0f, DM_MIT);

        // 等待云台电机到达目标位置，使用死区判定并带超时保护
        {
            uint32_t timeout = osKernelGetTickCount() + MOTOR_TIMEOUT_MS;
            float current_pos;

            while (1)
            {
                current_pos = MotorManager.MotorList[DM_4310_YAW - 1].motor_data.solved_data[0];

                // 检查是否到达目标位置（使用死区判定）
                if (fabsf(current_pos - GimablTarget) <= MOTOR_DEAD_ZONE)
                {
                    break; // 到达目标
                }

                // 超时保护
                if (osKernelGetTickCount() > timeout)
                {
                    // TODO: 添加错误处理（日志、报警等）
                    break;
                }

                DmMotorSendCfg(DM_4310_YAW, GimablTarget, 0.0f, DM_MIT);
                osDelay(10);
            }
        }

        DM_Motor_Disable(&MotorManager.MotorList[DM_4310_YAW - 1]);
        // 这里释放一个事件组
        xEventGroupSetBits(g_pxLoadEventGroupHandeler, EVENT_GIMBAL_READY);
    }
}

/***********************************
 * 函数名: StoreEnergyTaskFunc
 * 作用:   拉簧储能任务(控制左右3519电机转动)
 * 参数:   无
 **********************************/
void StoreEnergyTaskFunc(void *argument)
{
    // 等待事件组：云台和扳机都就绪后才开始
    xEventGroupWaitBits(g_pxLoadEventGroupHandeler, EVENT_ALL_READY, pdTRUE, pdTRUE, portMAX_DELAY);

    // 接收计数型信号量之后才可以正常,这里更新第一次目标值（下拉至换弹位置 -> 上拉至扳机位置 -> 释放（同时回到零点）-> 下一次循环）

    DM_Motor_Enable(&MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1]);
    DM_Motor_Enable(&MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1]);
    uint8_t StoreState = 0x00;
    vTaskDelay(POWER_ON_DELAY_MS); // 上电保护延迟

    // J3519电机运动从顶部到底部
    while (1)
    {
        // 由StoreState决定循环
        switch (StoreState)
        {
        case 0x00:
            // 发射滑台下移到达换弹位置
            // 循环到达发射位置,释放计数量/信号量/Stream流
            // 等待对方执行完任务,这可以直接读DartNum进行确定
            DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreBottom, 5.0f, DM_LOCATION_SPEED);
            DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreBottom, 5.0f, DM_LOCATION_SPEED);

            // 等待电机到达目标位置，带超时保护
            {
                uint32_t timeout = osKernelGetTickCount() + MOTOR_TIMEOUT_MS;
                float left_pos, right_pos;

                while (1)
                {
                    left_pos = MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1].motor_data.solved_data[0];
                    right_pos = MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1].motor_data.solved_data[0];

                    // 检查是否都到达目标位置（死区判定）
                    if (IS_IN_DEADZONE(left_pos, LeftStoreBottom, MOTOR_DEAD_ZONE) &&
                        IS_IN_DEADZONE(right_pos, RightStoreBottom, MOTOR_DEAD_ZONE))
                    {
                        break; // 到达目标
                    }

                    // 超时保护
                    if (osKernelGetTickCount() > timeout)
                    {
                        // TODO: 添加错误处理（日志、报警等）
                        break;
                    }

                    // 刷新电机数据
                    osDelay(10);
                }
            }

            // 这里通信,改变飞镖数值之后释放一个互斥量让其进行运动
            xSemaphoreGive(g_xStoreSemaphoreHandle);
            xSemaphoreGive(g_xLoadMutexHandle);
            StoreState++;
            break;
        case 0x01:
            // 发射滑台上移到达扳机位置
            // 释放一个信号量或者任务通知量进行通信
            DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreTrigger, 5.0f, DM_LOCATION_SPEED);
            DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreTrigger, 5.0f, DM_LOCATION_SPEED);

            // 等待电机到达目标位置，带超时保护
            {
                uint32_t timeout = osKernelGetTickCount() + MOTOR_TIMEOUT_MS;
                float left_pos, right_pos;

                while (1)
                {
                    left_pos = MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1].motor_data.solved_data[0];
                    right_pos = MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1].motor_data.solved_data[0];

                    // 检查是否都到达目标位置（死区判定）
                    if (IS_IN_DEADZONE(left_pos, LeftStoreTrigger, MOTOR_DEAD_ZONE) &&
                        IS_IN_DEADZONE(right_pos, RightStoreTrigger, MOTOR_DEAD_ZONE))
                    {
                        break; // 到达目标
                    }

                    // 超时保护
                    if (osKernelGetTickCount() > timeout)
                    {
                        // TODO: 添加错误处理（日志、报警等）
                        break;
                    }

                    // 刷新电机数据
                    osDelay(10);
                }
            }

            StoreState++;
            break;
        case 0x02:
            // 发射滑台飞速向上发射
            DmMotorSendCfg(DM_3519_STRENTH_LEFT, StoreLocPrimitive, ShotSpeed, DM_LOCATION_SPEED);
            DmMotorSendCfg(DM_3519_STRENTH_RIGHT, StoreLocPrimitive, ShotSpeed, DM_LOCATION_SPEED);

            // 等待电机到达目标位置，带超时保护
            {
                uint32_t timeout = osKernelGetTickCount() + MOTOR_TIMEOUT_MS;
                float left_pos, right_pos;

                while (1)
                {
                    left_pos = MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1].motor_data.solved_data[0];
                    right_pos = MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1].motor_data.solved_data[0];

                    // 检查是否都到达目标位置（死区判定）
                    if (IS_IN_DEADZONE(left_pos, StoreLocPrimitive, MOTOR_DEAD_ZONE) &&
                        IS_IN_DEADZONE(right_pos, StoreLocPrimitive, MOTOR_DEAD_ZONE))
                    {
                        break; // 到达目标
                    }

                    // 超时保护
                    if (osKernelGetTickCount() > timeout)
                    {
                        // TODO: 添加错误处理（日志、报警等）
                        break;
                    }

                    // 刷新电机数据
                    osDelay(10);
                }
            }

            vTaskDelay(RELOAD_BUFFER_MS); // 等待缓冲进行换弹
            StoreState = 0x00;
            break;
        default:
            break;
        }

        if (uxSemaphoreGetCount(g_xStoreSemaphoreHandle) >= 4)
        {
            // 4发打完，失能所有电机，实际上这里需要归中
            DM_Motor_Disable(&MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1]);
            DM_Motor_Disable(&MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1]);
            DM_Motor_Disable(&MotorManager.MotorList[DM_4310_YAW - 1]);

            // 挂起任务（可恢复）
            vTaskSuspend(NULL);
        }

        // 根据反馈数据进行确定是否放飞镖
        // xSemaphoreCreateCounting
        // xSemaphoreGive
        // xSemaphoreTake
    }
    // J3519电机底部等待换弹
}

/***********************************
 * 函数名: LoadTaskFunc
 * 作用:   换弹任务（传送带电机控制以及电机目标设定）
 * 参数:   无
 **********************************/
void LoadTaskFunc(void *argument)
{
    uint8_t servo_ids[3] = {0x01, 0x02, 0x03};
    uint16_t servo_angles[3] = {0x0000};
    uint16_t servo_work_angle[3] = {SeperationAngle, SeperationAngle, SeperationAngle};
    bool MutexTake = false;
    UBaseType_t dart_num = 0x0000;

    // 直接设置成0°,限位确认
    ServoControlMulti(3, servo_ids, servo_angles, 300); // 当前角度是0°,在设置后(MCU驱动板上电无法读取)
    float offset_angle = MotorManager.MotorList[RM_3508_GRIPPER - 1].motor_data.offset_ecd_angle;

    // 任务创建
    TaskHandle_t Load3508TaskHandle = NULL;
    xTaskCreate(LoadMotorTaskFunc, "LoadMotor", 64 * 4, NULL, osPriorityAboveNormal, &Load3508TaskHandle); // 换弹3508电机任务
    GripperTarget = RmMotorRemoveBias(RM_3508_GRIPPER, GripperTarget, false);
    while (1)
    {
        xSemaphoreTake(g_xLoadMutexHandle, portMAX_DELAY); // 等待互斥量,之后进一个循环

        dart_num = uxQueueMessagesWaiting((QueueHandle_t)g_xStoreSemaphoreHandle);
        switch (dart_num)
        {
        case 1:
            GripperTarget = 0.0f;
            MutexTake = false;
            xSemaphoreGive(g_xLoadMutexHandle); // 等待互斥量,之后进一个循环
            break;
        case 2:
            GripperTarget = FirstServoLoc;
            MutexTake = true;
            break;
        case 3:
            GripperTarget = SecondServoLoc;
            MutexTake = true;
            break;
        case 4:
            GripperTarget = ThirdServoLoc;
            MutexTake = true;
            break;
        default:
            break;
        }
        GripperTarget = RmMotorRemoveBias(RM_3508_GRIPPER, GripperTarget, true);
        while (MutexTake)
        {
            MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
            RmMotorPID_Calc(RM_3508_GRIPPER, GripperTarget);

            // 使用死区判断：MotorData 在 [GripperTarget-MOTOR_DEAD_ZONE, GripperTarget+MOTOR_DEAD_ZONE] 范围内视为到达
            if (IS_IN_DEADZONE(MotorData, FirstServoLoc, MOTOR_DEAD_ZONE) &&
                (dart_num == 2)) // 首次上电电机的零点为offest_ecd,负向转动,距离为-7547
            {
                xQueueSend(g_xLoad3508QueueHandler, (const void *)&DartNum, 0);
                vTaskDelay(SERVO_MOVE_TIME_MS);
                // 等待电机到达过渡位置
                MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
                // 回到上电零点位置：当前累计角度的负值，带超时保护
                {
                    uint32_t timeout = osKernelGetTickCount() + MOTOR_TIMEOUT_MS;
                    while (!IS_IN_DEADZONE(MotorData, 0, MOTOR_DEAD_ZONE))
                    {
                        // 超时保护
                        if (osKernelGetTickCount() > timeout)
                        {
                            // TODO: 添加错误处理
                            break;
                        }

                        GripperTarget = RmMotorRemoveBias(RM_3508_GRIPPER, -MotorData, true);
                        RmMotorPID_Calc(RM_3508_GRIPPER, GripperTarget);
                        MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER); // 更新位置数据
                        osDelay(10);
                    }
                }
                // 更新数据位置应该有所变化
                // GripperTarget = SecondServoLoc; // 这里的数据变化还得是靠别人传进来,不然就轧钢了
                // GripperTarget = RmMotorRemoveBias(RM_3508_GRIPPER, GripperTarget, true);
                MutexTake = false;
                xSemaphoreGive(g_xLoadMutexHandle); // 等待互斥量,之后进一个循环
            }

            if (IS_IN_DEADZONE(MotorData, SecondServoLoc, MOTOR_DEAD_ZONE) &&
                (dart_num == 3)) // 负向转动,距离为-5653
            {
                xQueueSend(g_xLoad3508QueueHandler, (const void *)&DartNum, 0);
                vTaskDelay(SERVO_MOVE_TIME_MS);
                // 等待电机到达过渡位置
                MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
                // 回到上电零点位置：当前累计角度的负值，带超时保护
                {
                    uint32_t timeout = osKernelGetTickCount() + MOTOR_TIMEOUT_MS;
                    while (!IS_IN_DEADZONE(MotorData, 0, MOTOR_DEAD_ZONE))
                    {
                        if (osKernelGetTickCount() > timeout)
                        {
                            // TODO: 添加错误处理
                            break;
                        }

                        GripperTarget = RmMotorRemoveBias(RM_3508_GRIPPER, -MotorData, true);
                        RmMotorPID_Calc(RM_3508_GRIPPER, GripperTarget);
                        MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER); // 更新位置数据
                        osDelay(10);
                    }
                }
                // GripperTarget = ThirdServoLoc;
                // GripperTarget = RmMotorRemoveBias(RM_3508_GRIPPER, GripperTarget, true);
                MutexTake = false;
                xSemaphoreGive(g_xLoadMutexHandle); // 等待互斥量,之后进一个循环
            }

            if (IS_IN_DEADZONE(MotorData, ThirdServoLoc, MOTOR_DEAD_ZONE) &&
                (dart_num == 4)) // 负向转动,距离为-5549
            {
                xQueueSend(g_xLoad3508QueueHandler, (const void *)&DartNum, 0);
                vTaskDelay(SERVO_MOVE_TIME_MS);
                // 等待电机到达过渡位置
                MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
                // 回到上电零点位置：当前累计角度的负值，带超时保护
                {
                    uint32_t timeout = osKernelGetTickCount() + MOTOR_TIMEOUT_MS;
                    while (!IS_IN_DEADZONE(MotorData, 0, MOTOR_DEAD_ZONE))
                    {
                        if (osKernelGetTickCount() > timeout)
                        {
                            // TODO: 添加错误处理
                            break;
                        }

                        GripperTarget = RmMotorRemoveBias(RM_3508_GRIPPER, -MotorData, true);
                        RmMotorPID_Calc(RM_3508_GRIPPER, GripperTarget);
                        MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER); // 更新位置数据
                        osDelay(10);
                    }
                }
                // GripperTarget = RmMotorRemoveBias(RM_3508_GRIPPER, 0.0f, true);
                MutexTake = false;
                xSemaphoreGive(g_xLoadMutexHandle); // 等待互斥量,之后进一个循环
            }
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
    uint8_t fQueueDartNum = 0; // 目标为0最初
    vTaskDelay(1);             // 最开始放一个Tick
    // 在这里驱动换弹的3508电机运动
    while (1)
    {
        xQueueReceive(g_xLoad3508QueueHandler, &fQueueDartNum, portMAX_DELAY); // 读取队列得到对应的数据, 不等待数据更新
        // 获取信号量之后进行判断
        if (fQueueDartNum == 3)
        {
            ServoControlPos(0x03, SeperationAngle, SERVO_MOVE_TIME_MS); // 转90°分离
            vTaskDelay(SERVO_MOVE_TIME_MS + 5);
            ServoControlPos(0x03, 0x0000, SERVO_MOVE_TIME_MS);
            vTaskDelay(SERVO_MOVE_TIME_MS + 5);
        }
        else if (fQueueDartNum == 2)
        {
            ServoControlPos(0x02, SeperationAngle, SERVO_MOVE_TIME_MS); // 转90°分离
            vTaskDelay(SERVO_MOVE_TIME_MS + 5);
            ServoControlPos(0x02, 0x0000, SERVO_MOVE_TIME_MS);
            vTaskDelay(SERVO_MOVE_TIME_MS + 5);
        }
        else if (fQueueDartNum == 1)
        {
            ServoControlPos(0x01, SeperationAngle, SERVO_MOVE_TIME_MS); // 转90°分离
            vTaskDelay(SERVO_MOVE_TIME_MS + 5);
            ServoControlPos(0x01, 0x0000, SERVO_MOVE_TIME_MS);
            vTaskDelay(SERVO_MOVE_TIME_MS + 5);
        }
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
    // 1.这里放一个队列，等待解析上位机发来的射程调整数据之后直接激活扳机位置调节，随时准备调节射程
    float TriggerTarget = 0.0f; // 先转1圈
    xStreamBufferReceive(xTriggerStreamBuf, &TriggerTarget, sizeof(float), portMAX_DELAY);

    // 等待扳机电机到达目标位置，使用死区判定并带超时保护
    {
        uint32_t timeout = osKernelGetTickCount() + MOTOR_TIMEOUT_MS;
        float current_angle;

        while (1)
        {
            current_angle = Motor_GetTotalAngle(RM_2006_TRIGGER) - MotorManager.MotorList[RM_2006_TRIGGER - 1].motor_data.offset_ecd;

            // 检查是否到达目标位置（使用死区判定）
            if (fabsf(current_angle - TriggerTarget) <= TRIGGER_DEAD_ZONE)
            {
                break; // 到达目标
            }

            // 超时保护
            if (osKernelGetTickCount() > timeout)
            {
                // TODO: 添加错误处理（日志、报警等）
                break;
            }

            TriggerTarget = RmMotorRemoveBias(RM_2006_TRIGGER, TriggerTarget, false); // 位置环,这里先转1圈看看,主要不知道限幅
            RmMotorPID_Calc(RM_2006_TRIGGER, TriggerTarget);
            // RmMotorSendCfg(RM_2006_TRIGGER, 5000);
            osDelay(10);
        }
    }

    // 转到指定位置,直接归位这个扳机
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MG996R_store); // 这个地方设置为未释放状态
    // 这里释放一个事件组
    xEventGroupSetBits(g_pxLoadEventGroupHandeler, EVENT_TRIGGER_READY);

    while (1)
    {
        // 2.这里进行正常的发射函数,TIM2_CH1-PD12->A板H接口
        // 根据同步带电机反馈数据进行发射
        // 接收任务通知 / 互斥量
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MG996R_store); // 这个地方设置为未释放状态

        // delay一会再次释放
        vTaskDelay(100);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MG996R_shoot);
    }
}

/************************************
 * 函数名：UartModuleTaskFunc
 * 作用：  获取串口数据包任务
 * 参数：  无
 * // TODO：  等待链接上位机和测试无上位机版本
 ***********************************/
void UartModuleTaskFunc(void *argument)
{
    // 等待TaskNotification
    uint32_t xTaskNotificationNum = 0x00;
    while (1)
    {
        // xTaskNotifyWait(0xffffffff, 0xffffffff, &xTaskNotificationNum, portMAX_DELAY); // 32位全部清空,直接死等
        // switch (xTaskNotificationNum)
        // {
        // case 0x00:
        //     break;

        //     // 有MCU控制协议
        // case 0x01:
        //     xTaskNotificationNum = 0x00;
        //     // 处理数据
        //     UartModule_GetServoPacket(BSP_UART6, &g_xServoData);
        //     // 给队列处理数据
        //     break;

        //     // 无MCU控制协议
        // case 0x02:
        //     xTaskNotificationNum = 0x00;
        //     // 处理数据
        //     UartModule_GetServoPacket(BSP_UART6, &g_xServoData);
        //     // 给队列处理数据
        //     break;

        //     // DART通信协议
        // case 0x03:
        //     xTaskNotificationNum = 0x00;
        //     // 处理数据
        //     // 对数据进行解析，看看是什么要求？是云台调整还是扳机调整
        //     UartModule_GetDartPacket(BSP_UART3, &g_xDartData);
        //     if (g_xDartData.is_valid)
        //     {
        //         // 4310电机调节
        //         if (g_xDartData.data[0] == 0x43)
        //         {
        //             // 这里给一个队列
        //         }
        //         // 4310电机调节射程
        //         if (g_xDartData.data[0] == 0x60)
        //         {
        //             // 这里给一个队列
        //         }
        //     }
        //     break;

        // default:
        //     Error_Handler();
        // }

        // 这部分逻辑跑无视觉上位机处理

        float gimbal_data = 20.0f;    // 云台偏移20
        float trigger_data = 5184.0f; // 先转一圈

        // 通知云台任务进行运动,使用任务通知将量传输过去,这可以考虑不用这个任务通知
        xStreamBufferSend(xGimbalStreamBuf, &gimbal_data, sizeof(float), portMAX_DELAY);
        xStreamBufferSend(xTriggerStreamBuf, &trigger_data, sizeof(float), portMAX_DELAY);
    }
}
