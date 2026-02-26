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
#include "IA6B.h"

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

/*============================== 遥控器总开关控制 ==============================*/
#if ENABLE_RC_MASTER_SWITCH
/**
 * @brief 遥控器总开关状态
 * @note  true = 允许执行正常流程（SWB=0）
 *        false = 只允许调试，不执行流程（SWB=1，默认）
 */
static volatile bool g_bRcFlowEnabled = false;

/**
 * @brief 遥控器数据有效标志
 */
static volatile bool g_bRcDataValid = false;

/**
 * @brief 获取遥控器流程使能状态
 * @return true-允许执行流程，false-只允许调试
 */
bool RC_IsFlowEnabled(void)
{
    return g_bRcFlowEnabled;
}

/**
 * @brief 获取遥控器数据有效状态
 * @return true-数据有效，false-数据无效
 */
bool RC_IsDataValid(void)
{
    return g_bRcDataValid;
}
#endif /* ENABLE_RC_MASTER_SWITCH */

// Store <-> Load 双向同步信号量
static StaticSemaphore_t g_xStore2LoadSemBuffer;
static SemaphoreHandle_t g_xStore2LoadSemHandle; // Store通知Load开始
static StaticSemaphore_t g_xLoad2StoreSemBuffer;
static SemaphoreHandle_t g_xLoad2StoreSemHandle; // Load通知Store完成

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
    g_xStore2LoadSemHandle = xSemaphoreCreateBinaryStatic(&g_xStore2LoadSemBuffer);
    g_xLoad2StoreSemHandle = xSemaphoreCreateBinaryStatic(&g_xLoad2StoreSemBuffer);
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

// 飞镖状态设置(设置Yaw和射程)
osThreadId_t StateSetTaskHandle;
const osThreadAttr_t StateSetTask_attributes = {
    .name = "StateSetTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

#if ENABLE_RC_MASTER_SWITCH
// 遥控器监控任务（高优先级，作为总开关）
osThreadId_t RcMonitorTaskHandle;
const osThreadAttr_t RcMonitorTask_attributes = {
    .name = "RcMonitorTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh, // 高优先级确保及时响应
};
void RcMonitorTaskFunc(void *argument);
#endif /* ENABLE_RC_MASTER_SWITCH */

void StoreEnergyTaskFunc(void *argument);
void LoadTaskFunc(void *argument);
void LoadMotorTaskFunc(void *argument);
void StateSetTaskFunc(void *argument);

/***********************************
 * 函数名: TaskInitFunc
 * 作用:   任务初始化创建
 * 参数:   无
 **********************************/
void TaskInitFunc(void)
{
    RTOS_ModuleInit();

#if ENABLE_RC_MASTER_SWITCH
    // 遥控器监控任务（最先创建，确保总开关优先运行）
    RcMonitorTaskHandle = osThreadNew(RcMonitorTaskFunc, NULL, &RcMonitorTask_attributes);
#endif

    // 任务初始化
    LoadTaskHandle = osThreadNew(LoadTaskFunc, NULL, &LoadTask_attributes);
    StoreEnergyTaskHandle = osThreadNew(StoreEnergyTaskFunc, NULL, &StoreEnergyTask_attributes);
    StateSetTaskHandle = osThreadNew(StateSetTaskFunc, NULL, &StateSetTask_attributes);
}

/***********************************
 * 函数名: StoreEnergyTaskFunc
 * 作用:   拉簧储能任务(控制左右3519电机转动)
 * 参数:   无
 **********************************/
void StoreEnergyTaskFunc(void *argument)
{
    // 等待事件组：云台和扳机都就绪后才开始
    xEventGroupWaitBits(g_pxStateSetEventGroupHandeler, EVENT_ALL_READY, pdFALSE, pdTRUE, portMAX_DELAY);
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

#if ENABLE_RC_MASTER_SWITCH
    // 等待遥控器数据有效且流程使能（SWB=0）
    while (!g_bRcDataValid || !g_bRcFlowEnabled)
    {
        // LED指示：等待遥控器切换到正常模式
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET); // 常亮表示进入正常流程
#endif

    // 接收计数型信号量之后才可以正常,这里更新第一次目标值（下拉至换弹位置 -> 上拉至扳机位置 -> 释放（同时回到零点）-> 下一次循环）

    DM_MotorEnable(DM_3519_STRENTH_LEFT);
    DM_MotorEnable(DM_3519_STRENTH_RIGHT);
    uint8_t StoreState = 0x00;
    uint8_t Dart = 4;
    vTaskDelay(POWER_ON_DELAY_MS); // 上电保护延迟
    float left_pos = 0.0f, right_pos = 0.0f;
    while (1)
    {
        switch (StoreState)
        {
        case 0x00:
        {
            // 发射滑台下移到达换弹位置
            DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreLoad, 5.0f, DM_LOCATION_SPEED);
            DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreLoad, 5.0f, DM_LOCATION_SPEED);
            bool temp = true;

            while (temp)
            {
                DM_Motor_RefreshData(DM_3519_STRENTH_LEFT);
                DM_Motor_RefreshData(DM_3519_STRENTH_RIGHT);
                left_pos = Motor_GetTotalAngle(DM_3519_STRENTH_LEFT);
                right_pos = Motor_GetTotalAngle(DM_3519_STRENTH_RIGHT);
                // 检查是否都到达目标位置（死区判定）
                if (IS_IN_DEADZONE(left_pos, LeftStoreLoad, MOTOR_DEAD_ZONE) &&
                    IS_IN_DEADZONE(right_pos, RightStoreLoad, MOTOR_DEAD_ZONE))
                {
                    temp = false; // 到达目标
                }
            }

            // 这里通信,改变飞镖数值之后通知Load任务开始工作
            xStreamBufferSend(xLoadStreamBuf, (const uint8_t *)(&Dart), 1, 0);
            xSemaphoreGive(g_xStore2LoadSemHandle);                // 通知Load开始
            xSemaphoreTake(g_xLoad2StoreSemHandle, portMAX_DELAY); // 等待Load完成
            // TODO:在打25米靶子的时候还需要将滑台和扳机移动到最底端，或者说扳机直接在最底端
            // TODO:到了之后还要等待一下扳机升起才可以发射
            // TODO:这个时候vTaskDelay(1500);
            StoreState++;
            break;
        }

        case 0x01:
        {
            // 发射滑台叉上移
            // 释放一个信号量或者任务通知量进行通信
            DmMotorSendCfg(DM_3519_STRENTH_LEFT, LeftStoreTop, 5.0f, DM_LOCATION_SPEED);
            DmMotorSendCfg(DM_3519_STRENTH_RIGHT, RightStoreTop, 5.0f, DM_LOCATION_SPEED);

            // 等待电机到达目标位置
            while (1)
            {
                left_pos = MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1].motor_data.solved_data[0];
                right_pos = MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1].motor_data.solved_data[0];

                // 检查是否都到达目标位置（死区判定）
                if (IS_IN_DEADZONE(left_pos, LeftStoreTop, MOTOR_DEAD_ZONE) &&
                    IS_IN_DEADZONE(right_pos, RightStoreTop, MOTOR_DEAD_ZONE))
                {
                    break; // 到达目标
                }

                // TODO: 添加错误处理（日志、报警等）
            }
            // 发射
            HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, MG996R_shoot);
            StoreState = 0x00;
            Dart--;
            break;
        }
        default:
            break;
        }

        if (Dart == 0)
        {
            // 4发打完，失能所有电机，实际上这里需要归中
            DmMotorSendCfg(DM_4310_YAW, 0.0f, 0.0f, DM_MIT);
            DM_Motor_Disable(&MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1]);
            DM_Motor_Disable(&MotorManager.MotorList[DM_3519_STRENTH_RIGHT - 1]);
            DM_Motor_Disable(&MotorManager.MotorList[DM_4310_YAW - 1]);
            vTaskSuspend(StoreEnergyTaskHandle);
        }
    }
}

/**
 * @brief 换弹到位后的通用处理：通知舵机、清积分、回零、通知储能
 * @param dart_num 当前飞镖编号
 * @param offset_angle 电机偏移角度
 */
static inline void LoadDart_ReturnToZero(uint8_t dart_num, float offset_angle)
{
    xQueueSend(g_xLoad3508QueueHandler, (const void *)&dart_num, 0);
    vTaskDelay(SERVO_MOVE_TIME_MS);
    CASCADE_PID_Clear_Integral(&MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid);
    GripperTarget = offset_angle;
    while (!IS_IN_DEADZONE(Motor_GetTotalAngle(RM_3508_GRIPPER), 0.0f, MOTOR_DEAD_ZONE))
    {
        RmMotorPID_Calc(RM_3508_GRIPPER, GripperTarget);
        osDelay(2);
    }
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
    xSemaphoreGive(g_xLoad2StoreSemHandle);
    if (dart_num == 1)
        vTaskDelay(200);
}

/***********************************
 * 函数名: LoadTaskFunc
 * 作用:   换弹任务（传送带电机控制以及电机目标设定）
 * 参数:   无
 **********************************/
void LoadTaskFunc(void *argument)
{
    xEventGroupWaitBits(g_pxStateSetEventGroupHandeler, EVENT_ALL_READY, pdFALSE, pdTRUE, portMAX_DELAY);

    uint8_t servo_ids[3] = {0x01, 0x02, 0x03};
    uint16_t servo_angles[3] = {0x0000};
    uint16_t servo_work_angle[3] = {SeperationAngle, SeperationAngle, SeperationAngle};
    bool MutexTake = false;
    uint8_t dart_num = 0x0000;

    // 直接设置成0°,限位确认
    ServoControlMulti(3, servo_ids, servo_angles, 300);

    // 获取电机偏移角度，用于目标值补偿
    float offset_angle = MotorManager.MotorList[RM_3508_GRIPPER - 1].motor_data.offset_ecd_angle;

    // 任务创建
    TaskHandle_t Load3508TaskHandle = NULL;
    xTaskCreate(LoadMotorTaskFunc, "LoadMotor", 64 * 4, NULL, osPriorityBelowNormal7, &Load3508TaskHandle);

    // 初始目标为当前位置（上电零点 + 偏移）
    GripperTarget = offset_angle;
    while (1)
    {
        xSemaphoreTake(g_xStore2LoadSemHandle, portMAX_DELAY); // 等待Store通知开始
        vTaskDelay(175);
        xStreamBufferReceive(xLoadStreamBuf, &dart_num, 1, portMAX_DELAY);
        switch (dart_num)
        {
        case 4:
        {
            GripperTarget = offset_angle;
            MutexTake = false;
            xSemaphoreGive(g_xLoad2StoreSemHandle); // 通知Store完成
            break;
        }
        case 3:
        {
            GripperTarget = FirstServoLoc + offset_angle;
            MutexTake = true;
            HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
            break;
        }
        case 2:
        {
            GripperTarget = SecondServoLoc + offset_angle;
            MutexTake = true;
            HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
            break;
        }
        case 1:
        {
            GripperTarget = ThirdServoLoc + offset_angle;
            MutexTake = true;
            HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
            break;
        }
        default:
        {
            break;
        }
        }
        while (MutexTake)
        {
            MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
            RmMotorPID_Calc(RM_3508_GRIPPER, GripperTarget);

            // 到位判断：根据 dart_num 匹配目标位置
            float target_loc = 0.0f;
            if (dart_num == 3)
                target_loc = FirstServoLoc;
            else if (dart_num == 2)
                target_loc = SecondServoLoc;
            else if (dart_num == 1)
                target_loc = ThirdServoLoc;

            if (IS_IN_DEADZONE(MotorData, target_loc, MOTOR_DEAD_ZONE))
            {
                MutexTake = false;
                LoadDart_ReturnToZero(dart_num, offset_angle);
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
    xEventGroupWaitBits(g_pxStateSetEventGroupHandeler, EVENT_ALL_READY, pdFALSE, pdTRUE, portMAX_DELAY);

    uint8_t fQueueDartNum = 0; // 目标为0最初
    vTaskDelay(1);             // 最开始放一个Tick
    // 在这里驱动换弹的3508电机运动
    while (1)
    {
        xQueueReceive(g_xLoad3508QueueHandler, &fQueueDartNum, portMAX_DELAY);
        if (fQueueDartNum >= 1 && fQueueDartNum <= 3)
        {
            ServoControlPos(fQueueDartNum, SeperationAngle, SERVO_MOVE_TIME_MS);
            vTaskDelay(SERVO_MOVE_TIME_MS + 5);
            ServoControlPos(fQueueDartNum, 0x0000, SERVO_MOVE_TIME_MS);
            vTaskDelay(SERVO_MOVE_TIME_MS + 5);
        }
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

        while (!IS_IN_DEADZONE(Motor_GetTotalAngle(RM_2006_TRIGGER), temp, MOTOR_DEAD_ZONE))
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
        vTaskDelay(1);

        // 最后进入阻塞态,等待遥控器解算唤醒 / 串口接收唤醒
        vTaskSuspend(NULL);
    }
}

#if ENABLE_RC_MASTER_SWITCH
/************************************
 * 函数名：RcMonitorTaskFunc
 * 作用：  遥控器总开关监控任务
 * 参数：  无
 *
 * 说明：
 * - 高优先级周期性任务，监控遥控器SWB开关状态
 * - SWB=1（默认）：调试模式，只允许电机调试，不执行发射和换弹流程
 * - SWB=0：正常模式，执行正常的发射和换弹流程
 * - 通过g_bRcFlowEnabled标志控制其他任务的执行
 ***********************************/
void RcMonitorTaskFunc(void *argument)
{
    // TODO:其实应该是上电前2一会进行检验的，否则就直接不管了，但是这个还是有点怪异
    uint32_t last_valid_time = 0;
    const uint32_t DATA_TIMEOUT_MS = 500; // 数据超时时间（毫秒）

    vTaskDelay(pdMS_TO_TICKS(100)); // 等待系统稳定

    while (1)
    {
        // 解析IBUS数据包
        if (IA6B_ProcessIbusPacket(BSP_UART6))
        {
            // 数据有效
            last_valid_time = HAL_GetTick();
            g_bRcDataValid = true;

            // 根据SWB状态更新流程使能标志
            // Channel[4] = 1 表示SWB位置1（默认，调试模式）
            // Channel[4] = 0 表示SWB位置2（手动切换，正常模式）
            int8_t swb_state = IA6B_ReadChannel(5); // Channel索引从1开始，所以Channel[4]用5

            if (swb_state == 0)
            {
                // SWB=0: 正常模式，允许执行流程
                g_bRcFlowEnabled = true;
            }
            else
            {
                // SWB=1: 调试模式，禁止执行流程
                g_bRcFlowEnabled = false;
                // 这个模式下将进行云台和储能电机展示，3508和2006不展示
                // TODO:要保证每一发中间可以调整目标靶子
            }
        }
        else
        {
            // 检查数据是否超时
            if (g_bRcDataValid && (HAL_GetTick() - last_valid_time > DATA_TIMEOUT_MS))
            {
                g_bRcDataValid = false;
                g_bRcFlowEnabled = false; // 数据超时，禁止执行流程（安全保护）
            }
        }

        vTaskDelay(pdMS_TO_TICKS(RC_MONITOR_TASK_PERIOD_MS));
    }
}
#endif /* ENABLE_RC_MASTER_SWITCH */
