/************************************************************************************************************************
 * 文件：UserTask.c
 * 用途：用户任务定义，所有的任务都在此
 * 创建者：邓金水
 * 创建日期：忘了
 * 目标：15s内完成4次发射过程
 * 流程：视觉/遥控器(可调射程)纠正方向->第一发飞镖->滑台下滑(同时扳机要放下来)->到达地点(扳机绷紧)->根据所需射程调节电机->释放扳机
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
static uint8_t DartNum = 4;
extern MotorManager_t MotorManager;
int16_t MotorData = 0x00;
float target = FirstServoLoc; // 位置不动

// 云台互斥量
static StaticSemaphore_t g_xGimbalMutexBuffer;
SemaphoreHandle_t g_xGimbalMutexHandle;
static uint8_t g_ucReloadQueueStorage[4 * sizeof(float)];
static StaticQueue_t g_xReloadQueueStruct;
static QueueHandle_t g_xLoad3508QueueHandler;

// 换弹结构电机队列

// Uart信号量
static StaticSemaphore_t g_xUartSemaphore;
SemaphoreHandle_t g_xUartSemaphoreHandle;

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
    // g_xUartSemaphoreHandle = xSemaphoreCreateCountingStatic(3, 0, &g_xUartSemaphore); // 初始值为0，上限为2，接收时候释放

    // 软件定时器：定时向上位机回复确认当前状态正常，否则加一些安全措施，Tick大概30s触发一次

    // QUEUE（队列）：1.融合到云台调节当中，保证云台调节正常<队列集>; 2.融入到换弹结构当中,确保换弹结构正常
    g_xLoad3508QueueHandler = xQueueCreateStatic(4, sizeof(uint8_t), g_ucReloadQueueStorage, &g_xReloadQueueStruct); // 这个队列用于换弹结构当中

    // TASK_Notifaation（任务通知）：任务通知确保发射正常（自检位置），也可能是互斥量或者信号量

    // MUTEX（互斥量）：调控云台方式只有一种，不允许上位机和遥控器同时调控
    // g_xGimbalMutexHandle = xSemaphoreCreateMutexStatic(&g_xGimbalMutexBuffer);
}

/***********************************************TASK******************************************************/
// 云台调节任务(yaw调整任务),默认优先度较低,继承中断函数优先级
osThreadId_t GimbalTaskHandle;
const osThreadAttr_t GimbalTask_attributes = {
    .name = "GimbalTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal7,
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
    .priority = (osPriority_t)osPriorityNormal1,
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
    // RTOS_ModuleInit();

    // 任务初始化
    GimbalTaskHandle = osThreadNew(GimbalTaskFunc, NULL, &GimbalTask_attributes);
    // StoreEnergyTaskHandle = osThreadNew(StoreEnergyTaskFunc, NULL, &StoreEnergyTask_attributes);
    // LoadTaskHandle = osThreadNew(LoadTaskFunc, NULL, &LoadTask_attributes);
    // ShootTaskHandle = osThreadNew(ShootTaskFunc, NULL, &ShootTask_attributes);
    // UartModuleTaskHandle = osThreadNew(UartModuleTaskFunc, NULL, &UartModuleTask_attributes);
}

// 87 FF 7F F0 0B 33 37 FF -> 位置 10 rad, 速度 0 rad/s, kp = 1.4591, kd = 1.0f, 转矩0.0f
// 7F FF 7F F0 0B 33 37 FF -> 位置变为 0 rad
void GimbalTaskFunc(void *argument)
{
    // 根据上位机和遥控器的命令对4310进行旋转
    DM_Motor_Enable(&MotorManager.MotorList[DM_4310_YAW - 1]);
    DmMotorSendCfg(DM_4310_YAW, 10.0f, 0.0f, DM_MIT);
    vTaskDelay(5000);
    DmMotorSendCfg(DM_4310_YAW, 0.0f, 0.0f, DM_MIT);
    DM_Motor_Disable(&MotorManager.MotorList[DM_4310_YAW - 1]);
    while (1)
    {
        /* code */
    }
}

// 00 00 20 41 00 00 A0 40 -> 位置 10 rad, 速度 5 rad/s
void StoreEnergyTaskFunc(void *argument)
{
    DM_Motor_Enable(&MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1]);
    DmMotorSendCfg(DM_3519_STRENTH_LEFT, 10.0f, 5.0f, DM_LOCATION_SPEED);
    // DM_Motor_Enable(DM_3519_STRENTH_RIGHT);
    // J3519电机运动从顶部到底部

    // vTaskDelay(2000);
    // DM_Motor_Disable(&MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1]);
    while (1)
    {
        DmMotorSendCfg(DM_3519_STRENTH_LEFT, 10.0f, 5.0f, DM_LOCATION_SPEED);
        // DM_Motor_Enable(&MotorManager.MotorList[DM_3519_STRENTH_LEFT - 1]);
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
    // 需要等待一个换弹指令,这个全由轮询决定
    // float target = FirstServoLoc; // 初始目标：最远位置（第一个飞镖）
    uint8_t servo_ids[3] = {0x01, 0x02, 0x03};
    uint16_t servo_angles[3] = {0x0000};
    uint16_t servo_work_angle[3] = {SeperationAngle, SeperationAngle, SeperationAngle};

    // 直接设置成0°,限位确认
    ServoControlMulti(3, servo_ids, servo_angles, 300); // 当前角度是0°,在设置后(MCU驱动板上电无法读取)
    float offset_angle = MotorManager.MotorList[RM_3508_GRIPPER - 1].motor_data.offset_ecd_angle;

    // 任务创建
    TaskHandle_t Load3508TaskHandle = NULL;
    xTaskCreate(LoadMotorTaskFunc, "LoadMotor", 64 * 4, NULL, osPriorityAboveNormal, &Load3508TaskHandle); // 换弹3508电机任务
    target = RmMotorRemoveBias(RM_3508_GRIPPER, target, false);
    while (1)
    {
        if (DartNum == 1)
        {
            DartNum = 4;

            // MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
            // target = RmMotorRemoveBias(RM_3508_GRIPPER, -MotorData, true);
        }
        while (DartNum)
        {
            // 在当前任务只负责对3508电机目标值进行调节，而不进行调控PID，当信号量发生将直接更改电机Target，当最后到达目标数值时候将直接返回

            RmMotorPID_Calc(RM_3508_GRIPPER, target);

// 定义死区范围 ±10, todo:是否增大死区
#define MOTOR_DEAD_ZONE 1

            // 使用死区判断：MotorData 在 [target-1, target+1] 范围内视为到达
            if ((MotorData >= FirstServoLoc - MOTOR_DEAD_ZONE) &&
                (MotorData <= FirstServoLoc + MOTOR_DEAD_ZONE) &&
                (DartNum == 4)) // 首次上电电机的零点为offest_ecd,负向转动,距离为-7547
            {
                /* code */
                xQueueSend(g_xLoad3508QueueHandler, &DartNum, 0);
                vTaskDelay(310);

                // 等待电机到达过渡位置
                MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
                // 回到上电零点位置：当前累计角度的负值
                while (!((MotorData >= (0 - MOTOR_DEAD_ZONE)) &&
                         (MotorData <= (0 + MOTOR_DEAD_ZONE))))
                {
                    target = RmMotorRemoveBias(RM_3508_GRIPPER, -MotorData, true);
                    RmMotorPID_Calc(RM_3508_GRIPPER, target);
                    MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER); // 更新位置数据
                }
                target = SecondServoLoc;
                target = RmMotorRemoveBias(RM_3508_GRIPPER, target, true);
                DartNum--;
            }

            if ((MotorData >= SecondServoLoc - MOTOR_DEAD_ZONE) &&
                (MotorData <= SecondServoLoc + MOTOR_DEAD_ZONE) &&
                (DartNum == 3)) // 负向转动,距离为-5653
            {
                xQueueSend(g_xLoad3508QueueHandler, &DartNum, 0);
                vTaskDelay(310);
                // 等待电机到达过渡位置
                MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
                // 回到上电零点位置：当前累计角度的负值
                while (!((MotorData >= (0 - MOTOR_DEAD_ZONE)) &&
                         (MotorData <= (0 + MOTOR_DEAD_ZONE))))
                {
                    target = RmMotorRemoveBias(RM_3508_GRIPPER, -MotorData, true);
                    RmMotorPID_Calc(RM_3508_GRIPPER, target);
                    MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER); // 更新位置数据
                }
                target = ThirdServoLoc;
                target = RmMotorRemoveBias(RM_3508_GRIPPER, target, true);
                DartNum--;
            }

            if ((MotorData >= ThirdServoLoc - MOTOR_DEAD_ZONE) &&
                (MotorData <= ThirdServoLoc + MOTOR_DEAD_ZONE) &&
                (DartNum == 2)) // 负向转动,距离为-5549
            {
                xQueueSend(g_xLoad3508QueueHandler, &DartNum, 0);
                vTaskDelay(310);
                // 等待电机到达过渡位置
                MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
                // 回到上电零点位置：当前累计角度的负值
                while (!((MotorData >= (0 - MOTOR_DEAD_ZONE)) &&
                         (MotorData <= (0 + MOTOR_DEAD_ZONE))))
                {
                    target = RmMotorRemoveBias(RM_3508_GRIPPER, -MotorData, true);
                    RmMotorPID_Calc(RM_3508_GRIPPER, target);
                    MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER); // 更新位置数据
                }
                target = RmMotorRemoveBias(RM_3508_GRIPPER, 0.0f, true);
                DartNum--;
            }
            MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
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
            ServoControlPos(0x03, SeperationAngle, 310); // 转90°分离,最大时长300ms
            vTaskDelay(315);
            ServoControlPos(0x03, 0x0000, 310);
            vTaskDelay(315);
        }
        else if (fQueueDartNum == 2)
        {
            ServoControlPos(0x02, SeperationAngle, 310); // 转90°分离,最大时长300ms
            vTaskDelay(315);
            ServoControlPos(0x02, 0x0000, 310);
            vTaskDelay(315);
        }
        else if (fQueueDartNum == 1)
        {
            ServoControlPos(0x01, SeperationAngle, 310); // 转90°分离,最大时长300ms
            vTaskDelay(315);
            ServoControlPos(0x01, 0x0000, 310);
            vTaskDelay(315);
        }
    }
}

/***********************************
 * 函数名: ShootTaskFunc
 * 作用:   发射任务（舵机释放以及电机目标设定）
 * 参数:   无
 * todo:   增加code注释,为之后的更新作准备
 * 备注:   MG996R舵机直接转动的范围是( 500 ~ 2500 )
 **********************************/
void ShootTaskFunc(void *argument)
{
    float trigger_target = 51840.0f;                            // 先转10圈
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MG996R_store); // 这个地方设置为未释放状态

    while (1)
    {
        // 1.这里放一个队列，等待解析上位机发来的射程调整数据之后直接激活扳机位置调节，随时准备调节射程
        /* code */
        // 这放一个意淫数据，接收信号量传输
        trigger_target = RmMotorRemoveBias(RM_2006_TRIGGER, trigger_target, false); // 位置环,这里先转10圈看看,主要不知道限幅
        RmMotorPID_Calc(RM_2006_TRIGGER, trigger_target);
        // RmMotorSendCfg(RM_2006_TRIGGER, 5000);

        // 2.这里进行正常的发射函数,TIM2_CH1-PD12->A板H接口
        // 根据同步带电机反馈数据进行发射
        // 接收任务通知 / 互斥量
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MG996R_store); // 这个地方设置为未释放状态

        // delay一会再次释放
        vTaskDelay(100);
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, MG996R_shoot);
    }
}

void UartModuleTaskFunc(void *argument)
{
    // 等待TaskNotification
    uint32_t xTaskNotificationNum = 0x00;
    while (1)
    {
        xTaskNotifyWait(0xffffffff, 0xffffffff, &xTaskNotificationNum, portMAX_DELAY); // 32位全部清空,直接死等
        switch (xTaskNotificationNum)
        {
        case 0x00:
            break;

            // 有MCU控制协议
        case 0x01:
            xTaskNotificationNum = 0x00;
            // 处理数据
            UartModule_GetServoPacket(BSP_UART6, &g_xServoData);
            // 给队列处理数据
            break;

            // 无MCU控制协议
        case 0x02:
            xTaskNotificationNum = 0x00;
            // 处理数据
            UartModule_GetServoPacket(BSP_UART6, &g_xServoData);
            // 给队列处理数据
            break;

            // DART通信协议
        case 0x03:
            xTaskNotificationNum = 0x00;
            // 处理数据
            // 对数据进行解析，看看是什么要求？是云台调整还是扳机调整
            UartModule_GetDartPacket(BSP_UART3, &g_xDartData);
            if (g_xDartData.is_valid)
            {
                // 4310电机调节
                if (g_xDartData.data[0] == 0x43)
                {
                    // 这里给一个队列
                }
                // 4310电机调节射程
                if (g_xDartData.data[0] == 0x60)
                {
                    // 这里给一个队列
                }
            }
            break;

        default:
            Error_Handler();
        }
    }
}
