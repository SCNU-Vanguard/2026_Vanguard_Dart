#include "UserTask.h"
// 19271（上限）<长度>
// 第三个（距离7547）
// 第二个（距离13200）<与第三个相距5653>
// 第一个（距离18555）<与第二个相距5549>

// 任务：
// 换弹任务
// 扳机任务 (包含调节扳机松紧任务)
// 通信任务 (can, uart)
// 存储拉簧动能任务
// yaw调整任务

/************************全局或静态作用域*********************/
static DartPacket_t g_xDartData;
static ServoPacket_t g_xServoData;
static uint8_t DartNum = 3;
extern MotorManager_t MotorManager;
int16_t MotorData = 0x00;

// 云台互斥量
static StaticSemaphore_t g_xGimbalMutexBuffer;
SemaphoreHandle_t g_xGimbalMutexHandle;

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
    g_xUartSemaphoreHandle = xSemaphoreCreateCountingStatic(3, 0, &g_xUartSemaphore); // 初始值为0，上限为2，接收时候释放

    // 软件定时器：定时向上位机回复确认当前状态正常，否则加一些安全措施，Tick大概30s触发一次

    // QUEUE（队列）：融合到云台调节当中，保证云台调节正常<队列集>

    // TASK_Notifaation（任务通知）：任务通知确保发射正常（自检位置），也可能是互斥量或者信号量

    // MUTEX（互斥量）：调控云台方式只有一种，不允许上位机和遥控器同时调控
    g_xGimbalMutexHandle = xSemaphoreCreateMutexStatic(&g_xGimbalMutexBuffer);
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
    .stack_size = 128 * 4,
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

// 任务初始化
void TaskInitFunc(void)
{
    // 任务初始化
    // GimbalTaskHandle = osThreadNew(GimbalTaskFunc, NULL, &GimbalTask_attributes);
    // StoreEnergyTaskHandle = osThreadNew(StoreEnergyTaskFunc, NULL, &StoreEnergyTask_attributes);
    LoadTaskHandle = osThreadNew(LoadTaskFunc, NULL, &LoadTask_attributes);
    // ShootTaskHandle = osThreadNew(ShootTaskFunc, NULL, &ShootTask_attributes);
    // UartModuleTaskHandle = osThreadNew(UartModuleTaskFunc, NULL, &UartModuleTask_attributes);
}

void GimbalTaskFunc(void *argument)
{
    // 根据上位机和遥控器的命令对4310进行旋转
}

void StoreEnergyTaskFunc(void *argument)
{
    // 3510电机运动从顶部到底部

    // 3510电机底部等待换弹
}

void LoadTaskFunc(void *argument)
{
    // 需要等待一个换弹指令,这个全由轮询决定
    float target = FirstServoLoc;
    uint8_t servo_ids[3] = {0x01, 0x02, 0x03};
    uint16_t servo_angles[3] = {0x0000};
    uint16_t servo_work_angle[3] = {SeperationAngle, SeperationAngle, SeperationAngle};

    // 回读角度,确认换弹之前状态正常
    // 直接设置成0°,限位确认
    ServoControlMulti(3, servo_ids, servo_angles, 500);     // 当前角度是0°,在设置后(MCU驱动板上电无法读取)
    vTaskDelay(2500);                                       // 2.5s之后会进行一次转动,之后复位
    ServoControlMulti(3, servo_ids, servo_work_angle, 910); // 转75°分离,最大时长910ms
    vTaskDelay(1000);
    ServoControlMulti(3, servo_ids, servo_angles, 500);
    uint8_t MotorDecFlag = 0x00;

    while (DartNum)
    {
        // 3508电机调节
        // 测试PID是否正常可以使用
        RmMotorRemoveBias(RM_3508_GRIPPER, target);
        RmMotorPID_Calc(RM_3508_GRIPPER, target);
        // 总线舵机运行,进行换弹指令

        // if (Motor_GetTotalAngle(RM_3508_GRIPPER) == FirstServoLoc)
        MotorData = Motor_GetTotalAngle(RM_3508_GRIPPER);
        if ((MotorData == FirstServoLoc) || (MotorDecFlag == 1)) // 首次上电电机的零点为offest_ecd,负向转动,距离为-7547
        {
            ServoControlPos(0x01, SeperationAngle, 910); // 转90°分离,最大时长910ms
            DartNum--;
						MotorDecFlag = 1;
            target = SecondServoLoc;
            vTaskDelay(200);
            ServoControlPos(0x01, 0x0000, 910);

            // 这个地方很难做到精准，需要加一个模糊
            if ((MotorData == SecondServoLoc) || (MotorDecFlag == 2)) // 负向转动,距离为-5653
            {
                MotorDecFlag = 2;
                ServoControlPos(0x02, SeperationAngle, 910); // 转90°分离,最大时长910ms
                DartNum--;
                target = ThirdServoLoc;
                vTaskDelay(200);
                ServoControlPos(0x02, 0x0000, 910);
                if ((MotorData == ThirdServoLoc) || (MotorDecFlag == 3)) // 负向转动,距离为-5549
                {
                    MotorDecFlag = 3;
                    ServoControlPos(0x03, SeperationAngle, 910); // 转90°分离,最大时长910ms
                    DartNum--;
                    target = -(MotorManager.MotorList->motor_data.solved_data[3] - MotorManager.MotorList->motor_data.offset_ecd);
                    vTaskDelay(200);
                    ServoControlPos(0x03, 0x0000, 910);
                }
            }
        }
    }
}

void ShootTaskFunc(void *argument)
{
    // 1.这里放一个队列，等待解析上位机发来的射程调整数据之后直接激活扳机位置调节

    // 2.这里进行正常的发射函数,TIM2_CH1-PD12->A板H接口
    // 根据同步带电机反馈数据进行发射
    // __HAL_TIM_SET_COMPARE();
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
                // 6020电机调节射程
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
