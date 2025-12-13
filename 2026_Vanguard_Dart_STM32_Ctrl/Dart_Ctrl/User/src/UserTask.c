#include "UserTask.h"

// 任务：
// 换弹任务
// 扳机任务 (包含调节扳机松紧任务)
// 通信任务 (can, uart)
// 存储拉簧动能任务
// yaw调整任务

/*---------------------------------------------------------------------------------------*/
// 模块驱动初始化
void Module_Init(void)
{
    BSP_POWER_DeInit(); // 失能无绿灯，亮红灯
    DWT_Init(180);
    MotorInit();
    CanFliterCfg();
    BSP_UART_Init();
    // HAL_TIMEx_PWM_Start();

    // 云台遥控器初始化
}

// MUTEX（互斥量）：调控云台方式只有一种，不允许上位机和遥控器同时调控

// 软件定时器：定时向上位机回复确认当前状态正常，否则加一些安全措施

// QUEUE（队列）：融合到云台调节当中，保证云台调节正常<队列集>

// TASK_Notifaation（任务通知）：任务通知确保发射正常（自检位置）

/***********************************************TASK******************************************************/
// 云台调节任务(yaw调整任务)
osThreadId_t GimbalTaskHandle;
const osThreadAttr_t GimbalTask_attributes = {
    .name = "GimbalTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
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
    GimbalTaskHandle = osThreadNew(GimbalTaskFunc, NULL, &GimbalTask_attributes);
    StoreEnergyTaskHandle = osThreadNew(StoreEnergyTaskFunc, NULL, &StoreEnergyTask_attributes);
    LoadTaskHandle = osThreadNew(LoadTaskFunc, NULL, &LoadTask_attributes);
    ShootTaskHandle = osThreadNew(ShootTaskFunc, NULL, &ShootTask_attributes);
    UartModuleTaskHandle = osThreadNew(UartModuleTaskFunc, NULL, &UartModuleTask_attributes);
}

void GimbalTaskFunc(void *argument)
{
}

void StoreEnergyTaskFunc(void *argument)
{
    // 3510电机运动从顶部到底部

    // 3510电机底部等待换弹,
}

void LoadTaskFunc(void *argument)
{
    // 回读角度,确认换弹状态正常

    // 3508电机调节

    // 总线舵机运行,进行换弹指令
}

void ShootTaskFunc(void *argument)
{
    // 1.这里放一个任务通知量，等待解析上位机发来的射程调整数据之后直接激活扳机位置调节

    // 2.这里进行正常的发射函数
    // __HAL_TIM_SET_COMPARE();
}

void UartModuleTaskFunc(void *argument)
{
}
