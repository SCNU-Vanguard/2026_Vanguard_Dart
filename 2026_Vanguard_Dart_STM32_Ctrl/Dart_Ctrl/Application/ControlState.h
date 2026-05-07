/**********************************************************
 * 文件名：ControlState.h
 * 用途：遥控器控制状态管理模块
 * 创建时间：2026-01-23
 *
 * 说明：
 * - 集中管理所有电机目标值和控制模式
 * - IbusTask解析遥控器通道，更新ControlInput
 * - ControlTask根据ControlInput计算目标并下发电机
 * - 支持正常单发模式和手动调试模式的互斥
 *
 * 通道映射（按RawChannel索引）：
 * - RawChannel[0] -> 右摇杆左右（回中）-> YAW
 * - RawChannel[1] -> 右摇杆上下（回中）-> 储能3519
 * - RawChannel[2] -> 左摇杆上下（不回中，默认底部1000）-> 扳机2006
 * - RawChannel[3] -> 左摇杆左右（回中）-> 3508位置/舵机触发
 * - RawChannel[4] -> SWB（两段）-> 模式切换
 * - RawChannel[5] -> SWC（三段）-> 子模式
 *********************************************************/
#ifndef __CONTROL_STATE_H_
#define __CONTROL_STATE_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <stdbool.h>
#include "semphr.h"

/*============================== 模式定义 ==============================*/

/**
 * @brief 控制模式枚举
 */
typedef enum
{
    CTRL_MODE_NORMAL_SINGLE = 0, // 正常单发模式：任务流程接管，忽略摇杆
    CTRL_MODE_MANUAL_DEBUG,      // 手动调试模式：遥控器直接控制电机
    CTRL_MODE_DEBUG_TIMER,       // 调试计时模式：单发结束后90s调试窗口
} CtrlMode_e;

/*============================== 参数配置 ==============================*/

// 任务周期
#define IBUS_TASK_PERIOD_MS 5  // IbusTask周期（毫秒）
#define CTRL_TASK_PERIOD_MS 10 // ControlTask周期（毫秒）
#define DEBUG_WINDOW_MS 90000  // 调试窗口时间（毫秒）
// 调试开关行为：1=SWB=0 时可随时手动调试；0=仅调试窗口内允许
#define DEBUG_MODE_ALWAYS_ALLOW 1

// 摇杆死区与归一化
#define STICK_CENTER 1500    // 摇杆中心值
#define STICK_RANGE 500      // 摇杆范围（±500）
#define STICK_DEADZONE 0.15f // 回中摇杆死区（±15%）

// 非回中摇杆三段判定阈值
#define NON_CENTER_HIGH 1700 // 高位阈值（>1700 -> +1）
#define NON_CENTER_LOW 1300  // 低位阈值（<1300 -> -1）

// 电机控制权重（单位：目标增量/秒）
#define WEIGHT_YAW 20.0f     // YAW旋转速度（调试窗口下手动调节）
#define WEIGHT_TRIGGER 50.0f // 扳机移动速度（rad）
#define WEIGHT_ENERGY 100.0f // 储能电机移动速度（rad）- 调试窗口用，10倍速
#define WEIGHT_LOAD3508 1.0f // 3508位置调整速度（rad）

// 电机目标限幅
#define YAW_MIN -160.0f // YAW最小角度
#define YAW_MAX 160.0f  // YAW最大角度
#define TRIGGER_MIN -10000.0f
#define TRIGGER_MAX 0.0f
#define ENERGY_LEFT_MIN (-910.0f)
#define ENERGY_LEFT_MAX (0.0f)
#define ENERGY_RIGHT_MIN (0.0f)
#define ENERGY_RIGHT_MAX (910.0f)

/*============================== 数据结构 ==============================*/

/**
 * @brief 遥控器输入结构体（IbusTask写入）
 * @note 只包含方向/比例，不直接包含电机目标
 */
typedef struct
{
    // 回中摇杆归一化值（-1.0 ~ +1.0，死区内为0）
    float yaw_norm;      // YAW方向（右摇杆左右）
    float load3508_norm; // 3508方向（左摇杆左右，回中时使用）
    float energy_norm;   // 储能方向（右摇杆上下，回中摇杆）

    // 非回中摇杆方向（-1/0/+1）
    int8_t trigger_dir;  // 扳机方向（左摇杆上下）
    int8_t load3508_dir; // 3508/舵机切换（左摇杆左右非回中时：-1=左，+1=右，0=回中）

    // 开关状态
    int8_t swb; // SWB状态（1=默认，0=手动）
    int8_t swc; // SWC状态（1/0/-1）

    // 模式判定结果
    CtrlMode_e mode; // 当前控制模式

    // 数据有效标志
    bool data_valid; // 数据是否有效
} ControlInput_t;

/**
 * @brief 控制状态结构体（ControlTask写入，其他任务只读）
 * @note 包含所有电机的目标值
 */
typedef struct
{
    // 电机目标值
    float yaw_target;          // YAW目标角度
    float trigger_target;      // 扳机目标位置
    float energy_left_target;  // 左储能电机目标
    float energy_right_target; // 右储能电机目标
    float load3508_target;     // 3508目标位置

    // 控制模式
    CtrlMode_e mode;      // 当前模式
    bool manual_override; // 手动覆盖标志

    // 调试窗口计时
    uint32_t debug_timer_start; // 调试窗口开始时间
    bool debug_window_active;   // 调试窗口是否激活

    // 任务流程状态
    bool single_shot_done; // 单发流程是否完成
} ControlState_t;

/*============================== 全局变量声明 ==============================*/

extern ControlInput_t g_ControlInput; // 遥控器输入（IbusTask更新）
extern ControlState_t g_ControlState; // 控制状态（ControlTask更新）

// 电机控制权互斥信号量（调试任务和自动任务互斥使用）
extern SemaphoreHandle_t g_xMotorCtrlSemHandle;
extern SemaphoreHandle_t g_xDebugFinishedSemHandle;

/*============================== 任务句柄声明 ==============================*/

extern osThreadId_t IbusTaskHandle;
extern osThreadId_t ControlTaskHandle;

/*============================== API函数声明 ==============================*/

/**
 * @brief 初始化控制状态模块
 * @note 在RTOS启动前调用
 */
void ControlState_Init(void);

/**
 * @brief 创建IbusTask和ControlTask
 * @note 在TaskInitFunc()中调用
 */
void ControlState_CreateTasks(void);

/**
 * @brief 获取当前控制模式
 * @return 当前模式枚举值
 */
CtrlMode_e ControlState_GetMode(void);

/**
 * @brief 检查是否处于手动覆盖模式
 * @return true-手动模式，false-正常模式
 */
bool ControlState_IsManualOverride(void);

/**
 * @brief 开启调试窗口（单发完成后调用）
 * @param duration_ms 调试窗口持续时间（毫秒）
 */
void ControlState_StartDebugWindow(uint32_t duration_ms);

/**
 * @brief 设置YAW目标（供StateSetTask使用）
 * @param target 目标角度
 */
void ControlState_SetYawTarget(float target);

/**
 * @brief 设置扳机目标（供StateSetTask使用）
 * @param target 目标位置
 */
void ControlState_SetTriggerTarget(float target);

/**
 * @brief 设置储能电机目标（供StoreEnergyTask使用）
 * @param left_target 左电机目标
 * @param right_target 右电机目标
 */
void ControlState_SetEnergyTarget(float left_target, float right_target);

/**
 * @brief 设置3508目标（供LoadTask使用）
 * @param target 目标位置
 */
void ControlState_SetLoad3508Target(float target);

/**
 * @brief 标记单发流程完成
 */
void ControlState_MarkSingleShotDone(void);

/*============================== 内部辅助函数 ==============================*/

/**
 * @brief 将原始摇杆值归一化（回中摇杆）
 * @param raw 原始值（1000-2000）
 * @return 归一化值（-1.0 ~ +1.0，死区内为0）
 */
static inline float ControlState_NormalizeStick(int16_t raw)
{
    float norm = (float)(raw - STICK_CENTER) / (float)STICK_RANGE;

    // 限幅
    if (norm > 1.0f)
        norm = 1.0f;
    else if (norm < -1.0f)
        norm = -1.0f;

    // 死区处理
    if (norm > -STICK_DEADZONE && norm < STICK_DEADZONE)
    {
        norm = 0.0f;
    }

    return norm;
}

/**
 * @brief 将原始摇杆值转换为激活状态（非回中摇杆）
 * @param raw 原始值（1000-2000）
 * @return 激活状态（0=未激活/底部，1=激活/向上推）
 * @note 非回中摇杆默认静止在底部(1000)，向上推时值增大
 */
static inline int8_t ControlState_GetDirection(int16_t raw)
{
    // 非回中摇杆：默认在底部(1000)，只有0和1两个状态
    // 超过阈值认为是激活状态
    if (raw > NON_CENTER_HIGH)
        return 1; // 激活（摇杆向上推）
    else
        return 0; // 未激活（摇杆在底部或中间区域）
}

/**
 * @brief 限幅函数
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的值
 */
static inline float ControlState_Clamp(float value, float min, float max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}

#endif /* __CONTROL_STATE_H_ */
