/****************************************************************
 * 换弹结构总线舵机
 * 创建时间：2025/11/24
 *
 * 协议选择说明：
 * other_mcu_forcing = 0: 使用无MCU驱动板协议（115200波特率，有CRC校验）
 * other_mcu_forcing = 1: 使用有MCU控制板协议（9600波特率，无CRC校验）
 ****************************************************************/

#ifndef __HX_06L_H_
#define __HX_06L_H_

#include "main.h"
#include "bsp_uart.h"
#include <stdint.h>

/*****************************协议选择宏***********************************/
// 0: 无MCU驱动板协议（115200波特率，帧格式：0x55 0x55 | ID | Length | Cmd | Param | CRC）
// 1: 有MCU控制板协议（9600波特率，帧格式：0x55 0x55 | Length | Cmd | Param）
#define other_mcu_forcing  1

/*****************************指令定义***********************************/

#if (other_mcu_forcing == 1)
/*********************有MCU控制板协议指令定义**********************/

// 控制板指令枚举
typedef enum
{
    CMD_SERVO_MOVE = 0x03,           // 控制任意个舵机的转动
    CMD_ACTION_GROUP_RUN = 0x06,     // 控制动作组运行
    CMD_ACTION_GROUP_STOP = 0x07,    // 停止正在运行的动作组
    CMD_ACTION_GROUP_COMPLETE = 0x08,// 动作组自然运行结束返回
    CMD_ACTION_GROUP_SPEED = 0x0B,   // 控制动作组的速度
    CMD_GET_BATTERY_VOLTAGE = 0x0F,  // 获取控制板电池电压
    CMD_MULT_SERVO_UNLOAD = 0x14,    // 控制多个舵机马达掉电卸力
    CMD_MULT_SERVO_POS_READ = 0x15   // 读取多个舵机的角度位置值
} ControlBoardCmd;

#else
/*********************无MCU驱动板协议指令定义**********************/

// @todo 可以考虑使用共用体
typedef struct
{
    uint8_t cmd;      // 指令值
    uint8_t data_len; // 数据长度
} ServoCommandInfo;

typedef enum
{
    SERVO_MOVE_TIME_WRITE,      // 写数据（立马），发送数值0 - 1000 ，对应0 - 240°，最小分度0.24°，存在转动时间参数，范围为0-30000ms
    SERVO_MOVE_TIME_READ,       // 读数据（立马）
    SERVO_MOVE_TIME_WAIT_WRITE, // 写数据（但是要等start之后才会启动）
    SERVO_MOVE_TIME_WAIT_READ,  // 读数据（和上面一样等待start）
    SERVO_MOVE_START,           // 启动
    SERVO_MOVE_STOP,            // 停止
    SERVO_ID_WRITE,             // 设置ID
    SERVO_ID_READ,              // 读取ID
    SERVO_ANGLE_OFFSET_ADJUST,  // 设置偏差数值（不支持掉电保存）
    SERVO_ANGLE_OFFSET_WRITE,   // 掉电保存偏差数值
    SERVO_ANGLE_OFFSET_READ,    // 读取舵机设定的偏差数值
    SERVO_ANGLE_LIMIT_WRITE,    // 设置舵机转动角度的限幅
    SERVO_ANGLE_LIMIT_READ,     // 读取舵机转动角度的限制值
    SERVO_VIN_LIMIT_WRITE,      // 设置总线舵机的输入电压限幅
    SERVO_VIN_LIMIT_READ,       // 读取输入电压限幅
    SERVO_TEMP_MAX_LIMIT_WRITE, // 温度报警阈值设置，默认85
    SERVO_TEMP_MAX_LIMIT_READ,  // 读取温度报警阈值
    SERVO_TEMP_READ,            // 读取舵机内部温度
    SERVO_VIN_READ,             // 读取输入电压
    SERVO_POS_READ,             // 读取当前实际角度位置值

    /**********
     * 舵机工作模式：
     * 1.舵机模式（位置控制模式/电机控制模式）
     * 2.转动模式（固定占空比模式/固定转速模式）
     *********/

    SERVO_OR_MOTOR_MODE_WRITE, // 舵机工作模式设置
    /**********
     * 参数设置
     * 参数1：舵机模式（0 -> 位置控制模式， 1 -> 电机控制模式）
     * 参数2：转动模式（0 -> 固定占空比模式， 1 -> 固定转速模式）
     * 参数3：转动速度的低8位
     * 参数4：转动速度的高8位
     *
     * @note 在固定占空比模式下范围是：-1000~1000，
     *       而在固定转速模式下范围是：-50~50。取值为负值表示舵机反转，正值表示舵机
     *       正转。写入的模式和速度不支持掉电保存。
     *       这里的数据格式以补码形式表示
     *       转动速度需要由符号数强转为无符号数
     *********/

    SERVO_OR_MOTOR_MODE_READ,   // 读取舵机模式相关参数
    SERVO_LOAD_OR_UNLOAD_WRITE, // 舵机内部电机是否上电，0代表掉电
    SERVO_LOAD_OR_UNLOAD_READ,  // 读取舵机内部电机的状态
    SERVO_LED_CTRL_WRITE,       // LED灯的亮灭状态，默认0 -> LED 常亮
    SERVO_LED_CTRL_READ,        // 读取LED的状态
    SERVO_LED_ERROR_WRITE,      // 写LED报警指令
    SERVO_LED_ERROR_READ,       // 读取LED故障报警值
    SERVO_CMD_COUNT             // 读取舵机的转动距离（4096/圈）
} ServoCommandName;

// 指令映射表（按顺序对应上面的 enum）
static const ServoCommandInfo servo_commands[SERVO_CMD_COUNT] = {
    {1, 7},  // SERVO_MOVE_TIME_WRITE
    {2, 3},  // SERVO_MOVE_TIME_READ
    {7, 7},  // SERVO_MOVE_TIME_WAIT_WRITE
    {8, 3},  // SERVO_MOVE_TIME_WAIT_READ
    {11, 3}, // SERVO_MOVE_START
    {12, 3}, // SERVO_MOVE_STOP
    {13, 4}, // SERVO_ID_WRITE
    {14, 3}, // SERVO_ID_READ
    {17, 4}, // SERVO_ANGLE_OFFSET_ADJUST
    {18, 3}, // SERVO_ANGLE_OFFSET_WRITE
    {19, 3}, // SERVO_ANGLE_OFFSET_READ
    {20, 7}, // SERVO_ANGLE_LIMIT_WRITE
    {21, 3}, // SERVO_ANGLE_LIMIT_READ
    {22, 7}, // SERVO_VIN_LIMIT_WRITE
    {23, 3}, // SERVO_VIN_LIMIT_READ
    {24, 4}, // SERVO_TEMP_MAX_LIMIT_WRITE
    {25, 3}, // SERVO_TEMP_MAX_LIMIT_READ
    {26, 3}, // SERVO_TEMP_READ
    {27, 3}, // SERVO_VIN_READ
    {28, 3}, // SERVO_POS_READ
    {29, 7}, // SERVO_OR_MOTOR_MODE_WRITE
    {30, 3}, // SERVO_OR_MOTOR_MODE_READ
    {31, 4}, // SERVO_LOAD_OR_UNLOAD_WRITE
    {32, 3}, // SERVO_LOAD_OR_UNLOAD_READ
    {33, 4}, // SERVO_LED_CTRL_WRITE
    {34, 3}, // SERVO_LED_CTRL_READ
    {35, 4}, // SERVO_LED_ERROR_WRITE
    {36, 3}  // SERVO_LED_ERROR_READ
};

// 工作模式
typedef enum
{
    Servo_PosCtrl = 0x00, // 位置控制模式
    Servo_MotorCtrl,      // 电机控制模式
    Rotate_Duty,          // 固定占空比模式
    Rotate_Speed          // 固定速度模式
} WorkMode;

#endif /* other_mcu_forcing */

/*****************************函数声明***********************************/

/// @brief 换弹舵机初始化
/// @param  无
void ServoInit(void);

/// @brief 总线舵机控制函数
/// @param ID 总线舵机ID
/// @param Angle 总线舵机转过的角度（0-1000对应0-240°）
/// @param Time 转动过程时间（0-30000ms）
/// @retval 无
/// @note 根据时间匀速转动到对应设置的角度
void ServoControlPos(uint8_t ID, uint16_t Angle, uint16_t Time);

#if (other_mcu_forcing == 1)
/***********************有MCU控制板协议专用函数***********************/

/// @brief 控制多个舵机同时转动（控制板协议）
/// @param servo_num 舵机个数
/// @param servo_ids 舵机ID数组
/// @param angles 角度数组（0-1000）
/// @param time 转动时间（ms）
void ServoControlMulti(uint8_t servo_num, uint8_t *servo_ids, uint16_t *angles, uint16_t time);

/// @brief 运行动作组（控制板协议）
/// @param group_num 动作组编号
/// @param run_times 运行次数（0表示无限次）
void ServoRunActionGroup(uint8_t group_num, uint16_t run_times);

/// @brief 停止动作组（控制板协议）
void ServoStopActionGroup(void);

/// @brief 设置动作组速度（控制板协议）
/// @param group_num 动作组编号（0xFF表示所有动作组）
/// @param speed_percent 速度百分比（100表示原速，200表示2倍速）
void ServoSetActionGroupSpeed(uint8_t group_num, uint16_t speed_percent);

/// @brief 控制多个舵机卸力（控制板协议）
/// @param servo_num 舵机个数
/// @param servo_ids 舵机ID数组
void ServoUnloadMulti(uint8_t servo_num, uint8_t *servo_ids);

/// @brief 获取电池电压（控制板协议）
/// @return 无（需要通过接收回调获取结果）
void ServoGetBatteryVoltage(void);

#endif /* other_mcu_forcing == 1 */

#endif /* __HX_06L_H_ */
