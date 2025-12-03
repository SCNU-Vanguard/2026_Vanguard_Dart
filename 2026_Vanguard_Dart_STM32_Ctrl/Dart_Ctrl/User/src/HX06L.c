// 舵机初始化的ID和信息都是只初始化一次就行，后面可以不用初始化，直接使用
// 可能久了会有偏差，建议使用时间长了之后有偏差回来更新BIAS

#include "HX06L.h"
#include "bsp_dwt.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

/// @brief CRC校验生成（调用bsp_uart中的统一CRC函数）
/// @param prtSendData 发送数据的指针（包含包头0x55 0x55）
/// @param DataLength 数据长度（不包含包头和CRC）
/// @return CRC校验码数据
static inline uint8_t CRC_GNERATOR(uint8_t *prtSendData, uint8_t DataLength)
{
    // 调用bsp_uart中的统一CRC函数
    // prtSendData[2]开始是ID，DataLength是从ID到参数的长度
    return UART_Calculate_CRC(&prtSendData[2], DataLength);
}

// 辅助函数：根据指令名获取数据长度
static inline uint8_t get_servo_data_length(ServoCommandName cmd_name)
{
    if (cmd_name < SERVO_CMD_COUNT)
    {
        return servo_commands[cmd_name].data_len;
    }
    return 0; // 无效指令
}

// 辅助函数：根据指令名获取指令值
static inline uint8_t get_servo_command_value(ServoCommandName cmd_name)
{
    if (cmd_name < SERVO_CMD_COUNT)
    {
        return servo_commands[cmd_name].cmd;
    }
    return 0xFF; // 无效
}

// uint8_t cmd_val = get_servo_command_value(SERVO_MOVE_TIME_WRITE); // 返回 1
// uint8_t len = get_servo_data_length(SERVO_MOVE_TIME_WRITE);       // 返回 7

/*************************************
 * HX06L总线舵机通信数据帧格式：
 * 帧头 | ID号 | 数据长度 | 指令 | 参数 | 校验 | 字段
 *
 * 各个字段作用：
 * 帧头：起始位，表示数据传输开始
 * ID号：各个舵机各自的ID号，ID号不可以相同
 * 数据长度：待发送的数据长度，数值为Length = DataLength + 3，这里预设好了数据长度(根据指令)
 * 指令：位置 / 速度
 * 校验码：用于验证数据的完整性
 *
 * 示例：
 *      0x55 0x55 | ID | Length | Cmd | Param(可以n多个) | CheckSum(校验和)
 * 注意：
 *      每个舵机的ID不同，ID号范围为0-253，转换为16进制0x00-0xFD
 *      存在广播ID（ID号）0xFE，使用广播ID所有的舵机不会返回信息（除读取舵机ID号外）
 * -----------------------------------
 *
 * 通信要求：
 *          波特率： 115200
 *          舵机ID：0-253(由用户设置，默认为1)
 *          支持角度回读
 *          转动范围：0-1000(线性映射为0°-240°)
 *
 * @todo 要增加一个通讯延时从而保证舵机执行完指令
 *************************************/

/// @brief 单个总线舵机初始化
/// @param ServoID 舵机ID
/// @param Bias 舵机角度偏差
/// @param vlimit_l 舵机工作电压下限
/// @param vlimit_h 舵机工作电压上限
/// @param Temp 舵机温度
/// @param alimit_l 舵机角度下限
/// @param alimit_h 舵机角度上限
/// @param on 舵机启动，1为启动，0为关闭
/// @return 1:初始化成功, 0:UART发送失败
static inline uint8_t SingleSerovoInit(uint8_t ServoID, uint8_t Bias, uint32_t vlimit_l, uint32_t vlimit_h, uint8_t Temp, uint32_t alimit_l, uint32_t alimit_h, uint8_t on)
{
    // 设置协议
    bool SERVO_COM = true;
    ServoPacket_t HxFb;
    UART_SetProtocol(BSP_UART6, SERVO_COM); // true为舵机, false为DART上位机通信协议

    uint8_t InitDataArr[16] = {0}; // 发送数据的暂存数组
    uint16_t send_result = 0;      // UART发送结果

    memset(InitDataArr, 0x55, 16);
    // 初始化舵机ID(读写)
    // 数据帧格式: 帧头(0x55 0x55) | ID | 数据长度 | 指令 | 参数 | 校验
    InitDataArr[2] = 0x01;                                                            // ID
    InitDataArr[3] = get_servo_data_length(SERVO_ID_READ);                            // 数据长度
    InitDataArr[4] = get_servo_command_value(SERVO_ID_READ);                          // 指令
    InitDataArr[5] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_ID_READ)); // 校验
    send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_ID_READ) + 3);
    if (send_result == 0)
        return 0; // UART发送失败
    HAL_Delay(2);
    UART_GetServoPacket(BSP_UART6, &HxFb);

    InitDataArr[3] = get_servo_data_length(SERVO_ID_WRITE);                            // 数据长度
    InitDataArr[4] = get_servo_command_value(SERVO_ID_WRITE);                          // 指令
    InitDataArr[5] = ServoID;                                                          // 参数: 设置ID
    InitDataArr[6] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_ID_WRITE)); // 校验
    send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_ID_WRITE) + 3);
    if (send_result == 0)
        return 0; // UART发送失败
    HAL_Delay(2);

    // 初始化舵机的工作模式(读写) ？？？？似乎不用
    // InitDataArr[2] = ServoID;
    // InitDataArr[3] = get_servo_data_length(SERVO_OR_MOTOR_MODE_READ);                                   // 数据长度
    // InitDataArr[4] = get_servo_command_value(SERVO_OR_MOTOR_MODE_READ);                                 // 指令
    // InitDataArr[5] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_OR_MOTOR_MODE_READ));        // 校验

    // HAL_Delay(1);

    // 初始化舵机的bias(读写)，偏差是 +-30°，这里的写范围是 -125~125 线性映射到 =+-30°
    // InitDataArr[2] = ServoID;
    // InitDataArr[3] = get_servo_data_length(SERVO_ANGLE_OFFSET_READ);                                    // 数据长度
    // InitDataArr[4] = get_servo_command_value(SERVO_ANGLE_OFFSET_READ);                                  // 指令
    // InitDataArr[5] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_ANGLE_OFFSET_READ));         // 校验
    // send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_ANGLE_OFFSET_READ) + 3);
    // if (send_result == 0)
    //     return 0; // UART发送失败
    // HAL_Delay(1);

    // InitDataArr[3] = get_servo_data_length(SERVO_ANGLE_OFFSET_ADJUST);                                  // 数据长度
    // InitDataArr[4] = get_servo_command_value(SERVO_ANGLE_OFFSET_ADJUST);                                // 指令
    // InitDataArr[5] = Bias;                                                                              // 参数
    // InitDataArr[6] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_ANGLE_OFFSET_ADJUST));       // 校验
    // send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_ANGLE_OFFSET_ADJUST) + 3);
    // if (send_result == 0)
    //     return 0; // UART发送失败
    // HAL_Delay(1);

    // InitDataArr[2] = ServoID;
    // InitDataArr[3] = get_servo_data_length(SERVO_ANGLE_OFFSET_WRITE);                                   // 数据长度
    // InitDataArr[4] = get_servo_command_value(SERVO_ANGLE_OFFSET_WRITE);                                 // 指令
    // InitDataArr[5] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_ANGLE_OFFSET_WRITE));        // 校验
    // send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_ANGLE_OFFSET_WRITE) + 3);
    // if (send_result == 0)
    //     return 0; // UART发送失败
    // HAL_Delay(1);

    // // 初始话舵机的温度阈值 电压阈值 角度阈值 上电状态(读写)
    // // 温度阈值
    // InitDataArr[2] = ServoID;
    // InitDataArr[3] = get_servo_data_length(SERVO_TEMP_MAX_LIMIT_READ);                                  // 数据长度
    // InitDataArr[4] = get_servo_command_value(SERVO_TEMP_MAX_LIMIT_READ);                                // 指令
    // InitDataArr[5] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_TEMP_MAX_LIMIT_READ));       // 校验
    // send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_TEMP_MAX_LIMIT_READ) + 3);
    // if (send_result == 0)
    //     return 0; // UART发送失败
    // HAL_Delay(1);

    // InitDataArr[3] = get_servo_data_length(SERVO_TEMP_MAX_LIMIT_WRITE);                                 // 数据长度
    // InitDataArr[4] = get_servo_command_value(SERVO_TEMP_MAX_LIMIT_WRITE);                               // 指令
    // InitDataArr[5] = Temp;                                                                              // 参数: 温度阈值范围为50到85
    // InitDataArr[6] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_TEMP_MAX_LIMIT_WRITE));      // 校验
    // send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_TEMP_MAX_LIMIT_WRITE) + 3);
    // if (send_result == 0)
    //     return 0; // UART发送失败
    // HAL_Delay(1);

    // // 电压阈值
    // // 最小初始化电压范围:4500 - 14000 mv, 最大输入电压范围:4500 - 14000 mv
    // InitDataArr[2] = ServoID;
    // InitDataArr[3] = get_servo_data_length(SERVO_VIN_LIMIT_READ);                                       // 数据长度
    // InitDataArr[4] = get_servo_command_value(SERVO_VIN_LIMIT_READ);                                     // 指令
    // InitDataArr[5] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_VIN_LIMIT_READ));            // 校验
    // send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_VIN_LIMIT_READ) + 3);
    // if (send_result == 0)
    //     return 0; // UART发送失败
    // HAL_Delay(1);

    // InitDataArr[3] = get_servo_data_length(SERVO_VIN_LIMIT_WRITE);                                      // 数据长度
    // InitDataArr[4] = get_servo_command_value(SERVO_VIN_LIMIT_WRITE);                                    // 指令
    // vlimit_l = vlimit_l < 4500 ? 4500 : vlimit_l;
    // vlimit_h = vlimit_h > 14000 ? 14000 : vlimit_h;
    // uint32_t real_limit_l = vlimit_l > vlimit_h ? vlimit_h : vlimit_l;
    // uint32_t real_limit_h = vlimit_l > vlimit_h ? vlimit_l : vlimit_h;
    // InitDataArr[5] = (uint8_t)real_limit_l;                                                             // 参数: 最小输入电压低8位
    // InitDataArr[6] = (uint8_t)(real_limit_l >> 8);                                                      // 参数: 最小输入电压高8位
    // InitDataArr[7] = (uint8_t)real_limit_h;                                                             // 参数: 最大输入电压低8位
    // InitDataArr[8] = (uint8_t)(real_limit_h >> 8);                                                      // 参数: 最大输入电压高8位
    // InitDataArr[9] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_VIN_LIMIT_WRITE));           // 校验
    // send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_VIN_LIMIT_WRITE) + 3);
    // if (send_result == 0)
    //     return 0; // UART发送失败
    // HAL_Delay(1);

    // // 回读确认数据之后再次保存防止掉电丢失
    // InitDataArr[3] = get_servo_data_length(SERVO_VIN_LIMIT_READ);                                       // 数据长度
    // InitDataArr[4] = get_servo_command_value(SERVO_VIN_LIMIT_READ);                                     // 指令
    // InitDataArr[5] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_VIN_LIMIT_READ));            // 校验
    // UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_VIN_LIMIT_READ) + 3);
    // HAL_Delay(1);

    // InitDataArr[3] = get_servo_data_length(SERVO_ANGLE_OFFSET_WRITE);                                   // 数据长度
    // InitDataArr[4] = get_servo_command_value(SERVO_ANGLE_OFFSET_WRITE);                                 // 指令
    // InitDataArr[5] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_ANGLE_OFFSET_WRITE));        // 校验
    // UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_ANGLE_OFFSET_WRITE) + 3);
    // HAL_Delay(1);

    // 角度阈值
    // 数据0 - 1000, 最小角度低八位 | 最小角度高八位 | 最大角度低八位 | 最大角度高8位
    // InitDataArr[2] = ServoID;
    // InitDataArr[3] = get_servo_data_length(SERVO_ANGLE_LIMIT_READ);                                     // 数据长度
    // InitDataArr[4] = get_servo_command_value(SERVO_ANGLE_LIMIT_READ);                                   // 指令
    // InitDataArr[5] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_ANGLE_LIMIT_READ));          // 校验
    // send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_ANGLE_LIMIT_READ) + 3);
    // if (send_result == 0)
    //     return 0; // UART发送失败
    // HAL_Delay(1);

    // InitDataArr[3] = get_servo_data_length(SERVO_ANGLE_LIMIT_WRITE);                                    // 数据长度
    // InitDataArr[4] = get_servo_command_value(SERVO_ANGLE_LIMIT_WRITE);                                  // 指令
    // alimit_l = alimit_l > 1000 ? 1000 : alimit_l;
    // alimit_h = alimit_h > 1000 ? 1000 : alimit_h;
    // real_limit_l = alimit_l > alimit_h ? alimit_h : alimit_l;
    // real_limit_h = alimit_l > alimit_h ? alimit_l : alimit_h;
    // InitDataArr[5] = (uint8_t)real_limit_l;                                                             // 参数: 最低角度低位
    // InitDataArr[6] = (uint8_t)(real_limit_l >> 8);                                                      // 参数: 最低角度高位
    // InitDataArr[7] = (uint8_t)real_limit_h;                                                             // 参数: 最高角度低位
    // InitDataArr[8] = (uint8_t)(real_limit_h >> 8);                                                      // 参数: 最高角度高位
    // InitDataArr[9] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_ANGLE_LIMIT_WRITE));         // 校验
    // send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_ANGLE_LIMIT_WRITE) + 3);
    // if (send_result == 0)
    //     return 0; // UART发送失败
    // HAL_Delay(1);

    // // 上电状态
    // InitDataArr[2] = ServoID;
    // InitDataArr[3] = get_servo_data_length(SERVO_LOAD_OR_UNLOAD_READ);                                  // 数据长度
    // InitDataArr[4] = get_servo_command_value(SERVO_LOAD_OR_UNLOAD_READ);                                // 指令
    // InitDataArr[5] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_LOAD_OR_UNLOAD_READ));       // 校验
    // send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_LOAD_OR_UNLOAD_READ) + 3);
    // if (send_result == 0)
    //     return 0; // UART发送失败
    // HAL_Delay(1);

    // InitDataArr[3] = get_servo_data_length(SERVO_LOAD_OR_UNLOAD_WRITE);                                 // 数据长度
    // InitDataArr[4] = get_servo_command_value(SERVO_LOAD_OR_UNLOAD_WRITE);                               // 指令
    // InitDataArr[5] = on;                                                                                // 参数
    // InitDataArr[6] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_LOAD_OR_UNLOAD_WRITE));      // 校验
    // send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_LOAD_OR_UNLOAD_WRITE) + 3);
    // if (send_result == 0)
    //     return 0; // UART发送失败
    // HAL_Delay(1);

    return 1; // 初始化成功
}

/// @brief 换弹舵机初始化
/// @param  无
/// @todo 参数都是瞎几把填的
void ServoInit(void)
{
    // 初始化3个总线舵机
    // 到时候一个一个来初始化
    SingleSerovoInit(1, 0, 4500, 14000, 90, 0, 90, 0);
    // SingleSerovoInit(2, 0, 0, 0, 0, 0, 0, 0);
    // SingleSerovoInit(3, 0, 0, 0, 0, 0, 0, 0);
}

/// @brief 总线舵机控制函数
/// @param ID 总线舵机ID
/// @param Angle 总线舵机转过的角度
/// @param Time 转动过程时间
/// @retval 无
/// @note 根据时间匀速转动到对应设置的角度
void ServoControlPos(uint8_t ID, uint16_t Angle, uint16_t Time)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;                                                             // 帧头
    data[1] = 0x55;                                                             // 帧头
    data[2] = ID;                                                               // ID号
    data[3] = get_servo_data_length(SERVO_MOVE_TIME_WRITE);                     // 数据长度
    data[4] = get_servo_command_value(SERVO_MOVE_TIME_WRITE);                   // 指令
    data[5] = (uint8_t)Angle;                                                   // 参数: 角度低八位
    data[6] = (uint8_t)(Angle >> 8);                                            // 参数: 角度高八位
    data[7] = (uint8_t)Time;                                                    // 参数: 时间低八位
    data[8] = (uint8_t)(Time >> 8);                                             // 参数: 时间高8位
    data[9] = CRC_GNERATOR(data, get_servo_data_length(SERVO_MOVE_TIME_WRITE)); // CRC校验
    UART_Send(BSP_UART6, (const uint8_t *)data, get_servo_data_length(SERVO_MOVE_TIME_WRITE) + 3);
}
