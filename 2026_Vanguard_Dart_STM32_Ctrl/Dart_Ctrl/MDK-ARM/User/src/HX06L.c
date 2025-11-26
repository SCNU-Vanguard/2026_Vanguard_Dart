#include "HX06L.h"
#include "bsp_dwt.h"
#include <string.h>

/// @brief CRC校验生成（调用bsp_uart中的统一CRC函数）
/// @param prtSendData 发送数据的指针（包含包头0x55 0x55）
/// @param DataLength 数据长度（不包含包头和CRC）
/// @return CRC校验码数据
static inline uint8_t CRC_GNERATOR(uint8_t *prtSendData, uint8_t DataLength) __attribute__((__always_inline__))
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
static inline uint8_t SerovoInit(uint8_t ServoID, WorkMode Mode, uint8_t Bias) __attribute__((__always_inline__))
{
    // 设置协议

    uint8_t InitDataArr[16] = {0}; // 发送数据的暂存数组
    uint8_t DataLength = 0;
    memset(InitDataArr, 0x55, 16);
    // 初始化舵机ID(读写)
    InitDataArr[2] = 0x01;
    InitDataArr[3] = get_servo_command_value(SERVO_ID_READ);
    DataLength = get_servo_data_length(SERVO_ID_READ);
    InitDataArr[4] = CRC_GNERATOR(InitDataArr, DataLength);
    UART_Send(BSP_UART1, (const uint8_t *)InitDataArr, DataLength + 3);
    HAL_Delay(1);

    InitDataArr[3] = get_servo_command_value(SERVO_ID_WRITE);
    DataLength = get_servo_data_length(SERVO_ID_WRITE);
    InitDataArr[4] = 0x0D;
    InitDataArr[5] = ServoID; // 设置ID
    InitDataArr[6] = CRC_GNERATOR(InitDataArr, DataLength);
    UART_Send(BSP_UART1, (const uint8_t *)InitDataArr, DataLength + 3);
    HAL_Delay(1);

    // 初始化舵机的工作模式(读写) ？？？？似乎不用
    // InitDataArr[2] = ServoID;
    // InitDataArr[3] = get_servo_command_value(SERVO_OR_MOTOR_MODE_READ);
    // DataLength = get_servo_data_length(SERVO_OR_MOTOR_MODE_READ);
    // InitDataArr[4] = ;

    // HAL_Delay(1);

    // 初始化舵机的bias(读写)，偏差是 +-30°，这里的写范围是 -125~125 线性映射到 =+-30°
    InitDataArr[2] = ServoID;
    InitDataArr[3] = get_servo_command_value(SERVO_ANGLE_OFFSET_READ);
    DataLength = get_servo_data_length(SERVO_ANGLE_OFFSET_READ);
    InitDataArr[4] = CRC_GNERATOR(InitDataArr, DataLength);
    UART_Send(BSP_UART1, (const uint8_t *)InitDataArr, DataLength + 3);
    HAL_Delay(1);

    InitDataArr[3] = get_servo_command_value(SERVO_ANGLE_OFFSET_ADJUST);
    DataLength = get_servo_data_length(SERVO_ANGLE_OFFSET_ADJUST);
    InitDataArr[4] = 0x11;
    InitDataArr[5] = Bias;
    InitDataArr[6] = CRC_GNERATOR(InitDataArr, DataLength);
    UART_Send(BSP_UART1, (const uint8_t *)InitDataArr, DataLength + 3);
    HAL_Delay(1);

    InitDataArr[2] = ServoID;
    InitDataArr[3] = get_servo_command_value(SERVO_ANGLE_OFFSET_WRITE);
    DataLength = get_servo_data_length(SERVO_ANGLE_OFFSET_WRITE);
    InitDataArr[4] = 0x12;
    InitDataArr[5] = CRC_GNERATOR(InitDataArr, DataLength);
    UART_Send(BSP_UART1, (const uint8_t *)InitDataArr, DataLength + 3);
    HAL_Delay(1);

    // 初始话舵机的温度阈值 电压阈值 角度阈值 上电状态(读写)
    // todo:这里还没有更改具体的参数
    // 温度阈值
    InitDataArr[2] = ServoID;
    InitDataArr[3] = get_servo_command_value(SERVO_TEMP_MAX_LIMIT_READ);
    DataLength = get_servo_data_length(SERVO_TEMP_MAX_LIMIT_READ);
    InitDataArr[4] = CRC_GNERATOR(InitDataArr, DataLength);
    UART_Send(BSP_UART1, (const uint8_t *)InitDataArr, DataLength + 3);
    HAL_Delay(1);

    InitDataArr[3] = get_servo_command_value(SERVO_TEMP_MAX_LIMIT_WRITE);
    DataLength = get_servo_data_length(SERVO_TEMP_MAX_LIMIT_WRITE);
    InitDataArr[4] = 0x16;
    InitDataArr[5] = Bias; // todo
    InitDataArr[6] = ;     // 最小输入电压低8位
    InitDataArr[7] = ;     // 最小输入电压高8位
    InitDataArr[8] = ;     // 最大输入电压低8位
    InitDataArr[9] = ;     // 最大输入电压高8位
    // 输入电压4500-14000mV

    InitDataArr[10] = CRC_GNERATOR(InitDataArr, DataLength);
    UART_Send(BSP_UART1, (const uint8_t *)InitDataArr, DataLength + 3);
    HAL_Delay(1);

    // 电压阈值
    InitDataArr[2] = ServoID;
    InitDataArr[3] = get_servo_command_value(SERVO_VIN_LIMIT_READ);
    DataLength = get_servo_data_length(SERVO_VIN_LIMIT_READ);
    InitDataArr[4] = CRC_GNERATOR(InitDataArr, DataLength);
    UART_Send(BSP_UART1, (const uint8_t *)InitDataArr, DataLength + 3);
    HAL_Delay(1);

    InitDataArr[3] = get_servo_command_value(SERVO_VIN_LIMIT_WRITE);
    DataLength = get_servo_data_length(SERVO_VIN_LIMIT_WRITE);
    InitDataArr[4] = ;
    InitDataArr[5] = Bias; // todo
    InitDataArr[6] = CRC_GNERATOR(InitDataArr, DataLength);
    UART_Send(BSP_UART1, (const uint8_t *)InitDataArr, DataLength + 3);
    HAL_Delay(1);

    // 角度阈值
    InitDataArr[2] = ServoID;
    InitDataArr[3] = get_servo_command_value(SERVO_ANGLE_LIMIT_READ);
    DataLength = get_servo_data_length(SERVO_ANGLE_LIMIT_READ);
    InitDataArr[4] = CRC_GNERATOR(InitDataArr, DataLength);
    UART_Send(BSP_UART1, (const uint8_t *)InitDataArr, DataLength + 3);
    HAL_Delay(1);

    InitDataArr[3] = get_servo_command_value(SERVO_ANGLE_LIMIT_WRITE);
    DataLength = get_servo_data_length(SERVO_ANGLE_LIMIT_WRITE);
    InitDataArr[4] = ;
    InitDataArr[5] = Bias; // todo
    InitDataArr[6] = CRC_GNERATOR(InitDataArr, DataLength);
    UART_Send(BSP_UART1, (const uint8_t *)InitDataArr, DataLength + 3);
    HAL_Delay(1);

    // 上电状态
    InitDataArr[2] = ServoID;
    InitDataArr[3] = get_servo_command_value(SERVO_LOAD_OR_UNLOAD_READ);
    DataLength = get_servo_data_length(SERVO_LOAD_OR_UNLOAD_READ);
    InitDataArr[4] = CRC_GNERATOR(InitDataArr, DataLength);
    UART_Send(BSP_UART1, (const uint8_t *)InitDataArr, DataLength + 3);
    HAL_Delay(1);

    InitDataArr[3] = get_servo_command_value(SERVO_LOAD_OR_UNLOAD_WRITE);
    DataLength = get_servo_data_length(SERVO_LOAD_OR_UNLOAD_WRITE);
    InitDataArr[4] = ;
    InitDataArr[5] = Bias; // todo
    InitDataArr[6] = CRC_GNERATOR(InitDataArr, DataLength);
    UART_Send(BSP_UART1, (const uint8_t *)InitDataArr, DataLength + 3);
    HAL_Delay(1);
}
