// 舵机初始化的ID和信息都是只初始化一次就行，后面可以不用初始化，直接使用
// 可能久了会有偏差，建议使用时间长了之后有偏差回来更新BIAS
// 与控制器的通信速率是9600，但是后面还得上其他的驱动，到时候再看看

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
/// @param on 舵机启动，1为启动，0为关闭
/// @return 1:初始化成功, 0:UART发送失败
static inline uint8_t SingleSerovoInit(uint8_t ServoID, uint8_t on)
{
    uint8_t InitDataArr[20];
    memset(InitDataArr, 0x55, sizeof(InitDataArr));
    // 上电状态 - 必须启用，否则舵机不会响应位置控制指令！
    InitDataArr[2] = ServoID;
    InitDataArr[3] = get_servo_data_length(SERVO_LOAD_OR_UNLOAD_READ);                            // 数据长度
    InitDataArr[4] = get_servo_command_value(SERVO_LOAD_OR_UNLOAD_READ);                          // 指令
    InitDataArr[5] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_LOAD_OR_UNLOAD_READ)); // 校验
    uint8_t send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_LOAD_OR_UNLOAD_READ) + 3);
    if (send_result == 0)
        return 0; // UART发送失败
    HAL_Delay(2);

    InitDataArr[3] = get_servo_data_length(SERVO_LOAD_OR_UNLOAD_WRITE);                            // 数据长度
    InitDataArr[4] = get_servo_command_value(SERVO_LOAD_OR_UNLOAD_WRITE);                          // 指令
    InitDataArr[5] = 1;                                                                            // 参数: 1为上电(启用)
    InitDataArr[6] = CRC_GNERATOR(InitDataArr, get_servo_data_length(SERVO_LOAD_OR_UNLOAD_WRITE)); // 校验
    send_result = UART_Send(BSP_UART6, (const uint8_t *)InitDataArr, get_servo_data_length(SERVO_LOAD_OR_UNLOAD_WRITE) + 3);
    if (send_result == 0)
        return 0; // UART发送失败
    HAL_Delay(2);

    return 1; // 初始化成功
}

/// @brief 换弹舵机初始化
/// @param  无
/// @todo 参数都是瞎几把填的
void ServoInit(void)
{
    // 初始化3个总线舵机
    // 到时候一个一个来初始化
    SingleSerovoInit(1, 1);
    SingleSerovoInit(2, 1);
    SingleSerovoInit(3, 1);
}

/// @brief 总线舵机控制函数
/// @param ID 总线舵机ID
/// @param Angle 总线舵机转过的角度
/// @param Time 转动过程时间
/// @retval 无
/// @note 根据时间匀速转动到对应设置的角度
void ServoControlPos(uint8_t ID, uint16_t Angle, uint16_t Time)
{
    // 设置舵机协议
    bool SERVO_COM = true;
    UART_SetProtocol(BSP_UART6, SERVO_COM);

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
    HAL_Delay(2); // 等待舵机处理命令
}
