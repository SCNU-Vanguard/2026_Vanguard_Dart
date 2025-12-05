/****************************************************************
 * 换弹结构总线舵机
 *
 * 协议说明：
 * other_mcu_forcing = 0: 无MCU驱动板协议（115200波特率，有CRC校验）
 * other_mcu_forcing = 1: 有MCU控制板协议（9600波特率，无CRC校验）
 *
 * todo 待测试
 ****************************************************************/

#include "HX06L.h"
#include "bsp_dwt.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"

#if (other_mcu_forcing == 1)
/*******************************************************************************
 * 有MCU控制板协议实现
 * 波特率：9600
 * 帧格式：0x55 0x55 | Length | Cmd | Param...
 * 无CRC校验
 ******************************************************************************/

/*************************************
 * 控制板通信数据帧格式：
 * 帧头：0x55 0x55
 * 数据长度：参数个数N + 指令 + 数据长度本身 = N + 2
 * 指令：控制指令
 * 参数：控制信息
 *
 * 示例：
 *      0x55 0x55 | Length | Cmd | Param1 | Param2 | ...
 *************************************/

/// @brief 换弹舵机初始化（控制板协议）
/// @param  无
void ServoInit(void)
{
    // 控制板协议下，舵机初始化由控制板自动完成
    // 只需要确保通信正常即可
    // 可以发送一个读取电压的命令来测试通信
    uint8_t data[4];
    data[0] = 0x55;                    // 帧头
    data[1] = 0x55;                    // 帧头
    data[2] = 0x02;                    // 数据长度 = 0 + 2
    data[3] = CMD_GET_BATTERY_VOLTAGE; // 指令：获取电池电压
    UART_Send(BSP_UART6, (const uint8_t *)data, 4);
    HAL_Delay(5);
}

/// @brief 总线舵机控制函数（控制板协议）
/// @param ID 总线舵机ID
/// @param Angle 总线舵机转过的角度（0-1000对应0-240°）
/// @param Time 转动过程时间（0-30000ms）
/// @retval 无
/// @note 控制单个舵机在指定时间内转到指定角度
void ServoControlPos(uint8_t ID, uint16_t Angle, uint16_t Time)
{
    // 设置舵机协议
    bool SERVO_COM = true;
    UART_SetProtocol(BSP_UART6, SERVO_COM);

    // 帧格式：0x55 0x55 | Length | Cmd | 舵机个数 | 时间L | 时间H | ID | 角度L | 角度H
    // 数据长度 = 舵机个数*3 + 5 = 1*3 + 5 = 8（参数个数6 + 2）
    uint8_t data[10];
    data[0] = 0x55;                    // 帧头
    data[1] = 0x55;                    // 帧头
    data[2] = 0x08;                    // 数据长度 = 1*3 + 5 = 8
    data[3] = CMD_SERVO_MOVE;          // 指令：控制舵机转动
    data[4] = 0x01;                    // 参数1：控制舵机的个数 = 1
    data[5] = (uint8_t)(Time & 0xFF);  // 参数2：时间低八位
    data[6] = (uint8_t)(Time >> 8);    // 参数3：时间高八位
    data[7] = ID;                      // 参数4：舵机ID号
    data[8] = (uint8_t)(Angle & 0xFF); // 参数5：角度位置低八位
    data[9] = (uint8_t)(Angle >> 8);   // 参数6：角度位置高八位

    UART_Send(BSP_UART6, (const uint8_t *)data, 10);
    HAL_Delay(2); // 等待控制板处理命令
}

/// @brief 控制多个舵机同时转动（控制板协议）
/// @param servo_num 舵机个数
/// @param servo_ids 舵机ID数组
/// @param angles 角度数组（0-1000）
/// @param time 转动时间（ms）
/// @retval 无
void ServoControlMulti(uint8_t servo_num, uint8_t *servo_ids, uint16_t *angles, uint16_t time)
{
    if (servo_num == 0 || servo_ids == NULL || angles == NULL)
        return;

    // 设置舵机协议
    bool SERVO_COM = true;
    UART_SetProtocol(BSP_UART6, SERVO_COM);

    // 帧格式：0x55 0x55 | Length | Cmd | 舵机个数 | 时间L | 时间H | [ID | 角度L | 角度H] * N
    // 数据长度 = 舵机个数*3 + 5
    uint8_t data_len = servo_num * 3 + 5;
    uint8_t data[64]; // 最大支持约20个舵机

    if (data_len + 2 > sizeof(data))
        return; // 数据太长

    data[0] = 0x55;                   // 帧头
    data[1] = 0x55;                   // 帧头
    data[2] = data_len;               // 数据长度
    data[3] = CMD_SERVO_MOVE;         // 指令
    data[4] = servo_num;              // 舵机个数
    data[5] = (uint8_t)(time & 0xFF); // 时间低八位
    data[6] = (uint8_t)(time >> 8);   // 时间高八位

    // 填充每个舵机的ID和角度
    for (uint8_t i = 0; i < servo_num; i++)
    {
        data[7 + i * 3] = servo_ids[i];                // 舵机ID
        data[8 + i * 3] = (uint8_t)(angles[i] & 0xFF); // 角度低八位
        data[9 + i * 3] = (uint8_t)(angles[i] >> 8);   // 角度高八位
    }

    UART_Send(BSP_UART6, (const uint8_t *)data, data_len + 2);
    HAL_Delay(2);
}

/// @brief 运行动作组（控制板协议）
/// @param group_num 动作组编号
/// @param run_times 运行次数（0表示无限次）
void ServoRunActionGroup(uint8_t group_num, uint16_t run_times)
{
    uint8_t data[7];
    data[0] = 0x55;                        // 帧头
    data[1] = 0x55;                        // 帧头
    data[2] = 0x05;                        // 数据长度 = 3 + 2
    data[3] = CMD_ACTION_GROUP_RUN;        // 指令
    data[4] = group_num;                   // 参数1：动作组编号
    data[5] = (uint8_t)(run_times & 0xFF); // 参数2：次数低八位
    data[6] = (uint8_t)(run_times >> 8);   // 参数3：次数高八位

    UART_Send(BSP_UART6, (const uint8_t *)data, 7);
    HAL_Delay(2);
}

/// @brief 停止动作组（控制板协议）
void ServoStopActionGroup(void)
{
    uint8_t data[4];
    data[0] = 0x55;                  // 帧头
    data[1] = 0x55;                  // 帧头
    data[2] = 0x02;                  // 数据长度 = 0 + 2
    data[3] = CMD_ACTION_GROUP_STOP; // 指令

    UART_Send(BSP_UART6, (const uint8_t *)data, 4);
    HAL_Delay(2);
}

/// @brief 设置动作组速度（控制板协议）
/// @param group_num 动作组编号（0xFF表示所有动作组）
/// @param speed_percent 速度百分比（100表示原速，200表示2倍速）
void ServoSetActionGroupSpeed(uint8_t group_num, uint16_t speed_percent)
{
    uint8_t data[7];
    data[0] = 0x55;                            // 帧头
    data[1] = 0x55;                            // 帧头
    data[2] = 0x05;                            // 数据长度 = 3 + 2
    data[3] = CMD_ACTION_GROUP_SPEED;          // 指令
    data[4] = group_num;                       // 参数1：动作组编号
    data[5] = (uint8_t)(speed_percent & 0xFF); // 参数2：速度百分比低八位
    data[6] = (uint8_t)(speed_percent >> 8);   // 参数3：速度百分比高八位

    UART_Send(BSP_UART6, (const uint8_t *)data, 7);
    HAL_Delay(2);
}

/// @brief 控制多个舵机卸力（控制板协议）
/// @param servo_num 舵机个数
/// @param servo_ids 舵机ID数组
void ServoUnloadMulti(uint8_t servo_num, uint8_t *servo_ids)
{
    if (servo_num == 0 || servo_ids == NULL)
        return;

    // 帧格式：0x55 0x55 | Length | Cmd | 舵机个数 | ID1 | ID2 | ...
    // 数据长度 = 舵机个数 + 3
    uint8_t data_len = servo_num + 3;
    uint8_t data[32];

    if (data_len + 2 > sizeof(data))
        return;

    data[0] = 0x55;                  // 帧头
    data[1] = 0x55;                  // 帧头
    data[2] = data_len;              // 数据长度
    data[3] = CMD_MULT_SERVO_UNLOAD; // 指令
    data[4] = servo_num;             // 舵机个数

    for (uint8_t i = 0; i < servo_num; i++)
    {
        data[5 + i] = servo_ids[i];
    }

    UART_Send(BSP_UART6, (const uint8_t *)data, data_len + 2);
    HAL_Delay(2);
}

/// @brief 获取电池电压（控制板协议）
/// @return 无（需要通过接收回调获取结果）
void ServoGetBatteryVoltage(void)
{
    uint8_t data[4];
    data[0] = 0x55;                    // 帧头
    data[1] = 0x55;                    // 帧头
    data[2] = 0x02;                    // 数据长度 = 0 + 2
    data[3] = CMD_GET_BATTERY_VOLTAGE; // 指令

    UART_Send(BSP_UART6, (const uint8_t *)data, 4);
}

#else
/*******************************************************************************
 * 无MCU驱动板协议实现
 * 波特率：115200
 * 帧格式：0x55 0x55 | ID | Length | Cmd | Param... | CRC
 * 有CRC校验
 ******************************************************************************/

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

/*************************************
 * HX06L总线舵机通信数据帧格式：
 * 帧头 | ID号 | 数据长度 | 指令 | 参数 | 校验 | 字段
 *
 * 各个字段作用：
 * 帧头：起始位，表示数据传输开始
 * ID号：各个舵机各自的ID号，ID号不可以相同
 * 数据长度：待发送的数据长度，数值为Length = DataLength + 3
 * 指令：位置 / 速度
 * 校验码：用于验证数据的完整性
 *
 * 示例：
 *      0x55 0x55 | ID | Length | Cmd | Param(可以n多个) | CheckSum(校验和)
 * 注意：
 *      每个舵机的ID不同，ID号范围为0-253，转换为16进制0x00-0xFD
 *      存在广播ID（ID号）0xFE，使用广播ID所有的舵机不会返回信息（除读取舵机ID号外）
 *
 * 通信要求：
 *          波特率： 115200
 *          舵机ID：0-253(由用户设置，默认为1)
 *          支持角度回读
 *          转动范围：0-1000(线性映射为0°-240°)
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

/// @brief 换弹舵机初始化（无MCU协议）
/// @param  无
void ServoInit(void)
{
    // 初始化3个总线舵机
    SingleSerovoInit(1, 1);
    SingleSerovoInit(2, 1);
    SingleSerovoInit(3, 1);
}

/// @brief 总线舵机控制函数（无MCU协议）
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

#endif /* other_mcu_forcing */
