/****************************************************************
 * 换弹结构总线舵机
 *
 * 协议说明：
 * other_mcu_forcing = 0: 无MCU驱动板协议（115200波特率，有CRC校验）
 * other_mcu_forcing = 1: 有MCU控制板协议（9600波特率，无CRC校验）
 *
 * todo 待测试
 * note 现在的舵机暂时无需手动初始化，当前已经通过上位机初始化了参数
 ****************************************************************/

#include "HX06L.h"
#include "bsp_dwt.h"
#include <string.h>
#include "UartProtocol.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "semphr.h"

/* UART3 舵机发送互斥量 */
static SemaphoreHandle_t g_xServoUartMtx = NULL;
static StaticSemaphore_t g_xServoUartMtxBuf;

static void Servo_InitMutex(void)
{
    if (g_xServoUartMtx == NULL)
    {
        g_xServoUartMtx = xSemaphoreCreateMutexStatic(&g_xServoUartMtxBuf);
    }
}

static inline void Servo_UART_Send(const uint8_t *data, uint16_t len)
{
    if (g_xServoUartMtx != NULL)
    {
        xSemaphoreTake(g_xServoUartMtx, portMAX_DELAY);
    }
    UART_Send(BSP_UART3, data, len);
    if (g_xServoUartMtx != NULL)
    {
        xSemaphoreGive(g_xServoUartMtx);
    }
}

static ServoRegistryItem_t g_servo_registry[SERVO_REG_MAX_COUNT];
static uint8_t g_servo_registry_count = 0;

void ServoRegistry_Reset(void)
{
    memset(g_servo_registry, 0, sizeof(g_servo_registry));
    g_servo_registry_count = 0;
}

bool ServoRegistry_RegisterBatch(const ServoRegistryItem_t *items, uint8_t count)
{
    if (items == NULL || count == 0 || count > SERVO_REG_MAX_COUNT)
    {
        return false;
    }

    for (uint8_t i = 0; i < count; i++)
    {
        for (uint8_t j = (uint8_t)(i + 1); j < count; j++)
        {
            if (items[i].id == items[j].id)
            {
                return false;
            }
        }
    }

    memset(g_servo_registry, 0, sizeof(g_servo_registry));
    memcpy(g_servo_registry, items, sizeof(ServoRegistryItem_t) * count);
    g_servo_registry_count = count;
    return true;
}

const ServoRegistryItem_t *ServoRegistry_Find(uint8_t id)
{
    for (uint8_t i = 0; i < g_servo_registry_count; i++)
    {
        if (g_servo_registry[i].id == id)
        {
            return &g_servo_registry[i];
        }
    }
    return NULL;
}

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
/// @retval true:初始化正常, false:初始化存在电压问题
bool ServoInit(void)
{
    Servo_InitMutex();
    // 控制板协议下，舵机初始化由控制板自动完成
    // 只需要确保通信正常即可
    // 可以发送一个读取电压的命令来测试通信
    static uint8_t ServoInitCount = 0;
    if (ServoInitCount == 0)
    {
        uint8_t data[4];
        data[0] = 0x55;                    // 帧头
        data[1] = 0x55;                    // 帧头
        data[2] = 0x02;                    // 数据长度 = 0 + 2
        data[3] = CMD_GET_BATTERY_VOLTAGE; // 指令：获取电池电压
        Servo_UART_Send((const uint8_t *)data, 4);
        vTaskDelay(pdMS_TO_TICKS(100)); // 这里等待100ms,之后读取回调数据,确认电压是否正常
        ServoInitCount = 1;
    }
    ServoPacket_t InitData; // 这里可以不用进行初始化,函数调用时会直接memcpy数据过来,即使是没接收到数据也有HasServoPacket保障
    uint16_t servo_voltage = 0;
    if (Protocol_HasPacket(BSP_UART3))
    {
        Protocol_GetServoPacket(BSP_UART3, &InitData);
        // 回读之后解算电压的正常数据,不正常就返回一个false,之后使用一些东西提示总线舵机初始化失败
        servo_voltage = InitData.params[0] | (((uint16_t)InitData.params[1]) << 8); // 单位mv
        if ((servo_voltage > 8800) || (servo_voltage < 6000))
        {
            return false;
        }
        else
        {
            return true;
        }
    }
    return false;
}

/// @brief 总线舵机控制函数（控制板协议）
/// @param ID 总线舵机ID
/// @param Angle 总线舵机转过的角度（0-1000对应0-240°）
/// @param Time 转动过程时间（0-30000ms）
/// @retval 无
/// @note 控制单个舵机在指定时间内转到指定角度
void ServoControlPos(uint8_t ID, uint16_t Angle, uint16_t Time)
{
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

    Servo_UART_Send((const uint8_t *)data, 10);
    vTaskDelay(pdMS_TO_TICKS(1)); // 等待控制板处理命令
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

    // 帧格式：0x55 0x55 | Length | Cmd | 舵机个数 | 时间L | 时间H | [ID | 角度L | 角度H] * N
    // 数据长度 = 舵机个数 * 3 + 5
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

    Servo_UART_Send((const uint8_t *)data, data_len + 2);
    vTaskDelay(pdMS_TO_TICKS(1));
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

    Servo_UART_Send((const uint8_t *)data, 7);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/// @brief 停止动作组（控制板协议）
void ServoStopActionGroup(void)
{
    uint8_t data[4];
    data[0] = 0x55;                  // 帧头
    data[1] = 0x55;                  // 帧头
    data[2] = 0x02;                  // 数据长度 = 0 + 2
    data[3] = CMD_ACTION_GROUP_STOP; // 指令

    Servo_UART_Send((const uint8_t *)data, 4);
    vTaskDelay(pdMS_TO_TICKS(1));
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

    Servo_UART_Send((const uint8_t *)data, 7);
    vTaskDelay(pdMS_TO_TICKS(1));
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

    Servo_UART_Send((const uint8_t *)data, data_len + 2);
    vTaskDelay(pdMS_TO_TICKS(1));
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

    Servo_UART_Send((const uint8_t *)data, 4);
}

#else
/*******************************************************************************
 * 无MCU驱动板协议实现
 * 波特率：115200
 * 帧格式：0x55 0x55 | ID | Length | Cmd | Param... | CRC
 * 有CRC校验
 ******************************************************************************/

/*************************************
 * HX06L总线舵机通信数据帧格式：
 * 帧头 | ID号 | 数据长度 | 指令 | 参数 | 校验 | 字段
 *
 * 各个字段作用：
 * 帧头：起始位，表示数据传输开始
 * ID号：各个舵机各自的ID号，ID号不可以相同
 * 数据长度：待发送的数据长度，数值为Length = 参数个数 + 3
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

/// @brief CRC校验生成（调用bsp_uart中的统一CRC函数）
/// @param prtSendData 发送数据的指针（包含包头0x55 0x55）
/// @param DataLength 数据长度（从ID到参数的长度，不包含CRC）
/// @return CRC校验码数据
static inline uint8_t CRC_GNERATOR(uint8_t *prtSendData, uint8_t DataLength)
{
    return UART_Calculate_CRC(&prtSendData[2], DataLength);
}

// 辅助函数：根据指令名获取数据长度
static inline uint8_t get_servo_data_length(ServoCommandName cmd_name)
{
    if (cmd_name < SERVO_CMD_COUNT)
    {
        return servo_commands[cmd_name].data_len;
    }
    return 0;
}

// 辅助函数：根据指令名获取指令值
static inline uint8_t get_servo_command_value(ServoCommandName cmd_name)
{
    if (cmd_name < SERVO_CMD_COUNT)
    {
        return servo_commands[cmd_name].cmd;
    }
    return 0xFF;
}

/// @brief 换弹舵机初始化（无MCU协议）
/// @param  无
/// @retval true:初始化正常, false:初始化失败
/// @note 先让舵机上电，再控制到初始位置
bool ServoInit(void)
{
    Servo_InitMutex();
    static uint8_t ServoInitCount = 0;

    if (ServoInitCount == 0)
    {
        // 第一步：让所有舵机上电
        ServoSetLoad(1, 1);
        ServoSetLoad(2, 1);
        ServoSetLoad(3, 1);
        vTaskDelay(pdMS_TO_TICKS(50)); // 等待舵机上电稳定

        // 第二步：控制舵机到初始位置0
        ServoControlPos(1, 0, 500);
        ServoControlPos(2, 0, 500);
        ServoControlPos(3, 0, 500);
        vTaskDelay(pdMS_TO_TICKS(100)); // 等待返回消息
        ServoInitCount = 1;
    }

    // 检查是否收到返回消息（可选，用于验证通信）
    if (Protocol_HasPacket(BSP_UART3))
    {
        ServoPacket_t InitData;
        Protocol_GetServoPacket(BSP_UART3, &InitData);
        return true;
    }

    // 即使没收到返回也认为初始化完成（舵机可能没配置返回）
    return (ServoInitCount == 1);
}

/// @brief 总线舵机控制函数（无MCU协议）
/// @param ID 总线舵机ID
/// @param Angle 总线舵机转过的角度（0-1000对应0-240°）
/// @param Time 转动过程时间（0-30000ms）
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
    data[5] = (uint8_t)(Angle & 0xFF);                                          // 参数: 角度低八位
    data[6] = (uint8_t)(Angle >> 8);                                            // 参数: 角度高八位
    data[7] = (uint8_t)(Time & 0xFF);                                           // 参数: 时间低八位
    data[8] = (uint8_t)(Time >> 8);                                             // 参数: 时间高8位
    data[9] = CRC_GNERATOR(data, get_servo_data_length(SERVO_MOVE_TIME_WRITE)); // CRC校验
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_MOVE_TIME_WRITE) + 3);
    vTaskDelay(pdMS_TO_TICKS(1)); // 等待舵机处理命令
}

/// @brief 读取舵机预设角度和时间（立即控制模式）
/// @param ID 舵机ID
/// @note 需要通过接收回调获取结果，返回数据包含：角度(2字节) + 时间(2字节)
void ServoReadMoveTime(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;                                                            // 帧头
    data[1] = 0x55;                                                            // 帧头
    data[2] = ID;                                                              // ID号
    data[3] = get_servo_data_length(SERVO_MOVE_TIME_READ);                     // 数据长度
    data[4] = get_servo_command_value(SERVO_MOVE_TIME_READ);                   // 指令
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_MOVE_TIME_READ)); // CRC校验
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_MOVE_TIME_READ) + 3);
}

/// @brief 控制多个舵机同时转动（无MCU协议）
/// @param servo_num 舵机个数
/// @param servo_ids 舵机ID数组
/// @param angles 角度数组（0-1000）
/// @param time 转动时间（ms）
/// @retval 无
/// @note 无MCU协议不支持单帧多舵机，通过循环逐个控制实现
void ServoControlMulti(uint8_t servo_num, uint8_t *servo_ids, uint16_t *angles, uint16_t time)
{
    if (servo_num == 0 || servo_ids == NULL || angles == NULL)
        return;

    for (uint8_t i = 0; i < servo_num; i++)
    {
        ServoControlPos(servo_ids[i], angles[i], time);
    }
}

/// @brief 运行动作组（无MCU协议 - 不支持）
/// @param group_num 动作组编号
/// @param run_times 运行次数
/// @note 无MCU协议不支持动作组功能，此函数为空实现
void ServoRunActionGroup(uint8_t group_num, uint16_t run_times)
{
    // 无MCU驱动板协议不支持动作组功能
    (void)group_num;
    (void)run_times;
}

/// @brief 停止动作组（无MCU协议 - 不支持）
/// @note 无MCU协议不支持动作组功能，此函数为空实现
void ServoStopActionGroup(void)
{
    // 无MCU驱动板协议不支持动作组功能
}

/// @brief 设置动作组速度（无MCU协议 - 不支持）
/// @param group_num 动作组编号
/// @param speed_percent 速度百分比
/// @note 无MCU协议不支持动作组功能，此函数为空实现
void ServoSetActionGroupSpeed(uint8_t group_num, uint16_t speed_percent)
{
    // 无MCU驱动板协议不支持动作组功能
    (void)group_num;
    (void)speed_percent;
}

/// @brief 控制多个舵机卸力（无MCU协议）
/// @param servo_num 舵机个数
/// @param servo_ids 舵机ID数组
void ServoUnloadMulti(uint8_t servo_num, uint8_t *servo_ids)
{
    if (servo_num == 0 || servo_ids == NULL)
        return;

    uint8_t data[16] = {0x00};

    for (uint8_t i = 0; i < servo_num; i++)
    {
        data[0] = 0x55;                                                                  // 帧头
        data[1] = 0x55;                                                                  // 帧头
        data[2] = servo_ids[i];                                                          // ID号
        data[3] = get_servo_data_length(SERVO_LOAD_OR_UNLOAD_WRITE);                     // 数据长度
        data[4] = get_servo_command_value(SERVO_LOAD_OR_UNLOAD_WRITE);                   // 指令
        data[5] = 0;                                                                     // 参数: 0为卸力
        data[6] = CRC_GNERATOR(data, get_servo_data_length(SERVO_LOAD_OR_UNLOAD_WRITE)); // CRC校验
        Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_LOAD_OR_UNLOAD_WRITE) + 3);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/// @brief 获取电池/输入电压（无MCU协议）
/// @note 需要通过接收回调获取结果
void ServoGetBatteryVoltage(void)
{
    // 使用广播ID 0xFE 读取电压（所有舵机都会返回）
    // 或者指定某个舵机ID
    uint8_t data[16] = {0x00};
    data[0] = 0x55;                                                      // 帧头
    data[1] = 0x55;                                                      // 帧头
    data[2] = 1;                                                         // ID号（使用舵机1）
    data[3] = get_servo_data_length(SERVO_VIN_READ);                     // 数据长度
    data[4] = get_servo_command_value(SERVO_VIN_READ);                   // 指令
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_VIN_READ)); // CRC校验
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_VIN_READ) + 3);
}

/// @brief 读取舵机当前角度位置（无MCU协议独有）
/// @param ID 舵机ID
/// @note 需要通过接收回调获取结果
void ServoReadPosition(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;                                                      // 帧头
    data[1] = 0x55;                                                      // 帧头
    data[2] = ID;                                                        // ID号
    data[3] = get_servo_data_length(SERVO_POS_READ);                     // 数据长度
    data[4] = get_servo_command_value(SERVO_POS_READ);                   // 指令
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_POS_READ)); // CRC校验
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_POS_READ) + 3);
}

/// @brief 设置舵机上电/卸力状态（无MCU协议独有）
/// @param ID 舵机ID
/// @param load 1为上电，0为卸力
void ServoSetLoad(uint8_t ID, uint8_t load)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;                                                                  // 帧头
    data[1] = 0x55;                                                                  // 帧头
    data[2] = ID;                                                                    // ID号
    data[3] = get_servo_data_length(SERVO_LOAD_OR_UNLOAD_WRITE);                     // 数据长度
    data[4] = get_servo_command_value(SERVO_LOAD_OR_UNLOAD_WRITE);                   // 指令
    data[5] = load;                                                                  // 参数: 1上电，0卸力
    data[6] = CRC_GNERATOR(data, get_servo_data_length(SERVO_LOAD_OR_UNLOAD_WRITE)); // CRC校验
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_LOAD_OR_UNLOAD_WRITE) + 3);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/// @brief 读取舵机上电/卸力状态
/// @param ID 舵机ID
/// @note 需要通过接收回调获取结果
void ServoReadLoadStatus(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_LOAD_OR_UNLOAD_READ);
    data[4] = get_servo_command_value(SERVO_LOAD_OR_UNLOAD_READ);
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_LOAD_OR_UNLOAD_READ));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_LOAD_OR_UNLOAD_READ) + 3);
}

/*****************************延时控制函数***********************************/

/// @brief 设置舵机延时转动参数（需配合ServoStart使用）
/// @param ID 舵机ID
/// @param Angle 目标角度（0-1000对应0-240°）
/// @param Time 转动时间（0-30000ms）
void ServoMoveTimeWaitWrite(uint8_t ID, uint16_t Angle, uint16_t Time)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_MOVE_TIME_WAIT_WRITE);
    data[4] = get_servo_command_value(SERVO_MOVE_TIME_WAIT_WRITE);
    data[5] = (uint8_t)(Angle & 0xFF);
    data[6] = (uint8_t)(Angle >> 8);
    data[7] = (uint8_t)(Time & 0xFF);
    data[8] = (uint8_t)(Time >> 8);
    data[9] = CRC_GNERATOR(data, get_servo_data_length(SERVO_MOVE_TIME_WAIT_WRITE));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_MOVE_TIME_WAIT_WRITE) + 3);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/// @brief 启动舵机转动（配合ServoMoveTimeWaitWrite使用）
/// @param ID 舵机ID
void ServoStart(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_MOVE_START);
    data[4] = get_servo_command_value(SERVO_MOVE_START);
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_MOVE_START));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_MOVE_START) + 3);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/// @brief 立即停止舵机转动并保持当前位置
/// @param ID 舵机ID
void ServoStop(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_MOVE_STOP);
    data[4] = get_servo_command_value(SERVO_MOVE_STOP);
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_MOVE_STOP));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_MOVE_STOP) + 3);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/*****************************ID设置函数***********************************/

/// @brief 设置舵机ID（支持掉电保存）
/// @param ID 当前舵机ID
/// @param newID 新舵机ID（0-253）
/// @return SERVO_SUCCESS或SERVO_FAIL
ServoResult_t ServoSetID(uint8_t ID, uint8_t newID)
{
    if (newID > 253)
    {
        return SERVO_FAIL;
    }

    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_ID_WRITE);
    data[4] = get_servo_command_value(SERVO_ID_WRITE);
    data[5] = newID;
    data[6] = CRC_GNERATOR(data, get_servo_data_length(SERVO_ID_WRITE));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_ID_WRITE) + 3);
    vTaskDelay(pdMS_TO_TICKS(1));
    return SERVO_SUCCESS;
}

/// @brief 读取舵机ID
/// @param ID 舵机ID
/// @note 需要通过接收回调获取结果
void ServoReadID(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_ID_READ);
    data[4] = get_servo_command_value(SERVO_ID_READ);
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_ID_READ));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_ID_READ) + 3);
}

/*****************************角度偏差设置***********************************/

/// @brief 设置舵机角度偏差（不保存到内存）
/// @param ID 舵机ID
/// @param offset 偏差值（-125~125，对应-30°~30°）
void ServoSetAngleOffset(uint8_t ID, int8_t offset)
{
    // 限制范围
    if (offset < -125)
        offset = -125;
    if (offset > 125)
        offset = 125;

    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_ANGLE_OFFSET_ADJUST);
    data[4] = get_servo_command_value(SERVO_ANGLE_OFFSET_ADJUST);
    data[5] = (uint8_t)offset; // 有符号转无符号
    data[6] = CRC_GNERATOR(data, get_servo_data_length(SERVO_ANGLE_OFFSET_ADJUST));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_ANGLE_OFFSET_ADJUST) + 3);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/// @brief 保存角度偏差到内存（掉电保存）
/// @param ID 舵机ID
void ServoSaveAngleOffset(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_ANGLE_OFFSET_WRITE);
    data[4] = get_servo_command_value(SERVO_ANGLE_OFFSET_WRITE);
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_ANGLE_OFFSET_WRITE));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_ANGLE_OFFSET_WRITE) + 3);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/// @brief 读取舵机角度偏差
/// @param ID 舵机ID
/// @note 需要通过接收回调获取结果
void ServoReadAngleOffset(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_ANGLE_OFFSET_READ);
    data[4] = get_servo_command_value(SERVO_ANGLE_OFFSET_READ);
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_ANGLE_OFFSET_READ));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_ANGLE_OFFSET_READ) + 3);
}

/*****************************角度限制设置***********************************/

/// @brief 设置舵机角度限制（支持掉电保存）
/// @param ID 舵机ID
/// @param minAngle 最小角度（0-1000）
/// @param maxAngle 最大角度（0-1000）
/// @return SERVO_SUCCESS或SERVO_FAIL
ServoResult_t ServoSetAngleLimit(uint8_t ID, uint16_t minAngle, uint16_t maxAngle)
{
    if (minAngle > 1000 || maxAngle > 1000 || minAngle >= maxAngle)
    {
        return SERVO_FAIL;
    }

    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_ANGLE_LIMIT_WRITE);
    data[4] = get_servo_command_value(SERVO_ANGLE_LIMIT_WRITE);
    data[5] = (uint8_t)(minAngle & 0xFF);
    data[6] = (uint8_t)(minAngle >> 8);
    data[7] = (uint8_t)(maxAngle & 0xFF);
    data[8] = (uint8_t)(maxAngle >> 8);
    data[9] = CRC_GNERATOR(data, get_servo_data_length(SERVO_ANGLE_LIMIT_WRITE));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_ANGLE_LIMIT_WRITE) + 3);
    vTaskDelay(pdMS_TO_TICKS(1));
    return SERVO_SUCCESS;
}

/// @brief 读取舵机角度限制
/// @param ID 舵机ID
/// @note 需要通过接收回调获取结果
void ServoReadAngleLimit(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_ANGLE_LIMIT_READ);
    data[4] = get_servo_command_value(SERVO_ANGLE_LIMIT_READ);
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_ANGLE_LIMIT_READ));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_ANGLE_LIMIT_READ) + 3);
}

/*****************************电压限制设置***********************************/

/// @brief 设置舵机输入电压限制（支持掉电保存）
/// @param ID 舵机ID
/// @param minVin 最小电压（单位mV，4500-14000）
/// @param maxVin 最大电压（单位mV，4500-14000）
/// @return SERVO_SUCCESS或SERVO_FAIL
ServoResult_t ServoSetVinLimit(uint8_t ID, uint16_t minVin, uint16_t maxVin)
{
    if (minVin < 4500 || minVin > 14000 || maxVin < 4500 || maxVin > 14000 || minVin >= maxVin)
    {
        return SERVO_FAIL;
    }

    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_VIN_LIMIT_WRITE);
    data[4] = get_servo_command_value(SERVO_VIN_LIMIT_WRITE);
    data[5] = (uint8_t)(minVin & 0xFF);
    data[6] = (uint8_t)(minVin >> 8);
    data[7] = (uint8_t)(maxVin & 0xFF);
    data[8] = (uint8_t)(maxVin >> 8);
    data[9] = CRC_GNERATOR(data, get_servo_data_length(SERVO_VIN_LIMIT_WRITE));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_VIN_LIMIT_WRITE) + 3);
    vTaskDelay(pdMS_TO_TICKS(1));
    return SERVO_SUCCESS;
}

/// @brief 读取舵机电压限制
/// @param ID 舵机ID
/// @note 需要通过接收回调获取结果
void ServoReadVinLimit(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_VIN_LIMIT_READ);
    data[4] = get_servo_command_value(SERVO_VIN_LIMIT_READ);
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_VIN_LIMIT_READ));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_VIN_LIMIT_READ) + 3);
}

/*****************************温度限制设置***********************************/

/// @brief 设置舵机最高温度限制（支持掉电保存）
/// @param ID 舵机ID
/// @param maxTemp 最高温度（50-100℃）
/// @return SERVO_SUCCESS或SERVO_FAIL
ServoResult_t ServoSetTempLimit(uint8_t ID, uint8_t maxTemp)
{
    if (maxTemp < 50 || maxTemp > 100)
    {
        return SERVO_FAIL;
    }

    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_TEMP_MAX_LIMIT_WRITE);
    data[4] = get_servo_command_value(SERVO_TEMP_MAX_LIMIT_WRITE);
    data[5] = maxTemp;
    data[6] = CRC_GNERATOR(data, get_servo_data_length(SERVO_TEMP_MAX_LIMIT_WRITE));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_TEMP_MAX_LIMIT_WRITE) + 3);
    vTaskDelay(pdMS_TO_TICKS(1));
    return SERVO_SUCCESS;
}

/// @brief 读取舵机温度限制
/// @param ID 舵机ID
/// @note 需要通过接收回调获取结果
void ServoReadTempLimit(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_TEMP_MAX_LIMIT_READ);
    data[4] = get_servo_command_value(SERVO_TEMP_MAX_LIMIT_READ);
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_TEMP_MAX_LIMIT_READ));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_TEMP_MAX_LIMIT_READ) + 3);
}

/*****************************状态读取函数***********************************/

/// @brief 读取舵机实时温度
/// @param ID 舵机ID
/// @note 需要通过接收回调获取结果
void ServoReadTemp(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_TEMP_READ);
    data[4] = get_servo_command_value(SERVO_TEMP_READ);
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_TEMP_READ));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_TEMP_READ) + 3);
}

/// @brief 读取舵机输入电压
/// @param ID 舵机ID
/// @note 需要通过接收回调获取结果
void ServoReadVin(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_VIN_READ);
    data[4] = get_servo_command_value(SERVO_VIN_READ);
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_VIN_READ));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_VIN_READ) + 3);
}

/*****************************工作模式设置***********************************/

/// @brief 设置舵机工作模式和速度
/// @param ID 舵机ID
/// @param servoMode 舵机模式（0=位置控制，1=电机控制）
/// @param rotateMode 转动模式（0=固定占空比，1=固定转速）
/// @param speed 转动速度（占空比模式-1000~1000，转速模式-50~50）
void ServoSetModeAndSpeed(uint8_t ID, uint8_t servoMode, uint8_t rotateMode, int16_t speed)
{
    // 速度范围限制
    if (rotateMode == 0) // 固定占空比模式
    {
        if (speed < -1000)
            speed = -1000;
        if (speed > 1000)
            speed = 1000;
    }
    else // 固定转速模式
    {
        if (speed < -50)
            speed = -50;
        if (speed > 50)
            speed = 50;
    }

    // 将有符号数转换为无符号数（补码形式）
    uint16_t speedUnsigned = (uint16_t)speed;

    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_OR_MOTOR_MODE_WRITE);
    data[4] = get_servo_command_value(SERVO_OR_MOTOR_MODE_WRITE);
    data[5] = servoMode;
    data[6] = rotateMode;
    data[7] = (uint8_t)(speedUnsigned & 0xFF);
    data[8] = (uint8_t)(speedUnsigned >> 8);
    data[9] = CRC_GNERATOR(data, get_servo_data_length(SERVO_OR_MOTOR_MODE_WRITE));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_OR_MOTOR_MODE_WRITE) + 3);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/// @brief 读取舵机工作模式和速度
/// @param ID 舵机ID
/// @note 需要通过接收回调获取结果
void ServoReadModeAndSpeed(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_OR_MOTOR_MODE_READ);
    data[4] = get_servo_command_value(SERVO_OR_MOTOR_MODE_READ);
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_OR_MOTOR_MODE_READ));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_OR_MOTOR_MODE_READ) + 3);
}

/*****************************LED控制函数***********************************/

/// @brief 设置舵机LED灯状态（支持掉电保存）
/// @param ID 舵机ID
/// @param ledOn 0=常亮，1=常灭
void ServoSetLED(uint8_t ID, uint8_t ledOn)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_LED_CTRL_WRITE);
    data[4] = get_servo_command_value(SERVO_LED_CTRL_WRITE);
    data[5] = ledOn ? 1 : 0;
    data[6] = CRC_GNERATOR(data, get_servo_data_length(SERVO_LED_CTRL_WRITE));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_LED_CTRL_WRITE) + 3);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/// @brief 读取舵机LED状态
/// @param ID 舵机ID
/// @note 需要通过接收回调获取结果
void ServoReadLED(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_LED_CTRL_READ);
    data[4] = get_servo_command_value(SERVO_LED_CTRL_READ);
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_LED_CTRL_READ));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_LED_CTRL_READ) + 3);
}

/// @brief 设置LED报警故障类型
/// @param ID 舵机ID
/// @param alarmCode 报警代码（使用ServoAlarmCode_t枚举）
void ServoSetLEDAlarm(uint8_t ID, ServoAlarmCode_t alarmCode)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_LED_ERROR_WRITE);
    data[4] = get_servo_command_value(SERVO_LED_ERROR_WRITE);
    data[5] = (uint8_t)alarmCode;
    data[6] = CRC_GNERATOR(data, get_servo_data_length(SERVO_LED_ERROR_WRITE));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_LED_ERROR_WRITE) + 3);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/// @brief 读取LED报警状态
/// @param ID 舵机ID
/// @note 需要通过接收回调获取结果
void ServoReadLEDAlarm(uint8_t ID)
{
    uint8_t data[16] = {0x00};
    data[0] = 0x55;
    data[1] = 0x55;
    data[2] = ID;
    data[3] = get_servo_data_length(SERVO_LED_ERROR_READ);
    data[4] = get_servo_command_value(SERVO_LED_ERROR_READ);
    data[5] = CRC_GNERATOR(data, get_servo_data_length(SERVO_LED_ERROR_READ));
    Servo_UART_Send((const uint8_t *)data, get_servo_data_length(SERVO_LED_ERROR_READ) + 3);
}

#endif /* other_mcu_forcing */
