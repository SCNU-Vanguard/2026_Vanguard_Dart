/**
 * @file UartModule.c
 * @brief UART模块用户接口源文件
 * @note  本模块提供数据包获取和发送的用户级API
 *        是对底层 UartProtocol 和 bsp_uart 的封装
 * 
 * 架构层次：
 *   用户应用层
 *       ↓
 *   UartModule.c (本文件) - 用户级数据包收发API
 *       ↓
 *   UartProtocol.c - 协议解析内部逻辑
 *       ↓
 *   bsp_uart.c - 底层硬件驱动
 */

#include "UartModule.h"
#include "UartProtocol.h"
#include "bsp_uart.h"
#include <string.h>

/* ========== 用户级数据包接口实现 ========== */

/**
 * @brief 检查是否有完整的舵机数据包
 */
bool UartModule_HasServoPacket(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return false;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    if (rb == NULL)
        return false;

    bool is_servo_protocol = (rb->protocol_type == PROTOCOL_SERVO_MCU ||
                              rb->protocol_type == PROTOCOL_SERVO_NO_MCU);

    return (Protocol_HasPacket(uart_num) && is_servo_protocol);
}

/**
 * @brief 检查是否有完整的DART数据包
 */
bool UartModule_HasDartPacket(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return false;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    if (rb == NULL)
        return false;

    return (Protocol_HasPacket(uart_num) && rb->protocol_type == PROTOCOL_DART);
}

/**
 * @brief 获取舵机协议数据包
 */
bool UartModule_GetServoPacket(BSP_UART_NUM_e uart_num, ServoPacket_t *packet)
{
    return Protocol_GetServoPacket(uart_num, packet);
}

/**
 * @brief 获取DART协议数据包
 */
bool UartModule_GetDartPacket(BSP_UART_NUM_e uart_num, DartPacket_t *packet)
{
    return Protocol_GetDartPacket(uart_num, packet);
}

/**
 * @brief 清除数据包标志
 */
void UartModule_ClearPacket(BSP_UART_NUM_e uart_num)
{
    Protocol_ClearPacket(uart_num);
}

/* ========== 用户级发送接口实现 ========== */

/**
 * @brief 发送舵机命令数据包
 * @note 根据协议类型自动构建数据包格式：
 *       - SERVO_MCU: 0x55 0x55 | Length | Cmd | Params（无CRC）
 *       - SERVO_NO_MCU: 0x55 0x55 | ID | Length | Cmd | Params | CRC（有CRC）
 */
bool UartModule_SendServoCmd(BSP_UART_NUM_e uart_num, uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_len)
{
    if (uart_num >= BSP_UART_MAX || param_len > 8)
        return false;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    if (rb == NULL)
        return false;

    uint8_t tx_buf[16];
    uint16_t tx_len = 0;

    // 帧头
    tx_buf[tx_len++] = 0x55;
    tx_buf[tx_len++] = 0x55;

    if (rb->protocol_type == PROTOCOL_SERVO_MCU)
    {
        // 有MCU控制板协议（无CRC）
        // Length = Cmd(1) + Params + Length本身(1) = param_len + 2
        tx_buf[tx_len++] = param_len + 2;
        tx_buf[tx_len++] = cmd;

        // 参数
        if (params != NULL && param_len > 0)
        {
            memcpy(&tx_buf[tx_len], params, param_len);
            tx_len += param_len;
        }
    }
    else // PROTOCOL_SERVO_NO_MCU
    {
        // 无MCU驱动板协议（有CRC）
        tx_buf[tx_len++] = id;
        // Length = Cmd(1) + Params + CRC(1) = param_len + 3
        uint8_t length = param_len + 3;
        tx_buf[tx_len++] = length;
        tx_buf[tx_len++] = cmd;

        // 参数
        if (params != NULL && param_len > 0)
        {
            memcpy(&tx_buf[tx_len], params, param_len);
            tx_len += param_len;
        }

        // 计算CRC（从ID开始到参数结束）
        uint8_t crc_buf[16];
        crc_buf[0] = id;
        crc_buf[1] = length;
        crc_buf[2] = cmd;
        if (params != NULL && param_len > 0)
        {
            memcpy(&crc_buf[3], params, param_len);
        }
        tx_buf[tx_len++] = Protocol_Calculate_CRC(crc_buf, 3 + param_len);
    }

    // 发送
    uint16_t sent = UART_Send(uart_num, tx_buf, tx_len);
    return (sent == tx_len);
}

/**
 * @brief 发送DART数据包
 * @note 数据包格式：DART | Length | Data... | CRC
 */
bool UartModule_SendDartPacket(BSP_UART_NUM_e uart_num, uint8_t *data, uint16_t data_len)
{
    if (uart_num >= BSP_UART_MAX || data == NULL || data_len > 64)
        return false;

    uint8_t tx_buf[72];
    uint16_t tx_len = 0;

    // 帧头 "DART"
    tx_buf[tx_len++] = 'D';
    tx_buf[tx_len++] = 'A';
    tx_buf[tx_len++] = 'R';
    tx_buf[tx_len++] = 'T';

    // 数据长度
    tx_buf[tx_len++] = (uint8_t)data_len;

    // 数据内容
    memcpy(&tx_buf[tx_len], data, data_len);
    tx_len += data_len;

    // CRC校验
    tx_buf[tx_len++] = Protocol_Calculate_CRC(data, data_len);

    // 发送
    uint16_t sent = UART_Send(uart_num, tx_buf, tx_len);
    return (sent == tx_len);
}

/**
 * @brief 发送原始数据
 */
uint16_t UartModule_SendRaw(BSP_UART_NUM_e uart_num, const uint8_t *data, uint16_t len)
{
    return UART_Send(uart_num, data, len);
}

/* ========== 用户级设置接口实现 ========== */

/**
 * @brief 设置UART协议类型
 */
void UartModule_SetProtocol(BSP_UART_NUM_e uart_num, PROTOCOL_TYPE_e protocol_type)
{
    Protocol_SetType(uart_num, protocol_type);
}

/**
 * @brief 获取有效帧数量
 */
uint8_t UartModule_GetFrameCount(BSP_UART_NUM_e uart_num)
{
    return Protocol_GetFrameCount(uart_num);
}
