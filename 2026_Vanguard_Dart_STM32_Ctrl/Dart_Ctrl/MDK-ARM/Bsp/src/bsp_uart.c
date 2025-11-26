#include "bsp_uart.h"
#include <string.h>
#include <stdbool.h>

/* 内部缓冲区定义（用户无需关心） */
static UartTxRingBuffer g_uart_tx_buffers[BSP_UART_MAX];
static UartRxRingBuffer g_uart_rx_buffers[BSP_UART_MAX];
static DataBuffer g_data_buffers[BSP_UART_MAX]; // DataBuffer实例（用于指针操作）
static uint8_t g_temp_tx_buffer[32]; // 临时发送缓冲区

/* 内部函数声明 */
static void TxRingBuffer_Init(UartTxRingBuffer *rb, UART_HandleTypeDef *huart);
static void RxRingBuffer_Init(UartRxRingBuffer *rb, UART_HandleTypeDef *huart);
static void StartTransmit(UartTxRingBuffer *rb);
static uint16_t TxRingBuffer_Write(UartTxRingBuffer *rb, const uint8_t *data, uint16_t len);
static uint16_t TxRingBuffer_Read(UartTxRingBuffer *rb, uint8_t *data, uint16_t len);
static bool RxRingBuffer_WriteByte(UartRxRingBuffer *rb, uint8_t byte);
static uint16_t RxRingBuffer_Read(UartRxRingBuffer *rb, uint8_t *data, uint16_t len);
static UART_HandleTypeDef *GetUartHandle(BSP_UART_NUM_e uart_num);
static BSP_UART_NUM_e GetUartNum(UART_HandleTypeDef *huart);
static void ParseProtocol(UartRxRingBuffer *rb, uint8_t byte);
static void ResetParser(UartRxRingBuffer *rb);

/**
 * @brief CRC校验生成（公开函数，供所有模块使用）
 * @param data 数据指针
 * @param length 数据长度
 * @return CRC校验码
 */
uint8_t UART_Calculate_CRC(uint8_t *data, uint8_t length)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        sum += data[i];
    }
    return (uint8_t)((~sum) & 0xFF);
}

/**
 * @brief 重置协议解析器
 */
static void ResetParser(UartRxRingBuffer *rb)
{
    rb->parse_state = PARSE_HEADER;
    rb->parse_index = 0;
    memset(&rb->servo_packet, 0, sizeof(ServoPacket_t));
    memset(&rb->dart_packet, 0, sizeof(DartPacket_t));
}

/**
 * @brief 协议解析状态机
 * @param rb 接收缓冲区指针
 * @param byte 接收到的字节
 */
static void ParseProtocol(UartRxRingBuffer *rb, uint8_t byte)
{
    if (rb->protocol_type == PROTOCOL_SERVO)
    {
        // 舵机协议解析：0x55 0x55 | ID | Length | Cmd | Params | CRC
        switch (rb->parse_state)
        {
        case PARSE_HEADER:
            if (rb->parse_index == 0)
            {
                if (byte == 0x55)
                {
                    rb->servo_packet.header[0] = byte;
                    rb->parse_index = 1;
                }
            }
            else if (rb->parse_index == 1)
            {
                if (byte == 0x55)
                {
                    rb->servo_packet.header[1] = byte;
                    rb->parse_state = PARSE_SERVO_ID;
                    rb->parse_index = 0;
                }
                else
                {
                    ResetParser(rb);
                }
            }
            break;

        case PARSE_SERVO_ID:
            rb->servo_packet.id = byte;
            rb->parse_state = PARSE_SERVO_LENGTH;
            break;

        case PARSE_SERVO_LENGTH:
            rb->servo_packet.length = byte;
            if (byte >= 3 && byte <= 11) // 合法长度范围
            {
                rb->parse_state = PARSE_SERVO_CMD;
            }
            else
            {
                ResetParser(rb);
            }
            break;

        case PARSE_SERVO_CMD:
            rb->servo_packet.cmd = byte;
            rb->servo_packet.param_len = rb->servo_packet.length - 3;
            if (rb->servo_packet.param_len > 0)
            {
                rb->parse_state = PARSE_SERVO_PARAMS;
                rb->parse_index = 0;
            }
            else
            {
                rb->parse_state = PARSE_SERVO_CRC;
            }
            break;

        case PARSE_SERVO_PARAMS:
            if (rb->parse_index < rb->servo_packet.param_len && rb->parse_index < 8)
            {
                rb->servo_packet.params[rb->parse_index++] = byte;
                if (rb->parse_index >= rb->servo_packet.param_len)
                {
                    rb->parse_state = PARSE_SERVO_CRC;
                }
            }
            else
            {
                ResetParser(rb);
            }
            break;

        case PARSE_SERVO_CRC:
        {
            rb->servo_packet.checksum = byte;
            // 验证CRC（从ID开始到参数结束）
            uint8_t temp_buf[16];
            temp_buf[0] = rb->servo_packet.id;
            temp_buf[1] = rb->servo_packet.length;
            temp_buf[2] = rb->servo_packet.cmd;
            for (uint8_t i = 0; i < rb->servo_packet.param_len; i++)
            {
                temp_buf[3 + i] = rb->servo_packet.params[i];
            }
            uint8_t calc_crc = UART_Calculate_CRC(temp_buf, 3 + rb->servo_packet.param_len);

            if (calc_crc == byte)
            {
                rb->servo_packet.is_valid = true;
                rb->packet_ready = true;
            }
            ResetParser(rb);
            break;
        }

        default:
            ResetParser(rb);
            break;
        }
    }
    else // PROTOCOL_DART
    {
        // DART协议解析：DART | Data... | CRC
        switch (rb->parse_state)
        {
        case PARSE_HEADER:
            if (rb->parse_index < 4)
            {
                const uint8_t dart_header[4] = {'D', 'A', 'R', 'T'};
                if (byte == dart_header[rb->parse_index])
                {
                    rb->dart_packet.header[rb->parse_index] = byte;
                    rb->parse_index++;
                    if (rb->parse_index == 4)
                    {
                        rb->parse_state = PARSE_DART_DATA;
                        rb->parse_index = 0;
                    }
                }
                else
                {
                    ResetParser(rb);
                }
            }
            break;

        case PARSE_DART_DATA:
            // 读取数据直到遇到CRC（假设最大64字节数据）
            if (rb->parse_index < 64)
            {
                rb->dart_packet.data[rb->parse_index++] = byte;
                // 简单处理：假设固定长度或者有结束标志
                // 这里需要根据实际协议定义来判断何时结束
                // 暂时假设接收到特定长度后进入CRC状态
                // TODO: 根据实际DART协议定义修改
            }
            break;

        case PARSE_DART_CRC:
        {
            rb->dart_packet.checksum = byte;
            rb->dart_packet.data_len = rb->parse_index;
            // 验证CRC
            uint8_t temp_buf[68];
            memcpy(temp_buf, rb->dart_packet.header, 4);
            memcpy(temp_buf + 4, rb->dart_packet.data, rb->dart_packet.data_len);
            uint8_t calc_crc = UART_Calculate_CRC(temp_buf, 4 + rb->dart_packet.data_len);

            if (calc_crc == byte)
            {
                rb->dart_packet.is_valid = true;
                rb->packet_ready = true;
            }
            ResetParser(rb);
            break;
        }

        default:
            ResetParser(rb);
            break;
        }
    }
}

/**
 * @brief 获取UART句柄
 */
static UART_HandleTypeDef *GetUartHandle(BSP_UART_NUM_e uart_num)
{
    switch (uart_num)
    {
    case BSP_UART1:
        return &huart1;
    case BSP_UART6:
        return &huart6;
    default:
        return NULL;
    }
}

/**
 * @brief 根据UART句柄获取UART编号
 */
static BSP_UART_NUM_e GetUartNum(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
        return BSP_UART1;
    if (huart == &huart6)
        return BSP_UART6;
    return BSP_UART_MAX;
}

/**
 * @brief 发送环形缓冲区初始化
 */
static void TxRingBuffer_Init(UartTxRingBuffer *rb, UART_HandleTypeDef *huart)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    rb->isSending = false;
    rb->huart = huart;
    memset(rb->buffer, 0, UART_TX_BUFFER_SIZE);
}

/**
 * @brief 接收环形缓冲区初始化
 */
static void RxRingBuffer_Init(UartRxRingBuffer *rb, UART_HandleTypeDef *huart)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    rb->isReceiving = false;
    rb->overflowFlag = false;
    rb->rxByte = 0;
    rb->huart = huart;
    rb->protocol_type = PROTOCOL_SERVO; // 默认舵机协议
    rb->packet_ready = false;
    ResetParser(rb);
    memset(rb->buffer, 0, UART_RX_BUFFER_SIZE);
}

/**
 * @brief 向发送缓冲区写入数据
 */
static uint16_t TxRingBuffer_Write(UartTxRingBuffer *rb, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
        return 0;

    uint16_t freeSpace = UART_TX_BUFFER_SIZE - rb->count;
    uint16_t writeLen = (len > freeSpace) ? freeSpace : len;

    for (uint16_t i = 0; i < writeLen; i++)
    {
        rb->buffer[rb->head] = data[i];
        rb->head = (rb->head + 1) % UART_TX_BUFFER_SIZE;
        rb->count++;
    }

    return writeLen;
}

/**
 * @brief 从发送缓冲区读取数据
 */
static uint16_t TxRingBuffer_Read(UartTxRingBuffer *rb, uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
        return 0;

    uint16_t readLen = (len > rb->count) ? rb->count : len;

    for (uint16_t i = 0; i < readLen; i++)
    {
        data[i] = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % UART_TX_BUFFER_SIZE;
        rb->count--;
    }

    return readLen;
}

/**
 * @brief 启动发送
 */
static void StartTransmit(UartTxRingBuffer *rb)
{
    if (rb->isSending || rb->count == 0)
        return;

    // 每次最多发送32字节
    uint16_t sendLen = (rb->count > sizeof(g_temp_tx_buffer)) ? sizeof(g_temp_tx_buffer) : rb->count;
    sendLen = TxRingBuffer_Read(rb, g_temp_tx_buffer, sendLen);

    if (sendLen > 0)
    {
        rb->isSending = true;
        HAL_UART_Transmit_IT(rb->huart, g_temp_tx_buffer, sendLen);
    }
}

/**
 * @brief 向接收缓冲区写入一个字节
 */
static bool RxRingBuffer_WriteByte(UartRxRingBuffer *rb, uint8_t byte)
{
    if (rb->count >= UART_RX_BUFFER_SIZE)
    {
        rb->overflowFlag = true;
        return false;
    }

    rb->buffer[rb->head] = byte;
    rb->head = (rb->head + 1) % UART_RX_BUFFER_SIZE;
    rb->count++;
    return true;
}

/**
 * @brief 从接收缓冲区读取数据
 */
static uint16_t RxRingBuffer_Read(UartRxRingBuffer *rb, uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
        return 0;

    uint16_t readLen = (len > rb->count) ? rb->count : len;

    for (uint16_t i = 0; i < readLen; i++)
    {
        data[i] = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % UART_RX_BUFFER_SIZE;
        rb->count--;
    }

    return readLen;
}

/* ========== API接口实现 ========== */

/**
 * @brief 初始化BSP UART模块
 */
void BSP_UART_Init(void)
{
    for (uint8_t i = 0; i < BSP_UART_MAX; i++)
    {
        UART_HandleTypeDef *huart = GetUartHandle((BSP_UART_NUM_e)i);
        if (huart != NULL)
        {
            TxRingBuffer_Init(&g_uart_tx_buffers[i], huart);
            RxRingBuffer_Init(&g_uart_rx_buffers[i], huart);
            
            // 初始化DataBuffer
            DataBuffer *db = &g_data_buffers[i];
            memset(db->RxDataBuffer, 0, UART_RX_BUFFER_SIZE);
            db->ReadPtr = db->RxDataBuffer;   // 读指针指向缓冲区起始
            db->WritePtr = db->RxDataBuffer;  // 写指针指向缓冲区起始
            db->DataLen = 0;
            db->OverflowFlag = false;
        }
    }
}

/**
 * @brief 设置协议类型
 * @param uart_num UART编号
 * @param is_servo_mode true=舵机协议, false=DART协议
 */
void UART_SetProtocol(BSP_UART_NUM_e uart_num, bool is_servo_mode)
{
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num];
    rb->protocol_type = is_servo_mode ? PROTOCOL_SERVO : PROTOCOL_DART;
    ResetParser(rb);
}

/**
 * @brief 发送数据
 */
uint16_t UART_Send(BSP_UART_NUM_e uart_num, const uint8_t *data, uint16_t len)
{
    if (uart_num >= BSP_UART_MAX || data == NULL || len == 0)
        return 0;

    UartTxRingBuffer *rb = &g_uart_tx_buffers[uart_num];
    uint16_t written = TxRingBuffer_Write(rb, data, len);

    if (!rb->isSending)
    {
        StartTransmit(rb);
    }

    return written;
}

/**
 * @brief 发送字符串
 */
uint16_t UART_SendString(BSP_UART_NUM_e uart_num, const char *str)
{
    if (str == NULL)
        return 0;

    return UART_Send(uart_num, (const uint8_t *)str, strlen(str));
}

/**
 * @brief 启动接收
 */
void UART_StartRx(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num];
    rb->isReceiving = true;
    HAL_UART_Receive_IT(rb->huart, &rb->rxByte, 1);
}

/**
 * @brief 读取接收数据（原始数据，读取后移动ReadPtr）
 */
uint16_t UART_Read(BSP_UART_NUM_e uart_num, uint8_t *data, uint16_t len)
{
    if (uart_num >= BSP_UART_MAX)
        return 0;

    DataBuffer *db = &g_data_buffers[uart_num];
    
    // 从环形缓冲区读取数据
    uint16_t readLen = RxRingBuffer_Read(&g_uart_rx_buffers[uart_num], data, len);
    
    // 同时移动DataBuffer的ReadPtr（用户读取数据时移动读指针）
    if (readLen > 0 && db->ReadPtr + readLen <= db->WritePtr)
    {
        db->ReadPtr += readLen;
        db->DataLen = db->WritePtr - db->ReadPtr; // 更新剩余数据长度
    }
    
    return readLen;
}

/**
 * @brief 获取接收缓冲区数据量
 */
uint16_t UART_GetRxCount(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return 0;

    return g_uart_rx_buffers[uart_num].count;
}

/**
 * @brief 检查是否有接收数据
 */
bool UART_HasData(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return false;

    return (g_uart_rx_buffers[uart_num].count > 0);
}

/**
 * @brief 清空接收缓冲区
 */
void UART_ClearRx(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num];
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    rb->overflowFlag = false;
    ResetParser(rb);
}

/* ========== 协议解析API实现 ========== */

/**
 * @brief 检查是否有完整的数据包
 */
bool UART_HasPacket(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return false;

    return g_uart_rx_buffers[uart_num].packet_ready;
}

/**
 * @brief 获取舵机协议数据包（读取数据包时移动ReadPtr）
 */
bool UART_GetServoPacket(BSP_UART_NUM_e uart_num, ServoPacket_t *packet)
{
    if (uart_num >= BSP_UART_MAX || packet == NULL)
        return false;

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num];
    DataBuffer *db = &g_data_buffers[uart_num];

    if (rb->packet_ready && rb->protocol_type == PROTOCOL_SERVO && rb->servo_packet.is_valid)
    {
        memcpy(packet, &rb->servo_packet, sizeof(ServoPacket_t));
        
        // 读取数据包后，移动ReadPtr（数据包总长度 = 2字节包头 + 1字节ID + 1字节长度 + 长度字节内容）
        uint16_t packet_total_len = 2 + 1 + 1 + rb->servo_packet.length;
        if (db->ReadPtr + packet_total_len <= db->WritePtr)
        {
            db->ReadPtr += packet_total_len;
            db->DataLen = db->WritePtr - db->ReadPtr; // 更新剩余数据长度
        }
        
        return true;
    }

    return false;
}

/**
 * @brief 获取DART协议数据包（读取数据包时移动ReadPtr）
 */
bool UART_GetDartPacket(BSP_UART_NUM_e uart_num, DartPacket_t *packet)
{
    if (uart_num >= BSP_UART_MAX || packet == NULL)
        return false;

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num];
    DataBuffer *db = &g_data_buffers[uart_num];

    if (rb->packet_ready && rb->protocol_type == PROTOCOL_DART && rb->dart_packet.is_valid)
    {
        memcpy(packet, &rb->dart_packet, sizeof(DartPacket_t));
        
        // 读取数据包后，移动ReadPtr（数据包总长度 = 4字节包头 + 数据长度 + 1字节CRC）
        uint16_t packet_total_len = 4 + rb->dart_packet.data_len + 1;
        if (db->ReadPtr + packet_total_len <= db->WritePtr)
        {
            db->ReadPtr += packet_total_len;
            db->DataLen = db->WritePtr - db->ReadPtr; // 更新剩余数据长度
        }
        
        return true;
    }

    return false;
}

/**
 * @brief 清除数据包标志
 */
void UART_ClearPacket(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num];
    rb->packet_ready = false;
    rb->servo_packet.is_valid = false;
    rb->dart_packet.is_valid = false;
}

/* ========== HAL回调函数实现 ========== */

/**
 * @brief UART发送完成回调
 */
void BSP_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_NUM_e uart_num = GetUartNum(huart);
    if (uart_num >= BSP_UART_MAX)
        return;

    UartTxRingBuffer *rb = &g_uart_tx_buffers[uart_num];
    rb->isSending = false;

    if (rb->count > 0)
    {
        StartTransmit(rb);
    }
}

/**
 * @brief UART接收完成回调（包含协议解析和DataBuffer写指针移动）
 */
void BSP_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_NUM_e uart_num = GetUartNum(huart);
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num];
    DataBuffer *db = &g_data_buffers[uart_num];

    // 将字节写入环形缓冲区（用于原始数据读取）
    RxRingBuffer_WriteByte(rb, rb->rxByte);

    // 同时写入DataBuffer并移动WritePtr（接收中断时移动写指针）
    if (db->WritePtr < db->RxDataBuffer + UART_RX_BUFFER_SIZE)
    {
        *db->WritePtr = rb->rxByte;
        db->WritePtr++;
        db->DataLen = db->WritePtr - db->ReadPtr; // 更新数据长度
    }
    else
    {
        // 缓冲区满，设置溢出标志
        db->OverflowFlag = true;
    }

    // 协议解析
    ParseProtocol(rb, rb->rxByte);

    // 继续接收下一个字节
    if (rb->isReceiving)
    {
        HAL_UART_Receive_IT(rb->huart, &rb->rxByte, 1);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_TxCpltCallback(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_RxCpltCallback(huart);
}
