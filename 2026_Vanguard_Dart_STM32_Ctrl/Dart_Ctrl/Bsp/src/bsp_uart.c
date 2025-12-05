#include "bsp_uart.h"
#include <string.h>
#include <stdbool.h>

// todo：解算舵机格式需要加上一个长度传输函数，这样才可以保证这个长度是正确的，方便解算
// todo: 后续应该使用其他手段比如上电或者信号量进行初始化与上位机的通信，从而保证稳定
// todo：要读取一下帧内固定的数据长度位

/* 内部缓冲区定义（用户无需关心） */
static UartTxRingBuffer g_uart_tx_buffers[BSP_UART_MAX];
static UartRxRingBuffer g_uart_rx_buffers[BSP_UART_MAX];
static DataBuffer g_data_buffers[BSP_UART_MAX]; // DataBuffer实例（用于指针操作）
static uint8_t g_temp_tx_buffer[32];            // 临时发送缓冲区

/* 内部函数声明 */
static void TxRingBuffer_Init(UartTxRingBuffer *rb, UART_HandleTypeDef *huart);
static void RxRingBuffer_Init(UartRxRingBuffer *rb, UART_HandleTypeDef *huart);
static void DataBuffer_Init(DataBuffer *db);
static void StartTransmit(UartTxRingBuffer *rb);
static uint16_t TxRingBuffer_Write(UartTxRingBuffer *rb, const uint8_t *data, uint16_t len);
static uint16_t TxRingBuffer_Read(UartTxRingBuffer *rb, uint8_t *data, uint16_t len);
static bool RxRingBuffer_WriteByte(UartRxRingBuffer *rb, uint8_t byte);
static uint16_t RxRingBuffer_Read(UartRxRingBuffer *rb, uint8_t *data, uint16_t len);
static UART_HandleTypeDef *GetUartHandle(BSP_UART_NUM_e uart_num);
static BSP_UART_NUM_e GetUartNum(UART_HandleTypeDef *huart);
static void ParseProtocol(UartRxRingBuffer *rb, uint8_t byte);
static void ResetParser(UartRxRingBuffer *rb);
static void DetectFrameHeader(DataBuffer *db, UartRxRingBuffer *rb, uint8_t byte, uint8_t currentPos);
static void RecordFrameTail(DataBuffer *db, UartRxRingBuffer *rb, uint8_t tailPos);
static void InvalidateOverwrittenFrames(DataBuffer *db, UartRxRingBuffer *rb, uint8_t overwritePos);
static bool IsPositionInRange(uint8_t pos, uint8_t start, uint8_t end);
static void MoveReadIndexToNextFrame(DataBuffer *db, UartRxRingBuffer *rb);

const uint8_t dart_header[4] = {'D', 'A', 'R', 'T'};

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
    return (uint8_t)(~sum);
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
 * @todo 协议解析状态机解析舵机的时候要加上这个长度识别,注意这个数据接收逻辑似乎有问题
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

        // 这里似乎逻辑并不正确
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
        // DART协议解析：DART | Length |Data... | CRC
        switch (rb->parse_state)
        {
        case PARSE_HEADER:
            if (rb->parse_index < 4)
            {
                if (byte == dart_header[rb->parse_index])
                {
                    rb->dart_packet.header[rb->parse_index] = byte;
                    rb->parse_index++;
                    if (rb->parse_index == 4)
                    {
                        rb->parse_state = PARSE_DART_LENGTH;
                        rb->parse_index = 0;
                    }
                }
                else
                {
                    ResetParser(rb);
                }
            }
            break;

        // 解析数据长度
        case PARSE_DART_LENGTH:
            rb->dart_packet.data_len = byte;
            rb->parse_state = PARSE_DART_DATA;
            break;

            //     rb->servo_packet.cmd = byte;
            //     rb->servo_packet.param_len = rb->servo_packet.length - 3;
            //     if (rb->servo_packet.param_len > 0)
            //     {
            //         rb->parse_state = PARSE_SERVO_PARAMS;
            //         rb->parse_index = 0;
            //     }
            //     else
            //     {
            //         rb->parse_state = PARSE_SERVO_CRC; // 这里有问题，要重新写逻辑
            //     }

        case PARSE_DART_DATA:
            // 读取数据直到遇到CRC（假设最大64字节数据）
            if (rb->parse_index < rb->dart_packet.data_len)
            {
                rb->dart_packet.data[rb->parse_index++] = byte;
                // 简单处理：假设固定长度或者有结束标志
                // 这里需要根据实际协议定义来判断何时结束
                // 暂时假设接收到特定长度后进入CRC状态
                // TODO: 根据实际DA议定义修改RT协
            }
            break;

        case PARSE_DART_CRC:
        {
            rb->dart_packet.checksum = byte;
            // 验证CRC
            uint8_t temp_buf[68];
            memcpy(temp_buf, rb->dart_packet.data, rb->dart_packet.data_len);
            uint8_t calc_crc = UART_Calculate_CRC(temp_buf, rb->dart_packet.data_len);

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
    case BSP_UART3:
        return &huart3;
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
    if (huart == &huart3)
        return BSP_UART3;
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
 * @brief DataBuffer初始化（包含帧索引缓冲区）
 */
static void DataBuffer_Init(DataBuffer *db)
{
    // 初始化数据缓冲区
    memset(db->RxDataBuffer, 0, UART_RX_BUFFER_SIZE);
    db->ReadIndex = 0;
    db->WriteIndex = 0;
    db->OverflowFlag = false;
    db->WrapAroundFlag = false; // 环绕标志初始化

    // 初始化帧头识别状态机
    db->HeaderMatchCount = 0;
    db->PendingHeaderPos = 0;
    db->HeaderCrossWrap = false; // 帧头跨边界标志初始化

    // 初始化帧索引缓冲区 - DART协议
    memset(db->FrameIndex.DartHeaderIndex, 0, DART_FRAME_INDEX_SIZE);
    memset(db->FrameIndex.DartTailIndex, 0, DART_FRAME_INDEX_SIZE);
    db->FrameIndex.DartFrameCount = 0;
    db->FrameIndex.DartWritePtr = 0;
    db->FrameIndex.DartReadPtr = 0;
    for (uint8_t i = 0; i < DART_FRAME_INDEX_SIZE; i++)
    {
        db->FrameIndex.DartFrameValid[i] = false;
    }

    // 初始化帧索引缓冲区 - Servo协议
    memset(db->FrameIndex.ServoHeaderIndex, 0, SERVO_FRAME_INDEX_SIZE);
    memset(db->FrameIndex.ServoTailIndex, 0, SERVO_FRAME_INDEX_SIZE);
    db->FrameIndex.ServoFrameCount = 0;
    db->FrameIndex.ServoWritePtr = 0;
    db->FrameIndex.ServoReadPtr = 0;
    for (uint8_t i = 0; i < SERVO_FRAME_INDEX_SIZE; i++)
    {
        db->FrameIndex.ServoFrameValid[i] = false;
    }
}

/**
 * @brief 判断位置是否在环形缓冲区的某个范围内
 * @param pos 要检查的位置
 * @param start 范围起始位置（包含）
 * @param end 范围结束位置（包含）
 * @return true 如果pos在[start, end]范围内（考虑环形回绕）
 */
static bool IsPositionInRange(uint8_t pos, uint8_t start, uint8_t end)
{
    if (start <= end)
    {
        // 正常情况：start <= pos <= end
        return (pos >= start && pos <= end);
    }
    else
    {
        // 环绕情况：pos >= start || pos <= end
        return (pos >= start || pos <= end);
    }
}

/**
 * @brief 使被覆盖位置的帧无效化（切割被破坏的数据帧）
 * @param db DataBuffer指针
 * @param rb RxRingBuffer指针（用于获取协议类型）
 * @param overwritePos 被覆盖的位置
 */
static void InvalidateOverwrittenFrames(DataBuffer *db, UartRxRingBuffer *rb, uint8_t overwritePos)
{
    FrameIndexBuffer *fi = &db->FrameIndex;

    if (rb->protocol_type == PROTOCOL_SERVO)
    {
        // 遍历所有Servo帧，检查是否被覆盖
        for (uint8_t i = 0; i < SERVO_FRAME_INDEX_SIZE; i++)
        {
            if (fi->ServoFrameValid[i])
            {
                uint8_t headerPos = fi->ServoHeaderIndex[i];
                uint8_t tailPos = fi->ServoTailIndex[i];

                // 检查覆盖位置是否在帧的范围内
                if (IsPositionInRange(overwritePos, headerPos, tailPos))
                {
                    // 帧被破坏，标记为无效
                    fi->ServoFrameValid[i] = false;
                    if (fi->ServoFrameCount > 0)
                    {
                        fi->ServoFrameCount--;
                    }

                    // 如果被破坏的是读指针指向的帧，移动读指针
                    if (i == fi->ServoReadPtr)
                    {
                        // 移动读指针到下一个有效帧
                        uint8_t nextPtr = (fi->ServoReadPtr + 1) % SERVO_FRAME_INDEX_SIZE;
                        while (nextPtr != fi->ServoWritePtr && !fi->ServoFrameValid[nextPtr])
                        {
                            nextPtr = (nextPtr + 1) % SERVO_FRAME_INDEX_SIZE;
                        }
                        fi->ServoReadPtr = nextPtr;
                    }
                }
            }
        }
    }

    // 这里要结合反馈的数据长度
    else // PROTOCOL_DART
    {
        // 遍历所有DART帧，检查是否被覆盖
        for (uint8_t i = 0; i < DART_FRAME_INDEX_SIZE; i++)
        {
            if (fi->DartFrameValid[i])
            {
                uint8_t headerPos = fi->DartHeaderIndex[i];
                uint8_t tailPos = fi->DartTailIndex[i];

                // 检查覆盖位置是否在帧的范围内
                if (IsPositionInRange(overwritePos, headerPos, tailPos))
                {
                    // 帧被破坏，标记为无效
                    fi->DartFrameValid[i] = false;
                    if (fi->DartFrameCount > 0)
                    {
                        fi->DartFrameCount--;
                    }

                    // 如果被破坏的是读指针指向的帧，移动读指针
                    if (i == fi->DartReadPtr)
                    {
                        // 移动读指针到下一个有效帧
                        uint8_t nextPtr = (fi->DartReadPtr + 1) % DART_FRAME_INDEX_SIZE;
                        while (nextPtr != fi->DartWritePtr && !fi->DartFrameValid[nextPtr])
                        {
                            nextPtr = (nextPtr + 1) % DART_FRAME_INDEX_SIZE;
                        }
                        fi->DartReadPtr = nextPtr;
                    }
                }
            }
        }
    }
}

/**
 * @brief 帧头识别函数（在接收中断中调用）
 * @param db DataBuffer指针
 * @param rb RxRingBuffer指针（用于获取协议类型）
 * @param byte 当前接收到的字节
 * @param currentPos 当前字节在RxDataBuffer中的位置
 * @note  支持帧头跨越缓冲区边界的情况
 */
static void DetectFrameHeader(DataBuffer *db, UartRxRingBuffer *rb, uint8_t byte, uint8_t currentPos)
{
    // 检测是否发生了环绕（写指针从末尾回到开头）
    if (currentPos == 0 && db->HeaderMatchCount > 0)
    {
        // 帧头正在跨越边界
        db->HeaderCrossWrap = true;
    }

    if (rb->protocol_type == PROTOCOL_SERVO)
    {
        // 0x55 0x55 协议帧头识别
        if (byte == 0x55)
        {
            if (db->HeaderMatchCount == 0)
            {
                // 第一个0x55，记录位置
                db->PendingHeaderPos = currentPos;
                db->HeaderMatchCount = 1;
                db->HeaderCrossWrap = false; // 重置跨边界标志
            }
            else if (db->HeaderMatchCount == 1)
            {
                // 第二个0x55，帧头确认，记录到帧索引数组
                FrameIndexBuffer *fi = &db->FrameIndex;
                uint8_t writeIdx = fi->ServoWritePtr;

                // 记录帧头位置
                fi->ServoHeaderIndex[writeIdx] = db->PendingHeaderPos;
                fi->ServoTailIndex[writeIdx] = 0;      // 帧尾待定
                fi->ServoFrameValid[writeIdx] = false; // 帧尚未完成，暂不设为有效

                // 移动写指针
                fi->ServoWritePtr = (fi->ServoWritePtr + 1) % SERVO_FRAME_INDEX_SIZE;

                // 如果写指针追上读指针，移动读指针并使旧帧无效
                if (fi->ServoWritePtr == fi->ServoReadPtr)
                {
                    fi->ServoFrameValid[fi->ServoReadPtr] = false;
                    if (fi->ServoFrameCount > 0)
                    {
                        fi->ServoFrameCount--;
                    }
                    fi->ServoReadPtr = (fi->ServoReadPtr + 1) % SERVO_FRAME_INDEX_SIZE;
                }

                db->HeaderMatchCount = 0;
                db->HeaderCrossWrap = false;
            }
        }
        else
        {
            // 不是0x55，重置匹配计数
            db->HeaderMatchCount = 0;
            db->HeaderCrossWrap = false;
        }
    }
    else // PROTOCOL_DART
    {
        // DART 协议帧头识别 ('D', 'A', 'R', 'T')
        const uint8_t dart_header[4] = {'D', 'A', 'R', 'T'};

        if (byte == dart_header[db->HeaderMatchCount])
        {
            if (db->HeaderMatchCount == 0)
            {
                // 第一个字符'D'，记录位置
                db->PendingHeaderPos = currentPos;
                db->HeaderCrossWrap = false; // 重置跨边界标志
            }
            db->HeaderMatchCount++;

            if (db->HeaderMatchCount == 4)
            {
                // 帧头"DART"完整匹配，记录到帧索引数组
                FrameIndexBuffer *fi = &db->FrameIndex;
                uint8_t writeIdx = fi->DartWritePtr;

                // 记录帧头位置
                fi->DartHeaderIndex[writeIdx] = db->PendingHeaderPos;
                fi->DartTailIndex[writeIdx] = 0;      // 帧尾待定
                fi->DartFrameValid[writeIdx] = false; // 帧尚未完成，暂不设为有效

                // 移动写指针
                fi->DartWritePtr = (fi->DartWritePtr + 1) % DART_FRAME_INDEX_SIZE;

                // 如果写指针追上读指针，移动读指针并使旧帧无效
                if (fi->DartWritePtr == fi->DartReadPtr)
                {
                    fi->DartFrameValid[fi->DartReadPtr] = false;
                    if (fi->DartFrameCount > 0)
                    {
                        fi->DartFrameCount--;
                    }
                    fi->DartReadPtr = (fi->DartReadPtr + 1) % DART_FRAME_INDEX_SIZE;
                }

                db->HeaderMatchCount = 0;
                db->HeaderCrossWrap = false;
            }
        }
        else
        {
            // 不匹配，重置匹配计数
            db->HeaderMatchCount = 0;
            db->HeaderCrossWrap = false;
        }
    }
}

/**
 * @brief 记录帧尾位置（在CRC校验通过后调用）
 * @param db DataBuffer指针
 * @param rb RxRingBuffer指针（用于获取协议类型）
 * @param tailPos 帧尾在RxDataBuffer中的位置
 * @note  同时将帧标记为有效，增加有效帧计数
 */
static void RecordFrameTail(DataBuffer *db, UartRxRingBuffer *rb, uint8_t tailPos)
{
    FrameIndexBuffer *fi = &db->FrameIndex;

    // 帧尾一般为CRC位置，可以采用CRC解算
    if (rb->protocol_type == PROTOCOL_SERVO)
    {
        // 找到最后一个记录的帧头对应的索引
        uint8_t lastHeaderPtr = (fi->ServoWritePtr == 0) ? (SERVO_FRAME_INDEX_SIZE - 1) : (fi->ServoWritePtr - 1);

        // 记录帧尾位置
        fi->ServoTailIndex[lastHeaderPtr] = tailPos;

        // 标记帧为有效
        fi->ServoFrameValid[lastHeaderPtr] = true;
        fi->ServoFrameCount++;
    }

    // 帧尾为设置的格式
    else // PROTOCOL_DART
    {
        // 找到最后一个记录的帧头对应的索引
        uint8_t lastHeaderPtr = (fi->DartWritePtr == 0) ? (DART_FRAME_INDEX_SIZE - 1) : (fi->DartWritePtr - 1);

        // 记录帧尾位置
        fi->DartTailIndex[lastHeaderPtr] = tailPos;

        // 标记帧为有效
        fi->DartFrameValid[lastHeaderPtr] = true;
        fi->DartFrameCount++;
    }
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
 * @brief 移动读指针到下一个有效帧头位置
 * @param db DataBuffer指针
 * @param rb RxRingBuffer指针（用于获取协议类型）
 * @note  当覆盖发生时调用，确保读指针指向有效的帧头
 */
static void MoveReadIndexToNextFrame(DataBuffer *db, UartRxRingBuffer *rb)
{
    FrameIndexBuffer *fi = &db->FrameIndex;

    if (rb->protocol_type == PROTOCOL_SERVO)
    {
        // 查找下一个有效的Servo帧头位置
        for (uint8_t i = 0; i < SERVO_FRAME_INDEX_SIZE; i++)
        {
            uint8_t idx = (fi->ServoReadPtr + i) % SERVO_FRAME_INDEX_SIZE;
            if (fi->ServoFrameValid[idx])
            {
                // 找到有效帧，移动ReadIndex到该帧头位置
                db->ReadIndex = fi->ServoHeaderIndex[idx];
                fi->ServoReadPtr = idx;
                return;
            }
        }
        // 没有找到有效帧，ReadIndex移动到WriteIndex位置（缓冲区为空）
        db->ReadIndex = db->WriteIndex;
    }
    else // PROTOCOL_DART
    {
        // 查找下一个有效的DART帧头位置
        for (uint8_t i = 0; i < DART_FRAME_INDEX_SIZE; i++)
        {
            uint8_t idx = (fi->DartReadPtr + i) % DART_FRAME_INDEX_SIZE;
            if (fi->DartFrameValid[idx])
            {
                // 找到有效帧，移动ReadIndex到该帧头位置
                db->ReadIndex = fi->DartHeaderIndex[idx];
                fi->DartReadPtr = idx;
                return;
            }
        }
        // 没有找到有效帧，ReadIndex移动到WriteIndex位置（缓冲区为空）
        db->ReadIndex = db->WriteIndex;
    }
}

/**
 * @brief 向接收缓冲区写入一个字节（支持覆盖写入）
 * @note  去除读写指针间隔1的限制，允许完全覆盖
 *        缓冲区满时自动覆盖旧数据
 */
static bool RxRingBuffer_WriteByte(UartRxRingBuffer *rb, uint8_t byte)
{
    // 直接写入，无论缓冲区是否满
    rb->buffer[rb->head] = byte;
    rb->head = (rb->head + 1) % UART_RX_BUFFER_SIZE;

    // 如果head追上tail，说明覆盖了旧数据，需要移动tail
    if (rb->head == rb->tail)
    {
        rb->tail = (rb->tail + 1) % UART_RX_BUFFER_SIZE;
        rb->overflowFlag = true;
        // count保持为最大值
    }
    else
    {
        rb->count++;
    }

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

            // 初始化DataBuffer（包含帧索引缓冲区）
            DataBuffer_Init(&g_data_buffers[i]);
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
 * @brief 读取接收数据（原始数据，读取后移动ReadIndex）
 */
uint16_t UART_Read(BSP_UART_NUM_e uart_num, uint8_t *data, uint16_t len)
{
    if (uart_num >= BSP_UART_MAX || data == NULL || len == 0)
        return 0;

    DataBuffer *db = &g_data_buffers[uart_num];

    // 计算环形缓冲区中可读数据量
    uint16_t dataCount = (db->WriteIndex - db->ReadIndex + UART_RX_BUFFER_SIZE) % UART_RX_BUFFER_SIZE;
    uint16_t readLen = (len > dataCount) ? dataCount : len;

    // 从环形缓冲区读取数据并移动ReadIndex
    for (uint16_t i = 0; i < readLen; i++)
    {
        data[i] = db->RxDataBuffer[db->ReadIndex];
        db->ReadIndex = (db->ReadIndex + 1) % UART_RX_BUFFER_SIZE;
    }

    // 同时从UartRxRingBuffer读取（保持同步）
    RxRingBuffer_Read(&g_uart_rx_buffers[uart_num], NULL, 0);

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
 * @brief 获取舵机协议数据包（读取数据包时移动ReadIndex）
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

        // 读取数据包后，移动ReadIndex（数据包总长度 = 2字节包头 + 1字节ID + 1字节长度 + 长度字节内容）
        uint16_t packet_total_len = 2 + 1 + 1 + rb->servo_packet.length;
        // 环形移动ReadIndex
        db->ReadIndex = (db->ReadIndex + packet_total_len) % UART_RX_BUFFER_SIZE;

        return true;
    }

    return false;
}

/**
 * @brief 获取DART协议数据包（读取数据包时移动ReadIndex）
 */
bool UART_GetDartPacket(BSP_UART_NUM_e uart_num, DartPacket_t *packet)
{
    if (uart_num >= BSP_UART_MAX || packet == NULL)
        return false;

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num];
    DataBuffer *db = &g_data_buffers[uart_num];

    if (rb->packet_ready && rb->protocol_type == PROTOCOL_DART && rb->dart_packet.is_valid)
    {
        memcpy(packet, &rb->dart_packet, rb->dart_packet.data_len + 6);

        // 读取数据包后，移动ReadIndex（数据包总长度 = 4字节包头 + 数据长度 + 1字节CRC）
        uint16_t packet_total_len = 4 + rb->dart_packet.data_len + 1;
        // 环形移动ReadIndex
        db->ReadIndex = (db->ReadIndex + packet_total_len) % UART_RX_BUFFER_SIZE;

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
 * @brief UART接收完成回调（包含协议解析、帧头识别和DataBuffer环形写入）
 * @note  环形缓冲区特性：
 *        - 写指针可循环回绕到数组开头，覆盖旧数据
 *        - 缓冲区满时自动覆盖旧数据
 *        - 去除读写指针间隔1的限制，允许完全覆盖
 *        - 覆盖时读指针强制移动到下一个有效帧头位置
 *        - 自动识别帧头位置并记录到帧索引数组
 *        - 溢出时自动使被覆盖的帧无效化
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

    // 记录当前写入位置（用于帧头识别）
    uint8_t currentWritePos = (uint8_t)db->WriteIndex;

    // 检测环绕：写指针即将从末尾回到开头
    if (db->WriteIndex == UART_RX_BUFFER_SIZE - 1)
    {
        db->WrapAroundFlag = true; // 标记即将发生环绕
    }

    // 环形缓冲区写入DataBuffer
    // 计算下一个写位置
    uint16_t nextWriteIndex = (db->WriteIndex + 1) % UART_RX_BUFFER_SIZE;

    // 检查写指针是否追上读指针（缓冲区满，发生覆盖）
    // 去除间隔1的限制，允许完全覆盖
    if (db->WriteIndex == db->ReadIndex && db->OverflowFlag)
    {
        // 已经处于覆盖状态，每次写入都需要处理
        // 1. 使被覆盖位置的帧无效化
        InvalidateOverwrittenFrames(db, rb, (uint8_t)db->WriteIndex);

        // 2. 移动读指针到下一个有效帧头位置
        MoveReadIndexToNextFrame(db, rb);
    }
    else if (nextWriteIndex == db->ReadIndex)
    {
        // 首次追上读指针，进入覆盖模式
        // 1. 使被覆盖位置的帧无效化
        InvalidateOverwrittenFrames(db, rb, (uint8_t)db->ReadIndex);

        // 2. 移动读指针到下一个有效帧头位置
        MoveReadIndexToNextFrame(db, rb);

        db->OverflowFlag = true; // 标记发生了数据覆盖
    }

    // 写入数据
    db->RxDataBuffer[db->WriteIndex] = rb->rxByte;
    db->WriteIndex = nextWriteIndex;

    // 帧头识别（在写入数据后进行）
    DetectFrameHeader(db, rb, rb->rxByte, currentWritePos);

    // 协议解析（在帧头识别后进行）
    ParseProtocol(rb, rb->rxByte);

    // 如果协议解析完成且CRC校验通过，记录帧尾位置并设置有效标志
    if (rb->packet_ready)
    {
        // 帧尾位置是当前写入位置（刚写入的CRC字节位置）
        RecordFrameTail(db, rb, currentWritePos);
    }

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
