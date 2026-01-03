/**
 * @file UartProtocol.c
 * @brief UART协议解析模块源文件
 * @note  本模块从bsp_uart中分离出来，负责协议解析相关功能
 *        与底层HAL库无强关联
 */

#include "UartProtocol.h"
#include <string.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"

/* 外部变量声明（在bsp_uart.c中定义） */
extern UartRxRingBuffer *BSP_UART_GetRxBuffer(BSP_UART_NUM_e uart_num);
extern DataBuffer *BSP_UART_GetDataBuffer(BSP_UART_NUM_e uart_num);

// 信号量与互斥量，当接收到数据并解算成功将置互斥量
extern osThreadId_t UartModuleTaskHandle;
extern SemaphoreHandle_t g_xGimbalSemaphoreHandle;

/* DART协议帧头 */
const uint8_t g_dart_header[4] = {'D', 'A', 'R', 'T'};

/**
 * @brief CRC校验生成（公开函数，供所有模块使用）
 * @param data 数据指针
 * @param length 数据长度
 * @return CRC校验码
 */
uint8_t Protocol_Calculate_CRC(uint8_t *data, uint8_t length)
{
    uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++)
    {
        sum += data[i];
    }
    return (uint8_t)(~sum);
}

/**
 * @brief 重置协议解析器状态（不清除数据包内容）
 */
void Protocol_ResetParserState(UartRxRingBuffer *rb)
{
    rb->parse_state = PARSE_HEADER;
    rb->parse_index = 0;
}

/**
 * @brief 完全重置协议解析器（清除数据包内容和状态）
 * @note 仅在初始化和协议切换时使用
 */
void Protocol_ResetParser(UartRxRingBuffer *rb)
{
    rb->parse_state = PARSE_HEADER;
    rb->parse_index = 0;
    rb->packet_ready = false;
    memset(&rb->servo_packet, 0, sizeof(ServoPacket_t));
    memset(&rb->dart_packet, 0, sizeof(DartPacket_t));
}

/**
 * @brief 协议解析状态机
 * @param rb 接收缓冲区指针
 * @param byte 接收到的字节
 * @note 支持三种协议：
 *       - PROTOCOL_SERVO_MCU: 有MCU控制板协议（无CRC）
 *       - PROTOCOL_SERVO_NO_MCU: 无MCU驱动板协议（有CRC）
 *       - PROTOCOL_DART: DART协议
 */
void Protocol_ParseByte(UartRxRingBuffer *rb, uint8_t byte)
{
    // 确保上下文切换
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (rb->protocol_type == PROTOCOL_SERVO_MCU)
    {

        // 有MCU控制板协议解析：0x55 0x55 | Length | Cmd | Params（无CRC）
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
                    rb->parse_state = PARSE_SERVO_LENGTH;
                    rb->parse_index = 0;
                }
                else
                {
                    Protocol_ResetParser(rb);
                }
            }
            break;

        case PARSE_SERVO_LENGTH:
            rb->servo_packet.length = byte;
            if (byte >= 2 && byte <= 64)
            {
                rb->parse_state = PARSE_SERVO_CMD;
            }
            else
            {
                Protocol_ResetParser(rb);
            }
            break;

        case PARSE_SERVO_CMD:
            rb->servo_packet.cmd = byte;
            rb->servo_packet.param_len = rb->servo_packet.length - 2;
            if (rb->servo_packet.param_len > 0)
            {
                rb->parse_state = PARSE_SERVO_PARAMS;
                rb->parse_index = 0;
            }
            else
            {
                rb->servo_packet.is_valid = true;
                rb->packet_ready = true;
                Protocol_ResetParserState(rb);
            }
            break;

        case PARSE_SERVO_PARAMS:
            if (rb->parse_index < rb->servo_packet.param_len && rb->parse_index < 8)
            {
                rb->servo_packet.params[rb->parse_index++] = byte;
                if (rb->parse_index >= rb->servo_packet.param_len)
                {
                    rb->servo_packet.is_valid = true;
                    rb->packet_ready = true;
                    Protocol_ResetParserState(rb);

                    // 这里放一个任务通知(裸机跑这部分不管)
                    // xTaskNotifyFromISR(UartModuleTaskHandle, 0x01, eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);
                }
            }
            else
            {
                Protocol_ResetParser(rb);
            }
            break;

        default:
            Protocol_ResetParser(rb);
            break;
        }
    }
    else if (rb->protocol_type == PROTOCOL_SERVO_NO_MCU)
    {
        // 无MCU驱动板协议解析：0x55 0x55 | ID | Length | Cmd | Params | CRC（有CRC）
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
                    Protocol_ResetParser(rb);
                }
            }
            break;

        case PARSE_SERVO_ID:
            rb->servo_packet.id = byte;
            rb->parse_state = PARSE_SERVO_LENGTH;
            break;

        case PARSE_SERVO_LENGTH:
            rb->servo_packet.length = byte;
            if (byte >= 3 && byte <= 11)
            {
                rb->parse_state = PARSE_SERVO_CMD;
            }
            else
            {
                Protocol_ResetParser(rb);
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

                    // 这里放一个任务通知
                    // xTaskNotifyFromISR(UartModuleTaskHandle, 0x02, eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);
                }
            }
            else
            {
                Protocol_ResetParser(rb);
            }
            break;

        case PARSE_SERVO_CRC:
        {
            rb->servo_packet.checksum = byte;
            uint8_t temp_buf[16];
            temp_buf[0] = rb->servo_packet.id;
            temp_buf[1] = rb->servo_packet.length;
            temp_buf[2] = rb->servo_packet.cmd;
            for (uint8_t i = 0; i < rb->servo_packet.param_len; i++)
            {
                temp_buf[3 + i] = rb->servo_packet.params[i];
            }
            uint8_t calc_crc = Protocol_Calculate_CRC(temp_buf, 3 + rb->servo_packet.param_len);

            if (calc_crc == byte)
            {
                rb->servo_packet.is_valid = true;
                rb->packet_ready = true;
                Protocol_ResetParserState(rb);
            }
            else
            {
                Protocol_ResetParser(rb);
            }
            break;
        }

        default:
            Protocol_ResetParser(rb);
            break;
        }
    }
    else // PROTOCOL_DART
    {
        // DART协议解析：DART | Length | Data... | CRC
        switch (rb->parse_state)
        {
        case PARSE_HEADER:
            if (rb->parse_index < 4)
            {
                if (byte == g_dart_header[rb->parse_index])
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
                    Protocol_ResetParser(rb);
                }
            }
            break;

        case PARSE_DART_LENGTH:
            rb->dart_packet.data_len = byte;
            if (rb->dart_packet.data_len <= sizeof(rb->dart_packet.data))
            {
                rb->parse_state = (rb->dart_packet.data_len == 0) ? PARSE_DART_CRC : PARSE_DART_DATA;
                rb->parse_index = 0;
            }
            else
            {
                Protocol_ResetParser(rb);
            }
            break;

        case PARSE_DART_DATA:
            if (rb->parse_index < rb->dart_packet.data_len)
            {
                rb->dart_packet.data[rb->parse_index++] = byte;
                if (rb->parse_index >= rb->dart_packet.data_len)
                {
                    rb->parse_state = PARSE_DART_CRC;
                    rb->parse_index = 0;
                }
            }
            break;

        case PARSE_DART_CRC:
        {
            rb->dart_packet.checksum = byte;
            uint8_t temp_buf[68];
            memcpy(temp_buf, rb->dart_packet.data, rb->dart_packet.data_len);
            uint8_t calc_crc = Protocol_Calculate_CRC(temp_buf, rb->dart_packet.data_len);

            if (calc_crc == byte)
            {
                rb->dart_packet.is_valid = true;
                rb->packet_ready = true;
                Protocol_ResetParserState(rb);

                // 这里放一个任务通知
                // xTaskNotifyFromISR(UartModuleTaskHandle, 0x03, eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);
            }
            else
            {
                Protocol_ResetParser(rb);
            }
            break;
        }

        default:
            Protocol_ResetParser(rb);
            break;
        }
    }

    // 如果有更高优先级任务被唤醒，请求上下文切换
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief DataBuffer初始化（包含帧索引缓冲区）
 */
void Protocol_DataBufferInit(DataBuffer *db)
{
    memset(db->RxDataBuffer, 0, UART_RX_BUFFER_SIZE);
    db->ReadIndex = 0;
    db->WriteIndex = 0;
    db->OverflowFlag = false;
    db->WrapAroundFlag = false;

    db->HeaderMatchCount = 0;
    db->PendingHeaderPos = 0;
    db->HeaderCrossWrap = false;

    // DART协议帧索引
    memset(db->FrameIndex.DartHeaderIndex, 0, DART_FRAME_INDEX_SIZE);
    memset(db->FrameIndex.DartTailIndex, 0, DART_FRAME_INDEX_SIZE);
    db->FrameIndex.DartFrameCount = 0;
    db->FrameIndex.DartWritePtr = 0;
    db->FrameIndex.DartReadPtr = 0;
    for (uint8_t i = 0; i < DART_FRAME_INDEX_SIZE; i++)
    {
        db->FrameIndex.DartFrameValid[i] = false;
    }

    // Servo协议帧索引
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
 */
static bool IsPositionInRange(uint8_t pos, uint8_t start, uint8_t end)
{
    if (start <= end)
    {
        return (pos >= start && pos <= end);
    }
    else
    {
        return (pos >= start || pos <= end);
    }
}

/**
 * @brief 使被覆盖位置的帧无效化
 */
void Protocol_InvalidateOverwrittenFrames(DataBuffer *db, UartRxRingBuffer *rb, uint8_t overwritePos)
{
    FrameIndexBuffer *fi = &db->FrameIndex;

    if (rb->protocol_type == PROTOCOL_SERVO_MCU || rb->protocol_type == PROTOCOL_SERVO_NO_MCU)
    {
        for (uint8_t i = 0; i < SERVO_FRAME_INDEX_SIZE; i++)
        {
            if (fi->ServoFrameValid[i])
            {
                uint8_t headerPos = fi->ServoHeaderIndex[i];
                uint8_t tailPos = fi->ServoTailIndex[i];

                if (IsPositionInRange(overwritePos, headerPos, tailPos))
                {
                    fi->ServoFrameValid[i] = false;
                    if (fi->ServoFrameCount > 0)
                    {
                        fi->ServoFrameCount--;
                    }

                    if (i == fi->ServoReadPtr)
                    {
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
    else // PROTOCOL_DART
    {
        for (uint8_t i = 0; i < DART_FRAME_INDEX_SIZE; i++)
        {
            if (fi->DartFrameValid[i])
            {
                uint8_t headerPos = fi->DartHeaderIndex[i];
                uint8_t tailPos = fi->DartTailIndex[i];

                if (IsPositionInRange(overwritePos, headerPos, tailPos))
                {
                    fi->DartFrameValid[i] = false;
                    if (fi->DartFrameCount > 0)
                    {
                        fi->DartFrameCount--;
                    }

                    if (i == fi->DartReadPtr)
                    {
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
 * @brief 帧头识别函数
 */
void Protocol_DetectFrameHeader(DataBuffer *db, UartRxRingBuffer *rb, uint8_t byte, uint8_t currentPos)
{
    if (currentPos == 0 && db->HeaderMatchCount > 0)
    {
        db->HeaderCrossWrap = true;
    }

    if (rb->protocol_type == PROTOCOL_SERVO_MCU || rb->protocol_type == PROTOCOL_SERVO_NO_MCU)
    {
        // 0x55 0x55 协议帧头识别
        if (byte == 0x55)
        {
            if (db->HeaderMatchCount == 0)
            {
                db->PendingHeaderPos = currentPos;
                db->HeaderMatchCount = 1;
                db->HeaderCrossWrap = false;
            }
            else if (db->HeaderMatchCount == 1)
            {
                FrameIndexBuffer *fi = &db->FrameIndex;
                uint8_t writeIdx = fi->ServoWritePtr;

                fi->ServoHeaderIndex[writeIdx] = db->PendingHeaderPos;
                fi->ServoTailIndex[writeIdx] = 0;
                fi->ServoFrameValid[writeIdx] = false;

                fi->ServoWritePtr = (fi->ServoWritePtr + 1) % SERVO_FRAME_INDEX_SIZE;

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
            db->HeaderMatchCount = 0;
            db->HeaderCrossWrap = false;
        }
    }
    else // PROTOCOL_DART
    {
        // DART 协议帧头识别
        if (byte == g_dart_header[db->HeaderMatchCount])
        {
            if (db->HeaderMatchCount == 0)
            {
                db->PendingHeaderPos = currentPos;
                db->HeaderCrossWrap = false;
            }
            db->HeaderMatchCount++;

            if (db->HeaderMatchCount == 4)
            {
                FrameIndexBuffer *fi = &db->FrameIndex;
                uint8_t writeIdx = fi->DartWritePtr;

                fi->DartHeaderIndex[writeIdx] = db->PendingHeaderPos;
                fi->DartTailIndex[writeIdx] = 0;
                fi->DartFrameValid[writeIdx] = false;

                fi->DartWritePtr = (fi->DartWritePtr + 1) % DART_FRAME_INDEX_SIZE;

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
            db->HeaderMatchCount = 0;
            db->HeaderCrossWrap = false;
        }
    }
}

/**
 * @brief 记录帧尾位置
 */
void Protocol_RecordFrameTail(DataBuffer *db, UartRxRingBuffer *rb, uint8_t tailPos)
{
    FrameIndexBuffer *fi = &db->FrameIndex;

    if (rb->protocol_type == PROTOCOL_SERVO_MCU || rb->protocol_type == PROTOCOL_SERVO_NO_MCU)
    {
        uint8_t lastHeaderPtr = (fi->ServoWritePtr == 0) ? (SERVO_FRAME_INDEX_SIZE - 1) : (fi->ServoWritePtr - 1);
        fi->ServoTailIndex[lastHeaderPtr] = tailPos;
        fi->ServoFrameValid[lastHeaderPtr] = true;
        fi->ServoFrameCount++;
    }
    else // PROTOCOL_DART
    {
        uint8_t lastHeaderPtr = (fi->DartWritePtr == 0) ? (DART_FRAME_INDEX_SIZE - 1) : (fi->DartWritePtr - 1);
        fi->DartTailIndex[lastHeaderPtr] = tailPos;
        fi->DartFrameValid[lastHeaderPtr] = true;
        fi->DartFrameCount++;
    }
}

/**
 * @brief 移动读指针到下一个有效帧头位置
 */
void Protocol_MoveReadIndexToNextFrame(DataBuffer *db, UartRxRingBuffer *rb)
{
    FrameIndexBuffer *fi = &db->FrameIndex;

    if (rb->protocol_type == PROTOCOL_SERVO_MCU || rb->protocol_type == PROTOCOL_SERVO_NO_MCU)
    {
        for (uint8_t i = 0; i < SERVO_FRAME_INDEX_SIZE; i++)
        {
            uint8_t idx = (fi->ServoReadPtr + i) % SERVO_FRAME_INDEX_SIZE;
            if (fi->ServoFrameValid[idx])
            {
                db->ReadIndex = fi->ServoHeaderIndex[idx];
                fi->ServoReadPtr = idx;
                return;
            }
        }
        db->ReadIndex = db->WriteIndex;
    }
    else // PROTOCOL_DART
    {
        for (uint8_t i = 0; i < DART_FRAME_INDEX_SIZE; i++)
        {
            uint8_t idx = (fi->DartReadPtr + i) % DART_FRAME_INDEX_SIZE;
            if (fi->DartFrameValid[idx])
            {
                db->ReadIndex = fi->DartHeaderIndex[idx];
                fi->DartReadPtr = idx;
                return;
            }
        }
        db->ReadIndex = db->WriteIndex;
    }
}

/* ========== 协议解析API实现 ========== */

/**
 * @brief 设置协议类型
 */
void Protocol_SetType(BSP_UART_NUM_e uart_num, PROTOCOL_TYPE_e protocol_type)
{
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    if (rb != NULL)
    {
        rb->protocol_type = protocol_type;
        Protocol_ResetParser(rb);
    }
}

/**
 * @brief 检查是否有完整的数据包
 */
bool Protocol_HasPacket(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return false;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    return (rb != NULL) ? rb->packet_ready : false;
}

/**
 * @brief 获取舵机协议数据包
 */
bool Protocol_GetServoPacket(BSP_UART_NUM_e uart_num, ServoPacket_t *packet)
{
    if (uart_num >= BSP_UART_MAX || packet == NULL)
        return false;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    DataBuffer *db = BSP_UART_GetDataBuffer(uart_num);

    if (rb == NULL || db == NULL)
        return false;

    bool is_servo_protocol = (rb->protocol_type == PROTOCOL_SERVO_MCU ||
                              rb->protocol_type == PROTOCOL_SERVO_NO_MCU);

    if (rb->packet_ready && is_servo_protocol && rb->servo_packet.is_valid)
    {
        memcpy(packet, &rb->servo_packet, sizeof(ServoPacket_t));

        uint16_t packet_total_len;
        if (rb->protocol_type == PROTOCOL_SERVO_MCU)
        {
            packet_total_len = 2 + 1 + rb->servo_packet.length;
        }
        else
        {
            packet_total_len = 2 + 1 + 1 + rb->servo_packet.length;
        }

        db->ReadIndex = (db->ReadIndex + packet_total_len) % UART_RX_BUFFER_SIZE;
        return true;
    }

    return false;
}

/**
 * @brief 获取DART协议数据包
 */
bool Protocol_GetDartPacket(BSP_UART_NUM_e uart_num, DartPacket_t *packet)
{
    if (uart_num >= BSP_UART_MAX || packet == NULL)
        return false;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    DataBuffer *db = BSP_UART_GetDataBuffer(uart_num);

    if (rb == NULL || db == NULL)
        return false;

    if (rb->packet_ready && rb->protocol_type == PROTOCOL_DART && rb->dart_packet.is_valid)
    {
        memcpy(packet, &rb->dart_packet, sizeof(DartPacket_t));

        uint16_t packet_total_len = 4 + 1 + rb->dart_packet.data_len + 1;
        db->ReadIndex = (db->ReadIndex + packet_total_len) % UART_RX_BUFFER_SIZE;

        return true;
    }

    return false;
}

/**
 * @brief 清除数据包标志
 */
void Protocol_ClearPacket(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    if (rb != NULL)
    {
        rb->packet_ready = false;
        rb->servo_packet.is_valid = false;
        rb->dart_packet.is_valid = false;
    }
}

/**
 * @brief 获取有效帧数量
 */
uint8_t Protocol_GetFrameCount(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return 0;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    DataBuffer *db = BSP_UART_GetDataBuffer(uart_num);

    if (rb == NULL || db == NULL)
        return 0;

    if (rb->protocol_type == PROTOCOL_SERVO_MCU || rb->protocol_type == PROTOCOL_SERVO_NO_MCU)
    {
        return db->FrameIndex.ServoFrameCount;
    }
    else
    {
        return db->FrameIndex.DartFrameCount;
    }
}
