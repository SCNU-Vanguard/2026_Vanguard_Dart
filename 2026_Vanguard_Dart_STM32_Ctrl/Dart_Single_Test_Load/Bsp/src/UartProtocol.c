/**
 * @file UartProtocol.c
 * @brief UART协议解析模块源文件（简化版）
 * @note  仅保留舵机协议解析与IBUS帧校验，DART协议移除
 */

#include "UartProtocol.h"
#include "CRC.h"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"

/* 外部变量声明（在bsp_uart.c中定义） */
extern UartRxRingBuffer *BSP_UART_GetRxBuffer(BSP_UART_NUM_e uart_num);

#define REFEREE_SOF_BYTE 0xA5U
#define REFEREE_HEADER_LEN 5U
#define REFEREE_CMD_ID_LEN 2U
#define REFEREE_TAIL_LEN 2U

volatile uint32_t RefereeProtocolDebug_SofCount = 0U;
volatile uint32_t RefereeProtocolDebug_HeaderCrcFailCount = 0U;
volatile uint32_t RefereeProtocolDebug_FrameCrcFailCount = 0U;
volatile uint32_t RefereeProtocolDebug_InvalidLengthCount = 0U;
volatile uint32_t RefereeProtocolDebug_FrameOverflowCount = 0U;
volatile uint32_t RefereeProtocolDebug_FrameOkCount = 0U;
volatile uint32_t RefereeProtocolDebug_QueuePushCount = 0U;
volatile uint32_t RefereeProtocolDebug_QueueFullDropCount = 0U;
volatile uint32_t RefereeProtocolDebug_QueuePopCount = 0U;
volatile uint32_t RefereeProtocolDebug_ClearCount = 0U;
volatile uint16_t RefereeProtocolDebug_LastDataLen = 0U;
volatile uint16_t RefereeProtocolDebug_LastFrameLen = 0U;
volatile uint16_t RefereeProtocolDebug_LastCmdID = 0U;

static void Protocol_ResetRefereeStream(UartRxRingBuffer *rb)
{
    if (rb == NULL)
        return;

    rb->referee_frame_len = 0;
    rb->referee_expected_len = 0;
}

static void Protocol_StartRefereeFrame(UartRxRingBuffer *rb, uint8_t byte)
{
    if (rb == NULL)
        return;

    RefereeProtocolDebug_SofCount++;
    rb->referee_frame[0] = byte;
    rb->referee_frame_len = 1;
    rb->referee_expected_len = 0;
}

static void Protocol_ResyncReferee(UartRxRingBuffer *rb, uint8_t byte)
{
    if (rb == NULL)
        return;

    if (byte == REFEREE_SOF_BYTE)
    {
        Protocol_StartRefereeFrame(rb, byte);
    }
    else
    {
        Protocol_ResetRefereeStream(rb);
    }
}

static void Protocol_QueueRefereePacket(UartRxRingBuffer *rb)
{
    RefereePacket_t *packet;

    if (rb == NULL)
        return;

    if (rb->referee_pkt_count >= REFEREE_PACKET_QUEUE_SIZE)
    {
        RefereeProtocolDebug_QueueFullDropCount++;
        return;
    }

    packet = &rb->referee_packet_queue[rb->referee_pkt_head];
    memcpy(packet->data, rb->referee_frame, rb->referee_expected_len);
    packet->length = rb->referee_expected_len;
    packet->is_valid = true;
    rb->referee_pkt_head = (rb->referee_pkt_head + 1) % REFEREE_PACKET_QUEUE_SIZE;
    rb->referee_pkt_count++;
    RefereeProtocolDebug_QueuePushCount++;
}

static void Protocol_ParseRefereeByte(UartRxRingBuffer *rb, uint8_t byte)
{
    if (rb == NULL)
        return;

    if (rb->referee_frame_len == 0)
    {
        if (byte == REFEREE_SOF_BYTE)
        {
            Protocol_StartRefereeFrame(rb, byte);
        }
        return;
    }

    if (rb->referee_frame_len >= REFEREE_FRAME_MAX_SIZE)
    {
        RefereeProtocolDebug_FrameOverflowCount++;
        Protocol_ResyncReferee(rb, byte);
        return;
    }

    rb->referee_frame[rb->referee_frame_len++] = byte;

    if (rb->referee_frame_len == REFEREE_HEADER_LEN)
    {
        uint16_t data_len;
        uint16_t total_len;

        if (!Verify_CRC8_Check_Sum(rb->referee_frame, REFEREE_HEADER_LEN))
        {
            RefereeProtocolDebug_HeaderCrcFailCount++;
            Protocol_ResyncReferee(rb, byte);
            return;
        }

        data_len = (uint16_t)rb->referee_frame[1] | ((uint16_t)rb->referee_frame[2] << 8);
        total_len = (uint16_t)(data_len + REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN + REFEREE_TAIL_LEN);
        RefereeProtocolDebug_LastDataLen = data_len;
        RefereeProtocolDebug_LastFrameLen = total_len;
        if (total_len < (REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN + REFEREE_TAIL_LEN) ||
            total_len > REFEREE_FRAME_MAX_SIZE)
        {
            RefereeProtocolDebug_InvalidLengthCount++;
            Protocol_ResyncReferee(rb, byte);
            return;
        }

        rb->referee_expected_len = total_len;
    }

    if (rb->referee_expected_len > 0 && rb->referee_frame_len == rb->referee_expected_len)
    {
        if (Verify_CRC16_Check_Sum(rb->referee_frame, rb->referee_expected_len))
        {
            RefereeProtocolDebug_FrameOkCount++;
            RefereeProtocolDebug_LastCmdID = (uint16_t)rb->referee_frame[5] | ((uint16_t)rb->referee_frame[6] << 8);
            Protocol_QueueRefereePacket(rb);
        }
        else
        {
            RefereeProtocolDebug_FrameCrcFailCount++;
        }
        Protocol_ResetRefereeStream(rb);
    }
}

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
    if (rb == NULL)
        return;

    rb->parse_state = PARSE_HEADER;
    rb->parse_index = 0;
}

/**
 * @brief 完全重置协议解析器（清除数据包内容和状态）
 * @note 仅在初始化和协议切换时使用
 */
void Protocol_ResetParser(UartRxRingBuffer *rb)
{
    if (rb == NULL)
        return;

    rb->parse_state = PARSE_HEADER;
    rb->parse_index = 0;
    memset(&rb->servo_packet_temp, 0, sizeof(ServoPacket_t));
    memset(rb->servo_packet_queue, 0, sizeof(rb->servo_packet_queue));
    rb->servo_pkt_head = 0;
    rb->servo_pkt_tail = 0;
    rb->servo_pkt_count = 0;
    rb->ibus_packet.is_valid = false;
    rb->ibus_ready = false;
    rb->ibus_error_count = 0;
    rb->ibus_stream_len = 0;
    memset(rb->ibus_stream, 0, sizeof(rb->ibus_stream));
    memset(rb->referee_packet_queue, 0, sizeof(rb->referee_packet_queue));
    memset(rb->referee_frame, 0, sizeof(rb->referee_frame));
    rb->referee_pkt_head = 0;
    rb->referee_pkt_tail = 0;
    rb->referee_pkt_count = 0;
    rb->referee_frame_len = 0;
    rb->referee_expected_len = 0;
}

/**
 * @brief 协议解析状态机（舵机协议）
 * @param rb 接收缓冲区指针
 * @param byte 接收到的字节
 */
void Protocol_ParseByte(UartRxRingBuffer *rb, uint8_t byte)
{
    if (rb == NULL || rb->protocol_type == PROTOCOL_IBUS)
        return;

    if (rb->protocol_type == PROTOCOL_REFEREE)
    {
        Protocol_ParseRefereeByte(rb, byte);
        return;
    }

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
                    rb->servo_packet_temp.header[0] = byte;
                    rb->parse_index = 1;
                }
            }
            else if (rb->parse_index == 1)
            {
                if (byte == 0x55)
                {
                    rb->servo_packet_temp.header[1] = byte;
                    rb->parse_state = PARSE_SERVO_LENGTH;
                    rb->parse_index = 0;
                }
                else
                {
                    Protocol_ResetParserState(rb);
                }
            }
            break;

        case PARSE_SERVO_LENGTH:
            rb->servo_packet_temp.length = byte;
            if (byte >= 2 && byte <= 64)
            {
                rb->parse_state = PARSE_SERVO_CMD;
            }
            else
            {
                Protocol_ResetParserState(rb);
            }
            break;

        case PARSE_SERVO_CMD:
            rb->servo_packet_temp.cmd = byte;
            rb->servo_packet_temp.param_len = rb->servo_packet_temp.length - 2;
            if (rb->servo_packet_temp.param_len > 0)
            {
                rb->parse_state = PARSE_SERVO_PARAMS;
                rb->parse_index = 0;
            }
            else
            {
                rb->servo_packet_temp.is_valid = true;
                if (rb->servo_pkt_count < SERVO_PACKET_QUEUE_SIZE)
                {
                    rb->servo_packet_queue[rb->servo_pkt_head] = rb->servo_packet_temp;
                    rb->servo_pkt_head = (rb->servo_pkt_head + 1) % SERVO_PACKET_QUEUE_SIZE;
                    rb->servo_pkt_count++;
                }
                Protocol_ResetParserState(rb);
            }
            break;

        case PARSE_SERVO_PARAMS:
            if (rb->parse_index < rb->servo_packet_temp.param_len && rb->parse_index < 8)
            {
                rb->servo_packet_temp.params[rb->parse_index++] = byte;
                if (rb->parse_index >= rb->servo_packet_temp.param_len)
                {
                    rb->servo_packet_temp.is_valid = true;
                    if (rb->servo_pkt_count < SERVO_PACKET_QUEUE_SIZE)
                    {
                        rb->servo_packet_queue[rb->servo_pkt_head] = rb->servo_packet_temp;
                        rb->servo_pkt_head = (rb->servo_pkt_head + 1) % SERVO_PACKET_QUEUE_SIZE;
                        rb->servo_pkt_count++;
                    }
                    Protocol_ResetParserState(rb);
                }
            }
            else
            {
                Protocol_ResetParserState(rb);
            }
            break;

        default:
            Protocol_ResetParserState(rb);
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
                    rb->servo_packet_temp.header[0] = byte;
                    rb->parse_index = 1;
                }
            }
            else if (rb->parse_index == 1)
            {
                if (byte == 0x55)
                {
                    rb->servo_packet_temp.header[1] = byte;
                    rb->parse_state = PARSE_SERVO_ID;
                    rb->parse_index = 0;
                }
                else
                {
                    Protocol_ResetParserState(rb);
                }
            }
            break;

        case PARSE_SERVO_ID:
            rb->servo_packet_temp.id = byte;
            rb->parse_state = PARSE_SERVO_LENGTH;
            break;

        case PARSE_SERVO_LENGTH:
            rb->servo_packet_temp.length = byte;
            if (byte >= 3 && byte <= 11)
            {
                rb->parse_state = PARSE_SERVO_CMD;
            }
            else
            {
                Protocol_ResetParserState(rb);
            }
            break;

        case PARSE_SERVO_CMD:
            rb->servo_packet_temp.cmd = byte;
            rb->servo_packet_temp.param_len = rb->servo_packet_temp.length - 3;
            if (rb->servo_packet_temp.param_len > 0)
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
            if (rb->parse_index < rb->servo_packet_temp.param_len && rb->parse_index < 8)
            {
                rb->servo_packet_temp.params[rb->parse_index++] = byte;
                if (rb->parse_index >= rb->servo_packet_temp.param_len)
                {
                    rb->parse_state = PARSE_SERVO_CRC;
                }
            }
            else
            {
                Protocol_ResetParserState(rb);
            }
            break;

        case PARSE_SERVO_CRC:
        {
            rb->servo_packet_temp.checksum = byte;
            uint8_t temp_buf[16];
            temp_buf[0] = rb->servo_packet_temp.id;
            temp_buf[1] = rb->servo_packet_temp.length;
            temp_buf[2] = rb->servo_packet_temp.cmd;
            for (uint8_t i = 0; i < rb->servo_packet_temp.param_len; i++)
            {
                temp_buf[3 + i] = rb->servo_packet_temp.params[i];
            }
            uint8_t calc_crc = Protocol_Calculate_CRC(temp_buf, 3 + rb->servo_packet_temp.param_len);

            if (calc_crc == byte)
            {
                rb->servo_packet_temp.is_valid = true;
                if (rb->servo_pkt_count < SERVO_PACKET_QUEUE_SIZE)
                {
                    rb->servo_packet_queue[rb->servo_pkt_head] = rb->servo_packet_temp;
                    rb->servo_pkt_head = (rb->servo_pkt_head + 1) % SERVO_PACKET_QUEUE_SIZE;
                    rb->servo_pkt_count++;
                }
                Protocol_ResetParserState(rb);
            }
            else
            {
                Protocol_ResetParserState(rb);
            }
            break;
        }

        default:
            Protocol_ResetParserState(rb);
            break;
        }
    }
    else
    {
        Protocol_ResetParserState(rb);
    }
}

/**
 * @brief 计算IBUS校验（0xFFFF - sum[0..len-3]）
 */
static uint16_t Protocol_IbusChecksum(const uint8_t *frame, uint16_t len)
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < (len - 2); i++)
    {
        sum += frame[i];
    }
    return (uint16_t)(0xFFFF - (uint16_t)sum);
}

/**
 * @brief 校验IBUS帧：帧头 + 16位校验
 */
static bool Protocol_IbusValidate(const uint8_t *frame, uint16_t len)
{
    if (frame == NULL || len != IBUS_FRAME_LEN)
        return false;
    if (frame[0] != IBUS_LENGTH || frame[1] != IBUS_COMMAND)
        return false;

    uint16_t expect = Protocol_IbusChecksum(frame, len);
    uint16_t recv = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8);
    return expect == recv;
}

/**
 * @brief 追加IBUS数据流到缓存
 */
static void Protocol_IbusStreamAppend(UartRxRingBuffer *rb, const uint8_t *data, uint16_t len)
{
    if (len > IBUS_STREAM_BUFFER_LEN)
    {
        data += (len - IBUS_STREAM_BUFFER_LEN);
        len = IBUS_STREAM_BUFFER_LEN;
    }

    if (rb->ibus_stream_len + len > IBUS_STREAM_BUFFER_LEN)
    {
        uint16_t overflow = (rb->ibus_stream_len + len) - IBUS_STREAM_BUFFER_LEN;
        memmove(rb->ibus_stream, rb->ibus_stream + overflow, rb->ibus_stream_len - overflow);
        rb->ibus_stream_len -= overflow;
    }

    memcpy(rb->ibus_stream + rb->ibus_stream_len, data, len);
    rb->ibus_stream_len += len;
}

/**
 * @brief 解析IBUS数据流（DMA事件回调中调用）
 * @param rb 接收缓冲区指针
 * @param data 原始接收数据
 * @param len 数据长度
 */
void Protocol_ParseIbusStream(UartRxRingBuffer *rb, const uint8_t *data, uint16_t len)
{
    if (rb == NULL || data == NULL || len == 0)
        return;

    // 快速路径：DMA 收到的刚好是完整帧，跳过 stream append 和 memmove
    if (len == IBUS_FRAME_LEN && rb->ibus_stream_len == 0 &&
        data[0] == IBUS_LENGTH && data[1] == IBUS_COMMAND)
    {
        if (Protocol_IbusValidate(data, IBUS_FRAME_LEN))
        {
            memcpy(rb->ibus_packet.data, data, IBUS_FRAME_LEN);
            rb->ibus_packet.is_valid = true;
            rb->ibus_ready = true;
            return;
        }
    }

    // 慢速路径：流式解析
    Protocol_IbusStreamAppend(rb, data, len);

    uint16_t offset = 0;
    while (rb->ibus_stream_len - offset >= IBUS_FRAME_LEN)
    {
        const uint8_t *candidate = rb->ibus_stream + offset;
        if (candidate[0] == IBUS_LENGTH && candidate[1] == IBUS_COMMAND)
        {
            if (Protocol_IbusValidate(candidate, IBUS_FRAME_LEN))
            {
                memcpy(rb->ibus_packet.data, candidate, IBUS_FRAME_LEN);
                rb->ibus_packet.is_valid = true;
                rb->ibus_ready = true;
                offset += IBUS_FRAME_LEN;
                continue;
            }
            rb->ibus_error_count++;
        }
        offset += 1;
    }

    if (offset > 0)
    {
        memmove(rb->ibus_stream, rb->ibus_stream + offset, rb->ibus_stream_len - offset);
        rb->ibus_stream_len -= offset;
    }
}

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
 * @brief 检查是否有完整的舵机数据包
 */
bool Protocol_HasPacket(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return false;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    return (rb != NULL) ? (rb->servo_pkt_count > 0) : false;
}

/**
 * @brief 获取舵机协议数据包
 */
bool Protocol_GetServoPacket(BSP_UART_NUM_e uart_num, ServoPacket_t *packet)
{
    if (uart_num >= BSP_UART_MAX || packet == NULL)
        return false;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    if (rb == NULL)
        return false;

    bool is_servo_protocol = (rb->protocol_type == PROTOCOL_SERVO_MCU ||
                              rb->protocol_type == PROTOCOL_SERVO_NO_MCU);

    if (is_servo_protocol && rb->servo_pkt_count > 0)
    {
        taskENTER_CRITICAL();
        memcpy(packet, &rb->servo_packet_queue[rb->servo_pkt_tail], sizeof(ServoPacket_t));
        rb->servo_pkt_tail = (rb->servo_pkt_tail + 1) % SERVO_PACKET_QUEUE_SIZE;
        rb->servo_pkt_count--;
        taskEXIT_CRITICAL();
        return true;
    }

    return false;
}

/**
 * @brief 清除舵机数据包标志
 */
void Protocol_ClearPacket(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    if (rb != NULL)
    {
        taskENTER_CRITICAL();
        rb->servo_pkt_head = 0;
        rb->servo_pkt_tail = 0;
        rb->servo_pkt_count = 0;
        taskEXIT_CRITICAL();
    }
}

/**
 * @brief 检查是否有完整的IBUS数据包
 */
bool Protocol_HasIbusPacket(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return false;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    if (rb == NULL)
        return false;

    return (rb->protocol_type == PROTOCOL_IBUS && rb->ibus_ready && rb->ibus_packet.is_valid);
}

/**
 * @brief 获取IBUS数据包
 */
bool Protocol_GetIbusPacket(BSP_UART_NUM_e uart_num, IbusPacket_t *packet)
{
    if (uart_num >= BSP_UART_MAX || packet == NULL)
        return false;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    if (rb == NULL)
        return false;

    if (rb->protocol_type == PROTOCOL_IBUS && rb->ibus_ready && rb->ibus_packet.is_valid)
    {
        taskENTER_CRITICAL();
        memcpy(packet, &rb->ibus_packet, sizeof(IbusPacket_t));
        rb->ibus_ready = false;
        taskEXIT_CRITICAL();
        return true;
    }

    return false;
}

/**
 * @brief 清除IBUS数据包标志
 */
void Protocol_ClearIbusPacket(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    if (rb != NULL)
    {
        rb->ibus_ready = false;
        rb->ibus_packet.is_valid = false;
    }
}

/**
 * @brief 检查是否有完整的裁判系统数据包
 */
bool Protocol_HasRefereePacket(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return false;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    if (rb == NULL)
        return false;

    return (rb->protocol_type == PROTOCOL_REFEREE && rb->referee_pkt_count > 0);
}

/**
 * @brief 获取裁判系统数据包
 */
bool Protocol_GetRefereePacket(BSP_UART_NUM_e uart_num, RefereePacket_t *packet)
{
    if (uart_num >= BSP_UART_MAX || packet == NULL)
        return false;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    if (rb == NULL)
        return false;

    if (rb->protocol_type == PROTOCOL_REFEREE && rb->referee_pkt_count > 0)
    {
        taskENTER_CRITICAL();
        memcpy(packet, &rb->referee_packet_queue[rb->referee_pkt_tail], sizeof(RefereePacket_t));
        rb->referee_pkt_tail = (rb->referee_pkt_tail + 1) % REFEREE_PACKET_QUEUE_SIZE;
        rb->referee_pkt_count--;
        RefereeProtocolDebug_QueuePopCount++;
        taskEXIT_CRITICAL();
        return true;
    }

    return false;
}

/**
 * @brief 清除裁判系统数据包标志
 */
void Protocol_ClearRefereePacket(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    if (rb != NULL)
    {
        taskENTER_CRITICAL();
        rb->referee_pkt_head = 0;
        rb->referee_pkt_tail = 0;
        rb->referee_pkt_count = 0;
        taskEXIT_CRITICAL();
        RefereeProtocolDebug_ClearCount++;
        Protocol_ResetRefereeStream(rb);
    }
}

/**
 * @brief 获取有效帧数量（舵机协议）
 */
uint8_t Protocol_GetFrameCount(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return 0;

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num);
    if (rb == NULL)
        return 0;

    if (rb->protocol_type == PROTOCOL_SERVO_MCU || rb->protocol_type == PROTOCOL_SERVO_NO_MCU)
    {
        return rb->servo_pkt_count;
    }
    return 0;
}
