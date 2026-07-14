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
extern UartRxRingBuffer *BSP_UART_GetRxBuffer(BSP_UART_NUM_e uart_num); /* 声明 BSP_UART_GetRxBuffer 接口。 */

#define REFEREE_SOF_BYTE 0xA5U /* 定义 REFEREE_SOF_BYTE。 */
#define REFEREE_HEADER_LEN 5U /* 定义 REFEREE_HEADER_LEN。 */
#define REFEREE_CMD_ID_LEN 2U /* 定义 REFEREE_CMD_ID_LEN。 */
#define REFEREE_TAIL_LEN 2U /* 定义 REFEREE_TAIL_LEN。 */

volatile uint32_t RefereeProtocolDebug_SofCount = 0U; /* 初始化 RefereeProtocolDebug_SofCount。 */
volatile uint32_t RefereeProtocolDebug_HeaderCrcFailCount = 0U; /* 初始化 RefereeProtocolDebug_HeaderCrcFailCount。 */
volatile uint32_t RefereeProtocolDebug_FrameCrcFailCount = 0U; /* 初始化 RefereeProtocolDebug_FrameCrcFailCount。 */
volatile uint32_t RefereeProtocolDebug_InvalidLengthCount = 0U; /* 初始化 RefereeProtocolDebug_InvalidLengthCount。 */
volatile uint32_t RefereeProtocolDebug_FrameOverflowCount = 0U; /* 初始化 RefereeProtocolDebug_FrameOverflowCount。 */
volatile uint32_t RefereeProtocolDebug_FrameOkCount = 0U; /* 初始化 RefereeProtocolDebug_FrameOkCount。 */
volatile uint32_t RefereeProtocolDebug_QueuePushCount = 0U; /* 初始化 RefereeProtocolDebug_QueuePushCount。 */
volatile uint32_t RefereeProtocolDebug_QueueFullDropCount = 0U; /* 初始化 RefereeProtocolDebug_QueueFullDropCount。 */
volatile uint32_t RefereeProtocolDebug_QueuePopCount = 0U; /* 初始化 RefereeProtocolDebug_QueuePopCount。 */
volatile uint32_t RefereeProtocolDebug_ClearCount = 0U; /* 初始化 RefereeProtocolDebug_ClearCount。 */
volatile uint16_t RefereeProtocolDebug_LastDataLen = 0U; /* 初始化 RefereeProtocolDebug_LastDataLen。 */
volatile uint16_t RefereeProtocolDebug_LastFrameLen = 0U; /* 初始化 RefereeProtocolDebug_LastFrameLen。 */
volatile uint16_t RefereeProtocolDebug_LastCmdID = 0U; /* 初始化 RefereeProtocolDebug_LastCmdID。 */

static void Protocol_ResetRefereeStream(UartRxRingBuffer *rb) /* 实现 Protocol_ResetRefereeStream。 */
{
    if (rb == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    rb->referee_frame_len = 0; /* 更新 referee_frame_len。 */
    rb->referee_expected_len = 0; /* 更新 referee_expected_len。 */
}
static void Protocol_StartRefereeFrame(UartRxRingBuffer *rb, uint8_t byte) /* 实现 Protocol_StartRefereeFrame。 */
{
    if (rb == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    RefereeProtocolDebug_SofCount++; /* 递增 RefereeProtocolDebug_SofCount。 */
    rb->referee_frame[0] = byte; /* 更新 referee_frame。 */
    rb->referee_frame_len = 1; /* 更新 referee_frame_len。 */
    rb->referee_expected_len = 0; /* 更新 referee_expected_len。 */
}

static void Protocol_ResyncReferee(UartRxRingBuffer *rb, uint8_t byte) /* 实现 Protocol_ResyncReferee。 */
{
    if (rb == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    if (byte == REFEREE_SOF_BYTE) /* 检查当前执行条件。 */
    {
        Protocol_StartRefereeFrame(rb, byte); /* 调用 Protocol_StartRefereeFrame。 */
    }
    else /* 处理其余情况。 */
    {
        Protocol_ResetRefereeStream(rb); /* 调用 Protocol_ResetRefereeStream。 */
    }
}
static void Protocol_QueueRefereePacket(UartRxRingBuffer *rb) /* 实现 Protocol_QueueRefereePacket。 */
{
    RefereePacket_t *packet; /* 保存 packet。 */

    if (rb == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    if (rb->referee_pkt_count >= REFEREE_PACKET_QUEUE_SIZE) /* 检查当前执行条件。 */
    {
        RefereeProtocolDebug_QueueFullDropCount++; /* 递增 RefereeProtocolDebug_QueueFullDropCount。 */
        return; /* 结束当前函数。 */
    }

    packet = &rb->referee_packet_queue[rb->referee_pkt_head]; /* 更新 packet。 */
    memcpy(packet->data, rb->referee_frame, rb->referee_expected_len); /* 调用 memcpy。 */
    packet->length = rb->referee_expected_len; /* 更新 length。 */
    packet->is_valid = true; /* 更新 is_valid。 */
    rb->referee_pkt_head = (rb->referee_pkt_head + 1) % REFEREE_PACKET_QUEUE_SIZE; /* 更新 referee_pkt_head。 */
    rb->referee_pkt_count++; /* 完成本行操作。 */
    RefereeProtocolDebug_QueuePushCount++; /* 递增 RefereeProtocolDebug_QueuePushCount。 */
}

static void Protocol_ParseRefereeByte(UartRxRingBuffer *rb, uint8_t byte) /* 实现 Protocol_ParseRefereeByte。 */
{
    if (rb == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    if (rb->referee_frame_len == 0) /* 检查当前执行条件。 */
    {
        if (byte == REFEREE_SOF_BYTE) /* 检查当前执行条件。 */
        {
            Protocol_StartRefereeFrame(rb, byte); /* 调用 Protocol_StartRefereeFrame。 */
        }
        return; /* 结束当前函数。 */
    }

    if (rb->referee_frame_len >= REFEREE_FRAME_MAX_SIZE) /* 检查当前执行条件。 */
    {
        RefereeProtocolDebug_FrameOverflowCount++; /* 递增 RefereeProtocolDebug_FrameOverflowCount。 */
        Protocol_ResyncReferee(rb, byte); /* 调用 Protocol_ResyncReferee。 */
        return; /* 结束当前函数。 */
    }

    rb->referee_frame[rb->referee_frame_len++] = byte; /* 更新 referee_frame。 */

    if (rb->referee_frame_len == REFEREE_HEADER_LEN) /* 检查当前执行条件。 */
    {
        uint16_t data_len; /* 保存 data_len。 */
        uint16_t total_len; /* 保存 total_len。 */

        if (!Verify_CRC8_Check_Sum(rb->referee_frame, REFEREE_HEADER_LEN)) /* 检查当前执行条件。 */
        {
            RefereeProtocolDebug_HeaderCrcFailCount++; /* 递增 RefereeProtocolDebug_HeaderCrcFailCount。 */
            Protocol_ResyncReferee(rb, byte); /* 调用 Protocol_ResyncReferee。 */
            return; /* 结束当前函数。 */
        }

        data_len = (uint16_t)rb->referee_frame[1] | ((uint16_t)rb->referee_frame[2] << 8); /* 更新 data_len。 */
        total_len = (uint16_t)(data_len + REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN + REFEREE_TAIL_LEN); /* 更新 total_len。 */
        RefereeProtocolDebug_LastDataLen = data_len; /* 定义 RefereeProtocolDebug_LastDataLen 枚举项。 */
        RefereeProtocolDebug_LastFrameLen = total_len; /* 定义 RefereeProtocolDebug_LastFrameLen 枚举项。 */
        if (total_len < (REFEREE_HEADER_LEN + REFEREE_CMD_ID_LEN + REFEREE_TAIL_LEN) || /* 检查当前执行条件。 */
            total_len > REFEREE_FRAME_MAX_SIZE) /* 继续当前语句。 */
        {
            RefereeProtocolDebug_InvalidLengthCount++; /* 递增 RefereeProtocolDebug_InvalidLengthCount。 */
            Protocol_ResyncReferee(rb, byte); /* 调用 Protocol_ResyncReferee。 */
            return; /* 结束当前函数。 */
        }

        rb->referee_expected_len = total_len; /* 更新 referee_expected_len。 */
    }

    if (rb->referee_expected_len > 0 && rb->referee_frame_len == rb->referee_expected_len) /* 检查当前执行条件。 */
    {
        if (Verify_CRC16_Check_Sum(rb->referee_frame, rb->referee_expected_len)) /* 检查当前执行条件。 */
        {
            RefereeProtocolDebug_FrameOkCount++; /* 递增 RefereeProtocolDebug_FrameOkCount。 */
            RefereeProtocolDebug_LastCmdID = (uint16_t)rb->referee_frame[5] | ((uint16_t)rb->referee_frame[6] << 8); /* 定义 RefereeProtocolDebug_LastCmdID 枚举项。 */
            Protocol_QueueRefereePacket(rb); /* 调用 Protocol_QueueRefereePacket。 */
        }
        else /* 处理其余情况。 */
        {
            RefereeProtocolDebug_FrameCrcFailCount++; /* 递增 RefereeProtocolDebug_FrameCrcFailCount。 */
        }
        Protocol_ResetRefereeStream(rb); /* 调用 Protocol_ResetRefereeStream。 */
    }
}

/**
 * @brief CRC校验生成（公开函数，供所有模块使用）
 * @param data 数据指针
 * @param length 数据长度
 * @return CRC校验码
 */
uint8_t Protocol_Calculate_CRC(uint8_t *data, uint8_t length) /* 实现 Protocol_Calculate_CRC。 */
{
    uint16_t sum = 0; /* 初始化 sum。 */
    for (uint8_t i = 0; i < length; i++) /* 遍历当前数据集合。 */
    {
        sum += data[i]; /* 更新 sum。 */
    }
    return (uint8_t)(~sum); /* 返回当前计算结果。 */
}

/**
 * @brief 重置协议解析器状态（不清除数据包内容）
 */
void Protocol_ResetParserState(UartRxRingBuffer *rb) /* 实现 Protocol_ResetParserState。 */
{
    if (rb == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    rb->parse_state = PARSE_HEADER; /* 更新 parse_state。 */
    rb->parse_index = 0; /* 更新 parse_index。 */
}

/**
 * @brief 完全重置协议解析器（清除数据包内容和状态）
 * @note 仅在初始化和协议切换时使用
 */
void Protocol_ResetParser(UartRxRingBuffer *rb) /* 实现 Protocol_ResetParser。 */
{
    if (rb == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    rb->parse_state = PARSE_HEADER; /* 更新 parse_state。 */
    rb->parse_index = 0; /* 更新 parse_index。 */
    memset(&rb->servo_packet_temp, 0, sizeof(ServoPacket_t)); /* 调用 memset。 */
    memset(rb->servo_packet_queue, 0, sizeof(rb->servo_packet_queue)); /* 调用 memset。 */
    rb->servo_pkt_head = 0; /* 更新 servo_pkt_head。 */
    rb->servo_pkt_tail = 0; /* 更新 servo_pkt_tail。 */
    rb->servo_pkt_count = 0; /* 更新 servo_pkt_count。 */
    rb->ibus_packet.is_valid = false; /* 更新 is_valid。 */
    rb->ibus_ready = false; /* 更新 ibus_ready。 */
    rb->ibus_error_count = 0; /* 更新 ibus_error_count。 */
    rb->ibus_stream_len = 0; /* 更新 ibus_stream_len。 */
    memset(rb->ibus_stream, 0, sizeof(rb->ibus_stream)); /* 调用 memset。 */
    memset(rb->referee_packet_queue, 0, sizeof(rb->referee_packet_queue)); /* 调用 memset。 */
    memset(rb->referee_frame, 0, sizeof(rb->referee_frame)); /* 调用 memset。 */
    rb->referee_pkt_head = 0; /* 更新 referee_pkt_head。 */
    rb->referee_pkt_tail = 0; /* 更新 referee_pkt_tail。 */
    rb->referee_pkt_count = 0; /* 更新 referee_pkt_count。 */
    rb->referee_frame_len = 0; /* 更新 referee_frame_len。 */
    rb->referee_expected_len = 0; /* 更新 referee_expected_len。 */
}

/**
 * @brief 协议解析状态机（舵机协议）
 * @param rb 接收缓冲区指针
 * @param byte 接收到的字节
 */
void Protocol_ParseByte(UartRxRingBuffer *rb, uint8_t byte) /* 实现 Protocol_ParseByte。 */
{
    if (rb == NULL || rb->protocol_type == PROTOCOL_IBUS) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    if (rb->protocol_type == PROTOCOL_REFEREE) /* 检查当前执行条件。 */
    {
        Protocol_ParseRefereeByte(rb, byte); /* 调用 Protocol_ParseRefereeByte。 */
        return; /* 结束当前函数。 */
    }

    if (rb->protocol_type == PROTOCOL_SERVO_MCU) /* 检查当前执行条件。 */
    {
        // 有MCU控制板协议解析：0x55 0x55 | Length | Cmd | Params（无CRC）
        switch (rb->parse_state) /* 按当前状态选择处理分支。 */
        {
        case PARSE_HEADER: /* 处理 PARSE_HEADER 分支。 */
            if (rb->parse_index == 0) /* 检查当前执行条件。 */
            {
                if (byte == 0x55) /* 检查当前执行条件。 */
                {
                    rb->servo_packet_temp.header[0] = byte; /* 更新 header。 */
                    rb->parse_index = 1; /* 更新 parse_index。 */
                }
            }
            else if (rb->parse_index == 1) /* 继续判断下一条件。 */
            {
                if (byte == 0x55) /* 检查当前执行条件。 */
                {
                    rb->servo_packet_temp.header[1] = byte; /* 更新 header。 */
                    rb->parse_state = PARSE_SERVO_LENGTH; /* 更新 parse_state。 */
                    rb->parse_index = 0; /* 更新 parse_index。 */
                }
                else /* 处理其余情况。 */
                {
                    Protocol_ResetParserState(rb); /* 调用 Protocol_ResetParserState。 */
                }
            }
            break; /* 结束当前循环或分支。 */

        case PARSE_SERVO_LENGTH: /* 处理 PARSE_SERVO_LENGTH 分支。 */
            rb->servo_packet_temp.length = byte; /* 更新 length。 */
            if (byte >= 2 && byte <= 64) /* 检查当前执行条件。 */
            {
                rb->parse_state = PARSE_SERVO_CMD; /* 更新 parse_state。 */
            }
            else /* 处理其余情况。 */
            {
                Protocol_ResetParserState(rb); /* 调用 Protocol_ResetParserState。 */
            }
            break; /* 结束当前循环或分支。 */

        case PARSE_SERVO_CMD: /* 处理 PARSE_SERVO_CMD 分支。 */
            rb->servo_packet_temp.cmd = byte; /* 更新 cmd。 */
            rb->servo_packet_temp.param_len = rb->servo_packet_temp.length - 2; /* 更新 param_len。 */
            if (rb->servo_packet_temp.param_len > 0) /* 检查当前执行条件。 */
            {
                rb->parse_state = PARSE_SERVO_PARAMS; /* 更新 parse_state。 */
                rb->parse_index = 0; /* 更新 parse_index。 */
            }
            else /* 处理其余情况。 */
            {
                rb->servo_packet_temp.is_valid = true; /* 更新 is_valid。 */
                if (rb->servo_pkt_count < SERVO_PACKET_QUEUE_SIZE) /* 检查当前执行条件。 */
                {
                    rb->servo_packet_queue[rb->servo_pkt_head] = rb->servo_packet_temp; /* 更新 servo_packet_queue。 */
                    rb->servo_pkt_head = (rb->servo_pkt_head + 1) % SERVO_PACKET_QUEUE_SIZE; /* 更新 servo_pkt_head。 */
                    rb->servo_pkt_count++; /* 完成本行操作。 */
                }
                Protocol_ResetParserState(rb); /* 调用 Protocol_ResetParserState。 */
            }
            break; /* 结束当前循环或分支。 */

        case PARSE_SERVO_PARAMS: /* 处理 PARSE_SERVO_PARAMS 分支。 */
            if (rb->parse_index < rb->servo_packet_temp.param_len && rb->parse_index < 8) /* 检查当前执行条件。 */
            {
                rb->servo_packet_temp.params[rb->parse_index++] = byte; /* 更新 params。 */
                if (rb->parse_index >= rb->servo_packet_temp.param_len) /* 检查当前执行条件。 */
                {
                    rb->servo_packet_temp.is_valid = true; /* 更新 is_valid。 */
                    if (rb->servo_pkt_count < SERVO_PACKET_QUEUE_SIZE) /* 检查当前执行条件。 */
                    {
                        rb->servo_packet_queue[rb->servo_pkt_head] = rb->servo_packet_temp; /* 更新 servo_packet_queue。 */
                        rb->servo_pkt_head = (rb->servo_pkt_head + 1) % SERVO_PACKET_QUEUE_SIZE; /* 更新 servo_pkt_head。 */
                        rb->servo_pkt_count++; /* 完成本行操作。 */
                    }
                    Protocol_ResetParserState(rb); /* 调用 Protocol_ResetParserState。 */
                }
            }
            else /* 处理其余情况。 */
            {
                Protocol_ResetParserState(rb); /* 调用 Protocol_ResetParserState。 */
            }
            break; /* 结束当前循环或分支。 */

        default: /* 处理默认分支。 */
            Protocol_ResetParserState(rb); /* 调用 Protocol_ResetParserState。 */
            break; /* 结束当前循环或分支。 */
        }
    }
    else if (rb->protocol_type == PROTOCOL_SERVO_NO_MCU) /* 继续判断下一条件。 */
    {
        // 无MCU驱动板协议解析：0x55 0x55 | ID | Length | Cmd | Params | CRC（有CRC）
        switch (rb->parse_state) /* 按当前状态选择处理分支。 */
        {
        case PARSE_HEADER: /* 处理 PARSE_HEADER 分支。 */
            if (rb->parse_index == 0) /* 检查当前执行条件。 */
            {
                if (byte == 0x55) /* 检查当前执行条件。 */
                {
                    rb->servo_packet_temp.header[0] = byte; /* 更新 header。 */
                    rb->parse_index = 1; /* 更新 parse_index。 */
                }
            }
            else if (rb->parse_index == 1) /* 继续判断下一条件。 */
            {
                if (byte == 0x55) /* 检查当前执行条件。 */
                {
                    rb->servo_packet_temp.header[1] = byte; /* 更新 header。 */
                    rb->parse_state = PARSE_SERVO_ID; /* 更新 parse_state。 */
                    rb->parse_index = 0; /* 更新 parse_index。 */
                }
                else /* 处理其余情况。 */
                {
                    Protocol_ResetParserState(rb); /* 调用 Protocol_ResetParserState。 */
                }
            }
            break; /* 结束当前循环或分支。 */

        case PARSE_SERVO_ID: /* 处理 PARSE_SERVO_ID 分支。 */
            rb->servo_packet_temp.id = byte; /* 更新 id。 */
            rb->parse_state = PARSE_SERVO_LENGTH; /* 更新 parse_state。 */
            break; /* 结束当前循环或分支。 */

        case PARSE_SERVO_LENGTH: /* 处理 PARSE_SERVO_LENGTH 分支。 */
            rb->servo_packet_temp.length = byte; /* 更新 length。 */
            if (byte >= 3 && byte <= 11) /* 检查当前执行条件。 */
            {
                rb->parse_state = PARSE_SERVO_CMD; /* 更新 parse_state。 */
            }
            else /* 处理其余情况。 */
            {
                Protocol_ResetParserState(rb); /* 调用 Protocol_ResetParserState。 */
            }
            break; /* 结束当前循环或分支。 */

        case PARSE_SERVO_CMD: /* 处理 PARSE_SERVO_CMD 分支。 */
            rb->servo_packet_temp.cmd = byte; /* 更新 cmd。 */
            rb->servo_packet_temp.param_len = rb->servo_packet_temp.length - 3; /* 更新 param_len。 */
            if (rb->servo_packet_temp.param_len > 0) /* 检查当前执行条件。 */
            {
                rb->parse_state = PARSE_SERVO_PARAMS; /* 更新 parse_state。 */
                rb->parse_index = 0; /* 更新 parse_index。 */
            }
            else /* 处理其余情况。 */
            {
                rb->parse_state = PARSE_SERVO_CRC; /* 更新 parse_state。 */
            }
            break; /* 结束当前循环或分支。 */

        case PARSE_SERVO_PARAMS: /* 处理 PARSE_SERVO_PARAMS 分支。 */
            if (rb->parse_index < rb->servo_packet_temp.param_len && rb->parse_index < 8) /* 检查当前执行条件。 */
            {
                rb->servo_packet_temp.params[rb->parse_index++] = byte; /* 更新 params。 */
                if (rb->parse_index >= rb->servo_packet_temp.param_len) /* 检查当前执行条件。 */
                {
                    rb->parse_state = PARSE_SERVO_CRC; /* 更新 parse_state。 */
                }
            }
            else /* 处理其余情况。 */
            {
                Protocol_ResetParserState(rb); /* 调用 Protocol_ResetParserState。 */
            }
            break; /* 结束当前循环或分支。 */

        case PARSE_SERVO_CRC: /* 处理 PARSE_SERVO_CRC 分支。 */
        {
            rb->servo_packet_temp.checksum = byte; /* 更新 checksum。 */
            uint8_t temp_buf[16]; /* 保存 temp_buf。 */
            temp_buf[0] = rb->servo_packet_temp.id; /* 更新 temp_buf。 */
            temp_buf[1] = rb->servo_packet_temp.length; /* 更新 temp_buf。 */
            temp_buf[2] = rb->servo_packet_temp.cmd; /* 更新 temp_buf。 */
            for (uint8_t i = 0; i < rb->servo_packet_temp.param_len; i++) /* 遍历当前数据集合。 */
            {
                temp_buf[3 + i] = rb->servo_packet_temp.params[i]; /* 更新 temp_buf。 */
            }
            uint8_t calc_crc = Protocol_Calculate_CRC(temp_buf, 3 + rb->servo_packet_temp.param_len); /* 初始化 calc_crc。 */

            if (calc_crc == byte) /* 检查当前执行条件。 */
            {
                rb->servo_packet_temp.is_valid = true; /* 更新 is_valid。 */
                if (rb->servo_pkt_count < SERVO_PACKET_QUEUE_SIZE) /* 检查当前执行条件。 */
                {
                    rb->servo_packet_queue[rb->servo_pkt_head] = rb->servo_packet_temp; /* 更新 servo_packet_queue。 */
                    rb->servo_pkt_head = (rb->servo_pkt_head + 1) % SERVO_PACKET_QUEUE_SIZE; /* 更新 servo_pkt_head。 */
                    rb->servo_pkt_count++; /* 完成本行操作。 */
                }
                Protocol_ResetParserState(rb); /* 调用 Protocol_ResetParserState。 */
            }
            else /* 处理其余情况。 */
            {
                Protocol_ResetParserState(rb); /* 调用 Protocol_ResetParserState。 */
            }
            break; /* 结束当前循环或分支。 */
        }

        default: /* 处理默认分支。 */
            Protocol_ResetParserState(rb); /* 调用 Protocol_ResetParserState。 */
            break; /* 结束当前循环或分支。 */
        }
    }
    else /* 处理其余情况。 */
    {
        Protocol_ResetParserState(rb); /* 调用 Protocol_ResetParserState。 */
    }
}

/**
 * @brief 计算IBUS校验（0xFFFF - sum[0..len-3]）
 */
static uint16_t Protocol_IbusChecksum(const uint8_t *frame, uint16_t len) /* 实现 Protocol_IbusChecksum。 */
{
    uint32_t sum = 0; /* 初始化 sum。 */
    for (uint16_t i = 0; i < (len - 2); i++) /* 遍历当前数据集合。 */
    {
        sum += frame[i]; /* 更新 sum。 */
    }
    return (uint16_t)(0xFFFF - (uint16_t)sum); /* 返回当前计算结果。 */
}

/**
 * @brief 校验IBUS帧：帧头 + 16位校验
 */
static bool Protocol_IbusValidate(const uint8_t *frame, uint16_t len) /* 实现 Protocol_IbusValidate。 */
{
    if (frame == NULL || len != IBUS_FRAME_LEN) /* 检查当前执行条件。 */
        return false; /* 返回 false。 */
    if (frame[0] != IBUS_LENGTH || frame[1] != IBUS_COMMAND) /* 检查当前执行条件。 */
        return false; /* 返回 false。 */

    uint16_t expect = Protocol_IbusChecksum(frame, len); /* 初始化 expect。 */
    uint16_t recv = (uint16_t)frame[len - 2] | ((uint16_t)frame[len - 1] << 8); /* 初始化 recv。 */
    return expect == recv; /* 返回当前计算结果。 */
}

/**
 * @brief 追加IBUS数据流到缓存
 */
static void Protocol_IbusStreamAppend(UartRxRingBuffer *rb, const uint8_t *data, uint16_t len) /* 实现 Protocol_IbusStreamAppend。 */
{
    if (len > IBUS_STREAM_BUFFER_LEN) /* 检查当前执行条件。 */
    {
        data += (len - IBUS_STREAM_BUFFER_LEN); /* 更新 data。 */
        len = IBUS_STREAM_BUFFER_LEN; /* 更新 len。 */
    }

    if (rb->ibus_stream_len + len > IBUS_STREAM_BUFFER_LEN) /* 检查当前执行条件。 */
    {
        uint16_t overflow = (rb->ibus_stream_len + len) - IBUS_STREAM_BUFFER_LEN; /* 初始化 overflow。 */
        memmove(rb->ibus_stream, rb->ibus_stream + overflow, rb->ibus_stream_len - overflow); /* 调用 memmove。 */
        rb->ibus_stream_len -= overflow; /* 更新 ibus_stream_len。 */
    }

    memcpy(rb->ibus_stream + rb->ibus_stream_len, data, len); /* 调用 memcpy。 */
    rb->ibus_stream_len += len; /* 更新 ibus_stream_len。 */
}

/**
 * @brief 解析IBUS数据流（DMA事件回调中调用）
 * @param rb 接收缓冲区指针
 * @param data 原始接收数据
 * @param len 数据长度
 */
void Protocol_ParseIbusStream(UartRxRingBuffer *rb, const uint8_t *data, uint16_t len) /* 实现 Protocol_ParseIbusStream。 */
{
    if (rb == NULL || data == NULL || len == 0) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    // 快速路径：DMA 收到的刚好是完整帧，跳过 stream append 和 memmove
    if (len == IBUS_FRAME_LEN && rb->ibus_stream_len == 0 && /* 检查当前执行条件。 */
        data[0] == IBUS_LENGTH && data[1] == IBUS_COMMAND) /* 继续更新 目标值。 */
    {
        if (Protocol_IbusValidate(data, IBUS_FRAME_LEN)) /* 检查当前执行条件。 */
        {
            memcpy(rb->ibus_packet.data, data, IBUS_FRAME_LEN); /* 调用 memcpy。 */
            rb->ibus_packet.is_valid = true; /* 更新 is_valid。 */
            rb->ibus_ready = true; /* 更新 ibus_ready。 */
            return; /* 结束当前函数。 */
        }
    }

    // 慢速路径：流式解析
    Protocol_IbusStreamAppend(rb, data, len); /* 调用 Protocol_IbusStreamAppend。 */

    uint16_t offset = 0; /* 初始化 offset。 */
    while (rb->ibus_stream_len - offset >= IBUS_FRAME_LEN) /* 条件满足时继续执行。 */
    {
        const uint8_t *candidate = rb->ibus_stream + offset; /* 初始化 candidate。 */
        if (candidate[0] == IBUS_LENGTH && candidate[1] == IBUS_COMMAND) /* 检查当前执行条件。 */
        {
            if (Protocol_IbusValidate(candidate, IBUS_FRAME_LEN)) /* 检查当前执行条件。 */
            {
                memcpy(rb->ibus_packet.data, candidate, IBUS_FRAME_LEN); /* 调用 memcpy。 */
                rb->ibus_packet.is_valid = true; /* 更新 is_valid。 */
                rb->ibus_ready = true; /* 更新 ibus_ready。 */
                offset += IBUS_FRAME_LEN; /* 更新 offset。 */
                continue; /* 跳过本轮剩余处理。 */
            }
            rb->ibus_error_count++; /* 完成本行操作。 */
        }
        offset += 1; /* 更新 offset。 */
    }

    if (offset > 0) /* 检查当前执行条件。 */
    {
        memmove(rb->ibus_stream, rb->ibus_stream + offset, rb->ibus_stream_len - offset); /* 调用 memmove。 */
        rb->ibus_stream_len -= offset; /* 更新 ibus_stream_len。 */
    }
}

/**
 * @brief 设置协议类型
 */
void Protocol_SetType(BSP_UART_NUM_e uart_num, PROTOCOL_TYPE_e protocol_type) /* 实现 Protocol_SetType。 */
{
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num); /* 初始化 rb。 */
    if (rb != NULL) /* 检查当前执行条件。 */
    {
        rb->protocol_type = protocol_type; /* 更新 protocol_type。 */
        Protocol_ResetParser(rb); /* 调用 Protocol_ResetParser。 */
    }
}

/**
 * @brief 检查是否有完整的舵机数据包
 */
bool Protocol_HasPacket(BSP_UART_NUM_e uart_num) /* 实现 Protocol_HasPacket。 */
{
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return false; /* 返回 false。 */

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num); /* 初始化 rb。 */
    return (rb != NULL) ? (rb->servo_pkt_count > 0) : false; /* 返回当前计算结果。 */
}

/**
 * @brief 获取舵机协议数据包
 */
bool Protocol_GetServoPacket(BSP_UART_NUM_e uart_num, ServoPacket_t *packet) /* 实现 Protocol_GetServoPacket。 */
{
    if (uart_num >= BSP_UART_MAX || packet == NULL) /* 检查当前执行条件。 */
        return false; /* 返回 false。 */

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num); /* 初始化 rb。 */
    if (rb == NULL) /* 检查当前执行条件。 */
        return false; /* 返回 false。 */

    bool is_servo_protocol = (rb->protocol_type == PROTOCOL_SERVO_MCU || /* 开始计算 is_servo_protocol。 */
                              rb->protocol_type == PROTOCOL_SERVO_NO_MCU); /* 更新 protocol_type。 */

    if (is_servo_protocol && rb->servo_pkt_count > 0) /* 检查当前执行条件。 */
    {
        taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
        memcpy(packet, &rb->servo_packet_queue[rb->servo_pkt_tail], sizeof(ServoPacket_t)); /* 调用 memcpy。 */
        rb->servo_pkt_tail = (rb->servo_pkt_tail + 1) % SERVO_PACKET_QUEUE_SIZE; /* 更新 servo_pkt_tail。 */
        rb->servo_pkt_count--; /* 完成本行操作。 */
        taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
        return true; /* 返回 true。 */
    }

    return false; /* 返回 false。 */
}

/**
 * @brief 清除舵机数据包标志
 */
void Protocol_ClearPacket(BSP_UART_NUM_e uart_num) /* 实现 Protocol_ClearPacket。 */
{
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num); /* 初始化 rb。 */
    if (rb != NULL) /* 检查当前执行条件。 */
    {
        taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
        rb->servo_pkt_head = 0; /* 更新 servo_pkt_head。 */
        rb->servo_pkt_tail = 0; /* 更新 servo_pkt_tail。 */
        rb->servo_pkt_count = 0; /* 更新 servo_pkt_count。 */
        taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
    }
}

/**
 * @brief 检查是否有完整的IBUS数据包
 */
bool Protocol_HasIbusPacket(BSP_UART_NUM_e uart_num) /* 实现 Protocol_HasIbusPacket。 */
{
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return false; /* 返回 false。 */

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num); /* 初始化 rb。 */
    if (rb == NULL) /* 检查当前执行条件。 */
        return false; /* 返回 false。 */

    return (rb->protocol_type == PROTOCOL_IBUS && rb->ibus_ready && rb->ibus_packet.is_valid); /* 返回当前计算结果。 */
}

/**
 * @brief 获取IBUS数据包
 */
bool Protocol_GetIbusPacket(BSP_UART_NUM_e uart_num, IbusPacket_t *packet) /* 实现 Protocol_GetIbusPacket。 */
{
    if (uart_num >= BSP_UART_MAX || packet == NULL) /* 检查当前执行条件。 */
        return false; /* 返回 false。 */

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num); /* 初始化 rb。 */
    if (rb == NULL) /* 检查当前执行条件。 */
        return false; /* 返回 false。 */

    if (rb->protocol_type == PROTOCOL_IBUS && rb->ibus_ready && rb->ibus_packet.is_valid) /* 检查当前执行条件。 */
    {
        taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
        memcpy(packet, &rb->ibus_packet, sizeof(IbusPacket_t)); /* 调用 memcpy。 */
        rb->ibus_ready = false; /* 更新 ibus_ready。 */
        taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
        return true; /* 返回 true。 */
    }

    return false; /* 返回 false。 */
}

/**
 * @brief 清除IBUS数据包标志
 */
void Protocol_ClearIbusPacket(BSP_UART_NUM_e uart_num) /* 实现 Protocol_ClearIbusPacket。 */
{
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num); /* 初始化 rb。 */
    if (rb != NULL) /* 检查当前执行条件。 */
    {
        rb->ibus_ready = false; /* 更新 ibus_ready。 */
        rb->ibus_packet.is_valid = false; /* 更新 is_valid。 */
    }
}

/**
 * @brief 检查是否有完整的裁判系统数据包
 */
bool Protocol_HasRefereePacket(BSP_UART_NUM_e uart_num) /* 实现 Protocol_HasRefereePacket。 */
{
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return false; /* 返回 false。 */

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num); /* 初始化 rb。 */
    if (rb == NULL) /* 检查当前执行条件。 */
        return false; /* 返回 false。 */

    return (rb->protocol_type == PROTOCOL_REFEREE && rb->referee_pkt_count > 0); /* 返回当前计算结果。 */
}

/**
 * @brief 获取裁判系统数据包
 */
bool Protocol_GetRefereePacket(BSP_UART_NUM_e uart_num, RefereePacket_t *packet) /* 实现 Protocol_GetRefereePacket。 */
{
    if (uart_num >= BSP_UART_MAX || packet == NULL) /* 检查当前执行条件。 */
        return false; /* 返回 false。 */

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num); /* 初始化 rb。 */
    if (rb == NULL) /* 检查当前执行条件。 */
        return false; /* 返回 false。 */

    if (rb->protocol_type == PROTOCOL_REFEREE && rb->referee_pkt_count > 0) /* 检查当前执行条件。 */
    {
        taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
        memcpy(packet, &rb->referee_packet_queue[rb->referee_pkt_tail], sizeof(RefereePacket_t)); /* 调用 memcpy。 */
        rb->referee_pkt_tail = (rb->referee_pkt_tail + 1) % REFEREE_PACKET_QUEUE_SIZE; /* 更新 referee_pkt_tail。 */
        rb->referee_pkt_count--; /* 完成本行操作。 */
        RefereeProtocolDebug_QueuePopCount++; /* 递增 RefereeProtocolDebug_QueuePopCount。 */
        taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
        return true; /* 返回 true。 */
    }

    return false; /* 返回 false。 */
}

/**
 * @brief 清除裁判系统数据包标志
 */
void Protocol_ClearRefereePacket(BSP_UART_NUM_e uart_num) /* 实现 Protocol_ClearRefereePacket。 */
{
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    UartRxRingBuffer *rb = BSP_UART_GetRxBuffer(uart_num); /* 初始化 rb。 */
    if (rb != NULL) /* 检查当前执行条件。 */
    {
        taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
        rb->referee_pkt_head = 0; /* 更新 referee_pkt_head。 */
        rb->referee_pkt_tail = 0; /* 更新 referee_pkt_tail。 */
        rb->referee_pkt_count = 0; /* 更新 referee_pkt_count。 */
        taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */
        RefereeProtocolDebug_ClearCount++; /* 递增 RefereeProtocolDebug_ClearCount。 */
        Protocol_ResetRefereeStream(rb); /* 调用 Protocol_ResetRefereeStream。 */
    }
}
