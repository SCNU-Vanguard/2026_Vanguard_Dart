/**
 * @file UartProtocol.h
 * @brief UART协议解析模块头文件
 * @note  本模块从bsp_uart中分离出来，负责协议解析相关功能
 *        与底层HAL库无强关联
 */

#ifndef __UART_PROTOCOL_H
#define __UART_PROTOCOL_H

#include "main.h"
#include "bsp_uart.h"
#include <stdbool.h>
#include <stdint.h>

extern volatile uint32_t RefereeProtocolDebug_SofCount;
extern volatile uint32_t RefereeProtocolDebug_HeaderCrcFailCount;
extern volatile uint32_t RefereeProtocolDebug_FrameCrcFailCount;
extern volatile uint32_t RefereeProtocolDebug_InvalidLengthCount;
extern volatile uint32_t RefereeProtocolDebug_FrameOverflowCount;
extern volatile uint32_t RefereeProtocolDebug_FrameOkCount;
extern volatile uint32_t RefereeProtocolDebug_QueuePushCount;
extern volatile uint32_t RefereeProtocolDebug_QueueFullDropCount;
extern volatile uint32_t RefereeProtocolDebug_QueuePopCount;
extern volatile uint32_t RefereeProtocolDebug_ClearCount;
extern volatile uint16_t RefereeProtocolDebug_LastDataLen;
extern volatile uint16_t RefereeProtocolDebug_LastFrameLen;
extern volatile uint16_t RefereeProtocolDebug_LastCmdID;

/* ========== 协议解析API ========== */

/**
 * @brief CRC校验生成（供其他模块使用）
 * @param data 数据指针
 * @param length 数据长度
 * @return CRC校验码
 */
uint8_t Protocol_Calculate_CRC(uint8_t *data, uint8_t length);

/**
 * @brief 设置协议类型
 * @param uart_num UART编号
 * @param protocol_type 协议类型
 */
void Protocol_SetType(BSP_UART_NUM_e uart_num, PROTOCOL_TYPE_e protocol_type);

/**
 * @brief 检查是否有完整的数据包
 * @param uart_num UART编号
 * @return true-有完整数据包，false-无
 */
bool Protocol_HasPacket(BSP_UART_NUM_e uart_num);

/**
 * @brief 获取舵机协议数据包
 * @param uart_num UART编号
 * @param packet 数据包指针
 * @return true-获取成功，false-失败
 */
bool Protocol_GetServoPacket(BSP_UART_NUM_e uart_num, ServoPacket_t *packet);

/**
 * @brief 清除舵机数据包标志
 * @param uart_num UART编号
 */
void Protocol_ClearPacket(BSP_UART_NUM_e uart_num);

/**
 * @brief 检查是否有完整的IBUS数据包
 * @param uart_num UART编号
 * @return true-有完整数据包，false-无
 */
bool Protocol_HasIbusPacket(BSP_UART_NUM_e uart_num);

/**
 * @brief 获取IBUS数据包
 * @param uart_num UART编号
 * @param packet 数据包指针
 * @return true-获取成功，false-失败
 */
bool Protocol_GetIbusPacket(BSP_UART_NUM_e uart_num, IbusPacket_t *packet);

/**
 * @brief 清除IBUS数据包标志
 * @param uart_num UART编号
 */
void Protocol_ClearIbusPacket(BSP_UART_NUM_e uart_num);

/**
 * @brief 检查是否有完整的裁判系统数据包
 * @param uart_num UART编号
 * @return true-有完整数据包，false-无
 */
bool Protocol_HasRefereePacket(BSP_UART_NUM_e uart_num);

/**
 * @brief 获取裁判系统数据包
 * @param uart_num UART编号
 * @param packet 数据包指针
 * @return true-获取成功，false-失败
 */
bool Protocol_GetRefereePacket(BSP_UART_NUM_e uart_num, RefereePacket_t *packet);

/**
 * @brief 清除裁判系统数据包标志
 * @param uart_num UART编号
 */
void Protocol_ClearRefereePacket(BSP_UART_NUM_e uart_num);

/**
 * @brief 获取有效帧数量（舵机协议）
 * @param uart_num UART编号
 * @return 有效帧数量
 */
uint8_t Protocol_GetFrameCount(BSP_UART_NUM_e uart_num);

/* ========== 内部函数（供bsp_uart.c调用） ========== */

/**
 * @brief 重置协议解析器状态
 * @param rb 接收缓冲区指针
 */
void Protocol_ResetParserState(UartRxRingBuffer *rb);

/**
 * @brief 完全重置协议解析器
 * @param rb 接收缓冲区指针
 */
void Protocol_ResetParser(UartRxRingBuffer *rb);

/**
 * @brief 协议解析状态机
 * @param rb 接收缓冲区指针
 * @param byte 接收到的字节
 */
void Protocol_ParseByte(UartRxRingBuffer *rb, uint8_t byte);

/**
 * @brief 解析IBUS数据流（DMA事件回调中调用）
 * @param rb 接收缓冲区指针
 * @param data 原始接收数据
 * @param len 数据长度
 */
void Protocol_ParseIbusStream(UartRxRingBuffer *rb, const uint8_t *data, uint16_t len);

#endif /* __UART_PROTOCOL_H */
