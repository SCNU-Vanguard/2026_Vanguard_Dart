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
 * @brief 获取DART协议数据包
 * @param uart_num UART编号
 * @param packet 数据包指针
 * @return true-获取成功，false-失败
 */
bool Protocol_GetDartPacket(BSP_UART_NUM_e uart_num, DartPacket_t *packet);

/**
 * @brief 清除数据包标志
 * @param uart_num UART编号
 */
void Protocol_ClearPacket(BSP_UART_NUM_e uart_num);

/**
 * @brief 获取有效帧数量
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
 * @brief DataBuffer初始化
 * @param db DataBuffer指针
 */
void Protocol_DataBufferInit(DataBuffer *db);

/**
 * @brief 帧头检测函数
 * @param db DataBuffer指针
 * @param rb RxRingBuffer指针
 * @param byte 当前接收到的字节
 * @param currentPos 当前字节位置
 */
void Protocol_DetectFrameHeader(DataBuffer *db, UartRxRingBuffer *rb, uint8_t byte, uint8_t currentPos);

/**
 * @brief 记录帧尾位置
 * @param db DataBuffer指针
 * @param rb RxRingBuffer指针
 * @param tailPos 帧尾位置
 */
void Protocol_RecordFrameTail(DataBuffer *db, UartRxRingBuffer *rb, uint8_t tailPos);

/**
 * @brief 使被覆盖位置的帧无效化
 * @param db DataBuffer指针
 * @param rb RxRingBuffer指针
 * @param overwritePos 被覆盖的位置
 */
void Protocol_InvalidateOverwrittenFrames(DataBuffer *db, UartRxRingBuffer *rb, uint8_t overwritePos);

/**
 * @brief 移动读指针到下一个有效帧头位置
 * @param db DataBuffer指针
 * @param rb RxRingBuffer指针
 */
void Protocol_MoveReadIndexToNextFrame(DataBuffer *db, UartRxRingBuffer *rb);

#endif /* __UART_PROTOCOL_H */
