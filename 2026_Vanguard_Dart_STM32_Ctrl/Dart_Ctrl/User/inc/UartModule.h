/**
 * @file UartModule.h
 * @brief UART模块用户接口头文件
 * @note  本模块提供数据包获取和发送的用户级API
 *        是对底层 UartProtocol 和 bsp_uart 的封装
 */

#ifndef __UART_MODULE_H
#define __UART_MODULE_H

#include "main.h"
#include "bsp_uart.h"
#include <stdbool.h>
#include <stdint.h>

/* ========== 用户级数据包接口 ========== */

/**
 * @brief 检查是否有完整的舵机数据包
 * @param uart_num UART编号
 * @return true-有完整数据包，false-无
 */
bool UartModule_HasServoPacket(BSP_UART_NUM_e uart_num);

/**
 * @brief 检查是否有完整的DART数据包
 * @param uart_num UART编号
 * @return true-有完整数据包，false-无
 */
bool UartModule_HasDartPacket(BSP_UART_NUM_e uart_num);

/**
 * @brief 获取舵机协议数据包
 * @param uart_num UART编号
 * @param packet 数据包指针
 * @return true-获取成功，false-失败
 */
bool UartModule_GetServoPacket(BSP_UART_NUM_e uart_num, ServoPacket_t *packet);

/**
 * @brief 获取DART协议数据包
 * @param uart_num UART编号
 * @param packet 数据包指针
 * @return true-获取成功，false-失败
 */
bool UartModule_GetDartPacket(BSP_UART_NUM_e uart_num, DartPacket_t *packet);

/**
 * @brief 清除数据包标志
 * @param uart_num UART编号
 */
void UartModule_ClearPacket(BSP_UART_NUM_e uart_num);

/* ========== 用户级发送接口 ========== */

/**
 * @brief 发送舵机命令数据包
 * @param uart_num UART编号
 * @param id 舵机ID
 * @param cmd 命令
 * @param params 参数数组
 * @param param_len 参数长度
 * @return true-发送成功，false-失败
 */
bool UartModule_SendServoCmd(BSP_UART_NUM_e uart_num, uint8_t id, uint8_t cmd, uint8_t *params, uint8_t param_len);

/**
 * @brief 发送DART数据包
 * @param uart_num UART编号
 * @param data 数据内容
 * @param data_len 数据长度
 * @return true-发送成功，false-失败
 */
bool UartModule_SendDartPacket(BSP_UART_NUM_e uart_num, uint8_t *data, uint16_t data_len);

/**
 * @brief 发送原始数据
 * @param uart_num UART编号
 * @param data 数据指针
 * @param len 数据长度
 * @return 实际发送字节数
 */
uint16_t UartModule_SendRaw(BSP_UART_NUM_e uart_num, const uint8_t *data, uint16_t len);

/* ========== 用户级设置接口 ========== */

/**
 * @brief 设置UART协议类型
 * @param uart_num UART编号
 * @param protocol_type 协议类型
 */
void UartModule_SetProtocol(BSP_UART_NUM_e uart_num, PROTOCOL_TYPE_e protocol_type);

/**
 * @brief 获取有效帧数量
 * @param uart_num UART编号
 * @return 有效帧数量
 */
uint8_t UartModule_GetFrameCount(BSP_UART_NUM_e uart_num);

#endif /* __UART_MODULE_H */
