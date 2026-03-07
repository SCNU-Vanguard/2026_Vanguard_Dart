#ifndef __IA6B_H_
#define __IA6B_H_

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "config.h"
#include "bsp_uart.h"

/*============================== IBUS通道数据 ==============================*/
/**
 * @brief IBUS原始通道数据 (1000-2000)
 * @note RawChannel[0-3]: 摇杆值, RawChannel[4-5]: 开关值
 */
extern int16_t RawChannel[13];

/**
 * @brief IBUS解析后通道数据 (-1/0/1)
 * @note Channel[0-3]: 摇杆方向, Channel[4-5]: 开关状态
 */
extern int8_t Channel[13];

/**
 * @brief IA6B模块初始化
 */
void IA6B_Init(void);

/**
 * @brief 解析IBUS通道数据到Channel数组
 * @param data 28字节通道数据指针（不含帧头和校验）
 */
void IA6B_HandleData2Channel(uint8_t *data);

/**
 * @brief 读取指定通道方向值
 * @param ChannelNum 通道号 (1-13)
 * @return 通道方向值（-1/0/1）
 */
int16_t IA6B_ReadChannel(uint8_t ChannelNum);

/**
 * @brief 处理IBUS数据包并解析通道
 * @param uart_num UART编号 (通常为BSP_UART6)
 * @return true-成功获取并解析, false-无数据包
 */
bool IA6B_ProcessIbusPacket(BSP_UART_NUM_e uart_num);

#endif
