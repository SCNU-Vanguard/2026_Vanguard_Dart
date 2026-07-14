#ifndef __IA6B_H_ /* 按 __IA6B_H_ 选择编译分支。 */
#define __IA6B_H_ /* 定义 __IA6B_H_。 */

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
extern int16_t RawChannel[13]; /* 声明外部变量 RawChannel。 */

/**
 * @brief IBUS解析后通道数据 (-1/0/1)
 * @note Channel[0-3]: 摇杆方向, Channel[4-5]: 开关状态
 */
extern int8_t Channel[13]; /* 声明外部变量 Channel。 */

/**
 * @brief 解析IBUS通道数据到Channel数组
 * @param data 28字节通道数据指针（不含帧头和校验）
 * @note  使用方式：IA6B_HandleData2Channel(&ibus_packet.data[2])
 */
void IA6B_HandleData2Channel(uint8_t *data); /* 声明 IA6B_HandleData2Channel 接口。 */

/**
 * @brief 读取指定通道的值
 * @param ChannelNum 通道号 (1-17)
 * @return 通道值
 */
int16_t IA6B_ReadChannel(uint8_t ChannelNum); /* 声明 IA6B_ReadChannel 接口。 */

/**
 * @brief 处理IBUS数据包并解析通道（便捷函数）
 * @param uart_num UART编号 (通常为BSP_UART6)
 * @return true-成功获取并解析, false-无数据包
 * @note 在主循环或任务中周期性调用
 * @example
 *   // 在任务中:
 *   if (IA6B_ProcessIbusPacket(BSP_UART6)) {
 *       int16_t ch1 = IA6B_ReadChannel(1);
 *       int16_t ch2 = IA6B_ReadChannel(2);
 *   }
 */
bool IA6B_ProcessIbusPacket(BSP_UART_NUM_e uart_num); /* 声明 IA6B_ProcessIbusPacket 接口。 */

#endif /* 结束条件编译。 */
