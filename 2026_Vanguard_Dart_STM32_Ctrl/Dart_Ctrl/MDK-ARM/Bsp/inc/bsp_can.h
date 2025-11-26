#ifndef __BSP_CAN_H_
#define __BSP_CAN_H_

#include "main.h"
#include "can.h"
#include <stdio.h>
#include <string.h>

/******************************************************
 * @note : 感觉宏函数版本似乎有点糖，不过一般都是用来配置的，应该没什么问题，可以把它改为正常的函数
 * @brief: 用来看你想过滤标准帧的哪一位
 * @param: Buffer : 像过滤的位的数组地址
 * @param: Length : 数组长度，也就是你想过滤的位数
 * @param: Result : 结果存储的变量
 * @todo:  也许还可以再优化
 ******************************************************/
#define PreserveBit(Buffer, Length, Result)                                                                                                   \
	do                                                                                                                                        \
	{                                                                                                                                         \
		uint16_t temp = 0x0000;                                                                                                               \
		for (uint8_t cnt = 0; cnt < Length; cnt++)                                                                                            \
		{                                                                                                                                     \
			uint16_t Mask = ((0x0000 >> ((*((volatile uint8_t *)(Buffer + cnt)) - 1))) + 1) << ((*((volatile uint8_t *)(Buffer + cnt)) - 1)); \
			temp += Mask;                                                                                                                     \
		}                                                                                                                                     \
		Result = temp;                                                                                                                        \
	} while (0)

#define g_CanMotorNum 5

// fifox register
// x could be 0 or 1
typedef enum
{
	fifo0 = CAN_FILTER_FIFO0,
	fifo1 = CAN_FILTER_FIFO1,
} CAN_FIFO;

/******************************************
 *简介: CAN初始化
 *参数: canHandle: CAN句柄
 *返回: 无
 ******************************************/
uint8_t CAN_Init(CAN_HandleTypeDef *canHandle, CAN_FIFO fifo, uint8_t FliterNum, uint8_t MaskStatus);

/******************************************
 *简介: CAN发送数据
 *参数: canHandle: CAN句柄
 *      TxHeader: 发送结构体指针
 *      data: 数据指针
 *返回: 0: 发送失败
 *      1: 发送成功
 ******************************************/
uint8_t CAN_SendData(CAN_HandleTypeDef *canHandle, CAN_TxHeaderTypeDef *TxHeader, uint8_t *data);

/// @brief 目前使用这个函数暴露接口方便简单地配置想要接收的CAN设备
/// @param canHandle CAN句柄
/// @param ID 给一个ID数组地址，数组长度大小必须为2
/// @param Mask 给一个想要过滤的位的数组
/// @param FliterNum 过滤器的位号
/// @param fifo fifo的位号
void FliterIdCfg_Init(CAN_HandleTypeDef *canHandle, uint16_t *ID, uint16_t *Mask, uint8_t FliterNum, uint8_t fifo);

#endif
