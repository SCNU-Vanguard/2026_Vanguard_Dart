/**
 ******************************************************************************
 * @file	bsp_dwt.h
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#ifndef __BSP_DWT_H__ /* 按 __BSP_DWT_H__ 选择编译分支。 */
#define __BSP_DWT_H__ /* 定义 __BSP_DWT_H__。 */

#ifdef __cplusplus /* 按 __cplusplus 选择编译分支。 */
extern "C" /* 使用 C 语言链接规则。 */
{
#endif /* 结束条件编译。 */

#include <stdint.h>
#include "main.h"

#define TIME_ELAPSE(dt, code) /* 继续当前语句。 */ \
	do /* 开始宏封装语句。 */ \
	{                                        \
		float t_start = DWT_GetTimeline_s(); /* 完成本行操作。 */ \
		code; /* 完成本行操作。 */ \
		dt = DWT_GetTimeline_s() - t_start; /* 完成本行操作。 */ \
	} while (0) /* 结束宏封装语句。 */

	typedef struct /* 开始定义数据类型。 */
	{
		uint32_t s; /* 保存 s。 */
		uint16_t ms; /* 保存 ms。 */
		uint16_t us; /* 保存 us。 */
	} DWT_clock_t; /* 结束 DWT_clock_t 类型定义。 */

	void DWT_Init(uint32_t CPU_Freq_mHz); /* 声明 DWT_Init 接口。 */

	float DWT_GetDeltaT(uint32_t *cnt_last); /* 声明 DWT_GetDeltaT 接口。 */

	double DWT_GetDeltaT64(uint32_t *cnt_last); /* 声明 DWT_GetDeltaT64 接口。 */

	float DWT_GetTimeline_s(void); /* 声明 DWT_GetTimeline_s 接口。 */

	float DWT_GetTimeline_ms(void); /* 声明 DWT_GetTimeline_ms 接口。 */

	uint64_t DWT_GetTimeline_us(void); /* 声明 DWT_GetTimeline_us 接口。 */

	void DWT_Delay(float Delay); /* 声明 DWT_Delay 接口。 */

	void DWT_Delay_us(uint32_t us); /* 声明 DWT_Delay_us 接口。 */

	void DWT_Delay_ms(uint32_t ms); /* 声明 DWT_Delay_ms 接口。 */

	extern DWT_clock_t system_time; /* 声明外部变量 system_time。 */

#ifdef __cplusplus /* 按 __cplusplus 选择编译分支。 */
}
#endif /* 结束条件编译。 */

#endif /* __BSP_DWT_H__ */
