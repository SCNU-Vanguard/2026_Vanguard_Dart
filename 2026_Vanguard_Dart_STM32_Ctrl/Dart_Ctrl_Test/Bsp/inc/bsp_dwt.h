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

#ifndef __BSP_DWT_H__
#define __BSP_DWT_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "main.h"

#define TIME_ELAPSE(dt, code)                \
	do                                       \
	{                                        \
		float t_start = DWT_GetTimeline_s(); \
		code;                                \
		dt = DWT_GetTimeline_s() - t_start;  \
	} while (0)

	typedef struct
	{
		uint32_t s;
		uint16_t ms;
		uint16_t us;
	} DWT_clock_t;

	void DWT_Init(uint32_t CPU_Freq_mHz);

	float DWT_GetDeltaT(uint32_t *cnt_last);

	double DWT_GetDeltaT64(uint32_t *cnt_last);

	float DWT_GetTimeline_s(void);

	float DWT_GetTimeline_ms(void);

	uint64_t DWT_GetTimeline_us(void);

	void DWT_Delay(float Delay);

	void DWT_Delay_us(uint32_t us);

	void DWT_Delay_ms(uint32_t ms);

	extern DWT_clock_t system_time;

#ifdef __cplusplus
}
#endif

#endif /* __BSP_DWT_H__ */
