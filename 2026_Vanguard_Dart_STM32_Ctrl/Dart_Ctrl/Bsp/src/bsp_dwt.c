/**
 ******************************************************************************
 * @file	bsp_dwt.c
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

#include "bsp_dwt.h"

DWT_clock_t system_time; /* 保存 system_time。 */
static uint32_t CPU_frequency, CPU_frequency_ms, CPU_frequency_us; /* 完成本行操作。 */
static uint32_t CYCCNT_round_count; /* 保存 CYCCNT_round_count。 */
static uint32_t CYCCNT_last; /* 保存 CYCCNT_last。 */
uint64_t CYCCNT64; /* 保存 CYCCNT64。 */

static void DWT_CNT_Update(void) /* 实现 DWT_CNT_Update。 */
{
	static volatile uint8_t bit_locker = 0; /* 初始化 bit_locker。 */
	if (!bit_locker) /* 检查当前执行条件。 */
	{
		bit_locker = 1; /* 更新 bit_locker。 */

		volatile uint32_t cnt_now = DWT->CYCCNT; /* 初始化 cnt_now。 */

		if (cnt_now < CYCCNT_last) /* 检查当前执行条件。 */
		{
			CYCCNT_round_count++; /* 递增 CYCCNT_round_count。 */
		}

		CYCCNT_last = cnt_now; /* 定义 CYCCNT_last 枚举项。 */

		bit_locker = 0; /* 更新 bit_locker。 */
	}
}

void DWT_Init(uint32_t CPU_Freq_mHz) /* 实现 DWT_Init。 */
{
	/* 使能DWT外设 */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; /* 更新 DEMCR。 */

	/* DWT CYCCNT寄存器计数清0 */
	DWT->CYCCNT = (uint32_t)0u; /* 更新 CYCCNT。 */

	/* 使能Cortex-M DWT CYCCNT寄存器 */
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; /* 更新 CTRL。 */

	CPU_frequency = CPU_Freq_mHz * 1000000; /* 定义 CPU_frequency 枚举项。 */
	CPU_frequency_ms = CPU_frequency / 1000; /* 定义 CPU_frequency_ms 枚举项。 */
	CPU_frequency_us = CPU_frequency / 1000000; /* 定义 CPU_frequency_us 枚举项。 */
	CYCCNT_round_count = 0; /* 定义 CYCCNT_round_count 枚举项。 */

	DWT_CNT_Update(); /* 调用 DWT_CNT_Update。 */
}

// 运行总时长
static void DWT_System_Time_Update(void) /* 实现 DWT_System_Time_Update。 */
{
	volatile uint32_t cnt_now = DWT->CYCCNT; /* 初始化 cnt_now。 */
	static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3; /* 完成本行操作。 */

	DWT_CNT_Update(); /* 调用 DWT_CNT_Update。 */

	CYCCNT64 = (uint64_t)CYCCNT_round_count * (uint64_t)UINT32_MAX + (uint64_t)cnt_now; /* 定义 CYCCNT64 枚举项。 */
	CNT_TEMP1 = CYCCNT64 / CPU_frequency; /* 定义 CNT_TEMP1 枚举项。 */
	CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_frequency; /* 定义 CNT_TEMP2 枚举项。 */
	system_time.s = CNT_TEMP1; /* 更新 s。 */
	system_time.ms = CNT_TEMP2 / CPU_frequency_ms; /* 更新 ms。 */
	CNT_TEMP3 = CNT_TEMP2 - system_time.ms * CPU_frequency_ms; /* 定义 CNT_TEMP3 枚举项。 */
	system_time.us = CNT_TEMP3 / CPU_frequency_us; /* 更新 us。 */
}

// 获取秒（float）
float DWT_GetDeltaT(uint32_t *cnt_last) /* 实现 DWT_GetDeltaT。 */
{
	volatile uint32_t cnt_now = DWT->CYCCNT; /* 初始化 cnt_now。 */
	float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_frequency)); /* 初始化 dt。 */
	*cnt_last = cnt_now; /* 更新 cnt_last。 */

	DWT_CNT_Update(); /* 调用 DWT_CNT_Update。 */

	return dt; /* 返回当前计算结果。 */
}

// 获取秒(double)
double DWT_GetDeltaT64(uint32_t *cnt_last) /* 实现 DWT_GetDeltaT64。 */
{
	volatile uint32_t cnt_now = DWT->CYCCNT; /* 初始化 cnt_now。 */
	double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(CPU_frequency)); /* 初始化 dt。 */
	*cnt_last = cnt_now; /* 更新 cnt_last。 */

	DWT_CNT_Update(); /* 调用 DWT_CNT_Update。 */

	return dt; /* 返回当前计算结果。 */
}

// 获取运行总时长(s)
float DWT_GetTimeline_s(void) /* 实现 DWT_GetTimeline_s。 */
{
	DWT_System_Time_Update(); /* 调用 DWT_System_Time_Update。 */

	float DWT_Timelinef32 = system_time.s + system_time.ms * 0.001f + system_time.us * 0.000001f; /* 初始化 DWT_Timelinef32。 */

	return DWT_Timelinef32; /* 返回当前计算结果。 */
}

// 获取运行总时长(ms)
float DWT_GetTimeline_ms(void) /* 实现 DWT_GetTimeline_ms。 */
{
	DWT_System_Time_Update(); /* 调用 DWT_System_Time_Update。 */

	float DWT_Timelinef32 = system_time.s * 1000 + system_time.ms + system_time.us * 0.001f; /* 初始化 DWT_Timelinef32。 */

	return DWT_Timelinef32; /* 返回当前计算结果。 */
}

// 获取运行总时长（us）
uint64_t DWT_GetTimeline_us(void) /* 实现 DWT_GetTimeline_us。 */
{
	DWT_System_Time_Update(); /* 调用 DWT_System_Time_Update。 */

	uint64_t DWT_Timelinef32 = system_time.s * 1000000 + system_time.ms * 1000 + system_time.us; /* 初始化 DWT_Timelinef32。 */

	return DWT_Timelinef32; /* 返回当前计算结果。 */
}

// 延时s
void DWT_Delay(float Delay) /* 实现 DWT_Delay。 */
{
	uint32_t tickstart = DWT->CYCCNT; /* 初始化 tickstart。 */
	float wait = Delay; /* 初始化 wait。 */

	while ((DWT->CYCCNT - tickstart) < wait * (float)CPU_frequency) /* 条件满足时继续执行。 */
	{
		; /* 结束当前语句。 */
	}
}

/**
 * @brief  微秒级延时
 * @param  us: 需要延时的微秒数 (0-16777215)
 * @note   最大延时时间受CPU频率限制:
 *         - 100MHz: 最长约16.7秒 (0xFFFFFFFF / 100)
 *         - 200MHz: 最长约8.3秒
 */
void DWT_Delay_us(uint32_t us) /* 实现 DWT_Delay_us。 */
{
	// 计算需要等待的CPU周期数
	uint32_t wait_cycles = us * CPU_frequency_us; /* 初始化 wait_cycles。 */

	// 处理超长延时（理论上不会触发，但作为安全保护）
	if (wait_cycles > 0xFFFFFFFE) /* 检查当前执行条件。 */
		wait_cycles = 0xFFFFFFFE; /* 更新 wait_cycles。 */

	uint32_t start = DWT->CYCCNT; /* 初始化 start。 */
	while ((DWT->CYCCNT - start) < wait_cycles) /* 条件满足时继续执行。 */
		; /* 结束当前语句。 */
}

/**
 * @brief  毫秒级延时
 * @param  ms: 需要延时的毫秒数 (0-4294967)
 * @note   最大延时时间:
 *         - 100MHz: 最长约4294秒
 */
void DWT_Delay_ms(uint32_t ms) /* 实现 DWT_Delay_ms。 */
{
	// 计算需要等待的CPU周期数
	uint32_t wait_cycles = ms * CPU_frequency_ms; /* 初始化 wait_cycles。 */

	// 处理超长延时（理论上不会触发，但作为安全保护）
	if (wait_cycles > 0xFFFFFFFE) /* 检查当前执行条件。 */
		wait_cycles = 0xFFFFFFFE; /* 更新 wait_cycles。 */

	uint32_t start = DWT->CYCCNT; /* 初始化 start。 */
	while ((DWT->CYCCNT - start) < wait_cycles) /* 条件满足时继续执行。 */
		; /* 结束当前语句。 */
}
