#ifndef __BSP_PWR_H_ /* 按 __BSP_PWR_H_ 选择编译分支。 */
#define __BSP_PWR_H_ /* 定义 __BSP_PWR_H_。 */

#include "main.h"
#include "gpio.h"
#include <stdbool.h>

/// @brief 给A板上的24V使能
/// @param 无
/// @return true使能成功，false使能失败
bool BSP_POWER_Init(void); /* 声明 BSP_POWER_Init 接口。 */

/// @brief 给A板上的24V失能
/// @param 无
/// @return true使能成功，false使能失败
bool BSP_POWER_DeInit(void); /* 声明 BSP_POWER_DeInit 接口。 */

#endif /* 结束条件编译。 */
