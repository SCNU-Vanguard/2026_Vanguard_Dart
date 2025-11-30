#ifndef __BSP_PWR_H_
#define __BSP_PWR_H_

#include "main.h"
#include "gpio.h"
#include <stdbool.h>

/// @brief 给A板上的24V使能
/// @param 无
/// @return true使能成功，false使能失败
bool BSP_POWER_Init(void);

#endif
