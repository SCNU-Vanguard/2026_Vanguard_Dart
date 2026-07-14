#include "bsp_pwr.h"

/// @brief 给A板上的24V使能
/// @param 无
/// @return true使能成功，false使能失败
bool BSP_POWER_Init(void) /* 实现 BSP_POWER_Init。 */
{
    if (GPIO_PIN_RESET == (HAL_GPIO_ReadPin(PWR1_GPIO_Port, PWR1_Pin) && HAL_GPIO_ReadPin(PWR2_GPIO_Port, PWR2_Pin) && HAL_GPIO_ReadPin(PWR3_GPIO_Port, PWR3_Pin) && HAL_GPIO_ReadPin(PWR4_GPIO_Port, PWR4_Pin))) /* 检查当前执行条件。 */
    {
        HAL_GPIO_WritePin(PWR1_GPIO_Port, PWR1_Pin, GPIO_PIN_SET); /* 调用 HAL_GPIO_WritePin。 */
        HAL_GPIO_WritePin(PWR2_GPIO_Port, PWR2_Pin, GPIO_PIN_SET); /* 调用 HAL_GPIO_WritePin。 */
        HAL_GPIO_WritePin(PWR3_GPIO_Port, PWR3_Pin, GPIO_PIN_SET); /* 调用 HAL_GPIO_WritePin。 */
        HAL_GPIO_WritePin(PWR4_GPIO_Port, PWR4_Pin, GPIO_PIN_SET); /* 调用 HAL_GPIO_WritePin。 */

        HAL_Delay(5); /* 调用 HAL_Delay。 */
        return true; /* 返回 true。 */
    }
    else if (GPIO_PIN_SET == (HAL_GPIO_ReadPin(PWR1_GPIO_Port, PWR1_Pin) && HAL_GPIO_ReadPin(PWR2_GPIO_Port, PWR2_Pin) && HAL_GPIO_ReadPin(PWR3_GPIO_Port, PWR3_Pin) && HAL_GPIO_ReadPin(PWR4_GPIO_Port, PWR4_Pin))) /* 继续判断下一条件。 */
    {
        HAL_Delay(5); /* 调用 HAL_Delay。 */
        return true; /* 返回 true。 */
    }
    HAL_Delay(5); /* 调用 HAL_Delay。 */
    return false; /* 返回 false。 */
}

/// @brief 给A板上的24V失能
/// @param 无
/// @return true使能成功，false使能失败
bool BSP_POWER_DeInit(void) /* 实现 BSP_POWER_DeInit。 */
{
    if (GPIO_PIN_SET == (HAL_GPIO_ReadPin(PWR1_GPIO_Port, PWR1_Pin) && HAL_GPIO_ReadPin(PWR2_GPIO_Port, PWR2_Pin) && HAL_GPIO_ReadPin(PWR3_GPIO_Port, PWR3_Pin) && HAL_GPIO_ReadPin(PWR4_GPIO_Port, PWR4_Pin))) /* 检查当前执行条件。 */
    {
        HAL_GPIO_WritePin(PWR1_GPIO_Port, PWR1_Pin, GPIO_PIN_RESET); /* 调用 HAL_GPIO_WritePin。 */
        HAL_GPIO_WritePin(PWR2_GPIO_Port, PWR2_Pin, GPIO_PIN_RESET); /* 调用 HAL_GPIO_WritePin。 */
        HAL_GPIO_WritePin(PWR3_GPIO_Port, PWR3_Pin, GPIO_PIN_RESET); /* 调用 HAL_GPIO_WritePin。 */
        HAL_GPIO_WritePin(PWR4_GPIO_Port, PWR4_Pin, GPIO_PIN_RESET); /* 调用 HAL_GPIO_WritePin。 */

        HAL_Delay(5); /* 调用 HAL_Delay。 */
        return true; /* 返回 true。 */
    }
    else if (GPIO_PIN_RESET == (HAL_GPIO_ReadPin(PWR1_GPIO_Port, PWR1_Pin) && HAL_GPIO_ReadPin(PWR2_GPIO_Port, PWR2_Pin) && HAL_GPIO_ReadPin(PWR3_GPIO_Port, PWR3_Pin) && HAL_GPIO_ReadPin(PWR4_GPIO_Port, PWR4_Pin))) /* 继续判断下一条件。 */
    {
        HAL_Delay(5); /* 调用 HAL_Delay。 */
        return true; /* 返回 true。 */
    }
    HAL_Delay(5); /* 调用 HAL_Delay。 */
    return false; /* 返回 false。 */
}
