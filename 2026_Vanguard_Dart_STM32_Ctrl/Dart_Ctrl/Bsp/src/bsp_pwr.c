#include "bsp_pwr.h"

/// @brief 给A板上的24V使能
/// @param 无
/// @return true使能成功，false使能失败
bool BSP_POWER_Init(void)
{
    if (GPIO_PIN_RESET == (HAL_GPIO_ReadPin(PWR1_GPIO_Port, PWR1_Pin) && HAL_GPIO_ReadPin(PWR2_GPIO_Port, PWR2_Pin) && HAL_GPIO_ReadPin(PWR3_GPIO_Port, PWR3_Pin) && HAL_GPIO_ReadPin(PWR4_GPIO_Port, PWR4_Pin)))
    {
        HAL_GPIO_WritePin(PWR1_GPIO_Port, PWR1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PWR2_GPIO_Port, PWR2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PWR3_GPIO_Port, PWR3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PWR4_GPIO_Port, PWR4_Pin, GPIO_PIN_SET);

        HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin, GPIO_PIN_RESET);
        return true;
    }
    else if (GPIO_PIN_SET == (HAL_GPIO_ReadPin(PWR1_GPIO_Port, PWR1_Pin) && HAL_GPIO_ReadPin(PWR2_GPIO_Port, PWR2_Pin) && HAL_GPIO_ReadPin(PWR3_GPIO_Port, PWR3_Pin) && HAL_GPIO_ReadPin(PWR4_GPIO_Port, PWR4_Pin)))
    {
        HAL_GPIO_WritePin(Green_GPIO_Port, Green_Pin, GPIO_PIN_SET);
        return true;
    }
    return false;
}
