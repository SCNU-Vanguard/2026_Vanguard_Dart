/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "UserTask.h"
#include "HX06L.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// 测试函数：使用阻塞方式直接发送舵机指令
// 无MCU驱动板协议：0x55 0x55 | ID | Length | Cmd | Param... | CRC
void Test_Servo_Blocking(void)
{
    uint8_t data[16];
    uint8_t crc;
    
    // ============ 第一步：舵机上电 (Load) ============
    // 指令：SERVO_LOAD_OR_UNLOAD_WRITE (cmd=31, length=4)
    data[0] = 0x55;  // 帧头
    data[1] = 0x55;  // 帧头
    data[2] = 0x01;  // ID = 1
    data[3] = 0x04;  // Length = 4
    data[4] = 31;    // Cmd = SERVO_LOAD_OR_UNLOAD_WRITE
    data[5] = 0x01;  // 参数: 1 = 上电
    
    // 计算 CRC (从 ID 到最后一个参数)
    crc = data[2] + data[3] + data[4] + data[5];
    crc = ~crc;
    data[6] = crc;
    
    HAL_UART_Transmit(&huart3, data, 7, 100);
    HAL_Delay(100);  // 等待舵机处理
    
    // ============ 第二步：控制舵机转动到位置500 ============
    // 指令：SERVO_MOVE_TIME_WRITE (cmd=1, length=7)
    data[0] = 0x55;  // 帧头
    data[1] = 0x55;  // 帧头
    data[2] = 0x01;  // ID = 1
    data[3] = 0x07;  // Length = 7
    data[4] = 1;     // Cmd = SERVO_MOVE_TIME_WRITE
    data[5] = 0xF4;  // 角度低字节 (500 = 0x01F4)
    data[6] = 0x01;  // 角度高字节
    data[7] = 0xE8;  // 时间低字节 (1000ms = 0x03E8)
    data[8] = 0x03;  // 时间高字节
    
    crc = data[2] + data[3] + data[4] + data[5] + data[6] + data[7] + data[8];
    crc = ~crc;
    data[9] = crc;
    
    HAL_UART_Transmit(&huart3, data, 10, 100);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_UART7_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_UART8_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  // 对各个外设进行初始化
  Module_Init();

  /* USER CODE END 2 */

  /* Init scheduler */
  // osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  // MX_FREERTOS_Init();

  /* Start scheduler */
  // osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // ====== 方式二测试：延时控制（同步运动）======
    
    // 动作1：三个舵机同步移动到不同位置
    // 先设置目标位置（舵机不动）
//    ServoMoveTimeWaitWrite(1, 500, 1000);   // 舵机1 -> 120°
//    ServoMoveTimeWaitWrite(2, 300, 1000);   // 舵机2 -> 72°
//    ServoMoveTimeWaitWrite(3, 700, 1000);   // 舵机3 -> 168°
//    HAL_Delay(10);  // 短暂延时确保指令发送完成
//    
//    // 同时启动所有舵机
//    ServoStart(1);
//    ServoStart(2);
//    ServoStart(3);
//    HAL_Delay(1200);  // 等待运动完成
//    
//    // 动作2：三个舵机同步回到起始位置
//    ServoMoveTimeWaitWrite(1, 0, 1000);     // 舵机1 -> 0°
//    ServoMoveTimeWaitWrite(2, 0, 1000);     // 舵机2 -> 0°
//    ServoMoveTimeWaitWrite(3, 0, 1000);     // 舵机3 -> 0°
//    HAL_Delay(10);
//    
//    ServoStart(1);
//    ServoStart(2);
//    ServoStart(3);
//    HAL_Delay(1200);
//    
//    // 动作3：三个舵机同步移动到最大位置
//    ServoMoveTimeWaitWrite(1, 1000, 1500);  // 舵机1 -> 240°
//    ServoMoveTimeWaitWrite(2, 1000, 1500);  // 舵机2 -> 240°
//    ServoMoveTimeWaitWrite(3, 1000, 1500);  // 舵机3 -> 240°
//    HAL_Delay(10);
//    
//    ServoStart(1);
//    ServoStart(2);
//    ServoStart(3);
//    HAL_Delay(1700);
//    
//    // 动作4：三个舵机同步回中间位置
//    ServoMoveTimeWaitWrite(1, 500, 1000);   // 舵机1 -> 120°
//    ServoMoveTimeWaitWrite(2, 500, 1000);   // 舵机2 -> 120°
//    ServoMoveTimeWaitWrite(3, 500, 1000);   // 舵机3 -> 120°
//    HAL_Delay(10);
//    
//    ServoStart(1);
//    ServoStart(2);
//    ServoStart(3);
//    HAL_Delay(1200);
    ServoInit();
		HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
