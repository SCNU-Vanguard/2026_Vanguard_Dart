/****************************************************************************
 * 使用本工程文件必看
 * 本工程文件默认基于标准帧和标准ID
 * 以下是标准帧和标准ID的通信的完整数据帧示意图
 * 如果你要配置过滤的ID必须看下面这个，否则可以直接跳过
 * -------------------------------------------------------------------------
 * 一共有两个16位过滤器，这里示例只讲一个
 * CAN_FxR1[15 :  8]       |       CAN_FxR1[7  :  0]        ->过滤器1示意图
 * CAN_FxR1[31 : 24]       |       CAN_FxR1[23 : 16]        ->过滤器1示意图
 *
 * 对应位的映像：
 * STD[10 :  3]            |   STD[2:0] [RTR] [IDE] [EXID[17:15]]
 * note:看位数即可，注意是要求哪一位相同
 * 下面是一些位的注释:  IDE(是否为扩展ID，我们不用扩展ID)
 *                    RTR（是什么帧，远程还是数据）
 *                    EXID(扩展ID的用不上，不用管)
 * TODO:等待将文件测试是否会出现其他神奇情况，如无直接更新到对应的稳定的bsp_can.c
 ****************************************************************************/

#include "bsp_can.h"
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define CAN_TX_MAILBOX_WAIT_TIMEOUT_MS 5U
#define CAN_TX_COMPLETE_TIMEOUT_MS 5U
#define CAN_TX_RETRY_MAX_ATTEMPTS 10U
#define CAN_TX_RETRY_DELAY_MS 3U
#define CAN_TX_RETRY_INTERFRAME_DELAY_MS 3U
#define CAN_TX_MUTEX_WAIT_MS 50U

static StaticSemaphore_t s_can_tx_mutex_buffer;
static SemaphoreHandle_t s_can_tx_mutex = NULL;

static SemaphoreHandle_t CAN_GetTxMutex(void)
{
  if (s_can_tx_mutex == NULL)
  {
    s_can_tx_mutex = xSemaphoreCreateMutexStatic(&s_can_tx_mutex_buffer);
  }

  return s_can_tx_mutex;
}

static uint8_t CAN_TakeTxMutex(CAN_TxRetryMode retry_mode)
{
  SemaphoreHandle_t mutex = CAN_GetTxMutex();

  if (mutex == NULL)
  {
    return 0U;
  }

  if (xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
  {
    return 1U;
  }

  if (retry_mode == CAN_TX_RETRY_ENABLE)
  {
    return (xSemaphoreTake(mutex, pdMS_TO_TICKS(CAN_TX_MUTEX_WAIT_MS)) == pdTRUE) ? 1U : 0U;
  }

  return (xSemaphoreTake(mutex, 0U) == pdTRUE) ? 1U : 0U;
}

static void CAN_GiveTxMutex(void)
{
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED && s_can_tx_mutex != NULL)
  {
    xSemaphoreGive(s_can_tx_mutex);
  }
}

static void CAN_TxDelayMs(uint32_t delay_ms)
{
  if (delay_ms == 0U)
  {
    return;
  }

  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
    vTaskDelay(pdMS_TO_TICKS(delay_ms));
  }
  else
  {
    HAL_Delay(delay_ms);
  }
}

static uint8_t CAN_WaitTxMailboxFree(CAN_HandleTypeDef *canHandle)
{
  uint32_t start_tick = HAL_GetTick();

  while (!HAL_CAN_GetTxMailboxesFreeLevel(canHandle))
  {
    if ((uint32_t)(HAL_GetTick() - start_tick) >= CAN_TX_MAILBOX_WAIT_TIMEOUT_MS)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t CAN_GetTxMailboxStatus(uint32_t tx_mailbox, uint32_t *rqcp_flag, uint32_t *txok_flag, uint32_t *clear_flag)
{
  if (rqcp_flag == NULL || txok_flag == NULL || clear_flag == NULL)
  {
    return 0U;
  }

  switch (tx_mailbox)
  {
  case CAN_TX_MAILBOX0:
    *rqcp_flag = CAN_TSR_RQCP0;
    *txok_flag = CAN_TSR_TXOK0;
    *clear_flag = CAN_FLAG_RQCP0;
    return 1U;
  case CAN_TX_MAILBOX1:
    *rqcp_flag = CAN_TSR_RQCP1;
    *txok_flag = CAN_TSR_TXOK1;
    *clear_flag = CAN_FLAG_RQCP1;
    return 1U;
  case CAN_TX_MAILBOX2:
    *rqcp_flag = CAN_TSR_RQCP2;
    *txok_flag = CAN_TSR_TXOK2;
    *clear_flag = CAN_FLAG_RQCP2;
    return 1U;
  default:
    return 0U;
  }
}

static void CAN_ClearTxMailboxStatus(CAN_HandleTypeDef *canHandle, uint32_t tx_mailbox)
{
  uint32_t rqcp_flag = 0U;
  uint32_t txok_flag = 0U;
  uint32_t clear_flag = 0U;

  (void)rqcp_flag;
  (void)txok_flag;

  if (CAN_GetTxMailboxStatus(tx_mailbox, &rqcp_flag, &txok_flag, &clear_flag))
  {
    __HAL_CAN_CLEAR_FLAG(canHandle, clear_flag);
  }
}

static void CAN_ClearAllTxMailboxStatus(CAN_HandleTypeDef *canHandle)
{
  CAN_ClearTxMailboxStatus(canHandle, CAN_TX_MAILBOX0);
  CAN_ClearTxMailboxStatus(canHandle, CAN_TX_MAILBOX1);
  CAN_ClearTxMailboxStatus(canHandle, CAN_TX_MAILBOX2);
}

static uint8_t CAN_WaitTxMailboxComplete(CAN_HandleTypeDef *canHandle, uint32_t tx_mailbox)
{
  uint32_t rqcp_flag = 0U;
  uint32_t txok_flag = 0U;
  uint32_t clear_flag = 0U;
  uint32_t start_tick = HAL_GetTick();

  if (!CAN_GetTxMailboxStatus(tx_mailbox, &rqcp_flag, &txok_flag, &clear_flag))
  {
    return 0U;
  }

  while (1)
  {
    uint32_t tsr = canHandle->Instance->TSR;

    if ((tsr & rqcp_flag) != 0U)
    {
      uint8_t tx_ok = ((tsr & txok_flag) != 0U) ? 1U : 0U;
      __HAL_CAN_CLEAR_FLAG(canHandle, clear_flag);
      return tx_ok;
    }

    if ((uint32_t)(HAL_GetTick() - start_tick) >= CAN_TX_COMPLETE_TIMEOUT_MS)
    {
      (void)HAL_CAN_AbortTxRequest(canHandle, tx_mailbox);
      CAN_ClearTxMailboxStatus(canHandle, tx_mailbox);
      return 0U;
    }
  }
}

/// @brief 目前使用这个函数暴露接口方便简单地配置想要接收的CAN设备
/// @param canHandle CAN句柄
/// @param ID 给一个ID数组地址，数组长度大小必须为2
/// @param Mask 给一个想要过滤的位的数组
/// @param FliterNum 过滤器的位号
/// @param fifo fifo的位号
/// @todo  似乎再配置失败了，需要重新更改
void FliterIdCfg_Init(CAN_HandleTypeDef *canHandle, uint16_t *ID, uint16_t *Mask, uint8_t FliterNum, uint8_t fifo)
{
  CAN_FilterTypeDef temp;
  temp.FilterBank = FliterNum;              // 过滤器编号
  temp.FilterFIFOAssignment = fifo;         // 过滤器看用户配置
  temp.FilterActivation = DISABLE;          // 要先失能过滤器
  temp.SlaveStartFilterBank = 14;           // CAN1使用0-13号过滤器 CAN2使用14-27号过滤器
  temp.FilterMode = CAN_FILTERMODE_IDMASK;  // 默认是需要过滤地址
  temp.FilterScale = CAN_FILTERSCALE_16BIT; // 默认使用16位

  temp.FilterIdHigh = ID[0];
  temp.FilterIdLow = ID[1];
  temp.FilterMaskIdHigh = Mask[0];
  temp.FilterMaskIdLow = Mask[1];

  if (HAL_CAN_ConfigFilter(canHandle, &temp) != HAL_OK)
  {
    Error_Handler(); // 配置错误
  }

  // 使能过滤器
  temp.FilterActivation = ENABLE;
  if (HAL_CAN_ConfigFilter(canHandle, &temp) != HAL_OK)
  {
    Error_Handler(); // 配置错误
  }
}

/******************************************
 *简介: CAN初始化
 *参数: canHandle: CAN句柄
 *返回: 1（uint8_t） 成功
 *note: 初始化失败则会直接进入ErrorHandler
 *todo: 尝试解决这个ID和MASK正常配置之后是否进入FIFO1中断，这里先全部不通过
 ******************************************/
uint8_t CAN_Init(CAN_HandleTypeDef *canHandle, CAN_FIFO fifo, uint8_t FliterNum, uint8_t MaskStatus)
{
  assert_param(canHandle != NULL);
  if (canHandle == NULL)
  {
    Error_Handler();
  }
  CAN_FilterTypeDef gFilterConfig;
  gFilterConfig.FilterBank = FliterNum;              // 过滤器编号
  gFilterConfig.FilterFIFOAssignment = fifo;         // 过滤器看用户配置
  gFilterConfig.FilterActivation = ENABLE;           // 激活过滤器
  gFilterConfig.SlaveStartFilterBank = 14;           // CAN1使用0-13号过滤器 CAN2使用14-27号过滤器
  gFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  // 默认是需要过滤地址
  gFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT; // 默认使用16位

  // 默认不接收总线数据
  gFilterConfig.FilterIdHigh = 0xFFFF;
  gFilterConfig.FilterIdLow = 0xFFFF;
  gFilterConfig.FilterMaskIdHigh = 0xFFFF;
  gFilterConfig.FilterMaskIdLow = 0xFFFF;

  HAL_CAN_MspInit(canHandle);

  uint8_t ConfigEnd = HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO0_MSG_PENDING);
  ConfigEnd &= HAL_CAN_ActivateNotification(canHandle, CAN_IT_RX_FIFO1_MSG_PENDING);
  if (ConfigEnd)
  {
    Error_Handler();
  }

  if (HAL_CAN_ConfigFilter(canHandle, &gFilterConfig) != HAL_OK)
  {
    Error_Handler(); // 配置错误
  }
  HAL_CAN_Start(canHandle);

  return 1;
}

/******************************************
 *简介: CAN发送数据
 *参数: canHandle: CAN句柄
 *      ID: 标识符
 *      data: 一字节数据指针
 *      len: 数据长度(最大8字节)
 *返回: 0: 发送成功
 *     -1: 发送失败
 *      1: 数据长度错误
 *
 * 注意：当CAN挂载设备多了应考虑是否直接写入缓冲区
 ******************************************/
uint8_t CAN_SendData(CAN_HandleTypeDef *canHandle, CAN_TxHeaderTypeDef *TxHeader, uint8_t *data, CAN_TxRetryMode retry_mode)
{
  uint32_t TxMailbox;

  if (canHandle == NULL || TxHeader == NULL || data == NULL)
  {
    return 0U;
  }

  if (!CAN_TakeTxMutex(retry_mode))
  {
    return 0U;
  }

  if (retry_mode == CAN_TX_RETRY_DISABLE)
  {
    if (!HAL_CAN_GetTxMailboxesFreeLevel(canHandle))
    {
      CAN_GiveTxMutex();
      return 0U;
    }

    if (HAL_CAN_AddTxMessage(canHandle, TxHeader, data, &TxMailbox) == HAL_OK)
    {
      HAL_GPIO_TogglePin(LED8_GPIO_Port, LED8_Pin);
      CAN_GiveTxMutex();
      return 1U;
    }

    HAL_GPIO_TogglePin(Red_GPIO_Port, Red_Pin);
    CAN_GiveTxMutex();
    return 0U;
  }

  const uint8_t max_attempts = (retry_mode == CAN_TX_RETRY_ENABLE) ? CAN_TX_RETRY_MAX_ATTEMPTS : 1U;

  for (uint8_t attempt = 0U; attempt < max_attempts; attempt++)
  {
    if (CAN_WaitTxMailboxFree(canHandle))
    {
      CAN_ClearAllTxMailboxStatus(canHandle);
      if (HAL_CAN_AddTxMessage(canHandle, TxHeader, data, &TxMailbox) == HAL_OK &&
          CAN_WaitTxMailboxComplete(canHandle, TxMailbox))
      {
        HAL_GPIO_TogglePin(LED8_GPIO_Port, LED8_Pin);
        CAN_TxDelayMs(CAN_TX_RETRY_INTERFRAME_DELAY_MS);
        CAN_GiveTxMutex();
        return 1U;
      }
    }

    if ((attempt + 1U) < max_attempts)
    {
      CAN_TxDelayMs(CAN_TX_RETRY_DELAY_MS);
    }
  }

  HAL_GPIO_TogglePin(Red_GPIO_Port, Red_Pin);
  CAN_GiveTxMutex();
  return 0U;
}

/******************************************
 *简介: 查看FIFO中当前数据数量
 *参数: canHandle: CAN句柄
 *参数: fifo_num： FIFO号
 *返回: FIFO当中的数据帧数量（0-3）
 *note: 可以使用另一种方法，这个也行可能精度不如while循环那个
 ******************************************/
uint8_t CAN_FIFO_DLC(CAN_HandleTypeDef *hcan, uint8_t fifo_num)
{
  if (fifo_num)
  {
    // fifo1
    return (uint8_t)((hcan->Instance->RF1R & CAN_RF1R_FMP1) >> CAN_RF1R_FMP1_Pos);
  }
  else
  {
    // fifo0
    return (uint8_t)((hcan->Instance->RF0R & CAN_RF0R_FMP0) >> CAN_RF0R_FMP0_Pos);
  }
}

/********************************************
 * while循环清空FIFO示例（RTOS的任务影响有待商榷，但是问题应该不大）
 * code(fifo0):
 * while (HAL_IS_BIT_SET(hcan->Instance->RF0R, CAN_RF0R_FMP0))
 * {
 *     HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, aData);
 * }
 *
 * code(fifo1):
 * while (HAL_IS_BIT_SET(hcan->Instance->RF1R, CAN_RF1R_FMP1))
 * {
 *     HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, aData);
 * }
 *
 * 前后两种方法都差不多，但是我更加偏向第一种调用函数的方法
 ********************************************/

extern void Error_Handler(void);
extern void CAN_FIFO_CBKHANDLER(uint32_t fifo_num, uint8_t FIFOmessageNum);

/// @todo 合并两个回调函数，进行回调的ID检验
/******************************************
 *简介: CANFIFO0接受回调
 *参数: canHandle: CAN句柄
 *返回: 无
 ******************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // 中断回调函数
{
  CAN_FIFO_CBKHANDLER(fifo0, CAN_FIFO_DLC(hcan, fifo0));
}

/******************************************
 *简介: CANFIFO1接受回调
 *参数: canHandle: CAN句柄
 *返回: 无
 ******************************************/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) // 中断回调函数
{
  CAN_FIFO_CBKHANDLER(fifo1, CAN_FIFO_DLC(hcan, fifo1));
}
