#include "bsp_uart.h"
#include "UartProtocol.h"
#include <string.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"

/* 内部缓冲区定义（用户无需关心） */
UartTxRingBuffer g_uart_tx_buffers[BSP_UART_MAX]; /* 保存 g_uart_tx_buffers。 */
UartRxRingBuffer g_uart_rx_buffers[BSP_UART_MAX]; /* 保存 g_uart_rx_buffers。 */
static uint8_t g_ibus_dma_buffer[IBUS_DMA_BUFFER_LEN]; /* 保存 g_ibus_dma_buffer。 */

volatile uint32_t Uart8RxDebug_ByteCount = 0U; /* 初始化 Uart8RxDebug_ByteCount。 */
volatile uint8_t Uart8RxDebug_LastByte = 0U; /* 初始化 Uart8RxDebug_LastByte。 */
volatile uint32_t Uart8RxDebug_LastTick = 0U; /* 初始化 Uart8RxDebug_LastTick。 */
volatile uint32_t Uart8RxDebug_RestartCount = 0U; /* 初始化 Uart8RxDebug_RestartCount。 */
volatile uint32_t Uart8RxDebug_ReceiveItFailCount = 0U; /* 初始化 Uart8RxDebug_ReceiveItFailCount。 */
volatile uint32_t Uart8RxDebug_LastReceiveStatus = HAL_OK; /* 初始化 Uart8RxDebug_LastReceiveStatus。 */
volatile uint32_t Uart8RxDebug_ErrorCount = 0U; /* 初始化 Uart8RxDebug_ErrorCount。 */
volatile uint32_t Uart8RxDebug_LastErrorCode = 0U; /* 初始化 Uart8RxDebug_LastErrorCode。 */

/* 内部函数声明 */
static void StartTransmit(UartTxRingBuffer *rb); /* 声明 StartTransmit 接口。 */
static uint16_t TxRingBuffer_Write(UartTxRingBuffer *rb, const uint8_t *data, uint16_t len); /* 声明 TxRingBuffer_Write 接口。 */
static uint16_t TxRingBuffer_Read(UartTxRingBuffer *rb, uint8_t *data, uint16_t len); /* 声明 TxRingBuffer_Read 接口。 */
static bool RxRingBuffer_WriteByte(UartRxRingBuffer *rb, uint8_t byte); /* 声明 RxRingBuffer_WriteByte 接口。 */
static uint16_t RxRingBuffer_Read(UartRxRingBuffer *rb, uint8_t *data, uint16_t len); /* 声明 RxRingBuffer_Read 接口。 */
static UART_HandleTypeDef *GetUartHandle(BSP_UART_NUM_e uart_num); /* 声明 GetUartHandle 接口。 */
static BSP_UART_NUM_e GetUartNum(UART_HandleTypeDef *huart); /* 声明 GetUartNum 接口。 */
static void StartIbusDma(UartRxRingBuffer *rb); /* 声明 StartIbusDma 接口。 */

/**
 * @brief CRC校验生成（公开函数，供所有模块使用）
 * @param data 数据指针
 * @param length 数据长度
 * @return CRC校验码
 * @note 协议解析状态机已移至 UartProtocol.c
 */
uint8_t UART_Calculate_CRC(uint8_t *data, uint8_t length) /* 实现 UART_Calculate_CRC。 */
{
    return Protocol_Calculate_CRC(data, length); /* 返回当前计算结果。 */
}

/**
 * @brief 获取UART句柄
 */
static UART_HandleTypeDef *GetUartHandle(BSP_UART_NUM_e uart_num) /* 实现 GetUartHandle。 */
{
    switch (uart_num) /* 按当前状态选择处理分支。 */
    {
    case BSP_UART3: /* 处理 BSP_UART3 分支。 */
        return &huart3; /* 返回当前计算结果。 */
    case BSP_UART6: /* 处理 BSP_UART6 分支。 */
        return &huart6; /* 返回当前计算结果。 */
    case BSP_UART7: /* 处理 BSP_UART7 分支。 */
        return &huart7; /* 返回当前计算结果。 */
    case BSP_UART8: /* 处理 BSP_UART8 分支。 */
        return &huart8; /* 返回当前计算结果。 */
    default: /* 处理默认分支。 */
        return NULL; /* 返回当前计算结果。 */
    }
}

/**
 * @brief 根据UART句柄获取UART编号
 */
static BSP_UART_NUM_e GetUartNum(UART_HandleTypeDef *huart) /* 实现 GetUartNum。 */
{
    if (huart == &huart3) /* 检查当前执行条件。 */
        return BSP_UART3; /* 返回当前计算结果。 */
    if (huart == &huart6) /* 检查当前执行条件。 */
        return BSP_UART6; /* 返回当前计算结果。 */
    if (huart == &huart7) /* 检查当前执行条件。 */
        return BSP_UART7; /* 返回当前计算结果。 */
    if (huart == &huart8) /* 检查当前执行条件。 */
        return BSP_UART8; /* 返回当前计算结果。 */
    return BSP_UART_MAX; /* 返回当前计算结果。 */
}

/**
 * @brief 向发送缓冲区写入数据
 */
static uint16_t TxRingBuffer_Write(UartTxRingBuffer *rb, const uint8_t *data, uint16_t len) /* 实现 TxRingBuffer_Write。 */
{
    if (data == NULL || len == 0) /* 检查当前执行条件。 */
        return 0; /* 返回状态值 0。 */

    uint16_t freeSpace = UART_TX_BUFFER_SIZE - rb->count; /* 初始化 freeSpace。 */
    uint16_t writeLen = (len > freeSpace) ? freeSpace : len; /* 初始化 writeLen。 */

    for (uint16_t i = 0; i < writeLen; i++) /* 遍历当前数据集合。 */
    {
        rb->buffer[rb->head] = data[i]; /* 更新 buffer。 */
        rb->head = (rb->head + 1) % UART_TX_BUFFER_SIZE; /* 更新 head。 */
        rb->count++; /* 完成本行操作。 */
    }

    return writeLen; /* 返回当前计算结果。 */
}

/**
 * @brief 从发送缓冲区读取数据
 */
static uint16_t TxRingBuffer_Read(UartTxRingBuffer *rb, uint8_t *data, uint16_t len) /* 实现 TxRingBuffer_Read。 */
{
    if (data == NULL || len == 0) /* 检查当前执行条件。 */
        return 0; /* 返回状态值 0。 */

    uint16_t readLen = (len > rb->count) ? rb->count : len; /* 初始化 readLen。 */

    for (uint16_t i = 0; i < readLen; i++) /* 遍历当前数据集合。 */
    {
        data[i] = rb->buffer[rb->tail]; /* 更新 data。 */
        rb->tail = (rb->tail + 1) % UART_TX_BUFFER_SIZE; /* 更新 tail。 */
        rb->count--; /* 完成本行操作。 */
    }

    return readLen; /* 返回当前计算结果。 */
}

/**
 * @brief 启动发送
 */
static void StartTransmit(UartTxRingBuffer *rb) /* 实现 StartTransmit。 */
{
    if (rb->isSending || rb->count == 0) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    // 每次最多发送32字节
    uint16_t sendLen = (rb->count > UART_TX_CHUNK_SIZE) ? UART_TX_CHUNK_SIZE : rb->count; /* 初始化 sendLen。 */
    sendLen = TxRingBuffer_Read(rb, rb->temp_buffer, sendLen); /* 更新 sendLen。 */

    if (sendLen > 0) /* 检查当前执行条件。 */
    {
        rb->isSending = true; /* 更新 isSending。 */
        
        // 根据是否配置了DMA选择发送方式
        HAL_StatusTypeDef status; /* 保存 status。 */
        if (rb->huart->hdmatx != NULL) /* 检查当前执行条件。 */
        {
            // 使用 DMA 发送（USART3 配置了 DMA）
            status = HAL_UART_Transmit_DMA(rb->huart, rb->temp_buffer, sendLen); /* 更新 status。 */
        }
        else /* 处理其余情况。 */
        {
            // 使用中断发送（其他 UART 没有 DMA）
            status = HAL_UART_Transmit_IT(rb->huart, rb->temp_buffer, sendLen); /* 更新 status。 */
        }
        
        if (status != HAL_OK) /* 检查当前执行条件。 */
        {
            rb->isSending = false; /* 更新 isSending。 */
        }
    }
}

/**
 * @brief 启动IBUS DMA接收（UART6专用）
 */
static void StartIbusDma(UartRxRingBuffer *rb) /* 实现 StartIbusDma。 */
{
    if (rb == NULL || rb->huart == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(rb->huart, g_ibus_dma_buffer, IBUS_DMA_BUFFER_LEN); /* 初始化 status。 */
    if (status == HAL_OK || status == HAL_BUSY) /* 检查当前执行条件。 */
    {
        rb->isReceiving = true; /* 更新 isReceiving。 */
        if (rb->huart->hdmarx != NULL) /* 检查当前执行条件。 */
        {
            __HAL_DMA_DISABLE_IT(rb->huart->hdmarx, DMA_IT_HT); /* 调用 __HAL_DMA_DISABLE_IT。 */
        }
        return; /* 结束当前函数。 */
    }

    // 异常状态下尝试一次恢复，避免IBUS偶发错误后长期失联
    rb->isReceiving = false; /* 更新 isReceiving。 */
    (void)HAL_UART_AbortReceive(rb->huart); /* 调用 HAL_UART_AbortReceive。 */
    status = HAL_UARTEx_ReceiveToIdle_DMA(rb->huart, g_ibus_dma_buffer, IBUS_DMA_BUFFER_LEN); /* 更新 status。 */
    if (status == HAL_OK || status == HAL_BUSY) /* 检查当前执行条件。 */
    {
        rb->isReceiving = true; /* 更新 isReceiving。 */
    }

    if (rb->huart->hdmarx != NULL) /* 检查当前执行条件。 */
    {
        __HAL_DMA_DISABLE_IT(rb->huart->hdmarx, DMA_IT_HT); /* 调用 __HAL_DMA_DISABLE_IT。 */
    }
}

/**
 * @brief 向接收缓冲区写入一个字节（支持覆盖写入）
 * @note  去除读写指针间隔1的限制，允许完全覆盖
 *        缓冲区满时自动覆盖旧数据
 */
static bool RxRingBuffer_WriteByte(UartRxRingBuffer *rb, uint8_t byte) /* 实现 RxRingBuffer_WriteByte。 */
{
    rb->buffer[rb->head] = byte; /* 更新 buffer。 */
    rb->head = (rb->head + 1) % UART_RX_BUFFER_SIZE; /* 更新 head。 */

    if (rb->count < UART_RX_BUFFER_SIZE) /* 检查当前执行条件。 */
    {
        rb->count++; /* 完成本行操作。 */
    }
    else /* 处理其余情况。 */
    {
        rb->tail = (rb->tail + 1) % UART_RX_BUFFER_SIZE; /* 更新 tail。 */
        rb->overflowFlag = true; /* 更新 overflowFlag。 */
    }

    return true; /* 返回 true。 */
}

/**
 * @brief 从接收缓冲区读取数据
 */
static uint16_t RxRingBuffer_Read(UartRxRingBuffer *rb, uint8_t *data, uint16_t len) /* 实现 RxRingBuffer_Read。 */
{
    if (data == NULL || len == 0) /* 检查当前执行条件。 */
        return 0; /* 返回状态值 0。 */

    uint16_t readLen = (len > rb->count) ? rb->count : len; /* 初始化 readLen。 */

    for (uint16_t i = 0; i < readLen; i++) /* 遍历当前数据集合。 */
    {
        data[i] = rb->buffer[rb->tail]; /* 更新 data。 */
        rb->tail = (rb->tail + 1) % UART_RX_BUFFER_SIZE; /* 更新 tail。 */
        rb->count--; /* 完成本行操作。 */
    }

    return readLen; /* 返回当前计算结果。 */
}

/* ========== API接口实现 ========== */

/**
 * @brief 初始化BSP UART模块
 * @note 自动开启所有UART的中断接收，并为每个串口设置默认协议
 *       - UART3: SERVO_MCU协议（舵机通信，有MCU控制板，无CRC）
 *       - UART6: IBUS协议（遥控器接收）
 *       - 其他: OTHER（暂未定义）
 */
void BSP_UART_Init(void) /* 实现 BSP_UART_Init。 */
{
    for (uint8_t i = 0; i < BSP_UART_MAX; i++) /* 遍历当前数据集合。 */
    {
        UART_HandleTypeDef *huart = GetUartHandle((BSP_UART_NUM_e)i); /* 初始化 huart。 */
        if (huart != NULL) /* 检查当前执行条件。 */
        {
            UartTxRingBuffer *tx = &g_uart_tx_buffers[i]; /* 初始化 tx。 */
            UartRxRingBuffer *rb = &g_uart_rx_buffers[i]; /* 初始化 rb。 */

            memset(tx, 0, sizeof(*tx)); /* 调用 memset。 */
            tx->huart = huart; /* 更新 huart。 */
            memset(rb, 0, sizeof(*rb)); /* 调用 memset。 */
            rb->huart = huart; /* 更新 huart。 */
            rb->protocol_type = PROTOCOL_SERVO; /* 更新 protocol_type。 */
            Protocol_ResetParser(rb); /* 调用 Protocol_ResetParser。 */

            // 为每个串口设置默认协议
            switch ((BSP_UART_NUM_e)i) /* 按当前状态选择处理分支。 */
            {
            case BSP_UART3: /* 处理 BSP_UART3 分支。 */
                rb->protocol_type = PROTOCOL_SERVO_NO_MCU; // UART3: 舵机通信（无MCU驱动板，有CRC）
                break; /* 结束当前循环或分支。 */
            case BSP_UART6: /* 处理 BSP_UART6 分支。 */
                rb->protocol_type = PROTOCOL_IBUS; // UART6: IBUS接收
                break; /* 结束当前循环或分支。 */
            case BSP_UART8: /* 处理 BSP_UART8 分支。 */
                rb->protocol_type = PROTOCOL_REFEREE; // UART8: 裁判系统接收
                break; /* 结束当前循环或分支。 */
            default: /* 处理默认分支。 */
                rb->protocol_type = PROTOCOL_OTHER; // 其他串口暂未定义
                break; /* 结束当前循环或分支。 */
            }

            // 自动开启中断接收
            if (rb->protocol_type == PROTOCOL_IBUS) /* 检查当前执行条件。 */
            {
                StartIbusDma(rb); /* 调用 StartIbusDma。 */
            }
            else /* 处理其余情况。 */
            {
                rb->isReceiving = true; /* 更新 isReceiving。 */
                HAL_UART_Receive_IT(huart, &rb->rxByte, 1); /* 调用 HAL_UART_Receive_IT。 */
            }
        }
    }
}

/**
 * @brief 设置协议类型（兼容旧接口）
 * @param uart_num UART编号
 * @param is_servo_mode true=舵机协议, false=IBUS协议
 */
void UART_SetProtocol(BSP_UART_NUM_e uart_num, bool is_servo_mode) /* 实现 UART_SetProtocol。 */
{
    UART_SetProtocolType(uart_num, is_servo_mode ? PROTOCOL_SERVO : PROTOCOL_IBUS); /* 调用 UART_SetProtocolType。 */
}

/**
 * @brief 设置协议类型（推荐使用）
 * @param uart_num UART编号
 * @param protocol_type 协议类型
 */
void UART_SetProtocolType(BSP_UART_NUM_e uart_num, PROTOCOL_TYPE_e protocol_type) /* 实现 UART_SetProtocolType。 */
{
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num]; /* 初始化 rb。 */
    Protocol_SetType(uart_num, protocol_type); /* 调用 Protocol_SetType。 */

    // 重新配置接收方式
    if (rb->huart != NULL) /* 检查当前执行条件。 */
    {
        HAL_UART_AbortReceive(rb->huart); /* 调用 HAL_UART_AbortReceive。 */
    }
    else /* 处理其余情况。 */
    {
        return; /* 结束当前函数。 */
    }

    if (protocol_type == PROTOCOL_IBUS && uart_num == BSP_UART6) /* 检查当前执行条件。 */
    {
        StartIbusDma(rb); /* 调用 StartIbusDma。 */
    }
    else /* 处理其余情况。 */
    {
        rb->isReceiving = true; /* 更新 isReceiving。 */
        HAL_UART_Receive_IT(rb->huart, &rb->rxByte, 1); /* 调用 HAL_UART_Receive_IT。 */
    }
}

/**
 * @brief 发送数据
 */
uint16_t UART_Send(BSP_UART_NUM_e uart_num, const uint8_t *data, uint16_t len) /* 实现 UART_Send。 */
{
    if (uart_num >= BSP_UART_MAX || data == NULL || len == 0) /* 检查当前执行条件。 */
        return 0; /* 返回状态值 0。 */

    UartTxRingBuffer *rb = &g_uart_tx_buffers[uart_num]; /* 初始化 rb。 */

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    uint16_t written = TxRingBuffer_Write(rb, data, len); /* 初始化 written。 */
    if (!rb->isSending) /* 检查当前执行条件。 */
    {
        StartTransmit(rb); /* 调用 StartTransmit。 */
    }
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    return written; /* 返回当前计算结果。 */
}

/**
 * @brief 发送字符串
 */
uint16_t UART_SendString(BSP_UART_NUM_e uart_num, const char *str) /* 实现 UART_SendString。 */
{
    if (str == NULL) /* 检查当前执行条件。 */
        return 0; /* 返回状态值 0。 */

    return UART_Send(uart_num, (const uint8_t *)str, strlen(str)); /* 返回当前计算结果。 */
}

/**
 * @brief 重启接收（在接收中断被意外关闭时使用）
 * @note 正常情况下无需调用，BSP_UART_Init()已自动启动接收
 */
void UART_RestartRx(BSP_UART_NUM_e uart_num) /* 实现 UART_RestartRx。 */
{
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num]; /* 初始化 rb。 */
    if (rb->huart == NULL) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */
    if (rb->protocol_type == PROTOCOL_IBUS && uart_num == BSP_UART6) /* 检查当前执行条件。 */
    {
        StartIbusDma(rb); /* 调用 StartIbusDma。 */
    }
    else /* 处理其余情况。 */
    {
        rb->isReceiving = true; /* 更新 isReceiving。 */
        HAL_UART_Receive_IT(rb->huart, &rb->rxByte, 1); /* 调用 HAL_UART_Receive_IT。 */
    }
}

/**
 * @brief 读取接收数据（原始数据，读取后移动ReadIndex）
 */
uint16_t UART_Read(BSP_UART_NUM_e uart_num, uint8_t *data, uint16_t len) /* 实现 UART_Read。 */
{
    if (uart_num >= BSP_UART_MAX || data == NULL || len == 0) /* 检查当前执行条件。 */
        return 0; /* 返回状态值 0。 */

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    uint16_t result = RxRingBuffer_Read(&g_uart_rx_buffers[uart_num], data, len); /* 初始化 result。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    return result; /* 返回当前计算结果。 */
}

/**
 * @brief 获取接收缓冲区数据量
 */
uint16_t UART_GetRxCount(BSP_UART_NUM_e uart_num) /* 实现 UART_GetRxCount。 */
{
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return 0; /* 返回状态值 0。 */

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    uint16_t cnt = g_uart_rx_buffers[uart_num].count; /* 初始化 cnt。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    return cnt; /* 返回当前计算结果。 */
}

/**
 * @brief 检查是否有接收数据
 */
bool UART_HasData(BSP_UART_NUM_e uart_num) /* 实现 UART_HasData。 */
{
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return false; /* 返回 false。 */

    taskENTER_CRITICAL(); /* 调用 taskENTER_CRITICAL。 */
    bool has = (g_uart_rx_buffers[uart_num].count > 0); /* 初始化 has。 */
    taskEXIT_CRITICAL(); /* 调用 taskEXIT_CRITICAL。 */

    return has; /* 返回当前计算结果。 */
}

/**
 * @brief 清空接收缓冲区
 */
void UART_ClearRx(BSP_UART_NUM_e uart_num) /* 实现 UART_ClearRx。 */
{
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num]; /* 初始化 rb。 */
    rb->head = 0; /* 更新 head。 */
    rb->tail = 0; /* 更新 tail。 */
    rb->count = 0; /* 更新 count。 */
    rb->overflowFlag = false; /* 更新 overflowFlag。 */
    rb->ibus_error_count = 0; /* 更新 ibus_error_count。 */
    rb->servo_pkt_head = 0; /* 更新 servo_pkt_head。 */
    rb->servo_pkt_tail = 0; /* 更新 servo_pkt_tail。 */
    rb->servo_pkt_count = 0; /* 更新 servo_pkt_count。 */
    Protocol_ResetParser(rb); /* 调用 Protocol_ResetParser。 */
}

/* ========== 协议解析API实现（调用UartProtocol模块） ========== */

/**
 * @brief 检查是否有完整的数据包（调用UartProtocol模块）
 */
bool UART_HasPacket(BSP_UART_NUM_e uart_num) /* 实现 UART_HasPacket。 */
{
    return Protocol_HasPacket(uart_num); /* 返回当前计算结果。 */
}

/**
 * @brief 获取舵机协议数据包（调用UartProtocol模块）
 */
bool UART_GetServoPacket(BSP_UART_NUM_e uart_num, ServoPacket_t *packet) /* 实现 UART_GetServoPacket。 */
{
    return Protocol_GetServoPacket(uart_num, packet); /* 返回当前计算结果。 */
}

/**
 * @brief 清除数据包标志（调用UartProtocol模块）
 */
void UART_ClearPacket(BSP_UART_NUM_e uart_num) /* 实现 UART_ClearPacket。 */
{
    Protocol_ClearPacket(uart_num); /* 调用 Protocol_ClearPacket。 */
}

/**
 * @brief 检查是否有完整的IBUS数据包
 */
bool UART_HasIbusPacket(BSP_UART_NUM_e uart_num) /* 实现 UART_HasIbusPacket。 */
{
    return Protocol_HasIbusPacket(uart_num); /* 返回当前计算结果。 */
}

/**
 * @brief 获取IBUS数据包
 */
bool UART_GetIbusPacket(BSP_UART_NUM_e uart_num, IbusPacket_t *packet) /* 实现 UART_GetIbusPacket。 */
{
    return Protocol_GetIbusPacket(uart_num, packet); /* 返回当前计算结果。 */
}

/**
 * @brief 清除IBUS数据包标志
 */
void UART_ClearIbusPacket(BSP_UART_NUM_e uart_num) /* 实现 UART_ClearIbusPacket。 */
{
    Protocol_ClearIbusPacket(uart_num); /* 调用 Protocol_ClearIbusPacket。 */
}

/**
 * @brief 检查是否有完整的裁判系统数据包
 */
bool UART_HasRefereePacket(BSP_UART_NUM_e uart_num) /* 实现 UART_HasRefereePacket。 */
{
    return Protocol_HasRefereePacket(uart_num); /* 返回当前计算结果。 */
}

/**
 * @brief 获取裁判系统数据包
 */
bool UART_GetRefereePacket(BSP_UART_NUM_e uart_num, RefereePacket_t *packet) /* 实现 UART_GetRefereePacket。 */
{
    return Protocol_GetRefereePacket(uart_num, packet); /* 返回当前计算结果。 */
}

/**
 * @brief 清除裁判系统数据包标志
 */
void UART_ClearRefereePacket(BSP_UART_NUM_e uart_num) /* 实现 UART_ClearRefereePacket。 */
{
    Protocol_ClearRefereePacket(uart_num); /* 调用 Protocol_ClearRefereePacket。 */
}

/* ========== HAL回调函数实现 ========== */

/**
 * @brief UART发送完成回调
 */
void BSP_UART_TxCpltCallback(UART_HandleTypeDef *huart) /* 实现 BSP_UART_TxCpltCallback。 */
{
    BSP_UART_NUM_e uart_num = GetUartNum(huart); /* 初始化 uart_num。 */
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    UartTxRingBuffer *rb = &g_uart_tx_buffers[uart_num]; /* 初始化 rb。 */
    rb->isSending = false; /* 更新 isSending。 */

    if (rb->count > 0) /* 检查当前执行条件。 */
    {
        StartTransmit(rb); /* 调用 StartTransmit。 */
    }
}

/**
 * @brief UART接收完成回调
 * @note  UART6 使用DMA接收IBUS固定32字节帧；
 *        其他串口使用中断按字节接收并解析舵机协议
 */
void BSP_UART_RxCpltCallback(UART_HandleTypeDef *huart) /* 实现 BSP_UART_RxCpltCallback。 */
{
    BSP_UART_NUM_e uart_num = GetUartNum(huart); /* 初始化 uart_num。 */
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num]; /* 初始化 rb。 */
    if (uart_num == BSP_UART6 && rb->protocol_type == PROTOCOL_IBUS) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    if (uart_num == BSP_UART8) /* 检查当前执行条件。 */
    {
        Uart8RxDebug_ByteCount++; /* 递增 Uart8RxDebug_ByteCount。 */
        Uart8RxDebug_LastByte = rb->rxByte; /* 定义 Uart8RxDebug_LastByte 枚举项。 */
        Uart8RxDebug_LastTick = HAL_GetTick(); /* 定义 Uart8RxDebug_LastTick 枚举项。 */
    }

    // 将字节写入环形缓冲区（用于原始数据读取）
    RxRingBuffer_WriteByte(rb, rb->rxByte); /* 调用 RxRingBuffer_WriteByte。 */

    // 舵机协议解析
    Protocol_ParseByte(rb, rb->rxByte); /* 调用 Protocol_ParseByte。 */

    // 继续接收下一个字节
    if (rb->isReceiving) /* 检查当前执行条件。 */
    {
        HAL_StatusTypeDef status = HAL_UART_Receive_IT(rb->huart, &rb->rxByte, 1); /* 初始化 status。 */
        if (uart_num == BSP_UART8) /* 检查当前执行条件。 */
        {
            Uart8RxDebug_LastReceiveStatus = (uint32_t)status; /* 定义 Uart8RxDebug_LastReceiveStatus 枚举项。 */
            if (status != HAL_OK) /* 检查当前执行条件。 */
            {
                Uart8RxDebug_ReceiveItFailCount++; /* 递增 Uart8RxDebug_ReceiveItFailCount。 */
            }
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) /* 实现 HAL_UART_TxCpltCallback。 */
{
    BSP_UART_TxCpltCallback(huart); /* 调用 BSP_UART_TxCpltCallback。 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) /* 实现 HAL_UART_RxCpltCallback。 */
{
    BSP_UART_RxCpltCallback(huart); /* 调用 BSP_UART_RxCpltCallback。 */
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) /* 实现 HAL_UART_ErrorCallback。 */
{
    BSP_UART_NUM_e uart_num = GetUartNum(huart); /* 初始化 uart_num。 */
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num]; /* 初始化 rb。 */
    if (!rb->isReceiving) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    if (uart_num == BSP_UART8) /* 检查当前执行条件。 */
    {
        Uart8RxDebug_ErrorCount++; /* 递增 Uart8RxDebug_ErrorCount。 */
        Uart8RxDebug_LastErrorCode = huart->ErrorCode; /* 定义 Uart8RxDebug_LastErrorCode 枚举项。 */
    }

    if (uart_num == BSP_UART6 && rb->protocol_type == PROTOCOL_IBUS) /* 检查当前执行条件。 */
    {
        StartIbusDma(rb); /* 调用 StartIbusDma。 */
    }
    else /* 处理其余情况。 */
    {
        HAL_StatusTypeDef status = HAL_UART_Receive_IT(rb->huart, &rb->rxByte, 1); /* 初始化 status。 */
        if (uart_num == BSP_UART8) /* 检查当前执行条件。 */
        {
            Uart8RxDebug_RestartCount++; /* 递增 Uart8RxDebug_RestartCount。 */
            Uart8RxDebug_LastReceiveStatus = (uint32_t)status; /* 定义 Uart8RxDebug_LastReceiveStatus 枚举项。 */
            if (status != HAL_OK) /* 检查当前执行条件。 */
            {
                Uart8RxDebug_ReceiveItFailCount++; /* 递增 Uart8RxDebug_ReceiveItFailCount。 */
            }
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) /* 实现 HAL_UARTEx_RxEventCallback。 */
{
    if (huart != &huart6) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    UartRxRingBuffer *rb = &g_uart_rx_buffers[BSP_UART6]; /* 初始化 rb。 */
    if (rb->protocol_type != PROTOCOL_IBUS) /* 检查当前执行条件。 */
        return; /* 结束当前函数。 */

    if (Size > 0 && Size <= IBUS_DMA_BUFFER_LEN) /* 检查当前执行条件。 */
    {
        Protocol_ParseIbusStream(rb, g_ibus_dma_buffer, Size); /* 调用 Protocol_ParseIbusStream。 */
    }

    if (rb->isReceiving) /* 检查当前执行条件。 */
    {
        StartIbusDma(rb); /* 调用 StartIbusDma。 */
    }
}

/* ========== 缓冲区访问函数（供UartProtocol模块使用） ========== */

/**
 * @brief 获取接收缓冲区指针
 * @param uart_num UART编号
 * @return 接收缓冲区指针，失败返回NULL
 */
UartRxRingBuffer *BSP_UART_GetRxBuffer(BSP_UART_NUM_e uart_num) /* 实现 BSP_UART_GetRxBuffer。 */
{
    if (uart_num >= BSP_UART_MAX) /* 检查当前执行条件。 */
        return NULL; /* 返回当前计算结果。 */
    return &g_uart_rx_buffers[uart_num]; /* 返回当前计算结果。 */
}
