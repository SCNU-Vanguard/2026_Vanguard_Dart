#include "bsp_uart.h"
#include "UartProtocol.h"
#include <string.h>
#include <stdbool.h>

// todo：解算舵机格式需要加上一个长度传输函数，这样才可以保证这个长度是正确的，方便解算
// todo: 后续应该使用其他手段比如上电或者信号量进行初始化与上位机的通信，从而保证稳定

/* 内部缓冲区定义（用户无需关心） */
static UartTxRingBuffer g_uart_tx_buffers[BSP_UART_MAX];
static UartRxRingBuffer g_uart_rx_buffers[BSP_UART_MAX];
static uint8_t g_ibus_dma_buffer[IBUS_DMA_BUFFER_LEN];

/* 内部函数声明 */
static void TxRingBuffer_Init(UartTxRingBuffer *rb, UART_HandleTypeDef *huart);
static void RxRingBuffer_Init(UartRxRingBuffer *rb, UART_HandleTypeDef *huart);
static void StartTransmit(UartTxRingBuffer *rb);
static uint16_t TxRingBuffer_Write(UartTxRingBuffer *rb, const uint8_t *data, uint16_t len);
static uint16_t TxRingBuffer_Read(UartTxRingBuffer *rb, uint8_t *data, uint16_t len);
static bool RxRingBuffer_WriteByte(UartRxRingBuffer *rb, uint8_t byte);
static uint16_t RxRingBuffer_Read(UartRxRingBuffer *rb, uint8_t *data, uint16_t len);
static UART_HandleTypeDef *GetUartHandle(BSP_UART_NUM_e uart_num);
static BSP_UART_NUM_e GetUartNum(UART_HandleTypeDef *huart);
static void StartIbusDma(UartRxRingBuffer *rb);

/**
 * @brief CRC校验生成（公开函数，供所有模块使用）
 * @param data 数据指针
 * @param length 数据长度
 * @return CRC校验码
 * @note 协议解析状态机已移至 UartProtocol.c
 */
uint8_t UART_Calculate_CRC(uint8_t *data, uint8_t length)
{
    return Protocol_Calculate_CRC(data, length);
}

/**
 * @brief 获取UART句柄
 */
static UART_HandleTypeDef *GetUartHandle(BSP_UART_NUM_e uart_num)
{
    switch (uart_num)
    {
    case BSP_UART3:
        return &huart3;
    case BSP_UART6:
        return &huart6;
    default:
        return NULL;
    }
}

/**
 * @brief 根据UART句柄获取UART编号
 */
static BSP_UART_NUM_e GetUartNum(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
        return BSP_UART3;
    if (huart == &huart6)
        return BSP_UART6;
    return BSP_UART_MAX;
}

/**
 * @brief 发送环形缓冲区初始化
 */
static void TxRingBuffer_Init(UartTxRingBuffer *rb, UART_HandleTypeDef *huart)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    rb->isSending = false;
    rb->huart = huart;
    memset(rb->buffer, 0, UART_TX_BUFFER_SIZE);
    memset(rb->temp_buffer, 0, UART_TX_CHUNK_SIZE);
}

/**
 * @brief 接收环形缓冲区初始化
 */
static void RxRingBuffer_Init(UartRxRingBuffer *rb, UART_HandleTypeDef *huart)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    rb->isReceiving = false;
    rb->overflowFlag = false;
    rb->rxByte = 0;
    rb->huart = huart;
    rb->protocol_type = PROTOCOL_SERVO; // 默认舵机协议
    rb->packet_ready = false;
    rb->ibus_ready = false;
    rb->ibus_error_count = 0;
    Protocol_ResetParser(rb);
    memset(rb->buffer, 0, UART_RX_BUFFER_SIZE);
}

/**
 * @brief 向发送缓冲区写入数据
 */
static uint16_t TxRingBuffer_Write(UartTxRingBuffer *rb, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
        return 0;

    uint16_t freeSpace = UART_TX_BUFFER_SIZE - rb->count;
    uint16_t writeLen = (len > freeSpace) ? freeSpace : len;

    for (uint16_t i = 0; i < writeLen; i++)
    {
        rb->buffer[rb->head] = data[i];
        rb->head = (rb->head + 1) % UART_TX_BUFFER_SIZE;
        rb->count++;
    }

    return writeLen;
}

/**
 * @brief 从发送缓冲区读取数据
 */
static uint16_t TxRingBuffer_Read(UartTxRingBuffer *rb, uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
        return 0;

    uint16_t readLen = (len > rb->count) ? rb->count : len;

    for (uint16_t i = 0; i < readLen; i++)
    {
        data[i] = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % UART_TX_BUFFER_SIZE;
        rb->count--;
    }

    return readLen;
}

/**
 * @brief 启动发送
 */
static void StartTransmit(UartTxRingBuffer *rb)
{
    if (rb->isSending || rb->count == 0)
        return;

    // 每次最多发送32字节
    uint16_t sendLen = (rb->count > UART_TX_CHUNK_SIZE) ? UART_TX_CHUNK_SIZE : rb->count;
    sendLen = TxRingBuffer_Read(rb, rb->temp_buffer, sendLen);

    if (sendLen > 0)
    {
        rb->isSending = true;
        if (HAL_UART_Transmit_IT(rb->huart, rb->temp_buffer, sendLen) != HAL_OK)
        {
            rb->isSending = false;
        }
    }
}

/**
 * @brief 启动IBUS DMA接收（UART6专用）
 */
static void StartIbusDma(UartRxRingBuffer *rb)
{
    if (rb == NULL || rb->huart == NULL)
        return;

    rb->isReceiving = true;
    if (HAL_UARTEx_ReceiveToIdle_DMA(rb->huart, g_ibus_dma_buffer, IBUS_FRAME_LEN) != HAL_OK)
    {
        rb->isReceiving = false;
        return;
    }
    if (rb->huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(rb->huart->hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief 向接收缓冲区写入一个字节（支持覆盖写入）
 * @note  去除读写指针间隔1的限制，允许完全覆盖
 *        缓冲区满时自动覆盖旧数据
 */
static bool RxRingBuffer_WriteByte(UartRxRingBuffer *rb, uint8_t byte)
{
    rb->buffer[rb->head] = byte;
    rb->head = (rb->head + 1) % UART_RX_BUFFER_SIZE;

    if (rb->count < UART_RX_BUFFER_SIZE)
    {
        rb->count++;
    }
    else
    {
        rb->tail = (rb->tail + 1) % UART_RX_BUFFER_SIZE;
        rb->overflowFlag = true;
    }

    return true;
}

/**
 * @brief 从接收缓冲区读取数据
 */
static uint16_t RxRingBuffer_Read(UartRxRingBuffer *rb, uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0)
        return 0;

    uint16_t readLen = (len > rb->count) ? rb->count : len;

    for (uint16_t i = 0; i < readLen; i++)
    {
        data[i] = rb->buffer[rb->tail];
        rb->tail = (rb->tail + 1) % UART_RX_BUFFER_SIZE;
        rb->count--;
    }

    return readLen;
}

/* ========== API接口实现 ========== */

/**
 * @brief 初始化BSP UART模块
 * @note 自动开启所有UART的中断接收，并为每个串口设置默认协议
 *       - UART3: SERVO_MCU协议（舵机通信，有MCU控制板，无CRC）
 *       - UART6: IBUS协议（遥控器接收）
 *       - 其他: OTHER（暂未定义）
 */
void BSP_UART_Init(void)
{
    for (uint8_t i = 0; i < BSP_UART_MAX; i++)
    {
        UART_HandleTypeDef *huart = GetUartHandle((BSP_UART_NUM_e)i);
        if (huart != NULL)
        {
            TxRingBuffer_Init(&g_uart_tx_buffers[i], huart);
            RxRingBuffer_Init(&g_uart_rx_buffers[i], huart);

            // 为每个串口设置默认协议
            UartRxRingBuffer *rb = &g_uart_rx_buffers[i];
            switch ((BSP_UART_NUM_e)i)
            {
            case BSP_UART3:
                rb->protocol_type = PROTOCOL_SERVO_MCU; // UART3: 舵机通信（有MCU控制板，无CRC）
                break;
            case BSP_UART6:
                rb->protocol_type = PROTOCOL_IBUS; // UART6: IBUS接收
                break;
            default:
                rb->protocol_type = PROTOCOL_OTHER; // 其他串口暂未定义
                break;
            }

            // 自动开启中断接收
            if (rb->protocol_type == PROTOCOL_IBUS)
            {
                StartIbusDma(rb);
            }
            else
            {
                rb->isReceiving = true;
                HAL_UART_Receive_IT(huart, &rb->rxByte, 1);
            }
        }
    }
}

/**
 * @brief 设置协议类型（兼容旧接口）
 * @param uart_num UART编号
 * @param is_servo_mode true=舵机协议, false=IBUS协议
 */
void UART_SetProtocol(BSP_UART_NUM_e uart_num, bool is_servo_mode)
{
    UART_SetProtocolType(uart_num, is_servo_mode ? PROTOCOL_SERVO : PROTOCOL_IBUS);
}

/**
 * @brief 设置协议类型（推荐使用）
 * @param uart_num UART编号
 * @param protocol_type 协议类型
 */
void UART_SetProtocolType(BSP_UART_NUM_e uart_num, PROTOCOL_TYPE_e protocol_type)
{
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num];
    Protocol_SetType(uart_num, protocol_type);

    // 重新配置接收方式
    if (rb->huart != NULL)
    {
        HAL_UART_AbortReceive(rb->huart);
    }
    else
    {
        return;
    }

    if (protocol_type == PROTOCOL_IBUS && uart_num == BSP_UART6)
    {
        StartIbusDma(rb);
    }
    else
    {
        rb->isReceiving = true;
        HAL_UART_Receive_IT(rb->huart, &rb->rxByte, 1);
    }
}

/**
 * @brief 发送数据
 */
uint16_t UART_Send(BSP_UART_NUM_e uart_num, const uint8_t *data, uint16_t len)
{
    if (uart_num >= BSP_UART_MAX || data == NULL || len == 0)
        return 0;

    UartTxRingBuffer *rb = &g_uart_tx_buffers[uart_num];
    uint16_t written = TxRingBuffer_Write(rb, data, len);

    if (!rb->isSending)
    {
        StartTransmit(rb);
    }

    return written;
}

/**
 * @brief 发送字符串
 */
uint16_t UART_SendString(BSP_UART_NUM_e uart_num, const char *str)
{
    if (str == NULL)
        return 0;

    return UART_Send(uart_num, (const uint8_t *)str, strlen(str));
}

/**
 * @brief 重启接收（在接收中断被意外关闭时使用）
 * @note 正常情况下无需调用，BSP_UART_Init()已自动启动接收
 */
void UART_RestartRx(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num];
    if (rb->huart == NULL)
        return;
    if (rb->protocol_type == PROTOCOL_IBUS && uart_num == BSP_UART6)
    {
        StartIbusDma(rb);
    }
    else
    {
        rb->isReceiving = true;
        HAL_UART_Receive_IT(rb->huart, &rb->rxByte, 1);
    }
}

/**
 * @brief 读取接收数据（原始数据，读取后移动ReadIndex）
 */
uint16_t UART_Read(BSP_UART_NUM_e uart_num, uint8_t *data, uint16_t len)
{
    if (uart_num >= BSP_UART_MAX || data == NULL || len == 0)
        return 0;

    return RxRingBuffer_Read(&g_uart_rx_buffers[uart_num], data, len);
}

/**
 * @brief 获取接收缓冲区数据量
 */
uint16_t UART_GetRxCount(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return 0;

    return g_uart_rx_buffers[uart_num].count;
}

/**
 * @brief 检查是否有接收数据
 */
bool UART_HasData(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return false;

    return (g_uart_rx_buffers[uart_num].count > 0);
}

/**
 * @brief 清空接收缓冲区
 */
void UART_ClearRx(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num];
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
    rb->overflowFlag = false;
    rb->ibus_error_count = 0;
    Protocol_ResetParser(rb);
}

/* ========== 协议解析API实现（调用UartProtocol模块） ========== */

/**
 * @brief 检查是否有完整的数据包（调用UartProtocol模块）
 */
bool UART_HasPacket(BSP_UART_NUM_e uart_num)
{
    return Protocol_HasPacket(uart_num);
}

/**
 * @brief 获取舵机协议数据包（调用UartProtocol模块）
 */
bool UART_GetServoPacket(BSP_UART_NUM_e uart_num, ServoPacket_t *packet)
{
    return Protocol_GetServoPacket(uart_num, packet);
}

/**
 * @brief 清除数据包标志（调用UartProtocol模块）
 */
void UART_ClearPacket(BSP_UART_NUM_e uart_num)
{
    Protocol_ClearPacket(uart_num);
}

/**
 * @brief 检查是否有完整的IBUS数据包
 */
bool UART_HasIbusPacket(BSP_UART_NUM_e uart_num)
{
    return Protocol_HasIbusPacket(uart_num);
}

/**
 * @brief 获取IBUS数据包
 */
bool UART_GetIbusPacket(BSP_UART_NUM_e uart_num, IbusPacket_t *packet)
{
    return Protocol_GetIbusPacket(uart_num, packet);
}

/**
 * @brief 清除IBUS数据包标志
 */
void UART_ClearIbusPacket(BSP_UART_NUM_e uart_num)
{
    Protocol_ClearIbusPacket(uart_num);
}

/* ========== HAL回调函数实现 ========== */

/**
 * @brief UART发送完成回调
 */
void BSP_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_NUM_e uart_num = GetUartNum(huart);
    if (uart_num >= BSP_UART_MAX)
        return;

    UartTxRingBuffer *rb = &g_uart_tx_buffers[uart_num];
    rb->isSending = false;

    if (rb->count > 0)
    {
        StartTransmit(rb);
    }
}

/**
 * @brief UART接收完成回调
 * @note  UART6 使用DMA接收IBUS固定32字节帧；
 *        其他串口使用中断按字节接收并解析舵机协议
 */
void BSP_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_NUM_e uart_num = GetUartNum(huart);
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num];
    if (uart_num == BSP_UART6 && rb->protocol_type == PROTOCOL_IBUS)
        return;

    // 将字节写入环形缓冲区（用于原始数据读取）
    RxRingBuffer_WriteByte(rb, rb->rxByte);

    // 舵机协议解析
    Protocol_ParseByte(rb, rb->rxByte);

    // 继续接收下一个字节
    if (rb->isReceiving)
    {
        HAL_UART_Receive_IT(rb->huart, &rb->rxByte, 1);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_TxCpltCallback(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_RxCpltCallback(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != &huart6)
        return;

    UartRxRingBuffer *rb = &g_uart_rx_buffers[BSP_UART6];
    if (rb->protocol_type != PROTOCOL_IBUS)
        return;

    if (Size > 0 && Size <= IBUS_DMA_BUFFER_LEN)
    {
        Protocol_ParseIbusStream(rb, g_ibus_dma_buffer, Size);
    }

    if (rb->isReceiving)
    {
        StartIbusDma(rb);
    }
}

/* ========== 缓冲区访问函数（供UartProtocol模块使用） ========== */

/**
 * @brief 获取接收缓冲区指针
 * @param uart_num UART编号
 * @return 接收缓冲区指针，失败返回NULL
 */
UartRxRingBuffer *BSP_UART_GetRxBuffer(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return NULL;
    return &g_uart_rx_buffers[uart_num];
}
