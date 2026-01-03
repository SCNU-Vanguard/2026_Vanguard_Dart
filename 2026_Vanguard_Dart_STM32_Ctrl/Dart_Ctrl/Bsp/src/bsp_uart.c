#include "bsp_uart.h"
#include "UartProtocol.h"
#include <string.h>
#include <stdbool.h>

// todo：解算舵机格式需要加上一个长度传输函数，这样才可以保证这个长度是正确的，方便解算
// todo: 后续应该使用其他手段比如上电或者信号量进行初始化与上位机的通信，从而保证稳定

/* 内部缓冲区定义（用户无需关心） */
static UartTxRingBuffer g_uart_tx_buffers[BSP_UART_MAX];
static UartRxRingBuffer g_uart_rx_buffers[BSP_UART_MAX];
static DataBuffer g_data_buffers[BSP_UART_MAX]; // DataBuffer实例（用于指针操作）

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
 * @brief 向接收缓冲区写入一个字节（支持覆盖写入）
 * @note  去除读写指针间隔1的限制，允许完全覆盖
 *        缓冲区满时自动覆盖旧数据
 */
static bool RxRingBuffer_WriteByte(UartRxRingBuffer *rb, uint8_t byte)
{
    // 直接写入，无论缓冲区是否满
    rb->buffer[rb->head] = byte;
    rb->head = (rb->head + 1) % UART_RX_BUFFER_SIZE;

    // 如果head追上tail，说明覆盖了旧数据，需要移动tail
    if (rb->head == rb->tail)
    {
        rb->tail = (rb->tail + 1) % UART_RX_BUFFER_SIZE;
        rb->overflowFlag = true;
        // count保持为最大值
    }
    else
    {
        rb->count++;
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
 *       - UART6: DART协议（上位机通信）
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

            // 初始化DataBuffer（包含帧索引缓冲区，调用UartProtocol模块）
            Protocol_DataBufferInit(&g_data_buffers[i]);

            // 为每个串口设置默认协议
            UartRxRingBuffer *rb = &g_uart_rx_buffers[i];
            switch ((BSP_UART_NUM_e)i)
            {
            case BSP_UART3:
                rb->protocol_type = PROTOCOL_SERVO_MCU; // UART3: 舵机通信（有MCU控制板，无CRC）
                break;
            case BSP_UART6:
                rb->protocol_type = PROTOCOL_DART; // UART6: 上位机通信，使用DART协议
                break;
            default:
                rb->protocol_type = PROTOCOL_OTHER; // 其他串口暂未定义
                break;
            }

            // 自动开启中断接收
            rb->isReceiving = true;
            HAL_UART_Receive_IT(huart, &rb->rxByte, 1);
        }
    }
}

/**
 * @brief 设置协议类型（调用UartProtocol模块）
 * @param uart_num UART编号
 * @param is_servo_mode true=舵机协议, false=DART协议
 * @note 协议设置逻辑已移至 UartProtocol.c
 */
void UART_SetProtocol(BSP_UART_NUM_e uart_num, bool is_servo_mode)
{
    Protocol_SetType(uart_num, is_servo_mode ? PROTOCOL_SERVO : PROTOCOL_DART);
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
    rb->isReceiving = true;
    HAL_UART_Receive_IT(rb->huart, &rb->rxByte, 1);
}

/**
 * @brief 读取接收数据（原始数据，读取后移动ReadIndex）
 */
uint16_t UART_Read(BSP_UART_NUM_e uart_num, uint8_t *data, uint16_t len)
{
    if (uart_num >= BSP_UART_MAX || data == NULL || len == 0)
        return 0;

    DataBuffer *db = &g_data_buffers[uart_num];

    // 计算环形缓冲区中可读数据量
    uint16_t dataCount = (db->WriteIndex - db->ReadIndex + UART_RX_BUFFER_SIZE) % UART_RX_BUFFER_SIZE;
    uint16_t readLen = (len > dataCount) ? dataCount : len;

    // 从环形缓冲区读取数据并移动ReadIndex
    for (uint16_t i = 0; i < readLen; i++)
    {
        data[i] = db->RxDataBuffer[db->ReadIndex];
        db->ReadIndex = (db->ReadIndex + 1) % UART_RX_BUFFER_SIZE;
    }

    // 同时从UartRxRingBuffer读取（保持同步）
    RxRingBuffer_Read(&g_uart_rx_buffers[uart_num], NULL, 0);

    return readLen;
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
 * @brief 获取DART协议数据包（调用UartProtocol模块）
 */
bool UART_GetDartPacket(BSP_UART_NUM_e uart_num, DartPacket_t *packet)
{
    return Protocol_GetDartPacket(uart_num, packet);
}

/**
 * @brief 清除数据包标志（调用UartProtocol模块）
 */
void UART_ClearPacket(BSP_UART_NUM_e uart_num)
{
    Protocol_ClearPacket(uart_num);
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
 * @brief UART接收完成回调（包含协议解析、帧头识别和DataBuffer环形写入）
 * @note  环形缓冲区特性：
 *        - 写指针可循环回绕到数组开头，覆盖旧数据
 *        - 缓冲区满时自动覆盖旧数据
 *        - 去除读写指针间隔1的限制，允许完全覆盖
 *        - 覆盖时读指针强制移动到下一个有效帧头位置
 *        - 自动识别帧头位置并记录到帧索引数组
 *        - 溢出时自动使被覆盖的帧无效化
 */
void BSP_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_UART_NUM_e uart_num = GetUartNum(huart);
    if (uart_num >= BSP_UART_MAX)
        return;

    UartRxRingBuffer *rb = &g_uart_rx_buffers[uart_num];
    DataBuffer *db = &g_data_buffers[uart_num];

    // 将字节写入环形缓冲区（用于原始数据读取）
    RxRingBuffer_WriteByte(rb, rb->rxByte);

    // 记录当前写入位置（用于帧头识别）
    uint8_t currentWritePos = (uint8_t)db->WriteIndex;

    // 检测环绕：写指针即将从末尾回到开头
    if (db->WriteIndex == UART_RX_BUFFER_SIZE - 1)
    {
        db->WrapAroundFlag = true; // 标记即将发生环绕
    }

    // 环形缓冲区写入DataBuffer
    // 计算下一个写位置
    uint16_t nextWriteIndex = (db->WriteIndex + 1) % UART_RX_BUFFER_SIZE;

    // 检查写指针是否追上读指针（缓冲区满，发生覆盖）
    // 去除间隔1的限制，允许完全覆盖
    if (db->WriteIndex == db->ReadIndex && db->OverflowFlag)
    {
        // 已经处于覆盖状态，每次写入都需要处理
        // 1. 使被覆盖位置的帧无效化（调用UartProtocol模块）
        Protocol_InvalidateOverwrittenFrames(db, rb, (uint8_t)db->WriteIndex);

        // 2. 移动读指针到下一个有效帧头位置（调用UartProtocol模块）
        Protocol_MoveReadIndexToNextFrame(db, rb);
    }
    else if (nextWriteIndex == db->ReadIndex)
    {
        // 首次追上读指针，进入覆盖模式
        // 1. 使被覆盖位置的帧无效化（调用UartProtocol模块）
        Protocol_InvalidateOverwrittenFrames(db, rb, (uint8_t)db->ReadIndex);

        // 2. 移动读指针到下一个有效帧头位置（调用UartProtocol模块）
        Protocol_MoveReadIndexToNextFrame(db, rb);

        db->OverflowFlag = true; // 标记发生了数据覆盖
    }

    // 写入数据
    db->RxDataBuffer[db->WriteIndex] = rb->rxByte;
    db->WriteIndex = nextWriteIndex;

    // 帧头识别（在写入数据后进行，调用UartProtocol模块）
    Protocol_DetectFrameHeader(db, rb, rb->rxByte, currentWritePos);

    // 协议解析（在帧头识别后进行，调用UartProtocol模块）
    Protocol_ParseByte(rb, rb->rxByte);

    // 如果协议解析完成且CRC校验通过，记录帧尾位置并设置有效标志
    if (rb->packet_ready)
    {
        // 帧尾位置是当前写入位置（刚写入的CRC字节位置，调用UartProtocol模块）
        Protocol_RecordFrameTail(db, rb, currentWritePos);
    }

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

/**
 * @brief 获取数据缓冲区指针
 * @param uart_num UART编号
 * @return 数据缓冲区指针，失败返回NULL
 */
DataBuffer *BSP_UART_GetDataBuffer(BSP_UART_NUM_e uart_num)
{
    if (uart_num >= BSP_UART_MAX)
        return NULL;
    return &g_data_buffers[uart_num];
}
