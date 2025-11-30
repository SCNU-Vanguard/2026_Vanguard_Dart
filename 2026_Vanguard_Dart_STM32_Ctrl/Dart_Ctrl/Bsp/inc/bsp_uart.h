#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "usart.h"

/* 循环缓冲区大小定义 */
#define UART_TX_BUFFER_SIZE 64
#define UART_RX_BUFFER_SIZE 64

/* 帧头帧尾位置记录数组大小定义 */
// DART协议（4字节帧头）：64字节缓冲区最多装16个协议帧
#define DART_FRAME_INDEX_SIZE 16
// 0x55 0x55协议（2字节帧头）：64字节缓冲区最多装32个协议帧
#define SERVO_FRAME_INDEX_SIZE 32

/* UART编号枚举 */
typedef enum
{
    BSP_UART3 = 0,
    BSP_UART6,
    BSP_UART7,
    BSP_UART8,
    BSP_UART_MAX
} BSP_UART_NUM_e;

/* 帧位置记录结构体 - 环形缓冲区记录帧头帧尾在数据缓冲区中的位置 */
typedef struct
{
    // DART协议帧位置记录（4字节帧头，最多16帧）
    uint8_t DartHeaderIndex[DART_FRAME_INDEX_SIZE]; // DART帧头在RxDataBuffer中的位置
    uint8_t DartTailIndex[DART_FRAME_INDEX_SIZE];   // DART帧尾在RxDataBuffer中的位置
    uint8_t DartFrameCount;                         // 有效帧数量
    uint8_t DartWritePtr;                           // DART帧位置写指针（中断中使用）
    uint8_t DartReadPtr;                            // DART帧位置读指针（用户读取时使用）
    bool DartFrameValid[DART_FRAME_INDEX_SIZE];     // 帧有效标志（用于标记被覆盖的帧）

    // 0x55 0x55协议帧位置记录（2字节帧头，最多32帧）
    uint8_t ServoHeaderIndex[SERVO_FRAME_INDEX_SIZE]; // Servo帧头在RxDataBuffer中的位置
    uint8_t ServoTailIndex[SERVO_FRAME_INDEX_SIZE];   // Servo帧尾在RxDataBuffer中的位置
    uint8_t ServoFrameCount;                          // 有效帧数量
    uint8_t ServoWritePtr;                            // Servo帧位置写指针（中断中使用）
    uint8_t ServoReadPtr;                             // Servo帧位置读指针（用户读取时使用）
    bool ServoFrameValid[SERVO_FRAME_INDEX_SIZE];     // 帧有效标志
} FrameIndexBuffer;

/* 接收缓冲区结构体 - 真正的环形缓冲区 */
typedef struct
{
    uint8_t RxDataBuffer[UART_RX_BUFFER_SIZE];
    uint16_t ReadIndex;  // 读索引（用户读取数据时移动），指向待读取的位置
    uint16_t WriteIndex; // 写索引（接收中断时移动），指向待写入的位置
    // 环形缓冲区特性：
    // - 空状态: WriteIndex == ReadIndex
    // - 满状态: (WriteIndex + 1) % SIZE == ReadIndex
    // - 可用数据长度: (WriteIndex - ReadIndex + SIZE) % SIZE
    // - 写指针可循环回绕到数组开头，覆盖旧数据
    bool OverflowFlag;   // 溢出标志位（表示发生了数据覆盖）
    bool WrapAroundFlag; // 环绕标志位（表示写指针已经从末尾回绕到开头）

    // 帧头帧尾位置记录
    FrameIndexBuffer FrameIndex;

    // 帧头识别状态机
    uint8_t HeaderMatchCount; // 帧头匹配字节计数
    uint8_t PendingHeaderPos; // 待确认的帧头起始位置
    bool HeaderCrossWrap;     // 帧头跨越缓冲区边界标志
} DataBuffer;

/* 循环发送缓冲区结构体 */
typedef struct
{
    uint8_t buffer[UART_TX_BUFFER_SIZE]; // 缓冲区数组
    uint16_t head;                       // 头指针（写入位置）
    uint16_t tail;                       // 尾指针（读取位置）
    uint16_t count;                      // 当前缓冲区数据量
    bool isSending;                      // 发送状态标志
    UART_HandleTypeDef *huart;           // 关联的UART句柄
} UartTxRingBuffer;

/* 协议类型枚举 */
typedef enum
{
    PROTOCOL_SERVO = 0, // 舵机协议 (0x55 0x55)
    PROTOCOL_DART = 1   // DART协议 (DART)
} PROTOCOL_TYPE_e;

/* 协议数据包结构体 - 舵机协议 */
typedef struct
{
    uint8_t header[2]; // 包头 0x55 0x55
    uint8_t id;        // 舵机ID
    uint8_t length;    // 数据长度
    uint8_t cmd;       // 指令
    uint8_t params[8]; // 参数（最多8字节）
    uint8_t checksum;  // CRC校验
    uint8_t param_len; // 实际参数长度
    bool is_valid;     // 数据包是否有效
} ServoPacket_t;

/* 协议数据包结构体 - DART协议 */
typedef struct
{
    uint8_t header[4]; // 包头 "DART"
    uint8_t data[64];  // 数据内容
    uint8_t checksum;  // CRC校验
    uint16_t data_len; // 数据长度
    bool is_valid;     // 数据包是否有效
} DartPacket_t;

/* 协议解析状态 */
typedef enum
{
    PARSE_HEADER,       // 解析包头
    PARSE_SERVO_ID,     // 解析舵机ID
    PARSE_SERVO_LENGTH, // 解析数据长度
    PARSE_SERVO_CMD,    // 解析指令
    PARSE_SERVO_PARAMS, // 解析参数
    PARSE_SERVO_CRC,    // 解析CRC

    PARSE_DART_LENGTH, // DART数据长度
    PARSE_DART_DATA,   // 解析DART数据
    PARSE_DART_CRC,    // 解析DART的CRC
    PARSE_COMPLETE     // 解析完成
} ParseState_e;

/* 循环接收缓冲区结构体 */
typedef struct
{
    uint8_t buffer[UART_RX_BUFFER_SIZE]; // 缓冲区数组
    uint16_t head;                       // 头指针（写入位置，中断中使用）
    uint16_t tail;                       // 尾指针（读取位置，用户读取时使用）
    uint16_t count;                      // 当前缓冲区数据量
    bool isReceiving;                    // 接收状态标志
    bool overflowFlag;                   // 溢出标志
    uint8_t rxByte;                      // 单字节接收缓冲（用于中断接收）
    UART_HandleTypeDef *huart;           // 关联的UART句柄

    // 协议解析相关
    PROTOCOL_TYPE_e protocol_type; // 协议类型（由SERVO_COM决定）
    ParseState_e parse_state;      // 解析状态
    uint16_t parse_index;          // 解析索引
    ServoPacket_t servo_packet;    // 舵机协议数据包
    DartPacket_t dart_packet;      // DART协议数据包
    bool packet_ready;             // 是否有完整数据包
} UartRxRingBuffer;

/* ========== 用户API接口 ========== */

// 初始化BSP UART模块（初始化所有UART的缓冲区）
void BSP_UART_Init(void);

// 设置协议类型（根据SERVO_COM标志）
void UART_SetProtocol(BSP_UART_NUM_e uart_num, bool is_servo_mode);

// 发送数据
uint16_t UART_Send(BSP_UART_NUM_e uart_num, const uint8_t *data, uint16_t len);

// 发送字符串
uint16_t UART_SendString(BSP_UART_NUM_e uart_num, const char *str);

// 启动接收（开启中断接收）
void UART_StartRx(BSP_UART_NUM_e uart_num);

// 读取接收数据
uint16_t UART_Read(BSP_UART_NUM_e uart_num, uint8_t *data, uint16_t len);

// 获取接收缓冲区数据量
uint16_t UART_GetRxCount(BSP_UART_NUM_e uart_num);

// 检查接收缓冲区是否有数据
bool UART_HasData(BSP_UART_NUM_e uart_num);

// 清空接收缓冲区
void UART_ClearRx(BSP_UART_NUM_e uart_num);

/* ========== 协议解析API ========== */

// 检查是否有完整的数据包
bool UART_HasPacket(BSP_UART_NUM_e uart_num);

// 获取舵机协议数据包
bool UART_GetServoPacket(BSP_UART_NUM_e uart_num, ServoPacket_t *packet);

// 获取DART协议数据包
bool UART_GetDartPacket(BSP_UART_NUM_e uart_num, DartPacket_t *packet);

// 清除数据包标志
void UART_ClearPacket(BSP_UART_NUM_e uart_num);

/* ========== 工具函数 ========== */

// CRC校验生成（供其他模块使用，如HX06L）
uint8_t UART_Calculate_CRC(uint8_t *data, uint8_t length);

/* ========== HAL回调函数（在stm32f4xx_it.c中调用） ========== */
void BSP_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void BSP_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif
