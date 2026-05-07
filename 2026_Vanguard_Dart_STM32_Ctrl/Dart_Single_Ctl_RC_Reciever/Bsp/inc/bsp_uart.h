#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include "config.h"
#include "usart.h"
#include <stdbool.h>
#include <stdint.h>

/* 循环缓冲区大小定义 */
#define UART_TX_BUFFER_SIZE 64
#define UART_RX_BUFFER_SIZE 64
#define UART_TX_CHUNK_SIZE 32

/* UART编号枚举 */
typedef enum
{
    BSP_UART3 = 0,
    BSP_UART6,
    BSP_UART1,
    BSP_UART7,
    BSP_UART8,
    BSP_UART_MAX
} BSP_UART_NUM_e;

#define BSP_UART_IBUS BSP_UART6
#define BSP_UART_SBUS BSP_UART1

/* 循环发送缓冲区结构体 */
typedef struct
{
    uint8_t buffer[UART_TX_BUFFER_SIZE]; // 缓冲区数组
    uint16_t head;                       // 头指针（写入位置）
    uint16_t tail;                       // 尾指针（读取位置）
    uint16_t count;                      // 当前缓冲区数据量
    bool isSending;                      // 发送状态标志
    UART_HandleTypeDef *huart;           // 关联的UART句柄
    uint8_t temp_buffer[UART_TX_CHUNK_SIZE];
} UartTxRingBuffer;

/* 协议类型枚举 */
typedef enum
{
    PROTOCOL_SERVO_MCU = 0,    // 有MCU控制板协议 (0x55 0x55 | Length | Cmd | Params) - 无CRC
    PROTOCOL_SERVO_NO_MCU = 1, // 无MCU驱动板协议 (0x55 0x55 | ID | Length | Cmd | Params | CRC) - 有CRC
    PROTOCOL_IBUS = 2,         // IBUS协议 (0x20 0x40 | ... | CRC16)
    PROTOCOL_OTHER = 3         // 其他协议
} PROTOCOL_TYPE_e;

// 为了兼容性，保留PROTOCOL_SERVO别名
#define PROTOCOL_SERVO PROTOCOL_SERVO_MCU

#pragma pack(push, 1)
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

/* 协议数据包结构体 - IBUS协议（原始32字节） */
typedef struct
{
    uint8_t data[IBUS_FRAME_LEN];
    bool is_valid;     // 数据包是否有效
} IbusPacket_t;
#pragma pack(pop)

/* 协议解析状态 */
typedef enum
{
    PARSE_HEADER,       // 解析包头
    PARSE_SERVO_ID,     // 解析舵机ID
    PARSE_SERVO_LENGTH, // 解析数据长度
    PARSE_SERVO_CMD,    // 解析指令
    PARSE_SERVO_PARAMS, // 解析参数
    PARSE_SERVO_CRC,    // 解析CRC
    PARSE_COMPLETE     // 解析完成
} ParseState_e;

#pragma pack(push, 1)
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
    bool packet_ready;             // 是否有完整数据包

    // IBUS协议数据包
    IbusPacket_t ibus_packet;
    bool ibus_ready;
    uint32_t ibus_error_count;
    uint8_t ibus_stream[IBUS_STREAM_BUFFER_LEN];
    uint16_t ibus_stream_len;
} UartRxRingBuffer;
#pragma pack(pop)

/* ========== 用户API接口 ========== */

// 初始化BSP UART模块（初始化所有UART的缓冲区，自动启动接收）
void BSP_UART_Init(void);

// 设置协议类型（根据SERVO_COM标志）
void UART_SetProtocol(BSP_UART_NUM_e uart_num, bool is_servo_mode);

// 设置协议类型（推荐使用）
void UART_SetProtocolType(BSP_UART_NUM_e uart_num, PROTOCOL_TYPE_e protocol_type);

// 发送数据
uint16_t UART_Send(BSP_UART_NUM_e uart_num, const uint8_t *data, uint16_t len);

// 发送字符串
uint16_t UART_SendString(BSP_UART_NUM_e uart_num, const char *str);

// 重启接收（在接收中断被意外关闭时使用）
void UART_RestartRx(BSP_UART_NUM_e uart_num);

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

// 清除数据包标志
void UART_ClearPacket(BSP_UART_NUM_e uart_num);

// 检查是否有完整的IBUS数据包
bool UART_HasIbusPacket(BSP_UART_NUM_e uart_num);

// 获取IBUS数据包
bool UART_GetIbusPacket(BSP_UART_NUM_e uart_num, IbusPacket_t *packet);

// 清除IBUS数据包标志
void UART_ClearIbusPacket(BSP_UART_NUM_e uart_num);

/* ========== 工具函数 ========== */

// CRC校验生成（供其他模块使用，如HX06L）
uint8_t UART_Calculate_CRC(uint8_t *data, uint8_t length);

/* ========== HAL回调函数（在stm32f4xx_it.c中调用） ========== */
void BSP_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void BSP_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* ========== 缓冲区访问函数（供UartProtocol模块使用） ========== */
UartRxRingBuffer *BSP_UART_GetRxBuffer(BSP_UART_NUM_e uart_num);

#endif
