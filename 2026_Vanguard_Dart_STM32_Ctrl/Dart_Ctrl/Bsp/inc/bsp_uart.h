#ifndef __BSP_UART_H /* 按 __BSP_UART_H 选择编译分支。 */
#define __BSP_UART_H /* 定义 __BSP_UART_H。 */

#include "main.h"
#include "config.h"
#include "usart.h"
#include <stdbool.h>
#include <stdint.h>

/* 循环缓冲区大小定义 */
#define UART_TX_BUFFER_SIZE 128 /* 定义 UART_TX_BUFFER_SIZE。 */
#define UART_RX_BUFFER_SIZE 64 /* 定义 UART_RX_BUFFER_SIZE。 */
#define UART_TX_CHUNK_SIZE 32 /* 定义 UART_TX_CHUNK_SIZE。 */

/* 舵机包队列深度 */
#define SERVO_PACKET_QUEUE_SIZE 2 /* 定义 SERVO_PACKET_QUEUE_SIZE。 */
#define REFEREE_PACKET_QUEUE_SIZE 4 /* 定义 REFEREE_PACKET_QUEUE_SIZE。 */
#define REFEREE_FRAME_MAX_SIZE 128 /* 定义 REFEREE_FRAME_MAX_SIZE。 */

/* UART编号枚举 */
typedef enum /* 开始定义数据类型。 */
{
    BSP_UART3 = 0, // 控制总线舵机
    BSP_UART6,     // IBUS通信口
    BSP_UART7, /* 定义 BSP_UART7 枚举项。 */
    BSP_UART8, // 裁判系统串口接收数据
    BSP_UART_MAX /* 定义 BSP_UART_MAX 枚举项。 */
} BSP_UART_NUM_e; /* 结束 BSP_UART_NUM_e 类型定义。 */

/* 循环发送缓冲区结构体 */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t buffer[UART_TX_BUFFER_SIZE]; // 缓冲区数组
    uint16_t head;                       // 头指针（写入位置）
    uint16_t tail;                       // 尾指针（读取位置）
    uint16_t count;                      // 当前缓冲区数据量
    bool isSending;                      // 发送状态标志
    UART_HandleTypeDef *huart;           // 关联的UART句柄
    uint8_t temp_buffer[UART_TX_CHUNK_SIZE]; /* 保存 temp_buffer。 */
} UartTxRingBuffer; /* 结束 UartTxRingBuffer 类型定义。 */

/* 协议类型枚举 */
typedef enum /* 开始定义数据类型。 */
{
    PROTOCOL_SERVO_MCU = 0,    // 有MCU控制板协议 (0x55 0x55 | Length | Cmd | Params) - 无CRC
    PROTOCOL_SERVO_NO_MCU = 1, // 无MCU驱动板协议 (0x55 0x55 | ID | Length | Cmd | Params | CRC) - 有CRC
    PROTOCOL_IBUS = 2,         // IBUS协议 (0x20 0x40 | ... | CRC16)
    PROTOCOL_OTHER = 3,        // 其他协议
    PROTOCOL_REFEREE = 4       // 裁判系统协议
} PROTOCOL_TYPE_e; /* 结束 PROTOCOL_TYPE_e 类型定义。 */

// 为了兼容性，保留PROTOCOL_SERVO别名
#define PROTOCOL_SERVO PROTOCOL_SERVO_MCU /* 定义 PROTOCOL_SERVO。 */

#pragma pack(push, 1) /* 配置编译选项 pack(push, 1)。 */
/* 协议数据包结构体 - 舵机协议 */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t header[2]; // 包头 0x55 0x55
    uint8_t id;        // 舵机ID
    uint8_t length;    // 数据长度
    uint8_t cmd;       // 指令
    uint8_t params[8]; // 参数（最多8字节）
    uint8_t checksum;  // CRC校验
    uint8_t param_len; // 实际参数长度
    bool is_valid;     // 数据包是否有效
} ServoPacket_t; /* 结束 ServoPacket_t 类型定义。 */

/* 协议数据包结构体 - IBUS协议（原始32字节） */
typedef struct /* 开始定义数据类型。 */
{
    uint8_t data[IBUS_FRAME_LEN]; /* 保存 data。 */
    bool is_valid; // 数据包是否有效
} IbusPacket_t; /* 结束 IbusPacket_t 类型定义。 */

typedef struct /* 开始定义数据类型。 */
{
    uint8_t data[REFEREE_FRAME_MAX_SIZE]; /* 保存 data。 */
    uint16_t length; /* 保存 length。 */
    bool is_valid; /* 保存 is_valid。 */
} RefereePacket_t; /* 结束 RefereePacket_t 类型定义。 */
#pragma pack(pop) /* 配置编译选项 pack(pop)。 */

/* 协议解析状态 */
typedef enum /* 开始定义数据类型。 */
{
    PARSE_HEADER,       // 解析包头
    PARSE_SERVO_ID,     // 解析舵机ID
    PARSE_SERVO_LENGTH, // 解析数据长度
    PARSE_SERVO_CMD,    // 解析指令
    PARSE_SERVO_PARAMS, // 解析参数
    PARSE_SERVO_CRC,    // 解析CRC
    PARSE_COMPLETE      // 解析完成
} ParseState_e; /* 结束 ParseState_e 类型定义。 */

#pragma pack(push, 1) /* 配置编译选项 pack(push, 1)。 */
/* 循环接收缓冲区结构体 */
typedef struct /* 开始定义数据类型。 */
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
    PROTOCOL_TYPE_e protocol_type;   // 协议类型（由SERVO_COM决定）
    ParseState_e parse_state;        // 解析状态
    uint16_t parse_index;            // 解析索引
    ServoPacket_t servo_packet_temp; // 解析临时包（状态机逐字节填充）

    // 舵机包队列（ISR写入，任务读取）
    ServoPacket_t servo_packet_queue[SERVO_PACKET_QUEUE_SIZE]; /* 保存 servo_packet_queue。 */
    uint8_t servo_pkt_head;  // ISR 写入位置
    uint8_t servo_pkt_tail;  // 任务读取位置
    uint8_t servo_pkt_count; // 队列中包数量

    // IBUS协议数据包
    IbusPacket_t ibus_packet; /* 保存 ibus_packet。 */
    bool ibus_ready; /* 保存 ibus_ready。 */
    uint32_t ibus_error_count; /* 保存 ibus_error_count。 */
    uint8_t ibus_stream[IBUS_STREAM_BUFFER_LEN]; /* 保存 ibus_stream。 */
    uint16_t ibus_stream_len; /* 保存 ibus_stream_len。 */

    // 裁判系统协议数据包
    RefereePacket_t referee_packet_queue[REFEREE_PACKET_QUEUE_SIZE]; /* 保存 referee_packet_queue。 */
    uint8_t referee_pkt_head; /* 保存 referee_pkt_head。 */
    uint8_t referee_pkt_tail; /* 保存 referee_pkt_tail。 */
    uint8_t referee_pkt_count; /* 保存 referee_pkt_count。 */
    uint8_t referee_frame[REFEREE_FRAME_MAX_SIZE]; /* 保存 referee_frame。 */
    uint16_t referee_frame_len; /* 保存 referee_frame_len。 */
    uint16_t referee_expected_len; /* 保存 referee_expected_len。 */
} UartRxRingBuffer; /* 结束 UartRxRingBuffer 类型定义。 */
#pragma pack(pop) /* 配置编译选项 pack(pop)。 */

/* ========== 用户API接口 ========== */

extern volatile uint32_t Uart8RxDebug_ByteCount; /* 声明外部变量 Uart8RxDebug_ByteCount。 */
extern volatile uint8_t Uart8RxDebug_LastByte; /* 声明外部变量 Uart8RxDebug_LastByte。 */
extern volatile uint32_t Uart8RxDebug_LastTick; /* 声明外部变量 Uart8RxDebug_LastTick。 */
extern volatile uint32_t Uart8RxDebug_RestartCount; /* 声明外部变量 Uart8RxDebug_RestartCount。 */
extern volatile uint32_t Uart8RxDebug_ReceiveItFailCount; /* 声明外部变量 Uart8RxDebug_ReceiveItFailCount。 */
extern volatile uint32_t Uart8RxDebug_LastReceiveStatus; /* 声明外部变量 Uart8RxDebug_LastReceiveStatus。 */
extern volatile uint32_t Uart8RxDebug_ErrorCount; /* 声明外部变量 Uart8RxDebug_ErrorCount。 */
extern volatile uint32_t Uart8RxDebug_LastErrorCode; /* 声明外部变量 Uart8RxDebug_LastErrorCode。 */

// 初始化BSP UART模块（初始化所有UART的缓冲区，自动启动接收）
void BSP_UART_Init(void); /* 声明 BSP_UART_Init 接口。 */

// 设置协议类型（根据SERVO_COM标志）
void UART_SetProtocol(BSP_UART_NUM_e uart_num, bool is_servo_mode); /* 声明 UART_SetProtocol 接口。 */

// 设置协议类型（推荐使用）
void UART_SetProtocolType(BSP_UART_NUM_e uart_num, PROTOCOL_TYPE_e protocol_type); /* 声明 UART_SetProtocolType 接口。 */

// 发送数据
uint16_t UART_Send(BSP_UART_NUM_e uart_num, const uint8_t *data, uint16_t len); /* 声明 UART_Send 接口。 */

// 发送字符串
uint16_t UART_SendString(BSP_UART_NUM_e uart_num, const char *str); /* 声明 UART_SendString 接口。 */

// 重启接收（在接收中断被意外关闭时使用）
void UART_RestartRx(BSP_UART_NUM_e uart_num); /* 声明 UART_RestartRx 接口。 */

// 读取接收数据
uint16_t UART_Read(BSP_UART_NUM_e uart_num, uint8_t *data, uint16_t len); /* 声明 UART_Read 接口。 */

// 获取接收缓冲区数据量
uint16_t UART_GetRxCount(BSP_UART_NUM_e uart_num); /* 声明 UART_GetRxCount 接口。 */

// 检查接收缓冲区是否有数据
bool UART_HasData(BSP_UART_NUM_e uart_num); /* 声明 UART_HasData 接口。 */

// 清空接收缓冲区
void UART_ClearRx(BSP_UART_NUM_e uart_num); /* 声明 UART_ClearRx 接口。 */

/* ========== 协议解析API ========== */

// 检查是否有完整的数据包
bool UART_HasPacket(BSP_UART_NUM_e uart_num); /* 声明 UART_HasPacket 接口。 */

// 获取舵机协议数据包
bool UART_GetServoPacket(BSP_UART_NUM_e uart_num, ServoPacket_t *packet); /* 声明 UART_GetServoPacket 接口。 */

// 清除数据包标志
void UART_ClearPacket(BSP_UART_NUM_e uart_num); /* 声明 UART_ClearPacket 接口。 */

// 检查是否有完整的IBUS数据包
bool UART_HasIbusPacket(BSP_UART_NUM_e uart_num); /* 声明 UART_HasIbusPacket 接口。 */

// 获取IBUS数据包
bool UART_GetIbusPacket(BSP_UART_NUM_e uart_num, IbusPacket_t *packet); /* 声明 UART_GetIbusPacket 接口。 */

// 清除IBUS数据包标志
void UART_ClearIbusPacket(BSP_UART_NUM_e uart_num); /* 声明 UART_ClearIbusPacket 接口。 */

// 检查是否有完整的裁判系统数据包
bool UART_HasRefereePacket(BSP_UART_NUM_e uart_num); /* 声明 UART_HasRefereePacket 接口。 */

// 获取裁判系统数据包
bool UART_GetRefereePacket(BSP_UART_NUM_e uart_num, RefereePacket_t *packet); /* 声明 UART_GetRefereePacket 接口。 */

// 清除裁判系统数据包标志
void UART_ClearRefereePacket(BSP_UART_NUM_e uart_num); /* 声明 UART_ClearRefereePacket 接口。 */

/* ========== 工具函数 ========== */

// CRC校验生成（供其他模块使用，如HX06L）
uint8_t UART_Calculate_CRC(uint8_t *data, uint8_t length); /* 声明 UART_Calculate_CRC 接口。 */

/* ========== HAL回调函数（在stm32f4xx_it.c中调用） ========== */
void BSP_UART_TxCpltCallback(UART_HandleTypeDef *huart); /* 声明 BSP_UART_TxCpltCallback 接口。 */
void BSP_UART_RxCpltCallback(UART_HandleTypeDef *huart); /* 声明 BSP_UART_RxCpltCallback 接口。 */

/* ========== 缓冲区访问函数（供UartProtocol模块使用） ========== */
UartRxRingBuffer *BSP_UART_GetRxBuffer(BSP_UART_NUM_e uart_num); /* 声明 BSP_UART_GetRxBuffer 接口。 */

#endif /* 结束条件编译。 */
