# UART 模块使用说明文档

## 架构概述

UART通信模块采用三层分离架构，确保底层驱动、协议解析和应用层接口的高度解耦：

```
┌─────────────────────────────────────────────┐
│            用户应用层 (UserTask.c 等)        │
└───────────────────────┬─────────────────────┘
                        ↓
┌─────────────────────────────────────────────┐
│     UartModule.c - 用户级数据包收发API       │
│   (User/src/UartModule.c)                   │
└───────────────────────┬─────────────────────┘
                        ↓
┌─────────────────────────────────────────────┐
│     UartProtocol.c - 协议解析逻辑与状态机    │
│   (Bsp/src/UartProtocol.c)                  │
└───────────────────────┬─────────────────────┘
                        ↓
┌─────────────────────────────────────────────┐
│     bsp_uart.c - 底层硬件驱动与环形缓冲区    │
│   (Bsp/src/bsp_uart.c)                      │
└─────────────────────────────────────────────┘
```

---

## 文件信息

| 层次 | 头文件 | 源文件 | 职责 |
|------|--------|--------|------|
| 用户层 | `User/inc/UartModule.h` | `User/src/UartModule.c` | 提供面向数据包的收发API、协议切换 |
| 协议层 | `Bsp/inc/UartProtocol.h` | `Bsp/src/UartProtocol.c` | 协议解析与校验（舵机协议、IBUS协议） |
| 驱动层 | `Bsp/inc/bsp_uart.h` | `Bsp/src/bsp_uart.c` | 硬件中断处理、环形发送/接收缓冲区管理 |

---

## 重要设计特性

### 1. 环形缓冲区与IBUS DMA
舵机协议使用环形缓冲区进行字节接收与解析；IBUS 协议由 UART6 通过 DMA 固定长度（32字节）接收并在中断中校验帧头和校验尾。
- **简化解析**: 舵机协议按字节状态机解析；IBUS 仅校验帧头与 16 位校验尾。
- **数据隔离**: IBUS 数据不会写入舵机环形缓冲区，避免协议耦合。

### 2. 多协议状态机
支持三种协议类型的实时解析：
- `PROTOCOL_SERVO_MCU`: 舵机控制板协议（0x55 0x55开头，无CRC）。
- `PROTOCOL_SERVO_NO_MCU`: 舵机驱动板直接协议（0x55 0x55开头，有CRC）。
- `PROTOCOL_IBUS`: 遥控器 IBUS 协议（0x20 0x40 帧头，16位校验）。

---

## 重要宏定义

```c
#define UART_TX_BUFFER_SIZE 64          // 发送环形缓冲区大小
#define UART_RX_BUFFER_SIZE 64          // 接收环形缓冲区大小
#define IBUS_FRAME_LEN 32               // IBUS固定帧长度
```

---

## 数据结构

### BSP_UART_NUM_e 串口编号枚举
```c
typedef enum {
    BSP_UART3 = 0,  // 对应 UART3 (通常用于舵机)
    BSP_UART6,      // 对应 UART6 (通常用于上位机)
    BSP_UART7,      // 对应 UART7
    BSP_UART8,      // 对应 UART8
    BSP_UART_MAX
} BSP_UART_NUM_e;
```

### 数据包结构体
- `ServoPacket_t`: 包含 ID、指令、参数数组（最大8字节）及有效性标志。
- `IbusPacket_t`: 原始32字节IBUS帧与有效性标志。

---

## 用户层API (UartModule.h) 【推荐使用】

### 1. 数据接收
- `UartModule_HasServoPacket(uart_num)`: 检查是否有完整舵机包。
- `UartModule_GetServoPacket(uart_num, packet)`: 获取并解析舵机包。
- `UartModule_ClearPacket(uart_num)`: 手动清除舵机数据包标志。
- `UartModule_HasIbusPacket(uart_num)`: 检查是否有完整IBUS包。
- `UartModule_GetIbusPacket(uart_num, packet)`: 获取IBUS包（原始32字节）。
- `UartModule_ClearIbusPacket(uart_num)`: 手动清除IBUS数据包标志。

### 2. 数据发送
- `UartModule_SendServoCmd(uart_num, id, cmd, params, len)`: 构建并发送舵机命令。
- `UartModule_SendRaw(uart_num, data, len)`: 发送不带协议封装的原始字节流。

### 3. 配置与状态
- `UartModule_SetProtocol(uart_num, type)`: 运行时切换串口协议。
- `UartModule_GetFrameCount(uart_num)`: 获取缓冲区中当前有效的完整帧数量。

---

## 典型使用示例

### 舵机双向通信
```c
// 发送控制命令
uint8_t move_params[] = {0xE8, 0x03}; // 目标位置1000
UartModule_SendServoCmd(BSP_UART3, 1, 0x03, move_params, 2);

// 处理反馈
ServoPacket_t rx_pkt;
if (UartModule_HasServoPacket(BSP_UART3)) {
    if (UartModule_GetServoPacket(BSP_UART3, &rx_pkt)) {
        // 处理 rx_pkt.params...
    }
}
```

### IBUS协议接收
```c
IbusPacket_t pkt;
if (UartModule_HasIbusPacket(BSP_UART6)) {
    if (UartModule_GetIbusPacket(BSP_UART6, &pkt)) {
        // 处理 pkt.data
    }
}
```

---

## 注意事项

1. **初始化**: 必须调用 `BSP_UART_Init()` 来初始化缓冲区和硬件中断。
2. **CRC校验**: 
   - `PROTOCOL_SERVO_NO_MCU` 使用 8位 CRC（`CRC = ~(Sum(data)) & 0xFF`）。
   - `PROTOCOL_IBUS` 使用 16位校验（`Checksum = 0xFFFF - Sum(frame[0..29])`）。
3. **并发安全**: 发送操作是线程安全的（通过 `isSending` 标志协调环形缓冲区读取）。
4. **中断恢复**: 若因过载导致串口停机，系统会自动通过 `UART_RestartRx` 尝试恢复。

---

## 更新日志

- **2026-01-03**:
  - 扩展串口支持至 UART8。
  - 优化协议解析结构，IBUS 与舵机协议分离。
  - UART6 使用 DMA 接收固定32字节 IBUS 帧。
