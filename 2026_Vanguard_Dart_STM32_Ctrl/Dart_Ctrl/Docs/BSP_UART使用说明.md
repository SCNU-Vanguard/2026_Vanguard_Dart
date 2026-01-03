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
| 协议层 | `Bsp/inc/UartProtocol.h` | `Bsp/src/UartProtocol.c` | 协议状态机、帧头识别、CRC校验、帧位置记录 |
| 驱动层 | `Bsp/inc/bsp_uart.h` | `Bsp/src/bsp_uart.c` | 硬件中断处理、环形发送/接收缓冲区管理 |

---

## 重要设计特性

### 1. 环形缓冲区与帧索引
底层采用高效的环形缓冲区（Ring Buffer），并配合 `FrameIndexBuffer` 记录每一帧在缓冲区中的起始和结束位置。
- **自动覆盖**: 当缓冲区满时，新数据会自动覆盖旧数据，同时相应的失效帧索引会被标记为无效。
- **零拷贝思想**: 协议解析直接在接收缓冲区上进行，仅在用户读取完整数据包时进行必要的内存拷贝。

### 2. 多协议状态机
支持三种协议类型的实时解析：
- `PROTOCOL_SERVO_MCU`: 舵机控制板协议（0x55 0x55开头，无CRC）。
- `PROTOCOL_SERVO_NO_MCU`: 舵机驱动板直接协议（0x55 0x55开头，有CRC）。
- `PROTOCOL_DART`: 飞镖系统自定义协议（"DART"字符串开头，有CRC）。

---

## 重要宏定义

```c
#define UART_TX_BUFFER_SIZE 64          // 发送环形缓冲区大小
#define UART_RX_BUFFER_SIZE 64          // 接收环形缓冲区大小
#define DART_FRAME_INDEX_SIZE 16        // DART协议最大记录帧数
#define SERVO_FRAME_INDEX_SIZE 32       // 舵机协议最大记录帧数
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
- `DartPacket_t`: 包含 64字节数据区、长度信息及有效性标志。

---

## 用户层API (UartModule.h) 【推荐使用】

### 1. 数据接收
- `UartModule_HasServoPacket(uart_num)`: 检查是否有完整舵机包。
- `UartModule_GetServoPacket(uart_num, packet)`: 获取并解析舵机包。
- `UartModule_HasDartPacket(uart_num)`: 检查是否有完整DART包。
- `UartModule_GetDartPacket(uart_num, packet)`: 获取并解析DART包。
- `UartModule_ClearPacket(uart_num)`: 手动清除当前数据包标志。

### 2. 数据发送
- `UartModule_SendServoCmd(uart_num, id, cmd, params, len)`: 构建并发送舵机命令。
- `UartModule_SendDartPacket(uart_num, data, len)`: 构建并发送DART协议包（自动加帧头和CRC）。
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

### DART协议通信
```c
uint8_t sensor_data[10] = { ... };
UartModule_SendDartPacket(BSP_UART6, sensor_data, 10);
```

---

## 注意事项

1. **初始化**: 必须调用 `BSP_UART_Init()` 来初始化缓冲区和硬件中断。
2. **CRC校验**: 
   - `PROTOCOL_SERVO_NO_MCU` 和 `PROTOCOL_DART` 强制使用 CRC。
   - 算法: `CRC = ~(Sum(data)) & 0xFF`。
3. **并发安全**: 发送操作是线程安全的（通过 `isSending` 标志协调环形缓冲区读取）。
4. **中断恢复**: 若因过载导致串口停机，系统会自动通过 `UART_RestartRx` 尝试恢复。

---

## 更新日志

- **2026-01-03**:
  - 扩展串口支持至 UART8。
  - 优化协议状态机，支持多字节帧头（"DART"）识别。
  - 引入 `FrameIndexBuffer` 实现高效的帧定位与失效机制。
  - 将协议逻辑完全从驱动层分离至 `UartProtocol`。
