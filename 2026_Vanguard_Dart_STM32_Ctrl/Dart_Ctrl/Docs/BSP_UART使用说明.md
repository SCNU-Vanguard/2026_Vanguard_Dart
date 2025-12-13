# UART 模块使用说明文档

## 架构概述

UART通信模块采用三层分离架构，便于维护和扩展：

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
│     UartProtocol.c - 协议解析内部逻辑        │
│   (Bsp/src/UartProtocol.c)                  │
└───────────────────────┬─────────────────────┘
                        ↓
┌─────────────────────────────────────────────┐
│     bsp_uart.c - 底层硬件驱动                │
│   (Bsp/src/bsp_uart.c)                      │
└─────────────────────────────────────────────┘
```

---

## 文件信息

| 层次 | 头文件 | 源文件 | 职责 |
|------|--------|--------|------|
| 用户层 | `User/inc/UartModule.h` | `User/src/UartModule.c` | 数据包获取/发送、协议设置 |
| 协议层 | `User/inc/UartProtocol.h` | `Bsp/src/UartProtocol.c` | 协议解析状态机、帧头识别、CRC校验 |
| 驱动层 | `Bsp/inc/bsp_uart.h` | `Bsp/src/bsp_uart.c` | 环形缓冲区、HAL回调、底层收发 |

**推荐**: 用户应用层只需包含 `UartModule.h`，无需直接操作底层。

---

## 重要宏定义

```c
#define UART_TX_BUFFER_SIZE 64          // 发送缓冲区大小
#define UART_RX_BUFFER_SIZE 64          // 接收缓冲区大小
#define DART_FRAME_INDEX_SIZE 16        // DART协议帧索引数组大小
#define SERVO_FRAME_INDEX_SIZE 32       // 舵机协议帧索引数组大小
```

---

## 数据结构

### BSP_UART_NUM_e 串口编号枚举
```c
typedef enum {
    BSP_UART3 = 0,  // UART3
    BSP_UART6,      // UART6
    BSP_UART_MAX
} BSP_UART_NUM_e;
```

### PROTOCOL_TYPE_e 协议类型枚举
```c
typedef enum {
    PROTOCOL_SERVO_MCU = 0,    // 有MCU控制板协议 (无CRC)
    PROTOCOL_SERVO_NO_MCU = 1, // 无MCU驱动板协议 (有CRC)
    PROTOCOL_DART = 2,         // DART协议
    PROTOCOL_OTHER = -1        // 其他协议
} PROTOCOL_TYPE_e;

#define PROTOCOL_SERVO PROTOCOL_SERVO_MCU  // 兼容性别名
```

### ServoPacket_t 舵机协议数据包
```c
typedef struct {
    uint8_t header[2];   // 包头 0x55 0x55
    uint8_t id;          // 舵机ID
    uint8_t length;      // 数据长度
    uint8_t cmd;         // 指令
    uint8_t params[8];   // 参数（最多8字节）
    uint8_t checksum;    // CRC校验
    uint8_t param_len;   // 实际参数长度
    bool is_valid;       // 数据包是否有效
} ServoPacket_t;
```

### DartPacket_t DART协议数据包
```c
typedef struct {
    uint8_t header[4];   // 包头 "DART"
    uint8_t data[64];    // 数据内容
    uint8_t checksum;    // CRC校验
    uint16_t data_len;   // 数据长度
    bool is_valid;       // 数据包是否有效
} DartPacket_t;
```

---

## 用户层API (UartModule.h) 【推荐使用】

### 数据包接收

#### UartModule_HasServoPacket()
**功能**: 检查是否有完整的舵机数据包
```c
bool UartModule_HasServoPacket(BSP_UART_NUM_e uart_num);
```

#### UartModule_HasDartPacket()
**功能**: 检查是否有完整的DART数据包
```c
bool UartModule_HasDartPacket(BSP_UART_NUM_e uart_num);
```

#### UartModule_GetServoPacket()
**功能**: 获取舵机协议数据包
```c
bool UartModule_GetServoPacket(BSP_UART_NUM_e uart_num, ServoPacket_t *packet);
```

**使用示例**:
```c
ServoPacket_t servo_pkt;
if (UartModule_HasServoPacket(BSP_UART3)) {
    if (UartModule_GetServoPacket(BSP_UART3, &servo_pkt)) {
        printf("ID: %d, Cmd: 0x%02X\n", servo_pkt.id, servo_pkt.cmd);
    }
}
```

#### UartModule_GetDartPacket()
**功能**: 获取DART协议数据包
```c
bool UartModule_GetDartPacket(BSP_UART_NUM_e uart_num, DartPacket_t *packet);
```

**使用示例**:
```c
DartPacket_t dart_pkt;
if (UartModule_HasDartPacket(BSP_UART6)) {
    if (UartModule_GetDartPacket(BSP_UART6, &dart_pkt)) {
        // 处理 dart_pkt.data，长度为 dart_pkt.data_len
    }
}
```

#### UartModule_ClearPacket()
**功能**: 清除数据包标志
```c
void UartModule_ClearPacket(BSP_UART_NUM_e uart_num);
```

---

### 数据包发送

#### UartModule_SendServoCmd()
**功能**: 发送舵机命令数据包（自动构建协议格式）
```c
bool UartModule_SendServoCmd(BSP_UART_NUM_e uart_num, uint8_t id, uint8_t cmd, 
                             uint8_t *params, uint8_t param_len);
```

**参数说明**:
- `uart_num`: UART编号
- `id`: 舵机ID
- `cmd`: 命令码
- `params`: 参数数组（可为NULL）
- `param_len`: 参数长度（0-8）

**使用示例**:
```c
// 发送读取位置命令
UartModule_SendServoCmd(BSP_UART3, 0x01, 0x1C, NULL, 0);

// 发送写入位置命令
uint8_t params[] = {0xE8, 0x03, 0x00, 0x00};  // 位置1000
UartModule_SendServoCmd(BSP_UART3, 0x01, 0x03, params, 4);
```

**注意**: 函数会根据当前协议类型（SERVO_MCU/SERVO_NO_MCU）自动处理CRC。

#### UartModule_SendDartPacket()
**功能**: 发送DART数据包（自动添加帧头和CRC）
```c
bool UartModule_SendDartPacket(BSP_UART_NUM_e uart_num, uint8_t *data, uint16_t data_len);
```

**使用示例**:
```c
uint8_t data[] = "Hello";
UartModule_SendDartPacket(BSP_UART6, data, 5);
// 实际发送: 'D' 'A' 'R' 'T' 0x05 'H' 'e' 'l' 'l' 'o' CRC
```

#### UartModule_SendRaw()
**功能**: 发送原始数据（不做协议封装）
```c
uint16_t UartModule_SendRaw(BSP_UART_NUM_e uart_num, const uint8_t *data, uint16_t len);
```

---

### 设置函数

#### UartModule_SetProtocol()
**功能**: 设置UART协议类型
```c
void UartModule_SetProtocol(BSP_UART_NUM_e uart_num, PROTOCOL_TYPE_e protocol_type);
```

**使用示例**:
```c
UartModule_SetProtocol(BSP_UART3, PROTOCOL_SERVO_MCU);    // 有MCU舵机协议
UartModule_SetProtocol(BSP_UART3, PROTOCOL_SERVO_NO_MCU); // 无MCU舵机协议
UartModule_SetProtocol(BSP_UART6, PROTOCOL_DART);         // DART协议
```

#### UartModule_GetFrameCount()
**功能**: 获取有效帧数量
```c
uint8_t UartModule_GetFrameCount(BSP_UART_NUM_e uart_num);
```

---

## 底层API (bsp_uart.h)

### 初始化

#### BSP_UART_Init()
**功能**: 初始化BSP UART模块
```c
void BSP_UART_Init(void);
```

**自动完成的工作**:
- 初始化所有UART的发送/接收缓冲区
- 设置默认协议（见下表）
- 自动启动中断接收

**默认协议配置**:
| 串口 | 默认协议 | 用途 |
|-----|---------|------|
| BSP_UART3 | PROTOCOL_SERVO_MCU | 舵机通信 |
| BSP_UART6 | PROTOCOL_DART | 上位机通信 |

---

### 基础收发

#### UART_Send()
**功能**: 发送数据
```c
uint16_t UART_Send(BSP_UART_NUM_e uart_num, const uint8_t *data, uint16_t len);
```

#### UART_SendString()
**功能**: 发送字符串
```c
uint16_t UART_SendString(BSP_UART_NUM_e uart_num, const char *str);
```

#### UART_Read()
**功能**: 读取原始接收数据
```c
uint16_t UART_Read(BSP_UART_NUM_e uart_num, uint8_t *data, uint16_t len);
```

#### UART_HasData()
**功能**: 检查是否有接收数据
```c
bool UART_HasData(BSP_UART_NUM_e uart_num);
```

#### UART_GetRxCount()
**功能**: 获取接收缓冲区数据量
```c
uint16_t UART_GetRxCount(BSP_UART_NUM_e uart_num);
```

#### UART_ClearRx()
**功能**: 清空接收缓冲区
```c
void UART_ClearRx(BSP_UART_NUM_e uart_num);
```

#### UART_RestartRx()
**功能**: 重启接收（接收中断被意外关闭时使用）
```c
void UART_RestartRx(BSP_UART_NUM_e uart_num);
```

---

## 协议格式说明

### 舵机协议 (SERVO_MCU - 有MCU控制板，无CRC)
```
帧头 | Length | Cmd | Params
2字节   1字节   1字节  N字节
0x55 0x55
```

**Length说明**: `Length = Cmd(1) + Params(N) + Length本身(1) = N + 2`

### 舵机协议 (SERVO_NO_MCU - 无MCU驱动板，有CRC)
```
帧头 | ID | Length | Cmd | Params | CRC
2字节 1字节  1字节   1字节 N字节   1字节
0x55 0x55
```

**Length说明**: `Length = Cmd(1) + Params(N) + CRC(1) = N + 3`

### DART协议
```
帧头   | Length | Data    | CRC
4字节   1字节    N字节    1字节
"DART"
```

---

## 典型应用示例

### 示例1: 舵机通信（推荐方式）
```c
#include "UartModule.h"

void ServoTask(void)
{
    // 初始化（在main中调用一次）
    BSP_UART_Init();
    
    // 发送舵机命令
    uint8_t pos_params[] = {0xE8, 0x03};  // 位置1000
    UartModule_SendServoCmd(BSP_UART3, 0x01, 0x03, pos_params, 2);
    
    // 接收舵机反馈
    ServoPacket_t pkt;
    if (UartModule_HasServoPacket(BSP_UART3)) {
        if (UartModule_GetServoPacket(BSP_UART3, &pkt)) {
            printf("收到舵机反馈: ID=%d, Cmd=0x%02X\n", pkt.id, pkt.cmd);
            for (int i = 0; i < pkt.param_len; i++) {
                printf("  Param[%d] = 0x%02X\n", i, pkt.params[i]);
            }
        }
        UartModule_ClearPacket(BSP_UART3);
    }
}
```

### 示例2: DART上位机通信
```c
#include "UartModule.h"

void DartCommTask(void)
{
    // 发送数据到上位机
    uint8_t tx_data[] = {0x01, 0x02, 0x03, 0x04};
    UartModule_SendDartPacket(BSP_UART6, tx_data, sizeof(tx_data));
    
    // 接收上位机数据
    DartPacket_t pkt;
    if (UartModule_HasDartPacket(BSP_UART6)) {
        if (UartModule_GetDartPacket(BSP_UART6, &pkt)) {
            printf("收到DART数据包: 长度=%d\n", pkt.data_len);
            // 处理 pkt.data
        }
        UartModule_ClearPacket(BSP_UART6);
    }
}
```

### 示例3: 切换协议类型
```c
// 切换到无MCU舵机协议（有CRC）
UartModule_SetProtocol(BSP_UART3, PROTOCOL_SERVO_NO_MCU);

// 切换回有MCU舵机协议（无CRC）
UartModule_SetProtocol(BSP_UART3, PROTOCOL_SERVO_MCU);
```

### 示例4: 原始数据收发
```c
// 发送原始数据
uint8_t raw_data[] = {0x55, 0x55, 0x01, 0x04, 0x01, 0xFA};
UartModule_SendRaw(BSP_UART3, raw_data, sizeof(raw_data));

// 或使用底层API
UART_Send(BSP_UART3, raw_data, sizeof(raw_data));
```

---

## 使用注意事项

1. **初始化顺序**
   ```c
   BSP_UART_Init();  // 必须首先调用
   // 之后可以使用所有UART功能
   ```

2. **推荐使用UartModule层**
   - 自动处理协议封装和解析
   - 简化应用层代码
   - 隐藏底层细节

3. **协议类型**
   - SERVO_MCU: 有MCU控制板，无CRC校验
   - SERVO_NO_MCU: 无MCU驱动板，有CRC校验
   - 根据实际舵机类型选择

4. **CRC校验**
   - 算法: `CRC = ~(累加和) & 0xFF`
   - UartModule_SendServoCmd 会自动计算CRC
   - UartModule_SendDartPacket 会自动计算CRC

5. **缓冲区溢出**
   - 缓冲区满时会自动覆盖旧数据
   - 建议定期读取数据避免溢出

---

## 调试建议

```c
// 检查帧数量
uint8_t count = UartModule_GetFrameCount(BSP_UART3);
printf("缓冲区中有 %d 个有效帧\n", count);

// 检查是否有数据包
if (UartModule_HasServoPacket(BSP_UART3)) {
    printf("有舵机数据包\n");
}
if (UartModule_HasDartPacket(BSP_UART6)) {
    printf("有DART数据包\n");
}
```

---

## 常见问题

**Q: 收不到数据？**
A: 
1. 检查是否调用了 `BSP_UART_Init()`
2. 检查协议类型是否正确
3. 如接收中断被关闭，调用 `UART_RestartRx()`

**Q: CRC校验失败？**
A: 
1. 检查协议类型（SERVO_MCU无CRC，SERVO_NO_MCU有CRC）
2. 检查数据格式是否正确

**Q: UartModule 和 bsp_uart 的区别？**
A: 
- `UartModule`: 用户层，提供数据包级别的API
- `bsp_uart`: 底层，提供字节级别的API
- 推荐使用 `UartModule`

---

## 更新日志

| 日期 | 更新内容 |
|-----|---------|
| 2025/12/13 | 重构为三层架构：UartModule → UartProtocol → bsp_uart |
| 2025/12/13 | 新增 UartModule 用户层API |
| 2025/12/13 | 支持 SERVO_MCU 和 SERVO_NO_MCU 两种舵机协议 |
| 2025/12/08 | `BSP_UART_Init()` 自动启动接收和设置默认协议 |
| - | 支持环形缓冲区自动管理 |
| - | 实现舵机和DART双协议解析 |
| - | 自动帧头识别和CRC校验 |
