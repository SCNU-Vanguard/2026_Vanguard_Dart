# BSP_UART 使用说明文档

## 文件信息
- **头文件**: `Bsp/inc/bsp_uart.h`
- **源文件**: `Bsp/src/bsp_uart.c`
- **作用**: UART串口通信的底层硬件抽象层(BSP)，支持环形缓冲区和协议解析

---

## 功能概述

本模块封装了STM32的UART串口通信功能，提供了强大的环形缓冲区管理和协议解析能力。

### 核心功能
1. **环形缓冲区** - 发送和接收数据的环形缓冲管理
2. **协议解析** - 支持舵机协议(0x55 0x55)和DART协议
3. **帧识别** - 自动识别数据帧的帧头和帧尾
4. **中断收发** - 基于中断的异步收发机制
5. **溢出保护** - 自动处理缓冲区溢出

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
    PROTOCOL_SERVO = 0,  // 舵机协议 (0x55 0x55)
    PROTOCOL_DART = 1    // DART协议 (DART)
} PROTOCOL_TYPE_e;
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

## 核心API函数

### 1. BSP_UART_Init()
**功能**: 初始化BSP UART模块
```c
void BSP_UART_Init(void);
```

**使用示例**:
```c
BSP_UART_Init();  // 初始化所有UART的缓冲区
```

**注意事项**:
- 必须在使用其他UART功能前调用
- 自动初始化所有配置的UART端口

---

### 2. UART_SetProtocol()
**功能**: 设置协议类型
```c
void UART_SetProtocol(BSP_UART_NUM_e uart_num, bool is_servo_mode);
```

**参数说明**:
- `uart_num`: UART编号
- `is_servo_mode`: true=舵机协议, false=DART协议

**使用示例**:
```c
UART_SetProtocol(BSP_UART6, true);   // UART6使用舵机协议
UART_SetProtocol(BSP_UART3, false);  // UART3使用DART协议
```

---

### 3. UART_StartRx()
**功能**: 启动接收(开启中断接收)
```c
void UART_StartRx(BSP_UART_NUM_e uart_num);
```

**使用示例**:
```c
UART_StartRx(BSP_UART6);  // 启动UART6接收
```

---

### 4. UART_Send()
**功能**: 发送数据
```c
uint16_t UART_Send(BSP_UART_NUM_e uart_num, const uint8_t *data, uint16_t len);
```

**参数说明**:
- `uart_num`: UART编号
- `data`: 数据指针
- `len`: 数据长度

**返回值**: 实际写入缓冲区的字节数

**使用示例**:
```c
uint8_t data[] = {0x55, 0x55, 0x01, 0x04, 0x01, 0xFA};
uint16_t sent = UART_Send(BSP_UART6, data, sizeof(data));
```

---

### 5. UART_SendString()
**功能**: 发送字符串
```c
uint16_t UART_SendString(BSP_UART_NUM_e uart_num, const char *str);
```

**使用示例**:
```c
UART_SendString(BSP_UART3, "Hello World!\r\n");
```

---

### 6. UART_Read()
**功能**: 读取接收数据
```c
uint16_t UART_Read(BSP_UART_NUM_e uart_num, uint8_t *data, uint16_t len);
```

**参数说明**:
- `uart_num`: UART编号
- `data`: 接收缓冲区指针
- `len`: 要读取的长度

**返回值**: 实际读取的字节数

**使用示例**:
```c
uint8_t buffer[32];
uint16_t received = UART_Read(BSP_UART6, buffer, sizeof(buffer));
```

---

### 7. UART_HasData()
**功能**: 检查接收缓冲区是否有数据
```c
bool UART_HasData(BSP_UART_NUM_e uart_num);
```

**返回值**: true=有数据, false=无数据

**使用示例**:
```c
if(UART_HasData(BSP_UART6)) {
    // 读取数据
    UART_Read(BSP_UART6, buffer, len);
}
```

---

### 8. UART_GetRxCount()
**功能**: 获取接收缓冲区数据量
```c
uint16_t UART_GetRxCount(BSP_UART_NUM_e uart_num);
```

**返回值**: 缓冲区中的数据字节数

---

### 9. UART_ClearRx()
**功能**: 清空接收缓冲区
```c
void UART_ClearRx(BSP_UART_NUM_e uart_num);
```

**使用示例**:
```c
UART_ClearRx(BSP_UART6);  // 清空UART6接收缓冲区
```

---

## 协议解析API

### 1. UART_HasPacket()
**功能**: 检查是否有完整的数据包
```c
bool UART_HasPacket(BSP_UART_NUM_e uart_num);
```

**返回值**: true=有完整数据包, false=无

**使用示例**:
```c
if(UART_HasPacket(BSP_UART6)) {
    // 读取数据包
}
```

---

### 2. UART_GetServoPacket()
**功能**: 获取舵机协议数据包
```c
bool UART_GetServoPacket(BSP_UART_NUM_e uart_num, ServoPacket_t *packet);
```

**参数说明**:
- `uart_num`: UART编号
- `packet`: 数据包结构体指针

**返回值**: true=成功获取, false=失败

**使用示例**:
```c
ServoPacket_t servo_pkt;
if(UART_GetServoPacket(BSP_UART6, &servo_pkt)) {
    if(servo_pkt.is_valid) {
        printf("ID: %d, Cmd: %d\n", servo_pkt.id, servo_pkt.cmd);
    }
}
```

---

### 3. UART_GetDartPacket()
**功能**: 获取DART协议数据包
```c
bool UART_GetDartPacket(BSP_UART_NUM_e uart_num, DartPacket_t *packet);
```

**使用示例**:
```c
DartPacket_t dart_pkt;
if(UART_GetDartPacket(BSP_UART3, &dart_pkt)) {
    if(dart_pkt.is_valid) {
        // 处理数据
    }
}
```

---

### 4. UART_ClearPacket()
**功能**: 清除数据包标志
```c
void UART_ClearPacket(BSP_UART_NUM_e uart_num);
```

**使用示例**:
```c
UART_ClearPacket(BSP_UART6);
```

---

## 工具函数

### UART_Calculate_CRC()
**功能**: CRC校验生成(公开函数，供其他模块使用)
```c
uint8_t UART_Calculate_CRC(uint8_t *data, uint8_t length);
```

**参数说明**:
- `data`: 数据指针
- `length`: 数据长度

**返回值**: CRC校验码

**算法**: `CRC = ~(SUM) & 0xFF` (取反后的累加和)

**使用示例**:
```c
uint8_t data[] = {0x01, 0x04, 0x01};
uint8_t crc = UART_Calculate_CRC(data, 3);
```

---

## 协议格式说明

### 舵机协议 (0x55 0x55)
```
帧头 | ID | Length | Cmd | Params | CRC
2字节 1字节  1字节   1字节 N字节    1字节
```

**示例**:
```
0x55 0x55 0x01 0x04 0x01 0xFA
帧头      ID   长度  指令  CRC
```

**Length说明**: `Length = 参数长度 + 3`

---

### DART协议
```
帧头   | Length | Data    | CRC
4字节   1字节    N字节    1字节
"DART"
```

**示例**:
```
'D' 'A' 'R' 'T' 0x05 [5字节数据] CRC
```

---

## 环形缓冲区原理

### 特性
1. **循环写入**: 写指针到达末尾后自动回到开头
2. **覆盖保护**: 缓冲区满时自动覆盖旧数据
3. **帧保护**: 被覆盖的不完整帧会自动标记为无效

### 状态判断
```c
// 空状态
WriteIndex == ReadIndex

// 满状态  
(WriteIndex + 1) % SIZE == ReadIndex

// 可用数据长度
(WriteIndex - ReadIndex + SIZE) % SIZE
```

---

## 典型应用场景

### 场景1: 舵机通信
```c
// 1. 初始化
BSP_UART_Init();
UART_SetProtocol(BSP_UART6, true);  // 舵机协议
UART_StartRx(BSP_UART6);

// 2. 发送舵机指令
uint8_t cmd[] = {0x55, 0x55, 0x01, 0x07, 0x01, 0x00, 0x00, 0x03, 0xE8, 0xFA};
UART_Send(BSP_UART6, cmd, sizeof(cmd));

// 3. 接收舵机反馈
ServoPacket_t pkt;
if(UART_HasPacket(BSP_UART6)) {
    UART_GetServoPacket(BSP_UART6, &pkt);
    if(pkt.is_valid) {
        // 处理反馈数据
    }
}
```

### 场景2: DART上位机通信
```c
// 1. 初始化
BSP_UART_Init();
UART_SetProtocol(BSP_UART3, false);  // DART协议
UART_StartRx(BSP_UART3);

// 2. 发送DART数据
uint8_t data[] = {'D', 'A', 'R', 'T', 0x05, 'H', 'e', 'l', 'l', 'o', 0xCRC};
UART_Send(BSP_UART3, data, sizeof(data));

// 3. 接收DART数据
DartPacket_t pkt;
if(UART_HasPacket(BSP_UART3)) {
    UART_GetDartPacket(BSP_UART3, &pkt);
    if(pkt.is_valid) {
        // 处理数据
    }
}
```

### 场景3: 原始数据收发
```c
// 发送
uint8_t tx_data[] = {0x01, 0x02, 0x03};
UART_Send(BSP_UART3, tx_data, 3);

// 接收
uint8_t rx_data[32];
if(UART_HasData(BSP_UART3)) {
    uint16_t len = UART_Read(BSP_UART3, rx_data, sizeof(rx_data));
    // 处理接收到的数据
}
```

---

## 使用注意事项

1. **初始化顺序**
   ```c
   BSP_UART_Init();              // 必须先初始化
   UART_SetProtocol(uart, mode); // 设置协议类型
   UART_StartRx(uart);           // 启动接收
   ```

2. **协议设置**
   - 每个UART只能同时使用一种协议
   - 切换协议前建议清空缓冲区

3. **CRC校验**
   - 舵机协议: CRC从ID开始到参数结束
   - DART协议: CRC对数据部分计算
   - 算法: `~(累加和) & 0xFF`

4. **缓冲区溢出**
   - 缓冲区满时会自动覆盖旧数据
   - 被覆盖的不完整帧会自动失效
   - 建议定期读取数据避免溢出

5. **中断安全**
   - 发送和接收都使用中断方式
   - 回调函数在中断中执行，避免耗时操作

6. **帧识别**
   - 自动识别帧头并记录位置
   - 支持跨缓冲区边界的帧头识别

---

## 性能参数

| 参数 | 值 | 说明 |
|-----|---|------|
| 发送缓冲区 | 64字节 | 可调整 |
| 接收缓冲区 | 64字节 | 可调整 |
| DART帧索引 | 16帧 | 最多同时缓存16个DART帧 |
| 舵机帧索引 | 32帧 | 最多同时缓存32个舵机帧 |
| 波特率 | 115200 | 舵机标准波特率 |

---

## 相关模块

- **HX06L**: 使用本模块进行舵机通信
- **上位机通信**: 使用DART协议与上位机交互
- 所有需要串口通信的模块

---

## 调试建议

1. **检查数据发送**
   ```c
   uint16_t sent = UART_Send(uart, data, len);
   if(sent != len) {
       // 发送缓冲区满
   }
   ```

2. **检查数据接收**
   ```c
   if(UART_HasPacket(uart)) {
       // 有完整数据包
   }
   ```

3. **CRC验证**
   ```c
   uint8_t calc_crc = UART_Calculate_CRC(data, len);
   if(calc_crc != received_crc) {
       // CRC错误
   }
   ```

---

## 常见问题

**Q: 为什么收不到数据？**
A: 检查是否调用了`UART_StartRx()`启动接收

**Q: CRC校验总是失败？**
A: 检查CRC计算范围，舵机协议从ID开始，不包括帧头

**Q: 缓冲区满了怎么办？**
A: 增大缓冲区大小或更频繁地读取数据

**Q: 如何切换协议？**
A: 先清空缓冲区，再调用`UART_SetProtocol()`

---

## 更新日志

- 支持环形缓冲区自动管理
- 实现舵机和DART双协议解析
- 自动帧头识别和CRC校验
- 溢出保护和帧完整性验证
