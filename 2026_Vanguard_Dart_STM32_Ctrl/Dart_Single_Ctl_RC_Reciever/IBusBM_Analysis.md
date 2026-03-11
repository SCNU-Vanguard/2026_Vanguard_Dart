# IBusBM 库代码分析文档

## 1. 项目概述

IBusBM 是一个用于处理 Flysky/Turnigy RC iBUS 协议的 Arduino 库，主要用于与 **IA6B** 等接收机通信。

### 1.1 支持的功能
- **接收伺服通道数据**：从接收机读取最多14个通道的控制信号
- **传感器/遥测数据发送**：将传感器数据（电压、温度、转速等）回传至发射机
- **多平台支持**：AVR、ESP32、STM32、MBED、MegaAVR

### 1.2 硬件接口
- TGY-IA6B 接收机有两个 iBUS 引脚：
  - **伺服输出引脚**：仅输出伺服控制数据
  - **传感器引脚**：使用半双工协议进行双向通信

---

## 2. iBUS 协议详解

### 2.1 协议特性
- **波特率**：115200
- **数据格式**：8N1 (8位数据，无校验，1位停止位)
- **数据包发送频率**：约每 7ms 一次
- **半双工**：传感器通道使用单线半双工通信

### 2.2 数据包结构

#### 伺服命令包 (0x40)
```
长度: 32 字节 (0x20)
格式: <len><cmd><ch0_L><ch0_H><ch1_L><ch1_H>...<ch13_L><ch13_H><chkL><chkH>
```

**示例数据包**：
```
20 40 DB 05 DC 05 54 05 DC 05 E8 03 D0 07 D2 05 E8 03 DC 05 DC 05 DC 05 DC 05 DC 05 DC 05 DA F3
```

**解析**：
| 字节 | 值 | 说明 |
|------|------|------|
| 0x20 | 32 | 协议长度 |
| 0x40 | 64 | 命令码（伺服命令） |
| 0x05DB | 1499 | 通道0值 |
| 0x05DC | 1500 | 通道1值 |
| ... | ... | ... |
| 0xF3DA | 校验和 | 所有前面字节相加后取反 |

#### 传感器协议命令
| 命令 | 值 | 说明 |
|------|------|------|
| DISCOVER | 0x80 + addr | 发现传感器 |
| TYPE | 0x90 + addr | 查询传感器类型 |
| VALUE | 0xA0 + addr | 请求传感器数据 |

### 2.3 校验和计算
```
checksum = 0xFFFF - (所有前面字节之和)
```

### 2.4 通道值范围
- **最小值**：1000 (0x03E8) - 油门最低/摇杆最左
- **中立值**：1500 (0x05DC)
- **最大值**：2000 (0x07D0) - 油门最高/摇杆最右

---

## 3. 核心数据结构

### 3.1 IBusBM 类定义 (`src/IBusBM.h`)

```cpp
class IBusBM {
public:
  // STM32专用begin函数
  void begin(HardwareSerial &serial, TIM_TypeDef * timerid=TIM1, 
             int8_t rxPin=-1, int8_t txPin=-1);
  
  // 通用begin函数
  void begin(HardwareSerial &serial, int8_t timerid=0, 
             int8_t rxPin=-1, int8_t txPin=-1);
  
  uint16_t readChannel(uint8_t channelNr);      // 读取伺服通道 0-13
  uint8_t addSensor(uint8_t type, uint8_t len=2); // 添加传感器
  void setSensorMeasurement(uint8_t adr, int32_t value); // 设置传感器值
  void loop(void);  // 主循环处理函数

  // 统计计数器
  volatile uint8_t cnt_poll;   // 传感器轮询消息计数
  volatile uint8_t cnt_sensor; // 传感器数据发送计数
  volatile uint8_t cnt_rec;    // 伺服消息接收计数

private:
  // 状态机状态
  enum State {GET_LENGTH, GET_DATA, GET_CHKSUML, GET_CHKSUMH, DISCARD};
  
  // 协议常量
  static const uint8_t PROTOCOL_LENGTH = 0x20;      // 数据包最大长度
  static const uint8_t PROTOCOL_OVERHEAD = 3;       // 开销字节数
  static const uint8_t PROTOCOL_TIMEGAP = 3;        // 数据包间隔时间(ms)
  static const uint8_t PROTOCOL_CHANNELS = 14;      // 最大通道数
  static const uint8_t SENSORMAX = 10;              // 最大传感器数
  
  // 协议命令
  static const uint8_t PROTOCOL_COMMAND40 = 0x40;         // 伺服命令
  static const uint8_t PROTOCOL_COMMAND_DISCOVER = 0x80;  // 发现传感器
  static const uint8_t PROTOCOL_COMMAND_TYPE = 0x90;      // 查询传感器类型
  static const uint8_t PROTOCOL_COMMAND_VALUE = 0xA0;     // 请求传感器数据
  
  // 成员变量
  uint8_t state;                       // 状态机当前状态
  HardwareSerial *stream;              // 串口指针
  uint32_t last;                       // 上次消息时间戳
  uint8_t buffer[PROTOCOL_LENGTH];     // 消息缓冲区
  uint8_t ptr;                         // 缓冲区指针
  uint8_t len;                         // 当前消息长度
  uint16_t channel[PROTOCOL_CHANNELS]; // 伺服通道数据
  uint16_t chksum;                     // 校验和
  
  // 传感器信息结构
  typedef struct {
    uint8_t sensorType;      // 传感器类型
    uint8_t sensorLength;    // 数据长度 (2或4字节)
    int32_t sensorValue;     // 传感器值
  } sensorinfo;
  sensorinfo sensors[SENSORMAX];
};
```

### 3.2 传感器类型定义

```cpp
#define IBUSS_INTV 0x00  // 内部电压 (单位: 0.01V)
#define IBUSS_TEMP 0x01  // 温度 (单位: 0.1°C, 0=-40°C)
#define IBUSS_RPM  0x02  // 转速 (RPM)
#define IBUSS_EXTV 0x03  // 外部电压 (单位: 0.01V)
#define IBUS_PRESS 0x41  // 压力 (Pa)
#define IBUS_SERVO 0xfd  // 伺服值
```

---

## 4. 状态机与数据接收流程

### 4.1 状态机工作流程

```
      ┌─────────────┐
      │   DISCARD   │◄────────────────────────────┐
      └──────┬──────┘                              │
             │ (超时 >= 3ms)                        │
             ▼                                      │
      ┌─────────────┐                              │
      │ GET_LENGTH  │                              │
      └──────┬──────┘                              │
             │ (len有效: 4-32)                      │
             ▼                                      │
      ┌─────────────┐                              │
      │  GET_DATA   │                              │
      └──────┬──────┘                              │
             │ (数据接收完成)                       │
             ▼                                      │
      ┌─────────────┐                              │
      │ GET_CHKSUML │                              │
      └──────┬──────┘                              │
             │                                      │
             ▼                                      │
      ┌─────────────┐      校验成功                │
      │ GET_CHKSUMH │──────────────► 处理命令 ─────┤
      └─────────────┘      校验失败                │
                           ─────────────────────────┘
```

### 4.2 loop() 函数核心逻辑

```cpp
void IBusBM::loop(void) {
  // 递归调用链表中的下一个实例
  if (IBusBMnext) IBusBMnext->loop(); 

  while (stream->available() > 0) {
    // 检测时间间隔，超过3ms认为是新数据包开始
    uint32_t now = millis();
    if (now - last >= PROTOCOL_TIMEGAP) {
      state = GET_LENGTH;
    }
    last = now;
    
    uint8_t v = stream->read();
    
    switch (state) {
      case GET_LENGTH:
        // 验证长度字段有效性 (4-32字节)
        if (v <= PROTOCOL_LENGTH && v > PROTOCOL_OVERHEAD) {
          ptr = 0;
          len = v - PROTOCOL_OVERHEAD;  // 实际数据长度
          chksum = 0xFFFF - v;          // 初始化校验和
          state = GET_DATA;
        } else {
          state = DISCARD;
        }
        break;

      case GET_DATA:
        buffer[ptr++] = v;
        chksum -= v;
        if (ptr == len) {
          state = GET_CHKSUML;
        }
        break;
        
      case GET_CHKSUML:
        lchksum = v;
        state = GET_CHKSUMH;
        break;

      case GET_CHKSUMH:
        // 校验和验证
        if (chksum == (v << 8) + lchksum) {
          // 处理有效命令
          processCommand();
        }
        state = DISCARD;
        break;
    }
  }
}
```

### 4.3 命令处理

#### 伺服命令 (0x40)
```cpp
if (buffer[0] == PROTOCOL_COMMAND40) {
  // 提取14个通道的数据 (小端序)
  for (uint8_t i = 1; i < PROTOCOL_CHANNELS * 2 + 1; i += 2) {
    channel[i / 2] = buffer[i] | (buffer[i + 1] << 8);
  }
  cnt_rec++;  // 增加接收计数
}
```

#### 传感器命令处理
```cpp
// DISCOVER命令 (0x80): 发现传感器
// 响应: 0x04, 0x81+addr, checksum_L, checksum_H

// TYPE命令 (0x90): 查询传感器类型
// 响应: 0x06, 0x91+addr, type, length, checksum_L, checksum_H

// VALUE命令 (0xA0): 请求传感器数据
// 响应: 0x04+len, 0xA1+addr, value_bytes..., checksum_L, checksum_H
```

---

## 5. STM32 特定实现

### 5.1 平台检测与定时器配置

STM32 使用 HardwareTimer 库实现定时中断：

```cpp
#if defined(_VARIANT_ARDUINO_STM32_)
  #if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION < 0x01090000)
    #error "Due to API change, this sketch is compatible with STM32_CORE_VERSION >= 0x01090000"
  #endif
  #define IBUSBM_NOTIMER NULL  // STM32使用NULL表示禁用定时器
  void begin(HardwareSerial &serial, TIM_TypeDef * timerid=TIM1, 
             int8_t rxPin=-1, int8_t txPin=-1);
#endif
```

### 5.2 STM32 定时器初始化

```cpp
#elif defined(_VARIANT_ARDUINO_STM32_)
  // 使用STM32 HardwareTimer库
  // 参考: https://github.com/stm32duino/wiki/wiki/HardwareTimer-library
  HardwareTimer *stimer_t = new HardwareTimer(timerid);
  stimer_t->setOverflow(1000, HERTZ_FORMAT);  // 1000 Hz (1ms周期)
  stimer_t->attachInterrupt(onTimer);          // 绑定中断回调
  stimer_t->resume();                          // 启动定时器
```

### 5.3 STM32 使用示例 (Ibus_diy_servo_STM32)

```cpp
// 关键配置
#include <IBusBM.h>
#include <PID_v1.h>

IBusBM IBUS;

void setup() {
  // 在Serial1上启动IBus，禁用内部定时器
  // STM32F103C8T6 (Blue Pill): PA10 = RX1
  IBUS.begin(Serial1, IBUSBM_NOTIMER);
  
  // 等待接收机连接
  while (IBUS.cnt_rec == 0) {
    IBUS.loop();
    delay(1);
  }
}

void loop() {
  // 创建1ms循环
  if (CurrentMillis != millis()) {
    IBUS.loop();  // 手动调用loop处理IBus数据
    MillisCount++;
    CurrentMillis = millis();
    
    if (MillisCount > 7) {  // 每8ms处理一次 (125Hz)
      // 读取通道数据
      int STEER = IBUS.readChannel(STEER_chan - 1);
      
      // 检测连接状态
      if (IBUS.cnt_rec == IbusCount) {
        // 无新数据 - 失联保护
        disableMotor();
      } else {
        // 正常处理伺服控制
        processServo();
      }
      
      IbusCount = IBUS.cnt_rec;
      MillisCount = 0;
    }
  }
}
```

### 5.4 STM32 引脚配置

| 型号 | 串口 | RX引脚 | TX引脚 | 说明 |
|------|------|--------|--------|------|
| STM32F103C8T6 | Serial1 | PA10 | PA9 | Blue Pill |
| STM32F103C8T6 | Serial2 | PA3 | PA2 | Blue Pill |
| STM32F103C8T6 | Serial3 | PB11 | PB10 | Blue Pill |

### 5.5 STM32 注意事项

1. **核心版本要求**：需要 `STM32_CORE_VERSION >= 0x01090000`
2. **默认定时器**：TIM1，如果TIM1被其他功能占用（如Servo或SoftwareSerial），需要更换
3. **NOTIMER模式**：使用 `IBUSBM_NOTIMER` (NULL) 禁用内部定时器，需手动调用 `loop()`
4. **串口波特率**：固定115200，无需手动配置

---

## 6. 接收机数据处理流程图

```
┌──────────────────────────────────────────────────────────────────┐
│                      IA6B 接收机                                  │
│  ┌────────────────┐    ┌────────────────┐                        │
│  │  伺服输出引脚   │    │  传感器引脚    │                        │
│  │ (单向输出)      │    │ (半双工)       │                        │
│  └───────┬────────┘    └───────┬────────┘                        │
└──────────┼─────────────────────┼────────────────────────────────┘
           │                     │
           │ (RX)                │ (RX + TX)
           │                     │ (需要二极管或1.2k电阻)
           ▼                     ▼
┌──────────────────────────────────────────────────────────────────┐
│                       STM32 / Arduino                             │
│                                                                   │
│   ┌─────────────────────────────────────────────────────────┐    │
│   │                    IBusBM 库                             │    │
│   │                                                          │    │
│   │  ┌──────────┐    ┌──────────────┐    ┌──────────────┐   │    │
│   │  │ 定时器   │───►│  loop()函数  │───►│ 状态机解析   │   │    │
│   │  │ (1ms)    │    │              │    │              │   │    │
│   │  └──────────┘    └──────────────┘    └──────┬───────┘   │    │
│   │                                              │           │    │
│   │                         ┌────────────────────┼───────┐   │    │
│   │                         │                    │       │   │    │
│   │                         ▼                    ▼       │   │    │
│   │               ┌──────────────┐    ┌──────────────┐   │   │    │
│   │               │ channel[14]  │    │ 传感器响应   │   │   │    │
│   │               │ 伺服通道数据 │    │              │   │   │    │
│   │               └──────┬───────┘    └──────────────┘   │   │    │
│   │                      │                               │   │    │
│   └──────────────────────┼───────────────────────────────┘   │    │
│                          │                                    │    │
│                          ▼                                    │    │
│   ┌──────────────────────────────────────────────────────────┐    │
│   │                    用户应用代码                           │    │
│   │                                                          │    │
│   │  readChannel(n)  ───► 获取通道值 (1000-2000)             │    │
│   │  addSensor()     ───► 添加传感器                         │    │
│   │  setSensorMeasurement() ───► 设置传感器数据              │    │
│   │                                                          │    │
│   └──────────────────────────────────────────────────────────┘    │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
```

---

## 7. API 接口总结

### 7.1 初始化函数

```cpp
// STM32版本
void begin(HardwareSerial &serial, 
           TIM_TypeDef * timerid = TIM1,    // 定时器，NULL禁用
           int8_t rxPin = -1,                // 默认使用串口默认引脚
           int8_t txPin = -1);

// 其他平台版本  
void begin(HardwareSerial &serial,
           int8_t timerid = 0,              // 定时器ID，-1禁用
           int8_t rxPin = -1,
           int8_t txPin = -1);
```

### 7.2 通道读取

```cpp
uint16_t readChannel(uint8_t channelNr);
// 参数: channelNr = 0-13 (对应遥控器通道1-14)
// 返回: 1000-2000 (对应PWM微秒值)
```

### 7.3 传感器操作

```cpp
uint8_t addSensor(uint8_t type, uint8_t len = 2);
// 参数: type = 传感器类型, len = 数据长度(2或4字节)
// 返回: 传感器地址 (1开始)

void setSensorMeasurement(uint8_t adr, int32_t value);
// 参数: adr = 传感器地址, value = 传感器值
```

### 7.4 状态计数器

```cpp
volatile uint8_t cnt_rec;    // 伺服消息接收计数 (用于连接检测)
volatile uint8_t cnt_poll;   // 传感器轮询计数
volatile uint8_t cnt_sensor; // 传感器响应计数
```

---

## 8. 故障安全 (Failsafe)

### 8.1 连接检测方法

```cpp
static uint8_t lastCnt = 0;

void loop() {
  IBUS.loop();
  
  // 检查cnt_rec是否更新
  if (IBUS.cnt_rec == lastCnt) {
    // 超过一定时间无更新 = 可能失联
    activateFailsafe();
  } else {
    // 正常接收数据
    lastCnt = IBUS.cnt_rec;
    processControl();
  }
}
```

### 8.2 注意事项

> **重要**：`cnt_rec` 不能直接用于失联检测，因为支持遥测的接收机即使与发射机断连也会继续发送伺服数据。
> 
> 建议方案：
> - 非遥测接收机 (如 FS-A8S V2)：可使用 `cnt_rec` 检测
> - 遥测接收机 (如 X6B)：在发射机端设置某个通道的失联值，通过 `readChannel()` 检测

---

## 9. 编译与使用

### 9.1 STM32 编译环境
- Arduino IDE + STM32 核心库 (版本 >= 1.9.0)
- PlatformIO + stm32duino

### 9.2 基本使用流程

```cpp
#include <IBusBM.h>

IBusBM ibus;

void setup() {
  // 初始化串口和IBus
  ibus.begin(Serial1);  // 使用Serial1，启用定时器中断
}

void loop() {
  // 读取通道值
  uint16_t ch1 = ibus.readChannel(0);  // 通道1
  uint16_t ch2 = ibus.readChannel(1);  // 通道2
  
  // 处理控制逻辑
  // ...
}
```

---

## 10. 总结

IBusBM 库通过状态机实现了对 iBUS 协议的完整解析，支持：

1. **多平台兼容**：AVR、ESP32、STM32、MBED 架构
2. **双向通信**：接收伺服数据 + 发送传感器数据
3. **中断驱动**：使用定时器中断确保实时响应
4. **灵活配置**：支持禁用定时器手动调用 loop()

对于 STM32 用户，关键点是：
- 使用 `TIM_TypeDef*` 类型指定定时器
- 默认使用 TIM1，如有冲突需更换
- 支持 IBUSBM_NOTIMER 模式手动调用
- 需要 STM32 核心库版本 >= 1.9.0
