# HX06L 总线舵机使用说明文档

## 文件信息

- **头文件**: `User/inc/HX06L.h`
- **源文件**: `User/src/HX06L.c`
- **作用**: 换弹结构总线舵机控制，支持双协议切换

---

## 功能概述

本模块封装了 HX06L 总线舵机的控制功能，支持两种通信协议，通过宏定义切换。

### 核心功能

1. **双协议支持** - 支持无 MCU 驱动板协议和有 MCU 控制板协议
2. **单舵机控制** - 控制单个舵机转动到指定角度
3. **多舵机控制** - 同时控制多个舵机（控制板协议）
4. **动作组控制** - 运行/停止预设动作组（控制板协议）
5. **舵机卸力** - 控制舵机马达掉电卸力

---

## 协议选择

通过 `HX06L.h` 中的宏定义选择协议：

```c
// 协议选择宏
#define other_mcu_forcing  1  // 0=无MCU驱动板协议, 1=有MCU控制板协议
```

### 协议对比

| 特性         | 无 MCU 驱动板协议 (0) | 有 MCU 控制板协议 (1) |
| ------------ | --------------------- | --------------------- | ------ | --- | ----- | ---- | ---------- | ------ | --- | ------ |
| 波特率       | 115200                | 9600                  |
| 帧格式       | `0x55 0x55            | ID                    | Length | Cmd | Param | CRC` | `0x55 0x55 | Length | Cmd | Param` |
| CRC 校验     | 有                    | 无                    |
| 舵机 ID 位置 | 在帧头后              | 在参数中              |
| 多舵机控制   | 需逐个发送            | 单帧控制多个          |
| 动作组功能   | 不支持                | 支持                  |

---

## 数据结构（有 MCU 控制板协议）

### ControlBoardCmd 控制板指令枚举

```c
typedef enum {
    CMD_SERVO_MOVE = 0x03,           // 控制任意个舵机的转动
    CMD_ACTION_GROUP_RUN = 0x06,     // 控制动作组运行
    CMD_ACTION_GROUP_STOP = 0x07,    // 停止正在运行的动作组
    CMD_ACTION_GROUP_COMPLETE = 0x08,// 动作组自然运行结束返回
    CMD_ACTION_GROUP_SPEED = 0x0B,   // 控制动作组的速度
    CMD_GET_BATTERY_VOLTAGE = 0x0F,  // 获取控制板电池电压
    CMD_MULT_SERVO_UNLOAD = 0x14,    // 控制多个舵机马达掉电卸力
    CMD_MULT_SERVO_POS_READ = 0x15   // 读取多个舵机的角度位置值
} ControlBoardCmd;
```

---

## 通用 API 函数

以下函数在两种协议下均可使用：

### 1. ServoInit()

**功能**: 舵机初始化

```c
void ServoInit(void);
```

**使用示例**:

```c
ServoInit();  // 初始化舵机
```

**注意事项**:

- 无 MCU 协议：初始化 3 个舵机(ID: 1, 2, 3)并上电
- 有 MCU 协议：发送读取电压命令测试通信

---

### 2. ServoControlPos()

**功能**: 控制单个舵机转动到指定角度

```c
void ServoControlPos(uint8_t ID, uint16_t Angle, uint16_t Time);
```

**参数说明**:

- `ID`: 舵机 ID 号 (0-253)
- `Angle`: 目标角度 (0-1000，对应 0°-240°)
- `Time`: 转动时间 (0-30000ms)

**使用示例**:

```c
// 控制1号舵机在1000ms内转到中间位置(500)
ServoControlPos(1, 500, 1000);

// 控制2号舵机在500ms内转到最大角度(1000)
ServoControlPos(2, 1000, 500);
```

**角度换算**:

```
角度值 = 实际角度(°) × 1000 / 240
实际角度(°) = 角度值 × 240 / 1000
```

| 角度值 | 实际角度 |
| ------ | -------- |
| 0      | 0°       |
| 500    | 120°     |
| 1000   | 240°     |

---

## 有 MCU 控制板协议专用函数

以下函数仅在 `other_mcu_forcing = 1` 时可用：

### 3. ServoControlMulti()

**功能**: 同时控制多个舵机转动

```c
void ServoControlMulti(uint8_t servo_num, uint8_t *servo_ids, uint16_t *angles, uint16_t time);
```

**参数说明**:

- `servo_num`: 舵机个数
- `servo_ids`: 舵机 ID 数组指针
- `angles`: 角度数组指针 (0-1000)
- `time`: 转动时间 (ms)

**使用示例**:

```c
uint8_t ids[] = {1, 2, 3};
uint16_t angles[] = {500, 800, 300};

// 控制3个舵机在1000ms内同时转到指定位置
ServoControlMulti(3, ids, angles, 1000);
```

---

### 4. ServoRunActionGroup()

**功能**: 运行预设的动作组

```c
void ServoRunActionGroup(uint8_t group_num, uint16_t run_times);
```

**参数说明**:

- `group_num`: 动作组编号
- `run_times`: 运行次数 (0 表示无限次循环)

**使用示例**:

```c
// 运行8号动作组1次
ServoRunActionGroup(8, 1);

// 运行2号动作组无限次
ServoRunActionGroup(2, 0);
```

---

### 5. ServoStopActionGroup()

**功能**: 停止当前正在运行的动作组

```c
void ServoStopActionGroup(void);
```

**使用示例**:

```c
ServoStopActionGroup();  // 停止动作组
```

---

### 6. ServoSetActionGroupSpeed()

**功能**: 设置动作组运行速度

```c
void ServoSetActionGroupSpeed(uint8_t group_num, uint16_t speed_percent);
```

**参数说明**:

- `group_num`: 动作组编号 (0xFF 表示所有动作组)
- `speed_percent`: 速度百分比 (100=原速，200=2 倍速，50=0.5 倍速)

**使用示例**:

```c
// 设置8号动作组以50%速度运行
ServoSetActionGroupSpeed(8, 50);

// 设置所有动作组以3倍速运行
ServoSetActionGroupSpeed(0xFF, 300);
```

**注意事项**:

- 速度设置关机不保存，每次开机需重新设置
- 超过舵机极限速度的设置无效

---

### 7. ServoUnloadMulti()

**功能**: 控制多个舵机卸力（马达掉电）

```c
void ServoUnloadMulti(uint8_t servo_num, uint8_t *servo_ids);
```

**参数说明**:

- `servo_num`: 舵机个数
- `servo_ids`: 舵机 ID 数组指针

**使用示例**:

```c
uint8_t ids[] = {1, 2, 3};

// 控制1,2,3号舵机卸力
ServoUnloadMulti(3, ids);
```

**注意事项**:

- 卸力后舵机可以用手自由转动
- 重新发送位置指令后舵机会重新上电

---

### 8. ServoGetBatteryVoltage()

**功能**: 获取控制板电池电压

```c
void ServoGetBatteryVoltage(void);
```

**使用示例**:

```c
ServoGetBatteryVoltage();
// 电压值通过接收回调获取，单位为毫伏(mV)
```

---

## 通信协议详解

### 有 MCU 控制板协议帧格式

```
帧头   | 数据长度 | 指令 | 参数1 | ... | 参数N
0x55 0x55  Length   Cmd   Prm1  ...   PrmN
```

**数据长度计算**: `Length = 参数个数N + 2`

#### 控制舵机转动 (CMD_SERVO_MOVE = 0x03)

```
0x55 0x55 | Length | 0x03 | 舵机个数 | 时间L | 时间H | [ID | 角度L | 角度H] × N
```

**示例** - 控制 1 号舵机在 1000ms 内转到 800 位置：

```
0x55 0x55 0x08 0x03 0x01 0xE8 0x03 0x01 0x20 0x03
```

**示例** - 控制 2 号和 9 号舵机在 800ms 内都转到 800 位置：

```
0x55 0x55 0x0B 0x03 0x02 0x20 0x03 0x02 0x20 0x03 0x09 0x20 0x03
```

---

### 无 MCU 驱动板协议帧格式

```
帧头   | ID | 数据长度 | 指令 | 参数 | 校验和
0x55 0x55  ID  Length   Cmd   Param  CRC
```

**数据长度计算**: `Length = 参数长度 + 3`

**CRC 校验算法**: `CRC = ~(SUM(ID到Param)) & 0xFF`

---

## 典型应用场景

### 场景 1: 换弹结构控制（有 MCU 控制板）

```c
// 初始化（BSP_UART_Init已在main中调用，UART6默认SERVO协议）
ServoInit();

// 换弹动作序列
void ReloadAction(void) {
    // 1. 打开弹仓
    ServoControlPos(1, 800, 500);
    vTaskDelay(600);

    // 2. 推弹
    ServoControlPos(2, 500, 300);
    vTaskDelay(400);

    // 3. 关闭弹仓
    ServoControlPos(1, 200, 500);
    vTaskDelay(600);
}

// 或使用预设动作组
void ReloadActionGroup(void) {
    ServoRunActionGroup(1, 1);  // 运行1号动作组1次
}
```

### 场景 2: 同步控制多舵机

```c
uint8_t servo_ids[] = {1, 2, 3};
uint16_t angles[] = {500, 500, 500};

// 所有舵机同步移动到中间位置
ServoControlMulti(3, servo_ids, angles, 1000);
```

### 场景 3: 舵机调试

```c
// 卸力后手动调整位置
uint8_t ids[] = {1, 2, 3};
ServoUnloadMulti(3, ids);

// 调整完成后恢复控制
ServoControlPos(1, 500, 1000);
```

---

## 使用注意事项

1. **初始化顺序**

   - 先调用 `BSP_UART_Init()` 初始化串口（自动启动接收）
   - UART6 默认配置为 SERVO 协议，无需额外设置
   - 再调用 `ServoInit()` 初始化舵机

2. **协议选择**

   - 修改宏 `other_mcu_forcing` 后需重新编译
   - 确保硬件波特率与协议匹配

3. **UART 配置**

   - 无 MCU 协议: 115200 波特率
   - 有 MCU 协议: 9600 波特率
   - 使用 BSP_UART6 进行通信（默认 SERVO 协议）

4. **角度范围**

   - 有效范围: 0-1000 (对应 0°-240°)
   - 超出范围可能导致舵机异常

4. **时间参数**

   - 有效范围: 0-30000ms
   - 时间为 0 时舵机以最快速度转动

5. **延时处理**

   - 发送命令后建议等待 1-2ms
   - 使用 `vTaskDelay()` 而非 `HAL_Delay()`

6. **动作组（仅控制板协议）**
   - 动作组需预先通过上位机下载到控制板
   - 速度设置关机不保存

---

## 硬件连接

| 信号 | MCU 引脚 | 说明                |
| ---- | -------- | ------------------- |
| TX   | UART6_TX | 发送到舵机/控制板   |
| RX   | UART6_RX | 接收舵机/控制板反馈 |
| GND  | GND      | 共地                |

---

## 常见问题

**Q: 舵机没有反应？**
A:

1. 检查波特率是否正确配置
2. 检查 TX/RX 连接是否正确
3. 确认舵机 ID 是否正确
4. 无 MCU 协议需先调用 `ServoInit()` 上电

**Q: 多个舵机如何同时运动？**
A:

- 控制板协议: 使用 `ServoControlMulti()` 函数
- 驱动板协议: 需要快速连续发送多个命令

**Q: 如何切换协议？**
A: 修改 `HX06L.h` 中的 `other_mcu_forcing` 宏并重新编译

**Q: 动作组如何创建？**
A: 使用控制板配套的上位机软件创建并下载到控制板

---

## 相关模块

- **BSP_UART**: 串口通信底层支持
- **FreeRTOS**: 任务延时功能

---

## 更新日志

| 日期       | 更新内容                       |
| ---------- | ------------------------------ |
| 2025/11/24 | 创建基础版本                   |
| 2025/12/05 | 添加有 MCU 控制板协议支持      |
| 2025/12/05 | 添加双协议切换宏               |
| 2025/12/06 | 移除发送函数中不必要的协议设置 |
| 2025/12/08 | 更新初始化说明（BSP_UART_Init 自动启动接收和设置协议） |
