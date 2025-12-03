# STM32 飞镖控制系统文档索引

## 文档概述

本目录包含STM32飞镖控制系统的完整技术文档,帮助后续开发者快速理解和使用代码。

---

## 文档结构

### BSP层文档(底层硬件抽象)
1. **BSP_CAN使用说明.md** - CAN总线通信驱动
2. **BSP_DWT使用说明.md** - 高精度计时和延时
3. **BSP_UART使用说明.md** - 串口通信与协议解析

### User层文档(应用层)
4. **CanMotor使用说明.md** - CAN电机统一管理
5. **DM_Motor** - 达妙电机控制(见下方快速参考)
6. **RM_Motor** - 大疆电机控制(见下方快速参考)
7. **PID** - PID控制算法(见下方快速参考)
8. **HX06L** - 总线舵机控制(见下方快速参考)
9. **UserTask** - 任务定义(见下方快速参考)

---

## 快速开始指南

### 系统初始化流程

```c
int main(void) {
    // 1. HAL库初始化
    HAL_Init();
    SystemClock_Config();
    
    // 2. 外设初始化(CubeMX生成)
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();
    
    // 3. BSP层初始化
    DWT_Init(168);        // 高精度计时器(168MHz)
    BSP_UART_Init();      // UART缓冲区
    
    // 4. 用户层初始化
    MotorInit();          // 电机系统
    
    // 5. 启动RTOS
    osKernelStart();
}
```

---

## 核心模块快速参考

### DM_Motor (达妙电机)

**文件位置**: `User/inc/DM_Motor.h`, `User/src/DM_Motor.c`

**主要功能**:
- 支持MIT模式控制
- 位置、速度、力矩控制
- **通用浮点数转换函数**（参数化设计）
- **速度滤波**（一阶低通滤波）

**关键API**:
```c
// 发送控制指令
void DmMotorSendCfg(uint8_t motor_id, float TargetPos, float TargetVel);

// 使能/失能电机
uint8_t DM_MotorDisable(uint8_t DM_MOTOR_ID);  // 公开接口

// 数据解算（带速度滤波）
void DM_MOTOR_CALCU(uint8_t motor_id_num, uint8_t *ReceiveData, float *solved_data);

// 通用转换函数（内部使用）
// float_to_uint_generic(x, x_min, x_max, bits) - 带限幅
// uint_to_float_generic(x_int, x_min, x_max, bits)
```

**使用示例**:
```c
// 控制DM电机到指定位置和速度
DmMotorSendCfg(1, 90.0f, 10.0f);  // 角度90°, 速度10rad/s
```

**数据解算输出** (5个float):
- `solved_data[0]`: 位置(°) - 4310用rad，3519用°
- `solved_data[1]`: 速度(rad/s) - **带一阶滤波**
- `solved_data[2]`: 力矩(N·m)
- `solved_data[3]`: MOS温度(℃)
- `solved_data[4]`: 转子温度(℃)

**电机类型切换**:
```c
// 在DM_Motor.h中定义宏切换
#define DM_USE_4310  // 使用4310电机（MIT协议范围）
// 不定义则使用3519等电机
```

**速度滤波参数**:
```c
#define DM_SPEED_FILTER_COEF 0.8f  // 滤波系数（0~1，越大越平滑）
```

---

### RM_Motor (大疆电机)

**文件位置**: `User/inc/RM_Motor.h`, `User/src/RM_Motor.c`

**主要功能**:
- 电流控制模式
- 支持M3508、M2006等系列
- 统一发送机制
- **多圈累计与过零检测**

**关键API**:
```c
// 发送控制指令
void RmMotorSendCfg(uint8_t motor_id, int16_t TargetCurrent);

// 发送控制函数
uint8_t RM_MotorSendControl(MotorTypeDef *st);

// 数据解算 (注意: 参数已改为 uint8_t*, 输出扩展为5个float)
void RM_MOTOR_CALCU(uint8_t motor_id_num, uint8_t *ReceiveData, float *solved_data);

// 辅助函数 (新增)
void RM_Motor_Reset_Zero(uint8_t motor_id_num);  // 重置单个电机零点
void RM_Motor_Reset_All(void);                    // 重置所有电机状态
```

**使用示例**:
```c
// 控制RM电机电流
RmMotorSendCfg(RM_3508_GRIPPER, 5000);  // 电流值 ±16384

// 重置电机零点(当前位置设为0)
RM_Motor_Reset_Zero(0);  // 重置电机0
```

**数据解算输出** (5个float):
- `solved_data[0]`: 单圈角度(°) 0~360
- `solved_data[1]`: 速度(rpm)
- `solved_data[2]`: 电流(A)
- `solved_data[3]`: **累计角度(°)** - 可超过360°，用于位置闭环
- `solved_data[4]`: **速度(rad/s)** - 弧度制速度

**注意事项**:
- RM电机采用统一发送(4个电机一帧)
- 发送ID固定为0x200
- 接收ID为0x200 + MotorID
- 累计角度会自动处理编码器过零问题

---

### PID (PID控制算法)

**文件位置**: `User/inc/PID.h`, `User/src/PID.c`

**主要功能**:
- 位置式PID
- 增量式PID
- 串级PID(位置-速度双环)
- 前馈控制
- **微分先行**（可选）

**关键API**:
```c
// 初始化PID（已移除min_output参数）
void PID_Init(PID_t *pid, PID_MODE_e mode, 
              float kp, float ki, float kd, float kf, 
              float max_out, float max_iout);

// 串级PID初始化
void CASCADE_PID_Init(CASCADE_PID_t *cascade_pid,
                      float outer_kp, float outer_ki, float outer_kd, float outer_kf,
                      float inner_kp, float inner_ki, float inner_kd, float inner_kf,
                      float outer_max_out, float outer_max_iout,
                      float inner_max_out, float inner_max_iout);

// PID计算（统一接口）
float PID_Calculate(PID_t *pid, float target, float measure);

// 串级PID计算
float CASCADE_PID_Calculate(CASCADE_PID_t *cascade_pid, 
                            float outer_target, float outer_measure, 
                            float inner_measure);

// 清零函数
void PID_Clear(PID_t *pid);           // 清除所有状态
void PID_Clear_Integral(PID_t *pid);  // 仅清积分
```

**使用示例**:
```c
// 位置式PID
PID_t speed_pid;
PID_Init(&speed_pid, PID_POSITION, 5.0f, 0.1f, 0.0f, 0.0f, 16384.0f, 5000.0f);
float output = PID_Calculate(&speed_pid, 1000.0f, current_speed);

// 增量式PID
PID_t delta_pid;
PID_Init(&delta_pid, PID_DELTA, 5.0f, 0.2f, 0.0f, 0.0f, 600.0f, 200.0f);

// 串级PID
CASCADE_PID_t pos_speed_pid;
CASCADE_PID_Init(&pos_speed_pid, 
                 10.0f, 0.1f, 0.5f, 0.0f,  // 外环参数
                 5.0f, 0.2f, 0.0f, 0.0f,   // 内环参数
                 100.0f, 50.0f,             // 外环限幅
                 16384.0f, 10000.0f);       // 内环限幅
```

**配置宏**:
```c
// 在PID.h中配置
#define PID_DERIVATIVE_ON_MEASUREMENT 0U  // 1=微分先行, 0=普通微分
```

---

### HX06L (总线舵机)

**文件位置**: `User/inc/HX06L.h`, `User/src/HX06L.c`

**主要功能**:
- 串口总线舵机控制
- 支持位置、速度、力矩控制
- 多种工作模式

**协议格式**:
```
帧头(0x55 0x55) | ID | Length | Cmd | Params | CRC
```

**工作模式**:
- 位置控制模式(Servo_PosCtrl)
- 电机控制模式(Servo_MotorCtrl)
- 固定占空比模式(Rotate_Duty)
- 固定速度模式(Rotate_Speed)

**使用说明**:
- 波特率: 115200
- ID范围: 0-253
- 角度范围: 0-1000 (映射到0°-240°)
- 电压范围: 4500-14000 mV
- 温度阈值: 默认85℃

**注意事项**:
- 使用UART6进行通信
- 需配合BSP_UART模块使用
- CRC校验算法: `~(累加和) & 0xFF`
- 数据帧格式: `帧头(0x55 0x55) | ID | 数据长度 | 指令 | 参数 | 校验`

---

### UserTask (任务定义)

**文件位置**: `User/inc/UserTask.h`, `User/src/UserTask.c`

**规划任务**:
1. 换弹任务
2. 扳机任务(含松紧调节)
3. 通信任务(CAN, UART)
4. 拉簧动能存储任务
5. Yaw调整任务
6. 云台调节任务

**使用FreeRTOS组件**:
- 任务管理
- 事件组
- 信号量
- 队列

---

## 系统架构

```
┌─────────────────────────────────────┐
│          应用层 (User)               │
├─────────────────────────────────────┤
│  CanMotor  │ DM_Motor │ RM_Motor    │
│  HX06L     │ PID      │ UserTask    │
└─────────────────────────────────────┘
                 ↓
┌─────────────────────────────────────┐
│        硬件抽象层 (BSP)              │
├─────────────────────────────────────┤
│  bsp_can   │ bsp_dwt  │ bsp_uart    │
└─────────────────────────────────────┘
                 ↓
┌─────────────────────────────────────┐
│        HAL层 (STM32 HAL)            │
└─────────────────────────────────────┘
```

---

## 数据流图

### CAN电机控制流程
```
用户应用
   ↓
RmMotorSendCfg / DmMotorSendCfg
   ↓
MotorManager (电机管理器)
   ↓
bsp_can (CAN_SendData)
   ↓
CAN总线
   ↓
电机电调
   ↓
CAN总线 (反馈)
   ↓
CAN中断 (HAL_CAN_RxFifoXMsgPendingCallback)
   ↓
CAN_FIFO_CBKHANDLER
   ↓
RM_MOTOR_CALCU / DM_MOTOR_CALCU
   ↓
全局数据数组 (rm_motor_solved_data / dm_motor_solved_data)
   ↓
用户应用读取
```

### 舵机控制流程
```
用户应用
   ↓
HX06L指令组包
   ↓
UART_Send (BSP_UART)
   ↓
UART发送中断
   ↓
舵机
   ↓
UART接收中断
   ↓
协议解析 (ParseProtocol)
   ↓
ServoPacket_t
   ↓
用户应用读取 (UART_GetServoPacket)
```

---

## 配置参数速查表

### 电机配置

| 电机 | 品牌 | ID | 用途 | 控制模式 |
|------|-----|----| -----|---------|
| RM3508 | 大疆 | 1 | 夹爪 | 电流(串级PID) |
| RM2006 | 大疆 | 2 | 扳机 | 电流(串级PID) |
| DM3510 | 达妙 | 1 | 左拉力 | MIT模式 |
| DM3510 | 达妙 | 2 | 右拉力 | MIT模式 |
| DM4310 | 达妙 | 3 | Yaw轴 | MIT模式 |

### CAN通信参数

| 参数 | 值 | 说明 |
|-----|---|------|
| 波特率 | 1Mbps | 标准CAN速度 |
| RM发送ID | 0x200 | 固定 |
| RM接收ID | 0x201-0x204 | ID1-4 |
| DM发送ID | 0x001-0x003 | 对应电机ID |
| DM接收ID | 0x011-0x013 | 发送ID+0x10 |

### UART通信参数

| 参数 | UART3 | UART6 |
|-----|-------|-------|
| 波特率 | 115200 | 115200 |
| 用途 | 上位机通信 | 舵机通信 |
| 协议 | DART | 0x55 0x55 |

---

## 调试技巧

### 1. CAN通信调试
```c
// 在CAN_FIFO_CBKHANDLER中添加
printf("RX ID: 0x%03X, Data: %02X %02X...\n", 
       pRxHeader.StdId, data[0], data[1]);
```

### 2. 电机数据监控
```c
// 定时打印电机状态
printf("Angle: %.2f°, Speed: %.2f rpm, Current: %.2f A\n",
       rm_motor_solved_data[0], rm_motor_solved_data[1], rm_motor_solved_data[2]);
```

### 3. PID参数调试
```c
// 输出PID中间变量
printf("Error: %.2f, P: %.2f, I: %.2f, D: %.2f, Out: %.2f\n",
       pid->error, p_out, i_out, d_out, pid->output);
```

### 4. 时间测量
```c
uint32_t cnt = 0;
DWT_GetDeltaT(&cnt);
// 执行代码
float time = DWT_GetDeltaT(&cnt);
printf("执行时间: %.6f s\n", time);
```

---

## 常见问题FAQ

**Q1: 电机不动？**
- 检查CAN线连接
- 检查电机ID配置
- 检查CAN过滤器设置
- 检查电机是否使能

**Q2: CRC校验失败？**
- 确认CRC计算范围
- 舵机: 从ID到参数结束
- DART: 数据部分

**Q3: PID振荡？**
- 降低P参数
- 增加D参数
- 检查输出限幅

**Q4: 通信丢帧？**
- 检查发送频率
- DM电机建议加200μs延时
- 检查缓冲区大小

**Q5: 延时不准？**
- 确认DWT_Init()已调用
- 检查CPU频率设置
- 避免在延时期间进长中断

---

## 开发建议

### 代码规范
1. 使用有意义的变量名
2. 添加必要的注释
3. 保持代码缩进一致
4. 函数功能单一

### 性能优化
1. 减少不必要的浮点运算
2. 使用DMA传输大量数据
3. 合理设置中断优先级
4. 避免在中断中执行耗时操作

### 安全性
1. 添加参数合法性检查
2. 实现电机过流保护
3. 添加温度监控
4. 实现急停机制

---

## 版本信息

- **项目**: 2026 Vanguard 飞镖控制系统
- **MCU**: STM32F427IIH6
- **IDE**: Keil MDK-ARM
- **HAL版本**: STM32Cube FW_F4 V1.27.0
- **RTOS**: FreeRTOS V10.3.1

---

## 文档维护

- **创建时间**: 2025/11/30
- **最后更新**: 2025/12/03
- **维护者**: Vanguard Team
- **文档版本**: v1.0

---

## 联系方式

如有问题或建议,请联系项目维护团队。

---

**文档结束**
