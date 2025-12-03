# CanMotor 使用说明文档

## 文件信息
- **头文件**: `User/inc/CanMotor.h`
- **源文件**: `User/src/CanMotor.c`
- **作用**: CAN电机统一管理层(支持RM电机和DM电机)

---

## 功能概述

CanMotor是电机管理的核心模块，负责统一管理RM电机和DM电机，提供统一的接口和电机注册机制。

### 核心功能
1. **电机注册管理** - 统一注册和管理多种品牌电机
2. **CAN通信管理** - 协调电机的CAN收发
3. **电机分类管理** - 区分RM和DM电机品牌
4. **中断回调处理** - 处理CAN接收中断

---

## 重要宏定义

```c
#define CtrlMotorLen 8       // 电机控制报文长度(8字节)
#define g_CanMotorNum 5      // CAN电机总数量
```

---

## 数据结构

### can_motor_band 电机品牌枚举
```c
typedef enum {
    RM_MOTOR_BAND = 0,  // 大疆(RoboMaster)电机
    DM_MOTOR_BAND       // 达妙电机
} can_motor_band;
```

### can_motor_cfg 电机配置枚举
```c
typedef enum {
    RM_3508_GRIPPER = 1,        // RM3508夹爪电机
    RM_2006_TRIGGER,            // RM2006扳机电机
    DM_3510_STRENTH_LEFT,       // DM3510左拉力电机
    DM_3510_STRENTH_RIGHT,      // DM3510右拉力电机
    DM_4310_YAW                 // DM4310云台Yaw轴电机
} can_motor_cfg;
```

### MotorTypeDef 电机结构体
```c
typedef struct _MotorTypeDef {
    uint8_t MotorID;                              // 电机ID
    uint8_t MotorBand;                            // 电机品牌
    uint8_t (*SendMotorControl)(struct _MotorTypeDef *st);  // 发送函数指针
    uint8_t ReceiveMotorData[8];                  // 接收数据缓冲
    uint8_t SendMotorData[8];                     // 发送数据缓冲
    CAN_TxHeaderTypeDef g_TxHeader;               // CAN发送报文头
    
    // PID控制器
    PID_t speed_pid;                              // 速度环PID
    CASCADE_PID_t cascade_pid;                    // 串级PID
    uint8_t use_cascade;                          // 是否使用串级: 0-单环，1-串级
} MotorTypeDef;
```

### MotorManager_t 电机管理器
```c
typedef struct {
    MotorTypeDef MotorList[g_CanMotorNum];       // 电机列表
    uint8_t registered_count;                     // 已注册电机数量
    uint8_t RM_MOTOR_DATA_ARRAY[8];              // RM电机发送数据数组
} MotorManager_t;

extern MotorManager_t MotorManager;  // 全局电机管理器
```

---

## 核心API函数

### 1. MotorInit()
**功能**: 电机初始化(上电后调用)
```c
void MotorInit(void);
```

**功能说明**:
- 注册所有电机
- 初始化CAN总线
- 配置CAN过滤器

**使用示例**:
```c
// 在main函数中调用
MotorInit();
```

**内部流程**:
1. 调用`MotorRegister()`注册电机
2. 初始化CAN1的FIFO0和FIFO1
3. 配置CAN过滤器(可选)

---

### 2. MotorRegister()
**功能**: 注册所有电机信息
```c
void MotorRegister(void);
```

**注册内容**:
- 电机ID和品牌
- 发送函数指针
- PID参数初始化
- CAN报文头配置

**使用示例**:
```c
// 通常由MotorInit()自动调用，无需手动调用
```

**电机配置示例**:
```c
// RM3508夹爪电机
MotorManager.MotorList[RM_3508_GRIPPER - 1].MotorID = RM_3508_GRIPPER;
MotorManager.MotorList[RM_3508_GRIPPER - 1].MotorBand = RM_MOTOR_BAND;
MotorManager.MotorList[RM_3508_GRIPPER - 1].SendMotorControl = RM_MotorSendControl;
MotorManager.MotorList[RM_3508_GRIPPER - 1].use_cascade = 1;  // 使用串级PID
```

---

### 3. CanRegisterMotorCfg()
**功能**: 注册电机的CAN通信信息
```c
void CanRegisterMotorCfg(MotorTypeDef *ptr);
```

**参数说明**:
- `ptr`: 电机结构体指针

**配置内容**:
- CAN报文头的StdId(标准ID)
- IDE(标准帧)
- RTR(数据帧)
- DLC(数据长度)

**自动计算ID**:
- RM电机: `StdId = 0x200`
- DM电机: `StdId = 0x000 + MotorID`

---

### 4. CanFliterCfg()
**功能**: 配置CAN过滤器
```c
void CanFliterCfg(void);
```

**功能说明**:
- 配置FIFO0接收RM电机反馈(ID: 0x200+)
- 配置FIFO1接收DM电机反馈(ID: 0x010+)
- 过滤非电机相关的CAN消息

**使用示例**:
```c
// 可选调用，用于过滤ID
CanFliterCfg();
```

---

### 5. CAN_FIFO_CBKHANDLER()
**功能**: CAN FIFO中断回调处理函数
```c
void CAN_FIFO_CBKHANDLER(uint32_t fifo_num, uint8_t FIFOmessageNum);
```

**参数说明**:
- `fifo_num`: FIFO编号(fifo0或fifo1)
- `FIFOmessageNum`: 待处理的消息数量

**功能说明**:
1. 循环读取FIFO中的所有消息
2. 根据ID匹配对应的电机
3. 调用对应电机的数据解算函数
4. 存储解算后的数据

**处理流程**:
```
收到CAN消息 → 读取ID → 匹配电机 → 调用解算函数 → 存储数据
```

---

### 6. GetPtrMotorManager()
**功能**: 获取电机管理器指针
```c
MotorManager_t GetPtrMotorManager(void);
```

**返回值**: 电机管理器结构体

**使用示例**:
```c
MotorManager_t manager = GetPtrMotorManager();
printf("已注册电机数量: %d\n", manager.registered_count);
```

---

## 电机访问方法

### 访问电机列表
```c
// 访问第1个电机(RM3508夹爪)
MotorTypeDef *motor = &MotorManager.MotorList[0];

// 访问指定配置的电机
MotorTypeDef *gripper = &MotorManager.MotorList[RM_3508_GRIPPER - 1];
```

### 发送电机控制指令
```c
// 方式1: 直接调用电机的发送函数
MotorManager.MotorList[RM_3508_GRIPPER - 1].SendMotorControl(&MotorManager.MotorList[RM_3508_GRIPPER - 1]);

// 方式2: 使用电机专用函数(推荐)
RmMotorSendCfg(RM_3508_GRIPPER, 1000);  // RM电机
DmMotorSendCfg(1, 90.0, 0.0);           // DM电机
```

### 读取电机反馈数据
```c
// 电机反馈数据在中断中自动更新到全局变量
extern float rm_motor_solved_data[5];  // RM电机解算数据
extern float dm_motor_solved_data[5];  // DM电机解算数据

// RM电机数据 (5个float)
float angle = rm_motor_solved_data[0];        // 单圈角度(°) 0~360
float speed_rpm = rm_motor_solved_data[1];    // 速度(rpm)
float current = rm_motor_solved_data[2];      // 电流(A)
float total_angle = rm_motor_solved_data[3];  // 累计角度(°) - 用于位置闭环
float speed_rad = rm_motor_solved_data[4];    // 速度(rad/s)
```

---

## 典型应用场景

### 场景1: 初始化所有电机
```c
int main(void) {
    // 系统初始化
    HAL_Init();
    SystemClock_Config();
    
    // 初始化电机
    MotorInit();
    
    while(1) {
        // 主循环
    }
}
```

### 场景2: 控制单个电机
```c
// 控制夹爪电机(RM3508)
void ControlGripper(int16_t target_current) {
    RmMotorSendCfg(RM_3508_GRIPPER, target_current);
}

// 控制Yaw轴电机(DM4310)
void ControlYaw(float target_angle, float target_speed) {
    DmMotorSendCfg(DM_4310_YAW - g_RM_MOTOR_NUM, target_angle, target_speed);
}
```

### 场景3: 使用PID控制
```c
// 初始化PID参数
void InitMotorPID(void) {
    // 设置夹爪电机串级PID
    CASCADE_PID_Init(
        &MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid,
        10.0f, 0.1f, 0.5f, 0.0f,  // 外环(位置环)
        5.0f, 0.2f, 0.0f, 0.0f,   // 内环(速度环)
        100.0f, 50.0f,             // 外环限幅
        16384.0f, 10000.0f         // 内环限幅
    );
}

// PID控制任务
void MotorControlTask(void) {
    float target_angle = 360.0f;  // 目标角度
    float output;
    
    // 串级PID计算
    output = CASCADE_PID_Calculate(
        &MotorManager.MotorList[RM_3508_GRIPPER - 1].cascade_pid,
        target_angle,              // 目标位置
        rm_motor_solved_data[0],   // 实际位置
        rm_motor_solved_data[1]    // 实际速度
    );
    
    // 发送控制指令
    RmMotorSendCfg(RM_3508_GRIPPER, (int16_t)output);
}
```

### 场景4: 读取电机状态
```c
void PrintMotorStatus(void) {
    printf("夹爪电机状态:\n");
    printf("  角度: %.2f°\n", rm_motor_solved_data[0]);
    printf("  速度: %.2f rpm\n", rm_motor_solved_data[1]);
    printf("  电流: %.2f A\n", rm_motor_solved_data[2]);
}
```

---

## 电机ID分配规则

### RM电机
- **发送ID**: 0x200 (固定)
- **接收ID**: 0x200 + MotorID
- **数据位置**: 根据ID在发送数组中的位置
  - ID 1: data[0-1]
  - ID 2: data[2-3]
  - ID 3: data[4-5]
  - ID 4: data[6-7]

### DM电机
- **发送ID**: 0x000 + MotorID
- **接收ID**: 0x010 + MotorID
- **独立发送**: 每个电机独立发送

---

## PID控制模式选择

### 单环PID (use_cascade = 0)
- 适用于: 速度控制
- 使用: `speed_pid`
- 示例:
```c
MotorManager.MotorList[i].use_cascade = 0;
PID_Init(&MotorManager.MotorList[i].speed_pid, 
         PID_DELTA, 5.0f, 0.2f, 0.0f, 0.0f, 16384.0f, 10000.0f);
```

### 串级PID (use_cascade = 1)
- 适用于: 位置-速度控制
- 使用: `cascade_pid`
- 外环: 位置环
- 内环: 速度环
- 示例:
```c
MotorManager.MotorList[i].use_cascade = 1;
CASCADE_PID_Init(&MotorManager.MotorList[i].cascade_pid,
                 10.0f, 0.1f, 0.5f, 0.0f,  // 外环
                 5.0f, 0.2f, 0.0f, 0.0f,   // 内环
                 100.0f, 50.0f, 16384.0f, 10000.0f);
```

---

## 中断回调流程

```
CAN接收中断 → HAL_CAN_RxFifo0MsgPendingCallback/HAL_CAN_RxFifo1MsgPendingCallback
              ↓
           CAN_FIFO_CBKHANDLER(fifo_num, message_count)
              ↓
           循环处理每条消息:
              - HAL_CAN_GetRxMessage() 读取消息
              - 根据ID匹配电机
              - 调用RM_MOTOR_CALCU()或DM_MOTOR_CALCU()解算
              - 存储数据到MotorList[i].ReceiveMotorData
              ↓
           更新全局解算数据数组
```

---

## 使用注意事项

1. **初始化顺序**
   - 必须先调用`MotorInit()`
   - 再调用其他电机控制函数

2. **ID冲突**
   - 确保每个电机的ID唯一
   - RM电机ID: 1-4
   - DM电机ID: 需避开RM电机ID范围

3. **数组索引**
   - 电机列表索引从0开始
   - 配置枚举值从1开始
   - 访问时注意: `MotorList[ID - 1]`

4. **PID参数**
   - 根据实际电机特性调整PID参数
   - 建议先在测试模式下调试参数

5. **中断安全**
   - 电机数据在中断中更新
   - 读取数据时注意数据一致性

6. **发送频率**
   - RM电机: 可以1kHz高频发送
   - DM电机: 建议添加200μs间隔(使用DWT_Delay_us)

---

## 测试模式

### 单电机测试
```c
// 在CanMotor.h中定义
#define TestUse 1U           // 启用测试模式
#define DM_TestUse 0U        // DM电机测试
#define RM_TestUse 1U        // RM电机测试

// 测试函数会自动注册单个电机
```

---

## 错误处理

### 常见错误
1. **CAN通信失败**: 检查CAN线路和波特率
2. **电机无响应**: 检查电机ID和过滤器配置
3. **数据解算异常**: 检查接收数据格式

### 调试建议
```c
// 在CAN_FIFO_CBKHANDLER中添加调试信息
if(CAN_RX_DATA_COUNT == 0) {
    printf("未匹配到电机ID: 0x%03X\n", pRxHeader.StdId);
}
```

---

## 相关模块

- **bsp_can**: CAN底层驱动
- **RM_Motor**: RM电机控制模块
- **DM_Motor**: DM电机控制模块
- **PID**: PID控制算法

---

## 性能参数

| 参数 | 值 | 说明 |
|-----|---|------|
| 最大电机数 | 5 | 可修改g_CanMotorNum |
| RM电机数 | 2 | 夹爪+扳机 |
| DM电机数 | 3 | 左右拉力+Yaw |
| CAN波特率 | 1Mbps | 标准CAN速度 |
| 控制频率 | 可达1kHz | 根据任务调度 |

---

## 更新日志

- 统一管理RM和DM电机
- 支持串级PID和单环PID
- 自动化电机注册流程
- 完善的中断回调机制
