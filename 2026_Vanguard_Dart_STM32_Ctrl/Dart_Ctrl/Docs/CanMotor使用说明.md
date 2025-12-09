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
3. **电机分类管理** - 区分RM和DM电机品牌及型号
4. **中断回调处理** - 处理CAN接收中断
5. **数据解算存储** - 每个电机独立的解算数据存储

---

## 重要宏定义

```c
#define CtrlMotorLen 8       // 电机控制报文长度(8字节)
#define g_CanMotorNum 5      // CAN电机总数量
#define TestUse 1U           // 测试模式开关

// 角度转换宏
#define DegreeToRad(degree) ((degree) * 0.01745329f)  // 度 → 弧度
#define RadToDegree(radian) ((radian) * 57.29578f)    // 弧度 → 度
```

---

## 数据结构

### can_motor_band 电机品牌枚举
```c
typedef enum {
    RM_MOTOR_BAND = 0,       // 大疆(RoboMaster)电机
    DM_MOTOR_BAND,           // 达妙电机
    CubeMars_MOTOR_BAND      // CubeMars电机
} can_motor_band;
```

### can_motor_model 电机型号枚举
```c
typedef enum {
    RmM2006 = 1,    // 大疆M2006电机
    RmM3508,        // 大疆M3508电机
    RmGM6020,       // 大疆GM6020电机
    DmS3510,        // 达妙S3510电机
    DmJ4310,        // 达妙J4310电机
    CmG80           // CubeMars G80电机
} can_motor_model;
```

### motor_inf 电机信息结构体
```c
typedef struct {
    can_motor_band band;     // 电机品牌
    can_motor_model model;   // 电机型号
} motor_inf;
```

### can_motor_cfg 电机配置枚举
```c
typedef enum {
    SingleMotorTest = 1,        // 单电机测试用
    RM_3508_GRIPPER = 1,        // RM3508夹爪电机
    RM_2006_TRIGGER,            // RM2006扳机电机
    DM_3510_STRENTH_LEFT,       // DM3510左拉力电机
    DM_3510_STRENTH_RIGHT,      // DM3510右拉力电机
    DM_4310_YAW                 // DM4310云台Yaw轴电机
} can_motor_cfg;
```

### MotorSolvedData_t 电机解算数据结构体
```c
typedef struct {
    float solved_data[5];      // 解算后的数据数组
    
    // RM电机: [0]单圈角度(°), [1]速度(rpm), [2]电流(A), [3]累计角度(°), [4]速度(rad/s)
    // DM电机: [0]位置(rad/°), [1]速度(rad/s), [2]力矩(N·m), [3]MOS温度(℃), [4]转子温度(℃)
    
    // 多圈累计相关变量
    int16_t last_ecd;          // 上次编码器值
    int32_t total_round;       // 累计圈数
    int32_t total_ecd;         // 累计编码器值
    int16_t offset_ecd;        // 零点偏移
    uint8_t init_flag;         // 初始化标志
    
    // 速度滤波相关
    float last_speed;          // 上次速度值
    uint8_t filter_init;       // 滤波器初始化标志
    
    // 控制相关变量
    float target_angle;        // 目标角度
    float last_target;         // 上次目标值
    float pre_last_target;     // 上上次目标值
} MotorSolvedData_t;
```

### MotorTypeDef 电机结构体
```c
typedef struct _MotorTypeDef {
    uint8_t MotorID;                              // 电机ID
    motor_inf MotorInf;                           // 电机信息(品牌+型号)
    uint8_t (*SendMotorControl)(struct _MotorTypeDef *st);  // 发送函数指针
    uint8_t ReceiveMotorData[8];                  // 接收数据缓冲
    uint8_t SendMotorData[8];                     // 发送数据缓冲
    CAN_TxHeaderTypeDef g_TxHeader;               // CAN发送报文头
    
    // 电机反馈数据解算存储（每个电机独立）
    MotorSolvedData_t motor_data;                 // 电机解算数据
    
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

### 2. MotorRegister()
**功能**: 注册所有电机信息
```c
void MotorRegister(void);
```

### 3. CanRegisterMotorCfg()
**功能**: 注册电机的CAN通信信息
```c
void CanRegisterMotorCfg(MotorTypeDef *ptr);
```

### 4. CanFliterCfg()
**功能**: 配置CAN过滤器
```c
void CanFliterCfg(void);
```

### 5. CAN_FIFO_CBKHANDLER()
**功能**: CAN FIFO中断回调处理函数
```c
void CAN_FIFO_CBKHANDLER(uint32_t fifo_num, uint8_t FIFOmessageNum);
```

### 6. GetPtrMotorManager()
**功能**: 获取电机管理器结构体
```c
MotorManager_t GetPtrMotorManager(void);
```

---

## RM电机专用函数 (参数使用 can_motor_cfg 枚举)

### RmMotorSendCfg()
```c
void RmMotorSendCfg(can_motor_cfg motor_cfg, int16_t TargetCurrent);
```
**说明**: 发送电流控制指令，使用反码形式表示负数

### RM_MotorSetTxData()
```c
void RM_MotorSetTxData(can_motor_cfg motor_cfg, uint8_t *data);
```
**说明**: 设置发送数据并触发发送

### RmMotorPID_Calc()
```c
void RmMotorPID_Calc(can_motor_cfg motor_cfg, float target);
```
**说明**: RM电机PID计算并发送

### RmMotorRemoveBias()
```c
float RmMotorRemoveBias(can_motor_cfg motor_cfg, float Target);
```
**说明**: 去除电机偏移对目标数值的影响

---

## DM电机专用函数 (参数使用 can_motor_cfg 枚举)

### DmMotorSendCfg()
```c
void DmMotorSendCfg(can_motor_cfg motor_cfg, float TargetPos, float TargetVel);
```
**说明**: 发送位置和速度控制指令(MIT模式)

### DM_MotorSetTxData()
```c
void DM_MotorSetTxData(can_motor_cfg motor_cfg, uint8_t *data);
```
**说明**: 设置DM电机发送数据

### DM_MotorDisable()
```c
uint8_t DM_MotorDisable(can_motor_cfg motor_cfg);
```
**说明**: 失能DM电机

### DmMotorPID_Calc()
```c
void DmMotorPID_Calc(can_motor_cfg motor_cfg, float target);
```
**说明**: DM电机PID计算

---

## 电机ID分配规则 (RM电机)

### M3508 电机 (C620电调)
| 项目 | 值 | 说明 |
|-----|---|------|
| 发送ID | 0x200 | ID 1-4 共用 |
| 反馈ID | 0x201-0x204 | 对应ID 1-4 |
| 数据位置 | ID×2-2 ~ ID×2-1 | 2字节/电机 |

### M2006 电机 (C610电调)
| 项目 | 值 | 说明 |
|-----|---|------|
| 发送ID | 0x1FF | ID 1-4 共用 |
| 反馈ID | 0x205-0x208 | 对应ID 1-4 |
| 数据位置 | ID×2-2 ~ ID×2-1 | 2字节/电机 |

### GM6020 电机 (电压控制)
| 项目 | 值 | 说明 |
|-----|---|------|
| 发送ID | 0x1FF (ID 1-4) / 0x2FF (ID 5-7) | 电压控制 |
| 发送ID | 0x1FE (ID 1-4) / 0x2FE (ID 5-7) | 电流控制 |
| 反馈ID | 0x205-0x20B | 对应ID 1-7 |

**⚠️ 注意**: GM6020 ID 1-4 的反馈ID与M2006冲突，建议将GM6020的ID设置为5-7

---

## 电机数据解算

### RM电机解算数据 (motor_data.solved_data[])
| 索引 | 数据 | 单位 | 说明 |
|-----|------|------|------|
| [0] | 单圈角度 | ° | 0~360 |
| [1] | 速度 | rpm | 带滤波 |
| [2] | 电流 | A | M3508: 16384→20A, M2006: /0.18, GM6020: 16384→3A |
| [3] | 累计角度 | ° | 用于位置闭环 |
| [4] | 速度 | rad/s | 弧度制 |

### DM电机解算数据 (motor_data.solved_data[])
| 索引 | 数据 | 单位 | 说明 |
|-----|------|------|------|
| [0] | 位置 | rad/° | MIT模式 |
| [1] | 速度 | rad/s | 带滤波 |
| [2] | 力矩 | N·m | - |
| [3] | MOS温度 | ℃ | - |
| [4] | 转子温度 | ℃ | - |

---

## 典型使用示例

### 1. 初始化
```c
int main(void) {
    HAL_Init();
    SystemClock_Config();
    
    // 初始化电机
    MotorInit();
    
    while(1) {
        // 主循环
    }
}
```

### 2. 控制RM电机
```c
// 使用枚举值直接控制
RmMotorSendCfg(RM_3508_GRIPPER, 1000);  // 发送电流1000
RmMotorSendCfg(RM_2006_TRIGGER, -500);  // 发送电流-500

// 使用PID控制
RmMotorPID_Calc(RM_3508_GRIPPER, 180.0f);  // 目标角度180度
```

### 3. 控制DM电机
```c
// 发送位置速度控制
DmMotorSendCfg(DM_4310_YAW, 90.0f, 10.0f);  // 目标位置90°，速度10

// 失能电机
DM_MotorDisable(DM_3510_STRENTH_LEFT);
```

### 4. 读取电机反馈
```c
// 访问电机解算数据
MotorTypeDef *motor = &MotorManager.MotorList[RM_3508_GRIPPER - 1];
float angle = motor->motor_data.solved_data[0];      // 角度
float speed = motor->motor_data.solved_data[1];      // 速度
float current = motor->motor_data.solved_data[2];    // 电流
float total_angle = motor->motor_data.solved_data[3]; // 累计角度
```

---

## 测试模式配置

```c
// 在 CanMotor.h 中
#define TestUse 1U        // 启用测试模式

// 在 RM_Motor.h 中
#define RM_TestUse 1U     // RM电机测试

// 在 DM_Motor.h 中
#define DM_TestUse 0U     // DM电机测试
```

---

## 注意事项

1. **枚举值使用**: 所有电机控制函数都使用 `can_motor_cfg` 枚举，值从1开始
2. **数组索引**: `MotorList[can_motor_cfg - 1]` 访问电机
3. **品牌和型号**: 必须同时设置 `MotorInf.band` 和 `MotorInf.model`
4. **ID冲突**: GM6020和M2006在ID 1-4时反馈ID冲突，需要规划好ID分配
5. **发送间隔**: DM电机连续发送需添加延时防止丢帧

---

## 更新日志

- 新增 `can_motor_model` 电机型号枚举
- 新增 `motor_inf` 电机信息结构体
- 新增 `MotorSolvedData_t` 解算数据结构体（每个电机独立）
- 所有电机控制函数参数改为 `can_motor_cfg` 枚举类型
- 支持M3508、M2006、GM6020三种RM电机型号
- 完善各型号的电流解算公式
