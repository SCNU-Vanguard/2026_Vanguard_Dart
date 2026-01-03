# CanMotor 使用说明文档

## 文件信息
- **头文件**: `User/inc/CanMotor.h`
- **源文件**: `User/src/CanMotor.c`
- **作用**: CAN电机统一管理层(支持RM电机和DM电机)

---

## 功能概述

CanMotor是电机管理的核心模块，负责统一管理RM电机和DM电机，引入了硬件抽象层(HAL)和面向对象(OO)的设计思想，提供了高度解耦和易于扩展的电机管理机制。

### 核心功能
1. **电机注册管理** - 统一注册和管理多种品牌电机
2. **硬件抽象层 (HAL)** - 解耦底层CAN发送和延时实现，便于移植
3. **面向对象设计** - 使用类结构体(Class)管理不同型号电机的特性和算法
4. **中断回调处理** - 统一处理CAN接收中断并分发给对应电机
5. **数据解算存储** - 每个电机独立的解算数据存储，支持多圈角度和速度滤波

---

## 重要宏定义

```c
#define CtrlMotorLen 8           // 电机控制报文长度(默认8字节)
#define g_CanMotorNum 5          // CAN电机总数量 (定义在 bsp_can.h)
#define TestUse 0U               // 测试模式开关

// 角度转换宏
#define DegreeToRad(degree) ((degree) * 0.01745329f)  // 度 → 弧度
#define RadToDegree(radian) ((radian) * 57.29578f)    // 弧度 → 度
```

---

## 硬件抽象层 (HAL)

为了提高代码的可移植性，CanMotor引入了 `MotorHAL_t` 结构体：

```c
typedef struct {
    uint8_t (*can_send)(CAN_TxHeaderTypeDef *hdr, uint8_t *data); // CAN发送函数
    void (*delay_ms)(uint32_t ms);                                // 延时函数
} MotorHAL_t;
```

可以通过 `Motor_SetHAL()` 在初始化时注入具体的硬件实现。

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
    DmS3519,        // 达妙S3519电机
    DmJ4310,        // 达妙J4310电机
    CmG80           // CubeMars G80电机
} can_motor_model;
```

### can_motor_cfg 电机配置枚举 (用户定义)
```c
typedef enum {
    RM_3508_GRIPPER = 1,        // 夹爪电机
    RM_2006_TRIGGER,            // 扳机电机
    DM_3519_STRENTH_LEFT,       // 左拉力电机
    DM_3519_STRENTH_RIGHT,      // 右拉力电机
    DM_4310_YAW                 // 云台Yaw轴电机
} can_motor_cfg;
```

### MotorSolvedData_t 电机解算数据结构体
```c
typedef struct {
    float solved_data[8];      // 解算后的数据数组
    // RM电机: [0]单圈角度(°), [1]速度(rpm), [2]电流(A), [3]累计角度(°), [4]速度(rad/s)
    // DM电机: [0]位置(rad/°), [1]速度(rad/s), [2]力矩(N·m), [3]MOS温度(℃), [4]转子温度(℃)

    int16_t last_ecd;          // 上次编码器值
    int32_t total_round;       // 累计圈数
    int32_t total_ecd;         // 累计编码器值
    int16_t offset_ecd;        // 零点偏移
    float offset_ecd_angle;    // 零点偏移角度
    float last_speed;          // 上次速度值(用于滤波)
    float target_angle;        // 目标角度
} MotorSolvedData_t;
```

### MotorTypeDef 电机结构体 (核心)
```c
typedef struct _MotorTypeDef {
    uint8_t MotorID;                              // 电机ID
    motor_inf MotorInf;                           // 电机品牌与型号
    uint8_t ReceiveMotorData[8];                  // 接收数据缓冲
    CAN_TxHeaderTypeDef g_TxHeader;               // CAN发送报文头
    MotorSolvedData_t motor_data;                 // 电机解算数据
    MotorConfig_t config;                         // 用户可调配置(方向、容限等)
    MotorParams_t params;                         // 电机固有参数(减速比、最大电流等)

    // 面向对象扩展：电机类指针
    union {
        const struct _RM_MotorClass *rm_motor_class;
        const struct _DM_MotorClass *dm_motor_class;
    } motor_class;

    // PID控制器
    PID_t inner_pid;                              // 内环PID
    CASCADE_PID_t cascade_pid;                    // 串级PID
    uint8_t use_cascade;                          // 0-单环，1-串级
    uint16_t CAN_Rid;                             // 接收ID
} MotorTypeDef;
```

---

## 核心API函数

### 1. 系统初始化与注册
- `MotorInit()`: 总初始化函数
- `MotorRegister()`: 注册所有电机
- `Motor_SetHAL(hal)`: 设置硬件抽象层实现

### 2. 电机句柄获取 (高性能内联函数)
- `Motor_GetHandle(motor_id)`: 获取电机结构体指针（带边界检查）
- `Motor_GetHandleFast(motor_id)`: 获取电机结构体指针（无检查，最高性能）

### 3. 数据读取接口
- `Motor_GetTotalAngle(motor_id)`: 获取累计角度(°)
- `Motor_GetTotalAngleRad(motor_id)`: 获取累计角度(rad)
- `Motor_GetSpeedRPM(motor_id)`: 获取速度(rpm)
- `Motor_GetSpeedRadS(motor_id)`: 获取速度(rad/s)
- `Motor_GetCurrent(motor_id)`: 获取电流(A)或力矩(N·m)

---

## 面向对象用法 (推荐)

新架构推荐通过电机的“类”接口进行操作，实现多态控制。

### RM电机控制
```c
MotorTypeDef *motor = Motor_GetHandle(RM_3508_GRIPPER);
// 1. 设置PID参数
RM_Motor_SetSpeedPID(motor, PID_POSITION, 10.0f, 0.1f, 0.0f, 0.0f, 5000, -5000, 1000);
// 2. 计算并发送控制
RM_Motor_Calculate(motor);       // 解算
RM_Motor_PIDCalc(motor, 1000);   // 计算PID(目标速度1000rpm)
RM_Motor_SendControl(motor);     // 发送控制报文
```

### DM电机控制
```c
MotorTypeDef *motor = Motor_GetHandle(DM_4310_YAW);
// 1. 设置MIT参数
DM_Motor_SetMITParams(motor, 10.0f, 0.5f, 0.0f); // KP, KD, TorqueFF
// 2. 发送控制
DM_Motor_SendControl(motor);
```

---

## 兼容性接口 (旧接口)

为了保持向后兼容，保留了部分基于 `can_motor_cfg` 枚举的接口：
- `RmMotorSendCfg(motor_cfg, current)`
- `DmMotorSendCfg(motor_cfg, pos, vel, mode)`
- `RmMotorPID_Calc(motor_cfg, target)`
- `DmMotorPID_Calc(motor_cfg, target)`

---

## 注意事项

1. **ID分配**: GM6020的反馈ID与M2006冲突，请参考RM电机手册避开重叠区域。
2. **数据单位**: RM电机默认使用度(°)和rpm，DM电机底层使用弧度(rad)和rad/s。
3. **HAL注入**: 务必在调用 `MotorInit` 前或过程中确保 HAL 接口已正确设置，否则 CAN 发送将失效。
4. **结构体对齐**: 所有关键结构体均使用 `#pragma pack(push, 1)` 保证无字节对齐空隙，便于直接进行内存拷贝。

---

## 更新日志

- **2026-01-03**: 
  - 引入硬件抽象层(HAL)支持
  - 新增面向对象(OO)类结构体设计
  - 电机型号 DM_3510 更新为 DM_3519
  - 新增大量高性能内联数据读取接口
