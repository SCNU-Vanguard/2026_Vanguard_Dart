# CanMotor 使用说明文档

## 文件信息
- **头文件**: `User/inc/CanMotor.h`
- **源文件**: `User/src/CanMotor.c`
- **作用**: CAN电机统一管理层(支持RM电机和DM电机)

---

## 功能概述

CanMotor是电机管理的核心模块，负责统一管理RM电机和DM电机，引入了硬件抽象层(HAL)和面向对象(OO)的设计思想，提供了高度解耦和易于扩展的电机管理机制。

### 2026-01-17 性能优化
- **数据展平**: 高频参数(减速比、ID等)直接存储在实例中，减少3层寻址开销。
- **快速分发**: 中断回调使用展平后的ID直接匹配，提升响应速度。

---

## 重要宏定义

```c
#define CtrlMotorLen 8           // 电机控制报文长度(默认8字节)
#define g_CanMotorNum 5          // CAN电机总数量 (定义在 bsp_can.h)
```

---

## 数据结构

### MotorTypeDef 电机结构体 (核心)
```c
typedef struct _MotorTypeDef {
    // ==================== 快速访问区 (展平数据 - 优化点) ====================
    uint8_t MotorID;              // 电机ID
    can_motor_band band;          // 电机品牌
    can_motor_model model;        // 电机型号
    uint16_t can_id_tx;           // CAN发送ID (直接访问)
    uint16_t can_id_rx;           // CAN接收ID (即原 CAN_Rid)
    float gear_ratio;             // 减速比 (直接从类拷贝)
    float current_ratio;          // 电流/力矩转换系数
    int16_t max_current;          // 最大电流/限幅值

    // ==================== 原始存储区与反馈 ====================
    motor_inf MotorInf;                           // 电机品牌与型号
    uint8_t ReceiveMotorData[8];                  // 接收数据缓冲
    CAN_TxHeaderTypeDef g_TxHeader;               // CAN发送报文头
    MotorSolvedData_t motor_data;                 // 电机解算数据
    MotorConfig_t config;                         // 用户可调配置
    MotorParams_t params;                         // 电机固有参数

    // ==================== 面向对象扩展 ====================
    union {
        const struct _RM_MotorClass *rm_motor_class;
        const struct _DM_MotorClass *dm_motor_class;
    } motor_class;

    // ==================== 兼容性接口 (保留旧代码) ====================
    uint8_t (*SendMotorControl)(struct _MotorTypeDef *st);
    void (*calculate)(struct _MotorTypeDef *self);

    // PID控制器
    PID_t inner_pid;                              // 内环PID
    CASCADE_PID_t cascade_pid;                    // 串级PID
    uint16_t CAN_Rid;                             // 接收ID (兼容旧接口)
} MotorTypeDef;
```

---

## 核心API函数

### 1. 系统初始化与注册
- `MotorInit()`: 总初始化函数
- `MotorRegister()`: 注册所有电机
- `CanFilterCfg()`: 配置CAN过滤器（修正了拼写错误 Filter）

### 2. 电机句柄获取 (内联函数)
- `Motor_GetHandle(motor_id)`: 获取电机指针（带检查）
- `Motor_GetHandleFast(motor_id)`: 获取电机指针（最高性能）

### 3. 数据读取接口
- `Motor_GetTotalAngle(motor_id)`: 获取累计角度(°)
- `Motor_GetSpeedRPM(motor_id)`: 获取速度(rpm)
- `Motor_GetCurrent(motor_id)`: 获取电流(A)或力矩(N·m)

---

## DM电机ID配置表

### 2026-01-21 新增：集中式ID管理

DM电机的CAN ID现在通过配置表集中管理，修改ID只需修改一处。

#### 配置结构体
```c
typedef struct {
    uint8_t  motor_id;      // 达妙上位机设置的电机ID (0-15)
    uint16_t tx_id;         // 发送CAN ID
    uint16_t rx_id;         // 接收CAN ID
    DM_WorkMode work_mode;  // 工作模式
} DM_MotorIdConfig_t;
```

#### 当前ID配置 (`DM_Motor.c`)
```c
static const DM_MotorIdConfig_t g_DM_IdTable[] = {
    [DM_3519_STRENTH_LEFT]  = {1, 0x101, 0x021, DM_LOCATION_SPEED},
    [DM_3519_STRENTH_RIGHT] = {2, 0x102, 0x022, DM_LOCATION_SPEED},
    [DM_4310_YAW]           = {3, 0x003, 0x013, DM_MIT},
};
```

#### ID计算规则（供参考）
| 模式 | 发送ID | 接收ID |
|------|--------|--------|
| MIT | `0x000 + motor_id` | `0x010 + motor_id` |
| 位置速度 | `0x100 + motor_id` | `0x020 + motor_id` |
| 速度 | `0x200 + motor_id` | `0x030 + motor_id` |

#### 新建DM电机步骤
1. **CanMotor.h**: 在 `can_motor_cfg` 枚举中添加新电机
2. **DM_Motor.c**: 在 `g_DM_IdTable[]` 中添加ID配置
3. **DM_Motor.c**: 修改 `DM_GetIdConfig()` 范围检查
4. **CanMotor.c**: 在 `MotorRegister()` 中注册电机
5. **CanMotor.h**: 确保 `g_CanMotorNum` 足够

---

## 更新日志

- **2026-01-21**:
  - 新增DM电机ID配置表，集中管理所有DM电机CAN ID
  - 添加 `DM_MotorIdConfig_t` 结构体和 `DM_GetIdConfig()` 函数
- **2026-01-17**: 
  - 实施数据展平架构优化。
  - 优化中断分发逻辑。
  - 修正全局 `Filter` 拼写错误。
- **2026-01-03**: 
  - 引入硬件抽象层(HAL)支持
  - 新增面向对象(OO)类结构体设计