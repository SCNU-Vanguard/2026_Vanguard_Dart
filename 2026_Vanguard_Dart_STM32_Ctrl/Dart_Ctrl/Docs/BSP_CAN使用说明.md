# BSP_CAN 使用说明文档

## 文件信息
- **头文件**: `Bsp/inc/bsp_can.h`
- **源文件**: `Bsp/src/bsp_can.c`
- **作用**: CAN总线通信的底层硬件抽象层(BSP)

---

## 功能概述

本模块封装了STM32的CAN总线通信功能，提供了简洁的API接口用于CAN初始化、数据收发和过滤器配置。

### 核心功能
1. **CAN总线初始化** - 配置CAN外设和过滤器
2. **数据发送** - 发送CAN数据帧
3. **数据接收** - 通过中断回调接收CAN数据
4. **过滤器配置** - 灵活配置ID过滤规则

---

## 重要宏定义

```c
#define g_CanMotorNum 5  // CAN电机数量
```

---

## 数据结构

### CAN_FIFO 枚举
```c
typedef enum {
    fifo0 = CAN_FILTER_FIFO0,  // FIFO0寄存器
    fifo1 = CAN_FILTER_FIFO1   // FIFO1寄存器
} CAN_FIFO;
```

---

## 核心API函数

### 1. CAN_Init()
**功能**: 初始化CAN外设
```c
uint8_t CAN_Init(CAN_HandleTypeDef *canHandle, 
                 CAN_FIFO fifo, 
                 uint8_t FliterNum, 
                 uint8_t MaskStatus);
```

**参数说明**:
- `canHandle`: CAN句柄指针
- `fifo`: 使用的FIFO编号(fifo0或fifo1)
- `FliterNum`: 过滤器编号
- `MaskStatus`: 掩码状态

**返回值**: 1表示成功，失败会进入Error_Handler

**使用示例**:
```c
CAN_Init(&hcan1, fifo0, 0, 0);    // 初始化CAN1，使用FIFO0
CAN_Init(&hcan1, fifo1, 10, 0);   // 初始化CAN1，使用FIFO1
```

---

### 2. CAN_SendData()
**功能**: 发送CAN数据
```c
uint8_t CAN_SendData(CAN_HandleTypeDef *canHandle, 
                     CAN_TxHeaderTypeDef *TxHeader, 
                     uint8_t *data);
```

**参数说明**:
- `canHandle`: CAN句柄指针
- `TxHeader`: 发送报文头指针
- `data`: 要发送的数据指针(8字节)

**返回值**: 
- 1: 发送成功
- 0: 发送失败

**使用示例**:
```c
CAN_TxHeaderTypeDef TxHeader;
TxHeader.StdId = 0x200;           // 标准ID
TxHeader.IDE = CAN_ID_STD;        // 标准帧
TxHeader.RTR = CAN_RTR_DATA;      // 数据帧
TxHeader.DLC = 8;                 // 数据长度8字节

uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
CAN_SendData(&hcan1, &TxHeader, data);
```

---

### 3. FliterIdCfg_Init()
**功能**: 配置CAN过滤器ID
```c
void FliterIdCfg_Init(CAN_HandleTypeDef *canHandle, 
                      uint16_t *ID, 
                      uint16_t *Mask, 
                      uint8_t FliterNum, 
                      uint8_t fifo);
```

**参数说明**:
- `canHandle`: CAN句柄指针
- `ID`: ID数组地址(长度为2)
- `Mask`: 掩码数组地址(长度为2)
- `FliterNum`: 过滤器编号
- `fifo`: FIFO编号

**使用示例**:
```c
uint16_t ID_ARR[2] = {0x0200, 0x0010};      // 设置接收ID
uint16_t MASK_ARR[2] = {0x0000, 0x0000};    // 掩码配置
FliterIdCfg_Init(&hcan1, ID_ARR, MASK_ARR, 0, fifo0);
```

---

## CAN过滤器配置说明

### 标准帧ID过滤器映射
```
CAN_FxR1[15:8]   | CAN_FxR1[7:0]
STD[10:3]        | STD[2:0] RTR IDE EXID[17:15]
```

**重要位说明**:
- `STD[10:0]`: 标准ID，11位
- `RTR`: 帧类型(数据帧/远程帧)
- `IDE`: 是否为扩展ID(本工程使用标准ID)
- `EXID`: 扩展ID(不使用)

### 过滤器分配
- **CAN1**: 使用0-13号过滤器
- **CAN2**: 使用14-27号过滤器

---

## 中断回调函数

### HAL_CAN_RxFifo0MsgPendingCallback()
```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
```
- FIFO0接收中断回调
- 调用用户定义的`CAN_FIFO_CBKHANDLER(fifo0, 消息数)`

### HAL_CAN_RxFifo1MsgPendingCallback()
```c
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
```
- FIFO1接收中断回调
- 调用用户定义的`CAN_FIFO_CBKHANDLER(fifo1, 消息数)`

---

## 辅助函数

### CAN_FIFO_DLC()
**功能**: 查看FIFO中当前数据数量
```c
uint8_t CAN_FIFO_DLC(CAN_HandleTypeDef *hcan, uint8_t fifo_num)
```

**返回值**: FIFO中的数据帧数量(0-3)

---

## 使用注意事项

1. **过滤器配置**
   - 必须先调用`CAN_Init()`初始化CAN
   - 再使用`FliterIdCfg_Init()`配置具体的过滤ID
   - 默认配置会屏蔽所有消息(0xFFFF)

2. **中断回调**
   - 用户需要在应用层实现`CAN_FIFO_CBKHANDLER()`函数
   - 该函数会在接收到CAN消息时被调用

3. **ID配置**
   - 本工程默认使用**标准帧**和**标准ID**
   - ID范围: 0x000-0x7FF (11位)

4. **数据长度**
   - CAN数据帧最大支持8字节数据

---

## 典型使用流程

```c
// 1. 初始化CAN
CAN_Init(&hcan1, fifo0, 0, 0);
HAL_Delay(5);

// 2. 配置过滤器
uint16_t ID_ARR[2] = {0x0200, 0x0010};
uint16_t MASK_ARR[2] = {0x0000, 0x0000};
FliterIdCfg_Init(&hcan1, ID_ARR, MASK_ARR, 0, fifo0);

// 3. 发送数据
CAN_TxHeaderTypeDef TxHeader;
TxHeader.StdId = 0x200;
TxHeader.IDE = CAN_ID_STD;
TxHeader.RTR = CAN_RTR_DATA;
TxHeader.DLC = 8;

uint8_t data[8] = {0};
CAN_SendData(&hcan1, &TxHeader, data);

// 4. 接收数据(在中断回调中处理)
void CAN_FIFO_CBKHANDLER(uint32_t fifo_num, uint8_t FIFOmessageNum) {
    // 用户处理接收到的数据
}
```

---

## 相关模块

- **CanMotor**: 电机管理层，使用本模块进行CAN通信
- **DM_Motor**: 达妙电机控制，通过本模块发送控制指令
- **RM_Motor**: 大疆电机控制，通过本模块发送控制指令

---

## 更新日志

- 默认基于标准帧和标准ID
- 支持FIFO0和FIFO1双FIFO接收
- 提供灵活的过滤器配置接口
