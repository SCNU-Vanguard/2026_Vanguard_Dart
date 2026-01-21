# BSP_CAN 使用说明文档

## 文件信息
- **头文件**: `Bsp/inc/bsp_can.h`
- **源文件**: `Bsp/src/bsp_can.c`
- **作用**: CAN总线通信的底层硬件抽象层(BSP)

---

## 核心API函数 (2026-01-17 更新)

### 1. CAN_Init()
初始化CAN外设及基础过滤器。

### 2. CAN_SendData()
发送8字节CAN数据帧。

### 3. FilterIdCfg_Init()
**注意**: 已统一修正拼写（原为 FliterIdCfg_Init）。
配置CAN过滤器ID掩码模式。

---

## 接收机制

本工程使用中断接收模式。底层 `stm32f4xx_it.c` 调用 `HAL_CAN_RxFifoXMsgPendingCallback`，随后分发至 `CanMotor.c` 中的 `CAN_FIFO_CBKHANDLER`。

---

## 更新日志
- **2026-01-17**: 修正全局 `Filter` 拼写错误，优化了过滤器配置接口的命名一致性。