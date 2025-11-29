# BSP_DWT 使用说明文档

## 文件信息
- **头文件**: `Bsp/inc/bsp_dwt.h`
- **源文件**: `Bsp/src/bsp_dwt.c`
- **作者**: Wang Hongxi
- **版本**: V1.1.0
- **作用**: DWT(数据观察点与跟踪)高精度计时模块

---

## 功能概述

DWT(Data Watchpoint and Trace)是ARM Cortex-M内核的调试组件，本模块利用其CYCCNT寄存器实现高精度的时间测量和延时功能。

### 核心功能
1. **高精度延时** - 微秒(μs)、毫秒(ms)、秒(s)级延时
2. **时间测量** - 计算代码执行时间
3. **系统时间** - 获取系统运行总时长
4. **时间差计算** - 计算两次调用之间的时间间隔

---

## 数据结构

### DWT_clock_t 系统时间结构体
```c
typedef struct {
    uint32_t s;   // 秒
    uint16_t ms;  // 毫秒
    uint16_t us;  // 微秒
} DWT_clock_t;

extern DWT_clock_t system_time;  // 全局系统时间变量
```

---

## 核心API函数

### 1. DWT_Init()
**功能**: 初始化DWT计时器
```c
void DWT_Init(uint32_t CPU_Freq_mHz);
```

**参数说明**:
- `CPU_Freq_mHz`: CPU频率(单位: MHz)，例如168表示168MHz

**使用示例**:
```c
DWT_Init(168);  // 初始化DWT，CPU频率为168MHz
```

**注意事项**:
- 必须在使用其他DWT功能前调用
- 通常在系统初始化时调用一次即可

---

### 2. 延时函数

#### DWT_Delay()
**功能**: 秒级精确延时
```c
void DWT_Delay(float Delay);
```

**参数说明**:
- `Delay`: 延时时间(单位: 秒)，支持小数

**使用示例**:
```c
DWT_Delay(0.5);   // 延时0.5秒
DWT_Delay(1.0);   // 延时1秒
```

---

#### DWT_Delay_us()
**功能**: 微秒级精确延时
```c
void DWT_Delay_us(uint32_t us);
```

**参数说明**:
- `us`: 延时时间(单位: 微秒)
- 范围: 0-16777215 μs (约16.7秒 @ 100MHz)

**使用示例**:
```c
DWT_Delay_us(100);   // 延时100微秒
DWT_Delay_us(1000);  // 延时1毫秒
```

**注意事项**:
- 最大延时时间受CPU频率限制
- 100MHz: 最长约16.7秒
- 200MHz: 最长约8.3秒

---

#### DWT_Delay_ms()
**功能**: 毫秒级精确延时
```c
void DWT_Delay_ms(uint32_t ms);
```

**参数说明**:
- `ms`: 延时时间(单位: 毫秒)
- 范围: 0-4294967 ms (约4294秒)

**使用示例**:
```c
DWT_Delay_ms(10);    // 延时10毫秒
DWT_Delay_ms(1000);  // 延时1秒
```

---

### 3. 时间测量函数

#### DWT_GetDeltaT()
**功能**: 获取两次调用之间的时间差(float精度)
```c
float DWT_GetDeltaT(uint32_t *cnt_last);
```

**参数说明**:
- `cnt_last`: 上次计数值指针(函数会自动更新)

**返回值**: 时间差(单位: 秒)

**使用示例**:
```c
uint32_t cnt = 0;
float dt;

// 第一次调用
dt = DWT_GetDeltaT(&cnt);  // dt为0或很小

// 执行某些操作...

// 第二次调用
dt = DWT_GetDeltaT(&cnt);  // dt为两次调用之间的时间差(秒)
```

---

#### DWT_GetDeltaT64()
**功能**: 获取两次调用之间的时间差(double精度)
```c
double DWT_GetDeltaT64(uint32_t *cnt_last);
```

**参数说明**:
- `cnt_last`: 上次计数值指针

**返回值**: 时间差(单位: 秒)，double精度

**使用场景**: 需要更高精度的时间测量

---

### 4. 系统运行时间函数

#### DWT_GetTimeline_s()
**功能**: 获取系统运行总时长(秒)
```c
float DWT_GetTimeline_s(void);
```

**返回值**: 系统运行时间(单位: 秒)

**使用示例**:
```c
float runtime = DWT_GetTimeline_s();
printf("系统已运行: %.2f 秒\n", runtime);
```

---

#### DWT_GetTimeline_ms()
**功能**: 获取系统运行总时长(毫秒)
```c
float DWT_GetTimeline_ms(void);
```

**返回值**: 系统运行时间(单位: 毫秒)

**使用示例**:
```c
float runtime = DWT_GetTimeline_ms();
printf("系统已运行: %.2f 毫秒\n", runtime);
```

---

#### DWT_GetTimeline_us()
**功能**: 获取系统运行总时长(微秒)
```c
uint64_t DWT_GetTimeline_us(void);
```

**返回值**: 系统运行时间(单位: 微秒)

**使用示例**:
```c
uint64_t runtime = DWT_GetTimeline_us();
printf("系统已运行: %llu 微秒\n", runtime);
```

---

## 实用宏定义

### TIME_ELAPSE 宏
**功能**: 自动测量代码块执行时间
```c
#define TIME_ELAPSE(dt, code)
```

**使用示例**:
```c
float execution_time;

TIME_ELAPSE(execution_time, {
    // 要测量的代码
    for(int i = 0; i < 1000; i++) {
        // 某些操作
    }
});

printf("代码执行时间: %.6f 秒\n", execution_time);
```

---

## 典型应用场景

### 场景1: 精确控制延时
```c
// 初始化
DWT_Init(168);

// 精确延时
DWT_Delay_us(100);   // 延时100μs
DWT_Delay_ms(10);    // 延时10ms
```

### 场景2: 测量函数执行时间
```c
uint32_t cnt = 0;
float exec_time;

DWT_GetDeltaT(&cnt);  // 开始计时

// 执行要测量的函数
my_function();

exec_time = DWT_GetDeltaT(&cnt);  // 获取执行时间
printf("函数执行时间: %.6f 秒\n", exec_time);
```

### 场景3: 周期性任务时间控制
```c
uint32_t last_time = 0;
float dt;

while(1) {
    dt = DWT_GetDeltaT(&last_time);
    
    if(dt >= 0.01) {  // 每10ms执行一次
        // 执行周期任务
        periodic_task();
    }
}
```

### 场景4: 获取系统运行时间
```c
float uptime_s = DWT_GetTimeline_s();
printf("系统运行时间: %.2f 秒\n", uptime_s);
```

---

## 实现原理

### DWT CYCCNT寄存器
- **CYCCNT**: 32位递增计数器，每个CPU周期加1
- **优势**: 硬件实现，高精度，不占用定时器资源
- **溢出处理**: 模块自动处理32位溢出，支持长时间运行

### 时间计算公式
```
时间(秒) = CYCCNT计数值 / CPU频率(Hz)
```

例如: CPU频率168MHz，计数1680000次 = 0.01秒(10ms)

---

## 使用注意事项

1. **必须先初始化**
   - 在使用任何DWT功能前必须调用`DWT_Init()`
   - 传入正确的CPU频率值

2. **延时精度**
   - DWT延时是CPU周期级精度
   - 比HAL_Delay()更精确
   - 不受中断影响(除非中断时间过长)

3. **计数器溢出**
   - 模块已处理32位计数器溢出问题
   - 支持长时间连续运行

4. **中断影响**
   - 如果中断处理时间过长，可能影响延时精度
   - 建议在关键延时期间关闭非必要中断

5. **资源占用**
   - 不占用硬件定时器
   - 仅使用DWT调试组件

---

## 性能对比

| 延时方式 | 精度 | 最小延时 | 资源占用 | 备注 |
|---------|------|---------|---------|------|
| HAL_Delay() | 毫秒级 | 1ms | SysTick定时器 | 系统延时 |
| DWT_Delay_us() | 微秒级 | 1μs | DWT组件 | 高精度延时 |
| DWT_Delay_ms() | 微秒级 | 1ms | DWT组件 | 高精度延时 |

---

## 相关模块

- **CanMotor**: 电机控制中可能需要精确的时间测量
- **DM_Motor**: 达妙电机发送间隔使用DWT延时
- **所有需要精确计时的模块**

---

## 更新日志

- V1.1.0: 完善了延时函数，增加了微秒和毫秒延时
- 支持float和double精度的时间差计算
- 自动处理计数器溢出
