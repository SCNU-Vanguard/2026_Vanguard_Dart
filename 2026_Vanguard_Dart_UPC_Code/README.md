# 上位机助手 (Host Computer Assistant)

**版本 v1.0.01 - UI 美化增强版**

---

## 📝 项目简介

本项目是一个基于 **Ubuntu 22.04** 和 **PyQt5** 开发的上位机助手，使用 **Python 3.10+**。专注于 **UART 串口通信**，支持 USB 虚拟串口和实体硬件串口（如 Jetson Nano 的/dev/ttyTHS1），用于与下位机（如 STM32、Arduino 等）进行通信、实时调节 PID 参数、多通道波形显示。

本项目原为华南师范大学 Vanguard 战队飞镖机器人上位机研发项目。

---

## ✨ 特性

### 核心功能

- **现代 UI 设计**: 采用黑色背景与粉色点缀的时尚界面风格
- **专注 UART 通信**: 精简的 UART 串口通信方案
  - 支持 USB 转 UART 虚拟串口（/dev/ttyUSB0, /dev/ttyACM0 等）
  - 支持实体 UART 硬件串口（Jetson Nano: /dev/ttyTHS1, Raspberry Pi: /dev/ttyAMA0 等）
  - **自动串口扫描和刷新** - 无需手动输入
- **模块化设计**: 通信功能封装在独立模块中，易于维护和扩展
- **实时 PID 调节**: 提供专门界面用于动态调整下位机的 PID 参数
- **手动控制**: 支持手动指令发送和 Hex 格式

### 🆕 v1.0.01 功能

- **智能日志系统**

  - 自动时间戳标记
  - 颜色分类显示（错误/成功/发送/接收）
  - 自动滚动到最新消息

- **多通道实时波形显示**

  - 支持同时显示多条曲线
  - 8 种预设颜色自动区分
  - 实时图例显示
  - 暂停/继续绘图控制
  - 自动缩放功能
  - 数据缓冲管理（默认 1000 点）

- **状态栏信息**

  - 实时连接状态显示
  - 数据接收统计
  - 绘图点数和通道数统计

- **配置持久化**

  - 自动保存上次使用的串口和波特率
  - 下次启动自动加载配置

- **数据导出**: 支持多通道波形数据导出为 CSV 格式

- **字体全面加粗**

  - 全局字体权重提升至 600 (Semi-Bold)
  - 所有文字更清晰醒目
  - 特别优化低 DPI 显示器显示效果

- **圆角美化**

  - 所有组件圆角增大 30-67%
  - 分组框: 10px → 15px
  - 输入框/按钮: 5px → 8px
  - 标签页: 8px → 10px
  - 主窗口: 0px → 12px

- **边框增强**

  - 所有边框加粗至 2px (原来 1px)
  - 组件边界更清晰
  - 视觉层次更分明

- **内边距优化**

  - 所有组件内边距增加 20-50%
  - 界面更宽松，呼吸感更好
  - 分组框增加 15px padding

- **新增样式**

  - 工具按钮圆角和悬停效果
  - 分组框标题增加背景和圆角
  - 复选框尺寸增大 20%
  - 滚动条加粗更易拖动

- **代码规范**: 遵循 PEP8 代码规范，确保代码高质量和可读性

---

## 🛠️ 环境要求

### 系统要求

- **操作系统**: Ubuntu 22.04 LTS（推荐）或其他 Linux 发行版
- **Python 版本**: Python 3.10 或更高
- **硬件**: 支持 USB 串口或板载硬件串口（如 Jetson Nano）

### 依赖包

- PyQt5 >= 5.15.0
- pyserial >= 3.5

---

## 📦 安装方法

### 方法一：使用 Miniconda 虚拟环境（推荐）

#### 1. 安装 Miniconda

如果还没有安装 Miniconda，请先安装：

```bash
# 下载Miniconda安装脚本
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh

# 运行安装脚本
bash Miniconda3-latest-Linux-x86_64.sh

# 按照提示完成安装，重启终端使其生效
```

#### 2. 创建并激活虚拟环境

```bash
# 创建名为uart_host的Python 3.10环境
conda create -n uart_host python=3.10 -y

# 激活环境
conda activate uart_host
```

#### 3. 安装依赖

```bash
# 进入项目目录
cd /path/to/LinuxUPC

# 安装系统依赖（仅首次需要）
sudo apt update
sudo apt install -y build-essential

# 使用Conda安装PyQt5
conda install -y pyqt

# 安装其他Python依赖
pip install -r requirements.txt
```

#### 4. 配置串口权限

```bash
# 将当前用户添加到dialout组（仅首次需要）
sudo usermod -a -G dialout $USER

# ⚠️ 需要重新登录或重启才能生效
```

#### 5. 运行程序

```bash
# 确保在uart_host环境中
conda activate uart_host

# 运行主程序
python SourceCode/main_app.py
```

---

### 方法二：系统级安装

#### 1. 更新系统包

```bash
sudo apt update && sudo apt upgrade -y
```

#### 2. 安装 Python 和系统依赖

```bash
# 确认Python版本
python3 --version

# 安装依赖
sudo apt install -y python3 python3-pip build-essential
sudo apt install -y python3-pyqt5 python3-pyqt5.qtsvg
```

#### 3. 安装 Python 包

```bash
pip3 install -r requirements.txt
```

#### 4. 配置串口权限

```bash
sudo usermod -a -G dialout $USER
# ⚠️ 需要重新登录或重启
```

#### 5. 运行程序

```bash
python3 SourceCode/main_app.py
```

---

### 方法三：使用自动化脚本

```bash
# 赋予执行权限
chmod +x setup_environment.sh

# 运行配置脚本（自动检测Conda环境）
./setup_environment.sh
```

---

## 🚀 快速开始

### 1. 识别串口设备

```bash
# 列出所有串口设备
ls /dev/tty*

# 查看USB串口（插拔USB观察变化）
ls /dev/ttyUSB*
ls /dev/ttyACM*

# Jetson Nano硬件串口
ls /dev/ttyTHS*
```

### 2. 测试串口连接

```bash
# 查看串口权限
ls -l /dev/ttyUSB0

# 应该看到类似：crw-rw---- 1 root dialout ...
# 确保你的用户在dialout组中
groups
```

### 3. 启动程序

```bash
# 使用Conda环境
conda activate uart_host
python SourceCode/main_app.py

# 或系统Python
python3 SourceCode/main_app.py
```

### 4. 连接设备

1. 在"UART 通信设置"区域输入串口端口（或使用快速选择按钮）
2. 选择波特率（默认 115200）
3. 点击"连接"按钮
4. 在日志区域查看连接状态

---

## 📂 项目结构

```
LinuxUPC/
├── SourceCode/
│   ├── main_app.py              # 主程序入口
│   └── communication/           # 通信模块
│       ├── __init__.py          # 模块初始化
│       └── uart_comm.py         # UART通信实现
├── Weekly/                      # 开发周报
├── requirements.txt             # Python依赖列表
├── setup_environment.sh         # 自动配置脚本
├── README.md                    # 本文件
└── LICENSE                      # 开源协议
```

---

## 🔧 通信协议

### PID 参数发送格式

```
PID,<Kp>,<Ki>,<Kd>
示例: PID,1.0000,0.1000,0.0100
```

### 手动控制指令格式

```
<COMMAND>,<PARAM1>,<PARAM2>,...
示例: MOTOR,1,255
```

### 绘图数据接收格式

```
AT+DRAW+<数据名称>+<数值>
示例:
  AT+DRAW+Temperature+25.5
  AT+DRAW+Voltage+3.3
  AT+DRAW+Humidity+65.2
```

详细绘图功能说明请参考：[docs/绘图功能说明.md](docs/绘图功能说明.md)

---

## 🎨 UI 优化

### 1. 串口管理

- 🔄 **自动扫描**: 启动时自动检测可用串口
- 🔍 **智能显示**: 显示串口设备名称和描述
- ⚡ **快速选择**: USB0/THS1/ACM0 快捷按钮

### 2. 日志增强

- ⏰ **时间戳**: 每条日志自动添加时间（HH:MM:SS 格式）
- 🎨 **颜色编码**:
  - 🔴 红色：错误和失败
  - 🟢 绿色：成功和连接
  - 🔵 蓝色：绘图数据
  - 🟡 黄色：发送命令
  - ⚪ 灰色：一般信息

### 3. 波形控制

- ⏸/▶ **暂停/继续**: 暂停数据绘制但保持接收
- 📐 **自动缩放**: 一键调整视图适应所有数据
- 🗑 **清空波形**: 清除所有曲线和数据
- 💾 **导出数据**: CSV 格式导出所有通道

### 4. 状态栏

- 📊 **实时统计**:
  - 连接状态和串口信息
  - 接收数据包总数
  - 绘图点总数
  - 活动通道数量

---

## 🎯 常见问题

### 1. 串口权限问题

**错误**: `Permission denied: '/dev/ttyUSB0'`

**解决**:

```bash
sudo usermod -a -G dialout $USER
# 重新登录或重启
```

### 2. 串口被占用

**错误**: `Serial port is already open`

**解决**:

```bash
# 查找占用进程
lsof /dev/ttyUSB0

# 结束占用进程
kill -9 <PID>
```

### 3. Jetson Nano 串口配置

对于 Jetson Nano 的硬件串口/dev/ttyTHS1，需要禁用控制台：

```bash
# 禁用串口控制台
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
```

### 4. PyQt5 安装问题

如果使用 pip 安装 PyQt5 失败，建议：

**Conda 环境**:

```bash
conda install -y pyqt
```

**系统环境**:

```bash
sudo apt install python3-pyqt5
```

---

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

---

## 📄 开源协议

本项目采用 MIT License，详见 [LICENSE](LICENSE) 文件。

---

## 👨‍💻 作者

**BaleDeng** - 华南师范大学 Vanguard 战队

项目开发日期: 2025-09

当前版本: v1.0.01

---

## 📋 更新日志

### v1.0.00 (2025-09-20) -> v1.0/01 (2025-11-18) - UART 专版

- 🎨 全局字体加粗至 600 (Semi-Bold)
- 🎨 所有圆角增大 30-67%
- 🎨 所有边框加粗至 2px
- 🎨 所有内边距增加 20-50%
- 🎨 新增工具按钮样式
- 🎨 分组框标题增加背景和圆角
- 🎨 复选框尺寸增大 20%
- 🎨 滚动条加粗更易拖动
- 📝 新增 UI 美化对比文档

- ✨ 新增自动串口扫描和刷新功能
- ✨ 新增带时间戳和颜色的智能日志系统
- ✨ 新增实时状态栏和统计信息
- ✨ 新增配置文件自动保存/加载
- ✨ 新增多通道波形显示功能
- ✨ 新增绘图暂停/继续控制
- ✨ 新增自动缩放功能
- 🎨 优化 UI 布局和窗口大小
- 🐛 修复断开连接时的潜在 bug
- 📝 完善文档和测试工具

- 🎉 初始版本发布
- ✨ UART 串口通信基础功能
- ✨ PID 参数调节
- ✨ 手动控制指令发送

---

## 📮 联系方式

如有问题或建议，请通过 GitHub Issues 联系。

---

# 飞镖上位机技术展望与开发方案

## 项目背景

基于当前战队采用拉簧式发射结构+无控飞镖，上位机与视觉暂未合体情况下，参考下面的开源工程以及报告总结。

目标功能：
- 具备场上参数修正功能
- 使用基于树莓派 5/香橙派 5Plus/Jetson Nano 等的可视化 GUI 进行调节 PID 等参数（测试情况下）
- 模拟比赛调节参数过程（梯度飞镖）
- 使用多次校准，从而确定随机固定靶的位置

---

## 飞镖相关开源项目参考

### 西北工业大学报告 (WMJ)

**链接**: <https://bbs.robomaster.com/article/809863?source=4>

**硬件部分**:
- 采用树莓派 Zero2W 平台
- 运行 RaspberryOS
- 使用 OpenCV 进行图像识别和处理
- 通过 CSI 相机数据流识别引导灯
- 串口通信基于 wiringpi 库，使用 CRC16 校验

**下位机**:
- STM32 使用 HAL 库裸机开发
- 板载 BMI088 + IST8310
- 采用 Mahony 算法进行四元数解算融合九轴 IMU

**镖架上位机**:
- 基于 Intel NUC10 平台（Ubuntu 20.04）
- 使用 OpenCV + GalaxyAPI（大恒工业相机）
- CAN 通信

---

### 四川大学 火锅战队 - 全站仪上位机开源

**链接**: 
- <https://bbs.robomaster.com/article/803749?source=4>
- <https://github.com/KangweiYang/dartRack_upper>

**技术方案**:
- 使用全站仪
- 软件以 Qt (C++) 为基础
- 集成串口界面进行标定数据通信
- 通过串口调节界面调节参数

---

### 西安交通大学 笃行战队

**链接**: <https://bbs.robomaster.com/article/803714?source=4>

**特点**:
- 基于皮筋发射系统
- 未开源上位机实现方案

---

### 西南大学 GKD 战队 (RM2021)

**链接**: 
- <https://bbs.robomaster.com/article/8355>
- <https://gitee.com/gong-yuzhi/rmfashejia> - 发射架结构
- <https://gitee.com/rogue_shadowdancer/gkdfeibiao> - 飞镖结构

**技术方案**:
- 皮筋发射结构
- 使用 nRF24L01 模块作为远程调参接收的载体
- 上位机采用 PyQt5 设计
- 理论最远 10 米进行调参
- 使用第三方串口软件（类似 VOFA、SerialPlot）进行可视化监控
- 支持波形导出为 .csv 文件，便于 Excel/Matlab 分析

---

### 西南交通大学 Helios 战队 - 制导飞镖

**链接**: 
- <https://bbs.robomaster.com/article/715957?source=4>
- <https://github.com/kid-king-x/DartGuide2025-Helios>

**技术方案**:
- RV1126 作为上位机
- 未提及上位机 GUI 方案

---

### 华南理工大学 华南虎 (RM2023)

**链接**: <https://bbs.robomaster.com/article/xxx>

**技术方案**:
- 使用 ESP32 与 STM32 系列控制板进行通信
- 基于 ESP32 开发上位机 HMI（人机界面）
- 应该使用 LVGL 实现
- 离线调参功能

---

### 大连理工大学 凌 Bug (2024)

**技术方案**:
- 上位机基于 Node.js 和 Electron
- 使用 JavaScript, HTML 和 CSS 编写
- 实现功能：
  - 机器人调试
  - 通信协议管理
  - 代码工程管理
- 通讯数据可视化，绘制成曲线或保存在本地
- 有控飞镖的视觉部分交由 STM32H7 系列主控实现

**机械要求**:
- 需要机械提供精准参数
- 拉簧选型
- 飞镖细节问题调整：
  - 飞镖内部重量分配
  - 缓冲位置精确控制
  - 滑台震动控制
  - 无控飞镖精确度与一致性
  - 不同场地不同方位时的飞镖校准

---

### 南航 (2022)

**技术方案**:
- 未使用上位机（无控飞镖）

---

## 飞镖上位机方案实现

### 核心需求

对于飞镖上位机，现在的要求非常明确：

1. **最紧迫需求**：打通视觉与下位机的通讯
2. **硬件平台**：Jetson 系列作为当前主控（性能最强劲方案）
3. **功能要求**：
   - 保证飞镖的瞄准
   - 处理视觉数据并解算发送给下位机
   - 下位机电控数据、参数上传上位机
   - 提供反馈曲线，方便调试人员数据调试总结

### 技术方案选型

目前尝试了两个方向：

#### 方案一：基于 C/C++ 的 LVGL 库

**优点**:
- CPU 资源占用较低
- 固定单核运行执行文件

**缺点**:
- 代码繁琐
- 使用 SquareLine Studio 开发（收费，可白嫖 30 天）

---

#### 方案二：基于 C++/Python 的 Qt 库（当前采用）

**优点**:
- 使用更便捷的 PyQt5/6 开发
- Qt Designer 免费
- 大部分开源工程选择 Qt
- 便于后期维护

**缺点**:
- 多核调度
- 资源占用稍高（但与 LVGL 差不多）

**决策**:
为了后期维护以及参考大部分开源工程，当前采用 Qt 进行开发。

---

### 开发进展

**已完成**:
- ✅ Qt 开发环境配置和编写
- ✅ 基础 UI 界面设计
- ✅ UART 串口通信模块
- ✅ PID 参数调节界面
- ✅ 实时波形显示功能
- ✅ 数据导出功能

**待完成**:
- ⏳ 多线程部分（需为视觉接口预留）
- ⏳ 各种数据流输入接口确定
- ⏳ 回调函数优化
- ⏳ 信号与槽的处理完善
- ⏳ 与下位机通信测试
- ⏳ 与视觉的数据对接

---

### 通信方案

下位机通信协议可选方案：
- ✅ UART 串口通信（当前采用）
- ⏳ CAN 通信
- ⏳ RS485 通信
- ⏳ I2C 通信

目前还未完全测试与下位机通信，可以先测试各种通信方式。

**视觉方面**：
- 仍然待定，需要继续与视觉进行沟通
- 需确定视觉数据包格式

---

## 总结

上位机的 GUI 以及通信部分都亟待处理与沟通：

1. **GUI 需求**：
   - 不仅仅是参数调节的加减
   - 需要反馈曲线显示
   - 需要数据记录和导出

2. **通信需求**：
   - 调用对应库进行编写逻辑嵌入上位机即可
   - 需要明确通信协议
   - 需要与视觉、下位机进行联调

3. **未来方向**：
   - 当前方案仍建议为无控飞镖
   - 为后续有控飞镖做技术基础方面的参照
   - 基于技术问题，循序渐进

---

**文档创建时间**: 2025-09-21  
**最后更新**: 2025-12-05  
**维护者**: BaleDeng & Vanguard Team
