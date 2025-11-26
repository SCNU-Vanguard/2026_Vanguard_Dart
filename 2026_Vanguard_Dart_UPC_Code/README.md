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
