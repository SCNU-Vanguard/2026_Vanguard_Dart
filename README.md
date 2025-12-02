# 2026 Vanguard Dart Project

## 项目简介 / Project Overview

本仓库为华南师范大学 Vanguard 战队 2026 赛季飞镖系统的综合仓库，包含 STM32 控制系统、视觉处理系统、飞镖系统机械结构和上位机图形化界面。

This repository contains the comprehensive codebase for SCNU Vanguard's 2026 season Dart system, including STM32 control system, vision processing system, dart's structure and upper computer GUI.

## 项目结构 / Project Structure

```
2026_Vanguard_Dart/
├── 2026_Vanguard_Dart_STM32_Ctrl/    # STM32控制系统 (C/C++)
│   ├── Dart_Ctrl/                     # STM32主控代码（A板）
│   └── 相关手册/                      # 相关技术文档
│
├── 2026_Vanguard_Dart_JestonPart/    # Jetson视觉处理系统 (Python/C++)
│   └── README.md                      # 视觉系统文档
│
└── 2026_Vanguard_Dart_UPC_Code/      # 上位机图形化界面 (Python)
|   ├── SourceCode/                    # 源代码
|   ├── docs/                          # 文档
|   ├── tests/                         # 测试代码
|   └── Weekly/                        # 周报记录
└── 2026_Vangaurd_Dart_Structure
    └──飞镖(V2improved)                #飞镖系统机械机构
```

## 功能模块 / Function Modules

### 1. STM32 控制系统 (STM32 Control System)

- **平台**: STM32F427xx（A 板）
- **开发环境**: Keil MDK-ARM
- **功能**:
  - 飞镖发射控制
  - CAN 总线通信
  - 电机控制 (大疆/达妙电机)
  - UART 通信
  - FreeRTOS 实时操作系统

### 2. 视觉处理系统 (Vision Processing System)

- **平台**: NVIDIA Jetson
- **功能**:
  - 目标识别与追踪
  - 图像处理
  - 视觉算法

### 3. 上位机系统 (Upper Computer System)

- **语言**: Python
- **功能**:
  - 实时数据监控
  - 参数调试界面
  - 数据可视化
  - UART 串口通信

## 开发环境 / Development Environment

### STM32 开发

- **IDE**: Keil MDK-ARM / STM32CubeIDE
- **调试器**: ST-Link
- **工具链**: ARM GCC

### 视觉处理

- **系统**: Ubuntu (Jetson)
- **语言**: Python 3.x / C++
- **依赖**: OpenCV, NumPy 等

### 上位机开发

- **语言**: Python 3.x
- **主要库**: PyQt5/PySide, pyserial, matplotlib 等
- **详见**: `2026_Vanguard_Dart_UPC_Code/requirements.txt`

## 快速开始 / Quick Start

### STM32 部分

```bash
cd 2026_Vanguard_Dart_STM32_Ctrl/Dart_Ctrl
# 使用Keil打开 Dart_Ctrl.uvprojx 工程文件
```

### 视觉处理部分

```bash
cd 2026_Vanguard_Dart_JestonPart
# 查看README.md了解详细说明
```

### 上位机部分

```bash
cd 2026_Vanguard_Dart_UPC_Code
# 安装依赖
pip install -r requirements.txt
# 运行主程序
python SourceCode/main_app.py
```

## 技术栈 / Tech Stack

- **嵌入式**: C/C++, FreeRTOS, STM32 HAL
- **视觉**: Python/C++, OpenCV
- **上位机**: Python, PyQt5, Matplotlib
- **通信**: UART, CAN
- **版本控制**: Git

## 贡献指南 / Contributing

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 提交 Pull Request

## 团队 / Team

华南师范大学 Vanguard 战队  
SCNU Vanguard Team

## 许可证 / License

本项目仅供华南师范大学 Vanguard 战队内部学习和比赛使用。

This project is for internal learning and competition use of SCNU Vanguard Team only.

---

_Last Updated: 2025 年 12 月_
