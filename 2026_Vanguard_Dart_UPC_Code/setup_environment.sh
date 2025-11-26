#!/bin/bash
# 上位机助手 - 环境配置脚本
# Host Computer Assistant - Environment Setup Script
# 支持系统级安装和Miniconda虚拟环境

set -e  # 遇到错误立即退出

echo "=========================================="
echo "  上位机助手环境配置工具"
echo "  Host Computer Assistant Setup"
echo "=========================================="
echo ""

# 检测是否使用Conda环境
if [ -n "$CONDA_DEFAULT_ENV" ]; then
    echo "✓ 检测到Conda环境: $CONDA_DEFAULT_ENV"
    USE_CONDA=true
else
    echo "使用系统Python环境"
    USE_CONDA=false
fi

echo ""
echo "步骤 1: 更新系统包..."
sudo apt update
sudo apt upgrade -y

echo ""
echo "步骤 2: 安装系统依赖..."
sudo apt install -y python3 python3-pip build-essential

echo ""
echo "步骤 3: 安装PyQt5..."
if [ "$USE_CONDA" = true ]; then
    echo "  使用Conda安装PyQt5..."
    conda install -y pyqt
else
    echo "  使用apt安装PyQt5..."
    sudo apt install -y python3-pyqt5 python3-pyqt5.qtsvg
fi

echo ""
echo "步骤 4: 安装Python依赖..."
if [ "$USE_CONDA" = true ]; then
    pip install -r requirements.txt
else
    pip3 install -r requirements.txt
fi

echo ""
echo "步骤 5: 配置串口权限..."
# 将当前用户添加到dialout组，以便访问串口
if ! groups $USER | grep -q dialout; then
    echo "  添加用户 $USER 到 dialout 组..."
    sudo usermod -a -G dialout $USER
    echo "  ⚠️  需要重新登录才能生效！"
else
    echo "  ✓ 用户已在 dialout 组中"
fi

echo ""
echo "=========================================="
echo "  环境配置完成！"
echo "=========================================="
echo ""
echo "运行程序："
if [ "$USE_CONDA" = true ]; then
    echo "  python SourceCode/main_app.py"
else
    echo "  python3 SourceCode/main_app.py"
fi
echo ""
