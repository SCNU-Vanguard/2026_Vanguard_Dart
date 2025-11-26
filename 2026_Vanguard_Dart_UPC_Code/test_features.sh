#!/bin/bash
# -*- coding: utf-8 -*-
#
# 上位机助手快速测试脚本
# 用于验证所有新功能是否正常工作
#

echo "========================================="
echo "   上位机助手 v3.1 功能测试"
echo "========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查依赖
echo "1. 检查Python依赖..."
python3 -c "import PyQt5; import serial; import pyqtgraph" 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ 所有依赖已安装${NC}"
else
    echo -e "${RED}✗ 缺少依赖，请运行: pip install -r requirements.txt${NC}"
    exit 1
fi

# 检查代码语法
echo ""
echo "2. 检查代码语法..."
python3 -m py_compile SourceCode/main_app.py 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ main_app.py 语法正确${NC}"
else
    echo -e "${RED}✗ main_app.py 存在语法错误${NC}"
    exit 1
fi

python3 -m py_compile SourceCode/communication/uart_comm.py 2>/dev/null
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ uart_comm.py 语法正确${NC}"
else
    echo -e "${RED}✗ uart_comm.py 存在语法错误${NC}"
    exit 1
fi

# 检查测试脚本
echo ""
echo "3. 检查测试工具..."
if [ -f "tests/test_plot_simulator.py" ]; then
    echo -e "${GREEN}✓ 绘图数据模拟器存在${NC}"
    chmod +x tests/test_plot_simulator.py
else
    echo -e "${YELLOW}⚠ 测试工具不存在${NC}"
fi

# 检查配置目录
echo ""
echo "4. 检查配置目录..."
CONFIG_DIR="$HOME/.config/uart_host"
if [ -d "$CONFIG_DIR" ]; then
    echo -e "${GREEN}✓ 配置目录已存在: $CONFIG_DIR${NC}"
    if [ -f "$CONFIG_DIR/config.json" ]; then
        echo -e "${GREEN}  └─ 找到配置文件${NC}"
    fi
else
    echo -e "${YELLOW}⚠ 配置目录不存在（首次运行时会自动创建）${NC}"
fi

# 检查串口权限
echo ""
echo "5. 检查串口权限..."
if groups | grep -q dialout; then
    echo -e "${GREEN}✓ 用户在 dialout 组中${NC}"
else
    echo -e "${YELLOW}⚠ 用户不在 dialout 组中${NC}"
    echo -e "${YELLOW}  运行以下命令添加权限（需要重新登录）:${NC}"
    echo -e "  sudo usermod -a -G dialout \$USER"
fi

# 列出可用串口
echo ""
echo "6. 扫描可用串口..."
PORTS=$(ls /dev/tty{USB,ACM,THS}* 2>/dev/null)
if [ -n "$PORTS" ]; then
    echo -e "${GREEN}✓ 找到以下串口:${NC}"
    for port in $PORTS; do
        echo "  - $port"
    done
else
    echo -e "${YELLOW}⚠ 未找到常见串口设备${NC}"
    echo -e "${YELLOW}  (USB串口需要插入设备后才能检测到)${NC}"
fi

# 功能清单
echo ""
echo "========================================="
echo "   v3.1 新功能清单"
echo "========================================="
echo ""
echo "✨ 1. 串口自动扫描和刷新"
echo "   - 启动时自动检测可用串口"
echo "   - 下拉框显示设备名称和描述"
echo "   - 一键刷新按钮"
echo ""
echo "✨ 2. 智能日志系统"
echo "   - 自动时间戳 (HH:MM:SS)"
echo "   - 颜色分类显示"
echo "   - 自动滚动到最新"
echo ""
echo "✨ 3. 实时状态栏"
echo "   - 连接状态显示"
echo "   - 数据接收统计"
echo "   - 通道数量统计"
echo ""
echo "✨ 4. 波形显示控制"
echo "   - ⏸/▶ 暂停/继续绘图"
echo "   - 📐 自动缩放"
echo "   - 🗑 清空波形"
echo "   - 💾 导出数据"
echo ""
echo "✨ 5. 配置持久化"
echo "   - 自动保存配置"
echo "   - 下次启动自动加载"
echo ""
echo "========================================="
echo ""

# 测试方案
echo "📋 测试方案建议："
echo ""
echo "方案1: 使用虚拟串口测试"
echo "  1. 安装 socat: sudo apt install socat"
echo "  2. 创建虚拟串口对:"
echo "     socat -d -d pty,raw,echo=0 pty,raw,echo=0"
echo "  3. 在主程序连接第一个虚拟串口"
echo "  4. 运行测试脚本:"
echo "     python tests/test_plot_simulator.py <第二个虚拟串口>"
echo ""
echo "方案2: 使用真实硬件测试"
echo "  1. 连接Arduino/STM32等设备"
echo "  2. 在设备上发送格式化数据:"
echo "     AT+DRAW+Temperature+25.5"
echo "  3. 观察波形显示"
echo ""

# 启动程序
echo "========================================="
echo ""
read -p "是否启动上位机程序? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "正在启动上位机程序..."
    echo ""
    cd SourceCode
    python3 main_app.py
else
    echo ""
    echo "测试完成！"
    echo "手动启动命令: python3 SourceCode/main_app.py"
    echo ""
fi
