# -*- coding: utf-8 -*-
"""
Host Computer Assistant Main Application
作者: BaleDeng
日期: 2025-11-218
版本: 1.0.01 (UI美化版)
note: 我操你妈，这傻比玩意要求真多，还报线程冲突错误
"""
import sys
import json
from datetime import datetime
from pathlib import Path
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QLineEdit,
                             QComboBox, QGroupBox, QTextEdit, QTabWidget,
                             QGridLayout, QMessageBox, QCheckBox, QFileDialog,
                             QStatusBar, QToolButton, QPlainTextEdit)
from PyQt5.QtCore import QThread, pyqtSignal, QObject, Qt, QTimer
from PyQt5.QtGui import QIcon, QFont, QTextCursor, QColor
import pyqtgraph as pg
from collections import defaultdict
from communication.uart_comm import UARTCommunication

# --- 配置文件路径 ---
CONFIG_FILE = Path.home() / '.config' / 'uart_host' / 'config.json'

# --- 图形化界面皮肤 ---
STYLE_SHEET = """
QWidget {
    background-color: #2C2F33;
    color: #E0E0E0;
    font-family: 'Segoe UI', 'Microsoft YaHei', sans-serif;
    font-size: 14px;
    font-weight: 600;
}
QMainWindow {
    border: 1px solid #4A4A4A;
    border-radius: 12px;
}
QGroupBox {
    border: 2px solid #FF69B4;
    border-radius: 15px;
    margin-top: 15px;
    font-weight: bold;
    background-color: #23272A;
    padding: 15px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top center;
    padding: 5px 15px;
    color: #FF69B4;
    font-size: 16px;
    font-weight: bold;
    background-color: #2C2F33;
    border-radius: 8px;
}
QLabel {
    color: #E0E0E0;
    font-weight: 600;
}
QLineEdit, QTextEdit, QPlainTextEdit, QComboBox {
    background-color: #3A3D40;
    border: 2px solid #4A4A4A;
    border-radius: 8px;
    padding: 8px;
    selection-background-color: #FF69B4;
    selection-color: #FFFFFF;
    font-weight: 600;
}
QLineEdit:focus, QTextEdit:focus, QPlainTextEdit:focus, QComboBox:focus {
    border: 2px solid #FF69B4;
}
QTextEdit, QPlainTextEdit {
    font-family: 'Consolas', 'Courier New', monospace;
    font-weight: 500;
}
QPushButton {
    background-color: qlineargradient(
    x1:0, y1:0, x2:0, y2:1, stop:0 #FF69B4, stop:1 #E05297);
    color: #FFFFFF;
    border: none;
    padding: 10px 18px;
    border-radius: 8px;
    font-weight: bold;
    text-transform: uppercase;
}
QPushButton:hover {
    background-color: qlineargradient(
    x1:0, y1:0, x2:0, y2:1, stop:0 #FF85C1, stop:1 #FF69B4);
}
QPushButton:pressed {
    background-color: #E05297;
    padding-top: 12px;
    padding-bottom: 8px;
}
QPushButton:disabled {
    background-color: #555555;
    color: #888888;
}
QTabWidget::pane {
    border-top: 2px solid #FF69B4;
    border-radius: 8px;
    margin-top: -1px;
}
QTabBar::tab {
    background: #2D2D2D;
    color: #B0B0B0;
    padding: 12px 28px;
    border-top-left-radius: 10px;
    border-top-right-radius: 10px;
    border: 2px solid #4A4A4A;
    border-bottom: none;
    margin-right: 3px;
    font-weight: bold;
}
QTabBar::tab:selected {
    background: #FF69B4;
    color: #FFFFFF;
    font-weight: bold;
}
QScrollBar:vertical {
    border: none;
    background: #2C2F33;
    width: 12px;
    margin: 0px 0px 0px 0px;
}
QScrollBar::handle:vertical {
    background: #FF69B4;
    min-height: 25px;
    border-radius: 6px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}
QScrollBar:horizontal {
    border: none;
    background: #2C2F33;
    height: 12px;
    margin: 0px 0px 0px 0px;
}
QScrollBar::handle:horizontal {
    background: #FF69B4;
    min-width: 25px;
    border-radius: 6px;
}
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {
    width: 0px;
}
QCheckBox {
    font-weight: 600;
    spacing: 8px;
}
QCheckBox::indicator {
    width: 18px;
    height: 18px;
    border: 2px solid #FF69B4;
    border-radius: 5px;
}
QCheckBox::indicator:checked {
    background-color: #FF69B4;
}
QStatusBar {
    background-color: #23272A;
    color: #E0E0E0;
    border-top: 2px solid #FF69B4;
    font-weight: 600;
}
QStatusBar::item {
    border: none;
}
QToolButton {
    background-color: #3A3D40;
    border: 2px solid #4A4A4A;
    border-radius: 6px;
    padding: 6px;
    font-weight: bold;
}
QToolButton:hover {
    background-color: #FF69B4;
    border-color: #FF69B4;
}
QToolButton:pressed {
    background-color: #E05297;
}
QComboBox::drop-down {
    border: none;
    border-left: 2px solid #4A4A4A;
    border-top-right-radius: 6px;
    border-bottom-right-radius: 6px;
    width: 25px;
}
QComboBox::down-arrow {
    image: none;
    border: 2px solid #FF69B4;
    width: 8px;
    height: 8px;
    border-top: none;
    border-right: none;
    transform: rotate(-45deg);
}
"""


class CommunicationWorker(QObject):
    """工作主线程(处理连续数据接受，防止UI冻结)"""
    data_received = pyqtSignal(str)
    plot_data_received = pyqtSignal(str, float)  # (数据名称, 数值)
    error_occurred = pyqtSignal(str)
    _is_running = True

    def __init__(self, comm_instance):
        super().__init__()
        self.comm = comm_instance

    def run(self):
        """持续监听数据"""
        self._is_running = True
        while self._is_running:
            try:
                raw_data = self.comm.receive_data()
                if raw_data:
                    data_str = str(raw_data)
                    self.data_received.emit(data_str)

                    # --- 解析 DRAW 协议数据用于绘图 ---
                    # 协议格式: AT+DRAW+<数据名称>+<数值>
                    # 示例: AT+DRAW+Temperature+25.5
                    #       AT+DRAW+Voltage+3.3
                    parts = data_str.strip().split('+')
                    if len(parts) == 4 and parts[0].upper(
                    ) == 'AT' and parts[1].upper() == 'DRAW':
                        try:
                            data_name = parts[2].strip()
                            data_value = float(parts[3].strip())
                            self.plot_data_received.emit(data_name, data_value)
                        except (ValueError, IndexError) as e:
                            pass

                QThread.msleep(50)
            except Exception as e:
                self.error_occurred.emit(f"Data reception error: {e}")
                break

    def stop(self):
        """停止监听循环"""
        self._is_running = False


class ParameterAdjustmentWindow(QWidget):
    """用于调节其他高级参数的窗口"""
    send_param_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("高级参数调节 (Advanced Parameter Tuning)")
        self.setWindowIcon(QIcon.fromTheme("preferences-system"))
        self.setGeometry(200, 200, 450, 250)
        self.initUI()
        self.setStyleSheet(STYLE_SHEET)

    def initUI(self):
        main_layout = QVBoxLayout(self)
        group = QGroupBox("姿态与距离控制 (Pose & Distance Control)")
        main_layout.addWidget(group)

        layout = QGridLayout(group)
        layout.setSpacing(15)
        layout.setContentsMargins(20, 20, 20, 20)

        layout.addWidget(QLabel("偏航角 (Yaw Angle):"), 0, 0)
        self.yaw_input = QPlainTextEdit()
        self.yaw_input.setPlaceholderText("请输入偏航角 (例如: 0.0)")
        self.yaw_input.setMaximumHeight(60)
        self.yaw_input.setPlainText("0.0")
        layout.addWidget(self.yaw_input, 0, 1)

        layout.addWidget(QLabel("翻滚角 (Roll Angle):"), 1, 0)
        self.roll_input = QPlainTextEdit()
        self.roll_input.setPlaceholderText("请输入翻滚角 (例如: 0.0)")
        self.roll_input.setMaximumHeight(60)
        self.roll_input.setPlainText("0.0")
        layout.addWidget(self.roll_input, 1, 1)

        layout.addWidget(QLabel("直线距离 (Distance):"), 2, 0)
        self.distance_input = QPlainTextEdit()
        self.distance_input.setPlaceholderText("请输入直线距离 (例如: 0.0)")
        self.distance_input.setMaximumHeight(60)
        self.distance_input.setPlainText("0.0")
        layout.addWidget(self.distance_input, 2, 1)

        send_button = QPushButton("发送参数 (Send Parameters)")
        send_button.clicked.connect(self.send_parameters)
        layout.addWidget(send_button, 3, 0, 1, 2)

    def send_parameters(self):
        """用于格式化并发送参数"""
        try:
            yaw = float(self.yaw_input.toPlainText().strip())
            roll = float(self.roll_input.toPlainText().strip())
            dist = float(self.distance_input.toPlainText().strip())
            command = f"PARAMS,{yaw:.2f},{roll:.2f},{dist:.2f}"
            self.send_param_signal.emit(command)
            QMessageBox.information(self, "成功 (Success)",
                                    "参数已发送！ (Parameters sent!)")
            self.close()
        except ValueError:
            QMessageBox.warning(self, "输入错误 (Input Error)",
                                "请输入有效的数值。 (Please enter valid numbers.)")


class MainWindow(QMainWindow):
    """主程序窗口"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("上位机助手v1.0.01")
        self.setWindowIcon(QIcon.fromTheme("utilities-terminal"))
        self.setGeometry(100, 100, 1200, 750)
        self.setMinimumSize(1000, 700)

        self.comm_instance = None
        self.worker_thread = None
        self.comm_worker = None
        self.param_window = None

        # 绘图相关
        self.plot_data_buffer = defaultdict(list)  # {数据名称: [数值列表]}
        self.plot_curves = {}  # {数据名称: PlotDataItem对象}
        self.plot_colors = [
            (255, 105, 180),  # 粉红色
            (64, 224, 208),  # 青绿色
            (255, 215, 0),  # 金色
            (147, 112, 219),  # 紫色
            (255, 127, 80),  # 珊瑚色
            (144, 238, 144),  # 浅绿色
            (255, 182, 193),  # 浅粉色
            (173, 216, 230),  # 浅蓝色
        ]
        self.color_index = 0
        self.max_data_points = 1000  # 每条曲线最多显示的数据点数
        self.plot_paused = False  # 绘图暂停标志

        # 统计信息
        self.total_received = 0  # 接收的数据包总数
        self.total_plot_points = 0  # 绘图点总数

        # 加载配置
        self.load_config()

        self.initUI()
        self.setStyleSheet(STYLE_SHEET)

        # 初始化状态栏
        self.init_statusbar()

        # 定时更新统计信息
        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self.update_statistics)
        self.stats_timer.start(1000)  # 每秒更新一次

    def initUI(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        main_layout.setSpacing(10)

        # --- 左侧面板 ---
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        main_layout.addWidget(left_panel, 1)

        # 通信设置
        conn_group = QGroupBox("UART通信设置")
        conn_layout = QGridLayout(conn_group)

        conn_layout.addWidget(QLabel("串口端口"), 0, 0)

        # 串口选择使用下拉框，并添加刷新按钮
        port_layout = QHBoxLayout()
        self.port_combo = QComboBox()
        self.port_combo.setEditable(True)
        self.port_combo.setPlaceholderText("选择或输入串口...")
        port_layout.addWidget(self.port_combo, 1)

        self.refresh_port_button = QToolButton()
        self.refresh_port_button.setText("🔄")
        self.refresh_port_button.setToolTip("刷新串口列表")
        self.refresh_port_button.clicked.connect(self.refresh_serial_ports)
        port_layout.addWidget(self.refresh_port_button)

        conn_layout.addLayout(port_layout, 0, 1)

        conn_layout.addWidget(QLabel("波特率"), 1, 0)
        self.baud_input = QComboBox()
        self.baud_input.addItems(
            ["9600", "115200", "230400", "460800", "921600"])
        self.baud_input.setCurrentText("115200")
        self.baud_input.setEditable(True)
        conn_layout.addWidget(self.baud_input, 1, 1)

        # 添加常用串口快速选择
        conn_layout.addWidget(QLabel("快速选择 (Quick Select):"), 2, 0)
        quick_select_layout = QHBoxLayout()
        self.usb_button = QPushButton("USB0")
        self.usb_button.clicked.connect(
            lambda: self.port_combo.setCurrentText("/dev/ttyUSB0"))
        self.jetson_button = QPushButton("THS1")
        self.jetson_button.clicked.connect(
            lambda: self.port_combo.setCurrentText("/dev/ttyTHS1"))
        self.acm_button = QPushButton("ACM0")
        self.acm_button.clicked.connect(
            lambda: self.port_combo.setCurrentText("/dev/ttyACM0"))
        quick_select_layout.addWidget(self.usb_button)
        quick_select_layout.addWidget(self.jetson_button)
        quick_select_layout.addWidget(self.acm_button)
        quick_select_layout.addStretch()
        conn_layout.addLayout(quick_select_layout, 2, 1)

        self.connect_button = QPushButton("连接 (Connect)")
        conn_layout.addWidget(self.connect_button, 3, 0, 1, 2)
        left_layout.addWidget(conn_group)

        # 日志
        log_group = QGroupBox("日志 (Log)")
        log_layout = QVBoxLayout(log_group)
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)
        log_layout.addWidget(self.log_display)
        left_layout.addWidget(log_group)

        # --- 右侧面板 ---
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        main_layout.addWidget(right_panel, 2)

        tabs = QTabWidget()
        pid_tab = QWidget()
        waveform_tab = QWidget()  # 波形显示选项卡
        other_params_tab = QWidget()

        tabs.addTab(pid_tab, "电机参数调节")
        tabs.addTab(waveform_tab, "波形显示")
        tabs.addTab(other_params_tab, "高级功能")
        right_layout.addWidget(tabs)

        # PID 参数面板
        pid_tab_layout = QVBoxLayout(pid_tab)
        pid_group = self.create_pid_group()
        manual_control_group = self.create_manual_control_group()
        pid_tab_layout.addWidget(pid_group)
        pid_tab_layout.addWidget(manual_control_group)
        pid_tab_layout.addStretch(1)

        # 波形显示面板
        waveform_layout = QVBoxLayout(waveform_tab)
        plot_group = QGroupBox("实时数据波形 (Real-time Data Waveform)")
        plot_group_layout = QVBoxLayout(plot_group)

        # 创建 pyqtgraph PlotWidget
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#1E1E1E')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel('left', '数值 (Value)')
        self.plot_widget.setLabel('bottom', '采样点 (Sample Point)')
        self.plot_widget.setTitle('多通道实时数据波形', color='#FF69B4', size='14pt')
        self.plot_widget.addLegend(offset=(10, 10))

        # 设置坐标轴颜色
        self.plot_widget.getAxis('left').setPen(
            pg.mkPen(color='#E0E0E0', width=1))
        self.plot_widget.getAxis('bottom').setPen(
            pg.mkPen(color='#E0E0E0', width=1))
        self.plot_widget.getAxis('left').setTextPen(pg.mkPen(color='#E0E0E0'))
        self.plot_widget.getAxis('bottom').setTextPen(
            pg.mkPen(color='#E0E0E0'))

        plot_group_layout.addWidget(self.plot_widget)

        plot_controls_layout = QHBoxLayout()

        self.pause_plot_button = QPushButton("⏸ 暂停绘图")
        self.pause_plot_button.setCheckable(True)
        self.pause_plot_button.clicked.connect(self.toggle_plot_pause)

        self.autoscale_button = QPushButton("📐 自动缩放")
        self.autoscale_button.clicked.connect(self.autoscale_plot)

        self.clear_plot_button = QPushButton("🗑 清空波形")
        self.clear_plot_button.clicked.connect(self.clear_plot)

        self.export_button = QPushButton("💾 导出数据")
        self.export_button.setEnabled(False)

        plot_controls_layout.addWidget(self.pause_plot_button)
        plot_controls_layout.addWidget(self.autoscale_button)
        plot_controls_layout.addWidget(self.clear_plot_button)
        plot_controls_layout.addStretch(1)
        plot_controls_layout.addWidget(self.export_button)
        plot_group_layout.addLayout(plot_controls_layout)

        waveform_layout.addWidget(plot_group)

        # 其他参数面板
        other_params_layout = QVBoxLayout(other_params_tab)
        self.open_param_window_button = QPushButton("打开高级参数调节窗口")
        self.open_param_window_button.setEnabled(False)
        other_params_layout.addWidget(self.open_param_window_button)
        other_params_layout.addStretch(1)

        # --- 信号连接 ---
        self.connect_button.clicked.connect(self.toggle_connection)
        self.send_pid_button.clicked.connect(self.send_pid_data)
        self.open_param_window_button.clicked.connect(self.open_param_window)
        self.export_button.clicked.connect(self.export_waveform_data)
        self.send_manual_button.clicked.connect(self.send_manual_data)

        # 初始加载串口列表（必须在所有UI组件创建完成后调用）
        self.refresh_serial_ports()

    def create_pid_group(self):
        pid_group = QGroupBox("PID 参数 (PID Parameters)")
        pid_layout = QGridLayout(pid_group)

        pid_layout.addWidget(QLabel("KP:"), 0, 0)
        self.kp_input = QLineEdit()
        self.kp_input.setPlaceholderText("请输入数值")
        pid_layout.addWidget(self.kp_input, 0, 1)

        pid_layout.addWidget(QLabel("KI:"), 1, 0)
        self.ki_input = QLineEdit()
        self.ki_input.setPlaceholderText("请输入数值")
        pid_layout.addWidget(self.ki_input, 1, 1)

        pid_layout.addWidget(QLabel("KD:"), 2, 0)
        self.kd_input = QLineEdit()
        self.kd_input.setPlaceholderText("请输入数值")
        pid_layout.addWidget(self.kd_input, 2, 1)

        self.kf_input = QLineEdit()
        self.kf_input.setPlaceholderText("请输入数值")
        pid_layout.addWidget(self.kf_input, 3, 1)
        pid_layout.addWidget(QLabel("Kf:"), 3, 0)

        self.motor_id = QLineEdit()
        self.motor_id.setPlaceholderText("请输入电机ID")
        pid_layout.addWidget(self.motor_id, 4, 1)

        self.send_pid_button = QPushButton("发送 PID 参数 (Send PID)")
        self.send_pid_button.setEnabled(False)
        pid_layout.addWidget(self.send_pid_button, 5, 0, 1, 2)
        return pid_group

    def create_manual_control_group(self):
        manual_group = QGroupBox("手动控制 (Manual Control)")
        layout = QVBoxLayout(manual_group)
        layout.setSpacing(10)

        # 第一行：输入框标签和输入框
        input_layout = QVBoxLayout()
        input_label = QLabel("指令 (Command):")
        input_layout.addWidget(input_label)

        self.manual_cmd_input = QTextEdit()
        self.manual_cmd_input.setPlaceholderText(
            "例如: MOTOR,1,255\n或: AT+DRAW+Temperature+25.5")
        self.manual_cmd_input.setMaximumHeight(100)
        self.manual_cmd_input.setMinimumHeight(60)
        input_layout.addWidget(self.manual_cmd_input)
        layout.addLayout(input_layout)

        # 第二行：发送格式选择
        format_layout = QHBoxLayout()
        format_label = QLabel("发送格式 (Format):")
        format_layout.addWidget(format_label)

        self.format_ascii_radio = QCheckBox("ASCII")
        self.format_ascii_radio.setChecked(True)
        format_layout.addWidget(self.format_ascii_radio)

        self.format_hex_radio = QCheckBox("HEX")
        format_layout.addWidget(self.format_hex_radio)

        self.format_utf8_radio = QCheckBox("UTF-8")
        format_layout.addWidget(self.format_utf8_radio)

        format_layout.addStretch()
        layout.addLayout(format_layout)

        # 第三行：发送按钮（大横条）
        self.send_manual_button = QPushButton("📤 发送指令 (Send Command)")
        self.send_manual_button.setEnabled(False)
        self.send_manual_button.setMinimumHeight(45)
        layout.addWidget(self.send_manual_button)

        return manual_group

    def toggle_connection(self):
        if self.comm_instance and self.comm_instance.is_connected():
            self.disconnect_device()
        else:
            self.connect_device()

    def connect_device(self):
        """连接UART设备"""
        port = self.port_combo.currentText().strip()
        if not port:
            QMessageBox.warning(self, "输入错误", "请选择或输入串口端口")
            return

        try:
            baud = int(self.baud_input.currentText())
            self.comm_instance = UARTCommunication(port, baud)
            self.comm_instance.connect()
            self.log(f"[UART] 连接成功 (Connected successfully) - {port} @ {baud}")

            # 保存配置
            self.save_config()

            # 启动通信工作线程
            self.worker_thread = QThread()
            self.comm_worker = CommunicationWorker(self.comm_instance)
            self.comm_worker.moveToThread(self.worker_thread)
            self.worker_thread.started.connect(self.comm_worker.run)
            self.comm_worker.data_received.connect(self.log)
            self.comm_worker.plot_data_received.connect(self.update_plot)
            self.comm_worker.error_occurred.connect(self.handle_comm_error)
            self.worker_thread.start()

            self.update_ui_state(connected=True)

        except ValueError:
            self.log("波特率错误 (Invalid baud rate)")
            QMessageBox.critical(self, "错误 (Error)", "波特率必须是数字")
        except Exception as e:
            self.log(f"连接错误 (Connection Error): {e}")
            QMessageBox.critical(self, "连接错误", f"无法连接到 {port}:\n{e}")
            self.comm_instance = None

    def disconnect_device(self):
        if self.comm_worker:
            self.comm_worker.stop()
        if self.worker_thread:
            self.worker_thread.quit()
            self.worker_thread.wait()
        if self.comm_instance:
            self.comm_instance.disconnect()

        self.log("连接已断开 (Disconnected)")
        self.comm_instance = None
        self.update_ui_state(connected=False)

    def update_ui_state(self, connected):
        """统一更新UI连接状态"""
        self.connect_button.setText(
            "断开 (Disconnect)" if connected else "连接 (Connect)")
        self.port_combo.setEnabled(not connected)
        self.baud_input.setEnabled(not connected)
        self.usb_button.setEnabled(not connected)
        self.jetson_button.setEnabled(not connected)
        self.acm_button.setEnabled(not connected)
        self.refresh_port_button.setEnabled(not connected)
        self.send_pid_button.setEnabled(connected)
        self.open_param_window_button.setEnabled(connected)
        self.export_button.setEnabled(connected)
        self.send_manual_button.setEnabled(connected)
        if not connected:
            # 断开连接时清空所有绘图数据
            if hasattr(self, 'plot_widget'):  # 确保plot_widget已初始化
                self.clear_plot()

    def send_data(self, data, format_type='ascii'):
        """
        发送数据
        format_type: 'ascii', 'hex', 'utf8'
        """
        if not self.comm_instance or not self.comm_instance.is_connected():
            self.log("未连接，发送失败 (Not connected, send failed)")
            return False
        try:
            if format_type == 'hex':
                # HEX格式：移除空格后转换为字节
                hex_str = data.replace(' ', '').replace('\n', '')
                payload = bytes.fromhex(hex_str)
                log_msg = f"发送 (HEX): {hex_str}"
            elif format_type == 'utf8':
                # UTF-8格式：直接编码为UTF-8，不添加换行符
                payload = data.encode('utf-8')
                log_msg = f"发送 (UTF-8): {data}"
            else:  # ascii
                # ASCII格式：添加换行符后编码
                payload = (data + '\n').encode('ascii', errors='ignore')
                log_msg = f"发送 (ASCII): {data}"

            self.comm_instance.send_data(payload)
            self.log(log_msg)
            return True
        except ValueError as e:
            self.log(f"发送错误 (Send Error) - 数据格式错误: {e}")
            QMessageBox.warning(self, "格式错误", f"数据格式不正确:\n{e}")
            return False
        except Exception as e:
            self.log(f"发送错误 (Send Error): {e}")
            return False

    def send_pid_data(self):
        try:
            kp = float(self.kp_input.text())
            ki = float(self.ki_input.text())
            kd = float(self.kd_input.text())
            command = f"PID,{kp:.4f},{ki:.4f},{kd:.4f}"
            self.send_data(command)
        except ValueError:
            QMessageBox.warning(self, "输入错误", "PID参数必须是有效的数值。")

    def send_manual_data(self):
        """发送手动控制指令"""
        command = self.manual_cmd_input.toPlainText().strip()
        if not command:
            QMessageBox.warning(self, "输入错误", "请输入要发送的指令")
            return

        # 确定发送格式
        if self.format_hex_radio.isChecked():
            format_type = 'hex'
        elif self.format_utf8_radio.isChecked():
            format_type = 'utf8'
        else:  # ASCII (默认)
            format_type = 'ascii'

        # 发送数据
        if self.send_data(command, format_type=format_type):
            # 发送成功后可选择清空输入框（可注释掉这行保留输入内容）
            # self.manual_cmd_input.clear()
            pass

    def send_custom_data(self, data):
        self.send_data(data)

    def open_param_window(self):
        if self.param_window is None:
            self.param_window = ParameterAdjustmentWindow()
            self.param_window.send_param_signal.connect(self.send_custom_data)
        self.param_window.show()
        self.param_window.activateWindow()

    def log(self, message):
        """改进的日志输出，带时间戳和颜色"""
        timestamp = datetime.now().strftime("%H:%M:%S")

        # 根据消息类型设置颜色
        if "ERROR" in message or "错误" in message or "失败" in message:
            color = "#FF6B6B"  # 红色
        elif "成功" in message or "Connected" in message:
            color = "#51CF66"  # 绿色
        elif "AT+DRAW" in message:
            color = "#4DABF7"  # 蓝色
        elif "发送" in message or "Send" in message:
            color = "#FFD93D"  # 黄色
        else:
            color = "#E0E0E0"  # 默认颜色

        # 格式化消息
        formatted_msg = f'<span style="color: #888;">[{timestamp}]</span> <span style="color: {color};">{message.strip()}</span>'

        # 添加到日志显示
        self.log_display.append(formatted_msg)

        # 自动滚动到底部
        self.log_display.moveCursor(QTextCursor.End)

    def handle_comm_error(self, error_message):
        self.log(f"ERROR: {error_message}")
        self.disconnect_device()
        QMessageBox.warning(self, "通信错误 (Communication Error)", error_message)

    def update_plot(self, data_name, data_value):
        """更新波形图 - 支持多条曲线"""
        # 增加统计
        self.total_received += 1
        self.total_plot_points += 1

        # 如果暂停，只更新统计不绘图
        if self.plot_paused:
            return

        # 将数据添加到对应名称的缓冲区
        self.plot_data_buffer[data_name].append(data_value)

        # 限制缓冲区大小，防止内存溢出
        if len(self.plot_data_buffer[data_name]) > self.max_data_points:
            self.plot_data_buffer[data_name].pop(0)

        # 如果这是新的数据名称，创建新的曲线
        if data_name not in self.plot_curves:
            # 获取颜色
            color = self.plot_colors[self.color_index % len(self.plot_colors)]
            self.color_index += 1

            # 创建曲线
            pen = pg.mkPen(color=color, width=2)
            curve = self.plot_widget.plot(name=data_name,
                                          pen=pen,
                                          symbol='o',
                                          symbolSize=5,
                                          symbolBrush=color)
            self.plot_curves[data_name] = curve

        # 更新曲线数据
        data_points = self.plot_data_buffer[data_name]
        self.plot_curves[data_name].setData(data_points)

    def clear_plot(self):
        """清空所有波形数据"""
        self.plot_data_buffer.clear()
        self.plot_widget.clear()
        self.plot_curves.clear()
        self.color_index = 0
        self.plot_widget.addLegend(offset=(10, 10))
        self.log("波形数据已清空 (Plot data cleared)")

    def export_waveform_data(self):
        """导出波形数据到文件 - 支持多条曲线"""
        if not self.plot_data_buffer:
            QMessageBox.information(self, "无数据", "没有可导出的波形数据。")
            return

        path, _ = QFileDialog.getSaveFileName(
            self, "保存文件", "", "CSV Files (*.csv);;Text Files (*.txt)")

        if path:
            try:
                with open(path, 'w', encoding='utf-8') as f:
                    # 获取所有数据通道名称
                    channels = sorted(self.plot_data_buffer.keys())

                    # 写入表头
                    header = ['Sample_Index'] + channels
                    f.write(','.join(header) + '\n')

                    # 找出最大的数据长度
                    max_length = max(
                        len(self.plot_data_buffer[ch]) for ch in channels)

                    # 逐行写入数据
                    for i in range(max_length):
                        row = [str(i)]
                        for ch in channels:
                            if i < len(self.plot_data_buffer[ch]):
                                row.append(str(self.plot_data_buffer[ch][i]))
                            else:
                                row.append('')  # 如果某个通道数据不足，填空
                        f.write(','.join(row) + '\n')

                self.log(f"数据已导出至 {path} (共 {len(channels)} 个通道)")
                QMessageBox.information(
                    self, "导出成功",
                    f"数据已成功导出！\n文件: {path}\n通道数: {len(channels)}\n数据点: {max_length}"
                )
            except Exception as e:
                self.log(f"导出失败: {e}")
                QMessageBox.critical(self, "导出失败", f"无法保存文件: {e}")

    def refresh_serial_ports(self):
        """刷新串口列表"""
        current_text = self.port_combo.currentText()
        self.port_combo.clear()

        # 获取可用串口
        available_ports = UARTCommunication.list_available_ports()

        if available_ports:
            for port_info in available_ports:
                device, description, hwid = port_info
                display_text = f"{device} - {description}"
                self.port_combo.addItem(display_text, device)

            # 如果之前有选中的，尝试恢复
            if current_text:
                index = self.port_combo.findData(current_text)
                if index >= 0:
                    self.port_combo.setCurrentIndex(index)
                else:
                    self.port_combo.setCurrentText(current_text)
        else:
            # 没有找到串口，添加常用选项
            self.port_combo.addItems([
                "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyTHS1"
            ])

        self.log(f"已扫描到 {len(available_ports)} 个串口设备")

    def toggle_plot_pause(self):
        """暂停/继续绘图"""
        self.plot_paused = self.pause_plot_button.isChecked()
        if self.plot_paused:
            self.pause_plot_button.setText("▶ 继续绘图")
            self.log("绘图已暂停")
        else:
            self.pause_plot_button.setText("⏸ 暂停绘图")
            self.log("绘图已继续")

    def autoscale_plot(self):
        """自动缩放波形图"""
        self.plot_widget.enableAutoRange()
        self.log("已启用自动缩放")

    def save_config(self):
        """保存配置到文件"""
        config = {
            'port': self.port_combo.currentText(),
            'baudrate': self.baud_input.currentText(),
        }

        try:
            CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)
            with open(CONFIG_FILE, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2)
        except Exception as e:
            self.log(f"配置保存失败: {e}")

    def load_config(self):
        """从文件加载配置"""
        if CONFIG_FILE.exists():
            try:
                with open(CONFIG_FILE, 'r', encoding='utf-8') as f:
                    config = json.load(f)

                # 将配置保存到变量，稍后在UI初始化后应用
                self.saved_port = config.get('port', '/dev/ttyUSB0')
                self.saved_baudrate = config.get('baudrate', '115200')
            except Exception as e:
                self.log(f"配置加载失败: {e}")
                self.saved_port = '/dev/ttyUSB0'
                self.saved_baudrate = '115200'
        else:
            self.saved_port = '/dev/ttyUSB0'
            self.saved_baudrate = '115200'

    def init_statusbar(self):
        """初始化状态栏"""
        self.statusbar = QStatusBar()
        self.setStatusBar(self.statusbar)

        # 状态信息标签
        self.status_label = QLabel("未连接")
        self.stats_label = QLabel("接收: 0 | 绘图点: 0 | 通道: 0")

        self.statusbar.addWidget(self.status_label, 1)
        self.statusbar.addPermanentWidget(self.stats_label)

        # 应用保存的配置
        if hasattr(self, 'saved_port') and hasattr(self, 'saved_baudrate'):
            self.port_combo.setCurrentText(self.saved_port)
            self.baud_input.setCurrentText(self.saved_baudrate)

    def update_statistics(self):
        """更新状态栏统计信息"""
        if self.comm_instance and self.comm_instance.is_connected():
            port = self.port_combo.currentText().split(' ')[0]  # 提取设备名
            baud = self.baud_input.currentText()
            self.status_label.setText(f"已连接: {port} @ {baud}")
        else:
            self.status_label.setText("未连接")

        # 统计信息
        num_channels = len(self.plot_curves)
        total_points = sum(
            len(data) for data in self.plot_data_buffer.values())

        self.stats_label.setText(
            f"接收: {self.total_received} | 绘图点: {total_points} | 通道: {num_channels}"
        )

    def closeEvent(self, event):
        self.disconnect_device()
        self.stats_timer.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
