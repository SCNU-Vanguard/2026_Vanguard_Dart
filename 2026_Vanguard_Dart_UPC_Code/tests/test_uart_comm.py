# -*- coding: utf-8 -*-
"""
UART通信模块单元测试
使用mock对象模拟串口硬件，不依赖实际硬件
"""

import pytest
from unittest.mock import Mock, MagicMock, patch
import serial

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'SourceCode'))

from communication.uart_comm import UARTCommunication


class TestUARTCommunication:
    """UART通信类测试"""

    def test_init(self):
        """测试初始化"""
        uart = UARTCommunication('/dev/ttyUSB0', 115200)
        assert uart.port == '/dev/ttyUSB0'
        assert uart.baudrate == 115200
        assert uart.timeout == 1
        assert uart.serial_conn is None

    def test_init_with_custom_timeout(self):
        """测试自定义超时初始化"""
        uart = UARTCommunication('/dev/ttyUSB0', 115200, timeout=2)
        assert uart.timeout == 2

    @patch('serial.Serial')
    def test_connect_success(self, mock_serial):
        """测试成功连接"""
        mock_conn = MagicMock()
        mock_conn.is_open = True
        mock_serial.return_value = mock_conn

        uart = UARTCommunication('/dev/ttyUSB0', 115200)
        uart.connect()

        assert uart.serial_conn is not None
        mock_serial.assert_called_once_with(
            port='/dev/ttyUSB0',
            baudrate=115200,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        mock_conn.reset_input_buffer.assert_called_once()
        mock_conn.reset_output_buffer.assert_called_once()

    @patch('serial.Serial')
    def test_connect_already_connected(self, mock_serial):
        """测试重复连接（应该忽略）"""
        mock_conn = MagicMock()
        mock_conn.is_open = True
        mock_serial.return_value = mock_conn

        uart = UARTCommunication('/dev/ttyUSB0', 115200)
        uart.connect()
        uart.connect()  # 第二次连接

        # 应该只调用一次
        mock_serial.assert_called_once()

    @patch('serial.Serial')
    def test_connect_serial_exception(self, mock_serial):
        """测试连接失败（串口异常）"""
        mock_serial.side_effect = serial.SerialException("Port not found")

        uart = UARTCommunication('/dev/ttyUSB0', 115200)

        with pytest.raises(ConnectionError) as excinfo:
            uart.connect()

        assert "打开端口失败" in str(excinfo.value)

    def test_disconnect(self):
        """测试断开连接"""
        uart = UARTCommunication('/dev/ttyUSB0', 115200)
        mock_conn = MagicMock()
        mock_conn.is_open = True
        uart.serial_conn = mock_conn

        uart.disconnect()

        mock_conn.close.assert_called_once()
        assert uart.serial_conn is None

    def test_disconnect_not_connected(self):
        """测试断开连接（未连接状态）"""
        uart = UARTCommunication('/dev/ttyUSB0', 115200)
        uart.disconnect()  # 不应抛出异常
        assert uart.serial_conn is None

    def test_is_connected_true(self):
        """测试连接状态检查（已连接）"""
        uart = UARTCommunication('/dev/ttyUSB0', 115200)
        mock_conn = MagicMock()
        mock_conn.is_open = True
        uart.serial_conn = mock_conn

        assert uart.is_connected() is True

    def test_is_connected_false(self):
        """测试连接状态检查（未连接）"""
        uart = UARTCommunication('/dev/ttyUSB0', 115200)
        assert uart.is_connected() is False

    def test_send_data_string(self):
        """测试发送字符串数据"""
        uart = UARTCommunication('/dev/ttyUSB0', 115200)
        mock_conn = MagicMock()
        mock_conn.is_open = True
        uart.serial_conn = mock_conn

        uart.send_data("Hello")

        mock_conn.write.assert_called_once_with(b"Hello")
        mock_conn.flush.assert_called_once()

    def test_send_data_bytes(self):
        """测试发送字节数据"""
        uart = UARTCommunication('/dev/ttyUSB0', 115200)
        mock_conn = MagicMock()
        mock_conn.is_open = True
        uart.serial_conn = mock_conn

        uart.send_data(b"Hello")

        mock_conn.write.assert_called_once_with(b"Hello")
        mock_conn.flush.assert_called_once()

    def test_send_data_not_connected(self):
        """测试发送数据（未连接）"""
        uart = UARTCommunication('/dev/ttyUSB0', 115200)

        with pytest.raises(ConnectionError) as excinfo:
            uart.send_data("Hello")

        assert "未连接" in str(excinfo.value)

    def test_send_data_timeout(self):
        """测试发送数据超时"""
        uart = UARTCommunication('/dev/ttyUSB0', 115200)
        mock_conn = MagicMock()
        mock_conn.is_open = True
        mock_conn.write.side_effect = serial.SerialTimeoutException("Timeout")
        uart.serial_conn = mock_conn

        with pytest.raises(TimeoutError) as excinfo:
            uart.send_data("Hello")

        assert "发送超时" in str(excinfo.value)

    def test_receive_data_readline(self):
        """测试接收数据（按行）"""
        uart = UARTCommunication('/dev/ttyUSB0', 115200)
        mock_conn = MagicMock()
        mock_conn.is_open = True
        mock_conn.readline.return_value = b"Hello World\n"
        uart.serial_conn = mock_conn

        data = uart.receive_data()

        assert data == "Hello World"
        mock_conn.readline.assert_called_once()

    def test_receive_data_fixed_bytes(self):
        """测试接收数据（固定字节数）"""
        uart = UARTCommunication('/dev/ttyUSB0', 115200)
        mock_conn = MagicMock()
        mock_conn.is_open = True
        mock_conn.read.return_value = b"Hello"
        uart.serial_conn = mock_conn

        data = uart.receive_data(num_bytes=5)

        assert data == "Hello"
        mock_conn.read.assert_called_once_with(5)

    def test_receive_data_not_connected(self):
        """测试接收数据（未连接）"""
        uart = UARTCommunication('/dev/ttyUSB0', 115200)

        with pytest.raises(ConnectionError) as excinfo:
            uart.receive_data()

        assert "未连接" in str(excinfo.value)

    def test_receive_data_unicode_error(self):
        """测试接收数据（解码错误，应返回空字符串）"""
        uart = UARTCommunication('/dev/ttyUSB0', 115200)
        mock_conn = MagicMock()
        mock_conn.is_open = True
        mock_conn.readline.side_effect = Exception("Decode error")
        uart.serial_conn = mock_conn

        data = uart.receive_data()

        assert data == ""

    @patch('serial.tools.list_ports.comports')
    def test_list_available_ports(self, mock_comports):
        """测试列出可用串口"""
        # 创建mock端口对象
        mock_port1 = Mock()
        mock_port1.device = '/dev/ttyUSB0'
        mock_port1.description = 'USB Serial Port'
        mock_port1.hwid = 'USB VID:PID=1234:5678'

        mock_port2 = Mock()
        mock_port2.device = '/dev/ttyUSB1'
        mock_port2.description = 'USB Serial Port 2'
        mock_port2.hwid = 'USB VID:PID=1234:5679'

        mock_comports.return_value = [mock_port1, mock_port2]

        ports = UARTCommunication.list_available_ports()

        assert len(ports) == 2
        assert ports[0] == ('/dev/ttyUSB0', 'USB Serial Port', 'USB VID:PID=1234:5678')
        assert ports[1] == ('/dev/ttyUSB1', 'USB Serial Port 2', 'USB VID:PID=1234:5679')

    @patch('serial.tools.list_ports.comports')
    def test_list_available_ports_empty(self, mock_comports):
        """测试列出可用串口（无串口）"""
        mock_comports.return_value = []

        ports = UARTCommunication.list_available_ports()

        assert len(ports) == 0
        assert ports == []


class TestUARTIntegration:
    """UART集成测试（连接+发送+接收流程）"""

    @patch('serial.Serial')
    def test_full_communication_flow(self, mock_serial):
        """测试完整通信流程"""
        mock_conn = MagicMock()
        mock_conn.is_open = True
        mock_conn.readline.return_value = b"Response\n"
        mock_serial.return_value = mock_conn

        uart = UARTCommunication('/dev/ttyUSB0', 115200)

        # 连接
        uart.connect()
        assert uart.is_connected()

        # 发送
        uart.send_data("Command")
        mock_conn.write.assert_called_with(b"Command")

        # 接收
        response = uart.receive_data()
        assert response == "Response"

        # 断开
        uart.disconnect()
        assert not uart.is_connected()


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
