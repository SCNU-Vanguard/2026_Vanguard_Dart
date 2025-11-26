# -*- coding: utf-8 -*-
"""
UART通信模块
支持USB转UART和实体UART（如Jetson Nano的硬件串口）
"""

import serial
import serial.tools.list_ports


class UARTCommunication:
    """
    基于pySerial实现UART通信
    支持：
    - USB转UART虚拟串口（如/dev/ttyUSB0, /dev/ttyACM0）
    - 实体UART硬件串口（如Jetson的/dev/ttyTHS1, Raspberry Pi的/dev/ttyAMA0）
    """

    def __init__(self, port, baudrate, timeout=1):
        """
        初始化UART通信对象

        参数:
            port (str): 串口名称
                       USB虚拟串口: /dev/ttyUSB0, /dev/ttyUSB1, /dev/ttyACM0
                       Jetson Nano: /dev/ttyTHS1 (40-pin header UART)
                       Raspberry Pi: /dev/ttyAMA0, /dev/ttyS0
            baudrate (int): 波特率 (9600, 115200, 230400, 460800, 921600等)
            timeout (int): 超时时间, 单位为秒
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None

    def connect(self):
        """连接到UART端口"""
        if self.serial_conn and self.serial_conn.is_open:
            return

        try:
            self.serial_conn = serial.Serial(port=self.port,
                                             baudrate=self.baudrate,
                                             timeout=self.timeout,
                                             bytesize=serial.EIGHTBITS,
                                             parity=serial.PARITY_NONE,
                                             stopbits=serial.STOPBITS_ONE)

            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
        except serial.SerialException as e:
            raise ConnectionError(f"打开端口失败 {self.port}: {e}")
        except Exception as e:
            raise ConnectionError(f"连接错误: {e}")

    def disconnect(self):
        """关闭UART连接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.serial_conn = None

    def is_connected(self):
        """确认是否连接"""
        return self.serial_conn and self.serial_conn.is_open

    def send_data(self, data):
        """
        通过UART发送数据

        参数:
            data (bytes or str): 要发送的数据
                               如果是str，会自动编码为UTF-8

        Raises:
            ConnectionError: 未连接
            TimeoutError: 发送超时
        """
        if not self.is_connected():
            raise ConnectionError("UART未连接")

        try:
            if isinstance(data, str):
                data = data.encode('utf-8')
            self.serial_conn.write(data)
            self.serial_conn.flush()  # 确保数据发送完毕
        except serial.SerialTimeoutException as e:
            raise TimeoutError(f"发送超时: {e}")
        except Exception as e:
            raise IOError(f"发送错误: {e}")

    def receive_data(self, num_bytes=None):
        """
        通过UART接收数据

        参数:
            num_bytes (int, optional): 读取目标字节数
                                      如果为None，则读取一行（直到\n）

        Returns:
            str: 接收到的数据，以UTF-8形式解码

        Raises:
            ConnectionError: 未连接
        """
        if not self.is_connected():
            raise ConnectionError("UART未连接")

        try:
            if num_bytes:
                received_data = self.serial_conn.read(num_bytes)
            else:
                received_data = self.serial_conn.readline()

            return received_data.decode('utf-8', errors='ignore').strip()
        except Exception as e:
            # 读取错误不抛异常，返回空字符串
            return ""

    @staticmethod
    def list_available_ports():
        """
        列出系统中所有可用的串口

        Returns:
            list: 可用串口列表，每项包含(设备名, 描述, 硬件ID)

        todo:
            这个逻辑仍有问题，需要保证正常的PC端可以使用
        """
        ports = serial.tools.list_ports.comports()
        return [(port.device, port.description, port.hwid) for port in ports]
