#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
绘图数据模拟器
用于测试上位机的多曲线绘图功能

使用方法:
1. 先运行主程序 main_app.py
2. 在主程序中连接到虚拟串口 (例如 /dev/ttyUSB0)
3. 运行此脚本来模拟发送数据

或者使用 socat 创建虚拟串口对:
socat -d -d pty,raw,echo=0 pty,raw,echo=0
然后将两个虚拟串口分别用于主程序和测试脚本
"""

import serial
import time
import math
import random
import argparse


def simulate_data(port, baudrate=115200, duration=60):
    """
    模拟发送多通道数据

    参数:
        port: 串口设备名
        baudrate: 波特率
        duration: 模拟持续时间（秒）
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"已连接到 {port} @ {baudrate}")
        print(f"开始发送数据，持续 {duration} 秒...")
        print("发送格式: AT+DRAW+<数据名称>+<数值>")
        print("-" * 50)

        start_time = time.time()
        sample = 0

        while time.time() - start_time < duration:
            # 模拟不同类型的数据

            # 1. 正弦波 - Temperature
            temperature = 25 + 10 * math.sin(sample * 0.1)
            cmd1 = f"AT+DRAW+Temperature+{temperature:.2f}\n"
            ser.write(cmd1.encode('utf-8'))
            print(f"发送: {cmd1.strip()}")
            time.sleep(0.05)

            # 2. 余弦波 - Voltage
            voltage = 3.3 + 0.5 * math.cos(sample * 0.15)
            cmd2 = f"AT+DRAW+Voltage+{voltage:.2f}\n"
            ser.write(cmd2.encode('utf-8'))
            print(f"发送: {cmd2.strip()}")
            time.sleep(0.05)

            # 3. 随机波动 - Humidity
            humidity = 50 + random.uniform(-10, 10)
            cmd3 = f"AT+DRAW+Humidity+{humidity:.2f}\n"
            ser.write(cmd3.encode('utf-8'))
            print(f"发送: {cmd3.strip()}")
            time.sleep(0.05)

            # 4. 锯齿波 - Current
            current = (sample % 50) * 0.1
            cmd4 = f"AT+DRAW+Current+{current:.2f}\n"
            ser.write(cmd4.encode('utf-8'))
            print(f"发送: {cmd4.strip()}")
            time.sleep(0.05)

            # 5. 阶跃函数 - Speed
            speed = 100 if (sample // 20) % 2 == 0 else 200
            cmd5 = f"AT+DRAW+Speed+{speed:.2f}\n"
            ser.write(cmd5.encode('utf-8'))
            print(f"发送: {cmd5.strip()}")

            sample += 1
            time.sleep(0.3)  # 每组数据间隔

        print("-" * 50)
        print("数据发送完成!")
        ser.close()

    except serial.SerialException as e:
        print(f"串口错误: {e}")
        print("\n提示: 请确保:")
        print("1. 串口设备存在且可访问")
        print("2. 没有其他程序占用该串口")
        print("3. 用户有权限访问该串口 (可能需要 sudo)")
    except KeyboardInterrupt:
        print("\n用户中断，停止发送")
        ser.close()


def main():
    parser = argparse.ArgumentParser(
        description='模拟发送绘图数据到上位机',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  python test_plot_simulator.py /dev/ttyUSB0
  python test_plot_simulator.py /dev/ttyUSB1 -b 115200 -d 120

如果没有物理串口，可以使用 socat 创建虚拟串口对:
  socat -d -d pty,raw,echo=0 pty,raw,echo=0
  # 输出类似: N PTY is /dev/pts/3 和 N PTY is /dev/pts/4
  # 然后在主程序连接 /dev/pts/3，在此脚本使用 /dev/pts/4
        """
    )

    parser.add_argument('port', help='串口设备名 (例如: /dev/ttyUSB0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                        help='波特率 (默认: 115200)')
    parser.add_argument('-d', '--duration', type=int, default=60,
                        help='发送持续时间（秒，默认: 60）')

    args = parser.parse_args()

    simulate_data(args.port, args.baudrate, args.duration)


if __name__ == "__main__":
    main()
