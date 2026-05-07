import time

import serial
import serial.tools.list_ports


# 串口协议常量
FRAME_HEADER = 0xAA
BAUDRATE = 921600
READ_TIMEOUT_S = 0.02
POST_WRITE_DELAY_S = 0.002
DEVICE_KEYWORDS = ("dart", "usb serial", "ch340")

CMD_QUERY_STATUS = 0x01
CMD_SET_COLOR = 0x02

COLOR_RED = 0x00
COLOR_BLUE = 0x01


# CRC-8/MAXIM-DOW，初始值为 0x00
def crc8_maxim_dow(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc & 0xFF


# 组装完整协议帧：帧头 + 命令码 + 数据长度 + 数据内容 + CRC
def build_frame(cmd_id, payload=b""):
    if len(payload) > 0xFF:
        raise ValueError("数据载荷长度不能超过 255 字节。")

    frame_without_crc = bytes([FRAME_HEADER, cmd_id, len(payload)]) + payload
    crc = crc8_maxim_dow(frame_without_crc)
    return frame_without_crc + bytes([crc])


def bytes_to_hex(data):
    return " ".join(f"{byte:02X}" for byte in data)


def color_name(color):
    if color == COLOR_RED:
        return "RED"
    if color == COLOR_BLUE:
        return "BLUE"
    return f"UNKNOWN(0x{color:02X})"


# 解析状态字节中的各个标志位
def parse_status_state(state):
    return {
        "raw": state,
        "binary": f"{state:08b}",
        "camp": "BLUE" if state & 0x08 else "RED",
        "visible_led_on": bool(state & 0x04),
        "voltage_error": bool(state & 0x02),
        "infrared_error": bool(state & 0x01),
    }


# 解析 0x01 状态查询返回帧
def parse_status_frame(frame):
    if frame is None:
        return None

    cmd_id = frame[1]
    data_len = frame[2]
    payload = frame[3 : 3 + data_len]

    if cmd_id != CMD_QUERY_STATUS:
        raise ValueError(f"状态查询回复命令码异常: 0x{cmd_id:02X}")

    if data_len != 3:
        raise ValueError(f"状态查询回复数据长度异常: {data_len}")

    version, temperature_c, state = payload
    return {
        "version": version,
        "temperature_c": temperature_c,
        "state": parse_status_state(state),
    }


def print_status_info(status_info):
    state = status_info["state"]
    visible_led = "点亮" if state["visible_led_on"] else "熄灭"
    voltage = "异常" if state["voltage_error"] else "正常"
    infrared = "异常" if state["infrared_error"] else "正常"
    camp = "蓝方" if state["camp"] == "BLUE" else "红方"

    print("状态解析结果:")
    print(f"  固件版本: v{status_info['version']}")
    print(f"  PCB 温度: {status_info['temperature_c']} C")
    print(f"  状态字节: 0x{state['raw']:02X} ({state['binary']})")
    print(f"  当前阵营: {camp}")
    print(f"  可见光灯珠: {visible_led}")
    print(f"  供电电压: {voltage}")
    print(f"  红外灯珠: {infrared}")


# 解析 0x02 设置阵营返回帧
def parse_set_color_frame(frame):
    if frame is None:
        return None

    cmd_id = frame[1]
    data_len = frame[2]
    payload = frame[3 : 3 + data_len]

    if cmd_id != CMD_SET_COLOR:
        raise ValueError(f"设置阵营回复命令码异常: 0x{cmd_id:02X}")

    if data_len == 1:
        error_code = payload[0]
        return {
            "payload_len": data_len,
            "payload_hex": bytes_to_hex(payload),
            "payload_raw": payload,
            "reply_type": "operation_result",
            "error_code": error_code,
            "success": error_code == 0x00,
        }

    return {
        "payload_len": data_len,
        "payload_hex": bytes_to_hex(payload),
        "payload_raw": payload,
        "reply_type": "unknown_format",
    }


def print_set_color_info(requested_color, reply_info):
    print("设置阵营回复解析:")
    print(f"  请求阵营: {'红方' if requested_color == COLOR_RED else '蓝方'}")

    if reply_info is None:
        print("  未收到返回数据。")
        return

    print(f"  回复类型: {reply_info['reply_type']}")
    print(f"  数据长度: {reply_info['payload_len']}")
    print(f"  数据内容: {reply_info['payload_hex'] or '(空)'}")

    if reply_info["reply_type"] == "operation_result":
        print(f"  错误码: 0x{reply_info['error_code']:02X}")
        if reply_info["success"]:
            print("  设置结果: 成功")
            print("  说明: 设备确认阵营设置成功。")
        else:
            print("  设置结果: 失败")
            print("  说明: 设备确认阵营设置失败，可能已超过上电后 3 秒设置窗口。")
        return

    print("  说明: 该回复格式暂未定义，当前仅保留原始载荷。")


def list_ports():
    return list(serial.tools.list_ports.comports())


def choose_port():
    ports = list_ports()
    if not ports:
        raise RuntimeError("未找到可用串口。")

    for port in ports:
        port_text = " ".join(
            filter(None, [port.device, port.description, port.manufacturer, port.hwid])
        ).lower()
        if any(keyword in port_text for keyword in DEVICE_KEYWORDS):
            return port.device

    return ports[0].device


def open_serial(port_name):
    return serial.Serial(
        port=port_name,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=READ_TIMEOUT_S,
        xonxoff=False,
        rtscts=False,
        write_timeout=1,
    )


# 按协议读取一整帧，并校验 CRC
def read_frame(ser):
    header = ser.read(1)
    if not header:
        return None

    if header[0] != FRAME_HEADER:
        raise ValueError(f"帧头异常: 0x{header[0]:02X}")

    fixed = ser.read(2)
    if len(fixed) != 2:
        raise ValueError("帧头读取不完整。")

    cmd_id = fixed[0]
    data_len = fixed[1]
    payload = ser.read(data_len)
    if len(payload) != data_len:
        raise ValueError("数据内容读取不完整。")

    crc = ser.read(1)
    if len(crc) != 1:
        raise ValueError("缺少 CRC 校验字节。")

    frame = header + fixed + payload + crc
    expected_crc = crc8_maxim_dow(frame[:-1])
    if crc[0] != expected_crc:
        raise ValueError(
            f"CRC 校验失败: 收到 0x{crc[0]:02X}，期望 0x{expected_crc:02X}"
        )

    return frame


def send_frame_and_wait_reply(ser, frame):
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()

    # 设备通常会在 0.1 ms 到 2 ms 内返回数据
    time.sleep(POST_WRITE_DELAY_S)

    return read_frame(ser)


def query_status(ser):
    frame = build_frame(CMD_QUERY_STATUS)
    return frame, send_frame_and_wait_reply(ser, frame)


def set_color(ser, color):
    if color not in (COLOR_RED, COLOR_BLUE):
        raise ValueError("不支持的阵营值。")

    frame = build_frame(CMD_SET_COLOR, bytes([color]))
    return frame, send_frame_and_wait_reply(ser, frame)


def print_available_ports():
    ports = list_ports()
    if not ports:
        print("未找到可用串口。")
        return []

    print("检测到以下串口:")
    for port in ports:
        print(f"  {port.device} | {port.description}")
    return ports


def print_menu():
    print("\n请选择功能:")
    print("1. 查询状态和错误码")
    print("2. 设置为红方（需在上电后 3 秒内发送）")
    print("3. 设置为蓝方（需在上电后 3 秒内发送）")
    print("4. 退出")


def main():
    ports = print_available_ports()
    if not ports:
        return

    port_name = choose_port()
    print(f"\n自动选择串口: {port_name}")

    try:
        with open_serial(port_name) as dart_serial:
            print("串口已打开。")

            while True:
                print_menu()
                choice = input("请输入 1/2/3/4: ").strip()

                if choice == "1":
                    tx_frame, rx_frame = query_status(dart_serial)
                    print(f"TX: {bytes_to_hex(tx_frame)}")
                    print(
                        "RX: "
                        + (bytes_to_hex(rx_frame) if rx_frame else "未收到返回数据。")
                    )
                    if rx_frame:
                        print_status_info(parse_status_frame(rx_frame))
                elif choice == "2":
                    tx_frame, rx_frame = set_color(dart_serial, COLOR_RED)
                    print(f"TX: {bytes_to_hex(tx_frame)}")
                    print(
                        "RX: "
                        + (bytes_to_hex(rx_frame) if rx_frame else "未收到返回数据。")
                    )
                    print_set_color_info(
                        COLOR_RED,
                        parse_set_color_frame(rx_frame) if rx_frame else None,
                    )
                elif choice == "3":
                    tx_frame, rx_frame = set_color(dart_serial, COLOR_BLUE)
                    print(f"TX: {bytes_to_hex(tx_frame)}")
                    print(
                        "RX: "
                        + (bytes_to_hex(rx_frame) if rx_frame else "未收到返回数据。")
                    )
                    print_set_color_info(
                        COLOR_BLUE,
                        parse_set_color_frame(rx_frame) if rx_frame else None,
                    )
                elif choice == "4":
                    break
                else:
                    print("输入无效。")
    except serial.SerialException as exc:
        print(f"串口错误: {exc}")
    except ValueError as exc:
        print(f"协议错误: {exc}")
    except RuntimeError as exc:
        print(f"运行错误: {exc}")


if __name__ == "__main__":
    main()
