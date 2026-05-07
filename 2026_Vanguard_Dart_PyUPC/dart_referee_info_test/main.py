#!/usr/bin/env python3
"""
RoboMaster 2026 裁判系统串口模拟器。

本脚本通过串口向 STM32 发送裁判系统常规链路数据帧，帧格式匹配
RoboMaster 2026 高校系列赛通信协议 V1.3.0，以及 Dart_Ctrl 工程中的
裁判系统解析代码：

    0xA5 | data_length:u16 | seq:u8 | crc8 | cmd_id:u16 | data | crc16

已实现的数据：
    0x0001 比赛状态                 11 字节，默认 1Hz
    0x0002 比赛结果                  1 字节，可选，结束时触发一次
    0x0105 飞镖发射相关数据          3 字节，默认 1Hz
    0x0201 机器人状态/性能体系      13 字节，默认 10Hz
    0x020A 飞镖发射站状态            6 字节，默认 3Hz

不指定 --port 时会进入串口选择菜单。连续发送模式会等待你按回车确认
7 分钟比赛计时开始，然后按协议频率持续发送。飞镖发射站状态和飞镖
目标只能在计时开始后通过运行时菜单选择。
"""

from __future__ import annotations

import argparse
import binascii
import dataclasses
import struct
import sys
import threading
import time
from typing import Callable, Iterable

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover - 环境检查
    serial = None
    list_ports = None
    _SERIAL_IMPORT_ERROR = exc
else:
    _SERIAL_IMPORT_ERROR = None


SOF = 0xA5
CMD_GAME_STATUS = 0x0001
CMD_GAME_RESULT = 0x0002
CMD_DART_INFO = 0x0105
CMD_ROBOT_STATUS = 0x0201
CMD_DART_LAUNCH_STATUS = 0x020A

CRC8_INIT = 0xFF
CRC8_POLY_REFLECTED = 0x8C
CRC16_INIT = 0xFFFF
CRC16_POLY_REFLECTED = 0x8408

DART_STATUS_OPEN = 0
DART_STATUS_CLOSED = 1
DART_STATUS_MOVING = 2

DART_PHASE_CLOSED = "closed"
DART_PHASE_OPENING = "opening"
DART_PHASE_OPEN = "open"
DART_PHASE_CLOSING = "closing"

DART_STATUS_LABELS = {
    DART_STATUS_OPEN: "已完全开启",
    DART_STATUS_CLOSED: "关闭",
    DART_STATUS_MOVING: "开启/关闭中",
}

DART_REQUEST_IDLE = 0
DART_REQUEST_CLOSE = 1
DART_REQUEST_OPEN_ONCE = 2
DART_REQUEST_FORCE_OPEN_FIRE = 3

DART_REQUEST_LABELS = {
    DART_REQUEST_IDLE: "等待",
    DART_REQUEST_CLOSE: "保持关闭",
    DART_REQUEST_OPEN_ONCE: "请求开启一次",
    DART_REQUEST_FORCE_OPEN_FIRE: "强制常开并保持发射状态",
}

DART_PHASE_LABELS = {
    DART_PHASE_CLOSED: "关闭",
    DART_PHASE_OPENING: "正在开启",
    DART_PHASE_OPEN: "完全开启",
    DART_PHASE_CLOSING: "正在关闭",
}


def int_auto(value: str) -> int:
    """解析十进制或 0x 前缀整数，供 argparse 使用。"""
    try:
        return int(value, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"无效整数：{value!r}") from exc


def require_range(name: str, value: int, minimum: int, maximum: int) -> int:
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be in [{minimum}, {maximum}], got {value}")
    return value


def crc8(data: bytes, init: int = CRC8_INIT) -> int:
    """裁判系统帧头 CRC8，算法与下位机 CRC.c 一致。"""
    crc = init & 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ CRC8_POLY_REFLECTED
            else:
                crc >>= 1
            crc &= 0xFF
    return crc


def crc16(data: bytes, init: int = CRC16_INIT) -> int:
    """裁判系统整帧 CRC16，帧尾按小端序写入。"""
    crc = init & 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ CRC16_POLY_REFLECTED
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


class FrameBuilder:
    def __init__(self, initial_seq: int = 0) -> None:
        self.seq = initial_seq & 0xFF

    def build(self, cmd_id: int, payload: bytes) -> bytes:
        header_without_crc = struct.pack("<BHB", SOF, len(payload), self.seq)
        header = header_without_crc + bytes([crc8(header_without_crc)])
        frame_without_tail = header + struct.pack("<H", cmd_id) + payload
        frame = frame_without_tail + struct.pack("<H", crc16(frame_without_tail))
        self.seq = (self.seq + 1) & 0xFF
        return frame


@dataclasses.dataclass
class SimulatorState:
    start_monotonic: float
    start_stage_remain: int
    sync_time: int | None
    countdown: bool
    game_type: int
    game_progress: int
    game_result: int | None
    robot_id: int
    robot_level: int
    current_hp: int
    max_hp: int
    cooling_value: int
    heat_limit: int
    chassis_power_limit: int
    power_flags: int
    dart_remaining_time: int
    last_hit_target: int
    hit_count: int
    selected_target: int
    dart_opening_status: int
    target_change_time: int
    latest_launch_cmd_time: int
    dart_first_open_after: int = 30
    dart_second_open_after: int = 240
    dart_open_duration: int = 30
    dart_transition_duration: int = 7
    auto_launch_confirm: bool = True
    auto_launch_confirm_delay: float = 1.0
    dart_request_mode: int = DART_REQUEST_IDLE
    dart_phase_start_at: float | None = None
    dart_transition_target_status: int = DART_STATUS_CLOSED
    dart_active_window: int = 0
    dart_launch_cmd_recorded: bool = False
    dart_first_window_used: bool = False
    dart_second_window_used: bool = False
    stop_requested: bool = False
    lock: threading.RLock = dataclasses.field(
        default_factory=threading.RLock, repr=False, compare=False
    )

    def stage_remain_time(self) -> int:
        if not self.countdown:
            return self.start_stage_remain
        elapsed = int(time.monotonic() - self.start_monotonic)
        return max(0, self.start_stage_remain - elapsed)

    def sync_timestamp(self) -> int:
        if self.sync_time is not None:
            return self.sync_time
        return int(time.time())


def match_elapsed_seconds(state: SimulatorState, now: float) -> float:
    if not state.countdown:
        return 0.0
    return max(0.0, now - state.start_monotonic)


def current_dart_phase_elapsed(state: SimulatorState, now: float) -> float:
    if state.dart_phase_start_at is None:
        return 0.0
    return max(0.0, now - state.dart_phase_start_at)


def current_dart_remaining_time(state: SimulatorState, now: float) -> int:
    if state.dart_request_mode == DART_REQUEST_FORCE_OPEN_FIRE:
        return state.dart_open_duration
    if not state.countdown:
        return 0
    if state.dart_opening_status != DART_STATUS_OPEN or state.dart_phase_start_at is None:
        return 0
    elapsed = int(now - state.dart_phase_start_at)
    return max(0, state.dart_open_duration - elapsed)


def available_dart_window(state: SimulatorState, elapsed: float) -> int:
    if elapsed >= state.dart_second_open_after:
        if not state.dart_second_window_used:
            return 2
        return 0
    if elapsed >= state.dart_first_open_after and not state.dart_first_window_used:
        return 1
    return 0


def update_dart_state_machine(state: SimulatorState, now: float) -> None:
    with state.lock:
        if state.dart_request_mode == DART_REQUEST_FORCE_OPEN_FIRE:
            state.dart_opening_status = DART_STATUS_OPEN
            state.dart_transition_target_status = DART_STATUS_OPEN
            state.dart_active_window = 0
            state.dart_phase_start_at = now
            if state.latest_launch_cmd_time == 0:
                state.latest_launch_cmd_time = state.stage_remain_time()
            state.dart_launch_cmd_recorded = True
            return

        if not state.countdown:
            state.dart_opening_status = DART_STATUS_CLOSED
            state.dart_phase_start_at = None
            state.dart_transition_target_status = DART_STATUS_CLOSED
            state.dart_active_window = 0
            return

        elapsed = match_elapsed_seconds(state, now)
        phase_elapsed = current_dart_phase_elapsed(state, now)

        if state.dart_opening_status == DART_STATUS_MOVING:
            if phase_elapsed < state.dart_transition_duration:
                return
            if state.dart_transition_target_status == DART_STATUS_OPEN:
                state.dart_opening_status = DART_STATUS_OPEN
                state.dart_phase_start_at = now
                state.dart_launch_cmd_recorded = False
                return
            state.dart_opening_status = DART_STATUS_CLOSED
            state.dart_phase_start_at = None
            state.dart_transition_target_status = DART_STATUS_CLOSED
            state.dart_active_window = 0
            state.dart_request_mode = DART_REQUEST_IDLE
            state.dart_launch_cmd_recorded = False
            return

        if state.dart_opening_status == DART_STATUS_OPEN:
            # 真实比赛中操作手会在完全开启窗口内确认发射；默认自动模拟这一次确认。
            if (
                state.auto_launch_confirm
                and not state.dart_launch_cmd_recorded
                and phase_elapsed >= state.auto_launch_confirm_delay
            ):
                state.latest_launch_cmd_time = state.stage_remain_time()
                state.dart_launch_cmd_recorded = True
            if phase_elapsed < state.dart_open_duration:
                return
            if state.dart_active_window == 1:
                state.dart_first_window_used = True
            elif state.dart_active_window == 2:
                state.dart_second_window_used = True
            state.dart_active_window = 0
            state.dart_opening_status = DART_STATUS_MOVING
            state.dart_transition_target_status = DART_STATUS_CLOSED
            state.dart_phase_start_at = now
            return

        if state.dart_request_mode == DART_REQUEST_CLOSE:
            return

        if state.dart_request_mode != DART_REQUEST_OPEN_ONCE:
            return

        if state.dart_active_window == 0:
            window = available_dart_window(state, elapsed)
            if window == 0:
                return
            state.dart_active_window = window

        state.dart_opening_status = DART_STATUS_MOVING
        state.dart_transition_target_status = DART_STATUS_OPEN
        state.dart_phase_start_at = now
        state.dart_request_mode = DART_REQUEST_IDLE


def build_game_status_payload(state: SimulatorState) -> bytes:
    with state.lock:
        game_status = ((state.game_progress & 0x0F) << 4) | (state.game_type & 0x0F)
        return struct.pack(
            "<BHQ",
            game_status,
            state.stage_remain_time(),
            state.sync_timestamp(),
        )


def build_game_result_payload(state: SimulatorState) -> bytes:
    with state.lock:
        if state.game_result is None:
            raise RuntimeError("未启用比赛结果帧")
        return struct.pack("<B", state.game_result)


def build_dart_info_payload(state: SimulatorState) -> bytes:
    with state.lock:
        remaining_time = current_dart_remaining_time(state, time.monotonic())
        state.dart_remaining_time = remaining_time
        dart_info = (
            (state.last_hit_target & 0x07)
            | ((state.hit_count & 0x07) << 3)
            | ((state.selected_target & 0x07) << 6)
        )
        return struct.pack("<BH", remaining_time, dart_info)


def build_robot_status_payload(state: SimulatorState) -> bytes:
    with state.lock:
        return struct.pack(
            "<BBHHHHHB",
            state.robot_id,
            state.robot_level,
            state.current_hp,
            state.max_hp,
            state.cooling_value,
            state.heat_limit,
            state.chassis_power_limit,
            state.power_flags & 0x07,
        )


def build_dart_launch_status_payload(state: SimulatorState) -> bytes:
    with state.lock:
        return struct.pack(
            "<BBHH",
            state.dart_opening_status,
            0,
            state.target_change_time,
            state.latest_launch_cmd_time,
        )


@dataclasses.dataclass
class MessageSpec:
    cmd_id: int
    name: str
    rate_hz: float
    payload_builder: Callable[[SimulatorState], bytes]
    expected_len: int
    next_send: float = 0.0

    @property
    def period(self) -> float:
        return 1.0 / self.rate_hz


def make_message_specs(args: argparse.Namespace, now: float) -> list[MessageSpec]:
    specs = [
        MessageSpec(CMD_GAME_STATUS, "game_status", args.game_status_hz, build_game_status_payload, 11),
        MessageSpec(CMD_DART_INFO, "dart_info", args.dart_info_hz, build_dart_info_payload, 3),
        MessageSpec(CMD_ROBOT_STATUS, "robot_status", args.robot_status_hz, build_robot_status_payload, 13),
        MessageSpec(
            CMD_DART_LAUNCH_STATUS,
            "dart_launch_status",
            args.dart_launch_status_hz,
            build_dart_launch_status_payload,
            6,
        ),
    ]
    for spec in specs:
        spec.next_send = now
    return specs


def build_state(args: argparse.Namespace) -> SimulatorState:
    return SimulatorState(
        start_monotonic=time.monotonic(),
        start_stage_remain=args.stage_remain,
        sync_time=args.sync_time,
        countdown=not args.no_countdown,
        game_type=args.game_type,
        game_progress=args.game_progress,
        game_result=args.game_result,
        robot_id=args.robot_id,
        robot_level=args.robot_level,
        current_hp=args.current_hp,
        max_hp=args.max_hp,
        cooling_value=args.cooling_value,
        heat_limit=args.heat_limit,
        chassis_power_limit=args.chassis_power_limit,
        power_flags=args.power_flags,
        dart_remaining_time=args.dart_remaining_time,
        last_hit_target=args.last_hit_target,
        hit_count=args.hit_count,
        selected_target=args.selected_target,
        dart_opening_status=DART_STATUS_CLOSED,
        target_change_time=args.target_change_time,
        latest_launch_cmd_time=args.latest_launch_cmd_time,
        dart_first_open_after=args.dart_first_open_after,
        dart_second_open_after=args.dart_second_open_after,
        dart_open_duration=args.dart_open_duration,
        dart_transition_duration=args.dart_transition_duration,
        auto_launch_confirm=not args.no_auto_launch_confirm,
        auto_launch_confirm_delay=args.auto_launch_confirm_delay,
        dart_request_mode=args.dart_request_mode,
    )


def apply_prestart_force_open_fire(state: SimulatorState) -> None:
    with state.lock:
        state.countdown = False
        state.game_progress = 0
        state.dart_request_mode = DART_REQUEST_FORCE_OPEN_FIRE
        state.dart_opening_status = DART_STATUS_OPEN
        state.dart_transition_target_status = DART_STATUS_OPEN
        state.dart_phase_start_at = time.monotonic()
        state.latest_launch_cmd_time = state.stage_remain_time()
        state.dart_launch_cmd_recorded = True


def hex_line(data: bytes) -> str:
    return binascii.hexlify(data, sep=" ").decode("ascii").upper()


def validate_payload_lengths(specs: Iterable[MessageSpec], state: SimulatorState) -> None:
    for spec in specs:
        payload = spec.payload_builder(state)
        if len(payload) != spec.expected_len:
            raise RuntimeError(
                f"{spec.name} 数据段长度为 {len(payload)}，应为 {spec.expected_len}"
            )
    if state.game_result is not None:
        payload = build_game_result_payload(state)
        if len(payload) != 1:
            raise RuntimeError(f"game_result 数据段长度为 {len(payload)}，应为 1")


def open_serial_port(args: argparse.Namespace):
    if serial is None:
        raise RuntimeError(
            "未安装 pyserial，请执行：pip install pyserial"
        ) from _SERIAL_IMPORT_ERROR

    return serial.serial_for_url(
        args.port,
        baudrate=args.baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,
        write_timeout=args.write_timeout,
    )


def print_ports() -> None:
    if list_ports is None:
        print("未安装 pyserial，无法列出串口。", file=sys.stderr)
        return
    ports = list(list_ports.comports())
    if not ports:
        print("未找到可用串口。")
        return
    for port in ports:
        desc = port.description or ""
        hwid = port.hwid or ""
        print(f"{port.device}\t{desc}\t{hwid}")


STATION_REQUEST_CHOICES = [
    ("等待", DART_REQUEST_IDLE),
    ("保持关闭", DART_REQUEST_CLOSE),
    ("请求开启一次", DART_REQUEST_OPEN_ONCE),
    ("强制常开并保持发射状态", DART_REQUEST_FORCE_OPEN_FIRE),
]

DART_TARGET_CHOICES = [
    ("前哨站/未选择", 0),
    ("基地固定目标", 1),
    ("基地随机固定目标", 2),
    ("基地随机移动目标", 3),
    ("基地末端移动目标", 4),
]


def read_choice(prompt: str, choices: list[tuple[str, int]], default: int | None = None) -> int:
    for label, value in choices:
        marker = "（默认）" if default is not None and value == default else ""
        print(f"  {value}: {label}{marker}")

    valid_values = {value for _, value in choices}
    while True:
        raw = input(prompt).strip()
        if raw == "" and default is not None:
            return default
        try:
            value = int(raw, 0)
        except ValueError:
            print("输入无效，请输入列表中的数字。")
            continue
        if value in valid_values:
            return value
        print("选择无效，请输入列表中的数字。")


def dart_request_hint(state: SimulatorState) -> str:
    now = time.monotonic()
    elapsed = match_elapsed_seconds(state, now)
    remain = state.stage_remain_time()
    status = DART_STATUS_LABELS.get(state.dart_opening_status, str(state.dart_opening_status))
    first_wait = max(0, state.dart_first_open_after - int(elapsed))
    second_wait = max(0, state.dart_second_open_after - int(elapsed))
    first_state = "已使用" if state.dart_first_window_used else ("可用" if first_wait == 0 else f"还需 {first_wait}s")
    second_state = "已使用" if state.dart_second_window_used else ("可用" if second_wait == 0 else f"还需 {second_wait}s")
    return (
        f"比赛剩余={remain}s，已进行={int(elapsed)}s，"
        f"发射站={status}，"
        f"第一次机会={first_state}，"
        f"第二次机会={second_state}，"
        f"当前模式={DART_REQUEST_LABELS.get(state.dart_request_mode, state.dart_request_mode)}"
    )


def interactive_setup(args: argparse.Namespace) -> None:
    if serial is None or list_ports is None:
        raise RuntimeError(
            "未安装 pyserial，请执行：pip install pyserial"
        ) from _SERIAL_IMPORT_ERROR

    if args.port is None:
        ports = list(list_ports.comports())
        if not ports:
            raise RuntimeError("未找到可用串口。")

        print("可用串口：")
        for index, port in enumerate(ports, start=1):
            desc = port.description or ""
            hwid = port.hwid or ""
            print(f"  {index}: {port.device}\t{desc}\t{hwid}")

        while True:
            raw = input("请选择串口序号：").strip()
            try:
                index = int(raw, 10)
            except ValueError:
                print("输入无效，请输入列表中的串口序号。")
                continue
            if 1 <= index <= len(ports):
                args.port = ports[index - 1].device
                break
            print("串口序号无效。")


def send_frame(ser, frame: bytes, dry_run: bool) -> None:
    if dry_run:
        return
    written = ser.write(frame)
    if written != len(frame):
        raise RuntimeError(f"串口写入不完整：{written}/{len(frame)} 字节")


def drain_rx(ser, rx_print: bool) -> None:
    if ser is None:
        return
    waiting = getattr(ser, "in_waiting", 0)
    if waiting <= 0:
        return
    data = ser.read(waiting)
    if rx_print and data:
        print(f"接收 {len(data):03d}: {hex_line(data)}")


def runtime_menu(state: SimulatorState) -> None:
    print()
    print("7 分钟计时中可使用运行菜单。")
    print("命令：s=选择发射站模式，t=选择飞镖目标，f=手动确认发射，i=查看状态，q=退出。")

    while True:
        try:
            command = input("运行菜单> ").strip().lower()
        except EOFError:
            return

        if command == "":
            continue
        if command in {"q", "quit", "exit"}:
            with state.lock:
                state.stop_requested = True
            return
        if command in {"h", "help", "?"}:
            print("命令：s=选择发射站模式，t=选择飞镖目标，f=手动确认发射，i=查看状态，q=退出。")
            continue
        if command in {"i", "info", "status"}:
            with state.lock:
                print(dart_request_hint(state))
            continue
        if command in {"s", "station"}:
            with state.lock:
                print(dart_request_hint(state))
                default = state.dart_request_mode
            value = read_choice("请选择发射站模式：", STATION_REQUEST_CHOICES, default=default)
            with state.lock:
                state.dart_request_mode = value
                remain = state.stage_remain_time()
                label = DART_REQUEST_LABELS.get(value, str(value))
            if value == DART_REQUEST_OPEN_ONCE:
                print(f"已请求开启一次，当前比赛剩余时间={remain} 秒；到达可开启窗口后会自动进入开启流程。")
            elif value == DART_REQUEST_FORCE_OPEN_FIRE:
                print(f"已进入调试模式：发射站将一直保持完全开启，并持续带有确认发射时间；当前比赛剩余时间={remain} 秒。")
            else:
                print(f"发射站模式已设置为 {label}，当前比赛剩余时间={remain} 秒。")
            continue
        if command in {"t", "target"}:
            with state.lock:
                default = state.selected_target
            value = read_choice("请选择飞镖目标：", DART_TARGET_CHOICES, default=default)
            with state.lock:
                state.selected_target = value
                state.target_change_time = state.stage_remain_time()
                remain = state.target_change_time
            print(f"飞镖目标已设置为 {value}，target_change_time={remain} 秒。")
            continue
        if command in {"f", "fire", "launch"}:
            with state.lock:
                if state.dart_opening_status != DART_STATUS_OPEN:
                    station = DART_STATUS_LABELS.get(state.dart_opening_status, str(state.dart_opening_status))
                    print(f"当前发射站不是完全开启状态（{station}），不记录发射确认时间。")
                    continue
                state.latest_launch_cmd_time = state.stage_remain_time()
                state.dart_launch_cmd_recorded = True
                remain = state.latest_launch_cmd_time
            print(f"操作手确认发射时间已记录为 {remain} 秒。")
            continue

        print("未知命令。可用命令：s、t、f、i、q、h。")


def run_once(args: argparse.Namespace) -> int:
    state = build_state(args)
    if args.prestart_force_open_fire:
        apply_prestart_force_open_fire(state)
    now = time.monotonic()
    update_dart_state_machine(state, now)
    specs = make_message_specs(args, now)
    validate_payload_lengths(specs, state)
    builder = FrameBuilder(args.seq)

    ser = None
    try:
        if not args.dry_run:
            ser = open_serial_port(args)
        for spec in specs:
            payload = spec.payload_builder(state)
            frame = builder.build(spec.cmd_id, payload)
            send_frame(ser, frame, args.dry_run)
            if args.hex or args.dry_run:
                print(f"发送 {spec.name} 0x{spec.cmd_id:04X}: {hex_line(frame)}")
        if state.game_result is not None:
            frame = builder.build(CMD_GAME_RESULT, build_game_result_payload(state))
            send_frame(ser, frame, args.dry_run)
            if args.hex or args.dry_run:
                print(f"发送 game_result 0x{CMD_GAME_RESULT:04X}: {hex_line(frame)}")
        if ser is not None:
            ser.flush()
            drain_rx(ser, args.rx_print)
    finally:
        if ser is not None:
            ser.close()
    return 0


def run_loop(args: argparse.Namespace) -> int:
    prestart_force_open_fire = args.prestart_force_open_fire
    if not args.start_now:
        raw = input("7 分钟比赛计时开始时按回车；输入 d 进入舱门常开且允许发射调试模式：").strip().lower()
        prestart_force_open_fire = prestart_force_open_fire or raw in {"d", "debug", "3"}

    state = build_state(args)
    if prestart_force_open_fire:
        apply_prestart_force_open_fire(state)
    now = time.monotonic()
    specs = make_message_specs(args, now)
    validate_payload_lengths(specs, state)
    builder = FrameBuilder(args.seq)
    ser = None
    send_count = 0
    last_status_print = now
    game_result_sent = False
    game_result_due = None
    if state.game_result is not None and args.game_result_delay is not None:
        game_result_due = now + args.game_result_delay
    menu_thread = None
    if not args.no_runtime_menu:
        menu_thread = threading.Thread(target=runtime_menu, args=(state,), daemon=True)
        menu_thread.start()

    try:
        if not args.dry_run:
            ser = open_serial_port(args)
            print(
                f"已打开 {args.port}，波特率 {args.baudrate}，8N1。"
                "按 Ctrl+C 停止。"
            )
        else:
            print("干跑模式，不打开串口。按 Ctrl+C 停止。")

        while True:
            now = time.monotonic()
            update_dart_state_machine(state, now)
            with state.lock:
                if state.stop_requested:
                    print("运行菜单请求退出。")
                    return 0
                if state.countdown and state.stage_remain_time() == 0 and state.game_progress == 4:
                    state.game_progress = 5

            for spec in specs:
                if now < spec.next_send:
                    continue
                payload = spec.payload_builder(state)
                frame = builder.build(spec.cmd_id, payload)
                send_frame(ser, frame, args.dry_run)
                send_count += 1
                if args.hex:
                    print(f"发送 {spec.name} 0x{spec.cmd_id:04X}: {hex_line(frame)}")
                spec.next_send = now + spec.period

            if state.game_result is not None and not game_result_sent:
                if game_result_due is not None:
                    should_send_result = now >= game_result_due
                else:
                    should_send_result = state.stage_remain_time() == 0 or state.game_progress == 5
                if should_send_result:
                    with state.lock:
                        state.game_progress = 5
                    frame = builder.build(CMD_GAME_RESULT, build_game_result_payload(state))
                    send_frame(ser, frame, args.dry_run)
                    send_count += 1
                    game_result_sent = True
                    if args.hex:
                        print(f"发送 game_result 0x{CMD_GAME_RESULT:04X}: {hex_line(frame)}")

            drain_rx(ser, args.rx_print)

            if args.status_interval > 0 and now - last_status_print >= args.status_interval:
                with state.lock:
                    remain = state.stage_remain_time()
                    elapsed = int(match_elapsed_seconds(state, now))
                    station = DART_STATUS_LABELS.get(state.dart_opening_status, str(state.dart_opening_status))
                    mode = DART_REQUEST_LABELS.get(state.dart_request_mode, str(state.dart_request_mode))
                    target = state.selected_target
                    first_used = "已使用" if state.dart_first_window_used else "未使用"
                    second_used = "已使用" if state.dart_second_window_used else "未使用"
                    target_change_time = state.target_change_time
                    latest_launch_cmd_time = state.latest_launch_cmd_time
                print(
                    "状态："
                    f"已发送={send_count}, "
                    f"比赛剩余={remain}s, "
                    f"已进行={elapsed}s, "
                    f"发射站={station}, "
                    f"模式={mode}, "
                    f"目标={target}, "
                    f"机会1={first_used}, "
                    f"机会2={second_used}, "
                    f"target_change_time={target_change_time}, "
                    f"latest_launch_cmd_time={latest_launch_cmd_time}, "
                    f"seq={builder.seq}"
                )
                last_status_print = now

            next_due = min(spec.next_send for spec in specs)
            sleep_s = max(0.001, min(0.02, next_due - time.monotonic()))
            time.sleep(sleep_s)
    except KeyboardInterrupt:
        print("已停止。")
        return 0
    finally:
        if ser is not None:
            ser.close()


def nonnegative_rate(value: str) -> float:
    try:
        rate = float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"无效频率：{value!r}") from exc
    if rate <= 0:
        raise argparse.ArgumentTypeError("频率必须大于 0")
    return rate


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="RoboMaster 裁判系统串口帧模拟器，适配 Dart_Ctrl。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-p", "--port", help="串口名，例如 COM6 或 loop://")
    parser.add_argument("--baudrate", type=int_auto, default=115200, help="串口波特率")
    parser.add_argument("--write-timeout", type=float, default=0.5, help="串口写超时时间")
    parser.add_argument("--list-ports", action="store_true", help="列出本机串口后退出")
    parser.add_argument("--dry-run", action="store_true", help="不打开串口，只生成并打印帧")
    parser.add_argument("--once", action="store_true", help="每类消息发送一帧后退出")
    parser.add_argument("--menu", action="store_true", help="兼容旧参数；运行时菜单默认开启")
    parser.add_argument("--no-menu", action="store_true", help="禁用交互式串口选择")
    parser.add_argument("--no-runtime-menu", action="store_true", help="禁用计时中的飞镖选项菜单")
    parser.add_argument("--start-now", action="store_true", help="不等待回车，立即开始连续发送")
    parser.add_argument(
        "--prestart-force-open-fire",
        action="store_true",
        help="7 分钟计时开始前直接进入舱门常开且允许发射调试状态",
    )
    parser.add_argument("--hex", action="store_true", help="打印发送帧的十六进制内容")
    parser.add_argument("--rx-print", action="store_true", help="打印串口接收到的字节")
    parser.add_argument("--seq", type=int_auto, default=0, help="初始帧序号")
    parser.add_argument(
        "--status-interval",
        type=float,
        default=2.0,
        help="连续模式状态打印间隔，单位秒，0 表示关闭",
    )

    rate_group = parser.add_argument_group("消息发送频率")
    rate_group.add_argument("--game-status-hz", type=nonnegative_rate, default=1.0)
    rate_group.add_argument("--dart-info-hz", type=nonnegative_rate, default=1.0)
    rate_group.add_argument("--robot-status-hz", type=nonnegative_rate, default=10.0)
    rate_group.add_argument("--dart-launch-status-hz", type=nonnegative_rate, default=3.0)

    game_group = parser.add_argument_group("0x0001 比赛状态")
    game_group.add_argument("--game-type", type=int_auto, default=1, help="低 4 位：比赛类型，默认 1=超级对抗赛")
    game_group.add_argument("--game-progress", type=int_auto, default=4, help="高 4 位：当前比赛阶段")
    game_group.add_argument("--stage-remain", type=int_auto, default=420, help="阶段剩余时间，单位秒")
    game_group.add_argument("--sync-time", type=int_auto, default=None, help="UNIX 时间戳，默认使用当前时间")
    game_group.add_argument("--no-countdown", action="store_true", help="保持阶段剩余时间不变")

    result_group = parser.add_argument_group("0x0002 比赛结果")
    result_group.add_argument(
        "--game-result",
        type=int_auto,
        default=None,
        help="启用一次性比赛结果帧：0 平局，1 红方胜利，2 蓝方胜利",
    )
    result_group.add_argument(
        "--game-result-delay",
        type=float,
        default=None,
        help="连续模式下延迟指定秒数发送比赛结果；默认在倒计时为 0 时发送",
    )

    robot_group = parser.add_argument_group("0x0201 机器人状态")
    robot_group.add_argument("--robot-id", type=int_auto, default=8, help="红方飞镖=8，蓝方飞镖=108")
    robot_group.add_argument("--robot-level", type=int_auto, default=1)
    robot_group.add_argument("--current-hp", type=int_auto, default=400)
    robot_group.add_argument("--max-hp", type=int_auto, default=400)
    robot_group.add_argument("--cooling-value", type=int_auto, default=0)
    robot_group.add_argument("--heat-limit", type=int_auto, default=0)
    robot_group.add_argument("--chassis-power-limit", type=int_auto, default=45)
    robot_group.add_argument("--power-flags", type=int_auto, default=0x07, help="bit0 云台，bit1 底盘，bit2 发射机构")

    dart_group = parser.add_argument_group("0x0105 飞镖发射相关数据")
    dart_group.add_argument("--dart-remaining-time", type=int_auto, default=15)
    dart_group.add_argument("--last-hit-target", type=int_auto, default=0)
    dart_group.add_argument("--hit-count", type=int_auto, default=0)
    dart_group.add_argument("--selected-target", type=int_auto, default=0)

    launch_group = parser.add_argument_group("0x020A 飞镖发射站状态")
    launch_group.add_argument("--dart-opening-status", type=int_auto, default=1, help="兼容旧参数；实际状态由规则状态机生成，默认关闭")
    launch_group.add_argument("--target-change-time", type=int_auto, default=0)
    launch_group.add_argument("--latest-launch-cmd-time", type=int_auto, default=0)
    launch_group.add_argument(
        "--dart-request-mode",
        type=int_auto,
        default=DART_REQUEST_IDLE,
        help="0 等待，1 保持关闭，2 请求开启一次，3 强制常开并保持发射状态",
    )
    launch_group.add_argument("--dart-first-open-after", type=int_auto, default=30, help="第一次开启机会：比赛开始后多少秒")
    launch_group.add_argument("--dart-second-open-after", type=int_auto, default=240, help="第二次开启机会：比赛开始后多少秒，默认 240=4 分钟")
    launch_group.add_argument("--dart-open-duration", type=int_auto, default=30, help="完全开启持续时间，单位秒")
    launch_group.add_argument("--dart-transition-duration", type=int_auto, default=7, help="开启/关闭过程耗时，单位秒")
    launch_group.add_argument(
        "--no-auto-launch-confirm",
        action="store_true",
        help="关闭自动模拟操作手确认发射；关闭后需要在完全开启时手动输入 f",
    )
    launch_group.add_argument(
        "--auto-launch-confirm-delay",
        type=float,
        default=1.0,
        help="完全开启后延迟多少秒自动写入 latest_launch_cmd_time",
    )
    return parser


def validate_args(parser: argparse.ArgumentParser, args: argparse.Namespace) -> None:
    if args.list_ports:
        return
    if not args.dry_run and args.no_menu and not args.port:
        parser.error("使用 --no-menu 时必须指定 --port")

    checks = [
        ("baudrate", args.baudrate, 1, 10_000_000),
        ("seq", args.seq, 0, 255),
        ("game_type", args.game_type, 0, 15),
        ("game_progress", args.game_progress, 0, 15),
        ("stage_remain", args.stage_remain, 0, 65535),
        ("robot_id", args.robot_id, 0, 255),
        ("robot_level", args.robot_level, 0, 255),
        ("current_hp", args.current_hp, 0, 65535),
        ("max_hp", args.max_hp, 0, 65535),
        ("cooling_value", args.cooling_value, 0, 65535),
        ("heat_limit", args.heat_limit, 0, 65535),
        ("chassis_power_limit", args.chassis_power_limit, 0, 65535),
        ("power_flags", args.power_flags, 0, 7),
        ("dart_remaining_time", args.dart_remaining_time, 0, 255),
        ("last_hit_target", args.last_hit_target, 0, 7),
        ("hit_count", args.hit_count, 0, 7),
        ("selected_target", args.selected_target, 0, 7),
        ("dart_opening_status", args.dart_opening_status, 0, 2),
        ("target_change_time", args.target_change_time, 0, 65535),
        ("latest_launch_cmd_time", args.latest_launch_cmd_time, 0, 65535),
        ("dart_request_mode", args.dart_request_mode, 0, 3),
        ("dart_first_open_after", args.dart_first_open_after, 0, 65535),
        ("dart_second_open_after", args.dart_second_open_after, 0, 65535),
        ("dart_open_duration", args.dart_open_duration, 0, 65535),
        ("dart_transition_duration", args.dart_transition_duration, 0, 65535),
    ]
    if args.game_result is not None:
        checks.append(("game_result", args.game_result, 0, 2))
    if args.sync_time is not None:
        checks.append(("sync_time", args.sync_time, 0, 0xFFFFFFFFFFFFFFFF))
    if args.game_result_delay is not None and args.game_result_delay < 0:
        parser.error("game_result_delay 必须大于或等于 0")
    if args.auto_launch_confirm_delay < 0:
        parser.error("auto_launch_confirm_delay 必须大于或等于 0")
    if args.dart_second_open_after < args.dart_first_open_after:
        parser.error("dart_second_open_after 必须大于或等于 dart_first_open_after")
    if args.dart_second_open_after > args.stage_remain:
        parser.error("dart_second_open_after 不能晚于当前比赛阶段总时长")

    try:
        for check in checks:
            require_range(*check)
    except ValueError as exc:
        parser.error(str(exc))


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.list_ports:
        print_ports()
        return 0
    if not args.dry_run and not args.no_menu:
        interactive_setup(args)
    validate_args(parser, args)
    if args.once:
        return run_once(args)
    return run_loop(args)


if __name__ == "__main__":
    raise SystemExit(main())
