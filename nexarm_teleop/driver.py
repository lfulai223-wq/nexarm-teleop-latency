"""NexArm 串口驱动：每次收发都带时间戳，供延迟测量使用。

时间戳统一用 time.perf_counter_ns()，主臂和从臂在同一个时钟下。
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import serial

from . import protocol as p

BAUDRATE = 1_000_000


class ConnectionLost(ConnectionError):
    """串口在运行中断开（通常是 USB 掉线后被系统重新枚举）。"""


@dataclass(frozen=True)
class Reading:
    """一次读位置的结果。"""

    positions: list[int]
    t_request_ns: int  # 请求帧写入串口之前
    t_reply_ns: int    # 回复帧完整解析出来之后

    @property
    def duration_ms(self) -> float:
        return (self.t_reply_ns - self.t_request_ns) / 1e6


class NexArmPort:
    """一条臂的串口连接。主臂和从臂共用。"""

    def __init__(self, port: str, baudrate: int = BAUDRATE):
        self.port_name = port
        self.baudrate = baudrate
        self._serial: serial.Serial | None = None
        self._parser = p.FrameParser()

    # ---- 连接 ----

    def open(self, settle_s: float = 0.2) -> None:
        self._serial = serial.Serial(self.port_name, self.baudrate, timeout=0.01, write_timeout=0.2)
        time.sleep(settle_s)
        self._drain()

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None

    def __enter__(self) -> "NexArmPort":
        self.open()
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ---- 底层收发 ----

    def _drain(self) -> None:
        """丢掉串口里已有的数据（调试文本、主动上报帧）。"""
        assert self._serial is not None
        self._serial.reset_input_buffer()
        self._parser.clear()

    def send(self, cmd: int, args: bytes = b"") -> int:
        """发一帧，不等回复。返回写入前的时间戳。"""
        assert self._serial is not None, "串口未打开"
        frame = p.build_frame(cmd, args)
        t = time.perf_counter_ns()
        try:
            self._serial.write(frame)
        except (serial.SerialException, OSError) as e:
            raise ConnectionLost(f"{self.port_name}: 串口写入失败，USB 可能已断开（{e}）") from e
        return t

    def request(self, cmd: int, args: bytes = b"", timeout_s: float = 0.15) -> tuple[p.Frame, int, int]:
        """发一帧并等同一指令号的回复。返回 (回复帧, 发送时间戳, 收到时间戳)。"""
        assert self._serial is not None, "串口未打开"
        try:
            self._drain()
        except (serial.SerialException, OSError) as e:
            raise ConnectionLost(f"{self.port_name}: 串口异常，USB 可能已断开（{e}）") from e
        t_request = self.send(cmd, args)
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            # 至少读 1 个字节（最多阻塞 10 ms），有多少读多少
            try:
                data = self._serial.read(max(1, self._serial.in_waiting))
            except (serial.SerialException, OSError) as e:
                raise ConnectionLost(f"{self.port_name}: 串口读取失败，USB 可能已断开（{e}）") from e
            if not data:
                continue
            for frame in self._parser.feed(data):
                if frame.cmd == cmd:
                    return frame, t_request, time.perf_counter_ns()
        raise TimeoutError(f"{self.port_name}: 指令 {cmd} 在 {timeout_s * 1000:.0f} ms 内没有回复")

    # ---- 指令 ----

    def read_positions(self, timeout_s: float = 0.15) -> Reading:
        frame, t_request, t_reply = self.request(p.CMD_READ_POS, timeout_s=timeout_s)
        return Reading(p.unpack_positions(frame.args), t_request, t_reply)

    def write_positions(self, positions: list[int]) -> int:
        """写 6 个目标位置（无回复）。返回写入前的时间戳。"""
        return self.send(p.CMD_WRITE_POS, p.pack_positions(positions))

    def set_torque(self, enable: bool) -> None:
        self.send(p.CMD_TORQUE, bytes([1 if enable else 0]))
        time.sleep(0.05)

    def set_motion_params(self, acc: int, speed: int = 0) -> None:
        """桥接模式下的舵机加速度（0–254，0 为不限）和速度上限（0 为不限）。"""
        acc = max(0, min(254, int(acc)))
        speed = max(0, min(3400, int(speed)))
        self.send(p.CMD_SET_MOTION_PARAMS, bytes([acc, speed & 0xFF, speed >> 8]))
        time.sleep(0.02)

    def enter_bridge_mode(self, ready_timeout_s: float = 12.0) -> None:
        """从臂专用：让 ESP32 把电脑的读写指令转发给舵机协处理器。

        从臂刚重启时要几秒才能响应，所以这里反复尝试，直到能读到位置为止。
        """
        deadline = time.perf_counter() + ready_timeout_s
        while True:
            self.send(p.CMD_BRIDGE_MODE, bytes([1]))
            time.sleep(0.3)
            self._drain()
            try:
                self.read_positions()
                return
            except TimeoutError:
                if time.perf_counter() > deadline:
                    raise TimeoutError(f"{self.port_name}: 进入桥接模式后 {ready_timeout_s:.0f} 秒内读不到位置") from None
                time.sleep(0.5)

    def exit_bridge_mode(self) -> bool:
        """从臂专用：退出桥接模式。固件的做法是直接重启 ESP32。

        用在清理阶段，所以串口已经断开时不抛异常，只返回 False。
        """
        try:
            self.send(p.CMD_BRIDGE_MODE, bytes([0]))
        except ConnectionLost:
            return False
        time.sleep(0.1)
        return True
