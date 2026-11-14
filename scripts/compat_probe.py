#!/usr/bin/env python3
"""兼容性探测：把官方 LeRobot SO-101 驱动连接时会发的飞特（Feetech）舵机报文，原样发给 NexArm 的 USB 口，看有没有回复。

官方 so101_follower / so101_leader 用 FeetechMotorsBus 直接和舵机总线对话：连接时先 PING 每个舵机、
读型号寄存器核对型号，然后同步读写位置。这里只发其中只读的几条（PING、读型号、读当前位置），
不写任何寄存器、不改力矩、不切模式。

    python scripts/compat_probe.py

只在两条臂都处于普通模式（不在桥接模式）时运行。
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import serial

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from nexarm_teleop import FOLLOWER_PORT, LEADER_PORT  # noqa: E402
from nexarm_teleop.protocol import CMD_READ_POS, FrameParser, build_frame  # noqa: E402

# 飞特 SCS/STS 协议：FF FF ID LEN INSTR PARAMS CHECKSUM，校验和为 ID 到最后一个参数求和取反
INSTR_PING = 0x01
INSTR_READ = 0x02
INSTR_SYNC_READ = 0x82
ADDR_MODEL_NUMBER = 3      # 2 字节；STS3215 的型号为 777
ADDR_PRESENT_POSITION = 56  # 2 字节


def feetech_packet(servo_id: int, instr: int, params: bytes = b"") -> bytes:
    body = bytes([servo_id, len(params) + 2, instr]) + params
    return b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF])


def status_packets(buf: bytes, servo_id: int) -> list[bytes]:
    """在收到的字节里找来自该舵机 ID 的状态包（校验和正确的才算）。"""
    found = []
    i = buf.find(b"\xff\xff")
    while 0 <= i and i + 4 <= len(buf):
        if buf[i + 2] == servo_id:
            length = buf[i + 3]
            end = i + 4 + length
            if 2 <= length < 64 and end <= len(buf):
                pkt = buf[i:end]
                if (~sum(pkt[2:-1])) & 0xFF == pkt[-1]:
                    found.append(pkt)
        i = buf.find(b"\xff\xff", i + 1)
    return found


def exchange(port: serial.Serial, packet: bytes, wait_s: float = 0.15) -> bytes:
    port.reset_input_buffer()
    port.write(packet)
    end = time.perf_counter() + wait_s
    buf = b""
    while time.perf_counter() < end:
        buf += port.read(port.in_waiting or 1)
    return buf


def read_u16(port: serial.Serial, servo_id: int, addr: int) -> int | None:
    replies = status_packets(exchange(port, feetech_packet(servo_id, INSTR_READ, bytes([addr, 2]))), servo_id)
    return int.from_bytes(replies[0][5:7], "little") if replies else None


def in_bridge_mode(port: serial.Serial) -> bool:
    """只有桥接模式下从臂才回复 96 号读位置指令；普通模式下不回复。

    （版本查询不是可靠的判据：刚上电时普通模式下它也可能不回复。）
    """
    raw = exchange(port, build_frame(CMD_READ_POS), 0.3)
    return any(f.cmd == CMD_READ_POS for f in FrameParser().feed(raw))


def probe(name: str, path: str, check_mode: bool) -> None:
    print(f"\n== {name}  {path}")
    with serial.Serial(path, 1_000_000, timeout=0.01) as port:
        time.sleep(0.3)
        if check_mode and in_bridge_mode(port):
            print("   处于桥接模式，跳过（先断电重启或退出桥接模式再测）")
            return
        print("   舵机 | PING | 型号（STS3215 为 777） | 当前位置")
        for sid in range(1, 7):
            ping = bool(status_packets(exchange(port, feetech_packet(sid, INSTR_PING)), sid))
            model = read_u16(port, sid, ADDR_MODEL_NUMBER)
            pos = read_u16(port, sid, ADDR_PRESENT_POSITION)
            print(f"   ID{sid}  | {'有回复' if ping else '无回复'} | {model if model is not None else '无回复':>8} | {pos if pos is not None else '无回复'}")
        # 官方驱动每个控制周期用一条 SYNC_READ 读所有舵机的位置
        body = bytes([0xFE, 2 + 2 + 6, INSTR_SYNC_READ, ADDR_PRESENT_POSITION, 2, 1, 2, 3, 4, 5, 6])
        raw = exchange(port, b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF]), 0.2)
        got = {sid: status_packets(raw, sid) for sid in range(1, 7)}
        values = {sid: (int.from_bytes(v[0][5:7], "little") if v else None) for sid, v in got.items()}
        print(f"   SYNC_READ 当前位置：{values}")


def main() -> None:
    probe("主臂", LEADER_PORT, check_mode=False)
    probe("从臂", FOLLOWER_PORT, check_mode=True)


if __name__ == "__main__":
    main()
