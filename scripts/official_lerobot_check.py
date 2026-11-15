#!/usr/bin/env python3
"""用官方 LeRobot 的 SO-101 主臂驱动连接 NexArm 主臂，看它停在哪一步。

需要在装了 LeRobot 和飞特 SDK（feetech-servo-sdk）的环境里运行，例如商家 LeRobot 的虚拟环境：

    vendor-nexarm/finsh/.venv/bin/python scripts/official_lerobot_check.py

调用的是官方 `SO101Leader.connect()`，和 `lerobot-teleoperate --teleop.type=so101_leader` 连接时走的是同一段代码。
为了不改写舵机参数，在串口发送处加了一道拦截：只放行 PING、READ、SYNC_READ 三种只读报文，
其他任何报文（写寄存器、关力矩等）都在发出之前报错，不会到达舵机。
"""

from __future__ import annotations

import collections
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig  # noqa: E402

from nexarm_teleop import LEADER_PORT  # noqa: E402

READ_ONLY = {0x01: "PING", 0x02: "READ", 0x82: "SYNC_READ"}


class BlockedWrite(RuntimeError):
    pass


def guard(port_handler, sent: collections.Counter) -> None:
    """包住 SDK 的 writePort：只读报文照常发出，其他报文直接报错。"""
    raw_write = port_handler.writePort

    def write(packet):
        inst = packet[4] if len(packet) > 4 else None
        if inst not in READ_ONLY:
            raise BlockedWrite(f"拦截了一条非只读报文（指令 0x{inst:02x}）：{bytes(packet).hex(' ')}")
        sent[READ_ONLY[inst]] += 1
        return raw_write(packet)

    port_handler.writePort = write


def main() -> None:
    teleop = SO101Leader(SO101LeaderConfig(port=LEADER_PORT, id="nexarm_official_check"))
    bus = teleop.bus
    sent: collections.Counter = collections.Counter()
    guard(bus.port_handler, sent)

    print(f"官方驱动期望的舵机：{ {m.id: (m.model, bus.model_number_table[m.model]) for m in bus.motors.values()} }")
    print("\n1) 官方 SO101Leader.connect()")
    try:
        teleop.connect()
        print("   连接成功（出乎意料）")
    except BlockedWrite as e:
        print(f"   握手通过，但随后要写舵机，已拦截：{e}")
    except Exception as e:  # noqa: BLE001
        print(f"   停在握手：{type(e).__name__}")
        print("   " + str(e).replace("\n", "\n   "))

    # 第二道检查：官方握手在型号检查之后还要求所有舵机固件版本一致。单独跑一次，看型号这一关过了之后会不会也停在这里。
    print("\n2) 握手的第二道检查（固件版本一致）")
    try:
        if not bus.port_handler.is_open:
            bus.port_handler.openPort()
        bus._assert_same_firmware()
        print("   通过")
    except Exception as e:  # noqa: BLE001
        print(f"   不通过：{type(e).__name__}")
        print("   " + str(e).replace("\n", "\n   "))
    finally:
        bus.port_handler.closePort()

    print(f"\n发出的报文：{dict(sent)}；非只读报文 0 条到达舵机。")


if __name__ == "__main__":
    main()
