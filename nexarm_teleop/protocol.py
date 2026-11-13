"""NexArm 串口帧协议：组帧、解帧、主从映射。纯函数，不碰硬件。

帧格式：FF FF | ID | LEN | CMD | ARGS... | CHECKSUM
  - LEN = 参数字节数 + 2
  - CHECKSUM = ID 到最后一个参数字节求和后按位取反
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

SYSTEM_ID = 0xFF

CMD_FIRMWARE_VERSION = 1
CMD_SET_MOTION_PARAMS = 56  # 桥接模式下的加速度和速度上限
CMD_BRIDGE_MODE = 68        # 进入/退出 LeRobot 桥接模式
CMD_READ_POS = 96           # 读 6 个舵机位置，回复 6 个小端 int16
CMD_WRITE_POS = 97          # 写 6 个舵机位置，无回复
CMD_TORQUE = 98             # 力矩开关

JOINT_COUNT = 6
POSITION_MIN = 0
POSITION_MAX = 4095
COUNTS_PER_REV = 4096

MAX_FRAME_LEN = 4 + 252


@dataclass(frozen=True)
class Frame:
    device_id: int
    cmd: int
    args: bytes


def checksum(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def build_frame(cmd: int, args: bytes = b"", device_id: int = SYSTEM_ID) -> bytes:
    body = bytes([device_id & 0xFF, len(args) + 2, cmd & 0xFF]) + args
    return b"\xff\xff" + body + bytes([checksum(body)])


def _frame_at(buf: bytes | bytearray, i: int) -> tuple[Frame | None, int]:
    """尝试把 buf[i:] 解成一帧。

    返回 (帧, 帧总长)。数据还不够返回 (None, 0)，确定不是帧返回 (None, -1)。
    """
    if len(buf) < i + 4:
        return None, 0
    length = buf[i + 3]
    if length < 2:
        return None, -1
    total = 4 + length
    if len(buf) < i + total:
        return None, 0
    got = buf[i + total - 1]
    full = checksum(bytes(buf[i + 2 : i + total - 1]))
    # 主臂固件回复帧的校验和只覆盖了 ID、LEN、CMD 三个字节，两种都接受
    short = checksum(bytes(buf[i + 2 : i + 5]))
    if got != full and got != short:
        return None, -1
    return Frame(buf[i + 2], buf[i + 4], bytes(buf[i + 5 : i + total - 1])), total


class FrameParser:
    """流式解帧器。

    两块板都会往串口里打调试文本，从臂还会主动上报状态帧，
    所以解析时要能跳过任何不是帧的字节。
    """

    def __init__(self) -> None:
        self._buf = bytearray()

    def clear(self) -> None:
        self._buf.clear()

    def feed(self, data: bytes) -> list[Frame]:
        self._buf.extend(data)
        frames: list[Frame] = []
        i = 0
        keep_from: int | None = None  # 最早一个"可能是帧但还没收全"的位置
        while True:
            i = self._buf.find(b"\xff\xff", i)
            if i < 0:
                break
            frame, total = _frame_at(self._buf, i)
            if frame is not None:
                frames.append(frame)
                i += total
                keep_from = None
            elif total == 0:
                if keep_from is None:
                    keep_from = i
                i += 1
            else:
                i += 1
        if keep_from is not None:
            del self._buf[:keep_from]
            # 一个假帧头后面跟了个很大的长度字节时，不让缓冲无限增长
            if len(self._buf) > 2 * MAX_FRAME_LEN:
                del self._buf[: len(self._buf) - MAX_FRAME_LEN]
        else:
            # 只留最后一个字节，防止帧头 FF FF 正好被切成两半
            del self._buf[: max(0, len(self._buf) - 1)]
        return frames


def pack_positions(positions: list[int]) -> bytes:
    if len(positions) != JOINT_COUNT:
        raise ValueError(f"需要 {JOINT_COUNT} 个位置，收到 {len(positions)} 个")
    clamped = [max(POSITION_MIN, min(POSITION_MAX, int(p))) for p in positions]
    return struct.pack("<6h", *clamped)


def unpack_positions(args: bytes) -> list[int]:
    if len(args) < JOINT_COUNT * 2:
        raise ValueError(f"位置数据太短：{len(args)} 字节")
    return list(struct.unpack_from("<6h", args))


def map_leader_to_follower(leader: list[int]) -> list[int]:
    """主臂原始位置 → 从臂目标位置（与出厂固件一致）。

    2 号关节镜像：4096 − pos
    6 号夹爪：2833 + (pos − 2048) × 4，限制在 [1195, 2833]
    """
    out = []
    for idx, pos in enumerate(leader):
        p = int(pos)
        if idx == 1:
            p = COUNTS_PER_REV - p
        elif idx == 5:
            p = max(1195, min(2833, 2833 + (p - 2048) * 4))
        out.append(max(POSITION_MIN, min(POSITION_MAX, p)))
    return out
