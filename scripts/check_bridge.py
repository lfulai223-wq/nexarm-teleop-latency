#!/usr/bin/env python3
"""只读检查：验证驱动能读主臂，也能让从臂进桥接模式后读位置。

不发任何运动指令，不改力矩。结束时让从臂退出桥接模式（固件会重启从臂的 ESP32）。

    python3 scripts/check_bridge.py
    python3 scripts/check_bridge.py --n 200 --stay   # 多采样，并且结束后留在桥接模式
"""

from __future__ import annotations

import argparse
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from nexarm_teleop import FOLLOWER_PORT, LEADER_PORT  # noqa: E402
from nexarm_teleop.driver import NexArmPort  # noqa: E402
from nexarm_teleop.protocol import map_leader_to_follower  # noqa: E402


def sample(arm: NexArmPort, n: int) -> tuple[list[int], list[float], int]:
    durations, last, timeouts = [], [], 0
    for _ in range(n):
        try:
            r = arm.read_positions()
        except TimeoutError:
            timeouts += 1
            continue
        durations.append(r.duration_ms)
        last = r.positions
    return last, durations, timeouts


def report(name: str, positions: list[int], durations: list[float], timeouts: int) -> None:
    if not durations:
        print(f"{name}: 全部超时（{timeouts} 次）")
        return
    d = sorted(durations)
    p95 = d[min(len(d) - 1, int(0.95 * len(d)))]
    print(
        f"{name}: 位置 {positions}\n"
        f"    读 {len(d)} 次，超时 {timeouts} 次；"
        f"耗时 均值 {statistics.mean(d):.2f} ms，中位 {statistics.median(d):.2f} ms，"
        f"P95 {p95:.2f} ms，最大 {d[-1]:.2f} ms"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--n", type=int, default=50, help="每条臂读多少次")
    parser.add_argument("--stay", action="store_true", help="结束后留在桥接模式，不重启从臂")
    args = parser.parse_args()

    with NexArmPort(LEADER_PORT) as leader:
        lpos, ldur, lto = sample(leader, args.n)
    report("主臂", lpos, ldur, lto)

    with NexArmPort(FOLLOWER_PORT) as follower:
        follower.enter_bridge_mode()
        try:
            fpos, fdur, fto = sample(follower, args.n)
        finally:
            if not args.stay:
                follower.exit_bridge_mode()
    report("从臂（桥接模式）", fpos, fdur, fto)

    if lpos and fpos:
        target = map_leader_to_follower(lpos)
        diff = [f - t for f, t in zip(fpos, target)]
        print(f"主臂映射后的目标 {target}\n从臂实际 − 目标   {diff}")


if __name__ == "__main__":
    main()
