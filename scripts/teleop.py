#!/usr/bin/env python3
"""最简电脑在环遥操作：读主臂 → 映射 → 写从臂，不做任何滤波或预测。

这是延迟测量的"裸链路"基线。每个周期记一行日志（同一时钟的纳秒时间戳）。

    python3 scripts/teleop.py --duration 30
    Ctrl+C 随时停止：停止发指令，从臂保持在当前位置。

保护措施：
  - 启动时主从姿态差太大，不发运动指令，实时提示操作者把主臂摆到和从臂相近的姿态
  - 启动后用 --ramp 秒从从臂当前位置平滑过渡到主臂目标
  - 每周期每个关节的指令变化量不超过 --max-step
  - 主臂读数贴近 0 或 4095 视为坏数据，沿用上一周期的值
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from nexarm_teleop import FOLLOWER_PORT, LEADER_PORT  # noqa: E402
from nexarm_teleop.driver import NexArmPort  # noqa: E402
from nexarm_teleop.predictor import LinearPredictor  # noqa: E402
from nexarm_teleop.protocol import JOINT_COUNT, map_leader_to_follower  # noqa: E402

CORRUPT_MARGIN = 5


def looks_corrupt(positions: list[int]) -> bool:
    return any(p <= CORRUPT_MARGIN or p >= 4095 - CORRUPT_MARGIN for p in positions)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=30.0, help="运行多少秒")
    ap.add_argument("--rate", type=float, default=50.0, help="目标控制频率 (Hz)")
    ap.add_argument("--max-step", type=int, default=60, help="每周期每关节最大指令变化（计数，4096 为一圈）")
    ap.add_argument("--max-start-diff", type=int, default=500, help="启动时允许的最大主从姿态差（计数，500 约 44°）")
    ap.add_argument("--align-timeout", type=float, default=90.0, help="姿态差太大时，最多等多少秒让操作者对齐")
    ap.add_argument("--ramp", type=float, default=2.0, help="启动过渡时间（秒）")
    ap.add_argument("--acc", type=int, default=100, help="舵机加速度参数 0–254（0 为不限）")
    ap.add_argument("--speed", type=int, default=2000, help="舵机速度上限（0 为不限）")
    ap.add_argument("--lookahead", type=float, nargs="+", default=[0.0],
                    help="前馈预测的前瞻时长 (ms)，0 为关闭。给 1 个值表示各关节相同，给 6 个值表示逐关节设置")
    ap.add_argument("--vel-cutoff", type=float, default=5.0, help="速度估计的低通截止频率 (Hz)")
    ap.add_argument("--no-read-follower", action="store_true", help="不读从臂位置（更快，但日志里没有从臂实际位置）")
    ap.add_argument("--stay", action="store_true", help="结束后留在桥接模式，不重启从臂")
    ap.add_argument("--log-dir", type=Path, default=ROOT / "results" / "logs")
    args = ap.parse_args()

    period = 1.0 / args.rate
    if len(args.lookahead) not in (1, JOINT_COUNT):
        raise SystemExit(f"--lookahead 需要 1 个或 {JOINT_COUNT} 个值")
    la = [x / 1000.0 for x in (args.lookahead if len(args.lookahead) == JOINT_COUNT else args.lookahead * JOINT_COUNT)]
    predictor = LinearPredictor(JOINT_COUNT, la, args.vel_cutoff, max_lead=150.0) if max(la) > 0 else None
    args.log_dir.mkdir(parents=True, exist_ok=True)
    log_path = args.log_dir / f"teleop_{datetime.now():%Y%m%d_%H%M%S}.csv"

    header = (
        ["cycle", "t_loop_ns", "t_leader_req_ns", "t_leader_reply_ns"]
        + [f"leader_{i + 1}" for i in range(JOINT_COUNT)]
        + [f"cmd_{i + 1}" for i in range(JOINT_COUNT)]
        + ["t_write_ns", "t_follower_req_ns", "t_follower_reply_ns"]
        + [f"follower_{i + 1}" for i in range(JOINT_COUNT)]
    )

    leader = NexArmPort(LEADER_PORT)
    follower = NexArmPort(FOLLOWER_PORT)
    leader.open()
    follower.open()
    rows: list[list] = []
    bad_reads = timeouts = clamped_cycles = 0
    try:
        follower.enter_bridge_mode()
        start_follower = follower.read_positions().positions
        # 主从姿态差太大时不发任何运动指令，而是实时提示，等操作者把主臂摆到和从臂相近的姿态。
        # 夹爪不参与这项检查：它开合幅度大、不会撞到东西，启动过渡会把它慢慢带到位。
        names = ["底座", "肩", "肘", "腕俯仰", "腕旋转"]
        give_up = time.perf_counter() + args.align_timeout
        waited = False
        while True:
            start_target = map_leader_to_follower(leader.read_positions().positions)
            diff = [abs(a - b) for a, b in zip(start_follower, start_target)]
            far = [i for i in range(5) if diff[i] > args.max_start_diff]
            if not far:
                break
            if time.perf_counter() > give_up:
                print()
                raise SystemExit(f"{args.align_timeout:.0f} 秒内主从姿态没有对齐，退出。")
            parts = "，".join(f"关节{i + 1}（{names[i]}）差 {diff[i] * 360 / 4096:.0f}°" for i in far)
            print(f"\r等待对齐，请把主臂摆成和从臂一样的姿态：{parts}        ", end="", flush=True)
            waited = True
            time.sleep(0.2)
        if waited:
            print("\n已对齐。")
        print(f"从臂当前 {start_follower}\n主臂目标 {start_target}\n姿态差   {diff}")

        follower.set_motion_params(args.acc, args.speed)
        follower.set_torque(True)

        last_cmd = list(start_follower)
        last_leader = None
        t_start = time.perf_counter()
        next_tick = t_start
        cycle = 0
        la_text = "关闭" if predictor is None else "/".join(f"{x:g}" for x in args.lookahead) + " ms"
        print(f"开始遥操作 {args.duration:.0f} 秒，目标 {args.rate:.0f} Hz，预测前瞻 {la_text}。Ctrl+C 停止。")
        while True:
            now = time.perf_counter()
            elapsed = now - t_start
            if elapsed >= args.duration:
                break
            t_loop = time.perf_counter_ns()

            try:
                reading = leader.read_positions(timeout_s=0.05)
            except TimeoutError:
                timeouts += 1
                continue
            if looks_corrupt(reading.positions):
                bad_reads += 1
                if last_leader is None:
                    continue
                leader_pos = last_leader
            else:
                leader_pos = reading.positions
                last_leader = leader_pos

            target = map_leader_to_follower(leader_pos)
            if predictor is not None:
                # 用读到主臂位置的时刻（请求和回复的中点）作为这次采样的时间
                t_sample = (reading.t_request_ns + reading.t_reply_ns) / 2e9
                target = [round(x) for x in predictor.step([float(v) for v in target], t_sample)]
            # 启动过渡：从"从臂起始位置"线性混合到"主臂目标"
            if elapsed < args.ramp:
                a = elapsed / args.ramp
                target = [round(s + a * (t - s)) for s, t in zip(start_follower, target)]
            # 每周期限幅
            cmd = [c + max(-args.max_step, min(args.max_step, t - c)) for c, t in zip(last_cmd, target)]
            if cmd != target:
                clamped_cycles += 1
            t_write = follower.write_positions(cmd)
            last_cmd = cmd

            if args.no_read_follower:
                f_req = f_reply = 0
                f_pos = [0] * JOINT_COUNT
            else:
                try:
                    fr = follower.read_positions(timeout_s=0.05)
                    f_req, f_reply, f_pos = fr.t_request_ns, fr.t_reply_ns, fr.positions
                except TimeoutError:
                    timeouts += 1
                    f_req = f_reply = 0
                    f_pos = [0] * JOINT_COUNT

            rows.append(
                [cycle, t_loop, reading.t_request_ns, reading.t_reply_ns, *leader_pos, *cmd, t_write, f_req, f_reply, *f_pos]
            )
            cycle += 1

            next_tick += period
            sleep = next_tick - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.perf_counter()  # 跟不上目标频率，不追赶
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，停止发指令。")
    except ConnectionError as e:
        print(f"\n{e}\n已停止。已经采到的数据照常保存。")
    finally:
        try:
            if not args.stay:
                follower.exit_bridge_mode()
        finally:
            follower.close()
            leader.close()
        if rows:
            with log_path.open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow(header)
                w.writerows(rows)

    if len(rows) > 2:
        t = [r[1] for r in rows]
        dts = sorted((b - a) / 1e6 for a, b in zip(t, t[1:]))
        total_s = (t[-1] - t[0]) / 1e9
        print(
            f"周期数 {len(rows)}，实际频率 {(len(rows) - 1) / total_s:.1f} Hz；"
            f"周期 中位 {statistics.median(dts):.2f} ms，P95 {dts[int(0.95 * len(dts))]:.2f} ms，最大 {dts[-1]:.2f} ms"
        )
        print(f"超时 {timeouts} 次，坏读数 {bad_reads} 次，被限幅的周期 {clamped_cycles} 个")
        print(f"日志：{log_path}")


if __name__ == "__main__":
    main()
