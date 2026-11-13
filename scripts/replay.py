#!/usr/bin/env python3
"""回放一段录好的主臂轨迹，当作"虚拟主臂"驱动从臂。

人手拖动每次都不一样，没法公平比较补偿前后的效果；回放同一段轨迹就可以。
轨迹文件可以是 scripts/teleop.py 的日志（用其中的主臂位置和时间戳）。

    python scripts/replay.py results/logs/teleop_XXXX.csv                 # 无处理
    python scripts/replay.py results/logs/teleop_XXXX.csv --lookahead 60  # 加 60 ms 前馈预测

注意：回放不经过真实主臂，所以不含"读主臂"那段延迟（单独测得约 13 ms）。
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from nexarm_teleop import FOLLOWER_PORT  # noqa: E402
from nexarm_teleop.driver import NexArmPort  # noqa: E402
from nexarm_teleop.latency import best_lag_ms, summarize, windowed_lags_ms  # noqa: E402
from nexarm_teleop.predictor import LinearPredictor  # noqa: E402
from nexarm_teleop.protocol import JOINT_COUNT, map_leader_to_follower  # noqa: E402


def load_trajectory(path: Path, skip_s: float) -> tuple[np.ndarray, np.ndarray]:
    """读 teleop 日志，返回 (时间秒, 从臂坐标系下的目标位置 N×6)。"""
    with path.open() as f:
        rows = list(csv.DictReader(f))
    t = np.array([(int(r["t_leader_req_ns"]) + int(r["t_leader_reply_ns"])) / 2e9 for r in rows])
    leader = [[int(float(r[f"leader_{i + 1}"])) for i in range(JOINT_COUNT)] for r in rows]
    target = np.array([map_leader_to_follower(p) for p in leader], dtype=float)
    keep = t - t[0] >= skip_s
    t, target = t[keep], target[keep]
    return t - t[0], target


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trajectory", type=Path)
    ap.add_argument("--rate", type=float, default=50.0, help="控制频率 (Hz)")
    ap.add_argument("--lookahead", type=float, nargs="+", default=[0.0],
                    help="前馈预测的前瞻时长 (ms)，0 为关闭。给 1 个值表示各关节相同，给 6 个值表示逐关节设置")
    ap.add_argument("--vel-cutoff", type=float, default=5.0, help="速度估计的低通截止频率 (Hz)")
    ap.add_argument("--skip", type=float, default=2.5, help="跳过轨迹开头多少秒（原日志的启动过渡段）")
    ap.add_argument("--ramp", type=float, default=2.0, help="先用多少秒把从臂移到轨迹起点")
    ap.add_argument("--max-step", type=int, default=80, help="每周期每关节最大指令变化（计数）")
    ap.add_argument("--max-start-diff", type=int, default=600)
    ap.add_argument("--label", type=str, default="")
    ap.add_argument("--stay", action="store_true")
    ap.add_argument("--log-dir", type=Path, default=ROOT / "results" / "logs")
    args = ap.parse_args()

    t_traj, target = load_trajectory(args.trajectory, args.skip)
    duration = float(t_traj[-1])
    period = 1.0 / args.rate
    if len(args.lookahead) not in (1, JOINT_COUNT):
        raise SystemExit(f"--lookahead 需要 1 个或 {JOINT_COUNT} 个值")
    per_joint = len(args.lookahead) == JOINT_COUNT
    la_list = [x / 1000.0 for x in (args.lookahead if per_joint else args.lookahead * JOINT_COUNT)]
    la_text = "逐关节 " + "/".join(f"{x:g}" for x in args.lookahead) if per_joint else f"{args.lookahead[0]:g}"
    predictor = LinearPredictor(JOINT_COUNT, la_list, args.vel_cutoff, max_lead=150.0) if max(la_list) > 0 else None

    def target_at(t: float) -> list[float]:
        return [float(np.interp(t, t_traj, target[:, j])) for j in range(JOINT_COUNT)]

    rows = []
    clamped = timeouts = 0
    arm = NexArmPort(FOLLOWER_PORT)
    arm.open()
    try:
        arm.enter_bridge_mode()
        start = arm.read_positions().positions
        first = target_at(0.0)
        diff = max(abs(a - b) for a, b in zip(start, first))
        if diff > args.max_start_diff:
            raise SystemExit(f"从臂当前位置与轨迹起点相差 {diff:.0f} 计数，超过 {args.max_start_diff}，拒绝启动。")
        arm.set_motion_params(100, 2000)
        arm.set_torque(True)

        # 先平滑移到轨迹起点
        steps = max(1, int(args.ramp * args.rate))
        for k in range(1, steps + 1):
            a = k / steps
            arm.write_positions([round(s + a * (f - s)) for s, f in zip(start, first)])
            time.sleep(period)
        time.sleep(0.5)

        last_cmd = [float(round(x)) for x in first]
        t0 = time.perf_counter()
        next_tick = t0
        while True:
            now = time.perf_counter() - t0
            if now >= duration:
                break
            ref = target_at(now)
            t_ref_ns = time.perf_counter_ns()
            want = predictor.step(ref, now) if predictor else ref
            cmd = [c + max(-args.max_step, min(args.max_step, w - c)) for c, w in zip(last_cmd, want)]
            if any(abs(c - w) > 0.5 for c, w in zip(cmd, want)):
                clamped += 1
            cmd_int = [round(c) for c in cmd]
            t_write = arm.write_positions(cmd_int)
            last_cmd = cmd
            try:
                fr = arm.read_positions(timeout_s=0.05)
            except TimeoutError:
                timeouts += 1
                continue
            rows.append([t_ref_ns, *[round(x, 2) for x in ref], *cmd_int, t_write, fr.t_request_ns, fr.t_reply_ns, *fr.positions])
            next_tick += period
            sleep = next_tick - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_tick = time.perf_counter()
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，停止。")
    finally:
        try:
            if not args.stay:
                arm.exit_bridge_mode()
        finally:
            arm.close()

    if len(rows) < 50:
        return
    tag = f"la{args.label}" if args.label else ("laPJ" if per_joint else f"la{args.lookahead[0]:g}")
    args.log_dir.mkdir(parents=True, exist_ok=True)
    log_path = args.log_dir / f"replay_{tag}_{datetime.now():%Y%m%d_%H%M%S}.csv"
    header = (
        ["t_ref_ns"]
        + [f"ref_{i + 1}" for i in range(JOINT_COUNT)]
        + [f"cmd_{i + 1}" for i in range(JOINT_COUNT)]
        + ["t_write_ns", "t_follower_req_ns", "t_follower_reply_ns"]
        + [f"follower_{i + 1}" for i in range(JOINT_COUNT)]
    )
    with log_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

    a = np.array(rows, dtype=float)
    t_ref = (a[:, 0] - a[0, 0]) / 1e9
    t_meas = ((a[:, 14] + a[:, 15]) / 2 - a[0, 0]) / 1e9
    print(f"\n前瞻 {la_text} ms；{len(rows)} 个周期，实际 {(len(rows) - 1) / t_ref[-1]:.1f} Hz；超时 {timeouts}，被限幅 {clamped}")
    print("关节 | 整段滞后(ms) | 分窗滞后 均值/P50/P95 (ms, 窗数) | 跟踪误差RMS(计数) | 抖动RMS(计数)")
    for j in range(JOINT_COUNT):
        ref, meas = a[:, 1 + j], a[:, 16 + j]
        if np.ptp(ref) < 80:
            print(f"  {j + 1}  | 运动范围太小，跳过")
            continue
        lag, _ = best_lag_ms(t_ref, ref, t_meas, meas)
        s = summarize(windowed_lags_ms(t_ref, ref, t_meas, meas))
        # 跟踪误差：从臂实测位置与"同一时刻"主臂目标之差（去掉常值偏差）
        err = meas - np.interp(t_meas, t_ref, ref)
        err = err - err.mean()
        # 抖动：从臂位置二阶差分的均方根，反映高频不平滑
        jitter = float(np.sqrt(np.mean(np.diff(meas, 2) ** 2)))
        print(
            f"  {j + 1}  |   {lag:6.0f}     |  {s['mean']:6.1f} / {s['p50']:6.1f} / {s['p95']:6.1f}  ({s['n']:2d})        |"
            f"     {np.sqrt(np.mean(err**2)):6.1f}       |   {jitter:5.2f}"
        )
    print(f"日志：{log_path}")


if __name__ == "__main__":
    main()
