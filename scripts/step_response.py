#!/usr/bin/env python3
"""从臂阶跃响应：不用主臂，电脑直接给从臂某个关节发一个小阶跃，高频读回位置。

用来单独测量"执行段"：舵机从收到指令到开始动（起动延迟）、到走完一半和九成的时间。
对每组（加速度, 速度上限）参数、每个关节，来回各做若干次。

    python3 scripts/step_response.py                      # 默认：关节 1、2、4，幅度约 10 度
    python3 scripts/step_response.py --joints 1 --accs 0 100 --repeats 5

所有时间都从"指令帧写入串口之前"算起，位置采样时刻取读取请求和回复的中点。
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

from nexarm_teleop import FOLLOWER_PORT  # noqa: E402
from nexarm_teleop.driver import NexArmPort  # noqa: E402

SAFE_MIN, SAFE_MAX = 300, 3800
MOVE_THRESHOLD = 4  # 计数；静止时读数抖动约 1–2 个计数


def crossing_ms(samples: list[tuple[float, float]], level: float) -> float | None:
    """samples 是 (相对指令的毫秒数, 朝目标方向走过的计数)。返回第一次越过 level 的时刻，线性插值。"""
    prev_t, prev_y = 0.0, 0.0
    for t, y in samples:
        if y >= level:
            if y == prev_y:
                return t
            return prev_t + (level - prev_y) / (y - prev_y) * (t - prev_t)
        prev_t, prev_y = t, y
    return None


def run_step(arm: NexArmPort, base: list[int], joint: int, target: int, hold_s: float) -> tuple[int, list[tuple[int, int]]]:
    """发一次阶跃并采样。返回 (指令时间戳, [(采样时间戳, 该关节位置), ...])。"""
    cmd = list(base)
    cmd[joint] = target
    # 阶跃前先读几次，得到真实的起点（舵机有稳态误差，实际位置不等于上一次的指令）
    before = [arm.read_positions(timeout_s=0.05).positions[joint] for _ in range(5)]
    p0 = statistics.mean(before)
    t_cmd = arm.write_positions(cmd)
    out = [(0, p0)]
    end = time.perf_counter() + hold_s
    while time.perf_counter() < end:
        try:
            r = arm.read_positions(timeout_s=0.05)
        except TimeoutError:
            continue
        out.append(((r.t_request_ns + r.t_reply_ns) // 2, r.positions[joint]))
    return t_cmd, out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joints", type=int, nargs="+", default=[1, 2, 4], help="关节编号 1–6")
    ap.add_argument("--amplitude", type=int, default=114, help="阶跃幅度（计数，114 约 10 度）")
    ap.add_argument("--accs", type=int, nargs="+", default=[100, 50, 20, 0, 200, 254], help="要测的加速度参数")
    ap.add_argument("--speed", type=int, default=2000, help="速度上限（0 为不限）")
    ap.add_argument("--repeats", type=int, default=3, help="每组参数来回各做几次")
    ap.add_argument("--max-short", type=float, default=14.0, help="少走超过这么多计数就认为碰到障碍，停止测试")
    ap.add_argument("--hold", type=float, default=0.8, help="每次阶跃后采样多少秒")
    ap.add_argument("--stay", action="store_true", help="结束后留在桥接模式")
    ap.add_argument("--log-dir", type=Path, default=ROOT / "results" / "logs")
    args = ap.parse_args()

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    args.log_dir.mkdir(parents=True, exist_ok=True)
    raw_path = args.log_dir / f"step_raw_{stamp}.csv"
    summary_path = ROOT / "results" / f"step_summary_{stamp}.csv"

    raw_rows, trials = [], []
    arm = NexArmPort(FOLLOWER_PORT)
    arm.open()
    try:
        arm.enter_bridge_mode()
        base = arm.read_positions().positions
        print(f"从臂起始位置 {base}")
        arm.set_torque(True)
        arm.write_positions(base)
        time.sleep(0.5)

        trial_id = 0
        for acc in args.accs:
            arm.set_motion_params(acc, args.speed)
            for joint_no in args.joints:
                j = joint_no - 1
                lo = base[j]
                hi = lo + args.amplitude
                if not (SAFE_MIN <= lo and hi <= SAFE_MAX):
                    print(f"关节 {joint_no} 位置 {lo} 离端点太近，跳过")
                    continue
                for _ in range(args.repeats):
                    for start, target in ((lo, hi), (hi, lo)):
                        t_cmd, samples = run_step(arm, base, j, target, args.hold)
                        sign = 1 if target > start else -1
                        p0 = samples[0][1]  # run_step 放在最前面的真实起点
                        rel = [((t - t_cmd) / 1e6, sign * (pos - p0)) for t, pos in samples[1:]]
                        final = statistics.mean(y for _, y in rel[-10:])
                        amp = final  # 百分比按实际走过的距离算
                        cmd_error = sign * (target - p0) - final  # 指令要求的距离 − 实际走的距离
                        trials.append(
                            {
                                "trial": trial_id,
                                "acc": acc,
                                "speed": args.speed,
                                "joint": joint_no,
                                "direction": "+" if sign > 0 else "-",
                                "travel": round(amp, 1),
                                "onset_ms": crossing_ms(rel, MOVE_THRESHOLD),
                                "t50_ms": crossing_ms(rel, 0.5 * amp),
                                "t90_ms": crossing_ms(rel, 0.9 * amp),
                                "overshoot": max(y for _, y in rel) - amp,
                                "final_error": cmd_error,
                                "samples": len(rel),
                            }
                        )
                        raw_rows += [[trial_id, acc, args.speed, joint_no, sign, round(t, 3), y] for t, y in rel]
                        trial_id += 1
                        if cmd_error > args.max_short:
                            # 差这么多走不到位，多半是碰到了东西或被顶住。立刻回起点并停止，不要让舵机继续顶着。
                            arm.write_positions(base)
                            time.sleep(0.5)
                            raise SystemExit(
                                f"关节 {joint_no} 方向 {'+' if sign > 0 else '-'} 少走了 {cmd_error:.0f} 计数（上限 {args.max_short}），"
                                "可能碰到障碍物，已回到起点并停止。请把从臂抬到四周有空间的姿态再测。"
                            )
        arm.set_motion_params(100, 2000)
        arm.write_positions(base)
        time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，停止。")
    except ConnectionError as e:
        print(f"\n{e}\n已停止。已经采到的数据照常保存。")
    finally:
        try:
            if not args.stay:
                arm.exit_bridge_mode()
        finally:
            arm.close()

    if not trials:
        return
    with raw_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trial", "acc", "speed", "joint", "sign", "t_ms", "travel"])
        w.writerows(raw_rows)
    with summary_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(trials[0]))
        w.writeheader()
        w.writerows(trials)

    def med(rows, key):
        vals = [r[key] for r in rows if r[key] is not None]
        return f"{statistics.median(vals):7.1f}" if vals else "    ---"

    print(f"\n幅度 {args.amplitude} 计数，速度上限 {args.speed}，每格为 {2 * args.repeats} 次的中位数")
    print(" acc  关节 | 起动(ms)  到50%(ms)  到90%(ms)  超调(计数)  未走完(计数)")
    for acc in args.accs:
        for joint_no in args.joints:
            rows = [r for r in trials if r["acc"] == acc and r["joint"] == joint_no]
            if rows:
                print(
                    f"{acc:4d}  {joint_no:3d}  | {med(rows, 'onset_ms')}   {med(rows, 't50_ms')}    {med(rows, 't90_ms')}   "
                    f"{med(rows, 'overshoot')}     {med(rows, 'final_error')}"
                )
    print(f"\n原始采样：{raw_path}\n汇总表：  {summary_path}")


if __name__ == "__main__":
    main()
