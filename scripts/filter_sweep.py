#!/usr/bin/env python3
"""离线比较输入滤波参数：不接硬件，只看滤波和预测对"发给从臂的指令"做了什么。

把录好的主臂轨迹按 50 Hz 控制周期过一遍滤波（和预测），对每个配置算：
  - 指令滞后：指令相对原始轨迹的时间偏移（与回放分析同一套分窗拟合；负数表示指令超前）；
  - 指令抖动：指令二阶差分的均方根（计数），与回放里从臂的抖动指标同一定义。
另外对每个"滤波 + 逐关节预测"配置，找一个把前瞻整体缩短、使指令超前相同的"只预测"配置作对照：
如果滤波值得用，同样的超前下它的抖动应该更小。

    python scripts/filter_sweep.py results/logs/teleop_XXXX.csv [更多轨迹...] --out results/filter_sweep_offline.csv

离线结果只说明指令层面的取舍；从臂真正的滞后和抖动要回放实测（scripts/replay.py --lowpass / --one-euro）。
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))

from replay import load_trajectory  # noqa: E402

from nexarm_teleop.filters import LowPassFilter, OneEuroFilter  # noqa: E402
from nexarm_teleop.latency import summarize, windowed_lags_ms  # noqa: E402
from nexarm_teleop.predictor import LinearPredictor  # noqa: E402

PER_JOINT_S = [0.065, 0.110, 0.080, 0.065, 0.095, 0.0]  # 推荐的逐关节前瞻
JOINTS = range(5)  # 关节 1–5（夹爪不参与，和回放统计一致）
PERIOD = 0.02

CONFIGS = {
    "none": {},
    "LP 5 Hz": {"lp": 5},
    "LP 10 Hz": {"lp": 10},
    "LP 15 Hz": {"lp": 15},
    "LP 20 Hz": {"lp": 20},
    "OE 3/0.0044 + LP 5 Hz (carried over)": {"oe": (3, 0.0044), "lp": 5},
    "OE 3/0.0044": {"oe": (3, 0.0044)},
    "OE 1/0.02": {"oe": (1, 0.02)},
    "OE 5/0.02": {"oe": (5, 0.02)},
}


def simulate(t_grid, ref, cfg: dict, lookahead_s: list[float] | None):
    stages = []
    if "oe" in cfg:
        stages.append(OneEuroFilter(6, *cfg["oe"]))
    if "lp" in cfg:
        stages.append(LowPassFilter(6, cfg["lp"]))
    if lookahead_s is not None:
        stages.append(LinearPredictor(6, lookahead_s, 5.0, max_lead=150.0))
    out = []
    for t, x in zip(t_grid, ref):
        y = list(x)
        for s in stages:
            y = s.step(y, t)
        out.append(y)
    out = np.array(out)
    lags = np.concatenate([windowed_lags_ms(t_grid, ref[:, j], t_grid, out[:, j]) for j in JOINTS])
    jitter = float(np.mean([np.sqrt(np.mean(np.diff(out[:, j], 2) ** 2)) for j in JOINTS]))
    return summarize(lags)["mean"], jitter


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trajectories", type=Path, nargs="+")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    rows = []
    for path in args.trajectories:
        t_traj, target = load_trajectory(path, skip_s=2.5)
        t_grid = np.arange(0.0, t_traj[-1], PERIOD)
        ref = np.stack([np.interp(t_grid, t_traj, target[:, j]) for j in range(6)], axis=1)
        print(f"\n{path.name}")
        print(f"  {'配置':38s} {'指令滞后(ms)':>12s} {'指令抖动':>8s} | 同等超前的只预测对照：前瞻缩放 滞后 抖动")
        for name, cfg in CONFIGS.items():
            for pred in (False, True):
                lag, jit = simulate(t_grid, ref, cfg, PER_JOINT_S if pred else None)
                label = name + (" + per-joint" if pred else "")
                note = ""
                row = {"trajectory": path.name, "config": label, "cmd_lag_ms": round(lag, 1), "cmd_jitter": round(jit, 2)}
                if pred and cfg and lag < 0:
                    # 前瞻缩放 s 越大，指令越超前（滞后越负）；二分找到与"滤波 + 预测"超前相同的 s
                    lo, hi = 0.0, 1.0
                    for _ in range(12):
                        s = (lo + hi) / 2
                        lag_s, jit_s = simulate(t_grid, ref, {}, [x * s for x in PER_JOINT_S])
                        lo, hi = (s, hi) if lag_s > lag else (lo, s)
                    note = f"x{s:.2f} {lag_s:6.1f} {jit_s:5.2f}"
                    row.update(equal_lead_scale=round(s, 2), equal_lead_cmd_lag_ms=round(lag_s, 1), equal_lead_cmd_jitter=round(jit_s, 2))
                elif pred and cfg:
                    note = "指令已不超前，无对照"
                print(f"  {label:38s} {lag:12.1f} {jit:8.2f} | {note}")
                rows.append(row)

    if args.out:
        keys = ["trajectory", "config", "cmd_lag_ms", "cmd_jitter", "equal_lead_scale", "equal_lead_cmd_lag_ms", "equal_lead_cmd_jitter"]
        with args.out.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
        print(f"\n已保存 {args.out}")


if __name__ == "__main__":
    main()
