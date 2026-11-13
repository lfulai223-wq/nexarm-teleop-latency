#!/usr/bin/env python3
"""汇总多次回放日志：按前瞻时长分组，把各次回放的分窗滞后合在一起统计。

    python scripts/analyze_replay.py results/logs/batch1/replay_*.csv

输出每个关节在每档前瞻下的：滞后均值 / P50 / P95、跟踪误差、抖动，并存成 CSV。
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from nexarm_teleop.latency import stale_mask, summarize, windowed_lags_ms  # noqa: E402
from nexarm_teleop.protocol import JOINT_COUNT  # noqa: E402


def load(path: Path):
    a = np.loadtxt(path, delimiter=",", skiprows=1)
    t_ref = (a[:, 0] - a[0, 0]) / 1e9
    t_meas = ((a[:, 14] + a[:, 15]) / 2 - a[0, 0]) / 1e9
    return t_ref, a[:, 1:7], t_meas, a[:, 16:22]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", type=Path, nargs="+")
    ap.add_argument("--out", type=Path, default=None, help="汇总 CSV 路径")
    ap.add_argument("--joints", type=int, nargs="+", default=[1, 2, 3, 4, 5], help="参与合并统计的关节（夹爪受限速支配，默认不含）")
    ap.add_argument("--keep-stale", action="store_true", help="不剔除卡住的读数（用于对比）")
    args = ap.parse_args()
    args.pool_label = "+".join(str(j) for j in args.joints)

    groups: dict[float, list[Path]] = defaultdict(list)
    for p in args.logs:
        m = re.search(r"replay_la([\d.]+)_", p.name)
        if m:
            groups[float(m.group(1))].append(p)

    out_rows = []
    print("前瞻(ms) 关节 回放次数 窗数 | 滞后 均值 / P50 / P95 (ms) | 跟踪误差RMS | 抖动RMS")
    for la in sorted(groups):
        pooled = np.array([])
        for j in range(JOINT_COUNT):
            lags, errs, jits, stale = [], [], [], 0
            for p in groups[la]:
                t_ref, ref, t_meas, meas = load(p)
                if np.ptp(ref[:, j]) < 80:
                    continue
                # 剔除从臂返回旧值的采样
                bad = stale_mask(meas[:, j]) if not args.keep_stale else np.zeros(len(t_meas), dtype=bool)
                stale += int(bad.sum())
                tm, mj = t_meas[~bad], meas[~bad, j]
                lags.append(windowed_lags_ms(t_ref, ref[:, j], tm, mj))
                e = mj - np.interp(tm, t_ref, ref[:, j])
                errs.append(float(np.sqrt(np.mean((e - e.mean()) ** 2))))
                # 抖动按均匀时间轴算：被剔除的位置用前后真实读数线性补上
                filled = np.interp(t_meas, tm, mj)
                jits.append(float(np.sqrt(np.mean(np.diff(filled, 2) ** 2))))
            if not lags:
                continue
            all_lags = np.concatenate(lags)
            if j + 1 in args.joints:
                pooled = np.concatenate([pooled, all_lags])
            s = summarize(all_lags)
            row = {
                "lookahead_ms": la, "joint": j + 1, "runs": len(lags), "windows": s["n"], "stale_samples": stale,
                "lag_mean_ms": round(s["mean"], 1), "lag_p50_ms": round(s["p50"], 1), "lag_p95_ms": round(s["p95"], 1),
                "track_rms": round(float(np.mean(errs)), 1), "jitter_rms": round(float(np.mean(jits)), 2),
            }
            out_rows.append(row)
            print(
                f"  {la:5g}    {j + 1}     {len(lags)}     {s['n']:3d}  |  {s['mean']:6.1f} / {s['p50']:6.1f} / {s['p95']:6.1f}"
                f"       |   {row['track_rms']:6.1f}    |  {row['jitter_rms']:5.2f}" + (f"   (剔除 {stale} 个旧值)" if stale else "")
            )
        s = summarize(pooled)
        tr = np.mean([r["track_rms"] for r in out_rows if r["lookahead_ms"] == la and r["joint"] in args.joints])
        ji = np.mean([r["jitter_rms"] for r in out_rows if r["lookahead_ms"] == la and r["joint"] in args.joints])
        out_rows.append({
            "lookahead_ms": la, "joint": args.pool_label, "runs": len(groups[la]), "windows": s["n"],
            "stale_samples": sum(r["stale_samples"] for r in out_rows if r["lookahead_ms"] == la),
            "lag_mean_ms": round(s["mean"], 1), "lag_p50_ms": round(s["p50"], 1), "lag_p95_ms": round(s["p95"], 1),
            "track_rms": round(float(tr), 1), "jitter_rms": round(float(ji), 2),
        })
        print(f"  {la:5g}   合并   {len(groups[la])}     {s['n']:3d}  |  {s['mean']:6.1f} / {s['p50']:6.1f} / {s['p95']:6.1f}       |   {tr:6.1f}    |  {ji:5.2f}\n")

    if args.out and out_rows:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        with args.out.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(out_rows[0]))
            w.writeheader()
            w.writerows(out_rows)
        print(f"汇总表：{args.out}")


if __name__ == "__main__":
    main()
