#!/usr/bin/env python3
"""Generate the README figures from the result summaries.

    python scripts/make_figures.py

Inputs:  results/replay_summary_batch1.csv, results/replay_summary_batch2_perjoint.csv,
         results/logs/step_raw_20261113_052916.csv (raw log, local only)
Outputs: docs/img/lookahead_sweep.png, docs/img/per_joint_lag.png, docs/img/step_response.png
"""

from __future__ import annotations

import csv
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "img"

SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
TEXT_2 = "#52514e"
GRID = "#e6e5e1"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a"]  # categorical slots 1-3, fixed order

plt.rcParams.update(
    {
        "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
        "font.size": 10, "text.color": TEXT, "axes.labelcolor": TEXT_2,
        "xtick.color": TEXT_2, "ytick.color": TEXT_2, "axes.edgecolor": GRID,
        "axes.spines.top": False, "axes.spines.right": False, "axes.spines.left": False,
        "axes.grid": True, "axes.grid.axis": "y", "grid.color": GRID, "grid.linewidth": 1.0,
        "axes.axisbelow": True, "xtick.major.size": 0, "ytick.major.size": 0,
        "axes.titlesize": 11, "axes.titleweight": "bold", "axes.titlelocation": "left",
        "legend.frameon": False,
    }
)


def read(path: Path) -> list[dict]:
    with path.open() as f:
        return list(csv.DictReader(f))


def line(ax, x, y, color, label=None):
    ax.plot(x, y, color=color, lw=2, solid_capstyle="round", solid_joinstyle="round", label=label,
            marker="o", ms=8, mec=SURFACE, mew=2)


def lookahead_sweep() -> None:
    rows = [r for r in read(ROOT / "results" / "replay_summary_batch1.csv") if not r["joint"].isdigit()]
    rows.sort(key=lambda r: float(r["lookahead_ms"]))
    x = [float(r["lookahead_ms"]) for r in rows]
    mean = [float(r["lag_mean_ms"]) for r in rows]
    p95 = [float(r["lag_p95_ms"]) for r in rows]
    track = [float(r["track_rms"]) for r in rows]
    jitter = [float(r["jitter_rms"]) for r in rows]

    fig, axes = plt.subplots(1, 3, figsize=(11, 3.6), constrained_layout=True)
    ax = axes[0]
    line(ax, x, mean, SERIES[0], "mean")
    line(ax, x, p95, SERIES[1], "P95")
    ax.set_title("Follower lag")
    ax.set_ylabel("ms")
    ax.set_ylim(0, 130)
    ax.legend(loc="upper right")
    ax.set_xlim(-8, 108)
    for ys in (mean, p95):
        ax.annotate(f"{ys[0]:.0f} ms", (x[0], ys[0]), textcoords="offset points", xytext=(9, 5), ha="left", color=TEXT)
        ax.annotate(f"{ys[-2]:.0f} ms", (x[-2], ys[-2]), textcoords="offset points", xytext=(0, 9), ha="center", color=TEXT)

    ax = axes[1]
    line(ax, x, track, SERIES[0])
    ax.set_title("Tracking error (RMS)")
    ax.set_ylabel("servo counts")
    ax.set_ylim(0, 16)
    i = int(np.argmin(track))
    ax.annotate(f"min {track[i]:.1f}", (x[i], track[i]), textcoords="offset points", xytext=(0, -16), ha="center", color=TEXT)

    ax = axes[2]
    line(ax, x, jitter, SERIES[0])
    ax.set_title("Jitter (RMS of 2nd difference)")
    ax.set_ylabel("servo counts")
    ax.set_ylim(0, 1.4)
    for xs, ys in ((x[0], jitter[0]), (x[-1], jitter[-1])):
        ax.annotate(f"{ys:.2f}", (xs, ys), textcoords="offset points", xytext=(0, 9), ha="center", color=TEXT)

    for ax in axes:
        ax.set_xlabel("prediction lookahead (ms)")
        ax.set_xticks(x)
    fig.savefig(OUT / "lookahead_sweep.png", dpi=160)
    plt.close(fig)


def per_joint_lag() -> None:
    rows = read(ROOT / "results" / "replay_summary_batch2_perjoint.csv")
    configs = [("0", "no prediction"), ("80", "uniform 80 ms"), ("PJ", "per-joint lookahead")]
    names = ["1 base", "2 shoulder", "3 elbow", "4 wrist pitch", "5 wrist roll"]
    fig, ax = plt.subplots(figsize=(8.5, 3.8), constrained_layout=True)
    width = 0.24
    for k, (key, label) in enumerate(configs):
        vals = [float(next(r["lag_mean_ms"] for r in rows if r["lookahead_ms"] == key and r["joint"] == str(j))) for j in range(1, 6)]
        xs = np.arange(5) + (k - 1) * (width + 0.02)
        ax.bar(xs, vals, width=width, color=SERIES[k], label=label, edgecolor=SURFACE, linewidth=2)
        if k != 1:  # label the before and the final after; the middle series reads off the axis
            for xi, v in zip(xs, vals):
                ax.annotate(f"{v:.0f}", (xi, v), textcoords="offset points", xytext=(0, 3), ha="center", color=TEXT, fontsize=9)
    ax.set_xticks(np.arange(5), names)
    ax.set_ylabel("mean follower lag (ms)")
    ax.set_ylim(0, 138)
    ax.set_title("Mean lag by joint")
    ax.legend(loc="upper right", ncols=3)
    fig.savefig(OUT / "per_joint_lag.png", dpi=160)
    plt.close(fig)


def step_response() -> None:
    path = ROOT / "results" / "logs" / "step_raw_20261113_052916.csv"
    if not path.exists():
        print(f"skip step_response: {path} not found (raw logs are local only)")
        return
    trials: dict[tuple[int, int], list[tuple[float, float]]] = defaultdict(list)
    for r in read(path):
        trials[(int(r["joint"]), int(r["trial"]))].append((float(r["t_ms"]), float(r["travel"])))
    grid = np.arange(0, 401, 2.0)
    names = {1: "1 base", 2: "2 shoulder", 4: "4 wrist pitch"}
    fig, ax = plt.subplots(figsize=(8.5, 3.8), constrained_layout=True)
    for k, joint in enumerate(sorted(names)):
        curves = []
        for (j, _), pts in trials.items():
            if j != joint:
                continue
            t, y = np.array(pts).T
            final = y[-10:].mean()
            curves.append(np.interp(grid, np.r_[0.0, t], np.r_[0.0, y]) / final * 100.0)
        med = np.median(curves, axis=0)
        t50 = float(np.interp(50, med, grid))
        ax.plot(grid, med, color=SERIES[k], lw=2, solid_capstyle="round", label=f"{names[joint]}: 50% at {t50:.0f} ms")
        ax.plot([t50], [50], "o", ms=8, color=SERIES[k], mec=SURFACE, mew=2)
    ax.axhline(100, color=GRID, lw=1)
    ax.set_xlim(0, 400)
    ax.set_ylim(-5, 115)
    ax.set_yticks([0, 50, 100])
    ax.set_xlabel("time since the command was written to the serial port (ms)")
    ax.set_ylabel("% of step travelled")
    ax.set_title("Follower response to a 10° step (median of 10 trials per joint)")
    ax.legend(loc="lower right")
    fig.savefig(OUT / "step_response.png", dpi=160)
    plt.close(fig)


def latency_breakdown() -> None:
    """端到端延迟（视频，走到 50%）拆成两段：串口日志看不到的部分 + 串口日志看到的部分。"""
    rows = read(ROOT / "results" / "video_latency.csv")

    def get(video: str, source: str) -> float:
        return float(next(r["d50_ms"] for r in rows if r["video"] == video and r["source"] == source))

    configs = [("flick_none.MOV", "No prediction"), ("flick_pred.MOV", "Per-joint lookahead")]
    fig, ax = plt.subplots(figsize=(9, 2.9), constrained_layout=True)
    for k, (video, label) in enumerate(configs):
        total, seen = get(video, "视频"), get(video, "串口日志")
        unseen = total - seen
        y = len(configs) - 1 - k
        ax.barh(y, unseen, height=0.42, color=SERIES[0], edgecolor=SURFACE, linewidth=2,
                label="leader moves → PC has the reading (video − log)" if k == 0 else None)
        ax.barh(y, seen, left=unseen, height=0.42, color=SERIES[1], edgecolor=SURFACE, linewidth=2,
                label="PC has the reading → follower at 50% (serial log)" if k == 0 else None)
        ax.annotate(f"{unseen:.0f}", (unseen / 2, y), ha="center", va="center", color=SURFACE, fontsize=9, fontweight="bold")
        ax.annotate(f"{seen:.0f}", (unseen + seen / 2, y), ha="center", va="center", color=SURFACE, fontsize=9, fontweight="bold")
        ax.annotate(f"{total:.0f} ms", (total, y), textcoords="offset points", xytext=(6, 0), va="center", color=TEXT)
    ax.set_yticks(range(len(configs)), [c[1] for c in reversed(configs)])
    ax.set_xlim(0, 120)
    ax.set_xlabel("ms, at 50% of a base-joint flick (240 fps video)")
    ax.grid(axis="x", color=GRID, linewidth=1)
    ax.grid(axis="y", visible=False)
    ax.set_title("End-to-end delay, leader motion to follower motion")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.32), ncols=2)
    fig.savefig(OUT / "latency_breakdown.png", dpi=160)
    plt.close(fig)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    lookahead_sweep()
    per_joint_lag()
    step_response()
    latency_breakdown()
    print(f"figures written to {OUT}")


if __name__ == "__main__":
    main()
