#!/usr/bin/env python3
"""用慢动作视频测真实的端到端延迟：从主臂实际动起来，到从臂实际动起来。

串口日志里的位置都是经被测链路读回来的，视频是独立的外部真值。
做法：在画面里给两条臂各框一块区域，用光流跟踪区域内的特征点，得到每帧的二维位移；
主臂每甩一次，把两条臂的位移投影到各自"起点→终点"的方向上，比较走到 10% / 50% / 90% 的时刻。

    # 先看框得对不对（坐标是缩小到 270×480 后的像素）
    python scripts/video_latency.py media/flick_none.MOV --preview
    # 测量，并和同一次运行的串口日志对比
    python scripts/video_latency.py media/flick_none.MOV --log results/logs/teleop_XXXX.csv

时间用每一帧自带的时间戳（iPhone 慢动作为 240 fps，偶有丢帧），不按帧号推算。
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

SMALL = (270, 480)  # 缩小后的宽、高；竖拍视频
LEVELS = (0.1, 0.5, 0.9)


def parse_roi(text: str) -> tuple[int, int, int, int]:
    x0, y0, x1, y1 = (int(v) for v in text.split(","))
    return x0, y0, x1, y1


def decode(path: Path, cache: Path) -> tuple[np.ndarray, np.ndarray]:
    """解码并缩小成灰度帧，结果缓存起来（解码 4K 级慢动作视频要几十秒）。"""
    if cache.exists():
        d = np.load(cache)
        return d["frames"], d["ts"]
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f"打不开视频：{path}")
    frames, ts = [], []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        ts.append(cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0)
        small = cv2.resize(frame, SMALL, interpolation=cv2.INTER_AREA)
        frames.append(cv2.cvtColor(small, cv2.COLOR_BGR2GRAY))
    frames, ts = np.array(frames), np.array(ts)
    cache.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(cache, frames=frames, ts=ts)
    return frames, ts


def track(frames: np.ndarray, roi: tuple[int, int, int, int]) -> np.ndarray:
    """在区域内跟踪特征点，返回每帧相对第一帧的累计位移 (N×2，像素)。"""
    x0, y0, x1, y1 = roi
    mask = np.zeros_like(frames[0])
    mask[y0:y1, x0:x1] = 255
    lk = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))
    pos = np.zeros((len(frames), 2))
    pts = None
    for i in range(1, len(frames)):
        if pts is None or len(pts) < 15:
            pts = cv2.goodFeaturesToTrack(frames[i - 1], maxCorners=60, qualityLevel=0.01, minDistance=4, mask=mask)
        if pts is None:
            pos[i] = pos[i - 1]
            continue
        nxt, st, _ = cv2.calcOpticalFlowPyrLK(frames[i - 1], frames[i], pts, None, **lk)
        ok = st.ravel() == 1
        if ok.sum() < 5:
            pos[i] = pos[i - 1]
            pts = None
            continue
        pos[i] = pos[i - 1] + np.median((nxt[ok] - pts[ok]).reshape(-1, 2), axis=0)
        pts = nxt[ok].reshape(-1, 1, 2)
        inside = (pts[:, 0, 0] >= x0 - 10) & (pts[:, 0, 0] < x1 + 10) & (pts[:, 0, 1] >= y0 - 10) & (pts[:, 0, 1] < y1 + 10)
        pts = pts[inside]  # 跑出区域的点丢掉，下一帧补检测
    return pos


def find_moves(t: np.ndarray, x: np.ndarray, min_speed: float, min_gap: float = 0.8) -> list[float]:
    """找每次甩动的速度峰值时刻。"""
    v = np.gradient(x, t)
    k = max(1, int(round(0.02 / np.median(np.diff(t)))))
    v = np.convolve(v, np.ones(k) / k, mode="same")
    peaks: list[int] = []
    for i in np.where(np.abs(v) > min_speed)[0]:
        if peaks and t[i] - t[peaks[-1]] < min_gap:
            if abs(v[i]) > abs(v[peaks[-1]]):
                peaks[-1] = i
        else:
            peaks.append(i)
    return [float(t[i]) for i in peaks]


def project(t: np.ndarray, xy: np.ndarray, t_peak: float) -> np.ndarray | None:
    """把二维位移投影到这次甩动"起点平台→终点平台"的方向上。"""
    before = (t >= t_peak - 0.35) & (t <= t_peak - 0.12)
    after = (t >= t_peak + 0.45) & (t <= t_peak + 0.9)
    if before.sum() < 3 or after.sum() < 3:
        return None
    p0, p1 = np.median(xy[before], axis=0), np.median(xy[after], axis=0)
    n = np.linalg.norm(p1 - p0)
    return None if n < 1e-6 else (xy - p0) @ ((p1 - p0) / n)


def crossings(t: np.ndarray, x: np.ndarray, t_peak: float) -> tuple[list[float], float] | None:
    """一次甩动中走到各百分比的时刻（线性插值），以及超调（占行程的比例）。"""
    before = (t >= t_peak - 0.35) & (t <= t_peak - 0.12)
    after = (t >= t_peak + 0.45) & (t <= t_peak + 0.9)
    if before.sum() < 3 or after.sum() < 3:
        return None
    x0, x1 = np.median(x[before]), np.median(x[after])
    if abs(x1 - x0) < 1e-6:
        return None
    w = (t >= t_peak - 0.35) & (t <= t_peak + 0.9)
    tw, f = t[w], (x[w] - x0) / (x1 - x0)
    times = []
    for level in LEVELS:
        idx = np.where(f >= level)[0]
        if len(idx) == 0 or idx[0] == 0:
            return None
        i = idx[0]
        times.append(tw[i - 1] + (level - f[i - 1]) / (f[i] - f[i - 1]) * (tw[i] - tw[i - 1]))
    return times, float(f.max() - 1.0)


def delays(t_lead, lead, t_fol, fol, peaks) -> list[list[float]]:
    rows = []
    for tp in peaks:
        a, b = crossings(t_lead, lead(tp), tp), crossings(t_fol, fol(tp), tp)
        if a is None or b is None:
            continue
        rows.append([tp] + [(b[0][k] - a[0][k]) * 1000 for k in range(3)] + [b[1] * 100])
    return rows


def summary(label: str, rows: list[list[float]]) -> dict:
    r = np.array(rows)
    m, sd = r[:, 1:5].mean(0), r[:, 1:5].std(0)
    print(
        f"{label:10s} n={len(r):2d} | 10%: {m[0]:6.1f} ±{sd[0]:4.1f} ms | 50%: {m[1]:6.1f} ±{sd[1]:4.1f} ms | "
        f"90%: {m[2]:6.1f} ±{sd[2]:4.1f} ms | 从臂超调 {m[3]:5.1f}%"
    )
    return {"source": label, "n": len(r), "d10_ms": round(m[0], 1), "d50_ms": round(m[1], 1), "d90_ms": round(m[2], 1),
            "d50_sd_ms": round(sd[1], 1), "overshoot_pct": round(m[3], 1)}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video", type=Path)
    ap.add_argument("--leader-roi", type=parse_roi, default=(222, 215, 270, 300), help="主臂跟踪区域 x0,y0,x1,y1（避开手）")
    ap.add_argument("--follower-roi", type=parse_roi, default=(115, 100, 215, 205), help="从臂跟踪区域 x0,y0,x1,y1")
    ap.add_argument("--log", type=Path, help="同一次运行的 teleop 日志，用来对比串口测得的延迟")
    ap.add_argument("--log-skip", type=float, default=2.0, help="日志开头多少秒内的甩动不算（启动过渡）")
    ap.add_argument("--preview", action="store_true", help="只把跟踪区域画在一帧上，存成图片")
    ap.add_argument("--out", type=Path, help="把汇总追加写入这个 CSV")
    args = ap.parse_args()

    cache = ROOT / "results" / "logs" / f"video_{args.video.stem}.npz"
    frames, ts = decode(args.video, cache)
    print(f"{args.video.name}: {len(frames)} 帧，{ts[-1] - ts[0]:.1f} 秒，帧间隔中位 {np.median(np.diff(ts)) * 1000:.2f} ms")

    if args.preview:
        img = cv2.cvtColor(frames[len(frames) // 2], cv2.COLOR_GRAY2BGR)
        for (x0, y0, x1, y1), color in ((args.leader_roi, (0, 200, 0)), (args.follower_roi, (0, 0, 255))):
            cv2.rectangle(img, (x0, y0), (x1, y1), color, 1)
        out = ROOT / "results" / "logs" / f"video_{args.video.stem}_roi.png"
        cv2.imwrite(str(out), cv2.resize(img, (SMALL[0] * 2, SMALL[1] * 2), interpolation=cv2.INTER_NEAREST))
        print(f"预览图：{out}（绿框主臂，红框从臂）")
        return

    lead_xy, fol_xy = track(frames, args.leader_roi), track(frames, args.follower_roi)
    peaks = find_moves(ts, lead_xy[:, 0], min_speed=60)
    rows = delays(ts, lambda tp: project(ts, lead_xy, tp), ts, lambda tp: project(ts, fol_xy, tp), peaks)
    results = []

    if args.log:
        a = np.loadtxt(args.log, delimiter=",", skiprows=1)
        tl = ((a[:, 2] + a[:, 3]) / 2 - a[0, 1]) / 1e9
        tf = ((a[:, 17] + a[:, 18]) / 2 - a[0, 1]) / 1e9
        lead1, fol1 = a[:, 4].astype(float), a[:, 19].astype(float)
        log_peaks = [p for p in find_moves(tl, lead1, min_speed=800) if p >= args.log_skip]
        log_rows = delays(tl, lambda tp: lead1, tf, lambda tp: fol1, log_peaks)
        # 用甩动时刻把视频时间和日志时间对齐，只保留两边都有的那几次
        all_log_peaks = find_moves(tl, lead1, min_speed=800)
        if len(all_log_peaks) == len(peaks):
            fit = np.polyfit(all_log_peaks, peaks, 1)
            resid = np.array(peaks) - np.polyval(fit, all_log_peaks)
            print(f"视频时钟 = {fit[0]:.5f} × 日志时钟 + {fit[1]:.3f} s（残差 {resid.std() * 1000:.0f} ms）")
            keep = {round(float(np.polyval(fit, p)), 1) for p in log_peaks}
            rows = [r for r in rows if any(abs(r[0] - k) < 0.3 for k in keep)]
        else:
            print(f"注意：视频里找到 {len(peaks)} 次甩动，日志里 {len(all_log_peaks)} 次，没有做逐次对齐")
        results.append(summary("视频", rows))
        results.append(summary("串口日志", log_rows))
    else:
        results.append(summary("视频", rows))

    if args.out:
        new = not args.out.exists()
        with args.out.open("a", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["video", "log"] + list(results[0]))
            if new:
                w.writeheader()
            for r in results:
                w.writerow({"video": args.video.name, "log": args.log.name if args.log else "", **r})


if __name__ == "__main__":
    main()
