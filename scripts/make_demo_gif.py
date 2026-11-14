#!/usr/bin/env python3
"""从两段慢动作视频剪出并排对比动图：左边不开预测，右边开逐关节预测。

每段对齐到主臂"走到 10%"的时刻（由 video_latency.py 的跟踪算出），放慢 10 倍。
画面只保留两条臂（裁掉房间和手）。

    python scripts/make_demo_gif.py
输出 docs/img/demo.gif
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs" / "img" / "demo.gif"

CROP = (slice(380, 1200), slice(400, 1080))  # 竖拍 1080×1920 画面里两条臂所在的区域
PANEL = (260, 314)  # 每个面板缩放后的宽、高
STEP_S = 1 / 80  # 每个输出帧对应的真实时间（240 fps 视频每 3 帧取 1 帧）
FPS_OUT = 8  # 播放帧率：真实时间放慢 80 / 8 = 10 倍
PRE_S, POST_S = 0.08, 0.42  # 每段从主臂起动前多少秒到之后多少秒

# (不开预测视频的起动时刻, 开预测视频的起动时刻)，单位秒，同方向的一次甩动
SEGMENTS = [(10.926, 10.315)]
VIDEOS = (ROOT / "media" / "flick_none.MOV", ROOT / "media" / "flick_pred.MOV")
LABELS = ("No prediction", "Per-joint prediction")

TRACKS = (ROOT / "results" / "logs" / "video_flick_none_tracks.npz", ROOT / "results" / "logs" / "video_flick_pred_tracks.npz")
LEADER_RGB = (42, 120, 214)
FOLLOWER_RGB = (235, 104, 52)

BG = (252, 252, 251)
INK = (11, 11, 11)
INK2 = (82, 81, 78)


def grab(path: Path, windows: list[tuple[float, float]]) -> list[list[tuple[float, np.ndarray]]]:
    """顺序解码，只保留落在各时间窗内的帧（已裁剪缩放）。"""
    cap = cv2.VideoCapture(str(path))
    out: list[list[tuple[float, np.ndarray]]] = [[] for _ in windows]
    last_end = max(w[1] for w in windows)
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        if t > last_end:
            break
        for i, (a, b) in enumerate(windows):
            if a <= t <= b:
                panel = cv2.resize(frame[CROP], PANEL, interpolation=cv2.INTER_AREA)
                out[i].append((t, cv2.cvtColor(panel, cv2.COLOR_BGR2RGB)))
    return out


def progress(track_path: Path, onset: float):
    """返回一个函数 t -> (主臂已走比例, 从臂已走比例)，用跟踪位移投影到这次甩动的方向上。"""
    import sys

    sys.path.insert(0, str(ROOT / "scripts"))
    import video_latency as vl

    d = np.load(track_path)
    ts, lead, fol = d["ts"], d["lead"], d["fol"]
    t_peak = vl.find_moves(ts, lead[:, 0], 60)
    t_peak = min(t_peak, key=lambda p: abs(p - onset))
    curves = []
    for xy in (lead, fol):
        x = vl.project(ts, xy, t_peak)
        before = (ts >= t_peak - 0.35) & (ts <= t_peak - 0.12)
        after = (ts >= t_peak + 0.45) & (ts <= t_peak + 0.9)
        x0, x1 = np.median(x[before]), np.median(x[after])
        curves.append((x - x0) / (x1 - x0))
    return lambda t: tuple(float(np.interp(t, ts, c)) for c in curves)


def bars(img: np.ndarray, x: int, y: int, w: int, fracs: tuple[float, float]) -> None:
    """两条进度条：主臂、从臂各走了这次动作的百分之多少。"""
    for i, (name, frac, color) in enumerate((("leader", fracs[0], LEADER_RGB), ("follower", fracs[1], FOLLOWER_RGB))):
        yy = y + i * 16
        text(img, name, (x, yy + 9), 0.38, INK2)
        bx, bw = x + 62, w - 62 - 38
        cv2.rectangle(img, (bx, yy), (bx + bw, yy + 9), (230, 229, 225), -1)
        fill = int(round(bw * max(0.0, min(1.0, frac))))
        if fill > 0:
            cv2.rectangle(img, (bx, yy), (bx + fill, yy + 9), color, -1)
        text(img, f"{100 * frac:3.0f}%", (bx + bw + 4, yy + 9), 0.38, INK2)


def nearest(frames: list[tuple[float, np.ndarray]], t: float) -> np.ndarray:
    return min(frames, key=lambda f: abs(f[0] - t))[1]


def text(img: np.ndarray, s: str, xy: tuple[int, int], scale: float, color, thick: int = 1, center: bool = False) -> None:
    (w, _), _ = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    x = xy[0] - w // 2 if center else xy[0]
    cv2.putText(img, s, (x, xy[1]), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def compose(left: np.ndarray, right: np.ndarray, rel_ms: float, fracs) -> np.ndarray:
    gap, head, foot = 8, 30, 64
    w, h = PANEL
    canvas = np.full((head + h + foot, 2 * w + gap, 3), BG, dtype=np.uint8)
    canvas[head:head + h, :w] = left
    canvas[head:head + h, w + gap:] = right
    for i, label in enumerate(LABELS):
        text(canvas, label, (i * (w + gap) + w // 2, 21), 0.55, INK, 2, center=True)
        bars(canvas, i * (w + gap) + 6, head + h + 8, w - 6, fracs[i])
    text(canvas, f"{rel_ms:+4.0f} ms after the leader starts moving  (10x slower)", (canvas.shape[1] // 2, head + h + 56), 0.42, INK2, 1, center=True)
    return canvas


def main() -> None:
    windows = [[(s[k] - PRE_S - 0.02, s[k] + POST_S + 0.02) for s in SEGMENTS] for k in range(2)]
    clips = [grab(VIDEOS[k], windows[k]) for k in range(2)]
    prog = [[progress(TRACKS[k], seg[k]) for k in range(2)] for seg in SEGMENTS]
    frames: list[np.ndarray] = []
    durations: list[int] = []
    for seg_i, seg in enumerate(SEGMENTS):
        rel = np.arange(-PRE_S, POST_S + 1e-9, STEP_S)
        for r in rel:
            left = nearest(clips[0][seg_i], seg[0] + r)
            right = nearest(clips[1][seg_i], seg[1] + r)
            fr = (prog[seg_i][0](seg[0] + r), prog[seg_i][1](seg[1] + r))
            frames.append(compose(left, right, r * 1000, fr))
            durations.append(int(1000 / FPS_OUT))
        durations[-1] = 1500  # 每段结束停一下
    # 共用一个调色板，压缩体积、避免逐帧闪烁
    sample = Image.fromarray(np.concatenate(frames[:: max(1, len(frames) // 8)], axis=0))
    palette = sample.quantize(colors=96, method=Image.Quantize.MEDIANCUT)
    images = [Image.fromarray(f).quantize(palette=palette, dither=Image.Dither.NONE) for f in frames]
    OUT.parent.mkdir(parents=True, exist_ok=True)
    images[0].save(OUT, save_all=True, append_images=images[1:], duration=durations, loop=0, optimize=True)
    print(f"{OUT}  {len(images)} 帧，{OUT.stat().st_size / 1e6:.1f} MB，播放约 {sum(durations) / 1000:.1f} 秒")


if __name__ == "__main__":
    main()
