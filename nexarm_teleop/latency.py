"""延迟估计：时间平移拟合。

把参考信号（主臂目标）在时间上往后平移 τ，找出让它和从臂实测位置最吻合的 τ。
整段算一个值，也可以切成小窗各算一个，得到分布（均值、P95）。
"""

from __future__ import annotations

import numpy as np


def best_lag_ms(
    t_ref: np.ndarray,
    ref: np.ndarray,
    t_meas: np.ndarray,
    meas: np.ndarray,
    max_lag_ms: float = 300.0,
    step_ms: float = 1.0,
    min_lag_ms: float = -150.0,
) -> tuple[float, float]:
    """返回 (最吻合的滞后毫秒数, 该滞后下的均方根残差)。时间单位为秒，去均值后比较。

    滞后为负表示实测信号超前于参考信号（预测过头时会出现）。
    """
    lags = np.arange(min_lag_ms, max_lag_ms + step_ms, step_ms) / 1000.0
    m = meas - meas.mean()
    best = (0.0, np.inf)
    for lag in lags:
        r = np.interp(t_meas - lag, t_ref, ref)
        r = r - r.mean()
        rms = float(np.sqrt(np.mean((m - r) ** 2)))
        if rms < best[1]:
            best = (lag * 1000.0, rms)
    return best


def windowed_lags_ms(
    t_ref: np.ndarray,
    ref: np.ndarray,
    t_meas: np.ndarray,
    meas: np.ndarray,
    window_s: float = 2.0,
    hop_s: float = 1.0,
    min_range: float = 80.0,
    max_lag_ms: float = 300.0,
) -> np.ndarray:
    """滑动窗口逐段估计滞后。窗口内参考信号的运动范围小于 min_range（计数）的跳过。"""
    out = []
    start = t_meas[0]
    while start + window_s <= t_meas[-1]:
        sel = (t_meas >= start) & (t_meas < start + window_s)
        ref_sel = (t_ref >= start - max_lag_ms / 1000.0) & (t_ref < start + window_s + 0.15)
        if sel.sum() > 20 and ref_sel.sum() > 20 and np.ptp(ref[ref_sel]) >= min_range:
            lag, _ = best_lag_ms(t_ref, ref, t_meas[sel], meas[sel], max_lag_ms)
            # 落在搜索范围边界上说明这一窗没有拟合出有效的极值，丢弃
            if -149.0 < lag < max_lag_ms - 1.0:
                out.append(lag)
        start += hop_s
    return np.array(out)


def stale_mask(meas: np.ndarray, min_run: int = 3, min_jump: float = 30.0) -> np.ndarray:
    """找出"卡住"的读数：连续 min_run 个以上完全相同的值，紧接着一步跳变不小于 min_jump。

    从臂偶尔会在读取失败时返回上一次的旧值，表现为读数不动、随后一步跳到真实位置。
    真正静止的关节不会被标记，因为静止之后没有跳变。
    返回布尔数组，True 表示该采样是旧值（每段的第一个采样是真实读数，不标记）。
    """
    mask = np.zeros(len(meas), dtype=bool)
    i = 0
    n = len(meas)
    while i < n:
        j = i
        while j + 1 < n and meas[j + 1] == meas[i]:
            j += 1
        if j - i + 1 >= min_run and j + 1 < n and abs(meas[j + 1] - meas[i]) >= min_jump:
            mask[i + 1 : j + 1] = True
        i = j + 1
    return mask


def summarize(values: np.ndarray) -> dict[str, float]:
    if len(values) == 0:
        return {"n": 0, "mean": float("nan"), "p50": float("nan"), "p95": float("nan")}
    return {
        "n": int(len(values)),
        "mean": float(np.mean(values)),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
    }
