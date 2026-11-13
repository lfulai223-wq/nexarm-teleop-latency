"""延迟估计的单元测试，不需要硬件。"""

import numpy as np

from nexarm_teleop.latency import best_lag_ms, stale_mask, summarize, windowed_lags_ms
from nexarm_teleop.predictor import LinearPredictor

T = np.arange(0, 20, 0.02)
REF = 300 * np.sin(2 * np.pi * 0.5 * T) + 100 * np.sin(2 * np.pi * 1.3 * T)


def test_recovers_known_lag():
    meas = np.interp(T - 0.083, T, REF)
    lag, rms = best_lag_ms(T, REF, T, meas)
    assert lag == 83.0 and rms < 1e-6


def test_recovers_negative_lag():
    meas = np.interp(T + 0.030, T, REF)
    lag, _ = best_lag_ms(T, REF, T, meas)
    assert lag == -30.0


def test_windowed_lags_are_consistent():
    meas = np.interp(T - 0.083, T, REF)
    s = summarize(windowed_lags_ms(T, REF, T, meas))
    assert s["n"] > 10 and s["mean"] == 83.0 and s["p95"] == 83.0


def test_stale_mask_flags_stuck_then_jump():
    meas = np.array([100, 110, 120, 120, 120, 120, 190, 200, 210], dtype=float)
    assert stale_mask(meas).tolist() == [False, False, False, True, True, True, False, False, False]


def test_stale_mask_ignores_real_standstill():
    meas = np.array([100, 110, 120, 120, 120, 120, 121, 123, 126], dtype=float)
    assert not stale_mask(meas).any()


def test_predictor_leads_by_roughly_the_lookahead():
    p = LinearPredictor(1, 0.08, vel_cutoff_hz=50)
    out = np.array([p.step([x], t)[0] for x, t in zip(REF, T)])
    lag, _ = best_lag_ms(T, REF, T, out)  # 预测输出应当超前于输入
    assert -90 <= lag <= -60
