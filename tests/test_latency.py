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


def test_lowpass_delays_a_slow_signal_by_about_its_time_constant():
    from nexarm_teleop.filters import LowPassFilter

    f = LowPassFilter(1, cutoff_hz=5.0)
    out = np.array([f.step([x], t)[0] for x, t in zip(REF, T)])
    lag, _ = best_lag_ms(T, REF, T, out)
    tau_ms = 1000.0 / (2 * np.pi * 5.0)  # 约 32 ms；对低频信号，一阶低通的群延迟接近时间常数
    assert abs(lag - tau_ms) < 8


def test_one_euro_holds_still_and_follows_motion():
    from nexarm_teleop.filters import OneEuroFilter

    rng = np.random.default_rng(0)
    f = OneEuroFilter(1, mincutoff_hz=1.0, beta=0.004)
    noisy = 2000 + rng.normal(0, 2.0, size=500)  # 静止加 2 计数噪声
    out = np.array([f.step([x], t)[0] for x, t in zip(noisy, T[:500])])
    assert out[100:].std() < 0.5 * noisy[100:].std()
