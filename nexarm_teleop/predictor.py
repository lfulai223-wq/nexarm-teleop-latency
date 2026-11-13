"""线性外推前馈：用估计出的速度把指令提前 lookahead 秒，抵消下游（舵机）的滞后。

    指令 = 位置 + 速度 × 前瞻时长

速度由相邻两次位置差分得到，再过一个一阶低通，否则 12 位位置的量化噪声会被放大。
"""

from __future__ import annotations

import math


class LinearPredictor:
    def __init__(self, n: int, lookahead_s: float | list[float], vel_cutoff_hz: float = 5.0, max_lead: float = 300.0):
        # 可以给一个统一的前瞻时长，也可以每个关节给一个
        self.lookahead_s = list(lookahead_s) if isinstance(lookahead_s, (list, tuple)) else [lookahead_s] * n
        self.vel_cutoff_hz = vel_cutoff_hz
        self.max_lead = max_lead  # 提前量上限（计数），防止速度估计出错时指令飞出去
        self._x: list[float] | None = None
        self._v = [0.0] * n
        self._t: float | None = None

    def reset(self) -> None:
        self._x = None
        self._v = [0.0] * len(self._v)
        self._t = None

    def step(self, x: list[float], t: float) -> list[float]:
        """输入当前位置和时间（秒），返回提前后的指令。"""
        if self._x is None or self._t is None:
            self._x, self._t = list(x), t
            return list(x)
        dt = max(1e-4, t - self._t)
        alpha = dt / (dt + 1.0 / (2.0 * math.pi * self.vel_cutoff_hz))
        out = []
        for i, xi in enumerate(x):
            raw_v = (xi - self._x[i]) / dt
            self._v[i] += alpha * (raw_v - self._v[i])
            lead = max(-self.max_lead, min(self.max_lead, self._v[i] * self.lookahead_s[i]))
            out.append(xi + lead)
        self._x, self._t = list(x), t
        return out
