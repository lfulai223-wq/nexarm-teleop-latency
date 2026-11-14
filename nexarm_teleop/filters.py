"""输入滤波：OneEuro 自适应低通 + 固定截止一阶低通。

思路沿用上一套系统（Alicia-D → FR3）：
  - OneEuro：截止频率随速度升高，静止时滤得狠、运动时放开，兼顾稳和跟手；
  - 固定低通：OneEuro 在快速运动时会把手抖也放过去，再加一级固定截止把震颤频段压下去。
参数要按 NexArm 的单位（舵机计数，4096 为一圈）重新定。滤波一定会增加延迟。
"""

from __future__ import annotations

import math


def _alpha(dt: float, cutoff_hz: float) -> float:
    tau = 1.0 / (2.0 * math.pi * cutoff_hz)
    return dt / (dt + tau)


class OneEuroFilter:
    """截止频率 = mincutoff + beta × |速度|，速度单位为计数/秒。"""

    def __init__(self, n: int, mincutoff_hz: float = 3.0, beta: float = 0.004, dcutoff_hz: float = 1.0):
        self.mincutoff_hz = mincutoff_hz
        self.beta = beta
        self.dcutoff_hz = dcutoff_hz
        self._x: list[float] | None = None
        self._dx = [0.0] * n
        self._t: float | None = None

    def step(self, x: list[float], t: float) -> list[float]:
        if self._x is None or self._t is None:
            self._x, self._t = list(x), t
            return list(x)
        dt = max(1e-4, t - self._t)
        self._t = t
        a_d = _alpha(dt, self.dcutoff_hz)
        out = []
        for i, xi in enumerate(x):
            self._dx[i] += a_d * ((xi - self._x[i]) / dt - self._dx[i])
            cutoff = self.mincutoff_hz + self.beta * abs(self._dx[i])
            self._x[i] += _alpha(dt, cutoff) * (xi - self._x[i])
            out.append(self._x[i])
        return out


class LowPassFilter:
    """固定截止频率的一阶低通。"""

    def __init__(self, n: int, cutoff_hz: float = 5.0):
        self.cutoff_hz = cutoff_hz
        self._y: list[float] | None = None
        self._t: float | None = None

    def step(self, x: list[float], t: float) -> list[float]:
        if self._y is None or self._t is None:
            self._y, self._t = list(x), t
            return list(x)
        dt = max(1e-4, t - self._t)
        self._t = t
        a = _alpha(dt, self.cutoff_hz)
        self._y = [y + a * (xi - y) for y, xi in zip(self._y, x)]
        return list(self._y)
