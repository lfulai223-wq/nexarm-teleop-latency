"""LeRobot 主臂：读主臂、映射到从臂坐标后，再做逐关节线性外推前馈。"""

import time

from lerobot.teleoperators.nexarm_leader import NexArmLeader
from nexarm_teleop.predictor import LinearPredictor
from nexarm_teleop.protocol import JOINT_COUNT, POSITION_MAX, POSITION_MIN

from .config_nexarm_leader_predictive import NexArmLeaderPredictiveConfig


class NexArmLeaderPredictive(NexArmLeader):
    config_class = NexArmLeaderPredictiveConfig
    name = "nexarm_leader_predictive"

    def __init__(self, config: NexArmLeaderPredictiveConfig):
        super().__init__(config)
        if len(config.lookahead_ms) != JOINT_COUNT:
            raise ValueError(f"lookahead_ms 需要 {JOINT_COUNT} 个值，收到 {len(config.lookahead_ms)} 个")
        self.predictor = LinearPredictor(
            JOINT_COUNT, [x / 1000.0 for x in config.lookahead_ms], config.vel_cutoff_hz, config.max_lead
        )
        self._last: list[float] | None = None

    def connect(self, calibrate: bool = True) -> None:
        super().connect(calibrate)
        self.predictor.reset()
        self._last = None

    def get_action(self) -> dict[str, float]:
        t0 = time.perf_counter()
        action = super().get_action()  # 商家实现：读主臂并映射到从臂坐标
        t_sample = (t0 + time.perf_counter()) / 2  # 与 nexarm_teleop 一致：取读取的中点作为采样时刻
        keys = list(action)
        out = self.predictor.step([action[k] for k in keys], t_sample)
        step = self.config.max_step
        if step > 0 and self._last is not None:
            out = [last + max(-step, min(step, v - last)) for last, v in zip(self._last, out)]
        out = [min(float(POSITION_MAX), max(float(POSITION_MIN), v)) for v in out]
        self._last = out
        return {k: v for k, v in zip(keys, out)}
