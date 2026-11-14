"""配置：在商家的 nexarm_leader 基础上加前馈预测参数。"""

from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.nexarm_leader import NexArmLeaderConfig


@TeleoperatorConfig.register_subclass("nexarm_leader_predictive")
@dataclass
class NexArmLeaderPredictiveConfig(NexArmLeaderConfig):
    """NexArm 主臂 + 逐关节线性外推前馈。

    lookahead_ms：每个关节的前瞻时长（毫秒），顺序为 底座、肩、肘、腕俯仰、腕旋转、夹爪。
    默认值来自 nexarm-teleop-latency 的回放实验（夹爪不预测：它的延迟来自限速，预测无效）。
    """

    lookahead_ms: list[float] = field(default_factory=lambda: [65.0, 110.0, 80.0, 65.0, 95.0, 0.0])
    vel_cutoff_hz: float = 5.0
    max_lead: float = 150.0
    # 每次 get_action 输出相对上一次的最大变化（计数）。LeRobot 的遥操作循环本身没有步长限制，
    # 商家驱动的速度参数实测也不生效，这里兜底，防止预测在急变时把指令推得太远。0 为不限。
    max_step: float = 80.0
