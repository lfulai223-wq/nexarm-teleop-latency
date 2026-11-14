"""NexArm 主臂的 LeRobot 插件：在商家驱动之上加逐关节前馈预测。

安装到装有商家 LeRobot 改版的环境后，LeRobot 脚本会自动导入它，用 --teleop.type=nexarm_leader_predictive 选用。
"""

from .config_nexarm_leader_predictive import NexArmLeaderPredictiveConfig
from .nexarm_leader_predictive import NexArmLeaderPredictive

__all__ = ["NexArmLeaderPredictive", "NexArmLeaderPredictiveConfig"]
