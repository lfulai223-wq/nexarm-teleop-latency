"""插件的离线测试：LeRobot 能找到并创建它，get_action 会把指令往运动方向提前。需要在装有 LeRobot 的环境里运行。"""

import math

from lerobot.teleoperators.utils import make_teleoperator_from_config
from lerobot.utils.import_utils import register_third_party_plugins


class FakeBus:
    """假串口：主臂底座做正弦运动，其他关节不动。"""

    def __init__(self):
        self.t0 = None
        self.is_connected = True

    def read_positions(self):
        import time

        if self.t0 is None:
            self.t0 = time.perf_counter()
        t = time.perf_counter() - self.t0
        return [2048 + int(300 * math.sin(2 * math.pi * 1.0 * t)), 2048, 2048, 2048, 2048, 2048]


def make():
    register_third_party_plugins()
    from lerobot_teleoperator_nexarm_predictive import NexArmLeaderPredictive, NexArmLeaderPredictiveConfig

    teleop = make_teleoperator_from_config(NexArmLeaderPredictiveConfig(port="/dev/null", id="test"))
    assert isinstance(teleop, NexArmLeaderPredictive)
    teleop.bus = FakeBus()
    return teleop


def test_lerobot_creates_the_plugin_teleoperator():
    make()


def test_prediction_leads_the_raw_reading():
    import time

    teleop = make()
    from lerobot.teleoperators.nexarm_leader import NexArmLeader

    leads = []
    for _ in range(100):
        raw = NexArmLeader.get_action(teleop)["shoulder_pan.pos"]
        pred = teleop.get_action()["shoulder_pan.pos"]
        leads.append((raw, pred))
        time.sleep(0.01)
    # 底座在动，预测值和原始值应当不同；其他关节不动，预测值等于原始值
    assert any(abs(p - r) > 5 for r, p in leads[20:])
    assert teleop.get_action()["elbow_flex.pos"] == 2048.0
