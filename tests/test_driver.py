"""驱动层的单元测试：用假串口模拟 USB 掉线，不需要硬件。"""

import pytest
import serial

from nexarm_teleop.driver import ConnectionLost, NexArmPort


class DeadSerial:
    """模拟一个已经掉线的串口：任何读写都抛 pyserial 的异常。"""

    in_waiting = 0

    def write(self, data):
        raise serial.SerialException("write failed: [Errno 5] Input/output error")

    def read(self, n):
        raise serial.SerialException("device reports readiness to read but returned no data")

    def reset_input_buffer(self):
        raise serial.SerialException("[Errno 5] Input/output error")

    def close(self):
        pass


def make_port():
    port = NexArmPort("/dev/fake")
    port._serial = DeadSerial()
    return port


def test_write_on_dead_port_raises_connection_lost():
    with pytest.raises(ConnectionLost):
        make_port().write_positions([2048] * 6)


def test_read_on_dead_port_raises_connection_lost():
    with pytest.raises(ConnectionLost):
        make_port().read_positions()


def test_exit_bridge_mode_on_dead_port_does_not_raise():
    assert make_port().exit_bridge_mode() is False
