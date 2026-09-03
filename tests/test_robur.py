"""UR.__init__ must not hand back an object with an unset TCP.

Every later move is computed against the TCP offset, so a UR built with the
robot not in remote control mode -- where set_tcp fails -- would silently drive
to the wrong place. The constructor used to swallow that failure.
"""
import sys
import types

import pytest

pytest.importorskip("PyQt5")
robUR = pytest.importorskip("common.robUR")
from urxe import ursecmon  # noqa: E402


class FakeRobot(object):
    def __init__(self, *args, **kwargs):
        self.closed = False

    def close(self):
        self.closed = True

    def set_payload(self, *args, **kwargs):
        pass


@pytest.fixture
def build_ur(monkeypatch):
    """Return (construct, built): construct(set_tcp) makes a UR against fakes."""
    built = {}

    class RobotiqGripper(object):
        def __init__(self, *args, **kwargs):
            pass

    def fake_robot(IP, use_rtde=False):
        built["robot"] = FakeRobot()
        return built["robot"]

    fake_module = types.ModuleType("urxe.robot")
    fake_module.Robot = fake_robot
    fake_module.RobotiqGripper = RobotiqGripper

    monkeypatch.setitem(sys.modules, "urxe.robot", fake_module)
    monkeypatch.setattr(robUR, "dashboard", lambda robot: object())
    monkeypatch.setattr(robUR, "camera", lambda IP=None: object())

    def _construct(set_tcp):
        monkeypatch.setattr(robUR.UR, "set_tcp", set_tcp)
        return robUR.UR("1.2.3.4", grippertype=0, cameratype=0)

    return _construct, built


def test_set_tcp_failure_closes_the_robot_and_propagates(build_ur):
    construct, built = build_ur

    def refuse(self, tcp):
        raise ursecmon.TimeoutException("robot not in remote control mode")

    with pytest.raises(ursecmon.TimeoutException):
        construct(refuse)

    assert built["robot"].closed, "robot left connected with an unset TCP"


def test_normal_construction_leaves_the_robot_connected(build_ur):
    construct, built = build_ur
    calls = []

    construct(lambda self, tcp: calls.append(tcp))

    assert len(calls) == 1
    assert built["robot"].closed is False
