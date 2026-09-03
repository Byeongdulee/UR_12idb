"""Robot.set_tcp must give up instead of spinning forever.

The robot silently ignores a TCP offset when it is not in remote control mode:
the secondary interface accepts the program and discards it. The original loop
polled for a matching readback with no timeout and no sleep, so it pinned a
core until the process was killed.
"""
import logging
import time

import pytest

pytest.importorskip("urxe.robot")
from urxe import ursecmon  # noqa: E402
from urxe.robot import Robot, URRobot  # noqa: E402

TCP = [0.0, 0.0, 0.15, 0.0, 0.0, 0.0]


@pytest.fixture
def make_robot(monkeypatch):
    """Build Robots whose TCP write is captured and readback is scriptable."""
    written = []
    monkeypatch.setattr(URRobot, "set_tcp",
                        lambda self, tcp: written.append(list(tcp)))

    def _make(readback):
        robot = Robot.__new__(Robot)
        robot.logger = logging.getLogger("test-set-tcp")
        robot.get_tcp = readback
        robot.written = written
        return robot

    return _make


def test_returns_once_the_tcp_is_applied(make_robot):
    robot = make_robot(lambda: list(TCP))
    start = time.monotonic()

    robot.set_tcp(list(TCP))

    assert time.monotonic() - start < 0.5
    assert robot.written[-1] == TCP


def test_times_out_when_the_tcp_is_never_applied(make_robot):
    robot = make_robot(lambda: [0.0] * 6)
    start = time.monotonic()

    with pytest.raises(ursecmon.TimeoutException) as excinfo:
        robot.set_tcp(list(TCP), timeout=0.4)

    assert 0.35 <= time.monotonic() - start < 2.0
    assert "remote control" in str(excinfo.value)


def test_tolerates_a_slow_robot(make_robot):
    polls = {"n": 0}

    def readback():
        polls["n"] += 1
        return list(TCP) if polls["n"] > 10 else [0.0] * 6

    make_robot(readback).set_tcp(list(TCP), timeout=3.0)

    assert polls["n"] > 10


def test_polling_is_throttled(make_robot):
    polls = {"n": 0}

    def readback():
        polls["n"] += 1
        return [0.0] * 6

    with pytest.raises(ursecmon.TimeoutException):
        make_robot(readback).set_tcp(list(TCP), timeout=0.5)

    assert polls["n"] < 200, "%d polls in 0.5 s looks like a busy spin" % polls["n"]


def test_survives_a_readback_of_none(make_robot):
    state = {"n": 0}

    def readback():
        state["n"] += 1
        return None if state["n"] < 5 else list(TCP)

    make_robot(readback).set_tcp(list(TCP), timeout=2.0)
