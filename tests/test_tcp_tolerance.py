"""Regression from PR #1: a TCP applied to within 1e-7 must be accepted.

set_tcp used to round both sides to five decimal places and test equality.
Rounding puts a hard boundary mid-range, so 0.0999950 and 0.0999949 -- a ten
millionth of a millimetre apart -- fall either side of it and compare unequal.
set_tcp then timed out on an offset the robot had applied correctly.
"""
import logging

import pytest

pytest.importorskip("urxe.robot")
from urxe import ursecmon  # noqa: E402
from urxe.robot import Robot, URRobot  # noqa: E402

REQUESTED = [0.099995, 0.0, 0.15, 0.0, 0.0, 0.0]
WITHIN_TOLERANCE = [0.0999949, 0.0, 0.15, 0.0, 0.0, 0.0]   # 1e-7 low
GENUINELY_WRONG = [0.001, 0.0, 0.15, 0.0, 0.0, 0.0]        # ~1 mm low


@pytest.fixture
def make_robot(monkeypatch):
    monkeypatch.setattr(URRobot, "set_tcp", lambda self, tcp: None)

    def _make(readback):
        robot = Robot.__new__(Robot)
        robot.logger = logging.getLogger("test-tcp-tolerance")
        robot.get_tcp = lambda: list(readback)
        return robot

    return _make


def test_accepts_a_tcp_within_tolerance(make_robot):
    # Must not raise.
    make_robot(WITHIN_TOLERANCE).set_tcp(list(REQUESTED), timeout=1.0)


def test_still_rejects_a_genuinely_wrong_tcp(make_robot):
    with pytest.raises(ursecmon.TimeoutException):
        make_robot(GENUINELY_WRONG).set_tcp(list(REQUESTED), timeout=0.3)
