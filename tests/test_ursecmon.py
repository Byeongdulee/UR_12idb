"""SecondaryMonitor.send_program must confirm the send, never assume it.

A silent timeout used to be indistinguishable from success, so a dropped move
or TCP change went unnoticed until the robot did the wrong thing.
"""
import logging
import socket
import threading
import time
import types

import pytest

ursecmon = pytest.importorskip("urxe.ursecmon")
from urxe.urmon_parser import ParsingException  # noqa: E402


class FakeSocket(object):
    """Stands in for the secondary-interface socket."""

    def __init__(self, fail=None):
        self.fail = fail
        self.written = b""
        self.calls = []

    def sendall(self, data):
        self.calls.append("sendall")
        if self.fail is not None:
            raise self.fail
        self.written += data

    def send(self, data):
        # Deliberately partial, so a regression back to send() is visible.
        self.calls.append("send")
        self.written += data[:1]
        return 1

    def close(self):
        pass


@pytest.fixture
def make_monitor():
    """Build SecondaryMonitors wired to a fake socket, and stop them after."""
    created = []

    def _make(sock):
        mon = ursecmon.SecondaryMonitor.__new__(ursecmon.SecondaryMonitor)
        threading.Thread.__init__(mon)
        mon.daemon = True
        mon.logger = logging.getLogger("test-ursecmon")
        mon._prog_queue = []
        mon._prog_queue_lock = threading.Lock()
        mon._dictLock = threading.Lock()
        mon._dict = {}
        mon._trystop = False
        mon.running = False
        mon._s_secondary = sock

        def _parse(_data):
            raise ParsingException("stubbed: no real packets in this test")

        mon._parser = types.SimpleNamespace(version=(5, 9), parse=_parse)
        # Keep the run loop turning over without a real robot feeding it.
        mon._get_data = lambda: (time.sleep(0.005), b"")[1]
        created.append(mon)
        return mon

    yield _make

    for mon in created:
        mon._trystop = True
    for mon in created:
        if mon.is_alive():
            mon.join(timeout=1.0)


def test_whole_program_is_written(make_monitor):
    sock = FakeSocket()
    mon = make_monitor(sock)
    mon.start()

    mon.send_program("movej(1)")

    assert sock.written == b"movej(1)\n"


def test_uses_sendall_not_send(make_monitor):
    # send() may write only part of the program and still report success,
    # leaving the robot with a truncated but still executable command.
    sock = FakeSocket()
    mon = make_monitor(sock)
    mon.start()

    mon.send_program("movej(1)")

    assert sock.calls == ["sendall"]


def test_surrounding_whitespace_is_stripped(make_monitor):
    # prog.strip() used to discard its result, so the trailing blank lines
    # below were sent to the robot.
    sock = FakeSocket()
    mon = make_monitor(sock)
    mon.start()

    mon.send_program("  movej(1)\n\n  ")

    assert sock.written == b"movej(1)\n"


def test_socket_error_reaches_the_caller(make_monitor):
    boom = socket.error("connection reset by peer")
    mon = make_monitor(FakeSocket(fail=boom))
    mon.start()

    with pytest.raises(socket.error) as excinfo:
        mon.send_program("movej(1)")

    assert excinfo.value is boom


def test_monitor_thread_survives_a_send_error(make_monitor):
    # This thread is the only reader of the secondary interface; if it dies,
    # all robot communication stops silently.
    mon = make_monitor(FakeSocket(fail=socket.error("boom")))
    mon.start()

    with pytest.raises(socket.error):
        mon.send_program("movej(1)")
    time.sleep(0.05)

    assert mon.is_alive()


def test_timeout_is_reported(make_monitor):
    mon = make_monitor(FakeSocket())  # never started, so nothing drains it
    start = time.monotonic()

    with pytest.raises(ursecmon.TimeoutException):
        mon.send_program("movej(1)", timeout=0.3)

    assert 0.25 <= time.monotonic() - start < 2.0


def test_timed_out_command_is_dequeued(make_monitor):
    mon = make_monitor(FakeSocket())

    with pytest.raises(ursecmon.TimeoutException):
        mon.send_program("movej(1)", timeout=0.2)

    assert mon._prog_queue == []


def test_timed_out_command_never_executes_later(make_monitor):
    # A stale move running after the caller gave up is the hazard here.
    sock = FakeSocket()
    mon = make_monitor(sock)

    with pytest.raises(ursecmon.TimeoutException):
        mon.send_program("movej(STALE)", timeout=0.2)
    mon.start()
    time.sleep(0.2)

    assert sock.written == b""
