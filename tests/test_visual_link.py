"""Phase 1 — visual link core. Headless: no display, no mujoco required."""

import os
import pathlib
import socket
import subprocess
import sys

import pytest

from plant.visual_link import (POSE_SIZE, PoseSender, pack_pose, unpack_pose,
                               visual_pose)


def test_pose_size_and_roundtrip():
    assert POSE_SIZE == 36
    t_us, pos, quat = 123456789, (1.5, -2.25, 3.0), (0.707, 0.0, 0.707, 0.0)
    out = unpack_pose(pack_pose(t_us, pos, quat))
    assert out[0] == t_us
    assert out[1] == pytest.approx(pos, abs=1e-6)
    assert out[2] == pytest.approx(quat, abs=1e-6)


def test_unpack_rejects_bad_size():
    with pytest.raises(ValueError):
        unpack_pose(b"\x00" * (POSE_SIZE - 1))


@pytest.mark.parametrize("plant,z_in,z_out", [
    ("mujoco", 0.012496, 0.012496),   # COM already at contact height: no offset
    ("rk4", 0.0, 0.0125),
    ("rk4", 2.0, 2.0125),
    ("rotorpy", -1.0, 0.0125),        # clamped to the floor first
])
def test_visual_pose_per_plant(plant, z_in, z_out):
    pos, quat = visual_pose(plant, (0.0, 0.0, z_in), (1.0, 0.0, 0.0, 0.0))
    assert pos[2] == pytest.approx(z_out, abs=1e-9)
    assert quat == (1.0, 0.0, 0.0, 0.0)


class _FakeSock:
    def __init__(self, fail_first=0):
        self.sent = []
        self.fail_first = fail_first
        self.closed = False

    def send(self, data):
        if self.fail_first > 0:
            self.fail_first -= 1
            raise ConnectionRefusedError("no viewer")
        self.sent.append(data)

    def close(self):
        self.closed = True


class _Clock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


def test_sender_backoff_then_retry():
    clock = _Clock(0.0)
    sock = _FakeSock(fail_first=1)
    s = PoseSender(hz=1000.0, clock=clock, sock=sock)

    assert s.publish(0, "rk4", (0, 0, 0), (1, 0, 0, 0)) is False  # failure
    assert s.failed == 1
    attempts = s.attempts
    clock.t = 0.5  # inside the 1 s backoff window: no attempt
    for _ in range(10):
        assert s.publish(0, "rk4", (0, 0, 0), (1, 0, 0, 0)) is False
    assert s.attempts == attempts  # skipped, not attempted
    # after the backoff: retries and succeeds (viewer started late)
    clock.t = 2.0
    assert s.publish(0, "rk4", (0, 0, 0), (1, 0, 0, 0)) is True
    assert s.failed == 0 and s.sent == 1 and s.healthy


def test_sender_caps_rate_on_wall_clock():
    clock = _Clock(0.0)
    sock = _FakeSock()
    s = PoseSender(hz=200.0, clock=clock, sock=sock)  # 5 ms min interval
    sent = 0
    for i in range(10):  # 0,1,...,9 ms
        clock.t = i * 0.001
        sent += bool(s.publish(i, "rk4", (0, 0, 0), (1, 0, 0, 0)))
    assert sent == 2  # t=0 and t=5 ms, independent of sim speed
    assert s.attempts == 2


def test_sender_close_idempotent_and_publish_after_close():
    s = PoseSender(clock=_Clock(0.0), sock=_FakeSock())
    s.close()
    s.close()
    assert s.publish(0, "rk4", (0, 0, 0), (1, 0, 0, 0)) is False


def test_sender_real_socket_to_closed_port_never_raises():
    s = PoseSender(port=1, hz=1e6)  # port 1: nothing listening
    try:
        for i in range(5):
            s.publish(i, "rk4", (0, 0, 0), (1, 0, 0, 0))
    finally:
        s.close()


def test_module_does_not_import_mujoco():
    """Check in a clean interpreter (the suite imports mujoco for other tests)."""
    root = pathlib.Path(__file__).resolve().parents[1]
    code = ("import plant.visual_link, sys; "
            "assert 'mujoco' not in sys.modules, 'visual_link pulled in mujoco'; "
            "print('ok')")
    out = subprocess.run([sys.executable, "-c", code], cwd=root, text=True,
                         capture_output=True, env={**os.environ, "PYTHONPATH": "."})
    assert out.returncode == 0, out.stderr


def test_sender_uses_datagram_socket():
    """connect() must be on UDP: no TCP handshake, publish never blocks/raises."""
    s = PoseSender(port=1)
    try:
        assert s._sock.type & socket.SOCK_DGRAM
        for i in range(5):
            assert s.publish(i, "rk4", (0, 0, 0), (1, 0, 0, 0)) in (True, False)
    finally:
        s.close()
