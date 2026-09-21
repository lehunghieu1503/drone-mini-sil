"""T3.11–T3.13 — SBUS golden frame, resync, endpoints, stuck/oversize."""

import ctypes

import pytest

from _bind import bind
from plant.rc import sbus_encode, FLAG_FAILSAFE, FLAG_FRAME_LOST

# Non-stuck base: ch5 high (arm), a couple of distinct values.
BASE = [992, 992, 992, 992, 1811, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
THR_MID = (992 - 172) / (1811 - 172)


@pytest.fixture
def rc(lib):
    return {
        "reset": bind(lib, "rc_parse_reset", None),
        "feed": bind(lib, "rc_parse_feed", ctypes.c_int,
                     [ctypes.POINTER(ctypes.c_uint8), ctypes.c_int]),
        "roll": bind(lib, "rc_roll", ctypes.c_float),
        "pitch": bind(lib, "rc_pitch", ctypes.c_float),
        "yaw": bind(lib, "rc_yaw", ctypes.c_float),
        "thr": bind(lib, "rc_throttle", ctypes.c_float),
        "arm": bind(lib, "rc_armed_switch", ctypes.c_int),
        "frame_ok": bind(lib, "rc_frame_ok", ctypes.c_int),
        "rx_fs": bind(lib, "rc_rx_failsafe", ctypes.c_int),
        "lost": bind(lib, "rc_frame_lost", ctypes.c_int),
    }


def _feed(rc, data):
    buf = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
    return rc["feed"](buf, len(data))


def test_t3_11_golden_frame(rc):
    rc["reset"]()
    assert _feed(rc, sbus_encode(BASE)) == 1
    assert rc["roll"]() == pytest.approx(0.0, abs=1e-6)
    assert rc["pitch"]() == pytest.approx(0.0, abs=1e-6)
    assert rc["thr"]() == pytest.approx(THR_MID, abs=1e-6)
    assert rc["arm"]() == 1
    assert rc["frame_ok"]() == 1
    assert rc["rx_fs"]() == 0


def test_t3_11_flags(rc):
    rc["reset"]()
    assert _feed(rc, sbus_encode(BASE, FLAG_FAILSAFE)) == 1
    assert rc["rx_fs"]() == 1
    assert rc["frame_ok"]() == 0
    rc["reset"]()
    assert _feed(rc, sbus_encode(BASE, FLAG_FRAME_LOST)) == 1
    assert rc["lost"]() == 1


def test_t3_12_resync(rc):
    rc["reset"]()
    frame = sbus_encode([992, 992, 1500, 992] + [0] * 12)
    assert _feed(rc, b"\x01\x02\x03\x04" + frame) == 1
    assert rc["thr"]() == pytest.approx((1500 - 172) / (1811 - 172), abs=1e-6)


def test_t3_12_upper_nibble_rejected(rc):
    rc["reset"]()
    frame = bytearray(sbus_encode(BASE))
    frame[23] = 0x10
    assert _feed(rc, bytes(frame)) == 0


def test_t3_13_endpoints(rc):
    rc["reset"]()
    assert _feed(rc, sbus_encode([172, 992, 172, 992] + [0] * 12)) == 1
    assert rc["roll"]() == pytest.approx(-1.0, abs=1e-6)
    rc["reset"]()
    assert _feed(rc, sbus_encode([1811, 992, 1811, 992] + [0] * 12)) == 1
    assert rc["roll"]() == pytest.approx(1.0, abs=1e-6)
    assert rc["thr"]() == pytest.approx(1.0, abs=1e-6)


def test_t3_13_stuck_rejected(rc):
    rc["reset"]()
    assert _feed(rc, sbus_encode([800] * 16)) == 0


def test_t3_13_oversize_rejected(rc):
    rc["reset"]()
    assert _feed(rc, b"\x00" * 70) == 0
    assert _feed(rc, sbus_encode(BASE)) == 1  # still works after the reject
