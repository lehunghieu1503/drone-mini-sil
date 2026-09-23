"""Phase 2 — mixer sign table, clip, desaturation (T2.1–T2.11)."""

import ctypes
import math

import pytest

from _bind import bind

F4 = ctypes.c_float * 4

# Independent hard-coded A1 sign table (must NOT be imported from plant).
A1_SIGN = {
    "roll": [-1, -1, +1, +1],
    "pitch": [+1, -1, +1, -1],
    "yaw": [-1, +1, +1, -1],
}


@pytest.fixture
def mixer(lib):
    fn = bind(lib, "mixer_write", ctypes.c_float,
              [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
               ctypes.POINTER(ctypes.c_float)])

    def call(thr, roll, pitch, yaw):
        out = (ctypes.c_float * 4)()
        shift = fn(thr, roll, pitch, yaw, out)
        return list(out), shift

    return call


def test_t2_1_hover(mixer):
    m, _ = mixer(0.5, 0.0, 0.0, 0.0)
    assert m == pytest.approx([0.5, 0.5, 0.5, 0.5], abs=1e-6)


def test_t2_2_roll_pos(mixer):
    m, _ = mixer(0.5, 0.2, 0.0, 0.0)
    assert m == pytest.approx([0.3, 0.3, 0.7, 0.7], abs=1e-6)


def test_t2_3_roll_neg(mixer):
    m, _ = mixer(0.5, -0.2, 0.0, 0.0)
    assert m == pytest.approx([0.7, 0.7, 0.3, 0.3], abs=1e-6)


def test_t2_4_pitch_pos(mixer):
    m, _ = mixer(0.5, 0.0, 0.2, 0.0)
    assert m == pytest.approx([0.7, 0.3, 0.7, 0.3], abs=1e-6)


def test_t2_5_yaw_pos(mixer):
    m, _ = mixer(0.5, 0.0, 0.0, 0.2)
    assert m == pytest.approx([0.3, 0.7, 0.7, 0.3], abs=1e-6)


def test_t2_6_idle_clip_no_desat(mixer):
    """D5: idle must clip, not lift the collective (shift is 0)."""
    m, shift = mixer(0.0, 0.0, 0.0, 0.5)
    assert m == pytest.approx([0.0, 0.5, 0.5, 0.0], abs=1e-6)
    assert shift == pytest.approx(0.0)


def test_t2_7_clip_high(mixer):
    m, shift = mixer(1.0, 0.0, 0.5, 0.0)
    assert max(m) <= 1.0 + 1e-6
    assert shift < 0


def test_t2_8_duty_differential_preserved(mixer):
    thr, roll = 0.9, 0.3
    pre = [thr - roll, thr - roll, thr + roll, thr + roll]
    m, _ = mixer(thr, roll, 0.0, 0.0)
    assert (max(m) - min(m)) == pytest.approx(max(pre) - min(pre), abs=1e-6)


def test_t2_9_nan_inf_guard(mixer):
    for bad in (math.nan, math.inf, -math.inf):
        m, _ = mixer(bad, 0.0, 0.0, 0.0)
        assert all(math.isfinite(x) and 0.0 <= x <= 1.0 for x in m)
    m, _ = mixer(0.5, math.nan, math.inf, 0.0)
    assert all(math.isfinite(x) and 0.0 <= x <= 1.0 for x in m)


def test_t2_10_not_armed_via_output(lib):
    mixer_fn = bind(lib, "mixer_write", ctypes.c_float,
                    [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
                     ctypes.POINTER(ctypes.c_float)])
    out = (ctypes.c_float * 4)()
    mixer_fn(0.5, 0.1, 0.1, 0.1, out)
    apply_fn = bind(lib, "output_stage_apply", ctypes.c_int,
                    [ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.c_int, ctypes.c_int])
    bind(lib, "mock_reset", None)()
    assert apply_fn(out, 0, 0, 1) == 0
    last = (ctypes.c_float * 4)()
    bind(lib, "mock_last_pwm", None, [ctypes.POINTER(ctypes.c_float)])(last)
    assert list(last) == [0.0, 0.0, 0.0, 0.0]


@pytest.mark.parametrize("axis", ["roll", "pitch", "yaw"])
def test_t2_11_sign_vs_a1(mixer, axis):
    d = 0.05
    inputs = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    inputs[axis] = d
    m, _ = mixer(0.5, inputs["roll"], inputs["pitch"], inputs["yaw"])
    for i, expected in enumerate(A1_SIGN[axis]):
        delta = m[i] - 0.5
        assert math.copysign(1, delta) == expected or abs(delta) < 1e-9


def test_t2_12_idle_no_collective_lift(mixer):
    """D5: an idle differential must not be desaturated up to 0.80."""
    m, shift = mixer(0.02, 0.40, 0.0, 0.0)
    assert m == pytest.approx([0.0, 0.0, 0.42, 0.42], abs=1e-6)
    assert max(m) <= 0.02 + 0.40 + 1e-6
    assert shift == pytest.approx(0.0)


def _flags_fn(lib):
    return bind(lib, "mixer_write_flags", ctypes.c_int,
                [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
                 ctypes.POINTER(ctypes.c_float)])


def test_t2_13_sat_flags_from_raw(lib):
    """D6: flags come from the raw channels, not the clipped duty."""
    fn = _flags_fn(lib)
    out = (ctypes.c_float * 4)()
    # Idle + roll: raw negative channels -> sat_neg only, output clipped.
    flags = fn(0.02, 0.40, 0.0, 0.0, out)
    assert flags & 2 and not (flags & 1)
    assert list(out) == pytest.approx([0.0, 0.0, 0.42, 0.42], abs=1e-6)
    # write(1, 0, 0.5, 0): raw all >= 0.5 -> high side only.
    flags = fn(1.0, 0.0, 0.5, 0.0, out)
    assert flags & 1 and not (flags & 2)
    assert list(out) == pytest.approx([1.0, 0.0, 1.0, 0.0], abs=1e-6)


def test_t2_14_rate_remembers_sat_flags(lib):
    """D6: the rate controller carries the pre-clip flags into the next tick."""
    F3 = ctypes.c_float * 3
    rate_reset = bind(lib, "rate_reset", None)
    rate_update = bind(lib, "rate_update", ctypes.c_float,
                       [ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
                        ctypes.c_float, ctypes.POINTER(ctypes.c_float)])
    rate_sat = bind(lib, "rate_sat_flags", ctypes.c_int, [ctypes.POINTER(ctypes.c_int)])
    out = (ctypes.c_float * 4)()
    rate_reset()
    rate_update(F3(0.0, 0.0, 0.0), F3(1.0, 0.0, 0.0), 0.02, out)  # idle, big roll sp
    flags = (ctypes.c_int * 2)()
    rate_sat(flags)
    assert flags[1] == 1 and flags[0] == 0  # sat_neg remembered, no high sat
