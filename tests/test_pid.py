"""T5.1–T5.6 — PID unit behavior: P, I clamp, conditional integration, D."""

import ctypes
import math

import pytest

from _bind import bind


@pytest.fixture
def pid(lib):
    bind(lib, "pid_reset", None)
    step = bind(lib, "pid_step", ctypes.c_float,
                [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_int, ctypes.c_int])
    setg = bind(lib, "pid_set_gains", None,
                [ctypes.c_float] * 6)
    reset = bind(lib, "pid_reset", None)
    return {"step": step, "set": setg, "reset": reset}


def test_t5_1_p_only(pid):
    pid["set"](2.0, 0.0, 0.0, 1.0, 10.0, 100.0)
    pid["reset"]()
    assert pid["step"](1.0, 0.0, 0.001, 0, 0) == pytest.approx(2.0, abs=1e-6)


def test_t5_2_i_clamp(pid):
    pid["set"](0.0, 1.0, 0.0, 0.05, 100.0, 100.0)
    pid["reset"]()
    out = 0.0
    for _ in range(200):
        out = pid["step"](1.0, 0.0, 0.001, 0, 0)
    assert abs(out) <= 0.05 + 1e-6


def test_t5_3_conditional_integration(pid):
    pid["set"](0.0, 1.0, 0.0, 10.0, 100.0, 100.0)
    pid["reset"]()
    a = pid["step"](1.0, 0.0, 0.001, 1, 0)  # saturated positive while e > 0
    b = pid["step"](1.0, 0.0, 0.001, 1, 0)
    assert b == pytest.approx(a, abs=1e-9)  # integral held

    pid["reset"]()
    c = pid["step"](1.0, 0.0, 0.001, 0, 0)
    d = pid["step"](1.0, 0.0, 0.001, 0, 0)
    assert d > c  # integral accumulates when not saturated


def test_t5_4_d_on_measurement(pid):
    pid["set"](0.0, 0.0, 1.0, 0.0, 1000.0, 100.0)
    pid["reset"]()
    # setpoint jump with static measurement -> no derivative kick
    assert pid["step"](1.0, 0.0, 0.001, 0, 0) == pytest.approx(0.0, abs=1e-6)
    assert pid["step"](1.0, 0.0, 0.001, 0, 0) == pytest.approx(0.0, abs=1e-6)
    # measurement movement -> D responds
    assert pid["step"](1.0, 0.2, 0.001, 0, 0) != pytest.approx(0.0, abs=1e-6)


def test_t5_5_d_low_pass(pid):
    pid["set"](0.0, 0.0, 1.0, 0.0, 1000.0, 5.0)  # heavy LPF
    pid["reset"]()
    pid["step"](0.0, 0.0, 0.001, 0, 0)
    slow = abs(pid["step"](0.0, 0.1, 0.001, 0, 0))

    pid["set"](0.0, 0.0, 1.0, 0.0, 1000.0, 1000.0)  # light LPF
    pid["reset"]()
    pid["step"](0.0, 0.0, 0.001, 0, 0)
    fast = abs(pid["step"](0.0, 0.1, 0.001, 0, 0))
    assert slow < fast


def test_t5_6_nan_guard(pid):
    pid["set"](1.0, 1.0, 1.0, 1.0, 1.0, 100.0)
    pid["reset"]()
    assert pid["step"](math.nan, 0.0, 0.001, 0, 0) == 0.0
    assert pid["step"](0.0, math.inf, 0.001, 0, 0) == 0.0
    assert math.isfinite(pid["step"](1.0, 0.0, 0.0, 0, 0))
