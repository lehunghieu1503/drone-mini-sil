"""T1.8 — the OutputStage safety invariant: armed=0 => 4 channels 0 + pwmOff."""

import ctypes
import itertools
import math

import pytest

from _bind import bind

F4 = ctypes.c_float * 4


@pytest.fixture
def api(lib):
    return {
        "apply": bind(lib, "output_stage_apply", ctypes.c_int,
                      [ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.c_int, ctypes.c_int]),
        "reset": bind(lib, "mock_reset", None),
        "pwm_offs": bind(lib, "mock_pwm_offs", ctypes.c_int),
        "pwm_writes": bind(lib, "mock_pwm_writes", ctypes.c_int),
        "last_pwm": bind(lib, "mock_last_pwm", None, [ctypes.POINTER(ctypes.c_float)]),
    }


def _last(api):
    out = (ctypes.c_float * 4)()
    api["last_pwm"](out)
    return list(out)


@pytest.mark.parametrize("armed,failsafe,imu_valid",
                         list(itertools.product([0, 1], repeat=3)))
def test_t1_8_gate_matrix(api, armed, failsafe, imu_valid):
    api["reset"]()
    inp = (ctypes.c_float * 4)(0.5, 0.5, 0.5, 0.5)
    wrote = api["apply"](inp, armed, failsafe, imu_valid)
    gate_open = bool(armed) and not failsafe and bool(imu_valid)
    if not gate_open:
        assert wrote == 0
        assert api["pwm_offs"]() == 1
        assert api["pwm_writes"]() == 0
        assert _last(api) == [0.0, 0.0, 0.0, 0.0]
    else:
        assert wrote == 1
        assert api["pwm_writes"]() == 1
        assert _last(api) == pytest.approx([0.5, 0.5, 0.5, 0.5])


def test_out_of_range_fails_closed(api):
    api["reset"]()
    inp = (ctypes.c_float * 4)(0.5, 1.5, 0.5, 0.5)
    assert api["apply"](inp, 1, 0, 1) == 0
    assert api["pwm_offs"]() == 1
    assert _last(api) == [0.0, 0.0, 0.0, 0.0]


def test_nan_fails_closed(api):
    api["reset"]()
    inp = (ctypes.c_float * 4)(0.5, math.nan, 0.5, 0.5)
    assert api["apply"](inp, 1, 0, 1) == 0
    assert api["pwm_offs"]() == 1
