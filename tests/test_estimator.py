"""T6.1–T6.6/T6.13 — complementary estimator + bias calibration (unit)."""

import ctypes
import math

import numpy as np
import pytest

from _bind import bind

DPS = math.pi / 180.0
F3 = ctypes.c_float * 3


@pytest.fixture
def est(lib):
    return {
        "reset": bind(lib, "estimator_reset", None),
        "calibrate": bind(lib, "estimator_calibrate", None,
                          [ctypes.POINTER(ctypes.c_float), ctypes.c_uint64]),
        "update": bind(lib, "estimator_update", None,
                       [ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
                        ctypes.c_float]),
        "update_v": bind(lib, "estimator_update_v", None,
                         [ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float),
                          ctypes.c_float, ctypes.c_int]),
        "att": bind(lib, "estimator_attitude", None, [ctypes.POINTER(ctypes.c_float)]),
        "calibrated": bind(lib, "estimator_calibrated", ctypes.c_int),
        "imu_valid": bind(lib, "estimator_imu_valid", ctypes.c_int),
        "bias": bind(lib, "estimator_bias", None, [ctypes.POINTER(ctypes.c_float)]),
    }


def _attitude(est):
    out = F3()
    est["att"](out)
    return [math.degrees(x) for x in out]


def _calibrate(est, bias):
    est["reset"]()
    g = F3(*bias)
    for i in range(250):
        est["calibrate"](g, i * 1000)
    assert est["calibrated"]() == 1


def test_t6_1_convergence(est):
    est["reset"]()
    rad = math.radians(20.0)
    ay = -math.sin(rad) * 9.81
    az = -math.cos(rad) * 9.81
    for _ in range(6000):
        est["update"](F3(0, 0, 0), F3(0, ay, az), 0.001)
    assert abs(_attitude(est)[0] - 20.0) < 1.0
    for _ in range(6000):
        est["update"](F3(0, 0, 0), F3(0, 0, -9.81), 0.001)
    assert abs(_attitude(est)[0]) < 2.0


def test_t6_2_bias_removed(est):
    bias = [5.0 * DPS, -3.0 * DPS, 2.0 * DPS]
    _calibrate(est, bias)
    for _ in range(10000):
        est["update"](F3(*bias), F3(0, 0, -9.81), 0.001)
    assert abs(_attitude(est)[2]) < 1.0  # no yaw drift


def test_t6_3_accel_noise(est):
    est["reset"]()
    rng = np.random.default_rng(0)
    peak = 0.0
    for _ in range(5000):
        a = [0.0, 0.0, -9.81] + rng.normal(0, 0.2, 3)
        est["update"](F3(0, 0, 0), F3(*a), 0.001)
        peak = max(peak, abs(_attitude(est)[0]))
    assert peak < 3.0


def test_t6_4_yaw_integration(est):
    _calibrate(est, [0, 0, 0])
    rate = (math.pi / 2) / 2.5  # 90 deg over 2.5 s
    for _ in range(2500):
        est["update"](F3(0, 0, rate), F3(0, 0, -9.81), 0.001)
    assert abs(_attitude(est)[2] - 90.0) < 5.0


def test_t6_5_nan_guard(est):
    est["reset"]()
    for _ in range(20):
        est["update_v"](F3(math.nan, 0, 0), F3(0, 0, -9.81), 0.001, 1)
        est["update_v"](F3(math.inf, 0, 0), F3(0, 0, -9.81), 0.001, 1)
    assert all(math.isfinite(x) for x in _attitude(est))
    assert est["imu_valid"]() == 0


def test_t6_6_calib_rejects_motion(est):
    est["reset"]()
    g = F3(0.5, 0, 0)  # > 20 dps
    for i in range(500):
        est["calibrate"](g, i * 1000)
    assert est["calibrated"]() == 0


def test_t6_13_imu_valid_debounce(est):
    est["reset"]()
    for _ in range(10):
        est["update_v"](F3(0, 0, 0), F3(0, 0, -9.81), 0.001, 0)
    assert est["imu_valid"]() == 0
    for _ in range(10):
        est["update_v"](F3(0, 0, 0), F3(0, 0, -9.81), 0.001, 1)
    assert est["imu_valid"]() == 1
