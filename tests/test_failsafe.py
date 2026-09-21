"""T7.1–T7.13/T7.18/T7.19 — arm FSM, failsafe latches, LED (unit)."""

import ctypes
import math

import pytest

from _bind import bind

F3 = ctypes.c_float * 3
F4 = ctypes.c_float * 4

REASON_RC_TIMEOUT = 1
REASON_RC_FLAG = 2
REASON_IMU_INVALID = 4
REASON_VBAT_CRIT = 8
REASON_BOOT_LATCH = 16


@pytest.fixture
def fs(lib):
    fu = bind(lib, "failsafe_update", None,
              [ctypes.POINTER(ctypes.c_float), ctypes.c_int, ctypes.c_uint64, ctypes.c_int,
               ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float), ctypes.c_float,
               ctypes.c_uint64, ctypes.c_int, ctypes.c_int])
    return {
        "reset": bind(lib, "failsafe_reset", None),
        "boot": bind(lib, "failsafe_boot", None, [ctypes.c_uint32]),
        "update": fu,
        "armed": bind(lib, "failsafe_armed", ctypes.c_int),
        "active": bind(lib, "failsafe_active", ctypes.c_int),
        "zero": bind(lib, "failsafe_output_zero", ctypes.c_int),
        "reason": bind(lib, "failsafe_reason", ctypes.c_int),
        "vbat_nan": bind(lib, "failsafe_vbat_nan", ctypes.c_int),
        "boot_latched": bind(lib, "failsafe_boot_latched", ctypes.c_int),
        "good": bind(lib, "failsafe_good_frames", ctypes.c_int),
    }


def step(fs, *, t_us=0, now=0, frame_ok=1, armed_switch=1, rx_fs=0, lost=0, throttle=0.02,
         imu_valid=1, vbat=3.9, calibrated=1, arm_test=0):
    flags = armed_switch | (frame_ok << 1) | (rx_fs << 2) | (lost << 3)
    fs["update"](F4(0.0, 0.0, 0.0, throttle), flags, t_us, imu_valid,
                 F3(0, 0, 0), F3(0, 0, -9.81), vbat, now, calibrated, arm_test)


def test_t7_1_disarmed_output_zero(fs):
    fs["reset"]()
    step(fs)
    assert fs["armed"]() == 0
    assert fs["zero"]() == 1


def test_t7_2_timeout_boundary(fs):
    fs["reset"]()
    step(fs, t_us=0, now=0)
    step(fs, t_us=0, now=99999, frame_ok=0)
    assert fs["active"]() == 0
    step(fs, t_us=0, now=100000, frame_ok=0)
    assert fs["active"]() == 1
    assert fs["reason"]() & REASON_RC_TIMEOUT


def test_t7_3_rx_failsafe_flag(fs):
    fs["reset"]()
    step(fs, rx_fs=1)
    assert fs["active"]() == 1
    assert fs["reason"]() & REASON_RC_FLAG


def test_t7_4_frame_lost_warn_only(fs):
    fs["reset"]()
    for _ in range(5):
        step(fs, lost=1, frame_ok=1)
    assert fs["active"]() == 0


def test_t7_5_latch_and_recovery(fs):
    fs["reset"]()
    step(fs, rx_fs=1)
    assert fs["active"]() == 1
    for k in range(1, 50):
        step(fs, t_us=k * 1000, now=k * 1000)
    assert fs["active"]() == 1  # stays latched


@pytest.mark.parametrize("n,expect", [(9, 0), (10, 1)])
def test_t7_6_arm_frame_window(fs, n, expect):
    fs["reset"]()
    for k in range(1, n + 1):
        step(fs, t_us=k * 1000, now=k * 1000)
    assert fs["armed"]() == expect


def test_t7_7_arm_high_throttle_rejected(fs):
    fs["reset"]()
    for k in range(1, 30):
        step(fs, t_us=k * 1000, now=k * 1000, throttle=0.5)
    assert fs["armed"]() == 0


def test_t7_8_arm_needs_calibration(fs):
    fs["reset"]()
    for k in range(1, 30):
        step(fs, t_us=k * 1000, now=k * 1000, calibrated=0)
    assert fs["armed"]() == 0


def test_t7_9_stale_frame_not_counted(fs):
    fs["reset"]()
    for _ in range(30):
        step(fs, t_us=1000, now=1000)  # non-increasing t_us
    assert fs["armed"]() == 0


def test_t7_10_imu_invalid_disarms(fs):
    fs["reset"]()
    step(fs, arm_test=1)
    assert fs["armed"]() == 1
    for _ in range(6):
        step(fs, arm_test=1, imu_valid=0)
    assert fs["armed"]() == 0
    assert fs["active"]() == 1
    assert fs["reason"]() & REASON_IMU_INVALID


@pytest.mark.parametrize("vbat,crit", [(3.4, 0), (3.2, 1)])
def test_t7_11_vbat_monitor(fs, vbat, crit):
    fs["reset"]()
    step(fs, t_us=1000, now=1000)
    step(fs, t_us=2000, now=2000, vbat=vbat)
    assert bool(fs["active"]()) == bool(crit)
    if crit:
        assert fs["reason"]() & REASON_VBAT_CRIT


def test_t7_12_vbat_nan_not_healthy(fs):
    fs["reset"]()
    step(fs, vbat=math.nan)
    assert fs["vbat_nan"]() == 1
    assert fs["active"]() == 0


@pytest.mark.parametrize("reason,latched", [(9, 1), (6, 1), (1, 0)])
def test_t7_13_reset_reason(fs, reason, latched):
    fs["boot"](reason)
    assert fs["boot_latched"]() == latched
    assert fs["armed"]() == 0


def test_t7_18_led_fsm(lib):
    led_set = bind(lib, "led_set", None, [ctypes.c_int])
    led_update = bind(lib, "led_update", None, [ctypes.c_uint32])
    led_on = bind(lib, "led_on", ctypes.c_int)
    led_mode = bind(lib, "led_mode", ctypes.c_int)
    led_set(2)  # armed -> solid
    led_update(1000)
    assert led_on() == 1 and led_mode() == 2
    led_set(3)  # error -> fast blink
    led_update(0)
    on0 = led_on()
    led_update(60)
    on60 = led_on()
    assert on0 != on60
