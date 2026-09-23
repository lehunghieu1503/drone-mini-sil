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
REASON_VBAT_NAN = 32


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


def step(fs, *, t_us=0, now=0, frame_ok=1, armed_switch=1, rx_fs=0, lost=0,
         roll=0.0, pitch=0.0, yaw=0.0, throttle=0.02,
         imu_valid=1, vbat=3.9, calibrated=1, arm_test=0):
    flags = armed_switch | (frame_ok << 1) | (rx_fs << 2) | (lost << 3)
    fs["update"](F4(roll, pitch, yaw, throttle), flags, t_us, imu_valid,
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


def test_t7_11_vbat_crit_debounce(fs):
    """D4: 19 samples below crit do not latch; the 20th consecutive does."""
    fs["reset"]()
    step(fs, t_us=1000, now=1000)  # healthy baseline
    step(fs, t_us=2000, now=2000, vbat=3.2)  # one sample: no latch
    assert fs["active"]() == 0
    for k in range(3, 21):  # 18 more -> 19 consecutive samples
        step(fs, t_us=k * 1000, now=k * 1000, vbat=3.2)
    assert fs["active"]() == 0
    step(fs, t_us=21000, now=21000, vbat=3.2)  # 20th consecutive
    assert fs["active"]() == 1
    assert fs["reason"]() & REASON_VBAT_CRIT


def test_t7_12_vbat_nan_latches(fs):
    """D4: one non-finite sample latches immediately and never arms."""
    fs["reset"]()
    step(fs, vbat=math.nan)
    assert fs["vbat_nan"]() == 1
    assert fs["active"]() == 1
    assert fs["reason"]() & REASON_VBAT_NAN
    assert fs["armed"]() == 0


def test_t7_24_stick_deadband_blocks_arm(fs):
    """D5: a stick outside the deadband blocks the arm transition."""
    fs["reset"]()
    for k in range(1, 30):
        step(fs, t_us=k * 1000, now=k * 1000, roll=0.2)
    assert fs["armed"]() == 0
    for k in range(30, 40):
        step(fs, t_us=k * 1000, now=k * 1000, roll=0.0)
    assert fs["armed"]() == 1


def test_t7_25_switch_cycle_clears_latch(fs):
    """D3: an in-flight latch clears on a switch off->on edge with a fresh frame."""
    fs["reset"]()
    step(fs, t_us=1000, now=1000, rx_fs=1)  # latch RX failsafe
    assert fs["active"]() == 1
    step(fs, t_us=2000, now=2000, armed_switch=0, rx_fs=1)  # switch off
    for k in range(3, 20):
        step(fs, t_us=k * 1000, now=k * 1000)  # switch on edge at k=3 clears it
    assert fs["active"]() == 0
    assert fs["armed"]() == 1


def test_t7_26_boot_latch_blocks_switch_cycle_and_arm_test(fs):
    """D3: boot latch is never cleared by a switch cycle or arm_test."""
    fs["boot"](9)
    step(fs, t_us=1000, now=1000, rx_fs=1)
    step(fs, t_us=2000, now=2000, armed_switch=0, rx_fs=1)
    for k in range(3, 20):
        step(fs, t_us=k * 1000, now=k * 1000)
    assert fs["armed"]() == 0
    for k in range(20, 30):
        step(fs, t_us=k * 1000, now=k * 1000, arm_test=1)
    assert fs["armed"]() == 0
    assert fs["boot_latched"]() == 1


def test_t7_27_arm_test_does_not_clear_latch(fs):
    """arm_test arms open-loop only when nothing is latched."""
    fs["reset"]()
    step(fs, t_us=1000, now=1000, rx_fs=1)  # latch
    assert fs["active"]() == 1
    for k in range(2, 12):
        step(fs, t_us=k * 1000, now=k * 1000, arm_test=1)
    assert fs["armed"]() == 0
    assert fs["active"]() == 1


def test_t7_28_arm_test_arms_when_clean(fs):
    fs["reset"]()
    step(fs, arm_test=1)
    assert fs["armed"]() == 1
    assert fs["active"]() == 0


@pytest.mark.parametrize("reason,latched", [(9, 1), (6, 1), (1, 0)])
def test_t7_13_reset_reason(fs, reason, latched):
    fs["boot"](reason)
    assert fs["boot_latched"]() == latched
    assert fs["armed"]() == 0


def test_t7_20_quiet_hold_keeps_armed(fs):
    """D1: a silent tick (frozen t_us) must not disarm or reset good_frames_."""
    fs["reset"]()
    step(fs, t_us=0, now=0, arm_test=1)  # arm on one fresh frame
    assert fs["armed"]() == 1
    g0 = fs["good"]()
    assert g0 >= 1
    for k in range(1, 51):
        step(fs, t_us=0, now=k * 1000, frame_ok=1)  # frozen t_us, switch held on
    assert fs["armed"]() == 1
    assert fs["good"]() >= g0  # quiet ticks must not reset the counter
    assert fs["active"]() == 0


def test_t7_21_quiet_timeout_latches(fs):
    """D1/D2: 100 ms after the last fresh frame the RC timeout latches."""
    fs["reset"]()
    step(fs, t_us=0, now=0, arm_test=1)
    assert fs["armed"]() == 1
    step(fs, t_us=0, now=99999, frame_ok=1)
    assert fs["active"]() == 0
    step(fs, t_us=0, now=100000, frame_ok=1)
    assert fs["active"]() == 1
    assert fs["armed"]() == 0
    assert fs["reason"]() & REASON_RC_TIMEOUT


def test_t7_22_quiet_switch_off_disarms(fs):
    """A decoded frame with the switch off disarms immediately."""
    fs["reset"]()
    step(fs, t_us=0, now=0, arm_test=1)
    assert fs["armed"]() == 1
    step(fs, t_us=1000, now=1000, armed_switch=0)
    assert fs["armed"]() == 0


def test_t7_23_held_frame_ok_still_armed(fs):
    """frame_ok=0 with a frozen t_us and switch on stays armed (held switch)."""
    fs["reset"]()
    step(fs, t_us=0, now=0, arm_test=1)
    assert fs["armed"]() == 1
    step(fs, t_us=0, now=50000, frame_ok=0)
    assert fs["armed"]() == 1
    assert fs["active"]() == 0
    step(fs, t_us=0, now=50000, frame_ok=0, armed_switch=0)
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
