"""Outer pilot: stick signs and the key map. No firmware."""

import numpy as np
import pytest

from plant.pilot import HoverPilot, Manual
from plant.stick_keys import manual_from_active


def test_manual_w_flies_forward():
    # Positive pitch accelerates world +x on this plant.
    manual = manual_from_active({"w"}, set())
    assert manual.pitch > 0.0
    assert manual.roll == 0.0


def test_keys_are_edges_or_holds():
    manual = manual_from_active({"a", "r"}, {" ", "l", "x"})
    assert manual.roll < 0.0
    assert manual.alt_rate > 0.0
    assert manual.toggle_arm and manual.land and manual.quit


def test_unarmed_stick_stays_inside_the_arm_gate():
    pilot = HoverPilot(1.0)
    cmd = pilot.command(np.zeros(3), np.zeros(3), np.array([1.0, 0, 0, 0]),
                        3.85, armed=False, manual=Manual(), dt=0.001)
    assert cmd.arm
    assert cmd.throttle <= 0.05
    assert cmd.roll == cmd.pitch == cmd.yaw == 0.0


def test_forward_speed_brakes_with_negative_pitch():
    pilot = HoverPilot(1.0)
    q = np.array([1.0, 0.0, 0.0, 0.0])
    origin = np.zeros(3)
    pilot.command(origin, np.zeros(3), q, 3.85, armed=False, manual=Manual(), dt=0.001)
    # Arm edge captures the setpoint. +x speed must brake, and +pitch would add +x.
    pilot.command(origin, np.zeros(3), q, 3.85, armed=True, manual=Manual(), dt=0.001)
    cmd = pilot.command(origin, np.array([1.0, 0.0, 0.0]), q, 3.85, armed=True,
                        manual=Manual(), dt=0.001)
    assert cmd.pitch < 0.0
    assert abs(cmd.roll) < 1e-6


def test_below_setpoint_adds_throttle():
    pilot = HoverPilot(1.0)
    q = np.array([1.0, 0.0, 0.0, 0.0])
    pilot.command(np.zeros(3), np.zeros(3), q, 3.85, False, Manual(), 0.001)
    idle = pilot.command(np.zeros(3), np.zeros(3), q, 3.85, True, Manual(), 0.001)
    low = pilot.command(np.array([0.0, 0.0, 0.2]), np.zeros(3), q, 3.85, True, Manual(), 0.001)
    assert low.throttle > idle.throttle


def test_bad_altitude_rejected():
    with pytest.raises(ValueError):
        HoverPilot(-1.0)
