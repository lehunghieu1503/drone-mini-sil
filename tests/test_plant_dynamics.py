"""T3.1/T3.2/T3.5/T3.10 — plant algebra, free fall, motor lag, sign contract."""

import ctypes
import math

import pytest

from _bind import bind
from plant import drone_mini_params as P
from plant.battery import Battery
from plant.vehicle import make_plant

F4 = ctypes.c_float * 4


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


def test_t3_1_hover_identity():
    duty = P.hover_duty_nominal()
    omega = P.pwm_to_omega(duty, P.BATTERY["vbat_nominal"])
    assert 4.0 * P.K_T * omega * omega == pytest.approx(P.MASS_KG * P.GRAVITY, rel=1e-6)
    assert duty == pytest.approx(0.5, abs=0.02)


def test_t3_2_free_fall():
    plant = make_plant("rk4", clean=True, seed=0)
    plant.reset(alt=10.0)
    steps = int(0.5 / P.DT)
    for _ in range(steps):
        plant.step(P.DT, [0, 0, 0, 0], P.BATTERY["vbat_nominal"])
    assert plant.vel[2] == pytest.approx(-P.GRAVITY * 0.5, rel=0.02)
    assert abs(plant.omega[2]) < 1e-9  # no yaw spin
    assert float(plant.motors.max()) == 0.0


def test_t3_5_motor_step_tau():
    vbat = P.BATTERY["vbat_nominal"]
    plant = make_plant("rk4", clean=True)
    plant.reset(alt=50.0)  # in the air so ground clamp cannot interfere
    om_max = P.pwm_to_omega(1.0, vbat)
    steps = int(round(P.TAU_M_S / P.DT))
    for _ in range(steps):
        plant.step(P.DT, [1, 1, 1, 1], vbat)
    assert plant.motors[0] / om_max == pytest.approx(1.0 - math.exp(-1.0), rel=0.02)


def test_t3_11_arm_geometry():
    """D10: positions are Descartes offsets; the model radius is d*sqrt(2)."""
    for pos in P.ROTOR_POSITIONS:
        assert abs(pos[0]) == pytest.approx(P.ARM_LENGTH_M)
        assert abs(pos[1]) == pytest.approx(P.ARM_LENGTH_M)
        assert math.hypot(pos[0], pos[1]) == pytest.approx(P.CENTER_TO_MOTOR_M)
    assert P.CENTER_TO_MOTOR_M == pytest.approx(P.ARM_LENGTH_M * math.sqrt(2.0))


def _spin_to_hover(plant, hover):
    """Start at alt=1 with motors already at steady-state hover speed."""
    plant.reset(alt=1.0)
    plant.x[13:17] = P.pwm_to_omega(hover, P.BATTERY["vbat_nominal"])


def test_t3_12_hover_equilibrium_default_soc():
    """D9: hover duty on the default pack is a true equilibrium.

    Open-loop from rest cannot hover (the craft falls during motor spool-up), so
    the plant starts at the steady-state hover speed. The ground clamp is not what
    makes this pass: alt starts at 1 m.
    """
    hover = P.hover_duty_nominal()

    plant = make_plant("rk4", clean=True, seed=0)
    _spin_to_hover(plant, hover)
    b = Battery(P.BATTERY)
    for _ in range(round(0.5 / P.DT)):
        v = b.step(P.DT, 4.0 * hover)
        plant.step(P.DT, [hover] * 4, v)
    assert abs(plant.pos[2] - 1.0) < 0.05
    assert abs(plant.vel[2]) < 0.05

    # A full pack at the same duty and altitude climbs.
    plant2 = make_plant("rk4", clean=True, seed=0)
    _spin_to_hover(plant2, hover)
    b2 = Battery(P.BATTERY, initial_charge=1.0)
    for _ in range(round(0.5 / P.DT)):
        v = b2.step(P.DT, 4.0 * hover)
        plant2.step(P.DT, [hover] * 4, v)
    assert plant2.pos[2] > 1.05


def test_t3_10_flat_accel_and_signs(mixer):
    plant = make_plant("rk4", clean=True)
    _gyro, accel = plant.sense()
    assert accel == pytest.approx([0.0, 0.0, -P.GRAVITY], abs=1e-6)

    vbat = P.BATTERY["vbat_nominal"]

    def wd_frd(duty):
        plant.x[13:17] = P.pwm_to_omega(duty, vbat)  # steady-state motors
        wd = plant.rate_deriv(duty, vbat)
        return [wd[0], -wd[1], -wd[2]]

    out, _ = mixer(0.5, 0.2, 0.0, 0.0)  # roll+
    assert wd_frd(out)[0] > 0.0

    out, _ = mixer(0.5, 0.0, 0.2, 0.0)  # pitch+ => nose down => q_frd < 0
    assert wd_frd(out)[1] < 0.0

    out, _ = mixer(0.5, 0.0, 0.0, 0.2)  # yaw+ => r_frd > 0
    assert wd_frd(out)[2] > 0.0
