"""V3 — sim-to-sim parity: RK4 (authoritative) vs MuJoCo (opt-in engine swap).

RotorPy is intentionally excluded from parity assertions: its rotor geometry and
propulsion constants are its own, so a duty differential maps to different axes
and magnitudes. It stays a reference-only smoke test (tests/test_rotorpy_adapter.py)
and can be included in the manual comparison via tools/sim_to_sim.py.
"""

import numpy as np
import pytest

from plant import drone_mini_params as P
from plant.vehicle import make_plant, to_frd

VB = P.BATTERY["vbat_nominal"]
TOL_RATE_REL = 0.25  # MuJoCo adds aero drag; ~11% measured difference


def _make(name):
    if name == "mujoco":
        pytest.importorskip("mujoco")
    return make_plant(name, clean=True, seed=0)


def _hover_accel(name):
    """Vertical acceleration at nominal hover duty after motor spin-up."""
    plant = _make(name)
    plant.reset(alt=50.0)
    duty = P.hover_duty_nominal()
    for _ in range(int(1.0 / P.DT)):
        plant.step(P.DT, [duty] * 4, VB)
    v1 = float(plant.vel[2])
    for _ in range(int(0.1 / P.DT)):
        plant.step(P.DT, [duty] * 4, VB)
    return (float(plant.vel[2]) - v1) / 0.1


def _rate_frd(name, duty):
    """Body angular velocity in FRD after a 0.5 s duty step."""
    plant = _make(name)
    plant.reset(alt=50.0)
    for _ in range(int(0.5 / P.DT)):
        plant.step(P.DT, duty, VB)
    return to_frd(plant.omega)


@pytest.mark.parametrize("name", ["rk4", "mujoco"])
def test_sim2sim_thrust_balance_at_hover(name):
    assert abs(_hover_accel(name)) < 0.1


@pytest.mark.parametrize("name", ["rk4", "mujoco"])
def test_sim2sim_free_fall(name):
    plant = _make(name)
    plant.reset(alt=20.0)
    for _ in range(int(0.5 / P.DT)):
        plant.step(P.DT, [0, 0, 0, 0], VB)
    assert plant.vel[2] == pytest.approx(-P.GRAVITY * 0.5, rel=0.15)


def test_sim2sim_rate_signs_and_magnitude_match():
    t = P.hover_duty_nominal()
    # duty vectors follow the firmware mixer (single-axis differentials)
    cases = {
        "roll": ([t - 0.2, t - 0.2, t + 0.2, t + 0.2], 0, +1),
        "pitch": ([t + 0.2, t - 0.2, t + 0.2, t - 0.2], 1, -1),
        "yaw": ([t - 0.2, t + 0.2, t + 0.2, t - 0.2], 2, +1),
    }
    for axis, (duty, idx, sign) in cases.items():
        rk4 = _rate_frd("rk4", duty)
        mj = _rate_frd("mujoco", duty)
        assert np.sign(rk4[idx]) == sign, f"{axis}: RK4 sign"
        assert np.sign(mj[idx]) == sign, f"{axis}: MuJoCo sign"
        assert mj[idx] == pytest.approx(rk4[idx], rel=TOL_RATE_REL), f"{axis}: magnitude"
