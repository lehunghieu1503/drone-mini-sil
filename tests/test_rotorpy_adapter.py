"""P4 — RotorPy adapter smoke test (conditional; RK4 is authoritative)."""

import numpy as np
import pytest

pytest.importorskip("rotorpy")

from plant.vehicle import make_plant, to_frd  # noqa: E402


def test_rotorpy_interface_smoke():
    plant = make_plant("rotorpy", clean=True, seed=0)
    plant.reset(alt=1.0)
    for _ in range(200):
        plant.step(0.001, [0.6, 0.6, 0.6, 0.6], 3.85)
    assert plant.finite()
    gyro, accel = plant.sense()
    assert np.all(np.isfinite(gyro)) and np.all(np.isfinite(accel))
    assert len(plant.motors) == 4
    assert plant.pos.shape == (3,)
    assert plant.quat.shape == (4,)


def _rate_frd(name, duty, t=0.2):
    plant = make_plant(name, clean=True, seed=0)
    plant.reset(alt=50.0)
    for _ in range(int(t / 0.001)):
        plant.step(0.001, duty, 3.85)
    return to_frd(plant.omega)


@pytest.mark.parametrize("axis,duty,idx", [
    ("roll", [0.3, 0.3, 0.7, 0.7], 0),
    ("pitch", [0.7, 0.3, 0.7, 0.3], 1),
    ("yaw", [0.3, 0.7, 0.7, 0.3], 2),
])
def test_rotorpy_axis_sign_matches_rk4(axis, duty, idx):
    """D11: the RotorPy duty permutation keeps every axis sign aligned with RK4."""
    rk4 = _rate_frd("rk4", duty)
    rpy = _rate_frd("rotorpy", duty)
    assert abs(rk4[idx]) > 0.0
    assert np.sign(rk4[idx]) == np.sign(rpy[idx]), f"{axis}: {rk4} vs {rpy}"
