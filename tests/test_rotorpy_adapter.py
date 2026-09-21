"""P4 — RotorPy adapter smoke test (conditional; RK4 is authoritative)."""

import numpy as np
import pytest

pytest.importorskip("rotorpy")

from plant.vehicle import make_plant  # noqa: E402


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
