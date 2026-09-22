"""V3 — MuJoCo (Menagerie Crazyflie 2) plant adapter. Skips if mujoco is absent."""

import numpy as np
import pytest

pytest.importorskip("mujoco")

from plant import drone_mini_params as P  # noqa: E402
from plant.vehicle import make_plant  # noqa: E402
from plant.vehicle_mujoco import MuJoCoPlant  # noqa: E402

VB = P.BATTERY["vbat_nominal"]


def test_mujoco_interface_smoke():
    plant = make_plant("mujoco", clean=True, seed=0)
    plant.reset(alt=1.0)
    for _ in range(200):
        plant.step(P.DT, [0.6, 0.6, 0.6, 0.6], VB)
    assert plant.finite()
    gyro, accel = plant.sense()
    assert np.all(np.isfinite(gyro)) and np.all(np.isfinite(accel))
    assert len(plant.motors) == 4
    assert plant.pos.shape == (3,)
    assert plant.quat.shape == (4,)
    assert plant.omega.shape == (3,)


def test_mujoco_rest_reads_gravity_in_frd():
    plant = make_plant("mujoco", clean=True, seed=0)
    plant.reset(alt=0.0)
    for _ in range(3000):
        plant.step(P.DT, [0, 0, 0, 0], VB)
    _gyro, accel = plant.sense()
    assert accel[2] == pytest.approx(-P.GRAVITY, abs=0.1)  # FRD z points down
    assert abs(accel[0]) < 0.1
    assert abs(accel[1]) < 0.1


def test_mujoco_free_fall_near_g():
    plant = make_plant("mujoco", clean=True, seed=0)
    plant.reset(alt=20.0)
    for _ in range(int(0.5 / P.DT)):
        plant.step(P.DT, [0, 0, 0, 0], VB)
    # MuJoCo models aero drag, so this is looser than the RK4 free-fall test.
    assert plant.vel[2] == pytest.approx(-P.GRAVITY * 0.5, rel=0.15)


def test_mujoco_params_override_defaults_to_drone_mini():
    plant = make_plant("mujoco", clean=True, seed=0)
    assert plant.m.body_mass[plant._bid] == pytest.approx(P.MASS_KG)
    assert plant.m.body_inertia[plant._bid] == pytest.approx(np.array(P.INERTIA))

    menagerie = MuJoCoPlant(seed=0, clean=True, params="menagerie")
    assert menagerie.m.body_mass[menagerie._bid] == pytest.approx(0.027)


def test_mujoco_deterministic_within_version():
    def run():
        plant = make_plant("mujoco", clean=True, seed=0)
        plant.reset(alt=5.0)
        for _ in range(500):
            plant.step(P.DT, [0.55, 0.55, 0.55, 0.55], VB)
        return np.array(plant.d.qpos, dtype=float)

    assert np.array_equal(run(), run())
