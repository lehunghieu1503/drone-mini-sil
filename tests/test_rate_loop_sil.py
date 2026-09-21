"""T5.7–T5.11 — rate-loop step response in SIL across tau_m (slow)."""

import pathlib

import numpy as np
import pytest

from _bind import sil_runner_path
from plant import drone_mini_params as P
from plant import runner

MAX_RATE = 5.236  # 300 dps
CASES = [
    ("rate_step_roll", "gyro_x", 0.25 * MAX_RATE),
    ("rate_step_pitch", "gyro_y", -0.25 * MAX_RATE),  # nose-down positive
    ("rate_step_yaw", "gyro_z", 0.5 * MAX_RATE),
]


@pytest.fixture(scope="session", autouse=True)
def _ensure_sim():
    sil_runner_path()


def _run(tmp_path, scenario, name):
    log = pathlib.Path(tmp_path) / name
    # Clean IMU isolates the control law; noise rejection is T5.11.
    argv = ["--mode", "rate", "--scenario", scenario, "--clean-imu",
            "--t-end", "3.0", "--log", str(log), "--seed", "1"]
    code = runner.run(runner.build_parser().parse_args(argv))
    return code, log


def load(path):
    header = None
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                continue
            p = line.strip().split(",")
            if header is None:
                header = p
            else:
                rows.append([float(x) for x in p])
    return header, np.array(rows)


def _settle_time(t, y, target, tol):
    """First time after the 1 s step where |y-target| stays within tol."""
    start = int(1.0 / P.DT)
    for k in range(start, len(y)):
        if np.all(np.abs(y[k:] - target) <= tol):
            return t[k] - 1.0
    return float("inf")


@pytest.mark.slow
@pytest.mark.parametrize("tau", [0.02, 0.03, 0.05])
@pytest.mark.parametrize("scenario,axis,target", CASES)
def test_rate_step_response(tmp_path, tau, scenario, axis, target):
    old = P.TAU_M_S
    P.TAU_M_S = tau
    try:
        code, log = _run(tmp_path, scenario, f"{scenario}_{tau}.csv")
    finally:
        P.TAU_M_S = old
    assert code == 0
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    t = data[:, i["t_us"]] * 1e-6
    y = data[:, i[axis]]
    tol = 0.02 * abs(target) + (0.5 * np.pi / 180.0)
    settle = _settle_time(t, y, target, tol)
    assert settle < 0.3, f"settle={settle:.3f}s tau={tau} {scenario}"
    # overshoot before settling
    after = y[int(1.2 / P.DT):]
    overshoot = (np.max(np.sign(target) * after) - abs(target)) / abs(target)
    assert overshoot < 0.20, f"overshoot={overshoot:.3f} {scenario} tau={tau}"


@pytest.mark.slow
def test_t5_10_saturation_no_windup(tmp_path):
    code, log = _run(tmp_path, "saturation", "sat.csv")
    assert code == 0
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    sat = data[:, i["sat_shift"]]
    assert np.max(np.abs(sat)) > 0.0  # mixer saturated at some point
    # no runaway / no NaN
    assert np.all(np.isfinite(data))
    assert np.max(np.abs(data[:, i["gyro_x"]])) < 2.0 * MAX_RATE


@pytest.mark.slow
def test_t5_11_noise_no_rumble(tmp_path):
    log = pathlib.Path(tmp_path) / "noise.csv"
    argv = ["--mode", "rate", "--scenario", "rate_hover",
            "--t-end", "3.0", "--log", str(log), "--seed", "7"]
    assert runner.run(runner.build_parser().parse_args(argv)) == 0
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    gyro = data[int(1.5 / P.DT):, i["gyro_x"]]
    assert np.std(gyro) < (30.0 * np.pi / 180.0)  # no sustained rumble
