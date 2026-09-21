"""T6.7–T6.13 — attitude-mode hover with noise/bias and sweep (slow)."""

import pathlib

import numpy as np
import pytest

from _bind import sil_runner_path
from plant import runner


@pytest.fixture(scope="session", autouse=True)
def _ensure_sim():
    sil_runner_path()


def _run(tmp_path, scenario, name, t_end=10.0, seed=1, extra=()):
    log = pathlib.Path(tmp_path) / name
    argv = ["--mode", "attitude", "--scenario", scenario, "--t-end", str(t_end),
            "--log", str(log), "--seed", str(seed), *extra]
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


def _hover_metrics(log):
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    start = int(2.0 / 0.001)
    roll = np.degrees(data[:, i["roll_est"]])
    pitch = np.degrees(data[:, i["pitch_est"]])
    yaw = np.degrees(data[:, i["yaw_est"]])
    return {
        "roll_max": np.abs(roll[start:]).max(),
        "pitch_max": np.abs(pitch[start:]).max(),
        "yaw_drift": abs(yaw[-1] - yaw[start]),
        "armed": data[:, i["armed"]].sum(),
        "finite": np.all(np.isfinite(data)),
    }


@pytest.mark.slow
@pytest.mark.parametrize("seed", [1, 2])
def test_t6_7_t6_8_hover_two_seeds(tmp_path, seed):
    code, log = _run(tmp_path, "att_hover", f"hover_{seed}.csv", seed=seed)
    assert code == 0
    m = _hover_metrics(log)
    assert m["armed"] > 9000
    assert m["roll_max"] < 5.0
    assert m["pitch_max"] < 5.0
    assert m["yaw_drift"] < 15.0
    assert m["finite"]


@pytest.mark.slow
def test_t6_9_hover_clean(tmp_path):
    code, log = _run(tmp_path, "att_hover", "clean.csv", extra=("--clean-imu",))
    assert code == 0
    m = _hover_metrics(log)
    assert m["roll_max"] < 5.0 and m["pitch_max"] < 5.0 and m["yaw_drift"] < 15.0


@pytest.mark.slow
def test_t6_10_stick_step_returns(tmp_path):
    code, log = _run(tmp_path, "stick_step", "step.csv", t_end=6.0)
    assert code == 0
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    roll = np.degrees(data[:, i["roll_est"]])
    tail = roll[int(4.5 / 0.001):]  # >= 2 s after the step ends
    assert np.abs(tail).max() < 2.0


@pytest.mark.slow
def test_t6_12_hover_sweep(tmp_path):
    code, log = _run(tmp_path, "hover_sweep", "sweep.csv", t_end=8.0)
    assert code == 0
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    roll = np.degrees(data[:, i["roll_est"]])
    pitch = np.degrees(data[:, i["pitch_est"]])
    assert np.abs(roll).max() < 15.0 and np.abs(pitch).max() < 15.0  # no flip
    assert np.all(np.isfinite(data))


@pytest.mark.slow
def test_t6_13_imu_invalid_output_zero(tmp_path):
    code, log = _run(tmp_path, "att_hover", "imu_drop.csv", t_end=4.0,
                     extra=("--imu-invalid-after", "2.0"))
    assert code == 0
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    # after the 5-tick debounce, outputs are forced to zero
    late = data[int(2.1 / 0.001):]
    assert all(np.abs(late[:, i[f"mot{j}"]]).max() == 0.0 for j in range(4))
    assert np.all(np.isfinite(data))
