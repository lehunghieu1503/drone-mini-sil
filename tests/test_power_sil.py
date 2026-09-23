"""T7.14/T7.15 — brownout boot latch and RC-loss failsafe (slow)."""

import pathlib

import numpy as np
import pytest

from _bind import sil_runner_path
from plant import runner


@pytest.fixture(scope="session", autouse=True)
def _ensure_sim():
    sil_runner_path()


def _run(tmp_path, scenario, name, t_end, extra=()):
    log = pathlib.Path(tmp_path) / name
    argv = ["--mode", "attitude", "--scenario", scenario, "--t-end", str(t_end),
            "--log", str(log), "--seed", "1", *extra]
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


@pytest.mark.slow
def test_t7_15_rc_loss_failsafe(tmp_path):
    code, log = _run(tmp_path, "rc_loss", "loss.csv", 5.0)
    assert code == 0
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    # Last frame is tick 2999 (t=2.999 s); the 100 ms timeout latches at tick 3099.
    at_305 = data[round(3.05 * 1000)]
    assert at_305[i["armed"]] == 1
    assert at_305[i["failsafe"]] == 0
    assert max(at_305[i[f"mot{j}"]] for j in range(4)) > 0.0
    at_311 = data[round(3.11 * 1000)]
    assert at_311[i["failsafe"]] == 1
    assert all(at_311[i[f"mot{j}"]] == 0.0 for j in range(4))
    assert np.all(np.isfinite(data))


@pytest.mark.slow
def test_t7_20_sbus_14ms_keeps_armed(tmp_path):
    """D1: one frame every 14 ms still arms and holds motors (real SBUS cadence)."""
    code, log = _run(tmp_path, "sbus_14ms", "sbus14.csv", 2.0)
    assert code == 0
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    late = data[round(0.5 * 1000):]
    assert late[:, i["armed"]].min() == 1
    duty_sum = sum(late[:, i[f"mot{j}"]] for j in range(4))
    assert duty_sum.max() > 0.5


@pytest.mark.slow
def test_t7_21_sbus_14ms_dead_before_calib_never_arms(tmp_path):
    """A held switch must not arm after the link dies mid-calibration."""
    code, log = _run(tmp_path, "sbus_14ms_dead", "sbusdead.csv", 1.0)
    assert code == 0
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    assert data[:, i["armed"]].max() == 0.0
    assert all(np.abs(data[:, i[f"mot{j}"]]).max() == 0.0 for j in range(4))


@pytest.mark.slow
def test_t7_22_boot_latch_blocks_arm_test(tmp_path):
    """D3: --arm-test must not clear or bypass a brownout boot latch."""
    log = pathlib.Path(tmp_path) / "bootarm.csv"
    argv = ["--mode", "open-loop", "--scenario", "att_hover", "--arm-test",
            "--reset-reason", "9", "--ol-thr", "0.52", "--t-end", "1.0",
            "--log", str(log), "--seed", "1"]
    code = runner.run(runner.build_parser().parse_args(argv))
    assert code == 0
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    assert data[:, i["armed"]].max() == 0.0
    assert all(np.abs(data[:, i[f"mot{j}"]]).max() == 0.0 for j in range(4))


@pytest.mark.slow
def test_t5_13_rearm_resets_rate_integrator(tmp_path):
    """D7: the arm edge resets the rate integrator inside the real SIL tick."""
    code, log = _run(tmp_path, "rearm", "rearm.csv", 2.0)
    assert code == 0
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    armed = data[:, i["armed"]]
    rearm = next((k for k in range(round(1.3 * 1000), len(armed)) if armed[k] == 1), None)
    assert rearm is not None, "never re-armed"
    window = slice(rearm, rearm + round(0.2 * 1000))
    gyro = np.degrees(data[window, i["gyro_x"]])
    assert np.abs(gyro).max() < 50.0
    assert np.abs(data[window, i["sat_shift"]]).max() < 0.06


@pytest.mark.slow
def test_t5_14_attitude_yaw_stick_commands_rate(tmp_path):
    """D8: attitude-mode yaw must go through stick_yaw_to_rate, not att_sp[2]=0."""
    code, log = _run(tmp_path, "rate_step_yaw", "attyaw.csv", 2.0)
    assert code == 0
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    t = data[:, i["t_us"]] * 1e-6
    gyro_z = data[:, i["gyro_z"]]
    assert np.abs(gyro_z[t < 0.9]).max() < 0.1
    assert gyro_z[t >= 1.5].mean() > 0.5  # 0.5 * 5.236 ~ 2.6 rad/s, same sign


@pytest.mark.slow
def test_t7_14_brownout_boot_latch(tmp_path):
    code, log = _run(tmp_path, "att_hover", "brownout.csv", 3.0,
                    extra=("--reset-reason", "9"))
    assert code == 0
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    assert data[:, i["armed"]].max() == 0.0  # forced disarmed
    assert data[:, i["led_mode"]].max() == 3.0  # LED ERROR
    assert all(np.abs(data[:, i[f"mot{j}"]]).max() == 0.0 for j in range(4))


def test_t7_19_single_sbus_parser():
    root = sil_runner_path().parents[2]
    offenders = []
    for path in (root / "firmware" / "main" / "flight").glob("*.cpp"):
        if path.name == "rc_parse.cpp":
            continue
        if "0x0F" in path.read_text() or "0x7FF" in path.read_text():
            offenders.append(path.name)
    assert offenders == []
