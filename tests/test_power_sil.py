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
    # failsafe must latch within 100 ms (+1 tick) of the last frame at ~3.0 s
    late = data[int(3.15 / 0.001):]
    assert late[:, i["failsafe"]].min() == 1
    assert all(np.abs(late[:, i[f"mot{j}"]]).max() == 0.0 for j in range(4))
    assert np.all(np.isfinite(data))


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
