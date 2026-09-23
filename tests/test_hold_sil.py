"""External pilot closes altitude. Firmware stays in attitude mode."""

import pathlib

import pytest

from _bind import sil_runner_path
from plant import runner
from plant.log import load_log


@pytest.fixture(scope="session", autouse=True)
def _ensure_sim():
    sil_runner_path()


@pytest.mark.slow
def test_hold_alt_settles(tmp_path):
    log = pathlib.Path(tmp_path) / "hold.csv"
    args = runner.build_parser().parse_args([
        "--hold-alt", "1.0", "--t-end", "8", "--seed", "1", "--log", str(log),
    ])
    metrics = {}
    assert runner.run(args, metrics) == 0
    assert metrics["alt"] == pytest.approx(1.0, abs=0.12)
    assert abs(metrics["vz"]) < 0.25
    assert abs(metrics["x"]) < 0.2
    assert abs(metrics["y"]) < 0.2
    assert int(load_log(log).columns["armed"][-1]) == 1
