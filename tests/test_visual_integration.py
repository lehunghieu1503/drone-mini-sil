"""Phase 2 — runner visual integration. Headless: no GL, no viewer process."""

import pathlib

import pytest

from _bind import sil_runner_path
from plant import runner


@pytest.fixture(scope="session", autouse=True)
def _ensure_sim():
    sil_runner_path()


def _args(tmp_path, name, *extra):
    return runner.build_parser().parse_args(
        ["--mode", "open-loop", "--scenario", "hover", "--t-end", "0.3",
         "--arm-test", "--ol-thr", "0.55", "--seed", "1",
         "--log", str(pathlib.Path(tmp_path) / name), *extra])


def test_visual_does_not_change_log(tmp_path):
    plain = _args(tmp_path, "plain.csv")
    visual = _args(tmp_path, "visual.csv", "--visual", "--visual-port", "1")
    assert runner.run(plain) == 0
    assert runner.run(visual) == 0
    # byte-identical: the visual link must not touch the log at all
    assert pathlib.Path(plain.log).read_bytes() == pathlib.Path(visual.log).read_bytes()
    assert pathlib.Path(visual.log).read_text().rstrip().endswith("# aborted=0")


def test_visual_without_listener_is_harmless(tmp_path):
    args = _args(tmp_path, "no_listener.csv", "--visual", "--visual-port", "1")
    assert runner.run(args) == 0
    assert pathlib.Path(args.log).read_text().rstrip().endswith("# aborted=0")


def test_visual_off_by_default(tmp_path, monkeypatch):
    built = []
    monkeypatch.setattr(runner, "PoseSender", lambda *a, **k: built.append(1))
    assert runner.run(_args(tmp_path, "off.csv")) == 0
    assert built == []


def test_missing_plant_dependency_returns_exit_config(tmp_path, monkeypatch):
    def boom(*_a, **_k):
        raise ImportError("No module named 'mujoco'")

    monkeypatch.setattr(runner, "make_plant", boom)
    args = _args(tmp_path, "nodep.csv", "--plant", "mujoco")
    assert runner.run(args) == runner.EXIT_CONFIG == 5


def test_early_return_before_try_constructs_no_sender(tmp_path, monkeypatch):
    built = []
    monkeypatch.setattr(runner, "PoseSender", lambda *a, **k: built.append(1))
    args = _args(tmp_path, "bogus.csv", "--visual", "--scenario", "bogus")
    with pytest.raises(ValueError):
        runner.run(args)
    assert built == []


def test_pace_uses_bounded_slices_and_never_accumulates_debt():
    clock_state = {"t": 0.0}
    slept = []

    def clock():
        return clock_state["t"]

    def sleep(dt):
        slept.append(dt)
        clock_state["t"] += dt

    runner._pace(0.0, 1.0, 1.0, clock=clock, sleep=sleep)  # 1 virtual s at 1x
    assert clock_state["t"] == pytest.approx(1.0, abs=1e-9)
    assert slept and all(d <= 0.05 + 1e-9 for d in slept)

    slept.clear()
    clock_state["t"] = 5.0  # far behind: never try to catch up
    runner._pace(0.0, 1.0, 1.0, clock=clock, sleep=sleep)
    assert slept == []

    runner._pace(0.0, 1.0, 0.0, clock=clock, sleep=sleep)  # rate 0 disables
    assert slept == []


@pytest.mark.parametrize("flag,value", [
    ("--visual-rate", "-1"),
    ("--visual-rate", "nan"),
    ("--visual-hz", "0"),
    ("--visual-port", "0"),
    ("--visual-port", "70000"),
])
def test_visual_flag_validators(flag, value):
    with pytest.raises(SystemExit):
        runner.build_parser().parse_args([flag, value])
