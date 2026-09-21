"""Phase 3 open-loop SIL (T3.3/T3.4/T3.16/T3.18/T3.22). Slow end-to-end."""

import pathlib

import pytest

from _bind import sil_runner_path
from plant import runner


@pytest.fixture(scope="session", autouse=True)
def _ensure_sim():
    sil_runner_path()  # builds build/sim/sil_runner if needed


def run_sil(tmp_path, extra=(), t_end=1.0, seed=1, name="sil.csv", metrics=None):
    log = pathlib.Path(tmp_path) / name
    argv = ["--t-end", str(t_end), "--log", str(log), "--seed", str(seed), *extra]
    code = runner.run(runner.build_parser().parse_args(argv), metrics=metrics)
    return code, log


def load_rows(path):
    header = None
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                continue
            parts = line.strip().split(",")
            if header is None:
                header = parts
            else:
                rows.append(parts)
    return header, rows


def test_t3_16_no_arm_test_zero(tmp_path):
    metrics = {}
    code, log = run_sil(tmp_path, ["--ol-thr", "0.5"], t_end=0.5, metrics=metrics)
    assert code == 0
    header, rows = load_rows(log)
    idx = [header.index(f"mot{i}") for i in range(4)]
    assert all(float(r[i]) == 0.0 for r in rows for i in idx)
    assert metrics["motors"] == pytest.approx([0.0, 0.0, 0.0, 0.0])


@pytest.mark.slow
def test_t3_3_climb_with_arm_test(tmp_path):
    metrics = {}
    code, log = run_sil(tmp_path, ["--arm-test", "--ol-thr", "0.52"], t_end=2.0, metrics=metrics)
    assert code == 0
    assert metrics["alt"] > 0.1
    header, rows = load_rows(log)
    # no yaw spin: |yaw_est| stays small
    max_yaw = max(abs(float(r[header.index("yaw_est")])) for r in rows)
    assert max_yaw < 0.2


@pytest.mark.slow
def test_t3_4_cut_after_hover(tmp_path):
    metrics = {}
    # rc-driven open loop: throttle 0.5 then 0 after 3 s (scenario "cut")
    code, log = run_sil(
        tmp_path,
        ["--arm-test", "--ol-from-rc", "--scenario", "cut"],
        t_end=4.0, metrics=metrics,
    )
    assert code == 0
    assert max(metrics["motors"]) < 0.05


@pytest.mark.slow
def test_t3_18_determinism(tmp_path):
    m1, m2 = {}, {}
    c1, l1 = run_sil(tmp_path, ["--arm-test", "--ol-from-rc"], t_end=1.0, seed=42,
                     name="a.csv", metrics=m1)
    c2, l2 = run_sil(tmp_path, ["--arm-test", "--ol-from-rc"], t_end=1.0, seed=42,
                     name="b.csv", metrics=m2)
    assert c1 == 0 and c2 == 0
    assert l1.read_bytes() == l2.read_bytes()
    assert m1["alt"] == pytest.approx(m2["alt"])


@pytest.mark.slow
def test_t3_22_open_loop_10s(tmp_path):
    metrics = {}
    code, log = run_sil(tmp_path, ["--arm-test", "--ol-thr", "0.52"], t_end=10.0,
                        metrics=metrics)
    assert code == 0
    header, rows = load_rows(log)
    assert len(header) == 23
    assert len(rows) == 10000
    assert metrics["motors"][0] > 0.0
