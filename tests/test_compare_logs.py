"""T8.4–T8.8 — alignment, gap rejection, jitter, gate, schema mismatch."""

import numpy as np
import pytest

from plant.log import CsvWriter, align_time, jitter_stats, load_log, residual
from tools.compare_logs import main as compare_main


def _write(path, n=500, spike=None, gap_at=None, arm_after=50):
    w = CsvWriter(path, meta={"kind": "SIL"})
    t = 0
    for i in range(n):
        if gap_at is not None and i == gap_at:
            t += 50_000
        row = dict(t_us=t, gyro_x=0.01, gyro_y=0.0, gyro_z=0.0,
                   roll_est=0.001 * (i / n), pitch_est=0.0, yaw_est=0.0,
                   armed=1 if i > arm_after else 0)
        if spike is not None and i == 300:
            row["roll_est"] += spike
        w.write_row(**row)
        t += 1000
    w.close()
    return path


def test_gate_pass_identical(tmp_path):
    a = _write(tmp_path / "a.csv")
    b = _write(tmp_path / "b.csv")
    assert compare_main([str(a), str(b)]) == 0


def test_gate_fail_spike(tmp_path):
    a = _write(tmp_path / "a.csv")
    b = _write(tmp_path / "b.csv", spike=1.0)
    assert compare_main([str(a), str(b)]) == 1


def test_schema_mismatch(tmp_path):
    a = _write(tmp_path / "a.csv")
    bad = tmp_path / "bad.csv"
    bad.write_text("# schema=2\nt_us,mot0\n0,0.1\n")
    assert compare_main([str(a), str(bad)]) == 2


def test_t8_4_align_arm(tmp_path):
    path = _write(tmp_path / "a.csv", arm_after=100)
    log = load_log(path)
    aligned = align_time(log["t_us"], log["armed"], "arm")
    assert aligned[0] == pytest.approx(-(101 * 1000), abs=1.0)
    start = align_time(log["t_us"], log["armed"], "start")
    assert start[0] == 0.0


def test_t8_5_gap_rejected(tmp_path):
    path = _write(tmp_path / "a.csv", gap_at=200)
    log = load_log(path)
    t = log["t_us"]
    dt = np.diff(t)
    med = np.median(dt)
    r = residual(t[1:], t[:-1], max_gap=5 * med)
    assert r["n_rejected"] >= 1


def test_t8_8_jitter_stats(tmp_path):
    path = _write(tmp_path / "a.csv")
    log = load_log(path)
    j = jitter_stats(log["t_us"])
    assert j["median"] == pytest.approx(1000.0)
    assert j["max"] >= j["median"]
    assert "count_gt_1p5x" in j
