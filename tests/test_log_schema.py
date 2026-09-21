"""T8.1/T8.2/T8.3/T8.11 — frozen log schema + metadata + abort marker."""

import numpy as np
import pytest

from plant.log import CSV_HEADER, META_KEYS, CsvWriter, load_log


def test_t8_1_header_frozen():
    assert tuple(CSV_HEADER) == (
        "t_us", "roll_cmd", "pitch_cmd", "yaw_cmd", "thr_cmd", "armed", "failsafe",
        "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z",
        "roll_est", "pitch_est", "yaw_est",
        "mot0", "mot1", "mot2", "mot3",
        "vbat", "led_mode", "sat_shift",
    )


def _write(tmp_path, n=1000, meta=None, aborted=False):
    path = tmp_path / "log.csv"
    w = CsvWriter(path, meta=meta or {"kind": "SIL", "reset_reason": 9, "vbat_nan": 1})
    for i in range(n):
        w.write_row(t_us=i * 1000, sat_shift=-0.02, roll_est=0.001 * i)
    w.close(aborted=aborted)
    return path


def test_t8_2_meta_keys(tmp_path):
    path = _write(tmp_path)
    log = load_log(path)
    for key in ("kind", "reset_reason", "vbat_nan"):
        assert key in log.meta
    assert log.meta["reset_reason"] == "9"
    assert log.meta["vbat_nan"] == "1"


def test_t8_3_writer_roundtrip(tmp_path):
    path = _write(tmp_path, n=1000)
    log = load_log(path)
    assert log.n == 1000
    assert log["sat_shift"].dtype == np.float64
    assert log["sat_shift"][10] == pytest.approx(-0.02)
    assert log["roll_est"][999] == pytest.approx(0.999)


def test_t8_11_aborted_marker(tmp_path):
    path = _write(tmp_path, n=10, aborted=True)
    log = load_log(path)
    assert log.aborted is True
