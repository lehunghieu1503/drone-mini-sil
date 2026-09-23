"""CSV log writer + metadata preamble + loader/analysis helpers.

Schema v2 = the 22 columns of SIL_STACK §12 (frozen order) + `sat_shift`
appended last. Append-only rule: new columns go to the end and bump `schema`.
Owner: P3 (writer); P8 extends with loader/align/jitter/residual.
"""

from __future__ import annotations

import numpy as np

SCHEMA_VERSION = 2

CSV_HEADER = [
    "t_us", "roll_cmd", "pitch_cmd", "yaw_cmd", "thr_cmd", "armed", "failsafe",
    "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z",
    "roll_est", "pitch_est", "yaw_est",
    "mot0", "mot1", "mot2", "mot3",
    "vbat", "led_mode", "sat_shift",
]
assert len(CSV_HEADER) == 23

META_KEYS = (
    "kind", "schema", "seed", "dt_us", "plant", "determinism", "versions",
    "params_hash", "aborted", "decimated", "reset_reason", "vbat_nan",
)


def write_meta(f, **kv) -> None:
    for key, value in kv.items():
        f.write(f"# {key}={value}\n")


class CsvWriter:
    def __init__(self, path, meta=None, buffering: int = 1 << 16):
        self.path = path
        self.f = open(path, "w", buffering=buffering, newline="\n")
        self.f.write(f"# schema={SCHEMA_VERSION}\n")
        if meta:
            write_meta(self.f, **meta)
        self.f.write(",".join(CSV_HEADER) + "\n")
        self._closed = False

    def write_row(self, **cols) -> None:
        self.f.write(",".join(str(cols.get(name, 0)) for name in CSV_HEADER) + "\n")

    def close(self, aborted: bool = False) -> None:
        if self._closed:
            return
        self.f.write(f"# aborted={1 if aborted else 0}\n")
        self.f.close()
        self._closed = True


class Log:
    """Column-oriented log loaded from disk."""

    def __init__(self, meta, aborted, header, columns):
        self.meta = meta
        self.aborted = aborted
        self.header = header
        self.columns = columns

    def __getitem__(self, name):
        return self.columns[name]

    @property
    def n(self):
        return len(self.columns[self.header[0]])

    def has(self, name):
        return name in self.columns


def load_log(path) -> Log:
    meta = {}
    aborted = False
    header = None
    rows = []
    with open(path) as f:
        for line in f:
            if line.startswith("#"):
                body = line[1:].strip()
                if "=" in body:
                    k, v = body.split("=", 1)
                    if k == "aborted":
                        aborted = v.strip() == "1"
                    else:
                        meta[k.strip()] = v.strip()
                continue
            parts = line.strip().split(",")
            if header is None:
                header = parts
            elif parts and parts[0] != "":
                rows.append(parts)
    data = np.array(rows, dtype=float) if rows else np.zeros((0, len(header)))
    columns = {name: data[:, i] for i, name in enumerate(header)}
    return Log(meta, aborted, header, columns)


def align_time(t_us, armed=None, mode: str = "arm"):
    """Return t_us relative to an epoch. mode: start|arm|t0."""
    t_us = np.asarray(t_us, dtype=float)
    if mode in ("start", "t0") or armed is None:
        return t_us - t_us[0]
    idx = np.argmax(np.asarray(armed) >= 0.5)
    if armed is None or not np.any(np.asarray(armed) >= 0.5):
        return t_us - t_us[0]
    return t_us - t_us[idx]


def jitter_stats(t_us):
    t_us = np.asarray(t_us, dtype=float)
    if len(t_us) < 3:
        return {"min": 0.0, "median": 0.0, "p99": 0.0, "max": 0.0, "count_gt_1p5x": 0}
    dt = np.diff(t_us)
    med = float(np.median(dt))
    return {
        "min": float(dt.min()),
        "median": med,
        "p99": float(np.percentile(dt, 99)),
        "max": float(dt.max()),
        "count_gt_1p5x": int(np.sum(dt > 1.5 * med)) if med > 0 else 0,
    }


def residual(a, b, max_gap=None, t=None):
    """Residual between two aligned signals a, b (same length).

    `max_gap` is a *time* span in the same units as `t` (us). A sample whose
    neighbour in `t` is farther than `max_gap` is a logging gap, not a control
    event, so it is excluded from max/rms/corr. This keeps a signal spike that
    happens to sit on a gap from being counted as a tracking error (T8.9).
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    n = min(len(a), len(b))
    a, b = a[:n], b[:n]
    if max_gap is not None and t is None:
        raise ValueError("residual: max_gap is a time span and needs t")
    diff = a - b
    keep = np.ones(n, dtype=bool)
    if t is not None and max_gap is not None and n > 1:
        tt = np.asarray(t, dtype=float)
        if tt.shape[0] < n:
            raise ValueError("residual: t shorter than the signals")
        tt = tt[:n]
        gap = np.abs(np.diff(tt)) > max_gap
        isolated = np.zeros(n, dtype=bool)
        isolated[1:] |= gap
        isolated[:-1] |= gap
        keep = ~isolated
    n_rejected = int(np.sum(~keep))
    kept = diff[keep]
    max_abs = float(np.max(np.abs(kept))) if kept.size else 0.0
    rms = float(np.sqrt(np.mean(kept * kept))) if kept.size else 0.0
    ak, bk = a[keep], b[keep]
    corr = (float(np.corrcoef(ak, bk)[0, 1])
            if ak.size > 2 and ak.std() > 0 and bk.std() > 0 else 1.0)
    return {
        "max_abs": max_abs,
        "rms": rms,
        "corr": corr,
        "n_rejected": n_rejected,
        "n": n,
    }
