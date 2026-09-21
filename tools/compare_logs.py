"""Overlay two logs and run an automatic pass/fail gate.

Exit codes: 0 pass, 1 gate fail (residual above tolerance), 2 schema mismatch.

Usage: python tools/compare_logs.py A.csv B.csv [--align arm] [--tol 0.15]
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np

from plant.log import align_time, jitter_stats, load_log, residual

# Signals compared by default (must exist in both logs).
COMPARE_SIGNALS = ["roll_est", "pitch_est", "yaw_est", "gyro_x", "gyro_y", "gyro_z"]


def _resample(t_new, t_old, y_old):
    order = np.argsort(t_old)
    return np.interp(t_new, t_old[order], y_old[order])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="compare_logs")
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--align", default="arm", choices=["start", "arm", "t0"])
    ap.add_argument("--tol", type=float, default=0.15)
    ap.add_argument("--max-gap", type=float, default=0.0)
    ap.add_argument("--csv-out", default=None)
    args = ap.parse_args(argv)

    la = load_log(args.a)
    lb = load_log(args.b)
    if la.header != lb.header:
        print("schema mismatch: headers differ", file=sys.stderr)
        return 2

    ta = align_time(la["t_us"], la["armed"] if la.has("armed") else None, args.align)
    tb = align_time(lb["t_us"], lb["armed"] if lb.has("armed") else None, args.align)

    dt_med = jitter_stats(la["t_us"])["median"]
    max_gap = args.max_gap if args.max_gap > 0 else 5.0 * dt_med

    print(f"# A={pathlib.Path(args.a).name} kind={la.meta.get('kind', '?')} "
          f"aborted={int(la.aborted)}")
    print(f"# B={pathlib.Path(args.b).name} kind={lb.meta.get('kind', '?')} "
          f"aborted={int(lb.aborted)}")
    if la.aborted or lb.aborted:
        print("# WARNING: an aborted run is being compared")

    print(f"# jitter A: {jitter_stats(la['t_us'])}")
    print(f"# jitter B: {jitter_stats(lb['t_us'])}")

    gate_fail = False
    rows = ["signal,max_abs,rms,corr,n_rejected"]
    for name in COMPARE_SIGNALS:
        if not (la.has(name) and lb.has(name)):
            continue
        b_on_a = _resample(ta, tb, lb[name])
        r = residual(la[name], b_on_a, max_gap=max_gap)
        rows.append(f"{name},{r['max_abs']:.6g},{r['rms']:.6g},{r['corr']:.4f},"
                    f"{r['n_rejected']}")
        if r["max_abs"] > args.tol:
            gate_fail = True

    print("signal,max_abs,rms,corr,n_rejected")
    for row in rows[1:]:
        print(row)

    if args.csv_out:
        pathlib.Path(args.csv_out).write_text("\n".join(rows) + "\n")

    if gate_fail:
        print(f"GATE FAIL: residual above tol={args.tol}")
        return 1
    print("GATE PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
