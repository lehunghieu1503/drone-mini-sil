"""Plot one SIL/HIL/FLIGHT log to PNG (headless by default).

Usage: python tools/plot_sil.py LOG [--out PNG] [--decimate N] [--show]
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from plant.log import load_log  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="plot_sil")
    ap.add_argument("log")
    ap.add_argument("--out", default="logs/plot.png")
    ap.add_argument("--decimate", type=int, default=1)
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args(argv)

    log = load_log(args.log)
    step = max(1, args.decimate)
    t = log["t_us"][::step] * 1e-6
    deg = np.degrees
    fig, axes = plt.subplots(5, 1, figsize=(11, 12), sharex=True)

    axes[0].plot(t, deg(log["gyro_x"][::step]), label="gx")
    axes[0].plot(t, deg(log["gyro_y"][::step]), label="gy")
    axes[0].plot(t, deg(log["gyro_z"][::step]), label="gz")
    axes[0].set_ylabel("gyro (deg/s)")
    axes[0].legend(loc="upper right")

    axes[1].plot(t, deg(log["roll_est"][::step]), label="roll")
    axes[1].plot(t, deg(log["pitch_est"][::step]), label="pitch")
    axes[1].plot(t, deg(log["yaw_est"][::step]), label="yaw")
    axes[1].set_ylabel("attitude (deg)")
    axes[1].legend(loc="upper right")

    for j in range(4):
        axes[2].plot(t, log[f"mot{j}"][::step], label=f"mot{j}")
    axes[2].set_ylabel("duty")
    axes[2].legend(loc="upper right")

    axes[3].plot(t, log["vbat"][::step])
    axes[3].set_ylabel("vbat (V)")

    axes[4].step(t, log["led_mode"][::step], where="post", label="led")
    axes[4].step(t, log["armed"][::step], where="post", label="armed")
    axes[4].step(t, log["failsafe"][::step], where="post", label="failsafe")
    axes[4].set_ylabel("state")
    axes[4].set_xlabel("t (s)")
    axes[4].legend(loc="upper right")

    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")
    if args.show:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
