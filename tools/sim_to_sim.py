"""Sim-to-sim comparison across plant engines (V3).

Drives every available plant with the SAME scripted duty profile
(spin-up -> hover -> roll pulse) and prints a summary table. Use it to
sanity-check a new engine against RK4 before trusting it in a SIL run.

RK4 is authoritative. MuJoCo is the opt-in high-fidelity engine
(`--plant mujoco`). RotorPy is reference-only: its rotor geometry and
propulsion constants differ, so rate signs/magnitudes will not match.

Usage:
    PYTHONPATH=. .venv/bin/python tools/sim_to_sim.py
    PYTHONPATH=. .venv/bin/python tools/sim_to_sim.py --plants rk4,mujoco,rotorpy
"""

from __future__ import annotations

import argparse
import sys

from plant import drone_mini_params as P
from plant.vehicle import make_plant, to_frd

VB = P.BATTERY["vbat_nominal"]


def _profile(plant, t_end=1.5):
    """Spin-up at hover (0-1 s) then a roll+ differential (1-1.5 s)."""
    hover = P.hover_duty_nominal()
    roll_duty = [hover - 0.2, hover - 0.2, hover + 0.2, hover + 0.2]
    plant.reset(alt=50.0)
    for _ in range(int(1.0 / P.DT)):
        plant.step(P.DT, [hover] * 4, VB)
    v1 = float(plant.vel[2])
    for _ in range(int(0.1 / P.DT)):
        plant.step(P.DT, [hover] * 4, VB)
    a_z = (float(plant.vel[2]) - v1) / 0.1
    for _ in range(int(0.4 / P.DT)):
        plant.step(P.DT, roll_duty, VB)
    return {
        "finite": plant.finite(),
        "a_z_hover": a_z,
        "roll_rate_frd": float(to_frd(plant.omega)[0]),
        "alt": float(plant.pos[2]),
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="sim_to_sim")
    ap.add_argument("--plants", default="rk4,mujoco,rotorpy")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)

    print(f"{'plant':10s} {'finite':6s} {'a_z@hover':>10s} {'roll_rate_frd':>14s} {'alt':>8s}")
    for name in args.plants.split(","):
        name = name.strip()
        try:
            plant = make_plant(name, clean=True, seed=args.seed)
            m = _profile(plant)
            print(f"{name:10s} {str(m['finite']):6s} {m['a_z_hover']:10.4f} "
                  f"{m['roll_rate_frd']:14.3f} {m['alt']:8.3f}")
        except ImportError as exc:
            print(f"{name:10s} skipped (missing dependency: {exc})")
        except Exception as exc:  # noqa: BLE001 - report and continue
            print(f"{name:10s} ERROR {type(exc).__name__}: {exc}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
