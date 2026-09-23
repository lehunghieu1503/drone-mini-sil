"""Plant-side parameters + the working-assumptions registry.

Every value that is not yet measured lives in ASSUMPTIONS with an id. Code that
reads an assumption must say so. Never use an assumed value as the oracle for a
test that is supposed to validate that same assumption.
"""

from __future__ import annotations

import math

import numpy as np

# --- Timing / protocol ------------------------------------------------------
DT_US = 1000
DT = DT_US * 1e-6
CONTROL_HZ = 1000
IMU_ODR_HZ = 1125.0
IMU_DECIM_NUM = 9
IMU_DECIM_DEN = 8
TRANSPORT_TIMEOUT_S = 30

# --- IMU (ICM-20948) --------------------------------------------------------
GYRO_FSR_DPS = 2000.0
GYRO_LSB_PER_DPS = 16.4
ACCEL_FSR_G = 16.0
ACCEL_LSB_PER_G = 2048.0
GYRO_NOISE_DPS = 0.335       # sigma at 1 kHz
ACCEL_NOISE_G = 0.00514
GYRO_BIAS_MAX_DPS = 5.0      # ZRO uniform range, random per run

# --- Plant ------------------------------------------------------------------
# ASSUMED(A2) arm length, ASSUMED(A3) mass, ASSUMED(A4) motor time constant,
# ASSUMED(A5) thrust/torque coefficients. See ASSUMPTIONS below.
MASS_KG = 0.03
ARM_LENGTH_M = 0.043
GRAVITY = 9.81
TAU_M_S = 0.03
K_T = 4.7e-8         # N / (rad/s)^2
K_Q = 8.0e-10        # N*m / (rad/s)^2
OMEGA_MAX_RAD_S = 2500.0
INERTIA = (1.4e-5, 1.4e-5, 2.6e-5)  # kg*m^2, roll, pitch, yaw
LINEAR_DRAG = (0.0, 0.0, 0.0)
ANGULAR_DRAG = (0.0, 0.0, 0.0)
# ARM_LENGTH_M is the Cartesian offset (±d, ±d) of each motor, so the actual
# center-to-motor radius of the model is d*sqrt(2) (60.8 mm), not 43 mm. Keep
# ROTOR_POSITIONS as measured/assumed; only the derived radius is named here.
CENTER_TO_MOTOR_M = ARM_LENGTH_M * math.sqrt(2.0)

# --- Mixer / rotor geometry (documentation only; never an oracle) -----------
# Mot1 rear-right CW, Mot2 front-right CCW, Mot3 rear-left CCW, Mot4 front-left CW.
# Positions are body FLU (x fwd, y left). `dir` is the sign of the yaw-reaction
# torque: CW rotor (viewed from above, +z up) = +1, CCW = -1. Mixer yaw+
# increases the CCW pair (Mot2/Mot3) => FRD yaw+ (clockwise from above).
ROTOR_MAP = {
    "Mot1": {"pos": (-ARM_LENGTH_M, -ARM_LENGTH_M, 0.0), "dir": +1, "gpio": 0},
    "Mot2": {"pos": (+ARM_LENGTH_M, -ARM_LENGTH_M, 0.0), "dir": -1, "gpio": 1},
    "Mot3": {"pos": (-ARM_LENGTH_M, +ARM_LENGTH_M, 0.0), "dir": -1, "gpio": 2},
    "Mot4": {"pos": (+ARM_LENGTH_M, +ARM_LENGTH_M, 0.0), "dir": +1, "gpio": 3},
}
ROTOR_POSITIONS = [ROTOR_MAP[f"Mot{i}"]["pos"] for i in range(1, 5)]
ROTOR_DIRECTIONS = [ROTOR_MAP[f"Mot{i}"]["dir"] for i in range(1, 5)]

# Sign table mirror of the firmware mixer (phase-02). +,0,- per motor column.
MIXER_TABLE = {
    "roll": [-1, -1, +1, +1],
    "pitch": [+1, -1, +1, -1],
    "yaw": [-1, +1, +1, -1],
}

# --- Battery (1S LiPo, Thevenin) --------------------------------------------
BATTERY = {
    "cells": 1,
    "r_int_ohm": 0.040,       # ASSUMED(A6)
    "ocv_full": 4.20,
    "ocv_empty": 3.30,
    "r0_ohm": 0.040,
    "r1_ohm": 0.010,
    "c1_f": 2.0,
    "vbat_nominal": 3.85,
    "vbat_warn": 3.5,
    "vbat_crit": 3.3,
    "bod_mv": 3000.0,
    "ldo_dropout_v": 0.220,   # ASSUMED(A10)
    # Duty the pack must supply at hover, from ASSUMPTIONS only (never plant
    # state). Used to pick a default SoC whose loaded voltage is nominal (D9).
    "hover_duty": math.sqrt(MASS_KG * GRAVITY / (4.0 * K_T)) / OMEGA_MAX_RAD_S,
}


def hover_duty_nominal() -> float:
    """Hover duty from ASSUMPTIONS only (never from plant state) — RT#4."""
    return BATTERY["hover_duty"]


def pwm_to_omega(duty, vbat: float):
    """Duty (scalar or array) -> motor speed. omega_max scales with battery
    voltage (A5)."""
    d = np.clip(np.asarray(duty, dtype=float), 0.0, 1.0)
    v = 0.0 if not math.isfinite(vbat) else max(vbat, 0.0)
    omega_max = OMEGA_MAX_RAD_S * (v / BATTERY["vbat_nominal"])
    return d * omega_max


# --- Working assumptions registry -------------------------------------------
ASSUMPTIONS = {
    "A1": {"what": "motor positions/spin on the X frame", "value": "table in ROTOR_MAP",
           "gate": "mandatory spin-bench before HIL/flight"},
    "A2": {"what": "arm length d", "value": ARM_LENGTH_M, "gate": "measure PCB (Tier C)"},
    "A3": {"what": "mass", "value": MASS_KG, "gate": "mandatory before §9.1/§9.4"},
    "A4": {"what": "motor time constant tau_m", "value": TAU_M_S, "gate": "thrust stand step"},
    "A5": {"what": "k_t/k_q/omega_max", "value": (K_T, K_Q, OMEGA_MAX_RAD_S),
           "gate": "mandatory before §9.1/§9.4"},
    "A6": {"what": "battery internal resistance", "value": BATTERY["r_int_ohm"],
           "gate": "load-step measurement"},
    "A7": {"what": "gyro ZRO / random walk", "value": GYRO_BIAS_MAX_DPS,
           "gate": "Allan variance static log"},
    "A8": {"what": "RC protocol on J6", "value": "SBUS", "gate": "sniff before Tier C"},
    "A9": {"what": "IMU mounting orientation", "value": "R_BS = I",
           "gate": "measure on PCB before Tier C"},
    "A10": {"what": "LDO dropout / tau_ldo", "value": BATTERY["ldo_dropout_v"],
            "gate": "scope 3V3 vs V_BAT"},
}
