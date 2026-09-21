"""ICM-20948 IMU model: white noise + per-run ZRO bias + quantize + clip.

Inputs/outputs are body FRD (already converted by the plant). Noise sigma at
1 kHz: gyro 0.335 dps, accel 5.14 mg (datasheet NSD)."""

from __future__ import annotations

import numpy as np

from . import drone_mini_params as P

DPS_TO_RPS = np.pi / 180.0


class Icm20948:
    def __init__(self, seed: int, clean: bool = False):
        self.clean = clean
        self.rng = np.random.default_rng(seed ^ 0x1C20948)
        self.gyro_lsb = 1.0 / P.GYRO_LSB_PER_DPS * DPS_TO_RPS  # rad/s per LSB
        self.acc_lsb = 1.0 / P.ACCEL_LSB_PER_G * P.GRAVITY     # m/s^2 per LSB
        if clean:
            self.bias = np.zeros(3)
            self.gyro_sigma = 0.0
            self.acc_sigma = 0.0
        else:
            bias_dps = self.rng.uniform(-P.GYRO_BIAS_MAX_DPS, P.GYRO_BIAS_MAX_DPS, size=3)
            self.bias = bias_dps * DPS_TO_RPS
            self.gyro_sigma = P.GYRO_NOISE_DPS * DPS_TO_RPS
            self.acc_sigma = P.ACCEL_NOISE_G * P.GRAVITY

    def _quantize(self, v, lsb, fsr):
        q = np.round(np.asarray(v) / lsb) * lsb
        return np.clip(q, -fsr, fsr)

    def sample(self, gyro_true, accel_true):
        if self.clean:
            gyro = np.asarray(gyro_true, dtype=float)
            accel = np.asarray(accel_true, dtype=float)
        else:
            gyro = np.asarray(gyro_true) + self.bias + self.rng.normal(0.0, self.gyro_sigma, 3)
            accel = np.asarray(accel_true) + self.rng.normal(0.0, self.acc_sigma, 3)
        gyro = self._quantize(gyro, self.gyro_lsb, P.GYRO_FSR_DPS * DPS_TO_RPS)
        accel = self._quantize(accel, self.acc_lsb, P.ACCEL_FSR_G * P.GRAVITY)
        return gyro, accel
