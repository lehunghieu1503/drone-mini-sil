"""T3.8/T3.9 — IMU quantize/clip and white-noise sigma at 1 kHz."""

import numpy as np
import pytest

from plant import drone_mini_params as P
from plant.icm20948 import Icm20948, DPS_TO_RPS


def test_t3_8_quantize_and_clip():
    icm = Icm20948(seed=1, clean=False)
    # Huge true accel is clipped to the +/-16 g full scale.
    _g, a = icm.sample([0, 0, 0], [1e6, 0, 0])
    assert a[0] == pytest.approx(P.ACCEL_FSR_G * P.GRAVITY, rel=1e-6)
    # A small change below one LSB quantizes to zero.
    _g2, a2 = icm.sample([0, 0, 0], [1e-6, 0, 0])
    assert abs(a2[0]) <= icm.acc_lsb + 1e-12
    # gyro clip
    g, _ = icm.sample([1e6, 0, 0], [0, 0, 0])
    assert g[0] == pytest.approx(P.GYRO_FSR_DPS * DPS_TO_RPS, rel=1e-6)


def test_t3_9_noise_sigma():
    icm = Icm20948(seed=3, clean=False)
    n = 40000
    samples = np.empty((n, 3))
    for i in range(n):
        g, _a = icm.sample([0, 0, 0], [0, 0, 0])
        samples[i] = g
    measured = samples[2000:, 0].std()  # drop initial transient? bias is constant
    # bias is constant so it does not inflate std
    expected = P.GYRO_NOISE_DPS * DPS_TO_RPS
    assert measured == pytest.approx(expected, rel=0.05)
