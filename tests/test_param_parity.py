"""T3.19 — C++ constants and Python params must not drift."""

import re

import pytest

from _bind import ROOT
from plant import drone_mini_params as P
from plant import sil_proto as sp

SENSOR = (ROOT / "firmware" / "main" / "flight" / "sensor_cfg.hpp").read_text()
WIRE = (ROOT / "firmware" / "main" / "hal" / "sil_wire.hpp").read_text()


def _f(text, name):
    m = re.search(rf"{name}\s*=\s*([0-9.]+)f", text)
    assert m, f"{name} not found"
    return float(m.group(1))


def _i(text, name):
    m = re.search(rf"{name}\s*=\s*([0-9]+)", text)
    assert m, f"{name} not found"
    return int(m.group(1))


def test_sensor_parity():
    assert _f(SENSOR, "kGyroFsrDps") == P.GYRO_FSR_DPS
    assert _f(SENSOR, "kGyroLsbPerDps") == P.GYRO_LSB_PER_DPS
    assert _f(SENSOR, "kAccelFsrG") == P.ACCEL_FSR_G
    assert _f(SENSOR, "kAccelLsbPerG") == P.ACCEL_LSB_PER_G
    assert _f(SENSOR, "kImuOdrHz") == P.IMU_ODR_HZ
    assert _f(SENSOR, "kGyroNoiseDps") == P.GYRO_NOISE_DPS
    assert _f(SENSOR, "kAccelNoiseG") == P.ACCEL_NOISE_G
    assert _f(SENSOR, "kGyroBiasMaxDps") == P.GYRO_BIAS_MAX_DPS


def test_decim_parity():
    assert _i(SENSOR, "kImuDecimNum") == P.IMU_DECIM_NUM
    assert _i(SENSOR, "kImuDecimDen") == P.IMU_DECIM_DEN


def test_timing_and_version_parity():
    assert _i(WIRE, "kSilDtUs") == P.DT_US
    assert _i(WIRE, "kProtoVersion") == sp.VERSION
    m = re.search(r"kMagic\s*=\s*0x([0-9A-Fa-f]+)", WIRE)
    assert m and int(m.group(1), 16) == sp.MAGIC
