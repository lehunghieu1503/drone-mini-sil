// ICM-20948 sensor configuration. Values mirror plant/drone_mini_params.py and
// are parity-checked by tests/test_param_parity.py (T3.19).
#pragma once

#include <cstdint>

namespace drone {

// Gyro: +/-2000 dps full scale, 16.4 LSB/dps.
inline constexpr float kGyroFsrDps = 2000.0f;
inline constexpr float kGyroLsbPerDps = 16.4f;

// Accel: +/-16 g full scale, 2048 LSB/g.
inline constexpr float kAccelFsrG = 16.0f;
inline constexpr float kAccelLsbPerG = 2048.0f;

// ODR: ICM-20948 has no 1000 Hz setting; 1125 Hz is used and decimated by 9/8
// to reach the 1 kHz control rate (drop 1 sample in 9).
inline constexpr float kImuOdrHz = 1125.0f;
inline constexpr int kImuDecimNum = 9;
inline constexpr int kImuDecimDen = 8;

// Noise (datasheet NSD expressed as white-noise sigma at the 1 kHz rate).
// ASSUMED(A7) -- measure before Tier C (Allan variance, static log).
inline constexpr float kGyroNoiseDps = 0.335f;
inline constexpr float kAccelNoiseG = 0.00514f;
inline constexpr float kGyroBiasMaxDps = 5.0f;  // ZRO uniform range, random per run

// Sensor axes aligned with body (R_BS = I).
// ASSUMED(A9) -- measure ICM orientation on the PCB before Tier C.
inline constexpr int kImuGyroSign[3] = {1, -1, -1};  // FLU -> FRD map applied by plant
inline constexpr int kImuAccelSign[3] = {1, -1, -1};

}  // namespace drone
