#include "flight/estimator.hpp"

#include <cmath>

#include "flight/failsafe.hpp"  // kImuInvalidDebounce

namespace drone {

namespace {
constexpr float kPi = 3.14159265359f;
bool finite3(const float* v) {
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}
void wrapPi(float& a) {
  while (a > kPi) a -= 2.0f * kPi;
  while (a < -kPi) a += 2.0f * kPi;
}
}  // namespace

void ComplementaryEstimator::reset() {
  state_ = State::kIdle;
  n_ = 0;
  sum_[0] = sum_[1] = sum_[2] = 0.0f;
  bias_[0] = bias_[1] = bias_[2] = 0.0f;
  roll_ = pitch_ = yaw_ = 0.0f;
  invalid_count_ = 0;
  imu_valid_ = true;
  decim_ = 0;
}

void ComplementaryEstimator::calibrateUpdate(const ImuSample& imu) {
  if (state_ == State::kDone) return;
  if (state_ == State::kIdle) state_ = State::kCollect;
  if (!imu.valid || !finite3(imu.gyro_rps)) return;
  const float mag = std::sqrt(imu.gyro_rps[0] * imu.gyro_rps[0] +
                              imu.gyro_rps[1] * imu.gyro_rps[1] +
                              imu.gyro_rps[2] * imu.gyro_rps[2]);
  if (mag > kCalibMaxGyroRps) {  // motion detected -> restart collection
    n_ = 0;
    sum_[0] = sum_[1] = sum_[2] = 0.0f;
    return;
  }
  sum_[0] += imu.gyro_rps[0];
  sum_[1] += imu.gyro_rps[1];
  sum_[2] += imu.gyro_rps[2];
  if (++n_ >= kCalibSamples) {
    const float inv = 1.0f / static_cast<float>(n_);
    bias_[0] = sum_[0] * inv;
    bias_[1] = sum_[1] * inv;
    bias_[2] = sum_[2] * inv;
    state_ = State::kDone;
  }
}

void ComplementaryEstimator::update(const ImuSample& imu, float dt) {
  const bool bad = !imu.valid || !finite3(imu.gyro_rps) || !finite3(imu.accel_mps2) || !(dt > 0.0f);
  if (bad) {
    if (invalid_count_ < kImuInvalidDebounce) invalid_count_++;
    imu_valid_ = invalid_count_ < kImuInvalidDebounce;
    return;
  }
  invalid_count_ = 0;
  imu_valid_ = true;

  if (++decim_ < kEstDecim) return;
  decim_ = 0;
  const float dte = dt * static_cast<float>(kEstDecim);

  const float* a = imu.accel_mps2;
  const float an = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
  float roll_a = roll_;
  float pitch_a = pitch_;
  if (an > 1e-3f) {
    roll_a = std::atan2(-a[1], -a[2]);
    pitch_a = std::atan2(-a[0], -a[2]);
  }

  // Caller passes bias-corrected gyro (ControlLoop subtracts est.gyroBias()).
  const float gx = imu.gyro_rps[0];
  const float gy = -imu.gyro_rps[1];  // nose-down positive
  const float gz = imu.gyro_rps[2];

  roll_ = alpha_ * (roll_ + gx * dte) + (1.0f - alpha_) * roll_a;
  pitch_ = alpha_ * (pitch_ + gy * dte) + (1.0f - alpha_) * pitch_a;
  yaw_ += gz * dte;
  wrapPi(roll_);
  wrapPi(pitch_);
  wrapPi(yaw_);
}

void ComplementaryEstimator::getAttitude(float out[3]) const {
  out[0] = roll_;
  out[1] = pitch_;
  out[2] = yaw_;
}

}  // namespace drone
