// Complementary attitude estimator with boot gyro-bias calibration (phase 6).
//
// Frame: FRD. Roll right-wing-down positive; pitch nose-down positive; yaw CCW
// positive. The plant exposes FRD gyro (standard: q nose-up positive), so the
// pitch axis integrates -gyro[1].
#pragma once

#include "flight/ports.hpp"

namespace drone {

inline constexpr int kCalibSamples = 200;
inline constexpr float kCalibMaxGyroRps = 0.349f;  // 20 dps
inline constexpr int kEstDecim = 2;                // 500 Hz fusion

class ComplementaryEstimator final : public IEstimator {
 public:
  void reset() override;
  void calibrateUpdate(const ImuSample& imu) override;
  void update(const ImuSample& imu, float dt) override;
  void getAttitude(float out[3]) const override;
  bool imuValid() const override { return imu_valid_; }
  bool calibrating() const override { return state_ == State::kCollect; }
  bool calibrated() const override { return state_ == State::kDone; }
  void gyroBias(float out[3]) const override {
    out[0] = bias_[0];
    out[1] = bias_[1];
    out[2] = bias_[2];
  }

  const float* bias() const { return bias_; }

 private:
  enum class State : uint8_t { kIdle, kCollect, kDone };

  State state_ = State::kIdle;
  int n_ = 0;
  float sum_[3] = {0.0f, 0.0f, 0.0f};
  float bias_[3] = {0.0f, 0.0f, 0.0f};
  float roll_ = 0.0f;
  float pitch_ = 0.0f;
  float yaw_ = 0.0f;
  int invalid_count_ = 0;
  bool imu_valid_ = true;
  int decim_ = 0;
  float alpha_ = 0.998f;
};

}  // namespace drone
