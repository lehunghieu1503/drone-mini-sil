// PID unit + rate cascade + attitude cascade (P5/P6).
#pragma once

#include "flight/ports.hpp"

namespace drone {

class Pid final : public IPid {
 public:
  void reset() override;
  float step(float sp, float meas, float dt, bool sat_pos, bool sat_neg) override;
  void setGains(float kp, float ki, float kd, float i_limit, float out_limit, float d_lpf_hz);

 private:
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;
  float i_limit_ = 0.0f;
  float out_limit_ = 0.0f;
  float d_tau_ = 0.0f;
  float i_ = 0.0f;
  float d_filt_ = 0.0f;
  float meas_prev_ = 0.0f;
  bool has_prev_ = false;
};

class RateController final : public IRateController {
 public:
  explicit RateController(IMixer& mixer);
  void reset() override;
  float update(const ImuSample& imu, const RateSp& sp, PwmCmd& out) override;

  bool lastSatPos() const { return last_sat_pos_; }
  bool lastSatNeg() const { return last_sat_neg_; }

 private:
  IMixer& mixer_;
  Pid pids_[3];
  // Pre-clip saturation flags from the previous mixer write, applied to this
  // tick's conditional integration (D6).
  bool last_sat_pos_ = false;
  bool last_sat_neg_ = false;
};

class AttitudeController final : public IAttitudeController {
 public:
  AttitudeController();
  void reset() override;
  void update(const float sp[3], const float est[3], float dt, float out[3]) override;

 private:
  float kp_[3];
  float limit_;
};

// Stick mapping. Roll/pitch sticks map to angles in attitude mode and to rates
// in rate mode; yaw is always a rate command. Forward pitch stick = nose down.
void rc_stick_to_rate_sp(const RcSample& rc, RateSp& out);
void rc_stick_to_att_sp(const RcSample& rc, float out[3]);

// Yaw stick -> yaw rate setpoint (rad/s), shared by rate and attitude modes so
// the mapping has one definition. Non-finite input maps to 0.
float stick_yaw_to_rate(float stick);

}  // namespace drone
