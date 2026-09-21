#include "flight/pid.hpp"

#include <cmath>

#include "flight/flight_params.hpp"

namespace drone {

namespace {
constexpr float kTwoPi = 6.28318530718f;
constexpr float kMaxRateRad = 5.236f;   // 300 dps
constexpr float kMaxAttRad = 0.5236f;   // 30 deg
}  // namespace

void Pid::setGains(float kp, float ki, float kd, float i_limit, float out_limit, float d_lpf_hz) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  i_limit_ = i_limit;
  out_limit_ = out_limit;
  d_tau_ = (d_lpf_hz > 0.0f) ? (1.0f / (kTwoPi * d_lpf_hz)) : 0.0f;
}

void Pid::reset() {
  i_ = 0.0f;
  d_filt_ = 0.0f;
  meas_prev_ = 0.0f;
  has_prev_ = false;
}

float Pid::step(float sp, float meas, float dt, bool sat_pos, bool sat_neg) {
  if (!std::isfinite(sp) || !std::isfinite(meas) || !(dt > 0.0f)) return 0.0f;
  const float e = sp - meas;

  // D on measurement (no derivative kick), first-order low-pass.
  float dmeas = 0.0f;
  if (has_prev_) dmeas = (meas - meas_prev_) / dt;
  meas_prev_ = meas;
  has_prev_ = true;
  const float alpha = (d_tau_ > 0.0f) ? (dt / (d_tau_ + dt)) : 1.0f;
  d_filt_ += alpha * (dmeas - d_filt_);
  const float dterm = -kd_ * d_filt_;

  // Conditional integration: hold I while saturated in the error's direction.
  const bool block = (sat_pos && e > 0.0f) || (sat_neg && e < 0.0f);
  if (!block) {
    i_ += ki_ * e * dt;
    if (i_ > i_limit_) i_ = i_limit_;
    if (i_ < -i_limit_) i_ = -i_limit_;
  }

  float out = kp_ * e + i_ + dterm;
  if (out > out_limit_) out = out_limit_;
  if (out < -out_limit_) out = -out_limit_;
  return std::isfinite(out) ? out : 0.0f;
}

RateController::RateController(IMixer& mixer) : mixer_(mixer) {
  for (int i = 0; i < 3; ++i) {
    pids_[i].setGains(kFlightParams.rate_kp[i], kFlightParams.rate_ki[i],
                      kFlightParams.rate_kd[i], kFlightParams.rate_i_limit,
                      kFlightParams.rate_out_limit, kFlightParams.rate_d_lpf_hz);
  }
}

void RateController::reset() {
  for (int i = 0; i < 3; ++i) pids_[i].reset();
  last_shift_ = 0.0f;
}

float RateController::update(const ImuSample& imu, const RateSp& sp, PwmCmd& out) {
  const float dt = kControlDtS;
  // Previous saturation: shift < 0 => high side pulled down (positive sat),
  // shift > 0 => low side pushed up (negative sat).
  const bool sat_pos = last_shift_ < 0.0f;
  const bool sat_neg = last_shift_ > 0.0f;

  // FRD gyro is nose-up positive; the controller works nose-down positive.
  const float meas_pitch = -imu.gyro_rps[1];
  const float dr = pids_[0].step(sp.roll, imu.gyro_rps[0], dt, sat_pos, sat_neg);
  const float dp = pids_[1].step(sp.pitch, meas_pitch, dt, sat_pos, sat_neg);
  const float dy = pids_[2].step(sp.yaw, imu.gyro_rps[2], dt, sat_pos, sat_neg);

  last_shift_ = mixer_.write(sp.throttle, dr, dp, dy, out);
  return last_shift_;
}

AttitudeController::AttitudeController() {
  for (int i = 0; i < 3; ++i) kp_[i] = kFlightParams.att_kp[i];
  limit_ = kFlightParams.att_rate_limit;
}

void AttitudeController::reset() {}

void AttitudeController::update(const float sp[3], const float est[3], float dt, float out[3]) {
  (void)dt;
  for (int i = 0; i < 3; ++i) {
    float e = sp[i] - est[i];
    if (!std::isfinite(e)) e = 0.0f;
    if (i == 2) {  // shortest-path yaw error
      while (e > 3.14159265f) e -= kTwoPi;
      while (e < -3.14159265f) e += kTwoPi;
    }
    float r = kp_[i] * e;
    if (r > limit_) r = limit_;
    if (r < -limit_) r = -limit_;
    out[i] = std::isfinite(r) ? r : 0.0f;
  }
}

void rc_stick_to_rate_sp(const RcSample& rc, RateSp& out) {
  const float r = std::isfinite(rc.roll) ? rc.roll : 0.0f;
  const float p = std::isfinite(rc.pitch) ? rc.pitch : 0.0f;
  const float y = std::isfinite(rc.yaw) ? rc.yaw : 0.0f;
  out.roll = r * kMaxRateRad;
  out.pitch = p * kMaxRateRad;  // nose-down positive
  out.yaw = y * kMaxRateRad;
  out.throttle = std::isfinite(rc.throttle) ? rc.throttle : 0.0f;
}

void rc_stick_to_att_sp(const RcSample& rc, float out[3]) {
  const float r = std::isfinite(rc.roll) ? rc.roll : 0.0f;
  const float p = std::isfinite(rc.pitch) ? rc.pitch : 0.0f;
  out[0] = r * kMaxAttRad;
  out[1] = p * kMaxAttRad;  // nose-down positive
  out[2] = 0.0f;
}

}  // namespace drone
