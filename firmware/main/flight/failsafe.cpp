#include "flight/failsafe.hpp"

#include <cmath>

namespace drone {

void FailsafeFsm::reset() {
  armed_ = false;
  active_ = false;
  latched_ = false;
  boot_latched_ = false;
  vbat_nan_ = false;
  arm_test_prev_ = false;
  reason_ = kReasonNone;
  last_good_us_ = 0;
  have_frame_ = false;
  good_frames_ = 0;
  lost_count_ = 0;
  imu_bad_count_ = 0;
}

void FailsafeFsm::boot(uint32_t reset_reason) {
  reset();
  const bool after_fault = (reset_reason == kResetBrownout) || (reset_reason == kResetIntWdt) ||
                           (reset_reason == kResetTaskWdt) || (reset_reason == kResetWdt);
  if (after_fault) {
    boot_latched_ = true;
    latched_ = true;
    reason_ |= kReasonBootLatch;
  }
}

void FailsafeFsm::update(const RcSample& rc, const ImuSample& imu, float vbat, uint64_t now_us,
                         bool calibrated, bool arm_test) {
  // --- liveness from virtual time -----------------------------------------
  // Only a *fresh* frame counts (a stale t_us is not a good frame, T7.9).
  const bool fresh = rc.frame_ok && (!have_frame_ || rc.t_us > last_good_us_);
  if (fresh) {
    good_frames_++;
    last_good_us_ = rc.t_us;
    have_frame_ = true;
    lost_count_ = 0;
  } else {
    good_frames_ = 0;
    if (rc.frame_lost && ++lost_count_ >= kRcFrameLostN) lost_count_ = kRcFrameLostN;
  }
  const bool rc_timeout = have_frame_ && (now_us - last_good_us_ >= kRcTimeoutUs);

  // --- IMU validity (debounced) -------------------------------------------
  const bool imu_bad = !imu.valid || !std::isfinite(imu.gyro_rps[0]) ||
                       !std::isfinite(imu.gyro_rps[1]) || !std::isfinite(imu.gyro_rps[2]) ||
                       !std::isfinite(imu.accel_mps2[0]) || !std::isfinite(imu.accel_mps2[1]) ||
                       !std::isfinite(imu.accel_mps2[2]);
  if (imu_bad) {
    if (imu_bad_count_ < kImuInvalidDebounce) imu_bad_count_++;
  } else {
    imu_bad_count_ = 0;
  }
  const bool imu_invalid = imu_bad_count_ >= kImuInvalidDebounce;

  // --- battery -------------------------------------------------------------
  vbat_nan_ = !std::isfinite(vbat);
  const bool vbat_crit = !vbat_nan_ && vbat < kVbatCrit;

  // --- latch ---------------------------------------------------------------
  uint32_t r = kReasonNone;
  if (rc_timeout) r |= kReasonRcTimeout;
  if (rc.rx_failsafe) r |= kReasonRcFlag;
  if (imu_invalid) r |= kReasonImuInvalid;
  if (vbat_crit) r |= kReasonVbatCrit;
  if (r != kReasonNone) {
    latched_ = true;
    active_ = true;
    reason_ |= r;
  }

  // --- arm / disarm --------------------------------------------------------
  // The low-throttle condition gates the arm *transition* only; once armed the
  // craft stays armed while the switch is on and nothing has faulted.
  if (arm_test) {
    if (!arm_test_prev_) {  // explicit arm request clears the latch
      latched_ = false;
      boot_latched_ = false;
      active_ = false;
      reason_ = kReasonNone;
      armed_ = true;
    } else {
      armed_ = armed_ && !latched_;
    }
  } else if (armed_) {
    armed_ = rc.frame_ok && rc.armed_switch && !latched_ && !boot_latched_ && !imu_invalid;
  } else {
    armed_ = rc.frame_ok && rc.armed_switch && calibrated && !latched_ && !boot_latched_ &&
             good_frames_ >= kArmMinGoodFrames && rc.throttle <= kArmThrottleMax && !imu_invalid;
  }
  if (active_) armed_ = false;
  arm_test_prev_ = arm_test;
}

}  // namespace drone
