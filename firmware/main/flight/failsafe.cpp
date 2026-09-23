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
  armed_switch_prev_ = false;
  reason_ = kReasonNone;
  last_good_us_ = 0;
  have_frame_ = false;
  good_frames_ = 0;
  lost_count_ = 0;
  imu_bad_count_ = 0;
  vbat_crit_count_ = 0;
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
  // Only a *fresh* frame counts (a stale t_us is not a good frame, T7.9). A
  // silent tick must not clear the counter: the 100 ms timeout is the only cut.
  const bool fresh = rc.frame_ok && (!have_frame_ || rc.t_us > last_good_us_);
  if (fresh) {
    good_frames_++;
    last_good_us_ = rc.t_us;
    have_frame_ = true;
    lost_count_ = 0;
  } else if (rc.frame_lost && ++lost_count_ >= kRcFrameLostN) {
    lost_count_ = kRcFrameLostN;
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

  // --- battery (D4) --------------------------------------------------------
  // One non-finite sample is an immediate latch; below-crit needs a debounce so
  // a single sagged sample cannot cut the motors mid-flight.
  vbat_nan_ = !std::isfinite(vbat);
  if (vbat_nan_ || vbat >= kVbatCrit) {
    vbat_crit_count_ = 0;
  } else if (vbat_crit_count_ < kVbatCritSamples) {
    vbat_crit_count_++;
  }
  const bool vbat_crit = vbat_crit_count_ >= kVbatCritSamples;

  // --- in-flight latch clear (D3) -----------------------------------------
  // A switch off->on edge with a fresh, centred, low-throttle frame clears any
  // in-flight latch. Boot latch and a held switch never clear (user: cycle).
  const bool switch_edge_up = rc.armed_switch && !armed_switch_prev_;
  armed_switch_prev_ = rc.armed_switch;
  const bool sticks_centered = std::fabs(rc.roll) <= kArmStickMax &&
                               std::fabs(rc.pitch) <= kArmStickMax &&
                               std::fabs(rc.yaw) <= kArmStickMax;
  if (switch_edge_up && !boot_latched_ && fresh && rc.throttle <= kArmThrottleMax &&
      sticks_centered && !imu_invalid && !vbat_nan_ && vbat >= kVbatCrit) {
    latched_ = false;
    active_ = false;
    reason_ = kReasonNone;
  }

  // --- latch ---------------------------------------------------------------
  uint32_t r = kReasonNone;
  if (rc_timeout) r |= kReasonRcTimeout;
  if (rc.rx_failsafe) r |= kReasonRcFlag;
  if (imu_invalid) r |= kReasonImuInvalid;
  if (vbat_nan_) r |= kReasonVbatNan;
  if (vbat_crit) r |= kReasonVbatCrit;
  if (r != kReasonNone) {
    latched_ = true;
    active_ = true;
    reason_ |= r;
  }

  // --- arm / disarm --------------------------------------------------------
  // The low-throttle / centred-stick condition gates the arm *transition* only;
  // once armed the craft stays armed while the switch is on and nothing faulted.
  if (arm_test) {
    if (!arm_test_prev_) {
      // Explicit arm request: never clears a latch, never overrides boot latch.
      if (!latched_ && !boot_latched_ && !active_) armed_ = true;
    } else {
      armed_ = armed_ && !latched_ && !boot_latched_;
    }
  } else if (armed_) {
    // A held frame keeps the craft armed between real frames; only a switch-off
    // on a decoded frame, a latch, or the 100 ms timeout disarms (D1).
    armed_ = rc.armed_switch && !latched_ && !boot_latched_ && !imu_invalid;
  } else {
    armed_ = fresh && rc.armed_switch && calibrated && !latched_ && !boot_latched_ &&
             good_frames_ >= kArmMinGoodFrames && rc.throttle <= kArmThrottleMax &&
             sticks_centered && !imu_invalid && !vbat_nan_ && vbat >= kVbatCrit;
  }
  if (active_) armed_ = false;
  arm_test_prev_ = arm_test;
}

}  // namespace drone
