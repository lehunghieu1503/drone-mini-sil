// Arm state machine + failsafe latches (phase 7). No PWM is written here; the
// OutputStage gate reads armed()/active()/imu validity.
#pragma once

#include "flight/ports.hpp"

namespace drone {

enum FailsafeReason : uint32_t {
  kReasonNone = 0,
  kReasonRcTimeout = 1u << 0,
  kReasonRcFlag = 1u << 1,
  kReasonImuInvalid = 1u << 2,
  kReasonVbatCrit = 1u << 3,
  kReasonBootLatch = 1u << 4,
  kReasonVbatNan = 1u << 5,
};

// esp_reset_reason_t values (mirrored; flight/ cannot include IDF headers).
inline constexpr uint32_t kResetIntWdt = 5;
inline constexpr uint32_t kResetTaskWdt = 6;
inline constexpr uint32_t kResetWdt = 7;
inline constexpr uint32_t kResetBrownout = 9;

inline constexpr uint64_t kRcTimeoutUs = 100000;  // 100 ms virtual time
inline constexpr int kRcFrameLostN = 3;
inline constexpr int kArmMinGoodFrames = 10;
inline constexpr int kImuInvalidDebounce = 5;
inline constexpr int kVbatCritSamples = 20;  // consecutive samples below crit
inline constexpr float kVbatWarn = 3.5f;
inline constexpr float kVbatCrit = 3.3f;
inline constexpr float kArmThrottleMax = 0.05f;
inline constexpr float kArmStickMax = 0.05f;

class FailsafeFsm final : public IFailsafe {
 public:
  void reset() override;
  void boot(uint32_t reset_reason) override;
  void update(const RcSample& rc, const ImuSample& imu, float vbat, uint64_t now_us,
              bool calibrated, bool arm_test) override;

  bool armed() const override { return armed_; }
  bool active() const override { return active_; }
  bool outputZero() const override { return !armed_ || active_; }
  uint32_t reason() const override { return reason_; }

  bool vbatNan() const override { return vbat_nan_; }
  bool bootLatched() const override { return boot_latched_; }
  int goodFrames() const { return good_frames_; }

 private:
  bool armed_ = false;
  bool active_ = false;
  bool latched_ = false;
  bool boot_latched_ = false;
  bool vbat_nan_ = false;
  bool arm_test_prev_ = false;
  bool armed_switch_prev_ = false;
  uint32_t reason_ = 0;
  uint64_t last_good_us_ = 0;
  bool have_frame_ = false;
  int good_frames_ = 0;
  int lost_count_ = 0;
  int imu_bad_count_ = 0;
  int vbat_crit_count_ = 0;
};

}  // namespace drone
