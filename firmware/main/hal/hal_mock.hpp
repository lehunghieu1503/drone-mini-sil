// Recording HAL used only by the host test lib (never on the chip).
#pragma once

#include <cstring>

#include "flight/ports.hpp"

namespace drone {

class MockHal final : public IHal {
 public:
  struct Log {
    int pwm_writes = 0;
    int pwm_offs = 0;
    int led_sets = 0;
    int imu_reads = 0;
    int rc_reads = 0;
    int vbat_reads = 0;
    PwmCmd last_pwm{};
    LedMode last_led = LedMode::kBoot;
    char last_log[128] = {0};
  };

  bool init() override { return true; }
  uint64_t nowUs() override { return now_us_; }

  void pwmWrite(const PwmCmd& out) override {
    log_.pwm_writes++;
    log_.last_pwm = out;
  }
  void pwmOff() override {
    log_.pwm_offs++;
    log_.last_pwm = PwmCmd{};
  }
  bool imuRead(ImuSample& out) override {
    log_.imu_reads++;
    if (!imu_ok_) {
      out.valid = false;
      return false;
    }
    out = imu_;
    return imu_.valid;
  }
  int rcRawRead(uint8_t* buf, int cap) override {
    log_.rc_reads++;
    const int n = (rc_len_ < cap) ? rc_len_ : cap;
    if (n > 0) std::memcpy(buf, rc_raw_, static_cast<size_t>(n));
    return n;
  }
  bool rcRead(RcSample& out) override {
    out = rc_;
    return rc_.frame_ok;
  }
  void ledSet(LedMode mode) override {
    log_.led_sets++;
    log_.last_led = mode;
  }
  bool vbatRead(float& volts) override {
    log_.vbat_reads++;
    volts = vbat_;
    return vbat_ok_;
  }
  void log(const char* msg) override {
    std::strncpy(log_.last_log, msg, sizeof(log_.last_log) - 1);
    log_.last_log[sizeof(log_.last_log) - 1] = '\0';
  }

  // --- test hooks ----------------------------------------------------------
  void setNowUs(uint64_t v) { now_us_ = v; }
  void setImu(const ImuSample& s) {
    imu_ = s;
    imu_ok_ = true;
  }
  void setImuInvalid() {
    imu_ok_ = false;
    imu_.valid = false;
  }
  void setRcRaw(const uint8_t* data, int n) {
    if (n > 64) n = 64;
    if (n > 0) std::memcpy(rc_raw_, data, static_cast<size_t>(n));
    rc_len_ = n;
  }
  void setVbat(float v, bool ok) {
    vbat_ = v;
    vbat_ok_ = ok;
  }
  const Log& log() const { return log_; }
  void resetLog() { log_ = Log{}; }

 private:
  Log log_{};
  uint64_t now_us_ = 0;
  ImuSample imu_{};
  bool imu_ok_ = false;
  uint8_t rc_raw_[64]{};
  int rc_len_ = 0;
  RcSample rc_{};
  float vbat_ = 0.0f;
  bool vbat_ok_ = false;
};

}  // namespace drone
