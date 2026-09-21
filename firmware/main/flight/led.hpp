// Status LED state machine (boot / calib / armed / error / failsafe).
#pragma once

#include "flight/ports.hpp"

namespace drone {

class LedFsm final : public ILed {
 public:
  void set(LedMode mode) override;
  void update(uint32_t t_ms) override;
  LedMode mode() const override { return mode_; }
  bool on() const { return on_; }

 private:
  LedMode mode_ = LedMode::kBoot;
  uint32_t t_ms_ = 0;
  bool on_ = false;
};

}  // namespace drone
