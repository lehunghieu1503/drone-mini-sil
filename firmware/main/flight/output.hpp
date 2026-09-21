// The single, fail-closed PWM output gate. Every PWM path goes through here.
#pragma once

#include "flight/ports.hpp"

namespace drone {

class OutputStage final : public IOutputStage {
 public:
  // gate = armed && !failsafe && imu_valid. Any violation (including a
  // non-finite or out-of-range channel) forces 0 and calls IHal::pwmOff().
  bool apply(const PwmCmd& in, bool armed, bool failsafe, bool imu_valid, IHal& hal) override;
};

}  // namespace drone
