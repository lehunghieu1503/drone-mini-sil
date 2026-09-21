#include "flight/output.hpp"

#include <cmath>

namespace drone {

bool OutputStage::apply(const PwmCmd& in, bool armed, bool failsafe, bool imu_valid, IHal& hal) {
  if (!armed || failsafe || !imu_valid) {
    hal.pwmOff();
    return false;
  }
  PwmCmd safe{};
  for (int i = 0; i < 4; ++i) {
    const float v = in.mot[i];
    if (!std::isfinite(v) || v < 0.0f || v > 1.0f) {
      hal.pwmOff();
      return false;
    }
    safe.mot[i] = v;
  }
  hal.pwmWrite(safe);
  return true;
}

}  // namespace drone
