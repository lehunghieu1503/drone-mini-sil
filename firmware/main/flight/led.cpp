#include "flight/led.hpp"

namespace drone {

void LedFsm::set(LedMode mode) {
  mode_ = mode;
}

void LedFsm::update(uint32_t t_ms) {
  t_ms_ = t_ms;
  switch (mode_) {
    case LedMode::kArmed:
      on_ = true;  // solid
      break;
    case LedMode::kBoot:
      on_ = (t_ms % 200u) < 100u;  // 200 ms blink
      break;
    case LedMode::kCalib:
      on_ = (t_ms % 500u) < 250u;  // 500 ms blink
      break;
    case LedMode::kError:
      on_ = (t_ms % 100u) < 50u;  // fast blink
      break;
    case LedMode::kFailsafe:
      on_ = (t_ms % 250u) < 125u;  // medium blink
      break;
  }
}

}  // namespace drone
