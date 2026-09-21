#include "hal/hal_hw.hpp"

// P9 deliverable. The HIL transport compiles only when CONFIG_DRONE_HIL is set,
// which requires firmware/main/Kconfig.projbuild to exist (RT#11).
#if defined(CONFIG_DRONE_HIL)
#error "HIL transport is not implemented yet (phase 9); CONFIG_DRONE_HIL must stay 'n'."
#endif

namespace drone {

// Fail-closed stub: no PWM is ever written. Replaced in P9.
bool HwHal::init() { return false; }
uint64_t HwHal::nowUs() { return 0; }
void HwHal::pwmWrite(const PwmCmd&) {}
void HwHal::pwmOff() {}
bool HwHal::imuRead(ImuSample& out) {
  out.valid = false;
  return false;
}
int HwHal::rcRawRead(uint8_t*, int) { return -1; }
bool HwHal::rcRead(RcSample&) { return false; }
void HwHal::ledSet(LedMode) {}
bool HwHal::vbatRead(float&) { return false; }
void HwHal::log(const char*) {}

}  // namespace drone
