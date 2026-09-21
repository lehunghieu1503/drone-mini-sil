// Hardware HAL for the ESP32-C3 (phase 9). Stub until a board is available.
#pragma once

#include "flight/ports.hpp"

namespace drone {

class HwHal final : public IHal {
 public:
  bool init() override;
  uint64_t nowUs() override;
  void pwmWrite(const PwmCmd& out) override;
  void pwmOff() override;
  bool imuRead(ImuSample& out) override;
  int rcRawRead(uint8_t* buf, int cap) override;
  bool rcRead(RcSample& out) override;
  void ledSet(LedMode mode) override;
  bool vbatRead(float& volts) override;
  void log(const char* msg) override;
};

}  // namespace drone
