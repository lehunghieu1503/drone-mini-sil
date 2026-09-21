// SBUS codec, decoupled from the transport. One byte path for SIL/HIL/chip:
// IHal::rcRawRead() -> IRcParser::feed().
#pragma once

#include "flight/ports.hpp"

namespace drone {

class SbusParser final : public IRcParser {
 public:
  void reset() override;
  bool feed(const uint8_t* data, int len, RcSample& out) override;

  int frames() const { return frames_; }

 private:
  static bool decodeFrame(const uint8_t* f, RcSample& out);
  static float norm(uint16_t v, float lo, float hi);
  static uint16_t channel(const uint8_t* f, unsigned i);

  static constexpr int kBuf = 64;
  uint8_t buf_[kBuf];
  int n_ = 0;
  int frames_ = 0;
};

}  // namespace drone
