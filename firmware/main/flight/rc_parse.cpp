#include "flight/rc_parse.hpp"

#include <cstring>

namespace drone {

namespace {
constexpr int kSbusLen = 25;
constexpr uint8_t kSbusHeader = 0x0F;
constexpr uint16_t kSbusMin = 172;
constexpr uint16_t kSbusMid = 992;
constexpr uint16_t kSbusMax = 1811;
constexpr unsigned kArmChannel = 4;  // ch5
}  // namespace

void SbusParser::reset() {
  n_ = 0;
  frames_ = 0;
  std::memset(buf_, 0, sizeof(buf_));
}

uint16_t SbusParser::channel(const uint8_t* f, unsigned i) {
  const unsigned bit = 11u * i;
  const unsigned byte = 1u + (bit >> 3);
  const unsigned shift = bit & 7u;
  uint32_t raw = 0;
  raw |= static_cast<uint32_t>(f[byte]);
  raw |= static_cast<uint32_t>(f[byte + 1]) << 8;
  if (shift != 0) raw |= static_cast<uint32_t>(f[byte + 2]) << 16;
  return static_cast<uint16_t>((raw >> shift) & 0x7FFu);
}

float SbusParser::norm(uint16_t v, float lo, float hi) {
  const float span = (v >= kSbusMid) ? static_cast<float>(kSbusMax - kSbusMid)
                                     : static_cast<float>(kSbusMid - kSbusMin);
  float out = (static_cast<float>(v) - static_cast<float>(kSbusMid)) / span;
  if (out < lo) out = lo;
  if (out > hi) out = hi;
  return out;
}

bool SbusParser::decodeFrame(const uint8_t* f, RcSample& out) {
  if (f[0] != kSbusHeader) return false;
  if (f[23] & 0xF0u) return false;  // upper nibble must be zero

  uint16_t ch[16];
  for (unsigned i = 0; i < 16; ++i) ch[i] = channel(f, i);

  // Stuck-frame guard: all channels identical is not a plausible command.
  bool stuck = true;
  for (unsigned i = 1; i < 16; ++i) {
    if (ch[i] != ch[0]) { stuck = false; break; }
  }
  if (stuck) return false;

  out.roll = norm(ch[0], -1.0f, 1.0f);
  out.pitch = norm(ch[1], -1.0f, 1.0f);
  out.throttle = (static_cast<float>(ch[2]) - static_cast<float>(kSbusMin)) /
                 (static_cast<float>(kSbusMax) - static_cast<float>(kSbusMin));
  if (!(out.throttle > 0.0f)) out.throttle = 0.0f;
  if (out.throttle > 1.0f) out.throttle = 1.0f;
  out.yaw = norm(ch[3], -1.0f, 1.0f);
  out.armed_switch = ch[kArmChannel] > kSbusMid;
  out.rx_failsafe = (f[23] & 0x08u) != 0;
  out.frame_lost = (f[23] & 0x04u) != 0;
  out.frame_ok = !out.rx_failsafe;
  return true;
}

bool SbusParser::feed(const uint8_t* data, int len, RcSample& out) {
  if (len <= 0 || len > kBuf) {  // reject oversized burst (T3.13 `n>64`)
    reset();
    return false;
  }
  if (n_ + len > kBuf) reset();
  for (int i = 0; i < len; ++i) buf_[n_++] = data[i];

  bool decoded = false;
  int i = 0;
  while (n_ - i >= kSbusLen) {
    if (buf_[i] != kSbusHeader) { i++; continue; }
    if (decodeFrame(buf_ + i, out)) {
      decoded = true;
      frames_++;
      i += kSbusLen;
    } else {
      i++;  // resync: skip one byte and rescan for the next header
    }
  }
  if (i > 0) {
    std::memmove(buf_, buf_ + i, static_cast<size_t>(n_ - i));
    n_ -= i;
  }
  return decoded;
}

}  // namespace drone
