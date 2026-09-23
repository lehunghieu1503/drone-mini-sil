// SIL HAL: lockstep transport client (AF_UNIX SOCK_SEQPACKET). Host-only.
// No exceptions: I/O errors are returned as IoResult.
#pragma once

#include <cstdint>

#include "flight/ports.hpp"
#include "hal/sil_wire.hpp"

namespace drone {

class SilHal final : public IHal {
 public:
  enum class IoResult { kOk = 0, kClosed = 1, kProto = 2, kTimeout = 3 };

  SilHal() = default;
  ~SilHal() override;

  // Connect with retry/backoff; sets SO_RCVTIMEO. `name` is the abstract UDS
  // name (no leading NUL in the string).
  IoResult connectTo(const char* name, int timeout_s);
  IoResult helloAck();   // client: send HELLO, expect matching ACK
  IoResult serveHello(); // server-side helper used by tests/loopback
  IoResult recvState();
  IoResult sendOut();
  IoResult sendBye();
  void close();

  // IHal
  bool init() override { return fd_ >= 0; }
  uint64_t nowUs() override { return state_.imu.t_us; }
  void pwmWrite(const PwmCmd& out) override;
  void pwmOff() override;
  bool imuRead(ImuSample& out) override;
  int rcRawRead(uint8_t* buf, int cap) override;
  bool rcRead(RcSample& out) override;
  void ledSet(LedMode mode) override;
  bool vbatRead(float& volts) override;
  void log(const char* msg) override;

  void setStatus(bool armed, bool failsafe, bool imu_valid, const float est[3], float sat_shift,
                 uint32_t tick);
  const wire::SilOut& lastOut() const { return out_; }
  int fd() const { return fd_; }

 private:
  IoResult sendMsg(wire::MsgType type, const void* payload, int len);
  IoResult recvInto(uint8_t* buf, int cap, int& out_len, wire::Hdr* hdr_out, const uint8_t** payload);

  int fd_ = -1;
  uint32_t seq_ = 0;
  uint32_t last_state_seq_ = 0;
  uint64_t last_state_us_ = 0;
  bool have_state_ = false;
  int timeout_s_ = 30;
  wire::StateRc state_{};
  wire::SilOut out_{};
  uint8_t rx_[256]{};
};

}  // namespace drone
