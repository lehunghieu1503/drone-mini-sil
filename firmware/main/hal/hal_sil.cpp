#include "hal/hal_sil.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <csignal>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace drone {

namespace {
constexpr int kConnectRetries = 50;
constexpr int kConnectBackoffUs = 100000;  // 100 ms
}  // namespace

SilHal::~SilHal() { close(); }

void SilHal::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

SilHal::IoResult SilHal::connectTo(const char* name, int timeout_s) {
  timeout_s_ = timeout_s;
  std::signal(SIGPIPE, SIG_IGN);

  fd_ = ::socket(AF_UNIX, SOCK_SEQPACKET, 0);
  if (fd_ < 0) return IoResult::kClosed;

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  const size_t namelen = std::strlen(name);
  addr.sun_path[0] = '\0';  // abstract namespace
  if (namelen > sizeof(addr.sun_path) - 1) return IoResult::kClosed;
  std::memcpy(addr.sun_path + 1, name, namelen);
  const socklen_t alen =
      static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + 1 + namelen);

  bool connected = false;
  for (int i = 0; i < kConnectRetries; ++i) {
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), alen) == 0) {
      connected = true;
      break;
    }
    ::usleep(kConnectBackoffUs);
  }
  if (!connected) return IoResult::kClosed;

  timeval tv{};
  tv.tv_sec = timeout_s_;
  tv.tv_usec = 0;
  ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  ::setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  return IoResult::kOk;
}

SilHal::IoResult SilHal::sendMsg(wire::MsgType type, const void* payload, int len) {
  if (fd_ < 0) return IoResult::kClosed;
  uint8_t buf[256];
  const int total = wire::packMsg(buf, static_cast<int>(sizeof(buf)), type, seq_++, payload, len);
  if (total < 0) return IoResult::kProto;
  const ssize_t n = ::send(fd_, buf, static_cast<size_t>(total), MSG_NOSIGNAL);
  if (n < 0) return (errno == EAGAIN || errno == EWOULDBLOCK) ? IoResult::kTimeout
                                                              : IoResult::kClosed;
  return IoResult::kOk;
}

SilHal::IoResult SilHal::recvInto(uint8_t* buf, int cap, int& out_len, wire::Hdr* hdr_out,
                                  const uint8_t** payload) {
  if (fd_ < 0) return IoResult::kClosed;
  const ssize_t n = ::recv(fd_, buf, static_cast<size_t>(cap), 0);
  if (n == 0) return IoResult::kClosed;
  if (n < 0) {
    if (errno == EINTR) return recvInto(buf, cap, out_len, hdr_out, payload);
    if (errno == EAGAIN || errno == EWOULDBLOCK) return IoResult::kTimeout;
    return IoResult::kClosed;
  }
  out_len = static_cast<int>(n);
  wire::Hdr h{};
  const int r = wire::unpackMsg(buf, out_len, h, *payload);
  if (r < 0) return IoResult::kProto;
  if (hdr_out) *hdr_out = h;
  return IoResult::kOk;
}

SilHal::IoResult SilHal::helloAck() {
  wire::SilHello local{};
  wire::fillHello(local);
  if (sendMsg(wire::MsgType::kHello, &local, sizeof(local)) != IoResult::kOk) {
    return IoResult::kClosed;
  }
  int len = 0;
  wire::Hdr h{};
  const uint8_t* payload = nullptr;
  if (recvInto(rx_, sizeof(rx_), len, &h, &payload) != IoResult::kOk) return IoResult::kClosed;
  if (h.type != static_cast<uint8_t>(wire::MsgType::kAck) || h.len != sizeof(wire::SilHello)) {
    return IoResult::kProto;
  }
  wire::SilHello peer{};
  std::memcpy(&peer, payload, sizeof(peer));
  return wire::helloMatches(local, peer) ? IoResult::kOk : IoResult::kProto;
}

SilHal::IoResult SilHal::serveHello() {
  int len = 0;
  wire::Hdr h{};
  const uint8_t* payload = nullptr;
  if (recvInto(rx_, sizeof(rx_), len, &h, &payload) != IoResult::kOk) return IoResult::kClosed;
  if (h.type != static_cast<uint8_t>(wire::MsgType::kHello) || h.len != sizeof(wire::SilHello)) {
    return IoResult::kProto;
  }
  wire::SilHello peer{};
  std::memcpy(&peer, payload, sizeof(peer));
  wire::SilHello local{};
  wire::fillHello(local);
  return sendMsg(wire::MsgType::kAck, &local, sizeof(local));
}

SilHal::IoResult SilHal::recvState() {
  int len = 0;
  wire::Hdr h{};
  const uint8_t* payload = nullptr;
  const IoResult r = recvInto(rx_, sizeof(rx_), len, &h, &payload);
  if (r != IoResult::kOk) return r;
  if (h.type == static_cast<uint8_t>(wire::MsgType::kBye)) return IoResult::kClosed;
  if (h.type != static_cast<uint8_t>(wire::MsgType::kState) || h.len != sizeof(wire::StateRc)) {
    return IoResult::kProto;
  }
  wire::StateRc st{};
  std::memcpy(&st, payload, sizeof(st));

  // Reject replay / frozen time so a stalled plant cannot hide a failsafe.
  if (have_state_ && (h.seq <= last_state_seq_ || st.imu.t_us <= last_state_us_)) {
    return IoResult::kProto;
  }
  last_state_seq_ = h.seq;
  last_state_us_ = st.imu.t_us;
  have_state_ = true;
  state_ = st;
  return IoResult::kOk;
}

SilHal::IoResult SilHal::sendOut() {
  return sendMsg(wire::MsgType::kFwOut, &out_, sizeof(out_));
}

SilHal::IoResult SilHal::sendBye() { return sendMsg(wire::MsgType::kBye, nullptr, 0); }

// --- IHal ------------------------------------------------------------------
void SilHal::pwmWrite(const PwmCmd& out) {
  for (int i = 0; i < 4; ++i) out_.mot[i] = out.mot[i];
}
void SilHal::pwmOff() {
  for (int i = 0; i < 4; ++i) out_.mot[i] = 0.0f;
}
bool SilHal::imuRead(ImuSample& out) {
  out.gyro_rps[0] = state_.imu.gyro_rps[0];
  out.gyro_rps[1] = state_.imu.gyro_rps[1];
  out.gyro_rps[2] = state_.imu.gyro_rps[2];
  out.accel_mps2[0] = state_.imu.accel_mps2[0];
  out.accel_mps2[1] = state_.imu.accel_mps2[1];
  out.accel_mps2[2] = state_.imu.accel_mps2[2];
  out.mag_uT[0] = state_.imu.mag_uT[0];
  out.mag_uT[1] = state_.imu.mag_uT[1];
  out.mag_uT[2] = state_.imu.mag_uT[2];
  out.temp_c = state_.imu.temp_c;
  out.t_us = nowUs();  // re-stamp from the transport clock (RT#8)
  out.valid = state_.imu.valid != 0;
  return out.valid;
}
int SilHal::rcRawRead(uint8_t* buf, int cap) {
  int n = static_cast<int>(state_.rc_len);
  if (n > cap) n = cap;
  if (n > 32) n = 32;
  if (n > 0) std::memcpy(buf, state_.rc_raw, static_cast<size_t>(n));
  return n;
}
bool SilHal::rcRead(RcSample&) { return false; }  // decoding lives in SbusParser
void SilHal::ledSet(LedMode mode) { out_.led_mode = static_cast<uint8_t>(mode); }
bool SilHal::vbatRead(float& volts) {
  volts = state_.vbat;
  return std::isfinite(state_.vbat);  // NaN is never "healthy" (RT#2)
}
void SilHal::log(const char* msg) { std::fprintf(stderr, "[sil] %s\n", msg); }

void SilHal::setStatus(bool armed, bool failsafe, bool imu_valid, const float est[3],
                       float sat_shift) {
  out_.armed = armed ? 1 : 0;
  out_.failsafe = failsafe ? 1 : 0;
  out_.imu_valid = imu_valid ? 1 : 0;
  out_.roll_est = est[0];
  out_.pitch_est = est[1];
  out_.yaw_est = est[2];
  out_.sat_shift = sat_shift;
}

}  // namespace drone
