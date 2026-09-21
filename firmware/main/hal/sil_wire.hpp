// SIL/HIL wire protocol v1. Packed POD + CRC framing shared by C++ and Python.
//
// Bump rule: ANY change to a wire layout or to kProtoVersion requires a version
// bump AND a HELLO sizeof comparison on both sides (applies to SIL and HIL).
#pragma once

#include <cstdint>

namespace drone::wire {

inline constexpr uint16_t kMagic = 0x534D;  // 'M''S' little-endian
inline constexpr uint8_t kProtoVersion = 1;
inline constexpr uint32_t kSilDtUs = 1000;  // 1 kHz virtual tick
inline constexpr uint32_t kTransportTimeoutS = 30;

enum class MsgType : uint8_t {
  kHello = 1,
  kAck = 2,
  kState = 3,
  kFwOut = 4,
  kBye = 5,
  kTelem = 6,
};

// Exit codes (both sides): 0 clean, 2 protocol violation, 3 timeout, 4 plant
// numerical failure.
inline constexpr int kExitClean = 0;
inline constexpr int kExitProto = 2;
inline constexpr int kExitTimeout = 3;
inline constexpr int kExitNumerical = 4;

#pragma pack(push, 1)

struct Hdr {
  uint16_t magic;
  uint8_t ver;
  uint8_t type;
  uint16_t len;
  uint32_t seq;
  uint32_t crc;
};

struct SilImu {
  float gyro_rps[3];
  float accel_mps2[3];
  float mag_uT[3];
  float temp_c;
  uint64_t t_us;
  uint8_t valid;
};

struct SilRc {
  float roll;
  float pitch;
  float yaw;
  float throttle;
  uint8_t armed_switch;
  uint8_t frame_ok;
  uint64_t t_us;
};

struct SilPwm {
  float mot[4];
};

struct StateRc {
  SilImu imu;
  float vbat;
  uint8_t rc_proto;
  uint8_t rc_len;
  uint8_t rc_raw[32];
};

struct SilOut {
  float mot[4];
  float roll_est;
  float pitch_est;
  float yaw_est;
  uint8_t armed;
  uint8_t failsafe;
  uint8_t led_mode;
  uint8_t imu_valid;
  uint32_t tick;
  float sat_shift;
};

struct SilTelem {
  uint64_t t_us;
  float roll, pitch, yaw, yaw_rate;
  float x, y, z;
  float vx, vy, vz;
  float roll_est, pitch_est, yaw_est;
  float mot[4];
  float vbat;
  uint8_t armed, failsafe, led_mode, imu_valid;
  float sat_shift;
};

struct SilHello {
  uint32_t dt_us;
  uint32_t sizes[7];
};

#pragma pack(pop)

static_assert(sizeof(Hdr) == 14, "wire Hdr layout drift");
static_assert(sizeof(SilImu) == 49, "wire SilImu layout drift");
static_assert(sizeof(SilRc) == 26, "wire SilRc layout drift");
static_assert(sizeof(SilPwm) == 16, "wire SilPwm layout drift");
static_assert(sizeof(StateRc) == 87, "wire StateRc layout drift");
static_assert(sizeof(SilOut) == 40, "wire SilOut layout drift");
static_assert(sizeof(SilTelem) == 88, "wire SilTelem layout drift");
static_assert(sizeof(SilHello) == 32, "wire SilHello layout drift");
static_assert(sizeof(bool) == 1, "ABI assumes 1-byte bool");
static_assert(sizeof(float) == 4, "ABI assumes 32-bit float");

// HELLO compares exactly these 7 sizes (imu, rc, pwm, state, out, telem, hdr).
inline constexpr int kHelloSizes[7] = {
    static_cast<int>(sizeof(SilImu)),  static_cast<int>(sizeof(SilRc)),
    static_cast<int>(sizeof(SilPwm)),  static_cast<int>(sizeof(StateRc)),
    static_cast<int>(sizeof(SilOut)),  static_cast<int>(sizeof(SilTelem)),
    static_cast<int>(sizeof(Hdr))};

uint32_t crc32Init();
uint32_t crc32Update(uint32_t crc, const void* data, int len);
uint32_t crc32Final(uint32_t crc);
uint32_t crc32(const void* data, int len);  // matches zlib.crc32

// Serialize hdr(crc zeroed)+payload, compute CRC, store it. Returns total bytes
// written, or -1 if it does not fit.
int packMsg(uint8_t* buf, int cap, MsgType type, uint32_t seq, const void* payload, int payload_len);

// Validate magic/ver/len/type-agnostic CRC. Returns total bytes consumed and
// points `payload` at the body, or a negative error code.
int unpackMsg(const uint8_t* buf, int len, Hdr& hdr, const uint8_t*& payload);

void fillHello(SilHello& h);
bool helloMatches(const SilHello& local, const SilHello& peer);

}  // namespace drone::wire
