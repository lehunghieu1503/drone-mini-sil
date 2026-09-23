// Frozen HAL + module port interfaces and POD types (C++20, no ESP-IDF here).
//
// Conventions: body frame FRD (x forward, y right, z down); gyro rad/s; accel
// m/s^2; motor duty 0..1; attitude rad; rates rad/s. Wire PODs are packed and
// static_asserted in hal/sil_wire.hpp -- nothing C++-specific crosses an ABI.
#pragma once

#include <cstdint>

namespace drone {

struct ImuSample {
  float gyro_rps[3];    // FRD, rad/s
  float accel_mps2[3];  // FRD, m/s^2
  float mag_uT[3];      // unused in v0
  float temp_c;
  uint64_t t_us;
  bool valid;
};

struct RcSample {
  float roll;      // [-1, 1]
  float pitch;     // [-1, 1]
  float yaw;       // [-1, 1]
  float throttle;  // [0, 1]
  bool armed_switch;
  bool frame_ok;
  bool rx_failsafe;  // SBUS failsafe flag (1<<3)
  bool frame_lost;   // SBUS frame-lost flag (1<<2)
  uint64_t t_us;
};

struct PwmCmd {
  float mot[4];  // duty [0, 1], mot[0..3] = Mot1..Mot4 = GPIO0..3
};

// Mixer result: the desaturation delta plus the pre-clip saturation flags. The
// flags are the source of the rate anti-windup, so they must survive even when
// the idle path clips the collective instead of desaturating (D5/D6).
struct MixOut {
  float shift;    // delta already applied to every channel (0 on the idle path)
  bool sat_pos;   // a raw channel exceeded 1 this tick
  bool sat_neg;   // a raw channel went below 0 this tick
};

struct RateSp {
  float roll;      // rad/s
  float pitch;     // rad/s
  float yaw;       // rad/s
  float throttle;  // [0, 1] collective passed to the mixer
};

enum class LedMode : uint8_t { kBoot = 0, kCalib = 1, kArmed = 2, kError = 3, kFailsafe = 4 };

enum class RcProto : uint8_t { kSbus = 0 };

// The single port that touches devices (10 operations).
class IHal {
 public:
  virtual ~IHal() = default;
  virtual bool init() = 0;
  virtual uint64_t nowUs() = 0;
  virtual void pwmWrite(const PwmCmd& out) = 0;
  virtual void pwmOff() = 0;
  virtual bool imuRead(ImuSample& out) = 0;
  virtual int rcRawRead(uint8_t* buf, int cap) = 0;  // bytes read, < 0 on error
  virtual bool rcRead(RcSample& out) = 0;
  virtual void ledSet(LedMode mode) = 0;
  virtual bool vbatRead(float& volts) = 0;
  virtual void log(const char* msg) = 0;
};

class IMixer {
 public:
  virtual ~IMixer() = default;
  // Returns the desaturation shift and the raw-channel saturation flags.
  virtual MixOut write(float thr, float roll, float pitch, float yaw, PwmCmd& out) = 0;
};

class IPid {
 public:
  virtual ~IPid() = default;
  virtual void reset() = 0;
  virtual float step(float sp, float meas, float dt, bool sat_pos, bool sat_neg) = 0;
};

class IRateController {
 public:
  virtual ~IRateController() = default;
  virtual void reset() = 0;
  virtual float update(const ImuSample& imu, const RateSp& sp, PwmCmd& out) = 0;
};

class IAttitudeController {
 public:
  virtual ~IAttitudeController() = default;
  virtual void reset() = 0;
  virtual void update(const float sp[3], const float est[3], float dt, float out[3]) = 0;
};

class IEstimator {
 public:
  virtual ~IEstimator() = default;
  virtual void reset() = 0;
  virtual void calibrateUpdate(const ImuSample& imu) = 0;
  virtual void update(const ImuSample& imu, float dt) = 0;
  virtual void getAttitude(float out[3]) const = 0;
  virtual bool imuValid() const = 0;
  virtual bool calibrating() const = 0;
  virtual bool calibrated() const = 0;
  virtual void gyroBias(float out[3]) const = 0;
};

class IFailsafe {
 public:
  virtual ~IFailsafe() = default;
  virtual void reset() = 0;
  virtual void boot(uint32_t reset_reason) = 0;
  virtual void update(const RcSample& rc, const ImuSample& imu, float vbat,
                      uint64_t now_us, bool calibrated, bool arm_test) = 0;
  virtual bool armed() const = 0;
  virtual bool active() const = 0;       // failsafe latched
  virtual bool outputZero() const = 0;
  virtual uint32_t reason() const = 0;   // reason bitmask
  virtual bool bootLatched() const = 0;
  virtual bool vbatNan() const = 0;
};

class ILed {
 public:
  virtual ~ILed() = default;
  virtual void set(LedMode mode) = 0;
  virtual void update(uint32_t t_ms) = 0;
  virtual LedMode mode() const = 0;
};

class IRcParser {
 public:
  virtual ~IRcParser() = default;
  virtual void reset() = 0;
  // Feed raw transport bytes; returns true when a full valid frame decoded.
  virtual bool feed(const uint8_t* data, int len, RcSample& out) = 0;
};

class IOutputStage {
 public:
  virtual ~IOutputStage() = default;
  // Sole owner of the PWM output path. Returns true when channels were written.
  virtual bool apply(const PwmCmd& in, bool armed, bool failsafe, bool imu_valid, IHal& hal) = 0;
};

}  // namespace drone
