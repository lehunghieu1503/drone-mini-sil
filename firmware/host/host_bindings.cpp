// extern "C" shims for pytest (host lib only; never on the chip).
//
// Only primitive/array arguments cross this seam so ctypes never has to match
// a C++ struct layout by hand. Static module instances hold the state.
#include <cmath>
#include <cstring>

#include "flight/estimator.hpp"
#include "flight/failsafe.hpp"
#include "flight/flight_params.hpp"
#include "flight/led.hpp"
#include "flight/mixer.hpp"
#include "flight/output.hpp"
#include "flight/pid.hpp"
#include "flight/rc_parse.hpp"
#include "hal/hal_mock.hpp"
#include "hal/sil_wire.hpp"

namespace {
drone::MockHal g_mock;
drone::OutputStage g_out;
drone::QuadXMixer g_mixer;
drone::Pid g_pid;
drone::RateController g_rate(g_mixer);
drone::ComplementaryEstimator g_est;
drone::FailsafeFsm g_fs;
drone::LedFsm g_led;
drone::SbusParser g_sbus;
drone::RcSample g_rc{};
}  // namespace

extern "C" {

// --- MockHal ---------------------------------------------------------------
void mock_reset(void) { g_mock.resetLog(); }
void mock_set_now_us(uint64_t v) { g_mock.setNowUs(v); }
void mock_set_imu(const float* gyro3, const float* accel3, uint64_t t_us, int valid) {
  drone::ImuSample s{};
  for (int i = 0; i < 3; ++i) {
    s.gyro_rps[i] = gyro3[i];
    s.accel_mps2[i] = accel3[i];
  }
  s.t_us = t_us;
  s.valid = valid != 0;
  g_mock.setImu(s);
}
void mock_set_imu_invalid(void) { g_mock.setImuInvalid(); }
void mock_set_rc_raw(const uint8_t* data, int len) { g_mock.setRcRaw(data, len); }
void mock_set_vbat(float v, int ok) { g_mock.setVbat(v, ok != 0); }
int mock_pwm_writes(void) { return g_mock.log().pwm_writes; }
int mock_pwm_offs(void) { return g_mock.log().pwm_offs; }
int mock_led_sets(void) { return g_mock.log().led_sets; }
void mock_last_pwm(float* out4) {
  for (int i = 0; i < 4; ++i) out4[i] = g_mock.log().last_pwm.mot[i];
}
int mock_last_led(void) { return static_cast<int>(g_mock.log().last_led); }

// --- OutputStage -----------------------------------------------------------
int output_stage_apply(const float* in4, int armed, int failsafe, int imu_valid) {
  drone::PwmCmd p{};
  for (int i = 0; i < 4; ++i) p.mot[i] = in4[i];
  return g_out.apply(p, armed != 0, failsafe != 0, imu_valid != 0, g_mock) ? 1 : 0;
}

// --- Mixer -----------------------------------------------------------------
float mixer_write(float thr, float roll, float pitch, float yaw, float* out4) {
  drone::PwmCmd p{};
  const drone::MixOut r = g_mixer.write(thr, roll, pitch, yaw, p);
  for (int i = 0; i < 4; ++i) out4[i] = p.mot[i];
  return r.shift;
}
int mixer_write_flags(float thr, float roll, float pitch, float yaw, float* out4) {
  drone::PwmCmd p{};
  const drone::MixOut r = g_mixer.write(thr, roll, pitch, yaw, p);
  for (int i = 0; i < 4; ++i) out4[i] = p.mot[i];
  return (r.sat_pos ? 1 : 0) | (r.sat_neg ? 2 : 0);
}

// --- SBUS parser -----------------------------------------------------------
void rc_parse_reset(void) { g_sbus.reset(); }
int rc_parse_feed(const uint8_t* data, int len) {
  return g_sbus.feed(data, len, g_rc) ? 1 : 0;
}
float rc_roll(void) { return g_rc.roll; }
float rc_pitch(void) { return g_rc.pitch; }
float rc_yaw(void) { return g_rc.yaw; }
float rc_throttle(void) { return g_rc.throttle; }
int rc_armed_switch(void) { return g_rc.armed_switch ? 1 : 0; }
int rc_frame_ok(void) { return g_rc.frame_ok ? 1 : 0; }
int rc_rx_failsafe(void) { return g_rc.rx_failsafe ? 1 : 0; }
int rc_frame_lost(void) { return g_rc.frame_lost ? 1 : 0; }

// --- PID -------------------------------------------------------------------
void pid_set_gains(float kp, float ki, float kd, float i_limit, float out_limit, float d_lpf_hz) {
  g_pid.setGains(kp, ki, kd, i_limit, out_limit, d_lpf_hz);
}
void pid_reset(void) { g_pid.reset(); }
float pid_step(float sp, float meas, float dt, int sat_pos, int sat_neg) {
  return g_pid.step(sp, meas, dt, sat_pos != 0, sat_neg != 0);
}
// Real function from libflight (not a host-only shim); yaw stick -> yaw rate.
float stick_yaw_to_rate(float stick) { return drone::stick_yaw_to_rate(stick); }

// --- Rate controller -------------------------------------------------------
void rate_reset(void) { g_rate.reset(); }
float rate_update(const float* gyro3, const float* sp3, float throttle, float* out4) {
  drone::ImuSample imu{};
  for (int i = 0; i < 3; ++i) imu.gyro_rps[i] = gyro3[i];
  imu.valid = true;
  drone::RateSp sp{sp3[0], sp3[1], sp3[2], throttle};
  drone::PwmCmd p{};
  const float shift = g_rate.update(imu, sp, p);
  for (int i = 0; i < 4; ++i) out4[i] = p.mot[i];
  return shift;
}
int rate_sat_flags(int* out2) {
  out2[0] = g_rate.lastSatPos() ? 1 : 0;
  out2[1] = g_rate.lastSatNeg() ? 1 : 0;
  return 2;
}

// --- Flight params ---------------------------------------------------------
int flight_params_get(float* out) {
  int k = 0;
  for (int i = 0; i < 3; ++i) out[k++] = drone::kFlightParams.rate_kp[i];
  for (int i = 0; i < 3; ++i) out[k++] = drone::kFlightParams.rate_ki[i];
  for (int i = 0; i < 3; ++i) out[k++] = drone::kFlightParams.rate_kd[i];
  out[k++] = drone::kFlightParams.rate_i_limit;
  out[k++] = drone::kFlightParams.rate_out_limit;
  out[k++] = drone::kFlightParams.rate_d_lpf_hz;
  for (int i = 0; i < 3; ++i) out[k++] = drone::kFlightParams.att_kp[i];
  out[k++] = drone::kFlightParams.att_rate_limit;
  return k;
}

// --- Estimator -------------------------------------------------------------
void estimator_reset(void) { g_est.reset(); }
void estimator_calibrate(const float* gyro3, const float* accel3, uint64_t t_us) {
  drone::ImuSample s{};
  for (int i = 0; i < 3; ++i) {
    s.gyro_rps[i] = gyro3[i];
    s.accel_mps2[i] = accel3[i];
  }
  s.t_us = t_us;
  s.valid = true;
  g_est.calibrateUpdate(s);
}
namespace {
void fillCorrected(drone::ImuSample& s, const float* gyro3, const float* accel3, int valid) {
  float bias[3] = {0.0f, 0.0f, 0.0f};
  if (g_est.calibrated()) g_est.gyroBias(bias);  // mirrors ControlLoop
  for (int i = 0; i < 3; ++i) {
    s.gyro_rps[i] = gyro3[i] - bias[i];
    s.accel_mps2[i] = accel3[i];
  }
  s.valid = valid != 0;
}
}  // namespace

void estimator_update(const float* gyro3, const float* accel3, float dt) {
  drone::ImuSample s{};
  fillCorrected(s, gyro3, accel3, 1);
  g_est.update(s, dt);
}
void estimator_update_v(const float* gyro3, const float* accel3, float dt, int valid) {
  drone::ImuSample s{};
  fillCorrected(s, gyro3, accel3, valid);
  g_est.update(s, dt);
}
void estimator_attitude(float* out3) {
  float a[3];
  g_est.getAttitude(a);
  out3[0] = a[0];
  out3[1] = a[1];
  out3[2] = a[2];
}
void estimator_bias(float* out3) {
  const float* b = g_est.bias();
  for (int i = 0; i < 3; ++i) out3[i] = b[i];
}
int estimator_imu_valid(void) { return g_est.imuValid() ? 1 : 0; }
int estimator_calibrated(void) { return g_est.calibrated() ? 1 : 0; }
int estimator_calibrating(void) { return g_est.calibrating() ? 1 : 0; }

// --- Failsafe --------------------------------------------------------------
void failsafe_reset(void) { g_fs.reset(); }
void failsafe_boot(uint32_t reset_reason) { g_fs.boot(reset_reason); }
void failsafe_update(const float* rc4, int rc_flags, uint64_t rc_t_us, int imu_valid,
                     const float* gyro3, const float* accel3, float vbat, uint64_t now_us,
                     int calibrated, int arm_test) {
  drone::RcSample rc{};
  rc.roll = rc4[0];
  rc.pitch = rc4[1];
  rc.yaw = rc4[2];
  rc.throttle = rc4[3];
  rc.armed_switch = (rc_flags & 1) != 0;
  rc.frame_ok = (rc_flags & 2) != 0;
  rc.rx_failsafe = (rc_flags & 4) != 0;
  rc.frame_lost = (rc_flags & 8) != 0;
  rc.t_us = rc_t_us;
  drone::ImuSample imu{};
  for (int i = 0; i < 3; ++i) {
    imu.gyro_rps[i] = gyro3[i];
    imu.accel_mps2[i] = accel3[i];
  }
  imu.valid = imu_valid != 0;
  g_fs.update(rc, imu, vbat, now_us, calibrated != 0, arm_test != 0);
}
int failsafe_armed(void) { return g_fs.armed() ? 1 : 0; }
int failsafe_active(void) { return g_fs.active() ? 1 : 0; }
int failsafe_output_zero(void) { return g_fs.outputZero() ? 1 : 0; }
int failsafe_reason(void) { return static_cast<int>(g_fs.reason()); }
int failsafe_vbat_nan(void) { return g_fs.vbatNan() ? 1 : 0; }
int failsafe_boot_latched(void) { return g_fs.bootLatched() ? 1 : 0; }
int failsafe_good_frames(void) { return g_fs.goodFrames(); }

// --- LED -------------------------------------------------------------------
void led_set(int mode) { g_led.set(static_cast<drone::LedMode>(mode)); }
void led_update(uint32_t t_ms) { g_led.update(t_ms); }
int led_mode(void) { return static_cast<int>(g_led.mode()); }
int led_on(void) { return g_led.on() ? 1 : 0; }

// --- Wire ------------------------------------------------------------------
uint32_t wire_crc32(const uint8_t* data, int len) { return drone::wire::crc32(data, len); }
int wire_hdr_size(void) { return static_cast<int>(sizeof(drone::wire::Hdr)); }
int wire_hello_fill(uint32_t* out8) {
  drone::wire::SilHello h{};
  drone::wire::fillHello(h);
  out8[0] = h.dt_us;
  for (int i = 0; i < 7; ++i) out8[1 + i] = h.sizes[i];
  return 8;
}
}  // extern "C"
