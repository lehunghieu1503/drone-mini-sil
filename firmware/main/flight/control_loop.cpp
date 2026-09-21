#include "flight/control_loop.hpp"

#include <cmath>

#include "flight/flight_params.hpp"
#include "flight/pid.hpp"

namespace drone {

void ControlLoop::tick(FlightContext& c, bool arm_test) const {
  // --- sensors -------------------------------------------------------------
  ImuSample imu{};
  if (!c.hal.imuRead(imu)) imu.valid = false;

  uint8_t raw[64];
  const int n = c.hal.rcRawRead(raw, static_cast<int>(sizeof(raw)));
  RcSample rc{};
  const bool rc_ok = (n > 0) && c.rc.feed(raw, n, rc);
  rc.frame_ok = rc_ok;
  rc.t_us = c.hal.nowUs();  // HAL is the single timestamp source (RT#8)

  float vbat = NAN;
  if (!c.hal.vbatRead(vbat)) vbat = NAN;

  // --- safety --------------------------------------------------------------
  c.fs.update(rc, imu, vbat, c.hal.nowUs(), c.est.calibrated(), arm_test);

  // --- estimation ----------------------------------------------------------
  const float dt = kControlDtS;
  ImuSample imu_ctl = imu;
  if (c.est.calibrated()) {  // remove boot ZRO bias before control/fusion
    float bias[3];
    c.est.gyroBias(bias);
    for (int i = 0; i < 3; ++i) imu_ctl.gyro_rps[i] -= bias[i];
  }
  if (c.fs.armed()) {
    c.est.update(imu_ctl, dt);
  } else {
    c.est.calibrateUpdate(imu);
  }

  // --- control -------------------------------------------------------------
  PwmCmd pwm{};
  float shift = 0.0f;
  switch (c.mode) {
    case ControlMode::kOpenLoop:
      if (c.ol_from_rc) {
        shift = c.mixer.write(rc.throttle, rc.roll, rc.pitch, rc.yaw, pwm);
      } else {
        shift = c.mixer.write(c.ol_thr, c.ol_roll, c.ol_pitch, c.ol_yaw, pwm);
      }
      break;
    case ControlMode::kRateOnly: {
      RateSp sp{};
      rc_stick_to_rate_sp(rc, sp);
      shift = c.rate.update(imu_ctl, sp, pwm);
      break;
    }
    case ControlMode::kAttitude: {
      float est[3];
      c.est.getAttitude(est);
      float att_sp[3];
      rc_stick_to_att_sp(rc, att_sp);
      float rate_sp[3];
      c.att.update(att_sp, est, dt, rate_sp);
      RateSp sp{rate_sp[0], rate_sp[1], rate_sp[2], rc.throttle};
      shift = c.rate.update(imu_ctl, sp, pwm);
      break;
    }
  }
  c.sat_shift = shift;
  c.tick++;

  // --- single output gate --------------------------------------------------
  c.out.apply(pwm, c.fs.armed(), c.fs.active(), c.est.imuValid(), c.hal);

  // --- status LED ----------------------------------------------------------
  if (c.fs.bootLatched()) {
    c.led.set(LedMode::kError);
  } else if (c.fs.active()) {
    c.led.set(LedMode::kFailsafe);
  } else if (c.fs.armed()) {
    c.led.set(LedMode::kArmed);
  } else if (!c.est.calibrated()) {
    c.led.set(LedMode::kCalib);
  } else {
    c.led.set(LedMode::kBoot);
  }
  c.led.update(static_cast<uint32_t>(c.hal.nowUs() / 1000u));
  c.hal.ledSet(c.led.mode());
}

}  // namespace drone
