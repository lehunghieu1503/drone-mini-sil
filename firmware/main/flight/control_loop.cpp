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
  if (n > 0 && c.rc.feed(raw, n, rc)) {
    // Only a decoded frame owns a new timestamp and replaces the hold (D1).
    rc.t_us = c.hal.nowUs();
    c.rc_hold = rc;
    c.have_rc = true;
  } else if (c.have_rc) {
    // Silent / partial / error tick: replay the last decoded frame unchanged so
    // the switch stays held and the 100 ms timeout is the only cut (D1).
    rc = c.rc_hold;
  }

  float vbat = NAN;
  if (!c.hal.vbatRead(vbat)) vbat = NAN;

  // --- safety --------------------------------------------------------------
  c.fs.update(rc, imu, vbat, c.hal.nowUs(), c.est.calibrated(), arm_test);

  // --- estimation ----------------------------------------------------------
  // Calibrate until done, then fuse every tick (armed or not) so the attitude
  // is real before the arm edge and stays tracked after a disarm (D7).
  const float dt = kControlDtS;
  ImuSample imu_ctl = imu;
  if (c.est.calibrated()) {  // remove boot ZRO bias before control/fusion
    float bias[3];
    c.est.gyroBias(bias);
    for (int i = 0; i < 3; ++i) imu_ctl.gyro_rps[i] -= bias[i];
    c.est.update(imu_ctl, dt);
  } else {
    c.est.calibrateUpdate(imu);
  }

  // --- arm edge ------------------------------------------------------------
  // Clear stale integrator state before the arm tick's mixer so re-arming does
  // not replay a wound-up command from before the disarm (D7).
  if (c.fs.armed() && !c.was_armed) c.rate.reset();

  // --- control -------------------------------------------------------------
  PwmCmd pwm{};
  float shift = 0.0f;
  switch (c.mode) {
    case ControlMode::kOpenLoop:
      if (c.ol_from_rc) {
        shift = c.mixer.write(rc.throttle, rc.roll, rc.pitch, rc.yaw, pwm).shift;
      } else {
        shift = c.mixer.write(c.ol_thr, c.ol_roll, c.ol_pitch, c.ol_yaw, pwm).shift;
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
      rate_sp[2] = stick_yaw_to_rate(rc.yaw);  // yaw is a rate command (D8)
      RateSp sp{rate_sp[0], rate_sp[1], rate_sp[2], rc.throttle};
      shift = c.rate.update(imu_ctl, sp, pwm);
      break;
    }
  }
  c.sat_shift = shift;
  c.tick++;

  // --- single output gate --------------------------------------------------
  c.out.apply(pwm, c.fs.armed(), c.fs.active(), c.est.imuValid(), c.hal);
  c.was_armed = c.fs.armed();

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
