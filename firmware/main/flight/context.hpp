// Injection context: references to the module ports, built once at boot.
#pragma once

#include "flight/ports.hpp"

namespace drone {

enum class ControlMode : uint8_t {
  kOpenLoop = 0,  // P3: pilot command drives the mixer directly
  kRateOnly = 1,  // P5: stick -> rate setpoint -> rate loop
  kAttitude = 2,  // P6: stick -> attitude cascade -> rate loop
};

struct FlightContext {
  IHal& hal;
  IMixer& mixer;
  IRcParser& rc;
  IEstimator& est;
  IFailsafe& fs;
  ILed& led;
  IRateController& rate;
  IAttitudeController& att;
  IOutputStage& out;

  ControlMode mode = ControlMode::kOpenLoop;
  bool ol_from_rc = false;  // open loop: take pilot sticks from RC instead of ol_*
  float ol_thr = 0.0f;
  float ol_roll = 0.0f;
  float ol_pitch = 0.0f;
  float ol_yaw = 0.0f;

  // Outputs written by ControlLoop::tick (for telemetry / logging).
  float sat_shift = 0.0f;
  uint32_t tick = 0;
};

}  // namespace drone
