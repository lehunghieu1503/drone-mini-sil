// One control tick, shared by the chip and the host SIL runner.
#pragma once

#include "flight/context.hpp"

namespace drone {

class ControlLoop {
 public:
  // Tick order (spec §7):
  //   imu -> rc raw -> parse -> vbat -> failsafe -> estimator -> attitude ->
  //   rate -> mixer -> OutputStage -> pwm -> led
  void tick(FlightContext& c, bool arm_test) const;
};

}  // namespace drone
