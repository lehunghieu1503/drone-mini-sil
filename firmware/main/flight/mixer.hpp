// Quad-X mixer. The only place in the firmware that knows the GPIO mapping:
// PwmCmd::mot[0..3] = Mot1..Mot4 = GPIO0..3.
//
// Sign table (viewed from above, nose +X body; FRD). ASSUMED(A1) -- the real
// PCB motor positions must be verified with a spin bench before HIL/flight.
//
//   input   Mot1(GPIO0) Mot2(GPIO1) Mot3(GPIO2) Mot4(GPIO3)
//   roll+        -           -           +           +
//   pitch+       +           -           +           -   (pitch+ = nose down)
//   yaw+         -           +           +           -
//
// Desaturation contract (single definition): write() returns `shift`, the
// delta already applied to every channel. shift > 0 means the low side was
// pushed up, shift < 0 means the high side was pulled down. Callers (the rate
// anti-windup) map sign -> sat_neg / sat_pos.
#pragma once

#include "flight/ports.hpp"

namespace drone {

class QuadXMixer final : public IMixer {
 public:
  float write(float thr, float roll, float pitch, float yaw, PwmCmd& out) override;

 private:
  static float clampf(float v, float lo, float hi);
  static float desaturate(float m[4]);
};

}  // namespace drone
