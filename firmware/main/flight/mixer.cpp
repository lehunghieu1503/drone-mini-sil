#include "flight/mixer.hpp"

#include <cmath>

namespace drone {

float QuadXMixer::clampf(float v, float lo, float hi) {
  if (!std::isfinite(v)) return 0.0f;
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float QuadXMixer::desaturate(float m[4]) {
  float shift = 0.0f;
  float hi = m[0];
  float lo = m[0];
  for (int i = 1; i < 4; ++i) {
    if (m[i] > hi) hi = m[i];
    if (m[i] < lo) lo = m[i];
  }
  if (hi > 1.0f) shift -= (hi - 1.0f);
  if (lo + shift < 0.0f) shift -= (lo + shift);
  for (int i = 0; i < 4; ++i) m[i] += shift;
  return shift;
}

float QuadXMixer::write(float thr, float roll, float pitch, float yaw, PwmCmd& out) {
  const float t = clampf(thr, 0.0f, 1.0f);
  const float r = clampf(roll, -1.0f, 1.0f);
  const float p = clampf(pitch, -1.0f, 1.0f);
  const float y = clampf(yaw, -1.0f, 1.0f);

  float m[4];
  m[0] = t - r + p - y;  // Mot1, GPIO0
  m[1] = t - r - p + y;  // Mot2, GPIO1
  m[2] = t + r + p + y;  // Mot3, GPIO2
  m[3] = t + r - p - y;  // Mot4, GPIO3

  const float shift = desaturate(m);
  for (int i = 0; i < 4; ++i) out.mot[i] = clampf(m[i], 0.0f, 1.0f);
  return shift;
}

}  // namespace drone
