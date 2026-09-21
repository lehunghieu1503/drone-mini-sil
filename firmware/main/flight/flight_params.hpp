// Single source of truth for controller gains. POD, shared with the test
// harness through the `flight_params_get` host binding.
#pragma once

#include <cstdint>

namespace drone {

inline constexpr float kControlDtS = 0.001f;  // 1 kHz control loop

struct flight_params_t {
  float rate_kp[3];      // roll, pitch, yaw  [duty / (rad/s)]
  float rate_ki[3];      // [duty / rad]
  float rate_kd[3];      // [duty / (rad/s^2)]
  float rate_i_limit;    // integral clamp magnitude
  float rate_out_limit;  // per-axis output clamp
  float rate_d_lpf_hz;   // D-term low-pass cutoff
  float att_kp[3];       // attitude P -> rate setpoint [1/s]
  float att_rate_limit;  // rad/s clamp on attitude output
};

// Initial gains; tuned in P5/P6. Kept as constexpr so there is one entity.
inline constexpr flight_params_t kFlightParams = {
    /*rate_kp=*/{0.15f, 0.15f, 0.16f},
    /*rate_ki=*/{2.40f, 2.40f, 2.00f},
    /*rate_kd=*/{0.006f, 0.006f, 0.004f},
    /*rate_i_limit=*/0.06f,
    /*rate_out_limit=*/0.40f,
    /*rate_d_lpf_hz=*/100.0f,
    /*att_kp=*/{6.0f, 6.0f, 0.0f},
    /*att_rate_limit=*/5.236f,  // 300 dps
};

}  // namespace drone
