// ESP32-C3 application entry (P9 fills in the drivers). app_main must be
// extern "C" when defined in a .cpp (ESP-IDF C++ guide).
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "flight/context.hpp"
#include "flight/control_loop.hpp"
#include "hal/hal_hw.hpp"

extern "C" void app_main(void) {
  static drone::HwHal hal;
  static drone::QuadXMixer mixer;
  static drone::SbusParser rc;
  static drone::ComplementaryEstimator est;
  static drone::FailsafeFsm fs;
  static drone::LedFsm led;
  static drone::RateController rate(mixer);
  static drone::AttitudeController att;
  static drone::OutputStage out;

  static drone::FlightContext ctx{hal, mixer, rc, est, fs, led, rate, att, out};
  static drone::ControlLoop loop;

  fs.boot(static_cast<uint32_t>(esp_reset_reason()));
  ctx.mode = drone::ControlMode::kAttitude;

  if (!hal.init()) {
    // Fail closed: do not enter the loop with an uninitialised HAL.
    for (;;) {
      hal.pwmOff();
      vTaskDelay(pdMS_TO_TICKS(100));
    }
  }

  for (;;) {
    loop.tick(ctx, false);
    vTaskDelay(1);  // 1 kHz tick at CONFIG_FREERTOS_HZ=1000
  }
}
