// SIL runner entry point (transport client). The tick order lives only in
// ControlLoop; this file only wires the HAL/context and pumps the lockstep.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "flight/context.hpp"
#include "flight/control_loop.hpp"
#include "flight/estimator.hpp"
#include "flight/failsafe.hpp"
#include "flight/led.hpp"
#include "flight/mixer.hpp"
#include "flight/output.hpp"
#include "flight/pid.hpp"
#include "flight/rc_parse.hpp"
#include "hal/hal_sil.hpp"

namespace {

struct Args {
  const char* sock = nullptr;
  int timeout_s = 30;
  drone::ControlMode mode = drone::ControlMode::kOpenLoop;
  bool arm_test = false;
  bool ol_from_rc = false;
  float thr = 0.0f;
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  uint32_t reset_reason = 0;
};

bool parse(int argc, char** argv, Args& a) {
  for (int i = 1; i < argc; ++i) {
    const char* k = argv[i];
    auto next = [&](const char*& v) -> bool {
      if (i + 1 >= argc) return false;
      v = argv[++i];
      return true;
    };
    const char* v = nullptr;
    if (std::strcmp(k, "--sock") == 0) {
      if (!next(v)) return false;
      a.sock = v;
    } else if (std::strcmp(k, "--transport-timeout") == 0) {
      if (!next(v)) return false;
      a.timeout_s = std::atoi(v);
    } else if (std::strcmp(k, "--mode") == 0) {
      if (!next(v)) return false;
      if (std::strcmp(v, "open-loop") == 0) a.mode = drone::ControlMode::kOpenLoop;
      else if (std::strcmp(v, "rate") == 0) a.mode = drone::ControlMode::kRateOnly;
      else if (std::strcmp(v, "attitude") == 0) a.mode = drone::ControlMode::kAttitude;
      else return false;
    } else if (std::strcmp(k, "--arm-test") == 0) {
      a.arm_test = true;
    } else if (std::strcmp(k, "--ol-from-rc") == 0) {
      a.ol_from_rc = true;
    } else if (std::strcmp(k, "--thr") == 0) {
      if (!next(v)) return false;
      a.thr = static_cast<float>(std::atof(v));
    } else if (std::strcmp(k, "--roll") == 0) {
      if (!next(v)) return false;
      a.roll = static_cast<float>(std::atof(v));
    } else if (std::strcmp(k, "--pitch") == 0) {
      if (!next(v)) return false;
      a.pitch = static_cast<float>(std::atof(v));
    } else if (std::strcmp(k, "--yaw") == 0) {
      if (!next(v)) return false;
      a.yaw = static_cast<float>(std::atof(v));
    } else if (std::strcmp(k, "--reset-reason") == 0) {
      if (!next(v)) return false;
      a.reset_reason = static_cast<uint32_t>(std::atoi(v));
    } else {
      return false;
    }
  }
  if (a.timeout_s <= 0) return false;  // 0 would mean "no timeout" on Linux
  return a.sock != nullptr;
}

}  // namespace

int main(int argc, char** argv) {
  Args a;
  if (!parse(argc, argv, a)) {
    std::fprintf(stderr,
                 "usage: sil_runner --sock NAME [--mode open-loop|rate|attitude] "
                 "[--arm-test] [--ol-from-rc] [--thr F] [--roll F] [--pitch F] [--yaw F] "
                 "[--reset-reason N] [--transport-timeout S]\n");
    return 2;
  }

  drone::SilHal hal;
  if (hal.connectTo(a.sock, a.timeout_s) != drone::SilHal::IoResult::kOk) {
    std::fprintf(stderr, "sil: connect failed\n");
    return 3;
  }
  const drone::SilHal::IoResult hello = hal.helloAck();
  if (hello == drone::SilHal::IoResult::kTimeout) {
    std::fprintf(stderr, "sil: HELLO timed out\n");
    return 3;
  }
  if (hello != drone::SilHal::IoResult::kOk) {
    std::fprintf(stderr, "sil: HELLO failed (size/version mismatch)\n");
    return 2;
  }

  static drone::QuadXMixer mixer;
  static drone::SbusParser rc;
  static drone::ComplementaryEstimator est;
  static drone::FailsafeFsm fs;
  static drone::LedFsm led;
  static drone::RateController rate(mixer);
  static drone::AttitudeController att;
  static drone::OutputStage out;

  drone::FlightContext ctx{hal, mixer, rc, est, fs, led, rate, att, out};
  ctx.mode = a.mode;
  ctx.ol_from_rc = a.ol_from_rc;
  ctx.ol_thr = a.thr;
  ctx.ol_roll = a.roll;
  ctx.ol_pitch = a.pitch;
  ctx.ol_yaw = a.yaw;

  fs.boot(a.reset_reason);

  drone::ControlLoop loop;
  for (;;) {
    const drone::SilHal::IoResult r = hal.recvState();
    if (r == drone::SilHal::IoResult::kClosed) break;  // clean end (BYE / EOF)
    if (r == drone::SilHal::IoResult::kProto) {
      std::fprintf(stderr, "sil: protocol violation at tick %u\n", ctx.tick);
      return 2;
    }
    if (r == drone::SilHal::IoResult::kTimeout) {
      std::fprintf(stderr, "sil: transport timeout at tick %u\n", ctx.tick);
      return 3;
    }

    loop.tick(ctx, a.arm_test);

    float est_att[3];
    ctx.est.getAttitude(est_att);
    hal.setStatus(ctx.fs.armed(), ctx.fs.active(), ctx.est.imuValid(), est_att, ctx.sat_shift,
                  ctx.tick);

    const drone::SilHal::IoResult s = hal.sendOut();
    if (s != drone::SilHal::IoResult::kOk) {
      std::fprintf(stderr, "sil: send failed at tick %u\n", ctx.tick);
      return (s == drone::SilHal::IoResult::kTimeout) ? 3 : 2;
    }
  }
  hal.sendBye();
  hal.close();
  return 0;
}
