# Drone mini — SIL/HIL stack

Flight-control core in **C++20** running over two HALs (`HwHal` on ESP32-C3,
`SilHal` against a Python lockstep plant), plus the plant/tools in Python.

**Invariant:** the controller is the only place flight logic lives. Python is
plant + runner + log only. **No PWM path bypasses `OutputStage`.**

## Layout

```
firmware/main/flight/   pure C++20 flight core (no ESP-IDF headers)
firmware/main/hal/      IHal implementations: sil / hw / mock + wire protocol
firmware/host/          extern "C" bindings for pytest (host lib only)
firmware/main/          ESP-IDF component (app_main, Kconfig)
sim/                    native SIL runner (transport client)
plant/                  Python plants (RK4 / RotorPy / MuJoCo), battery, IMU, RC, log, runner
tests/                  pytest ABI/gate/unit/SIL tests
tools/                  plot + compare overlay + sim-to-sim
docs/                   SIL_STACK.md copy + measurements registry
```

## Plants

`plant/vehicle.py::make_plant` selects the engine behind the same `Plant`
interface (`step` / `sense` / `finite` / `pos` / `vel` / `quat` / `omega` /
`motors`). Firmware and wire protocol are unchanged.

| `--plant` | Engine | Status |
|---|---|---|
| `rk4` (default) | in-repo 6-DOF RK4 | authoritative, bit-for-bit deterministic |
| `rotorpy` | RotorPy 3.0.0 | opt-in reference (own rotor geometry/params) |
| `mujoco` | Menagerie Crazyflie 2 (MIT) | opt-in high-fidelity (contacts/aero) |

The MuJoCo adapter (`plant/vehicle_mujoco.py`) drives the vendored Menagerie
model through `mjData.xfrc_applied` with the same propulsion model as RK4, and
overwrites the body mass/inertia with `drone_mini_params` by default so it is a
drop-in engine swap. MuJoCo is optional — install it with `make venv-visual`.
`make sim2sim` compares engines under an identical duty profile.

## Build & test

```sh
make venv        # uv venv (py3.13) + pinned deps
make venv-visual # optional: MuJoCo for --plant mujoco (not needed for gate)
make host        # native host lib (no ESP-IDF required)
make sil         # sim/sil_runner
make test        # unit tests only (excludes slow)
make gate        # unit + slow e2e criteria (the headline gate)
make sim2sim     # compare RK4 / MuJoCo / RotorPy under one duty profile
```

Chip build (`make fw`) requires ESP-IDF **v6.0.x**; P9/P10 are hardware-gated.

## ABI contract (frozen, v1)

Wire structs are `#pragma pack(1)` with `static_assert` on both sides:
header 14 B, IMU 49 B, RC 26 B, PWM 16 B, STATE 87 B, FW_OUT 40 B, TELEM 88 B.
Any layout change **must** bump `kProtoVersion` and is rejected by the HELLO
size check (SIL and HIL alike).

## Security / safety notes

- `OutputStage` is the single PWM gate: `armed ∧ ¬failsafe ∧ imu_valid`; any
  violation forces 0 and calls `IHal::pwmOff()`.
- The SIL transport uses a random abstract-namespace UDS name; the plant server
  verifies `SO_PEERCRED` UID and both sides reject non-monotonic `seq`/`t_us`.
  Trusted dev host only; no network port is opened.
- Logs never contain absolute paths, usernames, or hostnames, and are not
  committed (`.gitignore`).
