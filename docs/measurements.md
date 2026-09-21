# Measurements & assumptions registry

Every unmeasured value used by the stack is an **assumption**, tracked here and
in `plant/drone_mini_params.py::ASSUMPTIONS`. Assumed values are never used as
the oracle for a test that validates the same assumption.

## Environment (Step 0 — 2026-09-22)

| Item | Value |
|------|-------|
| Python | CPython 3.13.14 (uv venv `.venv`) |
| numpy | 2.2.6 |
| matplotlib | 3.10.9 |
| rotorpy | 3.0.0 |
| pytest | 9.1.1 |
| scipy | 1.18.1 (transitive, rotorpy) |
| g++ | 11.4.0 (C++20) |
| cmake | 3.22.1 |
| `socket.SOCK_SEQPACKET` | available (value 5) → primary UDS transport confirmed |
| ESP-IDF | not installed (chip build deferred to P9) |
| License | MIT; no vendored GPL/DMP code |

## Assumptions A1–A10

Authoritative copy is the `ASSUMPTIONS` dict. Summary:

| # | Assumption | Working value | Gate |
|---|------------|---------------|------|
| A1 | motor positions/spin | `ROTOR_MAP` | **spin bench before HIL/flight** |
| A2 | arm length | 0.043 m | PCB measure (Tier C) |
| A3 | mass | 0.03 kg | **before §9.1/§9.4** |
| A4 | `tau_m` | 0.03 s | thrust stand step |
| A5 | `k_t`,`k_q`,`omega_max` | 4.7e-8, 8e-10, 2500 | **before §9.1/§9.4** |
| A6 | battery `R_int` | 0.040 Ω | load step |
| A7 | gyro ZRO | ±5 dps | Allan variance |
| A8 | RC protocol on J6 | SBUS | logic-analyzer sniff |
| A9 | IMU mounting | R_BS = I | PCB measure before Tier C |
| A10 | LDO dropout | 220 mV | scope 3V3 vs V_BAT |

## Gate status (software, Tier A)

| Criterion | Status | Evidence |
|-----------|--------|----------|
| §9.2 mixer sign | software-verified, hardware-gated by A1 | `tests/test_mixer.py` T2.1–T2.11 |
| §9.5 log schema / overlay | pass — common 23-column schema, GATE PASS on identical runs | `tools/compare_logs.py`, T8.x |
| §9.1/§9.4 hover | pass **under assumptions A3/A5** — level (<0.02°), yaw drift <0.1°/10 s | `tests/test_hover_sil.py` T6.7–T6.12 |
| §9.3 failsafe | pass — RC-loss latches within 100 ms virtual (+1 tick), outputs 0 | `tests/test_power_sil.py` T7.15 |
| Safety invariant `armed=0 ⇒ 0` | pass for all flag combinations | `tests/test_output_gate.py` T1.8 |

## Tuning results (2026-09-22)

- Rate loop gains (`flight_params.hpp`): roll/pitch `Kp=0.15, Ki=2.4, Kd=0.006`;
  yaw `Kp=0.16, Ki=2.0, Kd=0.004`; `i_limit=0.06`, `out_limit=0.40`, `d_lpf=100 Hz`.
- Step response settles < 0.3 s with < 20 % overshoot for roll/pitch/yaw at
  `tau_m ∈ {0.02, 0.03, 0.05}` s (T5.7–T5.9). Clean IMU isolates the control law;
  noise rejection is checked separately (T5.11).
- Gyro boot-bias calibration is applied in the control path: `ControlLoop`
  subtracts `IEstimator::gyroBias()` once calibrated. Without it, the ±5 dps ZRO
  produced ~20° yaw drift over 10 s.

## P4 RotorPy (conditional)

- Spike: RotorPy 3.0.0 API is usable — `Multirotor.step(state, {'cmd_motor_speeds'}, dt)`.
- Adapter at `plant/vehicle_rotorpy.py` implements the `Plant` interface and is
  smoke-tested (`tests/test_rotorpy_adapter.py`). It is **opt-in**
  (`--plant rotorpy`); the **RK4 path is authoritative** and the only one with the
  bit-for-bit determinism guarantee (D20).

## Hardware notes

- **Do not add pull-ups on GPIO0–3** (R2 §7.4): the pull-up + 10 kΩ gate
  pulldown forms a divider that can partially turn a MOSFET on.
- V_BAT monitor (INA219/ADS1115) on I2C0 is mandatory in the BOM (D11); the
  internal ADC path is unavailable (ADC1 pins consumed, ADC2 broken on C3).
- HIL-1 bring-up targets the Super Mini dev board; HIL-2/flight targets the
  custom PCB. Every power/brownout conclusion must name the board (D22).
