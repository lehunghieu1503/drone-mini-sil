---
title: "Visual layer & sim-to-real upgrade — drone-mini SIL"
type: research-report
status: final
created: 2026-09-23
updated: 2026-09-23
tags: [uav, sil, hil, sim-to-real, visualization, mujoco, rerun, domain-randomization, system-identification]
scope: "Cách nâng cấp repo drone-mini để có visual 3D + tiến tới sim-to-real"
sources: 5
---

# Research Report: Visual layer & sim-to-real upgrade cho `drone-mini`

> Timestamp: **2026-09-23 05:18 (+07)**. Repo: `SIL/drone-mini`. Bối cảnh: SIL/HIL stack
> C++20 + plant Python lockstep, phase 1–8 xong, phase 9 (HIL)/10 (bay) pending.
> Mục tiêu người dùng: **thêm visual, hướng tới sim-to-real**.

## TOC

- [Executive Summary](#executive-summary)
- [Research Methodology](#research-methodology)
- [Key Findings](#key-findings)
  - [1. Technology Overview](#1-technology-overview)
  - [2. Current State & Trends](#2-current-state--trends)
  - [3. Best Practices](#3-best-practices)
  - [4. Security Considerations](#4-security-considerations)
  - [5. Performance Insights](#5-performance-insights)
- [Comparative Analysis](#comparative-analysis)
- [Implementation Recommendations](#implementation-recommendations)
- [Resources & References](#resources--references)
- [Appendices](#appendices)
- [Unresolved Questions](#unresolved-questions)
- [Next Steps](#next-steps)

---

## Executive Summary

**Nếu mục tiêu là sim-to-real thì "thêm visual" là sai bài toán.** Visual là công cụ
debug, không phải thứ đóng gap. Nghiên cứu 2024–2026 cho thấy gap của quadrotor nhỏ
đến từ: (1) **delay** trong firmware + state estimation, (2) **motor lag / action
latency**, (3) tham số động lực học sai (khối lượng, quán tính, `k_f`), (4) sensor
noise/latency. Không có renderer nào sửa được 4 thứ đó.

**Đề xuất tối ưu (YAGNI/KISS/DRY):** giữ kiến trúc hiện tại, thêm theo 4 tầng, làm từ
rẻ→đắt:

1. **Tầng 0 — Real2Sim data loop (ROI cao nhất, ~1 ngày):** tận dụng CSV log đã có +
   `tools/compare_logs.py` để SysID từ log bay thật → cập nhật A1–A10. Đây là nơi gap
   thực sự đóng. Không cần 3D.
2. **Tầng 1 — Visual debugging (~1–2 ngày):** **Rerun** (offline replay từ CSV + live
   sink tuỳ chọn). 3D pose + time-series trên **cùng timeline**, tái dùng log sẵn có,
   không đụng firmware/physics/test. Đây là câu trả lời cho "thêm visual".
3. **Tầng 2 — Fidelity plant (~3–5 ngày):** `vehicle_mujoco.py` (Menagerie Crazyflie)
   như plant **opt-in** (`--plant mujoco`), thêm **action latency + drag + ground
   effect**. RK4 vẫn là reference bit-for-bit. Dùng **sim-to-sim** (RK4 vs MuJoCo) làm
   sanity check.
4. **Tầng 3 — Robustness (~3–5 ngày):** `DomainRandomization` wrapper (selective DR:
   random `k_f/k_m/τ_m/latency`, SysID `m/I`), **sensor latency + sticky bias**.

**MuJoCo viewer / Gazebo / Foxglove không nên là xương sống.** MuJoCo chỉ nên vào như
plant thay thế, không phải lớp visual chính. Rerun thắng cho visual vì timeline-based
(xử lý thời gian ảo + chạy nhanh hơn realtime tự nhiên), gộp 3D + plot, và đọc được
log CSV có sẵn. **HIL (phase 9) mới là cây cầu sim-to-real thật** — nó bắt được delay
firmware/jitter mà mọi simulator trên PC đều bỏ qua.

---

## Research Methodology

- **Sources consulted:** 5 web searches (4 thành công; 1 bị rate-limit 429). Nguồn chính:
  Rerun docs/site, MuJoCo docs (`launch_passive`/`sync`), MuJoCo Menagerie,
  Foxglove comparison guides (2025), arXiv 2412.11764 (SimpleFlight, 2024),
  arXiv 2606.08039 (MuJoCo-drones-gym, 2025), RAPTOR (arXiv 2509.11481), reduct.store
  comparison (2025-07), mujoco_drones (Foxglove bridge).
- **Date range:** 2019 (Sim-to-Multi-Real) → 2026 (MuJoCo-drones-gym). Ưu tiên 2024–2026.
- **Search terms:** `rerun.io robotics visualization 3D telemetry 2025`;
  `MuJoCo passive viewer real-time python quadcopter 2025`;
  `drone simulation visualization alternatives Gazebo foxglove rerun mujoco comparison 2025`;
  `mujoco menagerie quadcopter crazyflie model MJCF 2025`;
  `sim-to-real quadrotor domain randomization system identification MuJoCo RotorPy 2025`.
- **Scope:** quyết định kiến trúc + chọn tool + kế hoạch triển khai cho repo này.
  Không đi sâu API chi tiết từng tool.

---

## Key Findings

### 1. Technology Overview

**Hai nhóm tool khác nhau, đừng trộn:**

| Nhóm | Vai trò | Ví dụ |
|---|---|---|
| **Visualizer** | Chỉ hiển thị state (3D + plot), không sinh vật lý | Rerun, Foxglove, meshcat, viser, PlotJuggler |
| **Physics engine / plant** | Sinh động lực học, có thể kèm viewer | MuJoCo (+ `mujoco.viewer`), Gazebo, PyBullet, Isaac |

**Rerun** — viewer + SDK code-first (Python/C++/Rust), entity-component + timeline,
render native (Rust/WGPU) và web (WASM). Log 3D transform, point cloud, ảnh, scalar
trên cùng timeline. Có `rerun-sdk` pip, `rr.init()` + `spawn`/`serve`. MCAP support
"early/experimental". Được dùng đúng bài toán quadrotor (Peng framework: log state,
trajectory, IMU). **Timeline-based** → không cần throttle realtime.

**MuJoCo** — physics engine Apache-2.0, viewer built-in. API quan trọng:
`mujoco.viewer.launch_passive(m, d)` **non-blocking**; user code tự bước physics/state;
phải gọi `viewer.sync()` để cập nhật; `viewer.lock()` khi sửa state; macOS cần
`mjpython`. Có thể dùng **visual-only**: load MJCF, set `qpos`, `mj_forward`, `sync`.
Menagerie có **Bitcraze Crazyflie 2 (MIT)** và **Skydio X2 (Apache-2.0)**.

**Foxglove** — platform (web+desktop) + SDK C++/Py/Rust, MCAP-first, team workflow
(Devices/Events/Projects). Mạnh cho team/enterprise; nặng hơn cho repo cá nhân; bản web
yếu khi data lớn.

**Khác:** `meshcat` (Drake/Pinocchio), `viser` (Python web 3D), `PlotJuggler`
(time-series), `Lichtblick` (fork mở của Foxglove Studio).

### 2. Current State & Trends

- **Visualization tách khỏi simulator** là xu hướng: mọi control stack tự ship viewer
  riêng → nhu cầu "một lớp neutral" (Rerun/Foxglove/Lichtblick).
- **MCAP** thành container chuẩn (ROS 2 Iron mặc định, Isaac Sim dùng).
- **GPU-parallel Python** (Isaac Lab, Genesis, Brax, MuJoCo Warp/MJX, ManiSkill) tăng
  nhanh — nhưng bottleneck là **throughput cho learning**, không phải fidelity cho
  engineering. Với repo này (control cổ điển, không RL) → **không cần**.
- **Sim-to-real cho quadrotor:** consensus 2024–2026:
  - **SysID** cho tham số đo được (mass, inertia) là bắt buộc; **DR ở đó phản tác dụng**
    (SimpleFlight).
  - **DR có chọn lọc** cho tham số nhạy/khó đo (`k_f`, motor lag, action latency).
  - **Motor lag + action latency là nguồn sai số lớn nhất** ở quadrotor nhỏ
    (MuJoCo-drones-gym; RAPTOR: "large part of the gap comes from delays in FC firmware
    and state estimation").
  - **Sim-to-sim** (nhiều simulator) là cách kiểm tra model không overfit.

### 3. Best Practices

**Cho sim-to-real (áp vào repo này):**

1. **SysID trước, DR sau.** Đo/ước lượng `m`, `I`, `k_f`, `k_q`, `τ_m`, `R_int` từ log
   thật → cập nhật A1–A10. Chỉ DR phần không đo được.
2. **Model latency, không chỉ noise.** Thêm `action_latency` (delay duty) và
   `sensor_latency` (delay IMU) vào plant. Đây là gap lớn nhất bị bỏ qua.
3. **DR chọn lọc (selective).** Theo SimpleFlight + MuJoCo-drones-gym:
   - Randomize: `k_f/k_m` ×[0.85,1.15], `τ_m` U[5,50]ms, action latency U[0,50]ms,
     sensor latency U[0,20]ms, drag ×[0.7,1.3], motor bias ×[0.95,1.05].
   - SysID (không DR): `m`, `I`.
4. **Per-episode DR** (giữ cố định trong 1 episode), **randomize coefficients không
   randomize forces** để policy/model học inverse mapping.
5. **Sim-to-sim trước sim-to-real:** RK4 ↔ RotorPy ↔ MuJoCo phải cho cùng xu hướng.
6. **Visual là debug aid:** overlay sim vs real trên cùng timeline; không dùng để "chứng
   minh" — gate test vẫn là oracle.
7. **HIL là bắt buộc** để bắt delay firmware + jitter (SIL PC không có).

**Cho visual layer (repo này):**

1. **Tách visual khỏi physics** — sink thuần tiêu thụ, không feedback, không chặn 1 kHz.
2. **Decimate** ra viewer (50–100 Hz), không log 1 kHz.
3. **Timeline-based viewer** (Rerun) để tránh throttle realtime.
4. **Một điểm đổi quy ước** (wxyz→xyzw, ENU→FRD) — như `to_frd`, verify bằng pose biết trước.
5. **Deps visual là optional** (`requirements-visual.txt`) để `make gate` không phình.

### 4. Security Considerations

Repo này là dev-host, không mở network port (UDS abstract + `SO_PEERCRED`) → bề mặt nhỏ.
Rủi ro mới khi thêm visual/fidelity:

- **Rerun `serve`/gRPC hoặc Foxglove WebSocket mở port** → chỉ bind `127.0.0.1`, không
  `0.0.0.0`, không expose ra LAN.
- **Log/`.rrd`/`.mcap` chứa dữ liệu chuyến bay** — repo đã `.gitignore` log; giữ nguyên,
  không commit artifact visual.
- **Supply chain:** thêm `rerun-sdk` / `mujoco` → pin version (repo đang pin chặt
  numpy/matplotlib/rotorpy/pytest). MuJoCo Apache-2.0, Rerun open-source, Menagerie model
  MIT/Apache → hợp license MIT của repo.
- **Không load MJCF/asset từ nguồn không tin** (MJCF có thể chứa mesh/plugin).

### 5. Performance Insights

| Tool | Đặc tính | Ghi chú cho repo |
|---|---|---|
| Rerun | Native Rust/WGPU, memory-mapped, zero-copy; giới hạn RAM (~75%, drop oldest) | Log 50–100 Hz → file `.rrd` nhỏ, ổn |
| MuJoCo | Rất nhanh; RGB ~2262 FPS (RTX4090), ~89 FPS (A100); contact cần dt nhỏ | Visual-only gần như free; plant-swap nặng hơn RK4 |
| Foxglove web | Giới hạn RAM browser, single-thread JS | Không hợp stream tần số cao |
| RK4 hiện tại | Nhanh nhất, tất định bit-for-bit | Giữ làm reference |

**Kết luận perf:** visual layer không phải bottleneck. Nếu plant-swap MuJoCo: dùng
`timestep` nhỏ + substep; chỉ bật khi cần fidelity, không chạy trong `make gate` mặc
định (để giữ test nhanh + tất định).

---

## Comparative Analysis

### A. Chọn lớp visual

| Tiêu chí | **Rerun** | MuJoCo viewer | Foxglove | meshcat/viser |
|---|---|---|---|---|
| Loại | Visualizer | Viewer của engine | Platform | Visualizer |
| Replay CSV log có sẵn | **Có** (đọc cột → log) | Không trực tiếp | Qua MCAP/CSV loader | Không |
| 3D + time-series 1 timeline | **Có** | Chỉ 3D | Có | 3D (+plot hạn chế) |
| Thời gian ảo / nhanh hơn realtime | **Có (timeline)** | Không (wall-clock) | Có (timeline) | Không |
| Live non-blocking | Có (`spawn`/`serve`) | Có (`launch_passive`) | Có (WebSocket) | Có |
| Dep Python | `rerun-sdk` | `mujoco` | SDK + app | `meshcat`/`viser` |
| Hợp repo cá nhân | **Cao** | Cao (nếu dùng MuJoCo) | Thấp (overkill) | Trung bình |
| Kết luận | **Chọn cho visual layer** | Chọn nếu MuJoCo là plant | Bỏ qua v0 | Phương án nhẹ |

### B. Chọn plant cho sim-to-real

| Plant | Fidelity | Tất định | Contact/aero | Effort | Vai trò |
|---|---|---|---|---|---|
| **RK4 (hiện tại)** | Thấp–trung | **Bit-for-bit** | Không (clamp) | 0 | Reference + `make gate` |
| **RotorPy (opt-in)** | Trung–cao | Trong cùng version | Có aero, có wind | Đã có | Đối chiếu |
| **MuJoCo (đề xuất)** | Cao | Trong cùng version | Contact + drag + ground | 3–5 ngày | Fidelity + sim-to-sim |

Không plant nào tự có: motor lag (đã có), **action latency**, sensor latency, SysID,
DR → phải tự thêm ở tầng runner/wrapper.

---

## Implementation Recommendations

### Kiến trúc đề xuất

```
                 ┌───────────────────────────────────────────────┐
                 │              FIRMWARE (C++20)                 │
                 │   estimator → failsafe → pid → mixer → gate   │
                 └───────────────┬───────────────────────────────┘
                                 │ StateRc / FwOut  (KHÔNG ĐỔI)
                 ┌───────────────┴───────────────┐
                 │        plant/runner.py         │  lockstep
                 │  + VisualSink (tuỳ chọn)       │
                 │  + DomainRandomization wrapper │
                 └───────┬───────────┬────────────┘
                         │           │
        ┌────────────────┴──┐   ┌────┴─────────────────────┐
        │ plant (chọn 1)    │   │ visual (tách rời)        │
        │  rk4 | rotorpy    │──►│  Rerun  (3D + plot)      │
        │  mujoco (mới)     │   │  hoặc MuJoCo viewer      │
        └───────────────────┘   └──────────────────────────┘
                 ▲
        ┌────────┴─────────┐
        │ SysID từ CSV log │  ← log bay thật (Tier C) → cập nhật A1–A10
        └──────────────────┘
```

Nguyên tắc bất biến giữ nguyên: **firmware không đổi; visual là consumer; gate test vẫn
là oracle; RK4 vẫn bit-for-bit.**

### Kế hoạch theo phase

| Phase | Nội dung | Effort | Rủi ro | Phụ thuộc |
|---|---|---|---|---|
| **V0** | Real2Sim: SysID từ CSV + cập nhật A1–A10 + overlay sim/real | 0.5–1 ngày | Thấp | có log bay thật |
| **V1** | Rerun offline replay `tools/visualize.py` (đọc CSV) | 0.5–1 ngày | Rất thấp | — |
| **V2** | Rerun live sink `--visual rerun` trong runner | 1 ngày | Thấp | V1 |
| **V3** | `vehicle_mujoco.py` plant opt-in + model Menagerie | 2–3 ngày | Trung | — |
| **V4** | Action/sensor latency + drag/ground vào plant | 1–2 ngày | Trung | V3 (hoặc RotorPy) |
| **V5** | DomainRandomization wrapper (selective) | 2–3 ngày | Trung | V4 |
| **V6** | Sim-to-sim validation (RK4 vs RotorPy vs MuJoCo) | 1 ngày | Thấp | V3 |
| **V7** | HIL bridge (phase 9) | theo plan | Cao | board |

**Thứ tự tối ưu:** V0 → V1 → V2 → V3 → V4 → V5 → V6 → V7. Nếu chỉ muốn "có visual":
làm V1 (+V2) là đủ, dừng ở đó.

### Quick Start Guide

```bash
cd drone-mini

# V1: replay một log có sẵn trong Rerun (không đụng physics)
.venv/bin/pip install rerun-sdk
PYTHONPATH=. .venv/bin/python tools/visualize.py logs/sil.csv

# V2: chạy SIL và stream live vào Rerun (opt-in)
PYTHONPATH=. .venv/bin/python -m plant.runner \
    --mode attitude --scenario att_hover --t-end 10 \
    --log logs/vis.csv --visual rerun

# V3: plant MuJoCo (opt-in, cần mujoco)
.venv/bin/pip install mujoco
PYTHONPATH=. .venv/bin/python -m plant.runner \
    --plant mujoco --mode attitude --scenario att_hover --t-end 10 --log logs/mj.csv
```

### Code Examples

**V1 — Rerun offline replay từ CSV (DRY: tái dùng log sẵn có).** Không đụng plant,
không đụng firmware.

```python
# tools/visualize.py  (khung sườn, chưa có trong repo)
import rerun as rr
from plant.log import load_log

def replay(path: str):
    log = load_log(path)
    rr.init("drone-mini", spawn=True)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    for i in range(log.n):
        t = float(log["t_us"][i]) * 1e-6
        rr.set_time_seconds("sim_time", t)
        # pose: cột log cần bổ sung x,y,z + quat (xem pitfall bên dưới)
        rr.log("world/drone", rr.Transform3D(
            translation=[log["x"][i], log["y"][i], log["z"][i]],
            rotation=rr.Quaternion(xyzw=[log["qx"][i], log["qy"][i],
                                         log["qz"][i], log["qw"][i]])))
        rr.log("plots/gyro_x", rr.Scalar(log["gyro_x"][i]))
        rr.log("plots/vbat",   rr.Scalar(log["vbat"][i]))
        rr.log("plots/mot0",   rr.Scalar(log["mot0"][i]))
```

**V2 — Live sink non-blocking (thêm vào `runner.py`, opt-in).** Sink là consumer thuần,
decimate, không feedback.

```python
# plant/visual_sink.py  (khung sườn)
class RerunSink:
    def __init__(self, hz: float = 50.0, dt: float = 0.001):
        import rerun as rr
        self._rr = rr
        self._stride = max(1, int(round((1.0 / hz) / dt)))
        self._n = 0
        rr.init("drone-mini", spawn=True)
        rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    def on_tick(self, t_us, pos, quat_wxyz, duty, gyro, vbat, armed, failsafe):
        if self._n % self._stride:
            self._n += 1
            return                       # decimate: không chặn loop 1 kHz
        self._n += 1
        rr = self._rr
        rr.set_time_seconds("sim_time", t_us * 1e-6)
        w, x, y, z = quat_wxyz
        rr.log("world/drone", rr.Transform3D(
            translation=list(pos), rotation=rr.Quaternion(xyzw=[x, y, z, w])))
        for k in range(4):
            rr.log(f"plots/mot{k}", rr.Scalar(float(duty[k])))
        rr.log("plots/vbat", rr.Scalar(float(vbat)))
```

**V3 — MuJoCo plant adapter (cùng interface `Plant` như `vehicle_rotorpy.py`).**

```python
# plant/vehicle_mujoco.py  (khung sườn)
import numpy as np
import mujoco
from . import drone_mini_params as P
from .vehicle import to_frd

class MuJoCoPlant:
    def __init__(self, seed: int = 0, clean: bool = False, xml: str | None = None):
        self.m = mujoco.MjModel.from_xml_path(xml or "assets/cf2/scene.xml")
        self.d = mujoco.MjData(self.m)
        self.m.opt.timestep = P.DT           # khớp tick ảo 1 kHz (hoặc substep)
        self.clean = clean
        self.reset()

    def reset(self, alt: float = 0.0):
        mujoco.mj_resetData(self.m, self.d)
        self.d.qpos[2] = alt
        mujoco.mj_forward(self.m, self.d)

    def step(self, dt, duty, vbat):
        om = np.asarray(P.pwm_to_omega(duty, vbat), dtype=float)
        # map duty->rotor speed: gán qvel của 4 rotor joints hoặc d.ctrl
        self.d.ctrl[:] = om
        mujoco.mj_step(self.m, self.d)
        if not np.all(np.isfinite(self.d.qpos)):
            raise FloatingPointError("mujoco state diverged")
        return self.d.qpos

    def sense(self, dt=0.001):
        # dùng sensor gyro/accelerometer của MuJoCo nếu có, đổi sang FRD
        gyro_frd = to_frd(self.d.sensor("gyro").data)
        accel_frd = to_frd(self.d.sensor("accel").data)
        return gyro_frd, accel_frd

    def finite(self): return bool(np.all(np.isfinite(self.d.qpos)))
    @property
    def pos(self): return self.d.qpos[0:3]
    # ... vel/quat/omega/motors tương tự
```

Đăng ký trong `plant/vehicle.py::make_plant`:

```python
if name == "mujoco":
    from .vehicle_mujoco import MuJoCoPlant
    return MuJoCoPlant(seed=seed, clean=clean)
```

**V5 — DomainRandomization wrapper (selective, per-episode).**

```python
# plant/randomize.py  (khung sườn)
import numpy as np

class DomainRandomization:
    """Bọc plant; resample per-episode. KHÔNG random thứ đo được (m, I)."""
    def __init__(self, plant, rng):
        self.plant, self.rng = plant, rng

    def resample(self):
        u = self.rng.uniform
        self.k_f_scale = u(0.85, 1.15)      # k_f nhạy -> DR
        self.k_m_scale = u(0.85, 1.15)
        self.tau_m     = u(0.005, 0.050)    # motor lag U[5,50]ms
        self.act_lat   = u(0.0, 0.050)      # action latency U[0,50]ms
        self.sensor_lat= u(0.0, 0.020)      # sensor latency U[0,20]ms
        # m, I: KHÔNG random — SysID từ cân/đo
```

**V0 — SysID từ log (khung sườn).** Tái dùng `plant/log.py`.

```python
# tools/sysid.py  (khung sườn)
# k_f từ hover: 4*k_f*omega_h^2 = m*g  (đo m, biết omega_h từ duty*hover)
# tau_m từ step response: fit exp -> 63% sau tau_m
# R_int từ load step: (V_oc - V_load)/I
# gyro bias từ log tĩnh: mean(gyro)
```

### Common Pitfalls

| # | Pitfall | Hậu quả | Fix |
|---|---|---|---|
| 1 | Render/step viewer trong vòng 1 kHz | Chặn lockstep, jitter | Sink riêng, decimate 50–100 Hz |
| 2 | Dùng MuJoCo viewer cho SIL chạy nhanh hơn realtime | Viewer wall-clock, không theo kịp | Dùng Rerun timeline, hoặc throttle realtime |
| 3 | Quên đổi quaternion `wxyz`→`xyzw` / ENU→FRD | Drone render sai hướng | 1 điểm đổi, verify pose biết trước |
| 4 | Log pose 1 kHz cả buổi bay | `.rrd` phình | Decimate, hoặc log khi pos đổi |
| 5 | Thêm `mujoco`/`rerun` vào deps bắt buộc | `make gate` chậm/phình | `requirements-visual.txt` optional |
| 6 | DR cả `m`/`I` (đo được) | Model học conservative, tệ hơn (SimpleFlight) | SysID `m`/`I`; DR chỉ `k_f/k_m/τ_m/latency` |
| 7 | Bỏ qua action latency | Gap lớn nhất không được model | Thêm delay duty + sensor latency |
| 8 | Plant-swap MuJoCo nhưng không chạy lại gate | Test cũ giả định RK4 | Chạy `make gate` + sim-to-sim |
| 9 | Rerun/Foxglove bind `0.0.0.0` | Lộ dữ liệu bay ra LAN | Bind `127.0.0.1` |
| 10 | Tin visual "nhìn đẹp" = đúng | Bỏ qua gate | Visual là debug aid, không phải oracle |
| 11 | MJCF dùng đơn vị/khung khác | Sai lực/mô men | Verify `sense()` khớp contract dấu (T3.10) |
| 12 | Contact + dt 1 ms | Nổ số / xuyên vật | Substep hoặc `timestep` nhỏ hơn |

---

## Resources & References

### Official Documentation

- Rerun — SDK & viewer: https://rerun.io/sdk · docs: https://rerun.io/docs/overview/what-is-rerun
- Rerun ROS node example (pattern logging pose/scans): https://rerun.io/examples/robotics/ros_node
- MuJoCo Python viewer (`launch_passive`, `sync`, `lock`): https://mujoco.readthedocs.io/en/stable/python.html
- MuJoCo model gallery / Menagerie: https://mujoco.readthedocs.io/en/stable/models.html
- MuJoCo Menagerie (Crazyflie 2, Skydio X2): https://github.com/google-deepmind/mujoco_menagerie
- Foxglove comparison guides (2025): https://foxglove.dev/robotics/rviz-vs-foxglove-vs-rerun · https://foxglove.dev/robotics/rerun-vs-foxglove

### Recommended Tutorials / Repos

- `Nil69420/mujoco_drones` — MuJoCo quad + Foxglove WebSocket bridge (tham khảo kiến trúc): https://github.com/Nil69420/mujoco_drones
- Peng quadrotor autonomy — Rerun integration (state/trajectory/IMU logging): https://deepwiki.com/makeecat/Peng/9.1-rerun.io-integration
- `varadVaidya/cf2_mujoco` — Crazyflie2 MJCF conversion: https://github.com/varadVaidya/cf2_mujoco

### Sim-to-real papers (2024–2026)

- SimpleFlight — 5 yếu tố zero-shot sim-to-real (SysID + selective DR): https://doi.org/10.48550/arxiv.2412.11764
- MuJoCo-drones-gym — 6 physics modes + DR ranges cho CF2X (2026): https://arxiv.org/html/2606.08039
- RAPTOR — foundation policy, emergent implicit SysID, delay là gap chính: https://arxiv.org/pdf/2509.11481
- Sim-to-(Multi)-Real (low-level policy, DR): https://ar5iv.labs.arxiv.org/html/1903.04628
- VPP MAV sim-to-real (SysID + DR + curriculum): https://arxiv.org/html/2504.07694
- Learning on the fly (differentiable sim, online adaptation): https://github.com/uzh-rpg/learning_on_the_fly
- SSI-MPC (simultaneous SysID + MPC): https://github.com/UM-iRaL/SSI-MPC

### Community Resources

- Rerun Discord (được cộng đồng đánh giá thân thiện).
- MuJoCo forum/discussions: https://github.com/google-deepmind/mujoco/discussions
- `gym-pybullet-drones` / RotorPy / Flightmare (sim đối chiếu).

---

## Appendices

### A. Glossary

| Term | Nghĩa |
|---|---|
| **Sim-to-real gap** | Khác biệt giữa hành vi trong sim và thật, làm policy/controller fail khi triển khai |
| **SysID** | System Identification — ước lượng tham số động lực học từ dữ liệu thật |
| **DR** | Domain Randomization — random hóa tham số khi train để tăng robustness |
| **Selective DR** | Chỉ DR tham số nhạy/khó đo; SysID tham số đo được |
| **Action latency** | Độ trễ từ lúc tính lệnh đến lúc motor đáp ứng |
| **Motor lag** | Hằng số thời gian `τ_m` của motor (bậc 1) |
| **Sim-to-sim** | Chuyển giữa các simulator để kiểm model không overfit |
| **Rerun / `.rrd`** | Viewer + định dạng log timeline của Rerun |
| **MJCF** | MuJoCo XML model format |
| **MCAP** | Container log robotics đa dụng (ROS 2, Isaac Sim) |

### B. Version Compatibility Matrix

| Thành phần | Version gợi ý | Ghi chú |
|---|---|---|
| Python | 3.13 (repo đang dùng) | Rerun/MuJoCo hỗ trợ 3.13 |
| numpy | 2.2.6 (đã pin) | — |
| rerun-sdk | pin theo bản mới nhất lúc làm | ghi version vào log metadata (như `versions`) |
| mujoco | 3.x | `launch_passive` ổn định; `mjpython` nếu macOS |
| Menagerie Crazyflie 2 | MIT | asset MJCF |
| rotorpy | 3.0.0 (đã pin) | đối chiếu sim-to-sim |

### C. Raw Research Notes

- Rerun: timeline-based, 3D+scalar, Python/C++/Rust, web/native, MCAP experimental,
  native Rust/WGPU, memory-mapped/zero-copy, RAM cap ~75%.
- MuJoCo: `launch_passive` non-blocking (2.3.3+), cần `sync()` (2.3.4+); `lock()` khi sửa
  state; `user_scn` để thêm geom debug; macOS cần `mjpython`.
- MuJoCo-drones-gym DR ranges (CF2X): mass/inertia ×[0.8,1.2], arm ×[0.98,1.02],
  kf/km ×[0.85,1.15], drag/ground/downwash ×[0.7,1.3], motor bias ×[0.95,1.05],
  τ_mot U[5,50]ms/ep, action latency U[0,50]ms/ep, MAX_RPM ×[0.95,1.05],
  pos/vel/RPY noise, gyro 0.05 rad/s, sensor latency U[0,20]ms/ep.
- SimpleFlight: SysID bắt buộc cho `m`,`I` (DR phản tác dụng); `k_f` nhạy, DR giúp;
  `τ_m` ít nhạy.
- RAPTOR: phần lớn gap = delay firmware + state estimation; đề xuất DR thêm delay.
- Foxglove web yếu khi point cloud/high-freq; Rerun native mạnh hơn cho data lớn.

---

## Unresolved Questions

1. **Log hiện tại có `x,y,z,quat` không?** `CSV_HEADER` 23 cột hiện **không** có pose
   đầy đủ (chỉ có gyro/accel/est attitude/mot/vbat). → V1 cần **thêm cột pose** (append
   cuối, bump `schema` lên v3) hoặc đọc trực tiếp state trong runner (live) thay vì CSV.
2. **Tick 1 kHz vs viewer 50–100 Hz:** chọn decimate ở sink hay ghi pose mỗi N tick?
   (đề xuất: sink tự decimate; log vẫn 1 kHz).
3. **MuJoCo model nào?** Crazyflie 2 (gần nhất, MIT) hay tự dựng từ tham số board thật?
   Board thật khác Crazyflie (ESP32-C3, brushed 1S) → cần SysID để khớp.
4. **Có dùng RL không?** Nếu không, bỏ MJX/GPU-parallel; nếu có, cân nhắc MuJoCo Warp /
   gym-pybullet-drones thay vì tự viết.
5. **Đo được gì trên board thật?** V0/V4 phụ thuộc có log bay thật + thrust stand +
   đo mass/inertia. Chưa có board → V0–V2 làm được, V4–V6 làm được (dùng giá trị giả
   định, ghi rõ), V7 chờ board.
6. **License asset:** Crazyflie 2 MIT ổn; nếu dùng Skydio X2 (Apache) cũng ổn — nhưng
   xác nhận không có asset GPL trong MJCF.

---

## Next Steps

1. **Chốt scope:** visual-only (V1+V2) hay hướng tới sim-to-real đầy đủ (V0→V6)?
2. **V1 ngay:** thêm cột pose vào log (schema v3) + viết `tools/visualize.py` (Rerun).
3. **V2:** thêm `--visual rerun` + `plant/visual_sink.py` (non-blocking, decimate).
4. **V0 song song:** viết `tools/sysid.py`; điền A1–A10 khi có log/đo thật.
5. **V3:** `plant/vehicle_mujoco.py` + MJCF Crazyflie; `--plant mujoco`; chạy sim-to-sim
   với RK4/RotorPy (`compare_logs.py`).
6. **V4–V5:** action/sensor latency + `DomainRandomization` (selective).
7. **V7:** ưu tiên HIL (phase 9) — cây cầu sim-to-real thật, bắt delay firmware/jitter.

**Nguyên tắc xuyên suốt:** firmware bất biến · visual là consumer · RK4 vẫn bit-for-bit ·
gate test vẫn là oracle · SysID trước, DR sau.

---

## Implementation Addendum — V3 shipped (branch `feature/visualization`)

**Status:** V3 implemented and verified. V1/V2/V0/V4–V7 not started.

**Delivered:**
- `plant/vehicle_mujoco.py` — `MuJoCoPlant`, cùng interface `Plant` (step/sense/finite/
  pos/vel/quat/omega/motors/rate_deriv/euler). Drive body bằng `mjData.xfrc_applied`
  (world frame, COM) với **cùng mô hình propulsion như `Rk4Plant`** (duty → motor bậc 1
  → `k_t·ω²` + `k_q`). Mặc định ghi đè mass/inertia bằng `drone_mini_params` để swap
  engine là drop-in; `params="menagerie"` giữ giá trị gốc (mass 0.027).
- `plant/assets/bitcraze_crazyflie_2/` — Menagerie Crazyflie 2 (MJCF + 39 OBJ + MIT
  LICENSE + attribution README), vendored 1.2 MB.
- `--plant mujoco` (runner) + `make_plant("mujoco")`; metadata `determinism` giờ động.
- `requirements-visual.txt` (mujoco==3.14.0) + `make venv-visual`; giữ `make gate` lean.
- `tools/sim_to_sim.py` + `make sim2sim`.
- Tests: `tests/test_mujoco_adapter.py` (5), `tests/test_sim_to_sim.py` (5).

**Verified evidence:**
- `make test` → 116 passed; `make gate` → **139 passed**, no regressions.
- RK4↔MuJoCo parity: `a_z@hover` ≈ 0 (rk4 −0.0000, mujoco +0.018); rate magnitude lệch
  ~11% (mujoco có aero drag); free-fall mujoco −4.52 vs −4.91 (−8%, drag).
- Full-stack sim-to-sim: `--plant rk4` vs `--plant mujoco`, `att_hover` 4 s, seed 1 →
  `compare_logs.py` **GATE PASS** (max_abs roll_est 0.0058, yaw_est 0.0004, gyro_z 0.0043).

**Findings (đính chính research ở trên):**
1. Menagerie Crazyflie 2 dùng **CTBR** (collective thrust + 3 moments), không phải 4
   rotor. Actuator moment của nó yếu hơn mô men mixer thật ~300× (1e-5 N·m, ≈0.42 rad/s²)
   → **không dùng actuator của model**; adapter áp wrench trực tiếp qua `xfrc_applied`.
2. `sense()` dùng sensor MuJoCo: `body_linacc` (FLU) → `to_frd` cho `[0,0,−g]` khớp RK4;
   `body_gyro` → FRD; quat `wxyz`.
3. **RotorPy không parameter-aligned**: dùng `k_eta` riêng, hình học rotor khác → cùng
   duty differential cho trục/magnitude khác (roll/pitch lệch trục, hover duty của ta làm
   nó rơi). Nên RotorPy chỉ là reference smoke test, **không** dùng cho parity assertion.
4. MuJoCo `density/viscosity` trong `<option>` tạo aero drag → free-fall lệch ~8% so với
   RK4 (đây là khác biệt vật lý thật, không phải bug).
5. `make test`/`make gate` fail khi shell đã source ROS (PYTHONPATH ROS 3.10 lọt vào venv
   3.13 → pytest auto-load plugin `launch_testing` thiếu `lark`). Đã fix: clear
   `PYTHONPATH` trong 2 target đó.

**Còn lại của V3 (chưa làm):** action/sensor latency; drag/ground-effect tuning cho khớp
board thật; sim-to-sim tự động ở tầng runner (hiện `compare_logs` chạy tay); SysID (V0).
