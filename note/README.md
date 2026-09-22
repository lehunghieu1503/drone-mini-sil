# Bộ note: Đọc hiểu & master UAV qua repo `drone-mini`

> Dành cho fresher C++ muốn hiểu **một hệ thống UAV hoàn chỉnh** đang làm gì, từ vật lý
> bay đến firmware realtime, từ thuật toán đến kiểm thử.
>
> Bộ note này giảng giải **chính source code trong repo này**, không phải giáo trình
> UAV chung chung. Mỗi khái niệm đều trỏ về file:line cụ thể.

---

## 1. Repo này là gì?

`drone-mini` là một **flight controller** (bộ điều khiển bay) viết bằng **C++20** cho
drone 4 cánh (quadcopter) cỡ nhỏ, chạy trên **ESP32-C3**. Điểm đặc biệt:

1. **Một firmware, hai HAL.** Cùng một code điều khiển (`firmware/main/flight/`)
   chạy được trên **chip thật** (`HwHal`) và trên **mô phỏng** (`SilHal`).
2. **Software-in-the-Loop (SIL).** Plant (mô hình vật lý drone) viết bằng Python,
   chạy **lockstep** với firmware: mỗi 1 ms, plant đưa cảm biến giả vào firmware,
   firmware trả về PWM giả, plant bước vật lý 1 bước.
3. **Hardware-in-the-Loop (HIL)** và **bay ràng buộc** là 2 tầng tiếp theo (chưa làm
   vì chưa có board — phase 9, 10 trong plan).

Nói ngắn: đây là cách **kiểm tra thuật toán bay trước khi gắn cánh quạt** — an toàn
hơn nhiều so với vừa code vừa thả drone thật.

---

## 2. Bạn sẽ master được gì sau khi học hết bộ note?

| Mảng | Nội dung | Ở đâu trong repo |
|---|---|---|
| Hệ quy chiếu | ENU / FLU / FRD, đổi trục, vì sao firmware dùng FRD | `plant/vehicle.py`, `flight/ports.hpp` |
| Động lực học 6-DOF | Quaternion, ma trận quay, lực/mô men, tích phân RK4 | `plant/vehicle.py` |
| Động cơ & chấp hành | PWM, duty → vận tốc góc, `k_t·ω²`, hằng số thời gian | `plant/drone_mini_params.py` |
| Pin | Mô hình Thevenin, sag, brownout, LDO 3V3 | `plant/battery.py` |
| Cảm biến IMU | Gyro/accel, noise density, bias, lượng tử hóa, ODR | `plant/icm20948.py`, `flight/sensor_cfg.hpp` |
| Ước lượng tư thế | Complementary filter, hiệu chuẩn bias, yaw drift | `flight/estimator.cpp` |
| Điều khiển | PID rời rạc, anti-windup, D-on-measurement, cascade | `flight/pid.cpp` |
| Phân phối lực | Mixer quad-X, dấu roll/pitch/yaw, desaturation | `flight/mixer.cpp` |
| RC & an toàn | SBUS, arm FSM, failsafe, latch, boot reset reason | `flight/rc_parse.cpp`, `flight/failsafe.cpp` |
| Kiến trúc nhúng | Ports & Adapters, DI, realtime policy, dual build | `flight/ports.hpp`, `flight/context.hpp` |
| SIL/HIL | Lockstep, wire protocol, CRC, UDS socket, determinism | `hal/sil_wire.hpp`, `plant/runner.py` |
| Kiểm thử & log | ctypes binding, pytest, log schema, overlay tool | `tests/`, `plant/log.py`, `tools/` |

Đây chính là bộ kỹ năng của một **embedded flight-software engineer** thực thụ.

---

## 3. Lộ trình 6 chặng

Đọc theo đúng thứ tự. Mỗi chặng có mục "Checkpoint" — chỉ đi tiếp khi trả lời được.

```
Chặng 0 ─ Chuẩn bị & bức tranh tổng thể          [README, 01]
   │
Chặng 1 ─ C++20 + kiến trúc hệ thống              [02, 10]
   │
Chặng 2 ─ Vật lý & mô hình plant                  [03, 04, 05]
   │
Chặng 3 ─ Thuật toán bay (estimation → control)   [06, 07, 08, 09]
   │
Chặng 4 ─ Hạ tầng SIL, test, log                  [11, 12]
   │
Chặng 5 ─ Làm chủ: bài tập, mở rộng, HIL          [13]
```

| File | Nội dung | Nên đọc sau |
|---|---|---|
| `README.md` | File này — index & cách học | — |
| `00-tu-dien-thuat-ngu.md` | Từ điển thuật ngữ (tra cứu bất cứ lúc nào) | — |
| `01-buc-tranh-tong-the.md` | Drone bay nhờ gì, 1 tick chạy qua đâu, 3 tầng SIL/HIL | README |
| `02-cpp20-trong-repo.md` | C++20 qua chính code này: interface, POD, packed, constexpr | 01 |
| `03-toa-do-va-dong-luc-hoc-6dof.md` | FRD/FLU/ENU, quaternion, 6-DOF, RK4 | 02 |
| `04-dong-co-pwm-va-pin.md` | PWM, motor bậc 1, `k_t`, hover, pin Thevenin | 03 |
| `05-imu-icm20948.md` | Gyro/accel, noise, bias, lượng tử hóa, decimation | 03 |
| `06-uoc-luong-tu-the.md` | Complementary filter, hiệu chuẩn, yaw drift | 05 |
| `07-pid-va-cascade.md` | PID, anti-windup, rate loop, attitude cascade | 06 |
| `08-mixer-va-dau-motor.md` | Mixer quad-X, bảng dấu, desaturation | 07 |
| `09-rc-sbus-arm-failsafe.md` | SBUS, arm FSM, failsafe, latch | 08 |
| `10-kien-truc-hal-va-realtime.md` | Ports & Adapters, DI, dual build, realtime | 02 |
| `11-sil-lockstep-wire-protocol.md` | Lockstep, wire format, CRC, UDS, determinism | 03, 10 |
| `12-test-log-tools.md` | ctypes/pytest, log schema, plot/compare, gate | 11 |
| `13-lo-trinh-hoc-va-bai-tap.md` | **Lộ trình tuần tự + bài tập + checklist master** | tất cả |

---

## 4. Chuẩn bị môi trường (làm trước khi đọc file 03)

```bash
cd drone-mini

# 1. Tạo venv + deps (cần uv; hoặc dùng python3 -m venv + pip)
make venv

# 2. Build thư viện host (không cần ESP-IDF)
make host

# 3. Build SIL runner (firmware client chạy trên PC)
make sil

# 4. Chạy unit test nhanh
make test

# 5. Chạy toàn bộ gate (gồm cả mô phỏng slow, mất vài phút)
make gate
```

Nếu `make gate` xanh hết: bạn đang có một hệ thống SIL hoàn chỉnh chạy được. Đây là
điểm xuất phát lý tưởng để **đọc code kèm chạy thử**.

Chạy một phiên SIL đầu tiên và xem log:

```bash
PYTHONPATH=. .venv/bin/python -m plant.runner \
    --mode open-loop --scenario hover --t-end 3 \
    --ol-thr 0.55 --log logs/demo.csv

PYTHONPATH=. .venv/bin/python tools/plot_sil.py logs/demo.csv --out logs/demo.png
```

---

## 5. Cách đọc code hiệu quả (dành cho fresher)

1. **Đọc theo luồng dữ liệu, không theo thứ tự file.** Bắt đầu từ
   `sim/main_sil.cpp` → `flight/control_loop.cpp` → từng module mà nó gọi.
2. **Mỗi lần chỉ đọc 1 module.** Trong repo này mỗi module rất nhỏ (30–130 dòng).
3. **Vừa đọc vừa vẽ.** Vẽ hệ trục, vẽ sơ đồ khối, vẽ dấu mô men. Vẽ tay là cách
   hiểu nhanh nhất với UAV.
4. **Đọc test song song với code.** Test là "đặc tả chạy được":
   `tests/test_mixer.py` nói chính xác mixer phải làm gì.
5. **Chạy thí nghiệm nhỏ.** Muốn hiểu PID? Sửa gain rồi chạy lại SIL, xem log.
   Không có cách học nào nhanh hơn tự tay phá code và quan sát.
6. **Luôn hỏi "đơn vị là gì?".** rad/s hay deg/s? duty 0..1 hay 0..100%? Sai đơn vị
   là nguồn bug số 1 trong UAV.

Quy ước trích dẫn trong bộ note: `path/file.cpp:123` nghĩa là file đó, dòng 123.
Ví dụ `flight/mixer.cpp:28` là hàm `QuadXMixer::write`.

---

## 6. Trạng thái repo & bối cảnh

- Phase 1–8 **hoàn thành**: firmware core, plant RK4, mixer, PID, estimator, failsafe,
  log, test. `make gate` xanh.
- Phase 9 (HIL — chạy trên chip thật) và phase 10 (bay ràng buộc) **chưa làm** vì
  chưa có board. Đây là phần bạn có thể tự làm sau khi học xong.
- Các tham số vật lý chưa đo nằm trong **registry giả định A1–A10**
  (`plant/drone_mini_params.py:100` và `docs/measurements.md`). Đọc kỹ file này —
  nó dạy bạn cách một kỹ sư trung thực làm việc với giả định.

---

## 7. Tài liệu gốc phải đọc kèm

| Tài liệu | Vị trí | Vai trò |
|---|---|---|
| `SIL_STACK.md` | `../SIL_STACK.md` và bản copy `docs/SIL_STACK.md` | Spec gốc của cả hệ thống — đọc mục 1–8 |
| `plan.md` + phase-01..10 | `../../plans/260921-2229-drone-mini-sil-hil-stack/` | Kế hoạch triển khai, quyết định D1–D27 |
| `docs/measurements.md` | `docs/` | Registry giả định A1–A10, kết quả tuning |
| `README.md` (repo) | `../README.md` | Tóm tắt build/test/ABI |

---

*Bắt đầu học: mở `01-buc-tranh-tong-the.md`.*
