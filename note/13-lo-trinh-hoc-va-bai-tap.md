# 13 — Lộ trình học và bài tập làm chủ

> Đây là file trung tâm của bộ note: lộ trình 8 tuần, bài tập theo cấp độ, ngân hàng câu
> hỏi tự kiểm tra, checklist master, và hướng mở rộng sau repo. Hãy dùng nó như một
> bảng điều khiển tiến độ — tick từng ô khi hoàn thành.

---

## 1. Nguyên tắc học của lộ trình này

1. **Đọc ít, làm nhiều.** Tỷ lệ vàng: 30% đọc + 70% chạy thí nghiệm và viết code.
2. **Mỗi tuần phải có "bằng chứng".** Không tính là xong nếu chỉ đọc. Bằng chứng là:
   log, plot, test pass, hoặc đoạn code tự viết.
3. **Không nhảy chặng.** Chặng 2 mà chưa hiểu hệ trục thì chặng 3 sẽ rối.
4. **Ghi chép lại.** Mỗi tuần viết 5–10 dòng "nhật ký học": hôm nay hiểu gì, còn vướng
   gì. Đây là cách bạn tự phát hiện lỗ hổng.
5. **Hỏi "vì sao" 3 lần.** Ví dụ: vì sao cần desaturate? → vì duty giới hạn 0..1. Vì
   sao duty giới hạn? → vì PWM 11-bit. Vì sao 11-bit? → vì đủ độ phân giải ở 20–32 kHz.
   Đi được 3 tầng là hiểu thật.

### Chuẩn bị trước tuần 1

```bash
cd drone-mini
make venv && make host && make sil && make test
# phải thấy: tất cả unit test pass
```

Nếu bước này fail, xử lý trước khi học tiếp (thiếu g++ 11, cmake 3.22, hoặc uv).

---

## 2. Lộ trình 8 tuần

Giả định bạn có 10–15 giờ/tuần. Có thể giãn thành 12–16 tuần nếu bận.

### Tuần 1 — Bức tranh tổng thể & C++ trong repo

| | |
|---|---|
| **Mục tiêu** | Hiểu drone bay nhờ gì; hiểu kiến trúc repo; đọc được C++ của `flight/` |
| **Đọc** | `README.md`, `01-buc-tranh-tong-the.md`, `02-cpp20-trong-repo.md`, `10-kien-truc-hal-va-realtime.md` |
| **Đọc code** | `flight/ports.hpp`, `flight/context.hpp`, `flight/control_loop.cpp` |
| **Làm** | Chạy `make gate`; vẽ tay sơ đồ 1 tick ra giấy; làm bài tập 02 mục 11 |
| **Checkpoint** | Trả lời được 5 câu hỏi checkpoint file 01 |
| **Bằng chứng** | Ảnh chụp `make gate` xanh + sơ đồ tick tự vẽ |

### Tuần 2 — Hệ quy chiếu & động lực học

| | |
|---|---|
| **Mục tiêu** | Hiểu FRD/FLU/ENU, quaternion, 6-DOF, RK4 |
| **Đọc** | `03-toa-do-va-dong-luc-hoc-6dof.md`, `plant/vehicle.py` |
| **Làm** | Chạy script kiểm tra dấu (như file 06 đã làm); tự tính hover ω và duty bằng tay; chạy `test_plant_dynamics.py` và đọc từng assert |
| **Checkpoint** | Vẽ được vector lực/mô men của quad-X; giải thích `tau_x`, `tau_y`, `tau_z` |
| **Bằng chứng** | Kết quả tính tay khớp với `hover_duty_nominal()` |

### Tuần 3 — Motor, PWM, pin và IMU

| | |
|---|---|
| **Mục tiêu** | Hiểu duty → lực; hiểu mô hình pin; hiểu gyro/accel và sai số |
| **Đọc** | `04-dong-co-pwm-va-pin.md`, `05-imu-icm20948.md`; `plant/battery.py`, `plant/icm20948.py` |
| **Làm** | Bài tập tính σ nhiễu; chạy plant với `clean=True` vs có nhiễu và so sánh plot; đọc `sensor_cfg.hpp` và `test_param_parity.py` |
| **Checkpoint** | Giải thích vì sao bias nguy hiểm hơn nhiễu; vì sao motor không tức thời |
| **Bằng chứng** | Hai plot (clean/noisy) và nhận xét |

### Tuần 4 — Ước lượng và điều khiển

| | |
|---|---|
| **Mục tiêu** | Hiểu complementary filter, hiệu chuẩn bias, PID, anti-windup, cascade |
| **Đọc** | `06-uoc-luong-tu-the.md`, `07-pid-va-cascade.md`; `flight/estimator.cpp`, `flight/pid.cpp` |
| **Làm** | Chạy `test_estimator.py`, `test_pid.py`; chạy SIL `--mode rate --scenario rate_step_roll`, plot và đo settling time bằng mắt; đổi `rate_kp` ×2 rồi chạy lại, quan sát dao động |
| **Checkpoint** | Giải thích `sat_pos/sat_neg` từ mixer; giải thích vì sao D dùng measurement |
| **Bằng chứng** | 3 plot step response: gain gốc, ×0.5, ×2 + nhận xét |

### Tuần 5 — Mixer, RC, an toàn

| | |
|---|---|
| **Mục tiêu** | Hiểu mixer và dấu; hiểu SBUS; hiểu FSM arm/failsafe |
| **Đọc** | `08-mixer-va-dau-motor.md`, `09-rc-sbus-arm-failsafe.md`; `flight/mixer.cpp`, `flight/rc_parse.cpp`, `flight/failsafe.cpp` |
| **Làm** | Thí nghiệm phá mixer (file 08 mục 6) với 3 kiểu đổi dấu; chạy `test_failsafe.py`; chạy SIL scenario `rc_loss` và `rx_failsafe`, xác nhận PWM về 0 trong 100 ms |
| **Checkpoint** | Kể 8 điều kiện arm; giải thích latch; giải thích chống replay |
| **Bằng chứng** | Bảng "đổi dấu nào → test nào fail"; plot scenario `rc_loss` |

### Tuần 6 — SIL, wire protocol, test, log

| | |
|---|---|
| **Mục tiêu** | Hiểu lockstep, wire format, CRC, HELLO; biết đọc log và chạy gate |
| **Đọc** | `11-sil-lockstep-wire-protocol.md`, `12-test-log-tools.md`; `hal/sil_wire.hpp`, `plant/sil_proto.py`, `plant/runner.py` |
| **Làm** | Vẽ sequence diagram 1 tick; tính băng thông HIL; thử gửi packet sai CRC (sửa tạm `sil_proto.pack_msg`) và xem exit code; đọc `test_abi.py` |
| **Checkpoint** | Giải thích HELLO bảo vệ điều gì; vì sao cần kiểm monotonic `t_us` |
| **Bằng chứng** | Sequence diagram + bảng tính băng thông |

### Tuần 7 — Làm chủ: bài tập cấp 2–3

| | |
|---|---|
| **Mục tiêu** | Tự tay thay đổi hệ thống và kiểm chứng |
| **Làm** | Chọn tối thiểu 4 bài ở mục 4 (Level 2–3) bên dưới |
| **Checkpoint** | Mỗi bài phải có test/log chứng minh trước–sau |
| **Bằng chứng** | Commit log + log/plot cho từng bài |

### Tuần 8 — Mở rộng: chọn một dự án nhỏ

| | |
|---|---|
| **Mục tiêu** | Đi từ "hiểu repo" sang "làm chủ chủ đề" |
| **Làm** | Chọn 1 dự án ở mục 7 (ví dụ: viết EKF đơn giản, thêm mode position-hold, viết HIL bridge stub, thêm motor model phi tuyến) |
| **Checkpoint** | Trình bày được thiết kế, kết quả, hạn chế |
| **Bằng chứng** | Báo cáo 1–2 trang + log/plot + test |

---

## 3. Ngân hàng câu hỏi tự kiểm tra

Tự trả lời trước khi mở đáp án. Nếu sai > 3 câu trong một nhóm, đọc lại file tương ứng.

### Nhóm A — Kiến trúc (file 01, 10)

1. Firmware giao tiếp với phần cứng qua mấy hàm? Kể tên nhóm chức năng.
2. Vì sao `flight/` không được include header ESP-IDF?
3. Output gate chạy ở đâu trong 1 tick và vì sao phải ở cuối?
4. Ba adapter của `IHal` là gì, dùng cho việc gì?
5. Vì sao object được tạo `static` trong `main()` mà không phải `new`?

### Nhóm B — Vật lý (file 03, 04)

6. Đổi vector từ FLU sang FRD làm gì?
7. Viết công thức lực đẩy và mô men yaw.
8. Tại sao hover duty khoảng 0.5 là hợp lý?
9. RK4 khác Euler thế nào?
10. Điện áp pin ảnh hưởng đến vận tốc motor ra sao?

### Nhóm C — Cảm biến & ước lượng (file 05, 06)

11. Gyro đo gì, accel đo gì? Cái nào drift?
12. Viết công thức tính roll/pitch từ accel (FRD).
13. `α = 0.998` nghĩa là gì theo tần số?
14. Vì sao cần hiệu chuẩn bias? Không hiệu chuẩn thì hậu quả số cụ thể?
15. Vì sao yaw không được hiệu chỉnh?

### Nhóm D — Điều khiển (file 07)

16. Viết PID rời rạc. Giải thích từng số hạng.
17. Windup là gì? Cách chống?
18. Vì sao D dùng measurement?
19. Cascade 2 vòng: vòng nào nhanh hơn, vì sao?
20. Khi nào mixer bão hòa và PID biết bằng cách nào?

### Nhóm E — Mixer & an toàn (file 08, 09)

21. Vì sao cần 2 CW + 2 CCW?
22. Desaturate giữ gì, đổi gì?
23. Kể 5 điều kiện arm.
24. Failsafe kích hoạt bởi những gì? Latch nghĩa là gì?
25. Sau brownout, firmware biết bằng cách nào và phản ứng ra sao?

### Nhóm F — SIL (file 11, 12)

26. Lockstep là gì? Ai là master clock?
27. HELLO so sánh gì? Vì sao cần?
28. CRC bảo vệ gì? Có phải bảo mật không?
29. Vì sao kiểm `seq`/`t_us` monotonic?
30. Log schema gồm gì? Vì sao append-only?

---

## 4. Bài tập theo cấp độ

### Level 1 — Đọc hiểu (bắt buộc, tuần 1–6)

| # | Bài | File liên quan | Bằng chứng |
|---|---|---|---|
| 1.1 | Vẽ sơ đồ khối toàn hệ thống từ `sim/main_sil.cpp` | `sim/main_sil.cpp` | ảnh vẽ tay |
| 1.2 | Vẽ sequence 1 tick (11 bước) | `control_loop.cpp` | ảnh vẽ |
| 1.3 | Liệt kê 10 interface + 1 câu mô tả | `ports.hpp` | ghi chú |
| 1.4 | Giải thích từng dòng `desaturate()` bằng lời | `mixer.cpp:14` | ghi chú |
| 1.5 | Giải thích từng dòng `channel()` (SBUS bit unpack) | `rc_parse.cpp:22` | ghi chú |
| 1.6 | Đọc 6 static_assert và giải thích bảo vệ gì | `sil_wire.hpp:106` | ghi chú |
| 1.7 | Đọc `test_output_gate.py`, giải thích vì sao kiểm cả `pwm_offs` | `tests/` | ghi chú |
| 1.8 | Tính hover ω, duty, dòng pin ở hover | `params` | phép tính |

### Level 2 — Sửa code nhỏ, quan sát test (tuần 4–6)

| # | Bài | Cách kiểm chứng |
|---|---|---|
| 2.1 | Đổi dấu yaw trong mixer → test nào fail? | `make test` |
| 2.2 | Đổi dấu roll trong mixer → test nào fail? | `make test` |
| 2.3 | Đặt `kCalibSamples = 10` → test calibrate có fail? Vì sao? | `make test` |
| 2.4 | Đặt `kImuInvalidDebounce = 1` → hành vi thay đổi thế nào? | `make test` + giải thích |
| 2.5 | Tắt anti-windup (bỏ `block`) → chạy scenario `saturation` → xem log | plot trước/sau |
| 2.6 | Tăng `rate_out_limit` lên 1.0 → step response thay đổi gì? | plot |
| 2.7 | Bỏ decimation estimator (`kEstDecim = 1`) → ảnh hưởng gì? | `make gate` |
| 2.8 | Đổi `kRcTimeoutUs` xuống 20 ms → scenario `rc_loss` failsafe lúc nào? | plot |
| 2.9 | Thêm trường `float x;` vào `SilImu` → build fail ở đâu? | `make host` |
| 2.10 | Sửa `sil_proto.pack_msg` ghi CRC sai → chạy SIL, exit code nào? | chạy runner |

### Level 3 — Tự viết (tuần 7)

| # | Bài | Gợi ý |
|---|---|---|
| 3.1 | Viết unit test mới cho mixer: kiểm desaturate giữ chênh lệch | bắt chước T2.8 |
| 3.2 | Thêm scenario RC mới: step yaw ở rate mode, kiểm đáp ứng | sửa `plant/rc.py` |
| 3.3 | Thêm trường `gyro_bias[3]` vào log + cập nhật schema lên v3 | `plant/log.py` + `runner.py` |
| 3.4 | Viết script Python mô phỏng PID bậc 1 và tune gain khớp với C++ | so sánh kết quả |
| 3.5 | Thêm mode `kPositionHold` (giữ độ cao bằng vbat/est z) — thiết kế trước, code sau | mở rộng `ControlMode` |
| 3.6 | Viết HAL mới `ReplayHal` đọc log cũ và phát lại — dùng để test offline | bắt chước `SilHal` |
| 3.7 | Thêm hỗ trợ yaw stick trong attitude mode (file 07 mục 5.3) | sửa `control_loop.cpp` |
| 3.8 | Viết test đo jitter của SIL (`jitter_stats`) và assert < 5 µs | `plant/log.py` |

### Level 4 — Đo đạc / phần cứng (khi có board, phase 9–10)

| # | Bài | Ghi chú |
|---|---|---|
| 4.1 | Spin bench: xác nhận A1 (motor nào quay, chiều nào) | bắt buộc trước bay |
| 4.2 | Cân khối lượng, đo arm length → cập nhật A2/A3 | cập nhật params + test |
| 4.3 | Đo `τ_m` bằng thrust stand step | thay A4 |
| 4.4 | Đo `R_int` pin bằng load step | thay A6 |
| 4.5 | Đo dropout LDO bằng scope | thay A10 |
| 4.6 | Sniff J6 để xác nhận SBUS (A8) | logic analyzer |
| 4.7 | Chạy HIL-1: đo jitter trên chip, I2C timeout, WDT | phase 9 |
| 4.8 | Bay ràng buộc, overlay log HIL vs bay | phase 10 |

### Level 5 — Nghiên cứu mở rộng (sau khóa học)

| # | Chủ đề | Hướng |
|---|---|---|
| 5.1 | Mahony / Madgwick filter | thay estimator, so sánh drift với complementary |
| 5.2 | EKF đầy đủ (attitude + vị trí) | thêm baro/GPS giả vào plant |
| 5.3 | Position control (PID vị trí → attitude setpoint) | cần ước lượng vị trí |
| 5.4 | Mixer phi tuyến / airmode | cải thiện hành vi khi bão hòa |
| 5.5 | Motor model phi tuyến (ESC, deadband, thrust curve) | cập nhật `pwm_to_omega` |
| 5.6 | Va chạm / ground effect | tích hợp MuJoCo như plant thay thế |
| 5.7 | System identification từ log | ước lượng `k_t`, `k_q`, `τ_m` từ dữ liệu |
| 5.8 | Formal verification bất biến an toàn | dùng TLA+/Alloy cho FSM |

---

## 5. Checklist master

Tick khi bạn **chứng minh được** (không phải khi đọc xong).

### Hiểu biết nền tảng
- [ ] Giải thích được vòng lặp điều khiển phản hồi bằng ví dụ giữ gậy
- [ ] Vẽ được hệ trục ENU/FLU/FRD và đổi vector giữa chúng
- [ ] Giải thích quaternion là gì và vì sao không dùng Euler
- [ ] Viết được phương trình 6-DOF của quadcopter
- [ ] Giải thích RK4 và vì sao cần

### Cảm biến & ước lượng
- [ ] Giải thích gyro vs accel: đo gì, drift ra sao
- [ ] Tính được σ nhiễu từ noise density
- [ ] Giải thích vì sao phải hiệu chuẩn bias và hậu quả nếu không
- [ ] Viết được complementary filter và giải thích α
- [ ] Biết vì sao yaw không hiệu chỉnh được nếu không có mag

### Điều khiển
- [ ] Viết được PID rời rạc và giải thích từng thành phần
- [ ] Giải thích derivative kick và cách tránh
- [ ] Giải thích windup và conditional integration
- [ ] Vẽ được sơ đồ cascade attitude → rate → mixer
- [ ] Tự tune được gain cho một hệ bậc 1 đơn giản

### Mixer & an toàn
- [ ] Viết được công thức mixer quad-X từ hình học
- [ ] Giải thích desaturation và vì sao không clip từng kênh
- [ ] Giải thích toàn bộ FSM arm/disarm
- [ ] Kể được 4 loại failsafe và cách chúng latch
- [ ] Giải thích vì sao brownout phải xử lý ở boot sau

### Kiến trúc & realtime
- [ ] Giải thích Ports & Adapters và lợi ích
- [ ] Giải thích DI qua `FlightContext`
- [ ] Kể 5 luật realtime và lý do
- [ ] Giải thích vì sao output gate là điểm duy nhất ghi PWM

### SIL & kiểm thử
- [ ] Vẽ được sequence lockstep
- [ ] Giải thích wire header, CRC, HELLO
- [ ] Giải thích chống replay và vì sao cần
- [ ] Đọc được log và biết dùng `plot_sil`/`compare_logs`
- [ ] Tự viết được một unit test với ctypes binding

### Thực hành
- [ ] Hoàn thành ít nhất 6 bài Level 1
- [ ] Hoàn thành ít nhất 4 bài Level 2
- [ ] Hoàn thành ít nhất 2 bài Level 3
- [ ] Viết báo cáo cuối khóa 2 trang: hệ thống làm gì, giới hạn gì, bạn sẽ mở rộng gì

---

## 6. Cách tự chấm và theo dõi tiến độ

Mỗi tuần, tạo một file `note/progress/tuan-N.md` gồm:

```markdown
# Tuần N
## Đã đọc
- [x] file...
## Đã chạy
- lệnh + kết quả tóm tắt
## Đã làm
- bài tập + link commit/log
## Vướng
- câu hỏi chưa trả lời được
## Tuần sau
- 3 việc ưu tiên
```

Sau 8 tuần, bạn sẽ có một **portfolio** gồm: log, plot, test tự viết, báo cáo. Đây là
thứ chứng minh năng lực mạnh hơn mọi chứng chỉ.

---

## 7. Dự án mở rộng gợi ý (tuần 8 và sau)

### Dự án A — "SIL Dashboard"
Viết script Python đọc log và in bảng tóm tắt: settling time, overshoot, drift, số lần
bão hòa, điện áp thấp nhất. Tự động chạy cho nhiều seed và nhiều gain.

### Dự án B — "Estimator bake-off"
Implement thêm Mahony filter trong C++ (theo đúng interface `IEstimator`), chạy cùng
bộ test SIL với complementary, so sánh drift/độ trễ bằng `compare_logs.py`.

### Dự án C — "HIL bridge"
Viết `tools/hil_bridge.py`: nhận TELEM từ UART, chạy plant, gửi lại STATE — bản phác
thảo của phase 9 chạy được trên dev board.

### Dự án D — "System identification"
Từ log scenario `hover_sweep`, ước lượng `k_t` và `τ_m` bằng least squares; so sánh với
giả định A4/A5. Đây là kỹ năng cực giá trị trong công nghiệp.

### Dự án E — "Fault injection"
Thêm vào plant khả năng tiêm lỗi: IMU spike, packet loss, pin sụt đột ngột, motor chết
1 cánh. Kiểm tra failsafe/estimator phản ứng đúng. (Đây là cách các hãng hàng không
test phần mềm.)

---

## 8. Tài liệu tham khảo

### Trong repo (đọc trước)
- `SIL_STACK.md` (bản gốc) — spec của toàn hệ thống.
- `docs/measurements.md` — registry giả định + kết quả tune.
- `plans/260921-2229-drone-mini-sil-hil-stack/plan.md` — quyết định D1–D27, red team
  findings. Đọc phần "Red Team Review" để học cách tự phản biện thiết kế.
- `README.md` repo — ABI contract, security notes.

### Sách / khóa học
- *Small Unmanned Aircraft: Theory and Practice* — Beard & McLain (kinh điển UAV).
- *Feedback Systems: An Introduction for Scientists and Engineers* — Åström & Murray
  (miễn phí online, nền tảng điều khiển).
- *Probabilistic Robotics* — Thrun et al. (nếu đi sâu vào estimation/EKF).
- *Making Embedded Systems* — Elecia White (kỹ năng embedded).
- *Real-Time C++* — Christopher Kormanyos (C++ cho nhúng).
- Betaflight / PX4 source code — đọc mixer và rate loop thật của họ.

### Datasheet & giao thức
- ICM-20948 datasheet DS-000189 — đọc mục noise, FSR, ODR, register map.
- SBUS protocol (Futaba) — 25 byte, 11-bit LSB-first.
- ESP32-C3 Technical Reference — LEDC, I2C, UART, brownout.
- ESP-IDF Programming Guide — FreeRTOS, watchdog, `esp_reset_reason()`.

### Kỹ năng công cụ
- `gdb`/`valgrind` — debug C++.
- `pytest` docs — fixture, parametrize, marker.
- `numpy` docs — vector hóa, random generator.
- Git — commit nhỏ, message rõ ràng (xem lịch sử repo làm mẫu).

---

## 9. Lời khuyên cuối khóa

1. **An toàn trước, bay sau.** Không bao giờ arm khi chưa làm spin bench. Không bao
   giờ bay khi chưa pass tầng A. Đây không phải thủ tục hành chính — đó là đạo đức
   nghề nghiệp của người làm robotics.

2. **Trung thực với giả định.** Repo này dạy một điều quý: mọi giá trị chưa đo đều
   được ghi rõ là giả định (A1–A10) và **không dùng giả định làm oracle cho test kiểm
   chính giả định đó**. Hãy mang thói quen này sang mọi dự án.

3. **Test là đặc tả.** Khi bạn không biết một module phải làm gì, hãy đọc test của nó.
   Khi bạn viết module mới, hãy viết test trước.

4. **Fail loud, fail early.** `static_assert`, HELLO, exit code, latch — tất cả đều
   theo một triết lý: lỗi phải lộ ra ngay, không âm thầm. Đây là phong cách của phần
   mềm an toàn.

5. **Học cách hỏi.** Câu hỏi tốt nhất không phải "code này làm gì" mà là "vì sao
   người viết chọn cách này thay vì cách kia". Red team review trong plan là ví dụ
   mẫu: 33 finding, 15 được accept — đó là cách hệ thống trưởng thành.

---

*Hết bộ note. Quay lại `README.md` nếu cần bản đồ tổng thể.*
