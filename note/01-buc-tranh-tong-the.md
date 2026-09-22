# 01 — Bức tranh tổng thể

> Đọc xong file này bạn sẽ trả lời được: drone bay nhờ cái gì? repo này giải quyết
> bài toán gì? một tick 1 ms chạy qua những bước nào? SIL/HIL là gì và vì sao cần?

---

## 1. Drone bay được là nhờ cái gì?

Hãy tưởng tượng bạn đang giữ thăng bằng một cây gậy trên đầu ngón tay. Bạn làm gì?

1. **Nhìn** gậy nghiêng về đâu (cảm biến).
2. **Não ước lượng** độ nghiêng (xử lý/estimation).
3. **Quyết định** cần đẩy tay về hướng nào, mạnh bao nhiêu (điều khiển).
4. **Đẩy tay** (chấp hành — actuator).
5. Lặp lại liên tục, rất nhanh.

Drone quadcopter làm y hệt, chỉ khác là:

| Bạn giữ gậy | Drone quadcopter |
|---|---|
| Mắt nhìn | IMU (gyro + accelerometer) đo vận tốc góc và hướng trọng lực |
| Não ước lượng | Estimator (complementary filter) tính roll/pitch/yaw |
| Quyết định | PID controller tính mức điều chỉnh cho 4 motor |
| Đẩy tay | 4 cánh quạt quay nhanh/chậm khác nhau |
| Lặp lại mỗi ~0.5 s | Lặp lại mỗi **1 ms (1 kHz)** |

Bốn cánh quạt tạo ra **lực đẩy** và **mô men**. Bằng cách tăng/giảm 4 motor lệch nhau,
drone nghiêng theo ý muốn. Đó là toàn bộ bí mật của bay đa cánh — phần còn lại là
kỹ thuật chính xác hóa.

### 1.1 Cây gậy chỉ có 1 chiều — drone có 3

- **Roll**: nghiêng trái/phải → drone trôi ngang trái/phải.
- **Pitch**: chúi mũi lên/xuống → drone trôi tới/lui.
- **Yaw**: xoay quanh trục đứng → đổi hướng mũi.
- **Throttle**: tổng lực đẩy → lên/xuống.

4 kênh điều khiển này map xuống **4 motor** qua một phép tính gọi là **mixer**.

---

## 2. Bài toán của repo này

Repo này **không** xây drone hoàn chỉnh. Nó xây **bộ não** (flight controller) và
**bãi thử an toàn** (SIL) cho bộ não đó:

```
                    ┌────────────────────────────────────────┐
                    │            FIRMWARE (C++20)            │
                    │  firmware/main/flight/                 │
                    │  estimator → failsafe → pid → mixer    │
                    │  + output gate (công tắc an toàn cuối) │
                    └───────────────┬────────────────────────┘
                                    │  IHal (10 hàm)
                    ┌───────────────┴────────────────┐
                    │                                │
              HwHal (chip ESP32-C3)            SilHal (PC)
              LEDC PWM · I2C IMU · UART RC     socket → plant Python
                    │                                │
              Board thật                      Plant 6-DOF + IMU + pin + RC
```

Nguyên tắc bất biến số 1 của repo:

> **Flight logic chỉ tồn tại một nơi duy nhất — C++ trong `flight/`. Python chỉ là
> plant + runner + log. Không có đường ra PWM nào đi vòng qua `OutputStage`.**

Điều này nghĩa là: khi thuật toán bay đã pass trên SIL, bạn **giữ nguyên 100% code đó**
để nạp lên chip. Chỉ thay lớp HAL. Không viết lại PID bằng Python. Đây là điểm khác
biệt giữa "làm mô phỏng cho vui" và "SIL nghiêm túc".

---

## 3. Ba tầng kiểm tra — tại sao không nhảy cóc?

```
Tầng A: SIL (PC)        firmware thật + plant Python, clock ảo
   │   Kiểm: logic thuật toán, dấu, gain, failsafe, log
   │   Không kiểm được: jitter chip, I2C timeout, rung thật
   ▼
Tầng B: HIL (chip thật) firmware thật chạy ESP32-C3, cảm biến/motor giả qua UART
   │   Kiểm: timing thật, watchdog, DMA, blocking, UART overflow
   │   Không kiểm được: khí động học thật, rung khung, pin thật
   ▼
Tầng C: Bay ràng buộc   drone thật, dây/lồng, cánh nhỏ
       Kiểm: mọi thứ còn lại — rung IMU, sag pin, lực đẩy thật
```

**Vì sao không nhảy thẳng lên tầng C?** Vì một dấu yaw ngược trong mixer sẽ khiến drone
lật ngay khi arm và có thể gây thương tích. SIL phát hiện điều đó trong 3 giây, trên
màn hình, với chi phí bằng 0.

**Vì sao không dừng ở tầng A?** Vì PC chạy nhanh hơn chip thật hàng nghìn lần và không
có jitter. Code đúng logic vẫn có thể chết trên chip vì blocking I2C, watchdog, hay
tràn UART. Repo này đã viết sẵn phase 9, 10 cho các tầng đó.

---

## 4. Kiến trúc repo — nhìn từ trên xuống

```
drone-mini/
├── firmware/
│   ├── main/
│   │   ├── app_main.cpp          # entry point trên chip (ESP32-C3)
│   │   ├── flight/               # ★ FLIGHT CORE — không biết gì về chip/plant
│   │   │   ├── ports.hpp         #   10 interface + POD types (hợp đồng)
│   │   │   ├── context.hpp       #   FlightContext — "hộp đựng" các module
│   │   │   ├── control_loop.cpp  #   ★ 1 tick — nơi mọi thứ ghép lại
│   │   │   ├── estimator.cpp     #   ước lượng tư thế
│   │   │   ├── pid.cpp           #   điều khiển rate + attitude
│   │   │   ├── mixer.cpp         #   phân phối lực ra 4 motor
│   │   │   ├── failsafe.cpp      #   arm/disarm + failsafe
│   │   │   ├── rc_parse.cpp      #   giải mã SBUS
│   │   │   ├── led.cpp           #   đèn báo trạng thái
│   │   │   ├── output.cpp        #   ★ công tắc PWM duy nhất
│   │   │   ├── flight_params.hpp #   gain PID (1 nguồn sự thật)
│   │   │   └── sensor_cfg.hpp    #   cấu hình IMU (1 nguồn sự thật)
│   │   └── hal/                  # adapter phần cứng
│   │       ├── sil_wire.hpp/cpp  #   định dạng packet SIL/HIL
│   │       ├── hal_sil.cpp       #   HAL cho PC (socket)
│   │       ├── hal_hw.cpp        #   HAL cho chip (stub — phase 9)
│   │       └── hal_mock.hpp      #   HAL giả cho test
│   ├── host/host_bindings.cpp    # cầu nối extern "C" cho pytest
│   └── CMakeLists.txt            # build host HOẶC chip
├── sim/main_sil.cpp              # entry point trên PC (SIL runner)
├── plant/                        # ★ MÔ HÌNH VẬT LÝ (Python)
│   ├── vehicle.py                #   6-DOF RK4
│   ├── drone_mini_params.py      #   tham số + registry giả định
│   ├── icm20948.py               #   mô hình IMU
│   ├── battery.py                #   mô hình pin 1S
│   ├── rc.py                     #   mã hóa SBUS + kịch bản bay
│   ├── sil_proto.py              #   mirror của sil_wire.hpp
│   ├── runner.py                 #   ★ vòng lockstep (server)
│   └── log.py                    #   ghi/đọc log CSV
├── tests/                        # pytest: unit + SIL e2e
├── tools/                        # plot_sil.py, compare_logs.py
└── docs/                         # SIL_STACK.md, measurements.md
```

Nhìn vào cây này, hãy nhớ 3 vùng:

1. **`firmware/main/flight/`** — bộ não. Chỉ C++ chuẩn. Không `#include` header ESP-IDF.
2. **`firmware/main/hal/`** — ống nhòm nối bộ não với thế giới. Có 3 adapter.
3. **`plant/` + `sim/` + `tests/`** — bãi thử và dụng cụ đo.

---

## 5. Một tick 1 ms chạy qua những gì?

Đây là phần **quan trọng nhất của file này**. Đọc chậm.

```
        PLANT (Python, server)                      FIRMWARE (C++, client)
        ──────────────────────                      ──────────────────────
  t=0   plant.step(1ms) một bước vật lý
        │
        │  đọc trạng thái thật → làm nhiễu như ICM-20948
        │  đóng gói STATE: imu(gyro,accel) + vbat + raw SBUS
        │
        ├──── STATE packet ─────────────────────────────►
        │                                              hal.recvState()
        │                                              loop.tick(ctx):
        │                                                1. hal.imuRead()
        │                                                2. hal.rcRawRead() → rc.feed()
        │                                                3. hal.vbatRead()
        │                                                4. fs.update()      ← failsafe
        │                                                5. est.update()     ← ước lượng
        │                                                6. att.update()     ← vòng ngoài
        │                                                7. rate.update()    ← vòng trong
        │                                                8. mixer.write()    ← 4 duty
        │                                                9. out.apply()      ← GATE an toàn
        │                                               10. led.set/update
        │                                               11. hal.setStatus()
        │◄──── FW_OUT packet (4 duty + armed/est/led) ──
        │
        │  duty → ω_cmd → motor bậc 1
        │  pin cập nhật theo dòng
        │  ghi 1 dòng log CSV
        ▼
  t=1ms  lặp lại
```

Chú ý thứ tự **rất quan trọng**:

- **Failsafe chạy TRƯỚC estimator và control.** Nếu mất sóng, ta không muốn tiêu tốn
  thời gian ước lượng nữa — nhưng vẫn phải chạy để log trạng thái. Quan trọng hơn:
  trạng thái arm/disarm phải được biết trước khi bất kỳ PWM nào được sinh ra.
- **Output gate chạy CUỐI CÙNG.** Dù tất cả module phía trên tính toán sai, gate vẫn
  chặn: `armed ∧ !failsafe ∧ imu_valid` — nếu không thỏa, 4 kênh = 0 và gọi
  `pwmOff()`.
- **LED cập nhật sau cùng** để phản ánh trạng thái của chính tick này.

Toàn bộ thứ tự này nằm trong **một hàm duy nhất**: `ControlLoop::tick()`
(`flight/control_loop.cpp:10`). Đây là file bạn nên đọc đầu tiên trong `flight/`.

---

## 6. Vòng đời của firmware — từ cắm điện đến bay

```
   ┌──────────┐   ┌───────────┐   ┌────────────┐   ┌────────┐   ┌──────────┐
   │  BOOT    │──►│ CALIBRATE │──►│  DISARMED  │──►│ ARMED  │──►│ FAILSAFE │
   │ LED nháy │   │ LED 500ms │   │ chờ switch │   │ bay    │   │ PWM = 0  │
   │ 200ms    │   │ 200 mẫu   │   │ + throttle │   │ LED sáng│  │ LED nháy │
   └──────────┘   └───────────┘   │ thấp       │   └────────┘   └──────────┘
                                  └────────────┘        │              │
                                        ▲               │              │
                                        └───────────────┴──────────────┘
                                            disarm / latch lỗi
```

Các trạng thái này là kết quả của 3 máy trạng thái (FSM) chạy song song:

| FSM | File | Nhiệm vụ |
|---|---|---|
| FailsafeFsm | `failsafe.cpp` | quyết định `armed` / `active` (đang failsafe) |
| Estimator | `estimator.cpp` | quyết định đã hiệu chuẩn xong chưa |
| LedFsm | `led.cpp` | hiển thị trạng thái ra đèn 2 màu |

Và một bộ đếm thời gian ảo: RC mất quá **100 ms** → failsafe. IMU lỗi **5 mẫu liên
tiếp** → coi như invalid. Pin dưới **3.3 V** → critical. Tất cả hằng số nằm trong
`flight/failsafe.hpp:20-28`.

---

## 7. Vai trò từng module — bảng tra nhanh

| Module | Trách nhiệm | Khái niệm UAV | File chính |
|---|---|---|---|
| Mixer | 4 lệnh điều khiển → 4 duty motor | Quad-X mixing | `mixer.cpp` |
| RC parser | bytes SBUS → roll/pitch/yaw/throttle/arm | Giao thức RC | `rc_parse.cpp` |
| Estimator | gyro + accel → roll/pitch/yaw | Sensor fusion | `estimator.cpp` |
| PID | sai số → lệnh điều chỉnh | Feedback control | `pid.cpp` |
| Failsafe | trạng thái an toàn + arm | Safety FSM | `failsafe.cpp` |
| OutputStage | cổng PWM duy nhất, fail-closed | Safety invariant | `output.cpp` |
| LED | trạng thái → ánh sáng | Human interface | `led.cpp` |
| HAL | chạm phần cứng/plant | Ports & Adapters | `hal_*.cpp` |
| Plant | mô hình vật lý 6-DOF | Flight dynamics | `vehicle.py` |
| IMU model | nhiễu + bias + lượng tử hóa | Sensor simulation | `icm20948.py` |
| Battery model | sag + brownout | Power simulation | `battery.py` |
| Runner | lockstep + log | SIL harness | `runner.py` |

---

## 8. Các con số phải nhớ

| Con số | Ý nghĩa | Ở đâu |
|---|---|---|
| **1 kHz** | tần số vòng điều khiển (1 ms/tick) | `flight_params.hpp:9` |
| **1125 Hz** | ODR thật của ICM-20948, decimate 9/8 về 1 kHz | `sensor_cfg.hpp:19` |
| **500 Hz** | tần số fusion attitude (decimate 2) | `estimator.hpp:19` |
| **100 ms** | timeout mất sóng RC → failsafe | `failsafe.hpp:25` |
| **200 mẫu** | số mẫu gyro để hiệu chuẩn bias lúc boot | `estimator.hpp:13` |
| **5 mẫu** | debounce IMU invalid | `failsafe.hpp:27` |
| **3.3 V** | ngưỡng pin critical | `failsafe.hpp:29` |
| **±5 dps** | bias gyro giả định (lớn gấp ~15 lần nhiễu) | `drone_mini_params.py:30` |
| **0.03 s** | hằng số thời gian motor brushed | `drone_mini_params.py:38` |
| **2500 rad/s** | vận tốc góc motor tối đa | `drone_mini_params.py:41` |
| **30° / 300 dps** | giới hạn góc và tốc độ quay từ stick | `pid.cpp:10-12` |

---

## 9. Checkpoint — trả lời được mới đi tiếp

1. Vì sao không được viết PID bằng Python trong repo này?
2. Kể tên 3 tầng kiểm tra và cái gì được kiểm ở mỗi tầng.
3. Trong 1 tick, module nào chạy cuối cùng trước khi PWM ra ngoài? Vì sao?
4. `armed = false` thì 4 kênh PWM bằng bao nhiêu? Cơ chế nào đảm bảo điều đó?
5. Nhìn vào cây thư mục, module nào "không biết gì về chip lẫn plant"?

<details>
<summary>Gợi ý đáp án</summary>

1. Vì sẽ có hai bộ não (C++ và Python) lệch nhau; code Python không chạy được trên
   chip. SIL chỉ có giá trị khi firmware được kiểm chính là firmware sẽ bay.
2. A: SIL trên PC (logic thuật toán). B: HIL trên chip (timing, jitter, watchdog,
   I/O). C: bay ràng buộc (khí động, rung, pin thật).
3. `OutputStage` — nó là công tắc an toàn cuối, chặn mọi trường hợp không an toàn dù
   các module trên tính sai.
4. Bằng 0. `OutputStage` gate `armed ∧ !failsafe ∧ imu_valid`, ngược lại gọi `pwmOff()`.
5. `firmware/main/flight/` — chỉ C++ chuẩn, chỉ biết `ports.hpp`.

</details>

---

*Tiếp theo: `02-cpp20-trong-repo.md` — học C++20 qua chính code này.*
