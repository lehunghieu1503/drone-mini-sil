# 10 — Kiến trúc, HAL và ràng buộc thời gian thực

> Đọc xong bạn hiểu vì sao cùng một firmware chạy được cả trên chip lẫn trên PC, cách
> "ports & adapters" hoạt động trong C++, và những luật realtime mà flight software
> phải tuân thủ. Đây là file về **kỹ thuật phần mềm nhúng**, không phải thuật toán.

---

## 1. Ports & Adapters (kiến trúc hexagonal)

### 1.1 Ý tưởng

Code nghiệp vụ (flight logic) **không được biết** nó đang chạy trên chip hay trên PC.
Nó chỉ biết các **cổng (port)** — interface trừu tượng. Bên ngoài cắm **adapter** vào:

```
                    ┌───────────────────────────────┐
                    │      FLIGHT CORE (domain)     │
                    │   estimator · pid · mixer     │
                    │   failsafe · rc · output      │
                    └──────────────┬────────────────┘
                                   │ chỉ biết ports.hpp
                    ┌──────────────┼──────────────┐
                    │              │              │
                SilHal          HwHal         MockHal
              (PC/plant)      (chip thật)   (test/unit)
```

Lợi ích:

1. **Test được không cần phần cứng.** MockHal giả lập mọi thứ.
2. **SIL khả thi.** SilHal nối với plant → kiểm thuật toán trước khi có board.
3. **Đổi phần cứng không đụng logic.** Nếu sau này đổi IMU khác, chỉ viết HwHal mới.
4. **Ranh giới rõ ràng.** Ai vi phạm (ví dụ gọi trực tiếp I2C trong PID) sẽ bị grep
   test bắt.

### 1.2 `IHal` — port duy nhất chạm thiết bị

`flight/ports.hpp:49`, 10 hàm:

| Hàm | Nhiệm vụ | `HwHal` (chip) | `SilHal` (PC) | `MockHal` (test) |
|---|---|---|---|---|
| `init()` | khởi tạo | cấu hình LEDC/I2C/UART | kết nối socket | return true |
| `nowUs()` | đồng hồ | `esp_timer_get_time()` | `state.imu.t_us` | biến test điều khiển |
| `pwmWrite()` | ghi 4 duty | `ledc_set_duty` ×4 | ghi buffer gửi plant | ghi log |
| `pwmOff()` | cắt motor | `ledc_stop(…, 0)` | 4 duty = 0 | ghi log |
| `imuRead()` | đọc IMU | I2C burst 14 byte | lấy từ STATE packet | dữ liệu test |
| `rcRawRead()` | đọc byte thô RC | UART DMA | lấy `rc_raw[32]` từ packet | dữ liệu test |
| `rcRead()` | (dự phòng) | — | luôn false | trả frame test |
| `ledSet()` | đặt LED | GPIO4 | ghi enum | ghi log |
| `vbatRead()` | đọc pin | INA219 qua I2C | `state.vbat` | giá trị test |
| `log()` | log | UART/console | stderr | ghi buffer |

Chú ý: **`rcRawRead` trả byte thô, không trả kênh đã giải mã.** Việc parse SBUS nằm
trong `flight/rc_parse.cpp` — nghĩa là **một codec duy nhất** dùng cho cả SIL, HIL và
chip. Nếu HAL tự parse, mỗi adapter sẽ parse một kiểu → lệch hành vi. Đây là quyết định
D5 của plan.

### 1.3 Các port khác

| Port | Impl | Trách nhiệm |
|---|---|---|
| `IMixer` | `QuadXMixer` | phân phối lực |
| `IPid` | `Pid` | một kênh PID |
| `IRateController` | `RateController` | 3 PID + mixer |
| `IAttitudeController` | `AttitudeController` | P vòng ngoài |
| `IEstimator` | `ComplementaryEstimator` | fusion + calibration |
| `IFailsafe` | `FailsafeFsm` | arm + failsafe |
| `ILed` | `LedFsm` | đèn |
| `IRcParser` | `SbusParser` | giải mã SBUS |
| `IOutputStage` | `OutputStage` | cổng PWM an toàn |

Mỗi port có **một impl duy nhất** trong v0 — nhưng ranh giới vẫn có giá trị: nó là
**hợp đồng** để test (mock) và để mở rộng (ví dụ thêm Mahony estimator sau này).

---

## 2. Dependency Injection qua `FlightContext`

### 2.1 Đọc code

`flight/context.hpp:14`:

```cpp
struct FlightContext {
  IHal& hal;
  IMixer& mixer;
  IRcParser& rc;
  IEstimator& est;
  IFailsafe& fs;
  ILed& led;
  IRateController& rate;
  IAttitudeController& att;
  IOutputStage& out;

  ControlMode mode = ControlMode::kOpenLoop;
  bool ol_from_rc = false;
  float ol_thr = 0.0f;
  // ...
};
```

`ControlLoop::tick(FlightContext& c)` nhận context và gọi `c.hal`, `c.est`, `c.rate`...
**ControlLoop không biết `SilHal` hay `HwHal` tồn tại.** Đây chính là DI.

### 2.2 Lắp ráp ở đâu?

Trên PC — `sim/main_sil.cpp:103`:

```cpp
static drone::QuadXMixer mixer;
static drone::SbusParser rc;
static drone::ComplementaryEstimator est;
static drone::FailsafeFsm fs;
static drone::LedFsm led;
static drone::RateController rate(mixer);
static drone::AttitudeController att;
static drone::OutputStage out;

drone::FlightContext ctx{hal, mixer, rc, est, fs, led, rate, att, out};
```

Trên chip — `app_main.cpp:12` giống hệt, chỉ khác `HwHal hal;`.

**Nhận xét quan trọng:** hai entry point khác nhau nhưng **cùng một `ControlLoop`**.
Thứ tự tick, logic, hằng số — tất cả giống hệt. Đây là điều kiện tiên quyết để SIL có
giá trị.

### 2.3 Vì sao object là `static`?

- Không cấp phát động → không phân mảnh, không thời gian bất định.
- Tạo một lần trong `main()` → thứ tự khởi tạo do bạn kiểm soát (tránh static-init
  order fiasco giữa các translation unit).
- `RateController rate(mixer)` — constructor nhận reference tới mixer, chứng minh thứ
  tự: `mixer` phải sống trước `rate`. Trong `main()`, thứ tự khai báo đảm bảo điều đó.

---

## 3. Dual build — một source, ba đích

### 3.1 Ba build

| Build | Lệnh | Entry | HAL | Dùng cho |
|---|---|---|---|---|
| **host lib** | `make host` | — (shared lib) | MockHal | pytest qua ctypes |
| **sim** | `make sil` | `sim/main_sil.cpp` | SilHal | SIL lockstep |
| **chip** | `make fw` (cần ESP-IDF) | `firmware/main/app_main.cpp` | HwHal | bay thật |

`firmware/CMakeLists.txt` rẽ nhánh:

```cmake
if(DRONE_HOST_LIB)
  project(drone_host LANGUAGES CXX)         # build bằng g++ trên PC
  ...
  add_library(drone_flight SHARED ...)
else()
  include($ENV{IDF_PATH}/tools/cmake/project.cmake)   # build bằng ESP-IDF
  project(drone_fw)
endif()
```

Và `sim/CMakeLists.txt` tái sử dụng chính `firmware/main/flight/`:

```cmake
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../firmware/main/flight
                 ${CMAKE_CURRENT_BINARY_DIR}/flight_build)
add_executable(sil_runner
  main_sil.cpp
  ../firmware/main/hal/sil_wire.cpp
  ../firmware/main/hal/hal_sil.cpp)
```

### 3.2 Luật bất biến: `flight/` không được include header ESP-IDF

Quy tắc trong `SIL_STACK.md` §4: *"`flight/` không `#include` header ESP-IDF, không gọi
`ledc_*`, `i2c_*`"*. Vi phạm = build host vỡ ngay (không có header IDF trên PC), và có
grep test chặn.

Nếu bạn cần hằng số của chip (ví dụ giá trị `esp_reset_reason_t`), hãy **mirror** nó
vào `flight/` — xem `failsafe.hpp:20`:

```cpp
// esp_reset_reason_t values (mirrored; flight/ cannot include IDF headers).
inline constexpr uint32_t kResetIntWdt = 5;
inline constexpr uint32_t kResetTaskWdt = 6;
inline constexpr uint32_t kResetWdt = 7;
inline constexpr uint32_t kResetBrownout = 9;
```

Đây là kỹ thuật "mirror hằng số phần cứng" — chấp nhận sao chép có kiểm soát để giữ
ranh giới sạch. Đổi lại: nếu IDF đổi giá trị, phải cập nhật tay (rủi ro nhỏ, có
comment ghi rõ).

### 3.3 Vì sao không dùng `idf.py set-target linux` cho SIL?

Plan (`SIL_STACK.md` §4) ghi rõ: ESP-IDF linux target **không giả lập LEDC/I2C/UART**
— component `driver` chỉ là mock rỗng. Vậy nó không thể làm plant motor. Cách sạch là
CMake native + `hal_sil`. Chỉ nên dùng linux target để test phần IDF thuần (timer,
log), không phải để bay.

---

## 4. OutputStage — bất biến an toàn trung tâm

### 4.1 Đọc code

`flight/output.cpp:7`:

```cpp
bool OutputStage::apply(const PwmCmd& in, bool armed, bool failsafe, bool imu_valid, IHal& hal) {
  if (!armed || failsafe || !imu_valid) {
    hal.pwmOff();
    return false;
  }
  PwmCmd safe{};
  for (int i = 0; i < 4; ++i) {
    const float v = in.mot[i];
    if (!std::isfinite(v) || v < 0.0f || v > 1.0f) {
      hal.pwmOff();
      return false;
    }
    safe.mot[i] = v;
  }
  hal.pwmWrite(safe);
  return true;
}
```

### 4.2 Bất biến (invariant)

```
(armed = false)  ∨  failsafe  ∨  (imu_valid = false)   ⟹   4 kênh PWM = 0
```

Đây là **định lý an toàn** của hệ thống. Nó được kiểm bằng test `test_output_gate.py`
cho **mọi tổ hợp** cờ (armed × failsafe × imu_valid = 8 trường hợp) — không phải chỉ
một vài trường hợp tiêu biểu.

### 4.3 Vì sao phải có tầng riêng?

Vì "phòng thủ nhiều lớp": dù code phía trên có bug (mixer tính sai, PID NaN, mode
lạ...), gate vẫn chặn. Nếu để logic gate rải rác trong mixer/PID/failsafe, chỉ cần một
đường đi quên kiểm tra là đủ gây tai nạn. Red team finding #1 (Critical) chính là về
điều này: *"Không có output stage fail-closed; open-loop/closed-loop có thể ra duty khi
armed=0"*.

Câu thần chú trong README repo:

> **No PWM path bypasses `OutputStage`.**

### 4.4 Ai gọi `apply()`?

Chỉ `ControlLoop::tick` (`control_loop.cpp:75`):

```cpp
c.out.apply(pwm, c.fs.armed(), c.fs.active(), c.est.imuValid(), c.hal);
```

Mọi mode (open-loop, rate, attitude) đều đi qua đây. Không có nhánh nào ghi PWM trực
tiếp. Grep `pwmWrite` trong `flight/` — chỉ `output.cpp` gọi.

---

## 5. Realtime policy — luật của vòng 1 kHz

### 5.1 Các luật

| Luật | Lý do | Nơi áp dụng |
|---|---|---|
| Không exception | Thời gian chạy bất định | `-fno-exceptions` |
| Không RTTI | Tốn flash, không cần | `-fno-rtti` |
| Không heap sau init | Phân mảnh + bất định | object static |
| Không `std::function` | Có thể cấp phát | dùng interface |
| Không `std::atomic` trên C3 | RV32IMC không lock-free | dùng `portMUX` nếu cần |
| Không virtual trong ISR | vtable ở flash → cache miss | chỉ gọi virtual ở task loop |
| `constinit` | Tránh static-init order | plan D27 |

### 5.2 Jitter — kẻ thù của vòng 1 kHz

Vòng lặp phải xong trong 1 ms. Nếu một tick mất 2 ms, drone nhận dữ liệu trễ → điều
khiển dao động. Nguồn jitter trên chip:

- I2C burst đọc IMU (410 µs) — phải không blocking quá lâu.
- UART nhận SBUS.
- Log/telemetry.
- Watchdog, scheduler của FreeRTOS.

Trên PC (SIL) gần như không có jitter — **đây là lý do SIL không thay được HIL**.
Plan R8 ghi: *"SIL host chạy nhanh hơn đời thật nên không thay HIL"*.

### 5.3 Cấu hình chip liên quan

`sdkconfig.defaults`:

```
CONFIG_FREERTOS_HZ=1000              # tick 1 ms -> vTaskDelay(1) = 1 ms
CONFIG_ESP_TASK_WDT_PANIC=y          # watchdog panic thay vì chỉ warn
CONFIG_ESP_TASK_WDT_TIMEOUT_S=2
CONFIG_COMPILER_CXX_EXCEPTIONS=n
CONFIG_COMPILER_CXX_RTTI=n
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y # giải phóng UART1 cho SBUS
```

`app_main.cpp:36`:

```cpp
for (;;) {
  loop.tick(ctx, false);
  vTaskDelay(1);  // 1 kHz tick at CONFIG_FREERTOS_HZ=1000
}
```

Trên chip, `vTaskDelay(1)` nhường CPU cho task khác rồi quay lại — không phải busy
loop, nhưng cũng không đảm bảo chính xác tuyệt đối (scheduler có thể trễ vài chục µs).
Đo jitter thật là việc của phase 9.

---

## 6. Testability — thành quả của kiến trúc

Nhờ ports & adapters, repo test được gần như mọi thứ:

| Cần test | Dùng adapter | File test |
|---|---|---|
| Mixer, PID, estimator, failsafe | MockHal + host bindings | `tests/test_*.py` (unit) |
| Toàn hệ thống với vật lý | SilHal + plant | `tests/test_*_sil.py` (slow) |
| ABI giữa C++ và Python | sil_wire + sil_proto | `tests/test_abi.py` |
| Bất biến an toàn | MockHal | `tests/test_output_gate.py` |
| HIL (chưa làm) | HwHal + bridge | phase 9 |

`MockHal` (`hal_mock.hpp`) là một HAL ghi log mọi lời gọi:

```cpp
struct Log {
  int pwm_writes = 0;
  int pwm_offs = 0;
  int led_sets = 0;
  PwmCmd last_pwm{};
  LedMode last_led = LedMode::kBoot;
  // ...
};
```

Nhờ nó, test có thể khẳng định: "sau 5 mẫu IMU invalid, `pwm_offs` tăng và
`last_pwm.mot` toàn 0" — chính xác đến từng lời gọi hàm.

---

## 7. Checkpoint

1. Ports & Adapters là gì? Kể 3 adapter của `IHal` trong repo.
2. Vì sao `rcRawRead` trả byte thô thay vì kênh đã giải mã? Lợi ích?
3. DI hoạt động thế nào qua `FlightContext`? Vì sao dùng reference thay vì con trỏ?
4. Kể 3 luật realtime và lý do của chúng.
5. Viết bất biến an toàn của `OutputStage` bằng công thức logic.
6. Vì sao SIL trên PC không thay thế được HIL?

<details>
<summary>Gợi ý đáp án</summary>

1. Kiến trúc tách domain logic khỏi chi tiết triển khai qua interface. Ba adapter:
   `SilHal` (PC/plant), `HwHal` (chip), `MockHal` (test).
2. Để chỉ có một codec SBUS duy nhất dùng chung SIL/HIL/chip; nếu HAL parse riêng thì
   hành vi sẽ lệch giữa các tầng.
3. `FlightContext` giữ reference tới mọi module; `ControlLoop` nhận context và gọi
   qua interface, không biết impl cụ thể. Reference vì không bao giờ null và thể hiện
   quan hệ "không sở hữu".
4. Không exception (thời gian bất định), không heap (phân mảnh + bất định), không
   atomic trên C3 (không lock-free). (Còn: không virtual trong ISR, không
   std::function.)
5. `(armed ∧ ¬failsafe ∧ imu_valid) = false ⟹ mot[i] = 0 ∀i`.
6. Vì PC không có jitter, không có blocking I2C/UART, không có watchdog — những vấn
   đề chỉ xuất hiện trên chip thật.

</details>

---

*Tiếp theo: `11-sil-lockstep-wire-protocol.md` — cỗ máy SIL.*
