# 02 — C++20 qua chính repo này

> Bạn không cần học hết C++ trước khi đọc repo. Repo dùng một tập con C++ rất kỷ luật.
> File này dạy bạn đúng tập con đó, qua code thật. Đọc xong, bạn đọc được toàn bộ
> `firmware/main/flight/`.

---

## 1. Vì sao flight firmware dùng C++ (chứ không phải C hay Python)?

| Tiêu chí | Python | C | C++20 (repo này) |
|---|---|---|---|
| Chạy trên vi điều khiển | Không | Có | Có |
| Trừu tượng hóa (interface) | Có | Không (phải tự chế) | Có, zero-cost |
| Kiểm tra lúc compile | Không | Ít | Nhiều (`constexpr`, `static_assert`) |
| Quản lý bộ nhớ xác định | Không | Có | Có (không dùng heap) |
| Test được trên PC | Không mô phỏng được firmware | Khó | Dễ — cùng code, đổi HAL |

Điểm mấu chốt: **trừu tượng không tốn runtime**. Một hàm `virtual` trong C++ chỉ là
một con trỏ trong bảng (vtable) — chi phí vài nano giây, đổi lại bạn có kiến trúc sạch
để test. Repo chấp nhận chi phí đó ở biên module (gọi 1 kHz), và cấm nó trong ISR.

---

## 2. Cấu trúc file: header và source

Quy ước của repo (xem `flight/mixer.hpp` và `flight/mixer.cpp`):

```cpp
// mixer.hpp — KHAI BÁO (declaration): "có gì"
#pragma once                       // thay cho include guard thủ công

#include "flight/ports.hpp"        // chỉ include cái mình cần

namespace drone {                  // tránh trùng tên toàn cục

class QuadXMixer final : public IMixer {
 public:
  float write(float thr, float roll, float pitch, float yaw, PwmCmd& out) override;

 private:
  static float clampf(float v, float lo, float hi);
  static float desaturate(float m[4]);
};

}  // namespace drone
```

```cpp
// mixer.cpp — ĐỊNH NGHĨA (definition): "làm thế nào"
#include "flight/mixer.hpp"
#include <cmath>

namespace drone {

float QuadXMixer::clampf(float v, float lo, float hi) { /* ... */ }

}  // namespace drone
```

**Vì sao tách?** Header được include ở nhiều nơi; source chỉ biên dịch một lần. Với
firmware nhúng, giữ header sạch giúp thời gian build và kích thước code dễ kiểm soát.

**`namespace drone`**: mọi thứ trong firmware nằm trong namespace này. Khi đọc code,
hễ thấy tên không rõ nguồn gốc thì mặc định nó thuộc `drone`.

---

## 3. POD types và số nguyên có độ rộng cố định

Mở `flight/ports.hpp:12`:

```cpp
struct ImuSample {
  float gyro_rps[3];    // FRD, rad/s
  float accel_mps2[3];  // FRD, m/s^2
  float mag_uT[3];      // unused in v0
  float temp_c;
  uint64_t t_us;
  bool valid;
};
```

Ba điều cần chú ý:

1. **POD (Plain Old Data)** — struct chỉ chứa dữ liệu, không hàm ảo, không constructor
   phức tạp. Dữ liệu nằm liền một khối trong bộ nhớ, có thể `memcpy` hoặc gửi qua
   socket nguyên xi. Đây là nền tảng của wire protocol (file 11).
2. **Hậu tố đơn vị trong tên biến** — `_rps` (rad/s), `_mps2` (m/s²), `_uT` (µT),
   `_us` (microsecond), `_c` (Celsius). Đây là quy ước **sống còn** trong UAV: bug
   đơn vị là bug chết người. Hãy bắt chước quy ước này.
3. **Số nguyên có độ rộng cố định** — `uint64_t`, `uint8_t`, `uint32_t` (từ `<cstdint>`)
   thay vì `int`, `long`. Trên vi điều khiển, kích thước `int` phụ thuộc nền tảng;
   dùng `uint32_t` đảm bảo 32 bit ở mọi nơi — và bắt buộc khi layout struct phải khớp
   giữa C++ và Python.

`t_us` là `uint64_t` vì thời gian microsecond tràn số 32-bit sau ~71 phút. Một buổi
bay có thể lâu hơn thế.

---

## 4. Class, `final`, `override` — đọc một interface

Đây là trái tim kiến trúc. `flight/ports.hpp:49`:

```cpp
class IHal {
 public:
  virtual ~IHal() = default;             // destructor ảo: xóa qua con trỏ base an toàn
  virtual bool init() = 0;               // = 0 nghĩa là "pure virtual"
  virtual uint64_t nowUs() = 0;
  virtual void pwmWrite(const PwmCmd& out) = 0;
  virtual void pwmOff() = 0;
  virtual bool imuRead(ImuSample& out) = 0;
  virtual int rcRawRead(uint8_t* buf, int cap) = 0;
  virtual bool rcRead(RcSample& out) = 0;
  virtual void ledSet(LedMode mode) = 0;
  virtual bool vbatRead(float& volts) = 0;
  virtual void log(const char* msg) = 0;
};
```

Giảng giải từng phần:

- **`virtual ... = 0`**: hàm "thuần ảo" — `IHal` chỉ định nghĩa *hợp đồng*, không có
  thân hàm. Không thể tạo object `IHal` trực tiếp. Đây giống "interface" trong Java.
- **`virtual ~IHal() = default`**: destructor ảo. Nếu xóa một `SilHal` thông qua con
  trỏ `IHal*`, destructor ảo đảm bảo destructor của `SilHal` được gọi. `= default` bảo
  compiler tự sinh.
- **`bool imuRead(ImuSample& out)`**: truyền tham chiếu để hàm *ghi kết quả vào* `out`
  và *trả về* thành công/thất bại. Vì sao không trả về struct? Vì struct lớn và cần
  phân biệt "đọc được nhưng giá trị 0" với "đọc lỗi".

Lớp hiện thực (`hal_sil.hpp:12`):

```cpp
class SilHal final : public IHal {
 public:
  bool init() override { return fd_ >= 0; }
  uint64_t nowUs() override { return state_.imu.t_us; }
  // ...
};
```

- **`final`**: không cho lớp khác kế thừa `SilHal` nữa. Với firmware, điều này cho
  compiler cơ hội "devirtualize" (bỏ gọi gián tiếp) và thể hiện ý định: adapter này
  là lá cuối của cây kế thừa.
- **`override`**: bắt buộc compiler kiểm tra hàm này thật sự ghi đè hàm ảo của lớp
  cha. Nếu bạn gõ sai chữ ký (ví dụ quên `const`), compiler báo lỗi thay vì âm thầm
  tạo hàm mới không bao giờ được gọi. **Luôn dùng `override`.**

Các interface khác trong `ports.hpp`: `IMixer`, `IPid`, `IRateController`,
`IAttitudeController`, `IEstimator`, `IFailsafe`, `ILed`, `IRcParser`, `IOutputStage`.
Đọc tên là đoán được trách nhiệm. File 10 giảng sâu về kiến trúc này.

---

## 5. Tham chiếu (`&`) — không copy, không null

```cpp
class RateController final : public IRateController {
 public:
  explicit RateController(IMixer& mixer);   // giữ THAM CHIẾU tới mixer
  // ...
 private:
  IMixer& mixer_;                            // thành viên là reference
};
```

- `IMixer& mixer` nghĩa là: "tôi dùng mixer của người khác, tôi không sở hữu nó,
  tôi không copy nó, và nó chắc chắn tồn tại".
- **`explicit`**: chặn chuyển đổi ngầm. Nếu không có `explicit`, ai đó có thể viết
  `RateController r = someMixer;` và compiler tự gọi constructor — không ai muốn thế.
- Reference không bao giờ null (khác con trỏ), nên code bên trong không cần kiểm tra
  `if (mixer_)`. Đây là lý do repo dùng reference cho DI, và con trỏ chỉ khi thật cần
  (ví dụ `SilHal` giữ `fd_` là số nguyên, không phải con trỏ).

So sánh nhanh:

| | `T&` | `T*` | `const T&` |
|---|---|---|---|
| Có thể null | Không | Có | Không |
| Đổi được đối tượng trỏ tới | Không | Có | Không (chỉ đọc) |
| Dùng khi | thành viên/DI | buffer, optional | tham số chỉ đọc, struct lớn |

Trong `pid.cpp:71` bạn thấy cả hai:

```cpp
float RateController::update(const ImuSample& imu, const RateSp& sp, PwmCmd& out)
```

- `const ImuSample& imu` — chỉ đọc, không copy (struct 49 byte).
- `PwmCmd& out` — sẽ bị ghi.

---

## 6. `constexpr` — hằng số và cấu hình một nguồn sự thật

`flight/flight_params.hpp:9`:

```cpp
inline constexpr float kControlDtS = 0.001f;  // 1 kHz control loop

struct flight_params_t {
  float rate_kp[3];
  float rate_ki[3];
  // ...
};

inline constexpr flight_params_t kFlightParams = {
    /*rate_kp=*/{0.15f, 0.15f, 0.16f},
    /*rate_ki=*/{2.40f, 2.40f, 2.00f},
    // ...
};
```

- **`constexpr`**: giá trị được tính lúc **biên dịch**, không tốn chu kỳ CPU lúc chạy,
  và có thể nằm trong bộ nhớ flash của chip.
- **`inline`** (C++17): cho phép định nghĩa biến trong header mà không vi phạm ODR
  (mọi translation unit dùng chung một thực thể). Trước C++17 phải khai báo trong
  header, định nghĩa trong .cpp — dễ quên.
- Tiền tố **`k`** là quy ước đặt tên cho hằng số (kControlDtS, kFlightParams,
  kRcTimeoutUs). Bạn sẽ gặp nó khắp repo.

**Vì sao gain PID để trong header mà không để trong biến?** Vì chỉ có **một nguồn sự
thật**. Test parity (`tests/test_param_parity.py`) đọc chính struct này để so với tham
số bên Python. Không có chuyện C++ dùng gain A còn Python tưởng gain B.

Hằng số cũng là **hợp đồng**: `kControlDtS = 0.001f` xuất hiện ở cả firmware và plant
(`DT_US = 1000`). Nếu một bên đổi, test ABI/param parity sẽ bắt được.

---

## 7. Packed struct + `static_assert` — ABI giữa C++ và Python

Đây là kỹ thuật quan trọng nhất để hiểu file 11. `hal/sil_wire.hpp:32`:

```cpp
#pragma pack(push, 1)          // tắt padding

struct Hdr {
  uint16_t magic;
  uint8_t ver;
  uint8_t type;
  uint16_t len;
  uint32_t seq;
  uint32_t crc;
};

struct SilImu {
  float gyro_rps[3];
  float accel_mps2[3];
  float mag_uT[3];
  float temp_c;
  uint64_t t_us;
  uint8_t valid;
};

#pragma pack(pop)

static_assert(sizeof(Hdr) == 14, "wire Hdr layout drift");
static_assert(sizeof(SilImu) == 49, "wire SilImu layout drift");
// ... thêm 6 dòng nữa
static_assert(sizeof(bool) == 1, "ABI assumes 1-byte bool");
static_assert(sizeof(float) == 4, "ABI assumes 32-bit float");
```

Giảng giải:

- **Padding**: bình thường compiler chèn byte trống để các trường căn lề (alignment),
  ví dụ `Hdr` có thể thành 16 byte. `#pragma pack(1)` cấm điều đó → layout khớp chính
  xác từng byte với `struct.pack("<HBBHII")` bên Python.
- **`static_assert`**: chạy lúc biên dịch. Nếu ai thêm trường vào `SilImu` mà không
  cập nhật kích thước, build **thất bại ngay** — không thể lọt ra production. Đây là
  "test" rẻ nhất và mạnh nhất trong repo.
- **`sizeof(bool) == 1`**: tiêu chuẩn C++ không đảm bảo điều này; repo chốt bằng
  assert vì wire format dùng `?` (1 byte) bên Python.

Bên Python, `plant/sil_proto.py:48` có các assert tương ứng:

```python
assert HDR_SIZE == 14, HDR_SIZE
assert IMU_SIZE == 49, IMU_SIZE
```

Hai bên cùng "thề" giữ layout. Nếu lệch, chương trình dừng ngay khi khởi động chứ
không âm thầm đọc sai dữ liệu. Triết lý: **fail loud, fail early**.

---

## 8. `std::isfinite` và NaN — phòng thủ số học

Firmware nhận dữ liệu từ thế giới bên ngoài (cảm biến, socket). Dữ liệu đó có thể là
NaN (Not a Number) hoặc vô cực. Repo phòng thủ ở mọi biên. Ví dụ `pid.cpp:31`:

```cpp
float Pid::step(float sp, float meas, float dt, bool sat_pos, bool sat_neg) {
  if (!std::isfinite(sp) || !std::isfinite(meas) || !(dt > 0.0f)) return 0.0f;
  // ...
  return std::isfinite(out) ? out : 0.0f;
}
```

Giảng giải:

- `std::isfinite(x)` trả về `false` nếu x là NaN hoặc ±Inf.
- `!(dt > 0.0f)` — chú ý viết kiểu này chứ không phải `dt <= 0.0f`. Vì nếu `dt` là
  NaN, mọi so sánh đều false; cách viết này bắt được cả NaN. Đây là mẹo phòng thủ
  đáng học.
- Khi đầu vào bất thường, PID trả 0 chứ không trả NaN — vì NaN sẽ lan truyền qua
  mixer, qua PWM, và làm motor nhảy loạn.

Tương tự ở `mixer.cpp:7`:

```cpp
float QuadXMixer::clampf(float v, float lo, float hi) {
  if (!std::isfinite(v)) return 0.0f;
  // ...
}
```

và `output.cpp:15`:

```cpp
if (!std::isfinite(v) || v < 0.0f || v > 1.0f) {
  hal.pwmOff();
  return false;
}
```

Ba lớp phòng thủ chồng nhau: PID chặn NaN → mixer chặn NaN → output gate chặn lần
cuối. **Defense in depth.**

---

## 9. Vì sao không exception, không RTTI, không heap?

`firmware/CMakeLists.txt`:

```cmake
target_compile_options(drone_flight PRIVATE
  -Wall -Wextra -Werror -fno-exceptions -fno-rtti)
```

`sdkconfig.defaults`:

```
CONFIG_COMPILER_CXX_EXCEPTIONS=n
CONFIG_COMPILER_CXX_RTTI=n
```

Lý do cho từng thứ:

| Tắt gì | Vì sao |
|---|---|
| **Exceptions** (`try/catch/throw`) | Exception có thể ném ở bất kỳ đâu → thời gian chạy bất định. Vòng 1 kHz cần worst-case biết trước. Xử lý lỗi bằng giá trị trả về (`IoResult`, `bool`) |
| **RTTI** (`dynamic_cast`, `typeid`) | Tốn flash và bảng metadata; repo không cần kiểm tra kiểu động |
| **Heap** (`new`/`delete`/`malloc`) | Cấp phát động gây phân mảnh và thời gian bất định. Mọi object là `static` (xem dưới) |
| **`std::function`** | Có thể cấp phát heap; repo dùng interface + vtable thay thế |
| **`std::atomic`** | Trên ESP32-C3 (RV32IMC), atomic không lock-free → sẽ sinh lock, phá realtime. Nếu cần đồng bộ dùng `portMUX` (plan D27) |

Trong repo bạn sẽ thấy object được tạo **một lần** ở `main`, ví dụ
`sim/main_sil.cpp:103`:

```cpp
static drone::QuadXMixer mixer;
static drone::SbusParser rc;
static drone::ComplementaryEstimator est;
// ...
drone::FlightContext ctx{hal, mixer, rc, est, fs, led, rate, att, out};
```

`static` ở đây nghĩa là object sống suốt chương trình, cấp phát tĩnh, không bao giờ
`new`/`delete`. `FlightContext` chỉ giữ **reference** tới chúng — không copy.

Điểm cần biết: thứ tự khởi tạo biến `static` toàn cục trong C++ không xác định giữa
các translation unit. Repo tránh bẫy này bằng cách tạo object trong `main()`, nơi thứ
tự do bạn kiểm soát. Plan D27 còn yêu cầu `constinit` cho các trường hợp khác.

---

## 10. Code tour — thứ tự đọc đề xuất

Đọc từng file một, mỗi file chỉ 30–130 dòng:

| Bước | File | Bạn học được |
|---|---|---|
| 1 | `flight/ports.hpp` | Toàn bộ "từ vựng" của hệ thống: 10 interface + 4 POD |
| 2 | `flight/context.hpp` | Cách ghép module bằng DI |
| 3 | `flight/control_loop.cpp` | Trình tự 1 tick — bản đồ của mọi thứ |
| 4 | `flight/output.cpp` | Gate an toàn — ngắn nhất nhưng quan trọng nhất |
| 5 | `flight/mixer.cpp` | Toán học phân phối lực |
| 6 | `flight/pid.cpp` | Thuật toán điều khiển |
| 7 | `flight/estimator.cpp` | Sensor fusion |
| 8 | `flight/failsafe.cpp` | FSM an toàn |
| 9 | `flight/rc_parse.cpp` | Xử lý bit |
| 10 | `flight/led.cpp` | FSM đơn giản nhất — đọc cho vui |
| 11 | `hal/hal_mock.hpp` | Cách HAL giả hoạt động |
| 12 | `hal/hal_sil.hpp` + `sil_wire.hpp` | Giao tiếp plant |
| 13 | `sim/main_sil.cpp` | Ghép tất cả lại |

Khi đọc, tự hỏi 3 câu cho mỗi hàm:
1. **Input là gì, output là gì, đơn vị gì?**
2. **Nếu input xấu (NaN, null, quá lớn) thì sao?**
3. **Ai gọi hàm này, tần số bao nhiêu?**

---

## 11. Bài tập nhỏ (làm ngay, 15–30 phút)

1. **Đọc interface**: mở `ports.hpp`, liệt kê 10 interface và viết 1 câu mô tả mỗi
   cái. Không nhìn đáp án — đây là cách kiểm tra bạn nắm "từ vựng" hệ thống.
2. **Tìm chỗ phòng thủ**: grep toàn bộ `flight/` tìm `isfinite` và `!(`. Đếm số chỗ
   chặn dữ liệu xấu. Giải thích vì sao `!(dt > 0.0f)` khác `dt <= 0.0f`.
3. **Thử phá build**: thêm một trường `float x;` vào `SilImu` trong `sil_wire.hpp`.
   Chạy `make host`. Ghi lại lỗi `static_assert`. Sau đó xóa trường vừa thêm.
4. **Tìm nguồn sự thật**: đổi `kControlDtS` thành `0.002f` trong `flight_params.hpp`,
   chạy `make test`. Ghi lại test nào fail. Khôi phục.
5. **Đọc `constinit`**: grep repo tìm `constinit` (gợi ý: plan D27 nhắc nó). Nếu chưa
   có trong code, giải thích vì sao `static` object trong `main()` đã đủ an toàn.

---

## 12. Checkpoint

1. `virtual bool init() = 0;` — giải thích từng token.
2. `final` và `override` khác nhau thế nào? Vì sao dùng cả hai?
3. Tại sao truyền `const ImuSample&` thay vì `ImuSample`?
4. `static_assert(sizeof(SilImu) == 49)` bảo vệ điều gì? Điều gì xảy ra nếu ai thêm
   trường mà quên cập nhật?
5. Vì sao repo cấm `new`/`delete` trong vòng bay?

<details>
<summary>Gợi ý đáp án</summary>

1. `virtual`: có thể bị ghi đè; `bool init()`: hàm không tham số trả bool; `= 0`:
   thuần ảo, không có thân, lớp con bắt buộc hiện thực.
2. `override` = kiểm tra đang ghi đè đúng hàm ảo của lớp cha; `final` = cấm kế thừa
   tiếp (trên lớp) hoặc cấm ghi đè tiếp (trên hàm). Dùng `override` để bắt lỗi chữ
   ký; `final` để chốt thiết kế và giúp devirtualize.
3. Struct 49 byte; truyền theo giá trị là copy; `const&` tránh copy và đảm bảo chỉ đọc.
4. Bảo vệ ABI giữa C++ và Python. Nếu thêm trường mà không sửa assert, build fail ngay
   — ngăn layout lệch làm Python đọc rác.
5. Heap gây phân mảnh bộ nhớ và thời gian cấp phát không xác định, phá ràng buộc
   realtime 1 kHz; trên chip nhỏ còn dễ hết RAM.

</details>

---

*Tiếp theo: `03-toa-do-va-dong-luc-hoc-6dof.md` — vật lý bay.*
