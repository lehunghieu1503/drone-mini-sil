# 09 — RC, SBUS, Arm và Failsafe

> Đọc xong bạn hiểu giao thức SBUS từng bit một, cách firmware quyết định "được phép
> quay motor hay không", và toàn bộ các con đường dẫn đến failsafe. Đây là file về
> **an toàn** — phần nghiêm túc nhất của flight software.

---

## 1. RC là gì?

RC (Remote Control) là bộ phát cầm tay + bộ thu trên drone. Bộ thu xuất ra tín hiệu
serial; firmware đọc và giải mã thành 5 thứ cần thiết:

| Kênh | Ý nghĩa | Dải |
|---|---|---|
| 1 (ch0) | Roll stick | −1 … +1 |
| 2 (ch1) | Pitch stick | −1 … +1 |
| 3 (ch2) | Throttle | 0 … 1 |
| 4 (ch3) | Yaw stick | −1 … +1 |
| 5 (ch4) | **Arm switch** | bật/tắt |

Repo chỉ hỗ trợ **SBUS** trong v0 (plan D23: không viết codec chưa dùng). Protocol
thật trên module RC (`J6`) là giả định **A8** — phải sniff bằng logic analyzer trước
khi bay thật.

---

## 2. SBUS — giao thức 25 byte

### 2.1 Cấu trúc frame

```
Byte:   0     1..22                23      24
      ┌─────┬────────────────────┬───────┬──────┐
      │0x0F │ 16 kênh × 11 bit   │ flags │ 0x00 │
      └─────┴────────────────────┴───────┴──────┘
        header      data (176 bit)    flags  footer
```

- **25 byte**, gửi ở 100 kbaud, tín hiệu **đảo** (inverted UART). Trên ESP32-C3 có
  thể đảo bằng phần mềm (`uart_set_line_inverse`) — plan D7, không cần mạch inverter
  ngoài.
- **16 kênh × 11 bit = 176 bit = 22 byte** dữ liệu.
- Mỗi kênh giá trị **172 … 1811**, giữa là **992**.
- `flags` byte: bit2 = frame-lost, bit3 = failsafe. Nửa cao (bit 4–7) phải bằng 0.

### 2.2 Giải nén bit — kỹ thuật đáng học

Các kênh **không căn byte**. Kênh 0 dùng bit 0–10, kênh 1 dùng bit 11–21, kênh 2 dùng
bit 22–32... nên một kênh có thể nằm vắt qua 3 byte. Đọc code `channel()`
(`rc_parse.cpp:22`):

```cpp
uint16_t SbusParser::channel(const uint8_t* f, unsigned i) {
  const unsigned bit = 11u * i;          // vị trí bit của kênh i
  const unsigned byte = 1u + (bit >> 3); // byte bắt đầu (bỏ qua header)
  const unsigned shift = bit & 7u;       // lệch trong byte đó
  uint32_t raw = 0;
  raw |= static_cast<uint32_t>(f[byte]);
  raw |= static_cast<uint32_t>(f[byte + 1]) << 8;
  if (shift != 0) raw |= static_cast<uint32_t>(f[byte + 2]) << 16;
  return static_cast<uint16_t>((raw >> shift) & 0x7FFu);
}
```

Giảng từng bước với ví dụ kênh 1 (`i = 1`):

```
bit   = 11
byte  = 1 + (11 >> 3) = 1 + 1 = 2
shift = 11 & 7 = 3
```

Vậy kênh 1 nằm ở bit 3 của byte 2, kéo dài 11 bit → cần byte 2, 3, 4. Code gom 3 byte
thành số 24-bit (`f[2] | f[3]<<8 | f[4]<<16`), dịch phải 3, lấy 11 bit cuối (`& 0x7FF`
= mask 11 bit). Đúng.

**Vì sao không dùng struct/union?** Vì layout bit không theo byte; cách shift/mask này
là cách chuẩn và portable (không phụ thuộc endian). Đây là kỹ năng xử lý bit cơ bản
mà mọi embedded engineer phải có.

### 2.3 Chuẩn hóa giá trị

`rc_parse.cpp:33`:

```cpp
float SbusParser::norm(uint16_t v, float lo, float hi) {
  const float span = (v >= kSbusMid) ? static_cast<float>(kSbusMax - kSbusMid)
                                     : static_cast<float>(kSbusMid - kSbusMin);
  float out = (static_cast<float>(v) - static_cast<float>(kSbusMid)) / span;
  if (out < lo) out = lo;
  if (out > hi) out = hi;
  return out;
}
```

Chú ý: dùng **hai span khác nhau** cho nửa dương (1811−992 = 819) và nửa âm
(992−172 = 820). Vì sao không dùng một span? Vì dải SBUS không đối xứng quanh 992 —
nếu dùng chung, một phía sẽ không bao giờ đạt ±1.0 chính xác. Đây là chi tiết nhỏ mà
nhiều codec làm sai.

Throttle chuẩn hóa riêng (`rc_parse.cpp:58`):

```cpp
out.throttle = (ch[2] − 172) / (1811 − 172);   // 0..1
```

### 2.4 Arm switch

```cpp
out.armed_switch = ch[kArmChannel] > kSbusMid;   // kArmChannel = 4 (ch5)
```

Switch gạt lên (> 992) = arm. Đơn giản nhưng phải kết hợp với FSM (mục 4).

### 2.5 Flags an toàn

```cpp
out.rx_failsafe = (f[23] & 0x08u) != 0;
out.frame_lost  = (f[23] & 0x04u) != 0;
out.frame_ok    = !out.rx_failsafe;
```

- **rx_failsafe**: bộ thu mất kết nối với bộ phát → gửi cờ này trong frame.
- **frame_lost**: bộ thu không nhận được frame nào từ phát.
- Cả hai đều dẫn tới failsafe (mục 5).

---

## 3. Parser — xử lý dòng byte liên tục

### 3.1 Vấn đề

HAL cho firmware **một đống byte thô** mỗi tick (`rcRawRead`), không phải frame đã
đóng gói. Parser phải:

1. Gom byte vào buffer.
2. Tìm header `0x0F`.
3. Kiểm tra đủ 25 byte + frame hợp lệ.
4. Trượt buffer, giữ phần dư cho lần sau.
5. Resync nếu frame hỏng (byte rác, nhiễu đường truyền).

Đọc `feed()` (`rc_parse.cpp:70`):

```cpp
bool SbusParser::feed(const uint8_t* data, int len, RcSample& out) {
  if (len <= 0 || len > kBuf) {     // burst quá lớn -> nghi ngờ, reset
    reset();
    return false;
  }
  if (n_ + len > kBuf) reset();
  for (int i = 0; i < len; ++i) buf_[n_++] = data[i];

  bool decoded = false;
  int i = 0;
  while (n_ - i >= kSbusLen) {
    if (buf_[i] != kSbusHeader) { i++; continue; }        // tìm header
    if (decodeFrame(buf_ + i, out)) {
      decoded = true; frames_++; i += kSbusLen;           // frame tốt
    } else {
      i++;                                                // resync từng byte
    }
  }
  if (i > 0) {                                            // giữ phần dư
    std::memmove(buf_, buf_ + i, static_cast<size_t>(n_ - i));
    n_ -= i;
  }
  return decoded;
}
```

Ba điểm thiết kế:

1. **`len > kBuf` → reset**: nếu ai đó gửi 1000 byte một lúc, có thể là lỗi giao tiếp;
   thà bỏ hết còn hơn tràn buffer. Đây là test T3.13 (`n > 64`).
2. **Resync từng byte**: khi frame hỏng, chỉ nhảy 1 byte rồi thử lại — không bỏ cả 25
   byte, vì header thật có thể nằm trong phần "hỏng" đó.
3. **`memmove` giữ phần dư**: byte của frame tiếp theo có thể đã đến. Đây là xử lý
   **stream** đúng cách — buffer luôn chứa phần chưa xử lý.

### 3.2 `decodeFrame` — kiểm tra hợp lệ

`rc_parse.cpp:42`:

```cpp
bool SbusParser::decodeFrame(const uint8_t* f, RcSample& out) {
  if (f[0] != kSbusHeader) return false;
  if (f[23] & 0xF0u) return false;   // nửa cao flags phải = 0

  uint16_t ch[16];
  for (unsigned i = 0; i < 16; ++i) ch[i] = channel(f, i);

  // Stuck-frame guard: tất cả kênh giống nhau là bất khả thi
  bool stuck = true;
  for (unsigned i = 1; i < 16; ++i) {
    if (ch[i] != ch[0]) { stuck = false; break; }
  }
  if (stuck) return false;
  // ... gán vào out
}
```

**Stuck-frame guard** là một ý tưởng thú vị: nếu tất cả 16 kênh có cùng giá trị, đó
gần như chắc chắn không phải lệnh người lái (16 kênh độc lập mà bằng nhau tuyệt đối?)
→ có thể là dữ liệu rác hoặc bộ thu hỏng → từ chối. Đây là phòng thủ sâu, thể hiện
tư duy "không tin dữ liệu ngoài".

---

## 4. Arm/Disarm FSM — khi nào được phép quay motor?

### 4.1 Nguyên tắc

> **Chưa arm → PWM = 0 tuyệt đối.** Không có ngoại lệ.

Nhưng "arm" không chỉ là gạt switch. Có một danh sách điều kiện. Đọc `failsafe.cpp`:

```cpp
// Chuyển từ chưa-arm sang arm (fresh: frame MỚI, xem §5.1)
armed_ = fresh && rc.armed_switch && calibrated && !latched_ && !boot_latched_ &&
         good_frames_ >= kArmMinGoodFrames && rc.throttle <= kArmThrottleMax &&
         sticks_centered && !imu_invalid && !vbat_nan_ && vbat >= kVbatCrit;
```

Từng điều kiện và lý do:

| Điều kiện | Giá trị | Vì sao |
|---|---|---|
| `fresh` | — | Chỉ frame MỚI (t_us tăng + `frame_ok`) mới được arm |
| `rc.armed_switch` | ch5 > mid | Người lái chủ động bật |
| `calibrated` | — | Gyro bias đã hiệu chuẩn (nếu không, drone sẽ trôi) |
| `!latched_` | — | Không có lỗi đang khóa |
| `!boot_latched_` | — | Boot sạch (không phải sau brownout/WDT) |
| `good_frames_ >= 10` | 10 frame **nhận được** | Chống arm bằng frame rác. Ở 14 ms/frame, 10 frame ≈ 140 ms |
| `rc.throttle <= 0.05` | 5% | **Ga phải thấp khi arm** — quy tắc an toàn phổ quát |
| `sticks_centered` | roll/pitch/yaw ≤ 0.05 | Stick lệch không được arm (D5) |
| `!imu_invalid` | — | Cảm biến phải khỏe |
| `!vbat_nan_ && vbat >= kVbatCrit` | ≥ 3.3 V | Mẫu pin hiện tại phải hữu hạn và trên ngưỡng crit (D4) |

### 4.2 Duy trì armed

```cpp
} else if (armed_) {
  // frame giữ (hold) vẫn giữ switch; chỉ switch-off trên frame decode được,
  // latch, hoặc timeout 100 ms mới disarm (D1)
  armed_ = rc.armed_switch && !latched_ && !boot_latched_ && !imu_invalid;
}
```

Chú ý: điều kiện `throttle <= 0.05` **không** còn ở đây — một khi đã arm, bạn được
phép tăng ga. Đây là điều comment trong code nhấn mạnh: *"The low-throttle condition
gates the arm transition only"*. Nếu để điều kiện ga trong nhánh duy trì, drone sẽ
disarm ngay khi cất cánh — tai nạn.

### 4.3 Disarm

```cpp
if (active_) armed_ = false;
```

Bất kỳ failsafe nào active → disarm ngay, bất kể switch. An toàn trên hết.

### 4.4 Chế độ `arm_test`

```cpp
if (arm_test) {
  if (!arm_test_prev_) {   // cạnh lên: arm nếu không có gì đang khóa
    if (!latched_ && !boot_latched_ && !active_) armed_ = true;
  } else {
    armed_ = armed_ && !latched_ && !boot_latched_;
  }
}
```

Đây là "cửa sau" cho **test SIL**: cho phép arm ngay không cần RC hợp lệ (vì test
không muốn chờ 10 frame hay giả lập switch). Có flag riêng, chỉ bật bằng CLI
`--arm-test`. Trong production không bao giờ bật.

`arm_test` **không** xóa `latched_`, `boot_latched_`, `active_`, hay `reason_`. Nếu
một trong các cờ đó đang bật thì `armed_` giữ `false`. Vì vậy `--arm-test` không thể
vượt qua boot latch sau brownout/WDT.

---

## 5. Failsafe — các con đường dẫn đến ngắt motor

### 5.1 Liveness: chỉ frame MỚI mới tính là sống

`failsafe.cpp`:

```cpp
const bool fresh = rc.frame_ok && (!have_frame_ || rc.t_us > last_good_us_);
if (fresh) {
  good_frames_++;
  last_good_us_ = rc.t_us;
  have_frame_ = true;
  lost_count_ = 0;
} else if (rc.frame_lost && ++lost_count_ >= kRcFrameLostN) {
  lost_count_ = kRcFrameLostN;
}
const bool rc_timeout = have_frame_ && (now_us - last_good_us_ >= kRcTimeoutUs);
```

Chi tiết tinh tế: **`rc.t_us > last_good_us_`**. Nếu plant (hoặc kẻ tấn công) replay
frame cũ với cùng timestamp, nó **không** được tính là frame mới → đồng hồ timeout
vẫn chạy → failsafe kích hoạt. Đây là red team finding #10: nếu chỉ kiểm `frame_ok`,
một packet replay có thể "đóng băng" thời gian và che mất failsafe.

Tick im lặng (`n == 0`) **không** xóa `good_frames_`: control loop đưa lại frame giữ
(hold) với `frame_ok` của frame tốt cuối, nên chỉ timeout 100 ms mới cắt. Đây là D2 —
SBUS thật gửi ~14 ms/frame, không phải mỗi tick.

Timeout = 100 ms (`kRcTimeoutUs`). Ở 1 kHz, đó là 100 tick — đủ để không báo động giả
khi mất 1–2 frame, đủ nhanh để cắt motor trước khi drone bay mất kiểm soát.

### 5.2 IMU validity (debounced)

```cpp
const bool imu_bad = !imu.valid || !std::isfinite(gyro[0..2]) || !std::isfinite(accel[0..2]);
if (imu_bad) { if (imu_bad_count_ < kImuInvalidDebounce) imu_bad_count_++; }
else imu_bad_count_ = 0;
const bool imu_invalid = imu_bad_count_ >= kImuInvalidDebounce;   // 5
```

IMU chết = không biết drone đang nghiêng thế nào = không thể điều khiển = phải cắt
motor. Debounce 5 mẫu (5 ms) để tránh lỗi I2C thoáng qua.

### 5.3 Pin

```cpp
vbat_nan_ = !std::isfinite(vbat);
if (vbat_nan_ || vbat >= kVbatCrit) vbat_crit_count_ = 0;
else if (vbat_crit_count_ < kVbatCritSamples) vbat_crit_count_++;
const bool vbat_crit = vbat_crit_count_ >= kVbatCritSamples;   // 20 mẫu
```

Hai mức (D4):

- **NaN**: một mẫu không hữu hạn là latch **ngay** (`kReasonVbatNan = 1 << 5`) và cắt
  motor tick đó. Xen kẽ NaN/3.9 V không thể lách qua debounce.
- **Dưới 3.3 V**: cần **20 mẫu liên tiếp** mới `kReasonVbatCrit`, để một mẫu sụt thoáng
  qua không cắt motor giữa chừng. Mẫu khỏe reset bộ đếm.

NaN một mình không còn "vô hại" như bản cũ: arm cũng đòi mẫu pin hiện tại hữu hạn và
≥ crit.

### 5.4 Latch — khóa lỗi

```cpp
uint32_t r = kReasonNone;
if (rc_timeout)  r |= kReasonRcTimeout;
if (rc.rx_failsafe) r |= kReasonRcFlag;
if (imu_invalid) r |= kReasonImuInvalid;
if (vbat_nan_)   r |= kReasonVbatNan;
if (vbat_crit)   r |= kReasonVbatCrit;
if (r != kReasonNone) {
  latched_ = true;
  active_ = true;
  reason_ |= r;
}
```

Hai tính chất:

1. **Latch (khóa)**: một khi lỗi xảy ra, `latched_ = true` cho đến khi **hạ cánh, tắt
   switch, rồi bật lại** (D3). Lỗi thoáng qua không tự phục hồi. Đây là quy tắc an
   toàn: không tự động bay tiếp sau sự cố.
2. **Bitmask**: `reason_ |= r` giữ **tất cả** lý do (không chỉ lý do đầu tiên) để
   debug. Các bit: `kReasonRcTimeout`, `kReasonRcFlag`, `kReasonImuInvalid`,
   `kReasonVbatNan`, `kReasonVbatCrit`, `kReasonBootLatch`.

Latch trong bay được xóa bởi **cạnh switch off→on** khi hội đủ: frame mới, ga ≤ 0.05,
stick trong deadband 0.05, IMU tốt, pin hữu hạn và ≥ crit:

```cpp
const bool switch_edge_up = rc.armed_switch && !armed_switch_prev_;
if (switch_edge_up && !boot_latched_ && fresh && rc.throttle <= kArmThrottleMax &&
    sticks_centered && !imu_invalid && !vbat_nan_ && vbat >= kVbatCrit) {
  latched_ = false; active_ = false; reason_ = kReasonNone;
}
```

Switch **giữ nguyên ON không xóa latch** — phải cycle. Boot latch thì **không xóa
được** bằng switch; cách duy nhất là rút nguồn.

### 5.5 Boot latch — nhớ tai nạn từ lần trước

`failsafe.cpp:22`:

```cpp
void FailsafeFsm::boot(uint32_t reset_reason) {
  reset();
  const bool after_fault = (reset_reason == kResetBrownout) || (reset_reason == kResetIntWdt) ||
                           (reset_reason == kResetTaskWdt) || (reset_reason == kResetWdt);
  if (after_fault) {
    boot_latched_ = true;
    latched_ = true;
    reason_ |= kReasonBootLatch;
  }
}
```

Vì brownout là reset phần cứng (không code nào chạy lúc sụt), cách duy nhất để phản
ứng là: **lần boot sau, đọc lý do reset**. Nếu là brownout hoặc watchdog → khóa an
toàn, không cho arm cho đến khi người dùng chủ động xử lý. `app_main.cpp:25` truyền
`esp_reset_reason()` vào đây.

---

## 6. LED — giao diện duy nhất với con người

`led.cpp` là FSM đơn giản nhất repo. Trạng thái được chọn trong `control_loop.cpp:78-90`
theo thứ tự ưu tiên:

| Điều kiện | LED | Nhịp |
|---|---|---|
| `boot_latched()` | kError | nháy nhanh 100 ms |
| `active()` (failsafe) | kFailsafe | nháy 250 ms |
| `armed()` | kArmed | sáng liên tục |
| chưa calibrated | kCalib | nháy 500 ms |
| còn lại | kBoot | nháy 200 ms |

Bạn có thể đọc trạng thái drone chỉ qua đèn — kỹ năng debug không thể thiếu khi drone
ở trên không.

---

## 7. Checkpoint

1. Frame SBUS dài bao nhiêu byte? Header? Giá trị giữa của kênh?
2. Giải thích cách lấy kênh 5 từ frame (bit offset, byte offset).
3. Vì sao cần `rc.t_us > last_good_us_` khi kiểm frame mới?
4. Kể tên 4 điều kiện arm chính. Vì sao phải ga thấp mới arm được?
5. Latch là gì? Vì sao lỗi không tự phục hồi?
6. Sau brownout, firmware phản ứng thế nào? Vì sao không thể phản ứng ngay lúc đó?
7. Vì sao `vbat_nan_` không kích failsafe mà chỉ ghi log?

<details>
<summary>Gợi ý đáp án</summary>

1. 25 byte, header 0x0F, giữa 992, dải 172–1811.
2. Kênh 5 (index 4): bit = 44, byte = 1 + 5 = 6, shift = 44 & 7 = 4 → gom byte 6,7,8.
3. Chống replay packet cũ: nếu không, frame cũ "đóng băng" thời gian và failsafe
   không bao giờ kích.
4. Frame hợp lệ + switch bật + calibrated + không latch + boot sạch + ≥10 good frames
   + throttle ≤ 5% + IMU khỏe. Ga thấp để tránh arm khi người lái đang vô tình giữ ga
   cao → motor quay mạnh đột ngột.
5. Khóa trạng thái lỗi, chỉ xóa khi arm lại từ đầu. Vì an toàn: không tự động tiếp
   tục bay sau sự cố.
6. Lần boot sau đọc `esp_reset_reason()`; nếu là brownout/WDT thì `boot_latched_`,
   chặn arm, LED báo lỗi. Không thể phản ứng ngay vì brownout là reset phần cứng tức
   thời.
7. Vì HIL-1 có thể chưa gắn monitor V_BAT (D11); NaN chỉ là đường phòng thủ. Nhưng
   NaN không bao giờ được coi là "pin khỏe" — chỉ là không kích failsafe.

</details>

---

*Tiếp theo: `10-kien-truc-hal-va-realtime.md` — kiến trúc và ràng buộc thời gian thực.*
