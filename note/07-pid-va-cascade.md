# 07 — PID và điều khiển tầng (Cascade Control)

> Đọc xong bạn hiểu PID từ số 0, biết vì sao repo dùng D-on-measurement, anti-windup
> là gì và tại sao cần, và hiểu kiến trúc cascade attitude → rate → mixer. Đây là trái
> tim của flight controller.

---

## 1. Điều khiển phản hồi là gì?

Mọi bộ điều khiển phản hồi đều có dạng:

```
   setpoint ──►(+)── error ──►[ BỘ ĐIỀU KHIỂN ]── lệnh ──►[ HỆ THỐNG ]──► output
                ▲                                                          │
                └──────────────────── đo lại ◄─────────────────────────────┘
```

- **Setpoint**: bạn muốn gì (ví dụ pitch = 10°).
- **Measured**: bạn đang có gì (estimator trả về pitch = 2°).
- **Error**: sai số `e = sp − meas` (8°).
- **Controller**: tính lệnh điều chỉnh từ sai số.

Với drone, "hệ thống" là vật lý (motor + thân drone), "đo lại" là IMU, "lệnh" là duty
cho 4 motor (qua mixer).

---

## 2. PID — ba thành phần, ba vai trò

```
output = Kp·e  +  Ki·∫e dt  +  Kd·de/dt
         └P┘     └─── I ───┘   └─── D ───┘
```

### 2.1 P — Proportional (tỉ lệ)

```
P = Kp · e
```

Sai càng lớn, sửa càng mạnh. Giống bạn đẩy gậy càng mạnh khi nó nghiêng càng nhiều.

- `Kp` nhỏ → sửa chậm, drone lười.
- `Kp` lớn → sửa mạnh, nhưng **dao động** (overshoot rồi lật qua lật lại).
- Vấn đề: chỉ P thì luôn còn **sai số tĩnh** (steady-state error) — vì cần một lực
  nào đó để giữ drone, mà lực chỉ sinh ra khi có sai số.

### 2.2 I — Integral (tích phân)

```
I = Ki · ∫e dt     (rời rạc: I += Ki · e · dt)
```

Cộng dồn sai số theo thời gian. Vai trò: **xóa sai số tĩnh**. Nếu drone cứ lệch 1°
mãi, I tăng dần cho đến khi sai số về 0.

- Nhược điểm: nếu không cẩn thận, I tích lũy khổng lồ trong lúc output đã bão hòa
  → gọi là **windup** → drone "đơ" một lúc sau khi nhả. Cần **anti-windup** (mục 4).

### 2.3 D — Derivative (đạo hàm)

```
D = Kd · de/dt
```

Nhìn **tốc độ thay đổi** của sai số để hãm trước. Giống bạn thấy gậy đang đổ nhanh thì
đẩy tay sớm hơn. Vai trò: **giảm overshoot, tăng damping**.

- Nhược điểm: đạo hàm rất nhạy với nhiễu (nhiễu tần số cao → đạo hàm lớn giả). Cần
  **lọc thông thấp** (mục 3.2).

### 2.4 Đơn vị — tại sao quan trọng

Trong repo, rate PID điều khiển vận tốc góc (rad/s) và output là duty (0..1):

| Gain | Đơn vị | Ý nghĩa số học |
|---|---|---|
| `rate_kp = 0.15` | duty / (rad/s) | sai 1 rad/s → thêm 0.15 duty |
| `rate_ki = 2.4` | duty / rad | — |
| `rate_kd = 0.006` | duty / (rad/s²) | — |

Nếu bạn đổi đơn vị đo (dps thay vì rad/s), gain phải đổi theo (×57.3). Đây là lý do
mọi tên biến trong repo có hậu tố đơn vị.

---

## 3. Đọc `Pid::step()` — từng dòng

`flight/pid.cpp:31`:

```cpp
float Pid::step(float sp, float meas, float dt, bool sat_pos, bool sat_neg) {
  if (!std::isfinite(sp) || !std::isfinite(meas) || !(dt > 0.0f)) return 0.0f;
  const float e = sp - meas;

  // D on measurement (no derivative kick), first-order low-pass.
  float dmeas = 0.0f;
  if (has_prev_) dmeas = (meas - meas_prev_) / dt;
  meas_prev_ = meas;
  has_prev_ = true;
  const float alpha = (d_tau_ > 0.0f) ? (dt / (d_tau_ + dt)) : 1.0f;
  d_filt_ += alpha * (dmeas - d_filt_);
  const float dterm = -kd_ * d_filt_;

  // Conditional integration: hold I while saturated in the error's direction.
  const bool block = (sat_pos && e > 0.0f) || (sat_neg && e < 0.0f);
  if (!block) {
    i_ += ki_ * e * dt;
    if (i_ > i_limit_) i_ = i_limit_;
    if (i_ < -i_limit_) i_ = -i_limit_;
  }

  float out = kp_ * e + i_ + dterm;
  if (out > out_limit_) out = out_limit_;
  if (out < -out_limit_) out = -out_limit_;
  return std::isfinite(out) ? out : 0.0f;
}
```

### 3.1 D on measurement — tránh "derivative kick"

Ngây thơ, D được tính từ sai số: `de/dt = (e − e_prev)/dt`. Vấn đề: khi bạn **giật
stick**, setpoint nhảy bậc → `de/dt` nhảy vô cực → cú giật điều khiển (derivative kick)
→ motor sốc.

Giải pháp: tính D từ **measurement** thay vì error:

```
dmeas/dt = (meas − meas_prev)/dt
D = −Kd · dmeas/dt
```

Vì `e = sp − meas` → `de/dt = dsp/dt − dmeas/dt`; bỏ số hạng `dsp/dt` (thứ gây giật)
và giữ dấu âm. Khi setpoint nhảy, D không nhảy theo — mượt hơn nhiều. Đây là kỹ thuật
chuẩn công nghiệp, repo áp dụng đúng.

### 3.2 Lọc thông thấp cho D

Nhiễu gyro có thành phần tần số cao. Đạo hàm khuếch đại tần số cao (nhân ω) → D sẽ
rất ồn. Repo lọc D bằng bộ lọc bậc 1:

```
d_filt += alpha · (dmeas − d_filt)
alpha = dt / (τ + dt),    τ = 1/(2π·f_c)
```

Với `rate_d_lpf_hz = 100 Hz` và `dt = 1 ms`: `τ = 1/(2π·100) ≈ 1.59 ms`,
`alpha ≈ 0.001/(0.00159+0.001) ≈ 0.386`.

Đây là công thức **bộ lọc thông thấp RC rời rạc** — bạn sẽ gặp nó khắp nơi (estimator
cũng dùng biến thể). Học thuộc: `α = dt/(τ+dt)`.

### 3.3 Tại sao D dùng `meas_prev_` mà không dùng mảng lịch sử?

Vì bộ lọc bậc 1 chỉ cần giá trị trước đó — tiết kiệm RAM và CPU. Trên MCU, mọi thứ
đều phải tiết kiệm.

---

## 4. Anti-windup — chi tiết khó nhất nhưng quan trọng nhất

### 4.1 Windup xảy ra thế nào?

Kịch bản: bạn muốn drone roll 30°/s, nhưng gió mạnh nên nó chỉ đạt 10°/s. Sai số 20°/s
tồn tại mãi → I cộng dồn mãi → sau 5 giây, I = 2.4 × 20 × 5 = 240 (khổng lồ so với
`out_limit = 0.40`). Output luôn bão hòa ở 0.40.

Khi gió tắt, sai số về 0 nhưng I vẫn = 240 → output vẫn bão hòa ngược lại → drone
lật. Phải mất vài giây I mới "xả" hết. Giai đoạn đó drone bay loạn. Đây là **windup**.

### 4.2 Cách repo chống: conditional integration

```cpp
const bool block = (sat_pos && e > 0.0f) || (sat_neg && e < 0.0f);
if (!block) {
  i_ += ki_ * e * dt;
  // clamp
}
```

Logic: **khi output đã bão hòa theo hướng của sai số, đừng tích phân thêm** (vì tích
thêm cũng vô ích — output không thể tăng nữa). Cụ thể:

- `sat_pos` = output đang bão hòa **dương** (chạm trần). Nếu `e > 0` (vẫn muốn tăng
  thêm) → block I.
- `sat_neg` = bão hòa **âm**. Nếu `e < 0` (vẫn muốn giảm thêm) → block I.

Khi hướng đảo chiều (e trái dấu với saturation), I được phép tích phân trở lại —
"xả" nhanh.

### 4.3 `sat_pos`/`sat_neg` đến từ đâu?

Từ **mixer** — nơi duy nhất biết motor nào bão hòa. Đọc `RateController::update`
(`pid.cpp:71`):

```cpp
float RateController::update(const ImuSample& imu, const RateSp& sp, PwmCmd& out) {
  const float dt = kControlDtS;
  // Previous saturation: shift < 0 => high side pulled down (positive sat),
  // shift > 0 => low side pushed up (negative sat).
  const bool sat_pos = last_shift_ < 0.0f;
  const bool sat_neg = last_shift_ > 0.0f;

  const float meas_pitch = -imu.gyro_rps[1];   // FRD nose-up -> nose-down positive
  const float dr = pids_[0].step(sp.roll,  imu.gyro_rps[0], dt, sat_pos, sat_neg);
  const float dp = pids_[1].step(sp.pitch, meas_pitch,     dt, sat_pos, sat_neg);
  const float dy = pids_[2].step(sp.yaw,   imu.gyro_rps[2], dt, sat_pos, sat_neg);

  last_shift_ = mixer_.write(sp.throttle, dr, dp, dy, out);
  return last_shift_;
}
```

Điểm tinh tế: saturation được dùng là của **tick trước** (`last_shift_`), không phải
tick này — vì PID phải chạy *trước* khi biết mixer bão hòa hay không. Đây là vòng
"trễ một tick" chấp nhận được ở 1 kHz.

Quy ước dấu shift (đọc `mixer.hpp`): `shift < 0` = mixer phải kéo kênh cao nhất xuống
= bão hòa **dương**; `shift > 0` = phải đẩy kênh thấp nhất lên = bão hòa **âm**.
Contract này được định nghĩa **một lần** trong mixer và ghi rõ ở header — red team
finding #13 yêu cầu điều này để tránh 3 định nghĩa mâu thuẫn.

### 4.4 Clamp I và output

```cpp
if (i_ > i_limit_) i_ = i_limit_;     // i_limit = 0.06
// ...
if (out > out_limit_) out = out_limit_;   // out_limit = 0.40
```

Hai lớp giới hạn nữa: I không bao giờ vượt 0.06 (15% của out_limit), output không bao
giờ vượt 0.40 duty. Nhờ vậy, kể cả anti-windup thất bại, thiệt hại bị chặn trên.

---

## 5. Cascade control — tại sao hai vòng lồng nhau?

### 5.1 Ý tưởng

Bạn có thể điều khiển trực tiếp góc (attitude) bằng PID: sai góc → duty. Nhưng cách
đó khó ổn định vì động lực học góc phức tạp (quán tính, mô men phi tuyến). Cách tốt
hơn: **hai vòng lồng nhau**:

```
stick ──► góc mong muốn ──►[ P vòng ngoài ]──► tốc độ góc mong muốn
                                                     │
                                                     ▼
                              gyro ──►[ PID vòng trong ]──► duty → mixer
```

- **Vòng ngoài (attitude)**: rất đơn giản — chỉ P. Nhiệm vụ: "tôi muốn nghiêng 10°,
  hiện tại 2° → hãy quay với tốc độ 48°/s". Chậm (không cần nhanh).
- **Vòng trong (rate)**: PID đầy đủ. Nhiệm vụ: "đạt đúng tốc độ quay đó". Nhanh
  (1 kHz), chính xác.

### 5.2 Lợi ích

1. **Vòng trong nhanh dập nhiễu trước khi nó ảnh hưởng vòng ngoài.** Gió giật làm
   drone quay → rate loop sửa ngay, attitude không kịp thấy.
2. **Vòng ngoài đơn giản → dễ tune.** Chỉ cần 1 gain `att_kp` cho mỗi trục.
3. **Mỗi vòng đo được trực tiếp.** Rate loop đo gyro (nhanh, sạch); attitude loop
   dùng góc đã lọc.

Đây là kiến trúc chuẩn của mọi flight controller (Betaflight, PX4, ArduPilot...).

### 5.3 Đọc `AttitudeController::update`

`pid.cpp:95`:

```cpp
void AttitudeController::update(const float sp[3], const float est[3], float dt, float out[3]) {
  (void)dt;
  for (int i = 0; i < 3; ++i) {
    float e = sp[i] - est[i];
    if (!std::isfinite(e)) e = 0.0f;
    if (i == 2) {  // shortest-path yaw error
      while (e > 3.14159265f) e -= kTwoPi;
      while (e < -3.14159265f) e += kTwoPi;
    }
    float r = kp_[i] * e;
    if (r > limit_) r = limit_;
    if (r < -limit_) r = -limit_;
    out[i] = std::isfinite(r) ? r : 0.0f;
  }
}
```

Giảng giải:

- **Chỉ P**, không I không D. Vòng ngoài không cần I vì rate loop bên trong đã xử lý
  sai số tĩnh; thêm I chỉ gây chậm.
- **`i == 2` — shortest-path yaw**: nếu bạn đang ở yaw = 170° và muốn 170°... hay
  đang ở 179° muốn −179°, sai số ngây thơ là −358°, nhưng đường ngắn nhất chỉ là +2°.
  Vòng `while` wrap sai số về `(−π, π]`. Nếu quên, drone sẽ quay một vòng dài.
- **`limit_` = 5.236 rad/s (300 dps)**: giới hạn tốc độ quay mà vòng ngoài được phép
  yêu cầu. An toàn: dù stick giật cực mạnh, drone không quay nhanh hơn 300°/s ở chế
  độ attitude.
- **`att_kp = {6.0, 6.0, 0.0}`**: chú ý yaw = 0! Vì yaw trong chế độ attitude được
  điều khiển như **rate** (stick yaw → rate), không phải góc. Xem `rc_stick_to_att_sp`:
  `out[2] = 0.0f` — yaw setpoint luôn 0 cho vòng attitude; stick yaw đi thẳng vào rate
  setpoint (trong `control_loop.cpp:66`: `RateSp sp{rate_sp[0], rate_sp[1], rate_sp[2], rc.throttle}` —
  thực ra rate_sp[2] từ attitude controller = 0... hãy đọc lại ControlLoop).

Chính xác hơn, trong chế độ attitude (`control_loop.cpp:59-68`):

```cpp
float att_sp[3];
rc_stick_to_att_sp(rc, att_sp);      // roll/pitch -> góc; yaw -> 0
float rate_sp[3];
c.att.update(att_sp, est, dt, rate_sp);   // P -> rate setpoint cho cả 3 trục
RateSp sp{rate_sp[0], rate_sp[1], rate_sp[2], rc.throttle};
shift = c.rate.update(imu_ctl, sp, pwm);
```

Vậy yaw trong chế độ attitude: `att_sp[2] = 0`, `est[2]` là yaw hiện tại → P sẽ cố
kéo yaw về 0! Nhưng `att_kp[2] = 0.0` → `rate_sp[2] = 0` → yaw không bị điều khiển
bởi vòng ngoài. Đây là cách "tắt" kênh yaw trong cascade: gain = 0. Stick yaw vẫn
điều khiển được vì... khoan — trong `rc_stick_to_att_sp`, `out[2] = 0`, và rate_sp[2]
= 0 → stick yaw **không** ảnh hưởng gì trong chế độ attitude?

Hãy kiểm tra lại: `rc_stick_to_att_sp` (`pid.cpp:121`):

```cpp
void rc_stick_to_att_sp(const RcSample& rc, float out[3]) {
  const float r = std::isfinite(rc.roll) ? rc.roll : 0.0f;
  const float p = std::isfinite(rc.pitch) ? rc.pitch : 0.0f;
  out[0] = r * kMaxAttRad;
  out[1] = p * kMaxAttRad;  // nose-down positive
  out[2] = 0.0f;
}
```

Đúng vậy: trong chế độ attitude hiện tại, yaw stick bị bỏ qua (att_sp[2]=0, att_kp[2]=0).
Đây là hạn chế của v0 — yaw chỉ điều khiển được ở chế độ rate. Nếu muốn yaw trong
attitude mode, phải cộng thêm stick yaw vào rate_sp[2] sau vòng attitude. Đây là một
**bài tập mở rộng tốt** (xem file 13).

---

## 6. Stick mapping — từ tay người lái đến setpoint

`pid.cpp:111`:

```cpp
void rc_stick_to_rate_sp(const RcSample& rc, RateSp& out) {
  const float r = std::isfinite(rc.roll) ? rc.roll : 0.0f;
  const float p = std::isfinite(rc.pitch) ? rc.pitch : 0.0f;
  const float y = std::isfinite(rc.yaw) ? rc.yaw : 0.0f;
  out.roll = r * kMaxRateRad;    // kMaxRateRad = 5.236 (300 dps)
  out.pitch = p * kMaxRateRad;   // nose-down positive
  out.yaw = y * kMaxRateRad;
  out.throttle = std::isfinite(rc.throttle) ? rc.throttle : 0.0f;
}
```

Stick trong `RcSample` đã chuẩn hóa về `[−1, 1]` (parser làm việc đó — file 09). Nhân
với `kMaxRateRad = 300 dps` → setpoint vận tốc góc. Ga (throttle) đi thẳng vào mixer.

Tương tự `rc_stick_to_att_sp` nhân với `kMaxAttRad = 30°`.

**Vì sao có giới hạn?** Để một cú giật stick không bao giờ yêu cầu drone quay 1000°/s.
An toàn + bảo vệ motor.

---

## 7. Gain trong repo và kết quả tune

`flight/flight_params.hpp:23`:

```cpp
inline constexpr flight_params_t kFlightParams = {
    /*rate_kp=*/{0.15f, 0.15f, 0.16f},
    /*rate_ki=*/{2.40f, 2.40f, 2.00f},
    /*rate_kd=*/{0.006f, 0.006f, 0.004f},
    /*rate_i_limit=*/0.06f,
    /*rate_out_limit=*/0.40f,
    /*rate_d_lpf_hz=*/100.0f,
    /*att_kp=*/{6.0f, 6.0f, 0.0f},
    /*att_rate_limit=*/5.236f,  // 300 dps
};
```

Đọc bảng này như thế nào:

- Roll và pitch **đối xứng** (drone đối xứng) → cùng gain. Yaw khác (quán tính z lớn
  hơn, phản lực cánh quạt yếu) → gain khác.
- `Ki > Kp` (2.4 vs 0.15) trông lạ nhưng hợp lý vì đơn vị khác nhau (xem mục 2.4).
- `att_kp = 6.0` nghĩa là sai 30° → yêu cầu quay 180°/s (6.0 × 0.5236 rad).

Kết quả tune (`docs/measurements.md`):

> Step response settles < 0.3 s with < 20% overshoot for roll/pitch/yaw at
> `tau_m ∈ {0.02, 0.03, 0.05}` s (T5.7–T5.9).

Chú ý: test chạy với **3 giá trị τ_m** (20/30/50 ms) để đảm bảo gain không bị "may
mắn" với một tham số motor cụ thể. Đây là nguyên tắc robustness — gain phải chịu được
sai số của giả định A4.

---

## 8. Cách tune PID (quy trình thực hành)

Repo đã tune sẵn, nhưng bạn nên biết quy trình để tự làm lại:

1. **Tắt I và D** (ki = 0, kd = 0). Tăng Kp từ từ cho đến khi vừa xuất hiện dao động,
   lùi lại ~50%. Với rate loop, dùng test step.
2. **Thêm D** để giảm overshoot. Tăng từ từ đến khi đáp ứng "săn" (nhiễu khuếch đại).
3. **Thêm I** để xóa sai số tĩnh. Bắt đầu nhỏ (`Kp/10`), tăng đến khi sai số tĩnh đạt
   yêu cầu. Bật anti-windup.
4. **Chạy lại với nhiều `tau_m` và nhiều seed bias** — nếu gain chỉ đẹp ở một cấu hình,
   nó chưa tốt.

Đọc test `tests/test_rate_loop_sil.py` để thấy cách repo tự động hóa bước 4.

---

## 9. Checkpoint

1. Viết công thức PID rời rạc. Vì sao D dùng `meas` thay vì `e`?
2. Windup là gì? Mô tả một kịch bản thực tế sinh ra nó.
3. `sat_pos` và `sat_neg` khác nhau thế nào? Vì sao chúng lấy từ mixer?
4. Vì sao cascade hai vòng ổn định hơn một vòng? Vòng nào chạy nhanh hơn?
5. Yaw trong chế độ attitude mode hiện tại hoạt động ra sao? Hạn chế gì?
6. Nếu đổi gyro từ rad/s sang dps, gain `rate_kp` phải đổi thế nào?

<details>
<summary>Gợi ý đáp án</summary>

1. `out = Kp·e + I + Kd·(meas_prev−meas)/dt` (dạng repo), với `I += Ki·e·dt`. D dùng
   meas để tránh derivative kick khi setpoint nhảy bậc.
2. I tích lũy khi output đã bão hòa; khi hết bão hòa phải "xả" lâu → drone phản ứng
   chậm/loạn. Ví dụ: chống gió mạnh trong 5 s rồi gió tắt.
3. `sat_pos` = bão hòa trần (shift<0); `sat_neg` = bão hòa sàn (shift>0). Lấy từ mixer
   vì chỉ mixer biết kênh nào chạm 0/1 sau khi mix.
4. Vòng trong nhanh dập nhiễu trước khi tới vòng ngoài; vòng ngoài đơn giản dễ tune.
   Vòng trong (rate) chạy 1 kHz, nhanh hơn vòng ngoài (attitude).
5. Yaw stick bị bỏ qua trong attitude mode: `att_sp[2]=0` và `att_kp[2]=0`. Muốn hỗ
   trợ phải cộng stick yaw trực tiếp vào `rate_sp[2]`.
6. Giá trị rad/s nhỏ hơn dps 57.3 lần → để output như cũ, `rate_kp` phải giảm 57.3
   lần (hoặc giữ nguyên và đổi đơn vị đo — nhất quán là điều duy nhất quan trọng).

</details>

---

*Tiếp theo: `08-mixer-va-dau-motor.md` — từ 4 lệnh điều khiển ra 4 duty.*
