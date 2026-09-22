# 06 — Ước lượng tư thế (Attitude Estimation)

> Đọc xong bạn hiểu vì sao không thể tin gyro mãi, vì sao không thể tin accel mãi, và
> cách complementary filter kết hợp hai nguồn "dở" thành một góc "tốt". Bạn đọc được
> `flight/estimator.cpp` từng dòng.

---

## 1. Bài toán

Ta cần biết **roll, pitch, yaw** của drone trong mọi tick. Nhưng:

- **Gyro** chỉ đo vận tốc góc → muốn có góc phải **tích phân**: `angle += ω·dt`.
- **Accel** chỉ cho biết hướng trọng lực → suy ra được roll/pitch khi drone ít tăng tốc.

Cả hai đều có nhược điểm chết người. Estimator là nghệ thuật **hợp nhất ưu điểm của
cả hai**.

### 1.1 Tích phân gyro

```
roll(t) = ∫ ω_x dt
```

| Ưu | Nhược |
|---|---|
| Nhanh, độ trễ ~0, chính xác tức thời | Bias nhỏ × thời gian dài = drift khổng lồ |
| Ít nhiễu trắng | Không tự biết mình sai |

Ví dụ số: bias 5 dps = 0.087 rad/s. Sau 10 s: `0.087 × 10 = 0.87 rad ≈ 50°` — drone
tưởng mình đã nghiêng 50° trong khi thực tế vẫn nằm ngang. **Đây là lý do phải hiệu
chuẩn bias** (mục 3).

### 1.2 Accel làm "cảm biến trọng lực"

Khi drone không tăng tốc, vector accel trong body chỉ là trọng lực phản chiếu:

```
roll_a  = atan2(−a_y, −a_z)
pitch_a = atan2(−a_x, −a_z)
```

| Ưu | Nhược |
|---|---|
| Không drift (trọng lực luôn chỉ xuống) | Nhiễu khi rung cánh quạt |
| Tự hiệu chỉnh dài hạn | Sai khi drone tăng tốc ngang/dọc |

Ví dụ sai: drone tăng tốc về trước 0.2 g → `a_x` thay đổi → `pitch_a` tính sai hàng
chục độ. Vì vậy accel chỉ đáng tin trong **dài hạn** (trung bình nhiều mẫu).

### 1.3 Ý tưởng complementary filter

```
góc = α × (góc + ω·dt)  +  (1−α) × góc_accel
      └──── tin gyro ────┘   └──── tin accel ────┘
```

- Gyro đóng góp phần lớn (`α ≈ 0.998`) → nhanh, mượt.
- Accel đóng góp phần nhỏ (`1−α = 0.002`) mỗi tick → từ từ kéo góc về đúng, chống drift.

Nó hoạt động như một **bộ lọc thông thấp cho accel và thông cao cho gyro**: gyro lo
tần số cao (chuyển động nhanh), accel lo tần số thấp (xu hướng dài hạn).

---

## 2. Toán của complementary filter

### 2.1 Chọn α

Với tần số fusion `fs` (ở đây 500 Hz sau decimation) và tần số cắt `fc` mong muốn:

```
α = exp(−2π·fc/fs)   ≈  1 − 2π·fc/fs
```

Repo dùng `alpha_ = 0.998` (`estimator.hpp:44`). Suy ngược:

```
1 − α = 0.002 = 2π·fc/500  →  fc ≈ 0.16 Hz
```

Nghĩa là: chỉ những thay đổi chậm hơn ~0.16 Hz của accel mới ảnh hưởng đến góc. Rung
cánh quạt (hàng chục–trăm Hz) bị lọc sạch. Đây là con số rất "khoan dung" với accel —
đúng cho môi trường rung mạnh.

### 2.2 Đơn vị thời gian

Vì estimator chạy 500 Hz, `dte = dt × kEstDecim = 0.001 × 2 = 0.002 s`
(`estimator.cpp:67`). Nếu quên nhân, góc sẽ cập nhật sai một nửa tốc độ — bug kinh
điển.

---

## 3. Hiệu chuẩn bias lúc boot — bước sống còn

### 3.1 Ý tưởng

Lúc khởi động, drone nằm yên trên bàn. Nếu gyro đọc `[0.05, −0.03, 0.01]` rad/s thì đó
chính là **bias**. Ta đo nó rồi trừ đi trong mọi phép tính sau.

Đọc `calibrateUpdate` (`estimator.cpp:31`):

```cpp
void ComplementaryEstimator::calibrateUpdate(const ImuSample& imu) {
  if (state_ == State::kDone) return;
  if (state_ == State::kIdle) state_ = State::kCollect;
  if (!imu.valid || !finite3(imu.gyro_rps)) return;
  const float mag = std::sqrt(imu.gyro_rps[0]*imu.gyro_rps[0] +
                              imu.gyro_rps[1]*imu.gyro_rps[1] +
                              imu.gyro_rps[2]*imu.gyro_rps[2]);
  if (mag > kCalibMaxGyroRps) {   // 0.349 rad/s = 20 dps
    n_ = 0;                        // phát hiện chuyển động -> thu thập lại từ đầu
    sum_[0] = sum_[1] = sum_[2] = 0.0f;
    return;
  }
  sum_[0] += imu.gyro_rps[0];
  // ...
  if (++n_ >= kCalibSamples) {    // đủ 200 mẫu
    const float inv = 1.0f / static_cast<float>(n_);
    bias_[0] = sum_[0] * inv;     // trung bình cộng = bias
    // ...
    state_ = State::kDone;
  }
}
```

Giảng giải:

- **Máy trạng thái 3 bước**: `kIdle → kCollect → kDone`. `calibrated()` trả true khi
  `kDone`. Arm bị chặn cho đến khi calibrated (`failsafe.cpp:93`).
- **`kCalibMaxGyroRps = 0.349` rad/s = 20 dps**: nếu phát hiện xoay > 20 dps thì coi
  như drone đang bị di chuyển → **xóa và thu thập lại**. Nhờ vậy, nếu bạn cầm drone
  lúc cắm pin, nó không hiệu chuẩn sai.
- **200 mẫu @1 kHz = 200 ms**. Đủ để trung bình hóa nhiễu trắng.
- **Trung bình cộng** là ước lượng hợp lý vì nhiễu trắng có trung bình 0.

### 3.2 Áp dụng bias ở đâu?

Trong `control_loop.cpp:29-35`:

```cpp
const float dt = kControlDtS;
ImuSample imu_ctl = imu;
if (c.est.calibrated()) {          // trừ bias trước khi control/fusion
  float bias[3];
  c.est.gyroBias(bias);
  for (int i = 0; i < 3; ++i) imu_ctl.gyro_rps[i] -= bias[i];
}
if (c.fs.armed()) {
  c.est.update(imu_ctl, dt);
} else {
  c.est.calibrateUpdate(imu);      // lúc chưa arm: tiếp tục thu thập
}
```

Hai điểm tinh tế:

1. **Bias được trừ ở ControlLoop, không ở estimator.** Estimator chỉ "sở hữu" giá trị
   bias; việc áp dụng nằm ở một chỗ duy nhất để cả PID lẫn fusion dùng cùng dữ liệu đã
   hiệu chỉnh. Comment trong `estimator.cpp:78` ghi rõ: *"Caller passes bias-corrected
   gyro"*.
2. **Khi chưa arm, estimator chỉ hiệu chuẩn, không fusion.** Vì chưa bay thì không cần
   góc.

**Bằng chứng thực nghiệm** từ `docs/measurements.md`:

> "Without it, the ±5 dps ZRO produced ~20° yaw drift over 10 s."

Không trừ bias → trôi 20°/10 s. Có trừ → drift < 0.1°/10 s. Đây là ví dụ hoàn hảo cho
sức mạnh của một bước hiệu chuẩn đúng.

---

## 4. Fusion — đọc `update()` từng dòng

`estimator.cpp:55`:

```cpp
void ComplementaryEstimator::update(const ImuSample& imu, float dt) {
  // 1) Kiểm tra dữ liệu xấu + debounce
  const bool bad = !imu.valid || !finite3(imu.gyro_rps) || !finite3(imu.accel_mps2) || !(dt > 0.0f);
  if (bad) {
    if (invalid_count_ < kImuInvalidDebounce) invalid_count_++;
    imu_valid_ = invalid_count_ < kImuInvalidDebounce;
    return;
  }
  invalid_count_ = 0;
  imu_valid_ = true;

  // 2) Decimation: chỉ fusion mỗi 2 mẫu (500 Hz)
  if (++decim_ < kEstDecim) return;
  decim_ = 0;
  const float dte = dt * static_cast<float>(kEstDecim);   // dt hiệu dụng = 2 ms

  // 3) Góc từ accel
  const float* a = imu.accel_mps2;
  const float an = std::sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
  float roll_a = roll_;
  float pitch_a = pitch_;
  if (an > 1e-3f) {
    roll_a  = std::atan2(-a[1], -a[2]);
    pitch_a = std::atan2(-a[0], -a[2]);
  }

  // 4) Tích phân gyro (đã trừ bias bởi caller)
  const float gx = imu.gyro_rps[0];
  const float gy = -imu.gyro_rps[1];   // nose-down positive
  const float gz = imu.gyro_rps[2];

  // 5) Trộn
  roll_  = alpha_ * (roll_  + gx * dte) + (1.0f - alpha_) * roll_a;
  pitch_ = alpha_ * (pitch_ + gy * dte) + (1.0f - alpha_) * pitch_a;
  yaw_  += gz * dte;

  wrapPi(roll_); wrapPi(pitch_); wrapPi(yaw_);
}
```

Phân tích từng khối:

### 4.1 Debounce (khối 1)

Một mẫu IMU xấu duy nhất không được làm mất trạng thái valid ngay — vì giao tiếp I2C
có thể nhiễu tạm thời. Nhưng **5 mẫu liên tiếp** xấu → `imu_valid_ = false`. Cờ này
đi thẳng tới output gate: mất 5 mẫu → PWM về 0. Vừa đủ nhạy, vừa đủ bền.

### 4.2 Decimation (khối 2)

Fusion chỉ chạy mỗi 2 mẫu. `dte = 2 ms` bù lại khoảng thời gian thực. Nếu quên bù,
góc tích phân chỉ bằng nửa thực tế.

### 4.3 Góc từ accel (khối 3)

`atan2(-a[1], -a[2])` cho roll; `atan2(-a[0], -a[2])` cho pitch. Dấu trừ đôi là do FRD
(z xuống, y phải). Kiểm tra với trường hợp đứng yên `a = [0, 0, −9.81]`:

```
roll_a  = atan2(−0, 9.81) = 0      ✓
pitch_a = atan2(−0, 9.81) = 0      ✓
```

Kiểm tra với trường hợp nghiêng, dùng chính plant để lấy số thật (chạy thử được):

| Tư thế thân drone | `accel_frd` đo được | Góc estimator tính ra |
|---|---|---|
| Nghiêng **phải** 30° (right-wing-down) | `[0, −4.9, −8.5]` | `roll = +30°` |
| Nghiêng **trái** 30° | `[0, +4.9, −8.5]` | `roll = −30°` |
| Chúi mũi **xuống** 30° | `[−4.9, 0, −8.5]` | `pitch = +30°` |

Vậy quy ước repo là: **roll dương = cánh phải chúc xuống**, **pitch dương = mũi chúc
xuống** — đúng chuẩn hàng không FRD. Trực giác: khi cánh phải chúc xuống, vector "lên"
của thế giới (thứ accelerometer đo, vì mặt đất đẩy lên) nghiêng về phía **trái** trong
hệ body → `a_y < 0` → công thức cho roll dương.

> **Bài học:** đừng đoán dấu — hãy kiểm bằng số. Repo có hẳn test khóa dấu
> (`tests/test_plant_dynamics.py` T3.10 và `tests/test_estimator.py` T6.1). Khi bạn
> sửa dấu ở một chỗ, phải chạy test để xem chỗ khác có vỡ không.

### 4.4 Trừ dấu pitch (khối 4)

```cpp
const float gy = -imu.gyro_rps[1];   // nose-down positive
```

Gyro FRD chuẩn đo pitch nose-**up** dương. Repo chọn quy ước nose-**down** dương (để
khớp với mixer và plant). Nên đảo dấu. Nếu quên, drone sẽ điều khiển pitch ngược —
lật ngay lập tức. Đây là lý do tồn tại của test sign contract.

### 4.5 Yaw (khối 5)

```cpp
yaw_ += gz * dte;
```

Yaw **chỉ** được tích phân từ gyro — không có nguồn hiệu chỉnh (mag bị bỏ). Hệ quả:
yaw sẽ drift. Với bias đã hiệu chuẩn, drift < 0.1°/10 s theo `docs/measurements.md` —
chấp nhận được cho bay treo ngắn. Muốn khóa yaw tuyệt đối cần magnetometer hoặc GPS,
mà cả hai đều không phù hợp whoop nhỏ.

### 4.6 `wrapPi`

Góc luôn được wrap về `(−π, π]`. Nếu không, sau nhiều vòng quay `yaw_` có thể lên
100 rad và mất độ chính xác float (số float 32-bit chỉ có ~7 chữ số ý nghĩa).

---

## 5. Cấu trúc dữ liệu và luồng

```
      ImuSample (từ HAL)
            │
   ┌────────┴─────────┐
   │                  │
chưa arm            đã arm
   │                  │
calibrateUpdate()   update()
   │  thu 200 mẫu    │  debounce → decimate → accel angles
   │  → bias_[3]     │  → gyro integrate → blend → wrap
   │  → kDone        │
   └────────┬─────────┘
            │
      getAttitude() → [roll, pitch, yaw]
            │
   AttitudeController (file 07)
```

Các API estimator (`ports.hpp:92`):

| Hàm | Trả về | Dùng bởi |
|---|---|---|
| `calibrateUpdate(imu)` | — | ControlLoop khi chưa arm |
| `update(imu, dt)` | — | ControlLoop khi đã arm |
| `getAttitude(out)` | roll/pitch/yaw | AttitudeController, telemetry |
| `calibrated()` | bool | FailsafeFsm (điều kiện arm) |
| `imuValid()` | bool | OutputStage (gate an toàn) |
| `gyroBias(out)` | bias[3] | ControlLoop (trừ bias) |

---

## 6. Mở rộng: khi nào cần bộ lọc xịn hơn?

Complementary filter đủ cho bay treo góc nhỏ. Các lựa chọn cao cấp hơn:

| Bộ lọc | Khi nào dùng | Chi phí |
|---|---|---|
| **Mahony** | Cần chính xác hơn, vẫn nhẹ; dùng feedback phi tuyến | ~50 dòng |
| **Madgwick** | Tối ưu cho MCU, có gradient descent | ~100 dòng |
| **EKF (Extended Kalman)** | Cần ước lượng cả vị trí/vận tốc, có GPS/baro | Hàng trăm dòng, cần ma trận |
| **UKF** | Hệ phi tuyến mạnh | Nặng hơn EKF |

Repo chọn complementary vì: (1) đủ cho tiêu chí hover < 5°, (2) dễ hiểu, dễ test,
(3) không cần ma trận. Đây là ví dụ của **"chọn công cụ đủ dùng, không chọn công cụ
oai"** — một đức tính quan trọng của kỹ sư.

---

## 7. Checkpoint

1. Vì sao tích phân gyro gây drift? Bias 1 dps gây sai bao nhiêu độ sau 60 s?
2. Khi nào accel cho góc sai? Cho ví dụ cụ thể.
3. Giải thích ý nghĩa `α = 0.998` bằng tần số cắt.
4. Vì sao phải có debounce 5 mẫu cho IMU?
5. Bias được trừ ở đâu trong luồng dữ liệu? Vì sao không trừ bên trong estimator?
6. Yaw có được hiệu chỉnh bởi accel không? Vì sao?

<details>
<summary>Gợi ý đáp án</summary>

1. Bias là hằng số nên tích phân theo thời gian thành `bias × t`. 1 dps = 0.01745 rad/s
   → sau 60 s: 1.047 rad ≈ 60°.
2. Khi drone tăng tốc (ngang hoặc dọc) hoặc khi rung mạnh. Ví dụ tăng tốc tới trước
   0.2 g làm `a_x` thay đổi → pitch_a sai ~11°.
3. `1−α = 0.002` tại 500 Hz → `fc ≈ 0.16 Hz`: chỉ thay đổi chậm hơn 0.16 Hz của accel
   ảnh hưởng đến góc.
4. Giao tiếp I2C có thể lỗi tạm thời; 1 mẫu xấu không nên cắt motor. Nhưng 5 mẫu liên
   tiếp nghĩa là cảm biến thật sự hỏng → phải failsafe.
5. Ở `ControlLoop::tick()` — nơi duy nhất, để cả PID và fusion dùng cùng dữ liệu đã
   hiệu chỉnh. Nếu trừ trong estimator, PID sẽ nhận gyro chưa hiệu chỉnh → rate loop
   bị lệch.
6. Không. Accel không quan sát được hướng quay quanh trục đứng (trọng lực trùng trục).
   Cần mag/GPS mới khóa được yaw.

</details>

---

*Tiếp theo: `07-pid-va-cascade.md` — bộ não điều khiển.*
