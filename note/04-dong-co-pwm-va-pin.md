# 04 — Động cơ, PWM và Pin

> Đọc xong bạn hiểu: duty 0..1 biến thành lực đẩy như thế nào, vì sao motor không đạt
> tốc độ mới ngay lập tức, vì sao pin yếu làm drone rơi, và `plant/battery.py` hoạt
> động ra sao.

---

## 1. PWM — cách vi điều khiển "vặn ga"

### 1.1 PWM là gì?

PWM (Pulse Width Modulation) là bật/tắt nguồn rất nhanh. Nếu bật 50% thời gian và tắt
50% thời gian, motor "cảm nhận" như được cấp 50% điện áp:

```
duty = 0.25:  ██░░░░░░██░░░░░░██░░░░░░
duty = 0.50:  ████░░░░████░░░░████░░░░
duty = 1.00:  ████████████████████████
```

Repo chuẩn hóa duty về **0.0 … 1.0** (`PwmCmd.mot[4]`). Trên chip ESP32-C3, duty
được nạp vào ngoại vi **LEDC** với độ phân giải 11 bit (0…2047), tần số 20–32 kHz
(plan D8). Tần số cao để ngoài dải nghe của người và tránh tiếng rít từ motor.

### 1.2 Phần cứng thật của board này

Theo schematic (`SIL_STACK.md` §2.2), mỗi motor nối qua:

```
V_BAT ──► diode 1N4148W ──► tụ 1µF ──► connector ──► MOTOR ──► MOSFET ──► GND
                                                                  ▲
GPIO0 ──► Rgate 100Ω ─────────────────────────────────────────────┘
                    │
                  10kΩ pulldown xuống GND
```

Hai chi tiết an toàn cực quan trọng:

1. **MOSFET low-side**: công tắc nằm giữa motor và GND. Khi GPIO chưa được điều khiển,
   điện trở pulldown 10 kΩ kéo cổng MOSFET xuống 0 → **motor tắt**. Đây là lý do
   firmware phải để PWM = 0 khi chưa arm: phần cứng đã thiết kế fail-safe.
2. **Motor lấy trực tiếp V_BAT** (pin 1S, 3.3–4.2 V), **không** lấy qua LDO 3.3 V.
   Nếu plant cho motor ăn 3.3 V cố định thì mô phỏng sai vật lý — pin yếu đi mà motor
   vẫn quay như cũ.

> **Cảnh báo từ `docs/measurements.md`:** không được thêm pull-up trên GPIO0–3. Pull-up
> + pulldown 10 kΩ tạo cầu phân áp có thể mở MOSFET một phần → motor quay khi chưa arm.

### 1.3 Đọc code HAL phần cứng

Trên chip, `HwHal::pwmWrite` sẽ gọi `ledc_set_duty` cho 4 kênh. Hiện tại nó là stub
(`hal_hw.cpp` — phase 9 mới làm). Trên PC, `SilHal::pwmWrite` chỉ ghi vào buffer
(`hal_sil.cpp:157`):

```cpp
void SilHal::pwmWrite(const PwmCmd& out) {
  for (int i = 0; i < 4; ++i) out_.mot[i] = out.mot[i];
}
```

Buffer này được đóng gói và gửi cho plant. Vậy là cùng một interface `pwmWrite`, hai
hiện thực hoàn toàn khác nhau. Đó chính là giá trị của HAL.

---

## 2. Mô hình động cơ — vì sao motor không "tức thì"

### 2.1 Phương trình bậc 1

Motor có quán tính: khi bạn tăng duty từ 0 lên 0.5, vận tốc góc không nhảy ngay lên
giá trị mới mà tăng theo hàm mũ:

```
ω̇ = (k_u · u · V_bat/V_nom − ω) / τ_m
```

Trong đó:

| Ký hiệu | Nghĩa | Giá trị repo |
|---|---|---|
| `ω` | vận tốc góc motor | rad/s |
| `u` | duty 0..1 | — |
| `V_bat/V_nom` | hệ số bù điện áp pin | — |
| `τ_m` | hằng số thời gian motor | 0.03 s |
| `k_u` | hệ số từ duty ra vận tốc tối đa | `OMEGA_MAX_RAD_S` |

Ý nghĩa `τ_m`: sau **1 τ_m**, motor đạt 63% tốc độ đích; sau ~3 τ_m (90 ms) coi như
đạt. Brushed motor 1S cỡ này có `τ_m ≈ 20–50 ms`, repo chọn 0.03 s.

Đọc code (`drone_mini_params.py:90`):

```python
def pwm_to_omega(duty, vbat: float):
    d = np.clip(np.asarray(duty, dtype=float), 0.0, 1.0)
    v = 0.0 if not math.isfinite(vbat) else max(vbat, 0.0)
    omega_max = OMEGA_MAX_RAD_S * (v / BATTERY["vbat_nominal"])
    return d * omega_max
```

Giảng giải:

- `np.clip` — duty luôn nằm 0..1, kể cả nếu firmware gửi số lạ.
- `vbat` NaN → coi như 0 V (an toàn: pin "không biết" thì không cho motor quay hết).
- `omega_max` **tỉ lệ với điện áp pin**. Pin 3.85 V nominal → ω_max = 2500. Pin tụt
  còn 3.5 V → ω_max chỉ còn ~2273 rad/s. Đây là hành vi thật của motor điện: điện áp
  thấp thì quay chậm hơn.

Trong `_deriv` (`vehicle.py:65`), phương trình bậc 1 được áp dụng:

```python
om_dot = (P.pwm_to_omega(duty, vbat) - om) / P.TAU_M_S
```

### 2.2 Từ vận tốc góc ra lực và mô men

Hai công thức khí động đơn giản (đủ tốt cho cỡ drone này):

```
Thrust:  T = k_t · ω²        (N)      — lực đẩy
Torque:  Q = k_q · ω²        (N·m)    — mô men cản của cánh quạt
```

Vì sao bình phương? Lực khí động tỉ lệ với bình phương vận tốc dòng khí, mà dòng khí
tỉ lệ với tốc độ quay. `k_t`, `k_q` là hằng số của cánh quạt cụ thể.

Điểm tinh tế: **tăng gấp đôi ω → lực đẩy gấp 4**. Đây là lý do drone rất nhạy ở ga
cao — và là lý do PID phải hoạt động chính xác.

### 2.3 Hover — điểm cân bằng

Bay treo nghĩa là tổng lực đẩy = trọng lượng:

```
4 · k_t · ω_h² = m · g
```

Suy ra (`drone_mini_params.py:84`):

```python
def hover_duty_nominal() -> float:
    omega_h = math.sqrt(MASS_KG * GRAVITY / (4.0 * K_T))
    return omega_h / OMEGA_MAX_RAD_S
```

Chú ý comment trong file: *"Hover duty from ASSUMPTIONS only (never from plant state)"*.
Nghĩa là: test hover **không được** hỏi plant "ga bao nhiêu thì treo?" rồi đem chính
câu trả lời đó làm tiêu chí. Test phải xuất phát từ giả định độc lập (A3, A5). Nếu
không, test chỉ tự khen mình. Đây là bài học thiết kế test quan trọng — red team finding
#4 trong plan.

---

## 3. Pin 1S — nguồn năng lượng và kẻ thù của mọi drone

### 3.1 Mô hình Thevenin

Pin không lý tưởng. Khi dòng tăng, điện áp đầu cực **sụt** (sag):

```
V_bat = V_oc(SoC) − I·R0 − V_R1C1
```

- `V_oc(SoC)`: điện áp mạch hở, phụ thuộc mức sạc. 4.20 V đầy → 3.30 V cạn.
- `R0`: điện trở nội, gây sụt tức thời theo dòng.
- Nhánh `R1C1`: đáp ứng chậm hơn (hiệu ứng điện hóa).

Đọc code (`battery.py:24`):

```python
def step(self, dt: float, duty_sum: float) -> float:
    i = self.current(duty_sum)                          # dòng motor
    self.i_rc += dt * (i - self.i_rc / (self.cfg["r1_ohm"] * self.cfg["c1_f"] + 1e-9))
    ocv = self._ocv()
    v = ocv - i * self.cfg["r0_ohm"] - self.i_rc * 0.0  # R0 dominant (A6)
    tau = 3e-4
    self.v3v3_lag += (v - self.cfg["ldo_dropout_v"]) * (dt / (tau + dt)) - (
        self.v3v3_lag * (dt / (tau + dt)))
    self.v3v3_lag = min(self.v3v3_lag, v)
    self.rail_low = self.v3v3_lag < 3.0
    self.brownout = v < 2.9
    self.charge = max(0.0, self.charge - dt * i / 3600.0 * 0.5)
    return v
```

Giảng từng phần:

- `current(duty_sum)` (`battery.py:19`): dòng motor xấp xỉ từ tổng duty:
  `0.3 A + 1.2 A × duty_sum`. Hover (duty_sum ≈ 2) → ~2.7 A. Đây là con số hợp lý cho
  whoop 1S.
- `i_rc`: trạng thái dòng của nhánh R1C1 (đáp ứng chậm). Hiện tại số hạng này nhân 0
  trong công thức `v` (R0 chiếm ưu thế — giả định A6), nhưng code vẫn giữ để sau này
  bật.
- `v3v3_lag`: điện áp ray 3V3 sau LDO. LDO không tức thời bù được sụt áp — nó có độ
  trễ τ ≈ 0.3 ms. Khi `v3v3_lag < 3.0 V` → `rail_low` (nguy hiểm cho ESP32/IMU).
- `brownout`: `V_bat < 2.9 V` → chip sẽ reset. Nhưng lưu ý quan trọng từ plan D19:
  **trên ESP32-C3, brownout là reset phần cứng — không có code nào chạy lúc sụt áp.**
  Cách duy nhất để xử lý là: lần boot sau đọc `esp_reset_reason()` và khóa an toàn
  (đã làm trong `failsafe.cpp:22`).
- `charge` giảm theo dòng: `dt·i/3600·0.5`. Hệ số 0.5 là ước lượng dung lượng hiệu
  dụng; đủ cho mô phỏng vài phút bay.

### 3.2 Vì sao pin yếu lại nguy hiểm?

Khi bạn tăng ga đột ngột:

```
I tăng → V_bat = V_oc − I·R0 sụt mạnh → LDO đầu vào thấp
      → nếu V_bat < V_dropout + 3.3V (≈ 3.5 V) → ray 3V3 tụt
      → ESP32 brownout reset giữa lúc đang bay
```

Drone rơi tự do. Vì vậy firmware có failsafe điện áp: `kVbatCrit = 3.3 V`
(`failsafe.hpp:29`) — dưới ngưỡng này coi như pin cạn, latch failsafe, cắt motor trước
khi chip tự reset. Đây là ví dụ điển hình của **"thiết kế cho thất bại"**.

### 3.3 LDO và ray 3.3 V

Board dùng RT9193-33GB (3.3 V / 2 A) nuôi ESP32-C3 + ICM-20948. LDO có **dropout**
(khoảng 220 mV theo giả định A10): muốn ra 3.3 V thì đầu vào phải ≥ 3.52 V. Khi pin
sụt xuống dưới ngưỡng này trong lúc 4 motor spike dòng, ray 3V3 dao động → chip lỗi.
Plant mô phỏng hiệu ứng này bằng `v3v3_lag` và cờ `rail_low`.

---

## 4. Từ firmware đến plant — đường đi của một lệnh ga

```
ControlLoop::tick()
   │  mixer.write() → PwmCmd.mot[4] (duty 0..1)
   ▼
OutputStage::apply()            ← GATE: armed ∧ !failsafe ∧ imu_valid
   │
   ▼
SilHal::pwmWrite()              ← chỉ ghi buffer out_.mot[]
   │
   ▼ sendOut() → packet FW_OUT
plant/runner.py:166
   duty = np.array(out["mot"])
   plant.step(P.DT, duty, vbat)
        │  _deriv: pwm_to_omega(duty, vbat) → motor bậc 1 → thrust = k_t·ω²
        │           → lực/mô men → RK4
        ▼
   batt.step(P.DT, duty_sum)   ← cập nhật pin theo dòng tiêu thụ
```

Chú ý thứ tự trong `runner.py`: pin được bước **trước** khi gửi STATE (dòng 130), plant
được bước **sau** khi nhận FW_OUT (dòng 180). Vậy một tick là:

```
sense (đọc trạng thái hiện tại) → gửi → nhận duty → áp duty vào plant
```

Plant trễ đúng 1 tick so với lệnh — giống thực tế (firmware tính toán rồi motor mới
đáp ứng ở tick sau).

---

## 5. Sai lầm phổ biến (đọc kỹ, đây là kinh nghiệm xương máu)

| Sai lầm | Hậu quả | Repo tránh thế nào |
|---|---|---|
| Cho motor ăn 3.3 V cố định | Pin yếu không ảnh hưởng → mô phỏng sai | `pwm_to_omega` scale theo `vbat` |
| Chọn `τ_m` 5–10 ms | Plant đáp ứng nhanh hơn motor thật → gain tune sai | Chốt 0.03 s (A4), test worst-case 50 ms |
| Bỏ qua dòng khởi động | Pin không sụt → không test được failsafe điện áp | `current()` tính từ duty_sum |
| Coi brownout là "code chạy được" | Không xử lý reset → drone tự arm lại sau sụt áp | `failsafe_boot()` đọc reset reason |
| Duty âm hoặc > 1 | Motor "quay ngược" trong sim | Clip ở cả mixer, HAL, và plant |
| Dùng `k_t` từ Crazyflie nguyên bản | Hover duty sai → test vô nghĩa | Hiệu chỉnh từ hover, ghi vào A5 |

---

## 6. Checkpoint

1. Duty 0.5 nghĩa là gì? Trên chip nó thành số nào (11-bit)?
2. Viết phương trình motor bậc 1. Sau 0.03 s motor đạt bao nhiêu phần trăm tốc độ đích?
3. Vì sao lực đẩy tỉ lệ `ω²` chứ không phải `ω`?
4. Pin 1S sụt áp khi tăng ga như thế nào? Ngưỡng failsafe điện áp là bao nhiêu?
5. Vì sao brownout trên ESP32-C3 không thể xử lý bằng code ngay lúc đó?
6. Nếu `k_t` tăng gấp đôi, hover duty tăng hay giảm bao nhiêu lần?

<details>
<summary>Gợi ý đáp án</summary>

1. Bật nguồn 50% thời gian. 11-bit: `0.5 × 2047 ≈ 1024`.
2. `ω̇ = (ω_cmd − ω)/τ_m`. Sau 1 τ_m = 0.03 s, đạt `1 − e⁻¹ ≈ 63%`.
3. Lực khí động tỉ lệ bình phương vận tốc dòng khí; dòng khí tỉ lệ tốc độ quay.
4. `V = V_oc − I·R0`; dòng tăng → V sụt. Failsafe ở `3.3 V` (kVbatCrit).
5. Vì brownout là reset phần cứng tức thời — không có chu kỳ CPU nào chạy khi điện áp
   dưới ngưỡng. Chỉ xử lý được ở lần boot kế tiếp qua `esp_reset_reason()`.
6. `ω_h ∝ 1/sqrt(k_t)` → duty giảm `1/√2 ≈ 0.707` lần.

</details>

---

*Tiếp theo: `05-imu-icm20948.md` — con mắt của drone.*
