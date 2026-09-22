# 08 — Mixer và dấu motor

> Mixer là module nhỏ nhất nhưng nguy hiểm nhất: sai một dấu yaw là drone lật ngay khi
> arm. Đọc xong bạn hiểu toán học quad-X, cách desaturate, và vì sao có hẳn 11 test
> chỉ để khóa dấu.

---

## 1. Mixer là gì và tại sao cần?

Controller tạo ra 4 **lệnh trừu tượng**:

```
throttle (ga tổng)   roll   pitch   yaw
```

Nhưng phần cứng cần 4 **duty motor cụ thể**. Mixer là bộ dịch:

```
(throttle, roll, pitch, yaw)  ──►  (mot0, mot1, mot2, mot3)
```

Không có mixer, drone không bay được vì 4 motor không biết phải nghe lệnh nào.

Với cấu hình **quad-X** (4 cánh xếp hình chữ X nhìn từ trên), mỗi cặp motor đảm nhận
một vai trò khác nhau tùy vị trí hình học. Đó là lý do mixer cần biết vị trí và chiều
quay của từng motor.

---

## 2. Hình học quad-X

Nhìn từ trên, mũi drone hướng lên trên (trục +x body). Quy ước giả định (A1) của repo
(`drone_mini_params.py:47`):

```
              MŨI (+x)
                ▲
       Mot4     │     Mot2
      (trước-   │    (trước-
       trái)    │     phải)
        CW ◄────┼────► CCW
                │
   ─────────────┼─────────────►  (+y = phải trong FRD)
                │
       Mot3     │     Mot1
      (sau-     │    (sau-
       trái)    │     phải)
       CCW ◄────┼────► CW
                │
              ĐUÔI
```

| Motor | GPIO | Vị trí | Chiều quay | `dir` (dấu mô men phản lực) |
|---|---|---|---|---|
| Mot1 | GPIO0 | sau-phải | CW | +1 |
| Mot2 | GPIO1 | trước-phải | CCW | −1 |
| Mot3 | GPIO2 | sau-trái | CCW | −1 |
| Mot4 | GPIO3 | trước-trái | CW | +1 |

### 2.1 Vì sao 2 CW + 2 CCW?

Cánh quạt quay tạo **mô men phản lực** lên thân drone (định luật 3 Newton). Nếu cả 4
quay cùng chiều, thân drone sẽ tự xoay. Hai cặp ngược chiều **triệt tiêu nhau** khi
bay treo (4 motor bằng nhau).

Khi bạn muốn **yaw**, chỉ cần làm lệch: tăng cặp CCW, giảm cặp CW → mô men không còn
triệt tiêu → drone xoay. Đây là cách duy nhất điều khiển yaw trên quadcopter (không có
bánh lái).

### 2.2 Bảng dấu — trái tim của mixer

Từ hình học, suy ra: muốn roll phải (right-wing-down) thì **cánh trái** (Mot3, Mot4)
phải đẩy mạnh hơn. Muốn pitch xuống (nose-down) thì **cánh sau** (Mot1, Mot3) mạnh
hơn. Muốn yaw dương (theo FRD) thì **cặp CCW** (Mot2, Mot3) mạnh hơn.

Bảng kết quả (`mixer.hpp:7` và `MIXER_TABLE` trong params):

| Input | Mot1 (GPIO0) | Mot2 (GPIO1) | Mot3 (GPIO2) | Mot4 (GPIO3) |
|---|---|---|---|---|
| roll+ | − | − | + | + |
| pitch+ | + | − | + | − |
| yaw+ | − | + | + | − |

Cách nhớ: **roll chia trái/phải, pitch chia trước/sau, yaw chia chéo (CCW vs CW)**.
Mot3 (sau-trái) là motor duy nhất tăng ở cả 3 lệnh dương.

### 2.3 Công thức mixer

`mixer.cpp:34`:

```cpp
float m[4];
m[0] = t - r + p - y;  // Mot1, GPIO0
m[1] = t - r - p + y;  // Mot2, GPIO1
m[2] = t + r + p + y;  // Mot3, GPIO2
m[3] = t + r - p - y;  // Mot4, GPIO3
```

Đây chỉ là bảng dấu ở trên viết thành số học. Ga `t` là nền; mỗi lệnh cộng/trừ thêm
vào từng motor.

**Nhận xét quan trọng:** mixer là **tuyến tính**. Với góc nhỏ, mô men roll tỉ lệ với
`(m2+m3) − (m0+m1) = 4r`. Tuyến tính đủ tốt cho v0 và giúp test dễ. Các flight
controller nâng cao dùng mixer phi tuyến (airmode, thrust linearization) nhưng đó là
chủ đề nâng cao.

---

## 3. Desaturation — khi motor chạm biên

### 3.1 Vấn đề

Duty motor chỉ có thể từ 0 đến 1. Giả sử `t = 0.8, r = 0.3`:

```
m0 = 0.5    m1 = 0.5    m2 = 1.1    m3 = 1.1
```

`m2`, `m3` vượt 1.0. Nếu chỉ clip riêng từng kênh về 1.0, ta được `[0.5, 0.5, 1.0, 1.0]`
→ chênh lệch roll thực tế chỉ còn 0.5 thay vì 0.6 mong muốn → drone phản ứng yếu hơn
dự kiến → **méo điều khiển**.

### 3.2 Giải pháp: dịch tất cả cùng lúc

Thay vì clip từng kênh, **trừ một lượng bằng nhau** khỏi cả 4 kênh:

```
shift = −(1.1 − 1.0) = −0.1
m = [0.4, 0.4, 1.0, 1.0]
```

Chênh lệch roll trước và sau **giữ nguyên**:

```
trước:  (m2+m3) − (m0+m1) = (1.1+1.1) − (0.5+0.5) = 1.2
sau:    (m2+m3) − (m0+m1) = (1.0+1.0) − (0.4+0.4) = 1.2
```

Điều duy nhất thay đổi là ga tổng giảm (drone hơi tụt) — chấp nhận được, vì đúng vật
lý: motor đã hết ga thì không thể vừa giữ độ cao vừa roll mạnh.

### 3.3 Đọc `desaturate()`

`mixer.cpp:14`:

```cpp
float QuadXMixer::desaturate(float m[4]) {
  float shift = 0.0f;
  float hi = m[0];
  float lo = m[0];
  for (int i = 1; i < 4; ++i) {
    if (m[i] > hi) hi = m[i];
    if (m[i] < lo) lo = m[i];
  }
  if (hi > 1.0f) shift -= (hi - 1.0f);          // kéo trần xuống
  if (lo + shift < 0.0f) shift -= (lo + shift); // đẩy sàn lên (sau khi đã kéo trần)
  for (int i = 0; i < 4; ++i) m[i] += shift;
  return shift;
}
```

Phân tích thứ tự hai câu `if`:

1. Nếu trần vượt 1 → kéo tất cả xuống (`shift` âm).
2. **Sau khi kéo trần**, kiểm tra sàn: nếu kéo quá tay làm sàn âm → đẩy tất cả lên
   (`shift` dương thêm).
3. Kết quả: dải `[lo, hi]` được "trượt" vào trong `[0, 1]` sao cho không kênh nào ra
   ngoài, và **chênh lệch giữa các kênh không đổi**.

Lưu ý `if (lo + shift < 0)` dùng `shift` đã cập nhật — đó là lý do thứ tự quan trọng.

### 3.4 `shift` trả về để làm gì?

Mixer trả về `shift` để **rate controller biết đã bão hòa** → anti-windup (file 07,
mục 4.3). Quy ước (ghi trong `mixer.hpp`):

| `shift` | Nghĩa |
|---|---|
| `shift < 0` | đã kéo trần xuống → bão hòa **dương** (`sat_pos`) |
| `shift > 0` | đã đẩy sàn lên → bão hòa **âm** (`sat_neg`) |
| `shift = 0` | không bão hòa |

Đây là **một định nghĩa duy nhất** cho toàn repo — bài học từ red team finding #13
(trước đó có 3 định nghĩa mâu thuẫn nhau).

### 3.5 `clampf` — chốt chặn cuối

```cpp
for (int i = 0; i < 4; ++i) out.mot[i] = clampf(m[i], 0.0f, 1.0f);
```

Sau desaturate, mixer vẫn clip lần cuối (và chặn NaN). Phòng thủ nhiều lớp: PID →
mixer → OutputStage → HAL → plant. Một giá trị xấu phải vượt 4 lớp mới ra được motor.

---

## 4. Ánh xạ GPIO và giả định A1

`PwmCmd.mot[0..3]` tương ứng **Mot1..Mot4 = GPIO0..GPIO3** (`ports.hpp:34`). Đây là
hợp đồng đã khóa. Nhưng **vị trí thật và chiều quay thật** trên PCB là **giả định A1**:

> Từ `docs/measurements.md`: A1 = "motor positions/spin", gate = **"mandatory spin-bench
> before HIL/flight"**.

Nghĩa là: toàn bộ bảng dấu trên được viết theo giả định Betaflight-style X. Trước khi
gắn cánh, bạn **phải** làm spin bench:

1. Cấp duty thấp (ví dụ 0.1), **không gắn cánh quạt**.
2. Tăng từng GPIO0–3, ghi lại motor nào quay, chiều nào.
3. So với bảng giả định. Nếu khác → sửa `ROTOR_MAP`/mixer và chạy lại test.

Đây là ví dụ đẹp của nguyên tắc: **phần mềm đúng không có nghĩa là hệ thống đúng** —
phần mềm chỉ đúng với giả định của nó.

---

## 5. Unit test — đọc như đặc tả

`tests/test_mixer.py` có 11 test T2.1–T2.11. Đọc chúng để hiểu mixer được kỳ vọng làm
gì. Ví dụ ý tưởng các test:

| Test | Kiểm gì |
|---|---|
| T2.1 | Hover (r=p=y=0) → 4 kênh bằng nhau |
| T2.2–T2.4 | Roll+/pitch+/yaw+ → đúng cặp motor tăng/giảm |
| T2.5 | Input clip: throttle 1.5 → coi như 1.0 |
| T2.6 | Clip 0..1 cho mọi kênh |
| T2.7 | NaN input → 0, không lan truyền |
| T2.8 | Desaturation: chênh lệch duty được giữ |
| T2.9–T2.11 | Dấu yaw đúng; hover 4 kênh xấp xỉ `hover_duty_nominal` |

Đọc `test_mixer.py:102`:

```python
@pytest.mark.parametrize("axis", ["roll", "pitch", "yaw"])
def test_...(lib, axis):
    # tăng axis dương, kiểm hiệu duty hai cặp motor đúng dấu
```

`parametrize` chạy cùng một logic cho 3 trục — cách viết test gọn và chặt.

Có một test đặc biệt quan trọng về **desaturation** (T2.8): nó không kiểm "có clip
không" (quá dễ) mà kiểm **"chênh lệch duty giữa hai cặp motor không đổi sau khi
desaturate"**. Đây là red team finding #13: test cũ là tautology (tự chứng minh chính
mình), đã được sửa thành kiểm tính chất thật.

---

## 6. Thí nghiệm: tự phá mixer

Đây là bài tập hiệu quả nhất để hiểu mixer. Làm thử:

1. Mở `flight/mixer.cpp`, đổi dấu yaw ở Mot2: `m[1] = t - r - p - y;`
2. Chạy `make test`. Test nào fail? Ghi lại.
3. Khôi phục. Đổi dấu roll ở Mot3: `m[2] = t - r + p + y;` → test nào fail?
4. Suy luận: nếu **không có** test, hậu quả thực tế của mỗi lỗi là gì? (Gợi ý: yaw
   sai dấu → drone quay ngược; roll sai dấu → drone lật ngay khi cất cánh.)

---

## 7. Checkpoint

1. Vì sao quadcopter cần 2 cánh quay CW và 2 CCW?
2. Viết công thức mixer cho Mot3. Giải thích từng số hạng.
3. Roll dương làm motor nào tăng? Vì sao đúng vật lý?
4. Desaturation khác gì so với clip từng kênh? Vì sao clip từng kênh gây méo điều
   khiển?
5. `shift` trả về được dùng thế nào bởi rate controller?
6. Giả định A1 là gì? Nếu sai, hậu quả ra sao và test nào sẽ phát hiện?

<details>
<summary>Gợi ý đáp án</summary>

1. Để mô men phản lực của cánh quạt triệt tiêu khi bay treo; và để có thể điều khiển
   yaw bằng cách làm lệch hai cặp.
2. `m[2] = t + r + p + y`. Ga nền + roll (Mot3 ở trái) + pitch (Mot3 ở sau) + yaw
   (Mot3 thuộc cặp CCW).
3. Mot3, Mot4 (cánh trái). Vì đẩy cánh trái mạnh hơn làm cánh phải chúc xuống = roll
   dương.
4. Clip từng kênh làm chênh lệch duty giữa các motor thay đổi → mô men roll/pitch/yaw
   bị méo. Desaturate dịch tất cả cùng lúc nên giữ nguyên chênh lệch.
5. Làm tín hiệu saturation cho anti-windup: `shift<0` → `sat_pos`, `shift>0` → `sat_neg`.
6. A1 = vị trí/chiều quay motor trên khung. Nếu sai, mixer xuất duty đúng theo giả
   định nhưng motor thật quay sai chỗ → drone lật/không bay được. Test phần mềm không
   phát hiện được; chỉ spin bench (đo phần cứng) mới phát hiện.

</details>

---

*Tiếp theo: `09-rc-sbus-arm-failsafe.md` — điều khiển từ xa và an toàn.*
