# 03 — Hệ quy chiếu & động lực học 6-DOF

> Đây là file "vật lý" của bộ note. Đọc xong bạn hiểu `plant/vehicle.py` từng dòng,
> biết quaternion là gì, và tự tính được lực/mô men của quadcopter bằng tay.

---

## 1. Vì sao phải có hệ quy chiếu?

Một vector như "đi lên 1 mét" chỉ có nghĩa khi biết **lên theo cái gì**. Trong UAV:

- **Lên theo mặt đất** (world) khác với **lên theo thân drone** (body).
- Khi drone nghiêng 45°, "lên theo body" và "lên theo world" lệch nhau 45°.
- Cảm biến đo trong body. Vật lý (trọng lực) tác động trong world. Ta phải đổi qua lại.

Repo dùng **ba hệ**:

| Hệ | Trục x | Trục y | Trục z | Dùng ở đâu |
|---|---|---|---|---|
| **ENU** (world) | East (Đông) | North (Bắc) | **Up** (lên) | `plant/vehicle.py` — vị trí, vận tốc |
| **FLU** (body, sim) | Forward (mũi) | Left (trái) | Up (lên) | `plant/vehicle.py` — thân drone trong mô phỏng |
| **FRD** (body, firmware) | Forward | Right (phải) | **Down** (xuống) | `flight/` — gyro, accel, mọi thứ firmware nhìn thấy |

### 1.1 Vì sao phải đến 3 hệ?

- **ENU** là quy ước phổ biến của robotics mô phỏng (z lên cho dễ hình dung).
- **FLU** cùng tay với ENU (cùng z-up) nên công thức quay trong sim đơn giản.
- **FRD** là quy ước hàng không (aerospace): trục z hướng xuống, roll phải dương khi
  cánh phải chúc xuống. Mọi tài liệu IMU/datasheet dùng FRD.

Firmware dùng FRD vì **datasheet ICM-20948, tài liệu điều khiển bay, và kinh nghiệm
ngành đều FRD**. Plant dùng FLU vì tính toán mô phỏng thuận tiện. Hai bên gặp nhau ở
**biên `sense()`** bằng một hàm 3 dòng (`vehicle.py:40`):

```python
def to_frd(v):
    return np.array([v[0], -v[1], -v[2]])
```

Chỉ đảo dấu y và z. Vì sao? Vì Right = −Left và Down = −Up; trục x (Forward) giống
nhau.

> **Bài học nghề nghiệp:** chuyển hệ quy chiếu là nguồn bug kinh điển. Cách chống:
> (1) ghi rõ hệ trong comment mọi biến, (2) chuyển đổi chỉ ở **một chỗ duy nhất** —
> ở đây là `sense()`, (3) có test khóa dấu (`tests/test_plant_dynamics.py` T3.10).

---

## 2. Quaternion — biểu diễn hướng không bị kẹt

### 2.1 Vấn đề với Euler angles

Ba góc roll/pitch/yaw dễ hiểu nhưng có điểm kỳ dị (gimbal lock): khi pitch = ±90°,
roll và yaw trở nên trùng nhau, công thức nổ. Drone bay có thể pitch gần 90° khi bay
nhanh → không thể dùng Euler làm biến trạng thái.

### 2.2 Quaternion là gì?

Quaternion là bộ 4 số `q = (w, x, y, z)` với `w² + x² + y² + z² = 1`, biểu diễn một
phép quay trong không gian 3D. Trực giác:

```
q = (cos(θ/2),  sin(θ/2)·ux,  sin(θ/2)·uy,  sin(θ/2)·uz)
```

với `(ux,uy,uz)` là trục quay và `θ` là góc quay. Quaternion tránh được điểm kỳ dị
và nhân hai phép quay chỉ tốn 16 phép nhân.

### 2.3 Đọc code `vehicle.py`

**Nhân quaternion** (`vehicle.py:16`) — "quay thêm b" lên q:

```python
def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])
```

Bạn không cần nhớ công thức — chỉ cần biết nó tương đương nhân ma trận quay, nhưng
rẻ hơn. Trong repo, `quat_mul` được dùng để tính đạo hàm quaternion (mục 3.3).

**Quaternion → ma trận quay** (`vehicle.py:27`):

```python
def quat_to_R(q):
    w, x, y, z = q
    n = w*w + x*x + y*y + z*z
    if n < 1e-12:
        return np.eye(3)          # quaternion hỏng -> trả về ma trận đơn vị
    s = 2.0 / n
    return np.array([...])        # ma trận 3x3
```

Ma trận `R` này có tính chất: `v_world = R @ v_body` (quay vector từ body sang world),
và `v_body = R.T @ v_world` (ngược lại, vì R trực giao nên nghịch đảo = chuyển vị).

Trong `_deriv` (`vehicle.py:67`) bạn thấy cả hai chiều được dùng:

```python
R = quat_to_R(q)
fb = np.array([0.0, 0.0, thrust.sum()])    # lực trong BODY (chỉ có thành phần z)
acc = R @ fb / P.MASS_KG + np.array([0.0, 0.0, -P.GRAVITY])   # body -> world
```

Lực đẩy chỉ có theo trục z của body; nhân `R` để biết nó hướng nào trong world.

**Chuẩn hóa** (`vehicle.py:87`) — sau mỗi bước tích phân, quaternion bị trôi khỏi
mặt cầu đơn vị do sai số số học, phải chuẩn hóa lại:

```python
self.x[6:10] = self.x[6:10] / np.linalg.norm(self.x[6:10])
```

---

## 3. Động lực học 6-DOF

### 3.1 Vector trạng thái — 17 số

`vehicle.py:7`:

```
x = [pos(3), vel(3), quat_wxyz(4), omega(3), Omega(4)]  → 17 phần tử
```

| Nhóm | Index | Nghĩa | Đơn vị |
|---|---|---|---|
| `pos` | 0–2 | vị trí trong world ENU | m |
| `vel` | 3–5 | vận tốc trong world | m/s |
| `quat` | 6–9 | hướng thân drone `(w,x,y,z)` | — |
| `omega` | 10–12 | vận tốc góc body FLU | rad/s |
| `Omega` | 13–16 | vận tốc 4 motor | rad/s |

Đây là "trạng thái đầy đủ" của drone: biết 17 số này + lệnh duty + điện áp pin là
biết tương lai chuyển động (với mô hình này).

### 3.2 Phương trình chuyển động

Vật lý Newton–Euler:

```
Tịnh tiến:   m·v̇ = m·g + R·(0,0,ΣTᵢ) + F_aero
Quay:        I·ω̇ + ω×(I·ω) = Σᵢ (rᵢ × Tᵢ·ẑ + Qᵢ·ẑ)
```

Giảng từng phần:

- `m·g = (0,0,−9.81·m)` — trọng lực kéo xuống trong world ENU.
- `R·(0,0,ΣTᵢ)` — tổng lực đẩy 4 cánh, quay từ body sang world.
- `rᵢ × Tᵢ·ẑ` — mô men do lực đẩy lệch tâm. Cánh trước đẩy mạnh hơn cánh sau →
  mô men pitch.
- `Qᵢ = sᵢ·k_q·ωᵢ²` — mô men phản lực của cánh quạt (định luật 3 Newton): cánh quay
  một chiều thì thân drone bị xoắn chiều ngược lại. Đây là cơ chế **yaw**.
- `ω×(I·ω)` — số hạng Coriolis/Euler của vật rắn quay; quan trọng khi drone quay nhanh.

Đọc code `_deriv` (`vehicle.py:60-78`):

```python
om_dot = (P.pwm_to_omega(duty, vbat) - om) / P.TAU_M_S   # motor bậc 1 (file 04)
thrust = P.K_T * om * om                                  # T = k_t·ω²
R = quat_to_R(q)
fb = np.array([0.0, 0.0, thrust.sum()])
acc = R @ fb / P.MASS_KG + np.array([0.0, 0.0, -P.GRAVITY])   # tịnh tiến

tau_x = float((self.rotor_pos[:, 1] * thrust).sum())      # roll từ chênh lệch trái/phải
tau_y = float(-(self.rotor_pos[:, 0] * thrust).sum())     # pitch từ chênh lệch trước/sau
tau_z = float((self.rotor_dir * P.K_Q * om * om).sum())   # yaw từ phản lực quay
tau = np.array([tau_x, tau_y, tau_z])
w_dot = self.Iinv @ (tau - np.cross(w, self.I @ w))       # Euler equation

acc = acc - np.asarray(P.LINEAR_DRAG) * v / P.MASS_KG     # (đang = 0)
w_dot = w_dot - np.asarray(P.ANGULAR_DRAG) * w / np.array(P.INERTIA)

q_dot = 0.5 * quat_mul(q, np.array([0.0, w[0], w[1], w[2]]))  # đạo hàm quaternion
return np.concatenate([v, acc, q_dot, w_dot, om_dot])
```

Chú ý cách tính `tau_x`, `tau_y`: dùng tích có hướng nhưng viết gọn. Cánh ở `y > 0`
(trái) tạo mô men roll dương khi đẩy mạnh; cánh ở `x > 0` (trước) tạo mô men pitch
âm (mũi chúi xuống) khi đẩy mạnh.

### 3.3 Đạo hàm quaternion

```python
q_dot = 0.5 * quat_mul(q, np.array([0.0, w[0], w[1], w[2]]))
```

Công thức chuẩn: `q̇ = ½·q ⊗ (0, ω)`. Bạn chỉ cần nhớ: vận tốc góc `ω` cho biết
quaternion thay đổi bao nhanh.

---

## 4. RK4 — tích phân số

### 4.1 Vì sao cần?

Phương trình chuyển động là phương trình vi phân: `ẋ = f(x)`. Máy tính không giải
được liên tục, phải "bước" từng `dt`. Cách ngây thơ (Euler): `x += dt·f(x)`. Nhược
điểm: tích lũy sai số nhanh, nhất là với con lắc/drone (hệ dao động).

RK4 (Runge–Kutta bậc 4) lấy **4 mẫu độ dốc** trong mỗi bước rồi pha trộn:

```
k1 = f(x)                    # độ dốc đầu bước
k2 = f(x + dt/2 · k1)        # độ dốc giữa bước (dự đoán 1)
k3 = f(x + dt/2 · k2)        # độ dốc giữa bước (dự đoán 2)
k4 = f(x + dt · k3)          # độ dốc cuối bước
x' = x + dt/6 · (k1 + 2k2 + 2k3 + k4)
```

Trọng số `1,2,2,1` là điều kỳ diệu của RK4 — sai số giảm từ `O(dt²)` xuống `O(dt⁴)`.
Với `dt = 1 ms`, RK4 chính xác hơn Euler hàng triệu lần.

Đọc code (`vehicle.py:80`):

```python
def step(self, dt, duty, vbat):
    duty = np.clip(np.asarray(duty, dtype=float), 0.0, 1.0)
    k1 = self._deriv(self.x, duty, vbat)
    k2 = self._deriv(self.x + 0.5 * dt * k1, duty, vbat)
    k3 = self._deriv(self.x + 0.5 * dt * k2, duty, vbat)
    k4 = self._deriv(self.x + dt * k3, duty, vbat)
    self.x = self.x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
    # ... hậu xử lý
```

Bốn lần gọi `_deriv` mỗi bước — đó là "giá" của độ chính xác. Với plant 6-DOF trên
PC, giá này không đáng kể.

### 4.2 Hậu xử lý sau mỗi bước (`vehicle.py:87-95`)

```python
self.x[6:10] = self.x[6:10] / np.linalg.norm(self.x[6:10])   # chuẩn hóa quat
self.x[13:17] = np.maximum(self.x[13:17], 0.0)               # motor không quay âm
if self.x[2] < 0.0:                                          # chạm đất đơn giản
    self.x[2] = 0.0
    if self.x[5] < 0.0:
        self.x[5] = 0.0
if not np.all(np.isfinite(self.x)):                          # phát hiện phân kỳ
    raise FloatingPointError("plant state diverged")
```

Từng dòng là một "sự thật vật lý":

- Quaternion phải nằm trên mặt cầu đơn vị.
- Motor không quay ngược (duty 0 → ω = 0, không âm).
- Drone không xuyên đất; vận tốc xuống bị chặn.
- Nếu trạng thái thành NaN/Inf → ném lỗi để runner thoát với exit code 4 thay vì ghi
  log rác. Đây là "fail loud".

---

## 5. Đọc trạng thái thật từ plant: `sense()`

`vehicle.py:100`:

```python
def sense(self, dt=0.001):
    om = self.x[13:17]
    thrust = P.K_T * om * om
    fb = np.array([0.0, 0.0, thrust.sum()]) / P.MASS_KG
    R = quat_to_R(self.x[6:10])
    a_world = R @ fb + np.array([0.0, 0.0, -P.GRAVITY])
    on_ground = (self.x[2] <= 1e-9) and (self.x[5] <= 1e-9)
    if on_ground:
        a_world = np.zeros(3)
    f_body = R.T @ (a_world + np.array([0.0, 0.0, P.GRAVITY]))
    accel_frd = to_frd(f_body)
    gyro_frd = to_frd(self.x[10:13])
    return gyro_frd, accel_frd
```

Đây là nơi mô phỏng "cảm biến":

- **Gyro** = vận tốc góc body, đổi sang FRD.
- **Accel** = specific force. Công thức `f_body = R.T @ (a_world − g_world)`; dấu cộng
  `+g` xuất hiện vì `a_world` đã chứa `−g` ở bước trên. Kết quả ở trạng thái đứng yên
  trên mặt đất là `[0, 0, −9.81]` trong FRD — đúng như gia tốc kế thật đọc (nó đo lực
  đẩy của mặt đất, không đo "gia tốc chuyển động").
- **`on_ground`**: khi đứng trên đất, gia tốc chuyển động bằng 0 nhưng gia tốc kế vẫn
  đọc `−g`. Nhánh này xử lý đúng trường hợp đó.

Nếu đoạn này khó, hãy quay lại sau khi đọc file 05 (IMU). Điều tối thiểu cần nhớ:
**`sense()` là biên duy nhất giữa FLU và FRD, và là nơi duy nhất sinh dữ liệu cảm biến.**

---

## 6. `euler()` — đổi quaternion sang góc để log

`vehicle.py:143`:

```python
def euler(self):
    q = self.x[6:10]
    R = quat_to_R(q)
    Rf = R @ np.diag([1.0, 1.0, -1.0])      # FLU -> FRD
    pitch = np.arcsin(np.clip(-Rf[2, 0], -1.0, 1.0))
    roll = np.arctan2(Rf[2, 1], Rf[2, 2])
    yaw = np.arctan2(Rf[1, 0], Rf[0, 0])
    return float(roll), float(pitch), float(yaw)
```

Góc Euler **chỉ để hiển thị/log**, không dùng trong tính toán (vì điểm kỳ dị). Chú ý
`np.clip` trước `arcsin`: nếu ma trận hơi lệch do sai số, giá trị có thể là 1.0000001
và `arcsin` sẽ trả NaN. Đây lại là phòng thủ số học.

---

## 7. Tự tính tay — bài tập thị giác hóa

### 7.1 Hover cần vận tốc motor bao nhiêu?

Với `m = 0.03 kg`, `k_t = 4.7e-8`, `g = 9.81`:

```
4·k_t·ω_h² = m·g
ω_h = sqrt(0.03·9.81 / (4·4.7e-8)) = sqrt(0.2943 / 1.88e-7) ≈ sqrt(1.565e6) ≈ 1251 rad/s
```

Và hover duty (`drone_mini_params.py:84`):

```
duty_h = ω_h / ω_max = 1251 / 2500 ≈ 0.50
```

**Một nửa ga để bay treo** — con số này xuất hiện trong kịch bản test (`thr = 0.52`).
Đây là dấu hiệu plant được hiệu chỉnh hợp lý: drone thật thường hover quanh 50% ga.

### 7.2 Khi roll trái mạnh thì motor nào nhanh hơn?

Nhìn bảng `ROTOR_MAP` (`drone_mini_params.py:51`) — tọa độ ghi trong body **FLU**
(x forward, y **left**):

| Motor | `pos` (FLU) | Vị trí thực tế |
|---|---|---|
| Mot1 | `(−d, −d, 0)` | sau–**phải** |
| Mot2 | `(+d, −d, 0)` | trước–**phải** |
| Mot3 | `(−d, +d, 0)` | sau–trái |
| Mot4 | `(+d, +d, 0)` | trước–trái |

Nếu roll dương (nghiêng phải, right-wing-down) thì cần **cánh trái đẩy mạnh hơn** để
đẩy cánh phải lên → Mot3/Mot4 (trái) tăng. Bảng mixer (`MIXER_TABLE`, dòng 61) xác
nhận: `roll = [-1, -1, +1, +1]`.

Hãy tự kiểm chứng với `tau_x = Σ yᵢ·Tᵢ` (công thức trong `_deriv`): Mot3/Mot4 có
`y = +0.043` (trái, FLU) nên tăng thrust ở đó làm `tau_x` dương. Trong FLU, mô men
dương quanh x (quy tắc bàn tay phải: y hướng lên z) = **cánh trái lên, cánh phải
xuống** = roll right-wing-down dương. Nhất quán với estimator và mixer.

---

## 8. Checkpoint

1. Vì sao firmware dùng FRD còn plant dùng FLU? Chúng gặp nhau ở đâu?
2. `to_frd` đảo dấu trục nào? Vì sao trục x không đảo?
3. Viết công thức tính mô men yaw của quadcopter. Tại sao phải có 2 cánh quay CW và
   2 cánh quay CCW?
4. RK4 khác Euler ở điểm nào? Vì sao sai số nhỏ hơn nhiều?
5. Sau mỗi bước RK4, vì sao phải chuẩn hóa quaternion?
6. Tính hover duty nếu khối lượng tăng lên 0.04 kg (giữ nguyên k_t, ω_max).

<details>
<summary>Gợi ý đáp án</summary>

1. FRD là chuẩn hàng không/IMU; FLU tiện cho mô phỏng z-up. Gặp nhau ở `sense()`
   trong `vehicle.py`.
2. Đảo y và z. Trục x (Forward) giống nhau ở cả FLU và FRD.
3. `tau_z = Σ sᵢ·k_q·ωᵢ²` với `sᵢ = ±1` theo chiều quay. Nếu cả 4 quay cùng chiều,
   tổng mô men phản lực không triệt tiêu → drone tự xoay; hai cặp ngược chiều triệt
   tiêu nhau khi bay treo và cho phép điều khiển yaw bằng chênh lệch.
4. RK4 lấy 4 mẫu độ dốc mỗi bước và pha trọng số 1-2-2-1; sai số `O(dt⁴)` so với
   `O(dt²)` của Euler.
5. Sai số số học làm vector 4 chiều rời khỏi mặt cầu đơn vị; chuẩn hóa giữ nó là
   quaternion hợp lệ.
6. `ω_h = sqrt(0.04·9.81/(4·4.7e-8)) ≈ 1445 rad/s`; duty ≈ 0.578.

</details>

---

*Tiếp theo: `04-dong-co-pwm-va-pin.md` — từ duty đến lực đẩy và nguồn điện.*
