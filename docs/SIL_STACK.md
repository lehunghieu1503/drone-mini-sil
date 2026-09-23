> **Banner:** đây là bản phác thảo. Source of truth là code C++20 trong
> `drone-mini/firmware/`; điều khiển và plant theo `drone-mini/README.md`.
> Tài liệu này để đọc hiểu, không phải hợp đồng.
>
> Bản root và bản `drone-mini/docs/` giữ nội dung như nhau.
> Khi lệch với code, code thắng.

# SIL / HIL stack — Drone Mini (ESP32-C3)

Tài liệu này mô tả stack giả lập và các bước làm **Software-in-the-Loop (SIL)** rồi **Hardware-in-the-Loop (HIL)** cho board Drone Mini: ESP32-C3 Super Mini, IMU ICM-20948, 4 motor brushed low-side MOSFET, RC UART, nguồn 1S.

Mục tiêu không phải “bay đẹp trong Gazebo”. Mục tiêu là **cùng một firmware** được kiểm trên plant trước khi gắn cánh.

---

## 1. Nguyên tắc

1. **Một firmware, hai HAL.** Estimator, PID, mixer, arm, failsafe, LED viết một lần. Chỉ lớp HAL khác nhau: `hal_hw.c` (chip) và `hal_sil.c` (plant).
2. **Không viết lại PID bằng Python rồi gọi là SIL.** Python chỉ là plant + runner + log. Controller là C trong `firmware/flight/`.
3. **Lockstep.** Plant là master clock. Mỗi tick: IMU/RC vào firmware → firmware chạy 1 vòng → PWM ra plant.
4. **Cùng schema log** giữa SIL, HIL và bay thật để overlay.
5. **Ba tầng, không nhảy cóc:**
   - A — SIL host
   - B — HIL chip thật
   - C — bay ràng buộc (dây / lồng)

MuJoCo, Gazebo, Isaac, AirSim **không** nằm trong xương sống. Chúng chỉ thêm sau nếu cần va chạm, RL, hoặc visual.

> **Cập nhật (visual opt-in):** tầng live 3D đã có — `make view` (viewer process riêng) +
> `make visual` (runner publish pose qua UDP). MuJoCo vẫn **ngoài xương sống**: nó là
> viewer/plant opt-in, không nằm trên đường tới hạn của firmware hay `make gate`.
> Chi tiết: `drone-mini/note/14-visual-live-mujoco.md`.

---

## 2. Phần cứng cần bám

Lấy từ schematic `Drone mini` (Power / ESP32_C3 / Motor) và PCB.

### 2.1 Nguồn

| Tín hiệu | Ghi chú |
|---|---|
| `V_BAT` | Pin 1S vào `J1`. Motor lấy **thẳng** `V_BAT`. |
| `+3V3` | LDO RT9193-33GB, 3.3 V / 2 A, nuôi ESP32-C3 và ICM-20948. |
| Tụ | Input C1 1 µF, C2 0.1 µF. BP C5 22 nF. Output C3 1 µF, C4 0.1 µF. ESP thêm C6 10 µF, C7/C8. |

Plant **không** được cho motor ăn 3.3 V.

### 2.2 GPIO đã chốt trên schematic

| Chức năng | GPIO | Ghi chú HW |
|---|---|---|
| Mot1 | GPIO0 | MOSFET Q1 IRLML2502, Rgate 100 Ω, pulldown 10 kΩ |
| Mot2 | GPIO1 | Q2, cùng topology |
| Mot3 | GPIO2 | Q3 |
| Mot4 | GPIO3 | Q4 |
| LED (R/G) | GPIO4 | LED 2 màu, điện trở 1 kΩ |
| IMU SDA | GPIO6 | ICM-20948 |
| IMU SCL | GPIO7 | I2C |
| IMU INT | GPIO8 | Data ready |
| RC RX | GPIO20 | Module RC `J6` |
| RC TX | GPIO21 | |

Mỗi kênh motor: `V_BAT` → diode 1N4148W + tụ 1 µF → connector 2P → MOSFET low-side xuống GND.

PWM đề xuất trên chip: LEDC ~20–32 kHz. Idle = 0 (đúng pulldown 10 kΩ — MOSFET tắt khi chưa arm).

### 2.3 Việc **chưa** chốt từ schematic

- Vị trí vật lý Mot1–4 trên khung X (nhìn từ trên, mũi drone).
- Chiều quay CW / CCW từng motor.
- Protocol RC trên `J6` (SBUS / CRSF / IBUS / custom), baud, inverted hay không.

GPIO mapping đã khóa. **Mixer dấu yaw và thứ tự cánh phải đối chiếu PCB trang layout trước khi bay.** Test bench: tăng từng GPIO, xem motor nào quay, chiều nào.

### 2.4 Mixer tạm — chỉ dùng sau khi đối chiếu PCB

Quy ước làm việc (nhìn từ trên, mũi = +X body). **Đây là giả định Betaflight-style X, chưa phải sự thật trên board.**

| Motor | GPIO | Vị trí giả định | Quay giả định |
|---|---|---|---|
| Mot1 | GPIO0 | sau-phải | CW |
| Mot2 | GPIO1 | trước-phải | CCW |
| Mot3 | GPIO2 | sau-trái | CCW |
| Mot4 | GPIO3 | trước-trái | CW |

Mix tuyến tính (đủ cho SIL đầu, clip 0…1):

```
Mot1 = T - roll + pitch - yaw
Mot2 = T - roll - pitch + yaw
Mot3 = T + roll + pitch + yaw
Mot4 = T + roll - pitch - yaw
```

Unit test mixer phải fail nếu đảo dấu yaw hoặc đảo cặp roll. Khi đã đo PCB, sửa bảng trên rồi khóa trong `mixer.c` + `drone_mini_params.py`.

---

## 3. Kiến trúc

```
                 ┌──────────────────────────────────┐
                 │  firmware/flight/                │
                 │  estimator · pid · mixer         │
                 │  failsafe · led · rc parse       │
                 └──────────────┬───────────────────┘
                                │ hal.h
                 ┌──────────────┴──────────────┐
                 │                             │
            hal_hw.c                      hal_sil.c
         ESP32-C3 thật                 socket / shm
         LEDC · I2C · UART             struct packed
                 │                             │
            Board thật                   plant Python
                                       (RotorPy hoặc RK4)
```

Plant là mô hình:

- 6-DOF rigid body
- 4 motor bậc 1 + thrust \(k_t \omega^2\)
- Pin Thevenin 1S
- IMU noisy theo datasheet ICM-20948
- RC UART (cùng protocol firmware parse)

---

## 4. Cấu trúc repo

```
drone-mini/
  firmware/
    CMakeLists.txt
    sdkconfig.defaults
    main/
      app_main.c
      flight/
        estimator.c / estimator.h
        pid.c        / pid.h
        mixer.c      / mixer.h
        failsafe.c   / failsafe.h
        led.c        / led.h
      drivers/
        icm20948.c   / icm20948.h    # chỉ gọi từ hal_hw
        rc_uart.c    / rc_uart.h
      hal/
        hal.h
        hal_hw.c
        hal_sil.c
  plant/
    requirements.txt
    drone_mini_params.py
    vehicle.py
    icm20948.py
    battery.py
    rc.py
    runner.py
  tools/
    plot_sil.py
    compare_logs.py
  tests/
    test_mixer.py
    test_failsafe.py
  docs/
    SIL_STACK.md          # file này
```

Quy tắc:

- `flight/` **không** `#include` header ESP-IDF, không gọi `ledc_*`, `i2c_*`.
- `drivers/` chỉ được `hal_hw.c` dùng.
- `hal_sil.c` nói chuyện với `plant/runner.py`.

### Build

Chip:

```bash
cd firmware
idf.py set-target esp32c3
idf.py build flash monitor
```

SIL host — **không** dựa vào ESP-IDF linux target cho PWM/I2C (component `driver` chỉ mock, không giả lập LEDC/I2C/UART). Cách sạch:

- Compile `flight/*` + `hal_sil.c` bằng CMake native (gcc trên PC).
- `app_main` SIL: vòng lockstep, không FreeRTOS nếu chưa cần.

ESP-IDF `idf.py --preview set-target linux` chỉ đáng khi muốn test phần IDF thuần (timer, log). Không dùng làm plant motor.

---

## 5. HAL contract

`firmware/main/hal/hal.h`:

```c
#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float mot[4];          /* 0..1, Mot1=GPIO0 ... Mot4=GPIO3 */
} pwm_cmd_t;

typedef struct {
    float gyro_rps[3];     /* rad/s, body */
    float accel_mps2[3];   /* m/s^2, specific force */
    float mag_uT[3];       /* tùy chọn */
    float temp_c;
    uint64_t t_us;
    bool valid;
} imu_sample_t;

typedef struct {
    float roll;            /* -1 .. 1 */
    float pitch;
    float yaw;
    float throttle;        /*  0 .. 1 */
    bool  armed_switch;
    bool  frame_ok;
    uint64_t t_us;
} rc_sample_t;

typedef enum {
    LED_OFF = 0,
    LED_BOOT,
    LED_CALIB,
    LED_ARMED,
    LED_ERROR
} led_mode_t;

void     hal_init(void);
uint64_t hal_now_us(void);
void     hal_pwm_write(const pwm_cmd_t *cmd);
bool     hal_imu_read(imu_sample_t *out);
bool     hal_rc_read(rc_sample_t *out);
void     hal_led_set(led_mode_t m);
float    hal_vbat_read(void);
void     hal_log(const char *fmt, ...);
```

### Ánh xạ HW / SIL

| API | `hal_hw.c` | `hal_sil.c` |
|---|---|---|
| `hal_pwm_write` | LEDC GPIO0–3, idle 0 | Gửi 4 float cho plant |
| `hal_imu_read` | I2C ICM-20948, WHO_AM_I = `0xEA` | Packet plant, đã scale SI |
| `hal_rc_read` | UART GPIO20/21 | Cùng struct hoặc cùng raw frame |
| `hal_now_us` | `esp_timer_get_time()` | Clock plant |
| `hal_vbat_read` | ADC nếu có, không thì NaN | \(V_{oc} - I R_{int}\) |
| `hal_led_set` | GPIO4 | Enum ghi log |

Giao thức lockstep gợi ý: struct packed binary trên TCP localhost hoặc stdin/stdout. JSON cũng được cho bước đầu, chậm hơn.

Thứ tự một tick:

1. Plant bước \(\Delta t\).
2. Plant gửi `imu_sample_t` + `rc_sample_t` + `vbat`.
3. Firmware: `estimator → failsafe → pid → mixer`.
4. Firmware gửi `pwm_cmd_t`.
5. Plant map duty → \(\omega_{cmd}\) → motor bậc 1.
6. Ghi một dòng log.

---

## 6. Model plant

Hai lựa chọn, cùng phương trình.

### 6.1 Nên dùng: RotorPy

```bash
pip install rotorpy numpy matplotlib
```

- Lớp `Multirotor` + file param riêng `drone_mini_params.py`.
- Input đúng tầng firmware: `cmd_motor_speeds` (hoặc `cmd_motor_thrusts` nếu map PWM→thrust ở runner).
- Có wrench khí động, gió, IMU sẵn — chỉnh noise cho khớp ICM-20948.
- Xương param Crazyflie (cùng lớp nano), **không** để nguyên số Crazyflie.

Tham số xương Crazyflie trong RotorPy (chỉ để copy rồi sửa):

| Key | CF mặc định | Việc phải làm |
|---|---|---|
| `mass` | 0.03 kg | Cân board + pin + 4 motor + cánh |
| arm `d` | 0.043 m | Offset Descartes `(±d, ±d)` của mỗi motor; bán kính tâm→motor là `d√2` = 60.8 mm. Đo PCB (Tier C) rồi mới đổi tọa độ |
| `k_eta` | \(2.3\times 10^{-8}\) N/(rad/s)\(^2\) | Hiệu chỉnh hover |
| `k_m` | \(7.8\times 10^{-10}\) Nm/(rad/s)\(^2\) | Ước lượng, tinh sau |
| `tau_m` | 0.072 s | Brushed 1S: bắt đầu **0.03 s** (20–50 ms) |
| `rotor_speed_max` | 2500 rad/s | Đo hoặc ước từ KV × V_BAT |

`rotor_pos` và `rotor_directions` phải khớp mixer đã khóa.

### 6.2 Fallback: RK4 ~300 dòng

Đủ cho tầng A. Không contact, không ground effect.

Động cơ:

\[
\dot{\omega}_i = \frac{1}{\tau_m}\big(k_u\, u_i\, \tfrac{V_{bat}}{V_{nom}} - \omega_i\big)
\]

\[
T_i = k_t\,\omega_i^2, \qquad Q_i = s_i\, k_q\,\omega_i^2
\]

\(u_i \in [0,1]\) là duty GPIO, \(s_i = \pm 1\) chiều quay, \(\tau_m \approx 0.03\,\mathrm{s}\).

Tịnh tiến / quay:

\[
m\dot{\mathbf{v}} = m\mathbf{g} + R\sum_i T_i\mathbf{e}_3 + \mathbf{F}_{aero}
\]

\[
I\dot{\boldsymbol{\omega}} + \boldsymbol{\omega}\times I\boldsymbol{\omega}
= \sum_i \big(\mathbf{r}_i \times T_i\mathbf{e}_3 + Q_i\mathbf{e}_3\big)
\]

Hiệu chỉnh \(k_t\) từ hover:

\[
4\,k_t\,\omega_h^2 = mg
\]

### 6.3 Pin Thevenin

\[
V_{bat} = V_{oc}(SoC) - I_{tot} R_{int}
\]

- \(V_{oc}\): ~4.20 V đầy → ~3.30 V cắt.
- Motor kéo từ \(V_{bat}\). MCU/IMU ở 3.3 V LDO.
- Brownout logic: khi \(V_{bat} < V_{dropout} + 3.3\) (RT9193, cỡ ~3.5 V tùy dòng).

Đừng để 4 motor spike làm 3V3 sụt trong plant nếu LDO + tụ output còn headroom — đúng schematic.

### 6.4 IMU — bám datasheet ICM-20948 (DS-000189 rev 1.6)

| Đại lượng | Typical |
|---|---|
| Gyro noise density | 0.015 dps/√Hz |
| Gyro ZRO | ±5 dps (component) |
| Gyro FSR | ±250 / 500 / 1000 / 2000 dps |
| Gyro scale | 131 / 65.5 / 32.8 / 16.4 LSB/dps |
| Accel noise density | 230 µg/√Hz |
| Accel zero-g board-level | ±50 mg |
| Accel FSR | ±2 / 4 / 8 / 16 g |
| Accel scale | 16384 / 8192 / 4096 / 2048 LSB/g |
| Mag | ±4900 µT |
| I2C | 400 kHz |
| Startup gyro / accel | ~35 ms / ~20–30 ms |
| WHO_AM_I | `0xEA` |

Whoop nhỏ: **đừng tin mag cho yaw** (dòng motor + PCB). Dùng mag chỉ để debug.

Model đo:

\[
\boldsymbol{\omega}_{meas} = R_{mis}(\boldsymbol{\omega} + \mathbf{b}_g + \mathbf{n}_g)
\]

\[
\mathbf{a}_{meas} = R_{mis}\big(R^\top(\dot{\mathbf{v}}-\mathbf{g}) + \mathbf{b}_a + \mathbf{n}_a\big)
\]

\(\mathbf{n}_g, \mathbf{n}_a\) trắng theo NSD trên. Bias random walk nhỏ. Quantize 16-bit đúng FSR firmware chọn (whoop thường ±2000 dps / ±16 g).

Plant “sạch” (không nhiễu, không bias) chỉ để debug mixer. Pass SIL phải bật nhiễu.

---

## 7. Vòng firmware

Tần số đề xuất trên ESP32-C3:

| Vòng | Tần số | Nguồn |
|---|---|---|
| IMU raw | 500–1000 Hz | INT GPIO8 hoặc poll |
| Rate PID | 1–2 kHz nếu IMU cho phép, không thì = ODR | gyro |
| Attitude + fusion | 200–500 Hz | accel + gyro |
| RC parse | theo frame | UART |
| Failsafe | mỗi vòng control | timeout RC + V_BAT |
| LED | 20–50 Hz | FSM |

Thứ tự một control tick:

```
hal_imu_read
hal_rc_read
hal_vbat_read
failsafe_update      # mất sóng / chưa arm → pwm = 0
estimator_update     # gyro integrate + accel correct
pid_rate / pid_att
mixer_write          # 4 duty
hal_pwm_write
hal_led_set
log
```

Chưa arm: 4 kênh = 0. Đúng pulldown MOSFET.

---

## 8. Các bước làm

### Bước 0 — Đo board (1 buổi, trước khi code plant)

- [ ] Cân mass: khung + pin 1S, không cánh và có cánh.
- [ ] Đo arm length 4 hướng.
- [ ] Tăng từng GPIO0–3 trên bench (duty thấp, **không cánh**): ghi motor nào quay, chiều nào.
- [ ] Chốt protocol `J6`: baud, inverted, thứ tự kênh.
- [ ] Tạo repo đúng layout mục 4. `flight/` chưa đụng driver.

### Bước 1 — Unit test mixer (không cần plant)

- [ ] Input throttle/roll/pitch/yaw chuẩn hóa.
- [ ] Output `mot[0..3]` = GPIO0–3.
- [ ] Hover: 4 kênh gần bằng nhau.
- [ ] Roll dương: đúng 2 motor một phía tăng.
- [ ] Yaw dương: đúng cặp chéo.
- [ ] Clip 0…1 + desaturation khi một kênh = 1.

Fail = mapping GPIO sai hoặc dấu yaw ngược. Sửa trước khi nối plant.

### Bước 2 — SIL A.1: plant open-loop

- [ ] Runner lockstep chạy.
- [ ] 4 PWM bằng nhau, chưa PID: drone tăng độ cao, không spin yaw.
- [ ] Cắt PWM: rơi, \(\omega\) về 0 theo \(\tau_m\).
- [ ] Log PWM, \(\omega\), vị trí, attitude.

### Bước 3 — SIL A.2: rate PID

- [ ] Attitude loop tắt.
- [ ] Step roll rate / pitch rate / yaw rate.
- [ ] Anti-windup khi motor bão hòa.
- [ ] Settling cỡ < 0.3 s, không dao động không tắt.

### Bước 4 — SIL A.3: attitude + hover

- [ ] Fusion (complementary hoặc Mahony) trên IMU **có nhiễu + bias**.
- [ ] Hover 10 s: |roll|, |pitch| < 5°.
- [ ] Drift yaw chấp nhận được nếu không dùng mag.
- [ ] Step stick nhỏ rồi thả: về gần 0.

### Bước 5 — SIL A.4: RC + arm + failsafe

- [ ] Parser cùng protocol chip sẽ dùng.
- [ ] Chưa arm → PWM = 0.
- [ ] Mất frame > timeout → failsafe (cắt hoặc hạ throttle đã định nghĩa).
- [ ] Frame lỗi / sai endian không arm nhầm.

### Bước 6 — SIL A.5: pin + LED

- [ ] Hover lâu: \(V_{BAT}\) tụt theo Thevenin.
- [ ] 3V3 ổn đến ngưỡng dropout.
- [ ] LED: `BOOT → CALIB → ARMED → ERROR` đúng phase.

### Bước 7 — SIL A.6: overlay log

- [ ] Cùng format CSV/ulog giữa các tầng.
- [ ] Plot PWM 4 kênh, gyro, attitude, RC, V_BAT.
- [ ] Không pass bằng “nhìn có vẻ bay”.

### Bước 8 — HIL (tầng B)

Cùng `flight/`, đổi `hal_hw.c`.

- [ ] Flash ESP32-C3.
- [ ] PWM ra PC: UART telemetry 4 duty, hoặc đo LEDC.
- [ ] PC nhét IMU + RC giả (packet UART, hoặc I2C bridge nếu muốn test driver).
- [ ] Lặp case bước 3–6 với clock chip thật.
- [ ] Thêm: period vòng lặp không trượt, I2C không timeout, UART không overflow, WDT không cắn.

HIL bắt jitter / DMA / blocking — SIL host chạy nhanh hơn đời thật nên **không** thay HIL.

### Bước 9 — Bay ràng buộc (tầng C)

- [ ] A và B đã xanh.
- [ ] Dây hoặc lồng, prop nhỏ, người ngoài vòng quay cánh.
- [ ] Arm lần đầu throttle thấp.
- [ ] Overlay log HIL vs log bay: rung IMU, motor lệch, sag pin thật.

Sim không thay tầng C cho rung PCB cứng + ICM-20948 sát khung.

---

## 9. Tiêu chí pass

Cùng logic firmware (khác mỗi HAL) phải làm được:

1. Arm → 4 PWM tăng đều → hover, không drift yaw lớn.
2. Step roll / pitch đúng dấu 4 motor (GPIO0–3).
3. Cắt RC → failsafe trong đúng timeout đã ghi.
4. IMU bias + nhiễu datasheet không làm lật trong 10 s hover.
5. Log SIL và HIL cùng schema, overlay được.
6. Chưa arm: 4 kênh = 0.

Chưa đủ 6 ý thì chưa xong tầng A/B.

---

## 10. Việc không làm

| Việc | Lý do |
|---|---|
| Viết PID Python rồi gọi SIL | Hai bộ não, lệch chip |
| Raw MuJoCo / Isaac / AirSim làm xương | Không có aero + HAL + IMU datasheet |
| Gazebo + PX4/ArduPilot | Chỉ đúng nếu **bỏ** firmware tự viết |
| Liftoff / VelociDrone | Tập tay lái, không test code |
| IMU sim sạch | Fusion luôn đẹp, ra bàn lật |
| \(\tau_m\) 5–10 ms như coreless tốt | Plant nhanh hơn motor brushed thật |
| Motor nuôi 3V3 trong plant | Sai schematic |
| Tin mag ICM-20948 trên whoop | Nhiễu dòng + sắt motor |
| Bay không dây khi chưa đo chiều quay | Lật ngay arm |
| Bỏ HIL vì SIL host “đã hover” | Jitter chip không xuất hiện trên PC |

MuJoCo-Drones-Gym / gym-pybullet-drones chỉ thêm **sau** tầng A, nếu cần RL hoặc va chạm. Không thay `hal_sil` + plant lockstep.

---

## 11. Gợi ý runner tối thiểu

`plant/runner.py` (ý, không copy-paste xong chạy):

```python
# 1. load drone_mini_params
# 2. Multirotor(...) hoặc RK4 plant
# 3. mở socket / pipe tới firmware SIL
# 4. loop:
#       imu, rc, vbat = plant.sense(t)
#       send(imu, rc, vbat)
#       pwm = recv()                  # 4 float 0..1
#       omega_cmd = pwm_to_omega(pwm, vbat)
#       plant.step(dt, omega_cmd)
#       log_row(t, pwm, state, imu, rc)
```

Map PWM → \(\omega_{cmd}\) lần đầu:

\[
\omega_{cmd} = u \cdot \omega_{max}(V_{bat})
\]

Sau có thrust stand thì đổi sang bảng \(u \mapsto T\) rồi \(T \mapsto \omega\).

---

## 12. Log schema đề xuất

Một dòng / control tick, CSV:

```
t_us, roll_cmd, pitch_cmd, yaw_cmd, thr_cmd, armed, failsafe,
gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z,
roll_est, pitch_est, yaw_est,
mot0, mot1, mot2, mot3,
vbat, led_mode
```

SIL, HIL, bay thật dùng chung header. `tools/compare_logs.py` overlay cùng trục thời gian.

---

## 13. SPICE (tách khỏi SIL bay)

Không cần để hover trong SIL. Làm 1 kênh nếu nghi FET/diode/LDO:

- IRLML2502, Vgs = 3.3 V, Rgate 100 Ω, PWM 20–32 kHz.
- Flyback 1N4148 + 1 µF khi tắt motor.
- LDO RT9193 khi 4 motor spike trên `V_BAT` — 3V3 có tụt dưới 3.0 V không.

Pass SPICE không đồng nghĩa pass SIL.

---

## 14. Thứ tự ưu tiên khi thiếu thời gian

1. Mixer unit test + đo chiều quay PCB.
2. Plant RK4 hoặc RotorPy + HAL SIL + rate PID.
3. Failsafe / arm.
4. Hover SIL có nhiễu IMU.
5. HIL period + UART/I2C.
6. Bay dây.

Bỏ visual, bỏ RL, bỏ mag yaw, bỏ Gazebo.

> **Cập nhật:** visual đã có ở dạng **opt-in ngoài đường tới hạn** (`make view` +
> `make visual`, viewer process riêng). Ưu tiên khi thiếu thời gian vẫn giữ nguyên:
> visual không chặn tiến độ tầng A/B/C.

---

## 15. Tài liệu gốc

- Schematic / PCB / BOM: file `Drone mini.PDF` (Power, ESP32_C3, Motor, layout, BOM).
- ICM-20948 datasheet DS-000189 rev 1.6 — noise density, FSR, WHO_AM_I.
- RotorPy: `pip install rotorpy`, param tham khảo `rotorpy.vehicles.crazyflie_params`.
- ESP-IDF host apps: linux target hữu ích cho unit test IDF, **không** giả lập LEDC/I2C/UART đủ cho SIL drone.

---

*Cập nhật khi đã đo mass, arm, chiều quay 4 motor và protocol RC. Ba số đó khóa mixer và `drone_mini_params.py` — không đoán mãi.*
