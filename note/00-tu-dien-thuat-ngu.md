# 00 — Từ điển thuật ngữ

> Tra cứu bất cứ lúc nào. Mỗi thuật ngữ kèm: nghĩa ngắn — vì sao quan trọng — xuất hiện
> ở đâu trong repo. Nhóm **in đậm** là nhóm chủ đề.

---

## A. Hệ thống mô phỏng

| Thuật ngữ | Nghĩa | Trong repo |
|---|---|---|
| **SIL** — Software-in-the-Loop | Chạy firmware thật (hoặc code điều khiển thật) với **plant mô phỏng**, tất cả trên PC | `sim/main_sil.cpp`, `plant/runner.py` |
| **HIL** — Hardware-in-the-Loop | Chạy firmware trên **chip thật**, nhưng cảm biến/motor là giả lập qua UART | Phase 9, `Kconfig.projbuild` |
| **Plant** | Mô hình toán học của drone + môi trường (vật lý, motor, pin, IMU) | `plant/vehicle.py` |
| **Lockstep** | Plant là **master clock**: plant bước 1 tick → gửi cảm biến → firmware chạy 1 vòng → trả PWM → lặp lại. Không bên nào chạy trước bên nào | `plant/runner.py:125`, `sim/main_sil.cpp:123` |
| **Determinism** (tính tất định) | Cùng seed + cùng input → cùng log, bit-for-bit. Chỉ RK4 path đảm bảo điều này | Plan D20, `plant/vehicle.py` |
| **Golden log / overlay** | Log SIL, HIL và bay thật dùng **cùng schema** để chồng đồ thị so sánh | `plant/log.py`, `tools/compare_logs.py` |

## B. Hệ quy chiếu & hình học

| Thuật ngữ | Nghĩa | Trong repo |
|---|---|---|
| **World frame** | Hệ quy chiếu gắn với mặt đất. Ở đây dùng **ENU**: x=East, y=North, z=**Up** | `vehicle.py:2` |
| **Body frame** | Hệ quy chiếu gắn với drone. Plant dùng **FLU**: x=Forward, y=Left, z=Up | `vehicle.py:51` |
| **FRD** | Forward–Right–Down. Chuẩn hàng không cho gyro/accel. Firmware dùng FRD; plant dùng FLU → đổi trục `(x, −y, −z)` | `vehicle.py:40` (`to_frd`) |
| **Roll / Pitch / Yaw** | Góc quay quanh trục x / y / z. Roll = nghiêng cánh trái-phải; pitch = chúi mũi; yaw = xoay ngang | `estimator.cpp` |
| **Quaternion** | Cách biểu diễn hướng 3D bằng 4 số `(w,x,y,z)` không bị "gimbal lock" | `vehicle.py:16-37` |
| **Rotation matrix R** | Ma trận 3×3 biến vector từ body sang world (và ngược lại bằng `R.T`) | `vehicle.py:27` |
| **Euler angles** | Roll/pitch/yaw — dễ hiểu nhưng có điểm kỳ dị, thường chỉ dùng để log/hiển thị | `vehicle.py:143` |
| **Specific force** | Lực trên mỗi đơn vị khối lượng **trừ trọng lực**; đó chính là thứ gia tốc kế đo | `icm20948.py`, `vehicle.py:100` |

> **Quy ước dấu của repo:** pitch dương = **mũi chúi xuống** (nose-down positive);
> yaw dương = xoay theo chiều kim đồng hồ nhìn từ trên xuống khi dùng FRD.
> Đây là quy ước nội bộ, ghi rõ để không lẫn.

## C. Động lực học & chấp hành

| Thuật ngữ | Nghĩa | Trong repo |
|---|---|---|
| **6-DOF** | 6 bậc tự do: 3 tịnh tiến (vị trí) + 3 quay (hướng) | `vehicle.py` state 17 phần tử |
| **RK4** | Runge–Kutta bậc 4 — phương pháp tích phân số chính xác cao để giải phương trình vi phân chuyển động | `vehicle.py:82-86` |
| **Thrust** | Lực đẩy của cánh quạt, tỉ lệ bình phương vận tốc góc: `T = k_t·ω²` | `drone_mini_params.py:39` |
| **Torque (mô men)** | Xoắn quanh trục, sinh từ chênh lệch lực 4 motor (roll/pitch) và phản lực cánh quạt (yaw) | `vehicle.py:70-74` |
| **Inertia (quán tính)** | Ma trận `I` — "khối lượng quay" quanh từng trục | `drone_mini_params.py:42` |
| **Duty** | Tỉ lệ phần trăm PWM, chuẩn hóa 0..1 trong repo | `PwmCmd.mot[4]` |
| **ESC / MOSFET** | Mạch công suất điều khiển motor. Board này dùng MOSFET low-side (IRLML2502) | `SIL_STACK.md` §2.2 |
| **`τ_m`** (tau_m) | Hằng số thời gian motor — motor không đạt vận tốc mới ngay lập tức mà theo hàm mũ | `drone_mini_params.py:38` |
| **`k_t`, `k_q`** | Hệ số thrust và torque của cánh quạt | `drone_mini_params.py:39-40` |
| **Hover** | Trạng thái bay treo: tổng lực đẩy = trọng lượng, drone đứng yên trong không khí | `hover_duty_nominal()` |
| **Desaturation** | Khi 1 motor vượt biên 0..1, **dịch tất cả** kênh để giữ tỉ lệ điều khiển | `mixer.cpp:14` |

## D. Cảm biến

| Thuật ngữ | Nghĩa | Trong repo |
|---|---|---|
| **IMU** — Inertial Measurement Unit | Cảm biến quán tính, thường = gyro + accel (+ mag). Ở đây là ICM-20948 | `icm20948.py` |
| **Gyro** | Đo **vận tốc góc** (rad/s). Nhanh, ít nhiễu, nhưng trôi theo thời gian (drift) | `estimator.cpp:79-85` |
| **Accelerometer** | Đo **specific force** (m/s²). Chậm hơn, nhiễu hơn khi rung, nhưng biết được "hướng trọng lực" để chống trôi | `estimator.cpp:69-76` |
| **Bias / ZRO** | Zero-Rate Offset — gyro báo khác 0 khi đứng yên. Nguồn sai số lớn nhất; phải hiệu chuẩn lúc boot | `estimator.cpp:31-53` |
| **Noise density (NSD)** | Mật độ phổ nhiễu (dps/√Hz). Nhân `sqrt(fs/2)` để ra sigma nhiễu trắng tại tần số lấy mẫu | `drone_mini_params.py:28-29` |
| **FSR** — Full Scale Range | Dải đo tối đa (ví dụ ±2000 dps gyro, ±16 g accel) | `sensor_cfg.hpp` |
| **LSB** | Least Significant Bit — đơn vị nhỏ nhất ADC đọc được; 16.4 LSB/dps nghĩa là độ phân giải | `sensor_cfg.hpp:11-17` |
| **Quantization** | Làm tròn số đo về bội số của LSB | `icm20948.py:31-33` |
| **ODR** — Output Data Rate | Tần số cảm biến đẩy dữ liệu ra. ICM-20948 không có 1000 Hz → dùng 1125 Hz rồi **decimate 9/8** | `sensor_cfg.hpp:19-21` |
| **Decimation** | Bỏ bớt mẫu để hạ tần số, ở đây bỏ 1 trong 9 mẫu | `estimator.hpp:19` (kEstDecim) |
| **Mag / magnetometer** | Từ kế. Repo **không dùng** cho yaw vì nhiễu dòng motor + sắt trên khung nhỏ | `SIL_STACK.md` §6.4 |

## E. Điều khiển

| Thuật ngữ | Nghĩa | Trong repo |
|---|---|---|
| **Setpoint (sp)** | Giá trị mong muốn (góc hoặc vận tốc góc) | `RateSp` |
| **Measured (meas)** | Giá trị đo được | gyro |
| **Error (e)** | `sp − meas` — sai số điều khiển | `pid.cpp:33` |
| **P / I / D** | Tỉ lệ / Tích phân / Đạo hàm — 3 thành phần của PID | `pid.cpp:31-56` |
| **Anti-windup** | Chống tích lũy I khi output đã bão hòa (nếu không, drone "đơ" khi nhả) | `pid.cpp:45-50` |
| **Derivative kick** | Cú giật D khi setpoint thay đổi đột ngột → tránh bằng "D on measurement" | `pid.cpp:35-42` |
| **LPF** — Low-Pass Filter | Lọc thông thấp bậc 1, làm mượt tín hiệu/đạo hàm | `pid.cpp:40-41` |
| **Cascade control** | Vòng ngoài attitude → đặt setpoint vòng trong rate → PID → mixer. Vòng trong nhanh hơn vòng ngoài | `control_loop.cpp:59-69` |
| **Rate loop** | Vòng điều khiển vận tốc góc (inner loop) | `RateController` |
| **Saturation** | Output PID chạm giới hạn → báo cho anti-windup | `rate_out_limit` |
| **Settling time** | Thời gian đáp ứng về gần giá trị đích (< 0.3 s ở repo này) | `docs/measurements.md` |
| **Overshoot** | Vọt lố quá giá trị đích trước khi ổn định | tests T5.x |

## F. RC, an toàn, trạng thái

| Thuật ngữ | Nghĩa | Trong repo |
|---|---|---|
| **RC** — Remote Control | Bộ phát/thu điều khiển từ xa. 4 kênh cơ bản: roll, pitch, throttle, yaw + kênh arm | `rc_parse.cpp` |
| **SBUS** | Giao thức serial 25 byte, 16 kênh × 11 bit, chạy 100 kbaud đảo tín hiệu | `rc_parse.cpp` |
| **Arm / Disarm** | "Bật" / "tắt" động cơ. Chưa arm → PWM = 0 tuyệt đối | `failsafe.cpp` |
| **Failsafe** | Cơ chế an toàn tự động khi mất sóng / pin yếu / IMU lỗi → ngắt động cơ | `failsafe.cpp:33-98` |
| **Latch** | Khóa trạng thái lỗi lại — lỗi xảy ra thì phải "arm lại từ đầu" mới bay tiếp, không tự phục hồi | `failsafe.cpp:71-75` |
| **Debounce** | Chờ vài mẫu lỗi liên tiếp mới kết luận (tránh báo động giả) | `kImuInvalidDebounce = 5` |
| **Brownout** | Sụt áp nguồn làm chip reset. Boot lại phải đọc lý do reset và khóa an toàn | `failsafe.cpp:22-31` |
| **Watchdog (WDT)** | Bộ đếm phần cứng: nếu code treo không "cho ăn", chip tự reset | `sdkconfig.defaults` |

## G. Kiến trúc phần mềm & nhúng

| Thuật ngữ | Nghĩa | Trong repo |
|---|---|---|
| **HAL** — Hardware Abstraction Layer | Lớp trừu tượng che giấu phần cứng; code bay chỉ gọi `IHal`, không biết chip hay plant | `flight/ports.hpp:49` |
| **Port / Adapter** | Kiến trúc hexagonal: `IHal` là port (giao diện), `SilHal`/`HwHal`/`MockHal` là adapter | `firmware/main/hal/` |
| **DI** — Dependency Injection | Truyền phụ thuộc từ ngoài vào thay vì tự tạo bên trong; ở đây là `FlightContext` giữ reference | `flight/context.hpp:14` |
| **Interface / Pure virtual** | Hợp đồng chỉ có hàm ảo thuần (`= 0`), không hiện thực; mọi adapter `final` kế thừa | `ports.hpp:49-141` |
| **POD** | Plain Old Data — struct chỉ có số liệu, không hàm ảo. Bắt buộc cho wire ABI | `ports.hpp:12-42` |
| **`#pragma pack(1)`** | Bỏ padding của compiler để layout struct khớp chính xác giữa C++ và Python | `sil_wire.hpp:32` |
| **`static_assert`** | Kiểm tra lúc biên dịch; ở đây chặn mọi thay đổi kích thước struct làm vỡ ABI | `sil_wire.hpp:106-115` |
| **ABI** | Application Binary Interface — "hợp đồng" nhị phân giữa 2 chương trình (C++ ↔ Python) | `test_abi.py` |
| **CRC32** | Checksum phát hiện lỗi truyền dữ liệu | `sil_wire.cpp:23-40` |
| **Realtime** | Ràng buộc thời gian cứng: vòng lặp 1 kHz phải xong trong 1 ms | `kControlDtS = 0.001f` |
| **`-fno-exceptions -fno-rtti`** | Tắt exception và RTTI để giảm kích thước, tăng tốc, tránh bất định thời gian | `firmware/CMakeLists.txt` |
| **`constinit`** | Đảm bảo object tĩnh được khởi tạo lúc compile, tránh thứ tự khởi tạo rối rắm | `sim/main_sil.cpp` |

## H. Giao thức SIL

| Thuật ngữ | Nghĩa | Trong repo |
|---|---|---|
| **Wire protocol** | Định dạng nhị phân trao đổi giữa firmware và plant | `sil_wire.hpp` |
| **Header (Hdr)** | 14 byte đầu mọi message: magic, version, type, length, seq, CRC | `sil_wire.hpp:34-41` |
| **HELLO handshake** | Bắt tay đầu phiên: 2 bên so 7 kích thước struct + dt để chắc chắn cùng version | `sil_wire.cpp:86-97` |
| **UDS** — Unix Domain Socket | Socket nội bộ máy, nhanh hơn TCP. Repo dùng `AF_UNIX SOCK_SEQPACKET` abstract namespace | `hal_sil.cpp:34` |
| **`SO_PEERCRED`** | Kiểm tra UID của tiến trình kết nối — chống process lạ xen vào | `runner.py:32` |
| **Monotonic seq / t_us** | Số thứ tự và thời gian phải luôn tăng — chống replay packet cũ che mất failsafe | `hal_sil.cpp:140` |
| **BYE** | Message kết thúc phiên êm đẹp | `sil_wire.hpp:22` |
| **Replay** | Gửi lại packet cũ. Bị chặn để không "đóng băng" thời gian và vô hiệu failsafe | Plan R10 |

## I. Công cụ & kiểm thử

| Thuật ngữ | Nghĩa | Trong repo |
|---|---|---|
| **ctypes** | Thư viện Python gọi hàm C/C++ trong file `.so` | `tests/_bind.py` |
| **`extern "C"`** | Ngăn C++ name-mangling để Python tìm thấy symbol theo tên C | `host_bindings.cpp` |
| **Unit test** | Test từng module nhỏ, chạy nhanh, không cần mô phỏng đầy đủ | `tests/test_mixer.py`, ... |
| **E2E / slow test** | Test tích hợp chạy cả phiên SIL vài giây — đánh dấu `@pytest.mark.slow` | `tests/test_hover_sil.py` |
| **Gate** | Bộ tiêu chí pass/fail tự động — "cổng" để coi một tầng là xong | `make gate` |
| **Log schema** | Danh sách 23 cột CSV cố định cho mọi tầng | `plant/log.py:11-15` |
| **Params hash** | Mã MD5 của bộ tham số, ghi vào log để biết run nào dùng cấu hình nào | `runner.py:109` |

---

## J. Bảng đơn vị hay dùng

| Đại lượng | Đơn vị repo | Ghi chú |
|---|---|---|
| Thời gian | `t_us` (microsecond), `dt` = 0.001 s | 1 kHz = 1000 Hz = 1 ms/tick |
| Góc | radian (firmware), độ (log/hiển thị) | `deg = rad × 180/π` |
| Vận tốc góc | rad/s | 300 dps ≈ 5.236 rad/s |
| Gia tốc | m/s² | 1 g = 9.81 m/s² |
| Từ trường | µT | không dùng |
| Duty | 0.0 … 1.0 | nhân 2047 cho LEDC 11-bit |
| PWM tần số | 20–32 kHz | ngoài dải nghe được, tránh tiếng rít |
| Pin | V, Ω, A | 1S LiPo: 4.20 V đầy → 3.30 V cạn |
| Kích thước | m, kg | arm 0.043 m, mass 0.03 kg |

---

*Quay lại `README.md` để xem lộ trình, hoặc sang `01-buc-tranh-tong-the.md` để bắt đầu.*
