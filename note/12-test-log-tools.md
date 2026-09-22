# 12 — Kiểm thử, Log và Công cụ

> Đọc xong bạn hiểu cách test firmware C++ bằng Python, cách log được thiết kế để so
> sánh giữa các tầng, và cách `make gate` tự động quyết định pass/fail. Đây là file về
> **bằng chứng** — thứ biến "có vẻ chạy được" thành "đã được chứng minh".

---

## 1. Vì sao test firmware C++ bằng Python?

### 1.1 Vấn đề

Test C++ thuần (GoogleTest, Catch2...) rất tốt cho logic, nhưng repo cần thêm:

- Chạy **cả phiên SIL** (plant Python + firmware C++) và kiểm kết quả.
- Phân tích log bằng numpy (đỉnh, RMS, độ trôi).
- So sánh log giữa các tầng.

Viết tất cả bằng C++ sẽ trùng lặp hạ tầng (build system, plotting). Giải pháp của
repo: **build flight core thành shared library, gọi từ Python qua ctypes**.

### 1.2 Cơ chế

```
firmware/main/flight/*.cpp  ──┐
firmware/main/hal/*.cpp       ├─► libdrone_flight.so ──► ctypes ──► pytest
firmware/host/host_bindings.cpp┘        (g++)
```

`host_bindings.cpp` là lớp `extern "C"` — không có C++ name-mangling, không có struct
C++ lộ ra ngoài. Ví dụ:

```cpp
extern "C" {

void mock_reset(void) { g_mock.resetLog(); }
void mock_set_imu(const float* gyro3, const float* accel3, uint64_t t_us, int valid) { ... }
float mixer_write(float thr, float roll, float pitch, float yaw, float* out4) { ... }
int output_stage_apply(const float* in4, int armed, int failsafe, int imu_valid) { ... }

}  // extern "C"
```

Quy tắc: **chỉ tham số nguyên thủy/mảng đi qua ranh giới**. Không truyền struct C++
trực tiếp — vì Python phải tự dựng lại layout, dễ sai. Mảng `float*` + độ dài là cách
an toàn nhất.

### 1.3 `tests/_bind.py`

```python
def build_host(force: bool = False) -> pathlib.Path:
    return _cmake_build(ROOT / "firmware", ROOT / "build" / "host",
                        ["-DDRONE_HOST_LIB=ON"], force=force)

def load_lib() -> ctypes.CDLL:
    so = build_host() / "libdrone_flight.so"
    if not so.exists():
        build_host(force=True)
    return ctypes.CDLL(str(so))

def bind(lib, name, restype, argtypes=None):
    fn = getattr(lib, name)
    fn.restype = restype
    fn.argtypes = argtypes or []
    return fn
```

`conftest.py` tạo fixture `lib` dùng chung cho cả session (build một lần). Trong test:

```python
def test_...(lib):
    mixer = bind(lib, "mixer_write", ctypes.c_float,
                 [ctypes.c_float]*4 + [ctypes.POINTER(ctypes.c_float)])
```

Ba dòng là đủ để gọi hàm C++ từ Python. Đây là kỹ năng cực thực dụng — áp dụng được
cho bất kỳ dự án embedded nào.

---

## 2. Phân loại test

### 2.1 Unit test (nhanh)

Test từng module, dùng `MockHal` hoặc gọi trực tiếp. Ví dụ `test_output_gate.py`:

```python
@pytest.mark.parametrize("armed,failsafe,imu_valid",
                         list(itertools.product([0, 1], repeat=3)))
def test_t1_8_gate_matrix(api, armed, failsafe, imu_valid):
    api["reset"]()
    inp = (ctypes.c_float * 4)(0.5, 0.5, 0.5, 0.5)
    wrote = api["apply"](inp, armed, failsafe, imu_valid)
    gate_open = bool(armed) and not failsafe and bool(imu_valid)
    if not gate_open:
        assert wrote == 0
        assert api["pwm_offs"]() == 1
        assert api["pwm_writes"]() == 0
        assert _last(api) == [0.0, 0.0, 0.0, 0.0]
    else:
        assert wrote == 1
        assert api["pwm_writes"]() == 1
        assert _last(api) == pytest.approx([0.5, 0.5, 0.5, 0.5])
```

Ba kỹ thuật đáng học:

1. **`itertools.product`** — test **toàn bộ 8 tổ hợp** cờ, không chỉ vài trường hợp
   "tiêu biểu". Bất biến an toàn phải đúng cho mọi tổ hợp.
2. **Kiểm cả side effect**: không chỉ `wrote == 0` mà còn `pwm_offs() == 1` và
   `pwm_writes() == 0`. Nghĩa là gate phải **chủ động cắt** motor, không chỉ "không
   ghi".
3. **Test NaN và out-of-range riêng** (`test_nan_fails_closed`): dữ liệu xấu cũng phải
   fail-closed.

### 2.2 SIL e2e test (slow)

Chạy cả phiên mô phỏng vài giây, phân tích log. Ví dụ `test_hover_sil.py`:

```python
def _hover_metrics(log):
    header, data = load(log)
    i = {n: k for k, n in enumerate(header)}
    start = int(2.0 / 0.001)                      # bỏ 2 s đầu (chưa arm)
    roll = np.degrees(data[:, i["roll_est"]])
    pitch = np.degrees(data[:, i["pitch_est"]])
    yaw = np.degrees(data[:, i["yaw_est"]])
    return {
        "roll_max": np.abs(roll[start:]).max(),
        "pitch_max": np.abs(pitch[start:]).max(),
        "yaw_drift": abs(yaw[-1] - yaw[start]),
        "armed": data[:, i["armed"]].sum(),
        "finite": np.all(np.isfinite(data)),
    }

@pytest.mark.slow
@pytest.mark.parametrize("seed", [1, 2])
def test_t6_7_t6_8_hover_two_seeds(tmp_path, seed):
    code, log = _run(tmp_path, "att_hover", f"hover_{seed}.csv", seed=seed)
    assert code == 0
    m = _hover_metrics(log)
    assert m["roll_max"] < 5.0
    assert m["pitch_max"] < 5.0
    assert m["yaw_drift"] < 15.0
```

Điểm đáng học:

- **Bỏ 2 s đầu** trước khi đo: giai đoạn boot/calibrate/arm không phải là hover.
- **Chạy 2 seed**: chống "may mắn" với một chuỗi bias cụ thể.
- **Tiêu chí số cụ thể**: < 5° roll/pitch, < 15° yaw drift. Không phải "nhìn có vẻ
  thẳng".

### 2.3 Phân tách bằng marker

`pyproject.toml`:

```toml
markers = [
    "slow: end-to-end simulations (kept in `make gate`; excluded from `make test`)",
]
```

| Lệnh | Chạy gì | Thời gian |
|---|---|---|
| `make test` | `pytest -m "not slow"` — unit only | vài giây |
| `make gate` | `pytest` — tất cả | vài phút |

Vòng phát triển hàng ngày dùng `make test`; trước khi commit/merge dùng `make gate`.

---

## 3. Log — hợp đồng giữa các tầng

### 3.1 Schema 23 cột

`plant/log.py:11`:

```python
CSV_HEADER = [
    "t_us", "roll_cmd", "pitch_cmd", "yaw_cmd", "thr_cmd", "armed", "failsafe",
    "gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z",
    "roll_est", "pitch_est", "yaw_est",
    "mot0", "mot1", "mot2", "mot3",
    "vbat", "led_mode", "sat_shift",
]
assert len(CSV_HEADER) == 23
```

Nguyên tắc **append-only**: thêm cột mới thì thêm vào **cuối** và tăng `schema`
(hiện v2). Không bao giờ đổi thứ tự cột — vì log cũ phải đọc được mãi mãi. Đây là kỷ
luật dữ liệu quan trọng.

### 3.2 Metadata preamble

```python
class CsvWriter:
    def __init__(self, path, meta=None, buffering=1 << 16):
        self.f = open(path, "w", buffering=buffering, newline="\n")
        self.f.write(f"# schema={SCHEMA_VERSION}\n")
        if meta:
            write_meta(self.f, **meta)
        self.f.write(",".join(CSV_HEADER) + "\n")
```

File log bắt đầu bằng các dòng `# key=value`: kind (SIL/HIL/FLIGHT), seed, dt_us,
plant, params_hash, versions, aborted... Nhờ vậy log **tự mô tả**. Khi bạn mở một log
cũ, bạn biết nó từ đâu, chạy điều kiện gì.

### 3.3 Cờ `aborted`

```python
def close(self, aborted: bool = False):
    self.f.write(f"# aborted={1 if aborted else 0}\n")
```

Nếu phiên chạy kết thúc bất thường (timeout, lỗi protocol), log được đánh dấu
`aborted=1`. Công cụ phân tích có thể từ chối so sánh log aborted — tránh kết luận sai
từ dữ liệu dở dang.

### 3.4 Bộ công cụ phân tích (`plant/log.py`)

| Hàm | Công dụng |
|---|---|
| `load_log(path)` | đọc log thành dict cột numpy, tách metadata |
| `align_time(t_us, armed, mode)` | căn thời gian theo `start`/`arm`/`t0` |
| `jitter_stats(t_us)` | min/median/p99/max của `dt`, đếm `dt > 1.5×median` |
| `residual(a, b)` | max_abs, rms, corr giữa hai tín hiệu |

**`align_time` mode `arm`**: hai log có thời điểm arm khác nhau (boot nhanh chậm khác
nhau). Muốn so sánh đáp ứng, phải căn theo lúc arm chứ không theo lúc bắt đầu ghi. Chi
tiết nhỏ nhưng quyết định tính đúng đắn của so sánh.

**`jitter_stats`**: đo chất lượng timing. Trên SIL, dt gần như tuyệt đối 1000 µs. Trên
HIL/chip, jitter sẽ xuất hiện — đây là chỉ số chính của tầng B.

---

## 4. Công cụ

### 4.1 `tools/plot_sil.py` — nhìn để hiểu

Vẽ 5 đồ thị chồng thời gian:

1. Gyro 3 trục (deg/s) — nhiễu, dao động.
2. Attitude ước lượng roll/pitch/yaw (deg) — hành vi bay.
3. 4 kênh duty — hoạt động motor.
4. V_BAT — pin.
5. LED/armed/failsafe (step plot) — trạng thái.

```bash
PYTHONPATH=. .venv/bin/python tools/plot_sil.py logs/demo.csv --out logs/demo.png
```

**Kỹ năng quan trọng:** luôn nhìn log trước khi đọc số. Mắt người phát hiện dao động,
trôi, bão hòa nhanh hơn mọi phép thống kê.

### 4.2 `tools/compare_logs.py` — overlay tự động

```bash
PYTHONPATH=. .venv/bin/python tools/compare_logs.py log_sil.csv log_hil.csv --align arm --tol 0.15
```

Cách hoạt động:

1. Load 2 log, kiểm **cùng header** (schema mismatch → exit 2).
2. Căn thời gian theo `arm`.
3. So 6 tín hiệu: `roll_est, pitch_est, yaw_est, gyro_x, gyro_y, gyro_z`.
4. Tính residual (max_abs, rms, corr) và **gate tự động**: vượt `tol` → exit 1.

Đây là hiện thực của tiêu chí §9.5: *"Log SIL và HIL cùng schema, overlay được, có gate
pass/fail tự động"*. Không pass bằng "nhìn có vẻ giống".

### 4.3 Bảng tiêu chí pass

`docs/measurements.md` ghi trạng thái từng tiêu chí §9 kèm **bằng chứng**:

| Tiêu chí | Trạng thái | Bằng chứng |
|---|---|---|
| §9.2 mixer sign | software-verified | `tests/test_mixer.py` T2.1–T2.11 |
| §9.5 log schema | pass | `tools/compare_logs.py`, T8.x |
| §9.1/§9.4 hover | pass (dưới giả định A3/A5) | `tests/test_hover_sil.py` T6.7–T6.12 |
| §9.3 failsafe | pass | `tests/test_power_sil.py` T7.15 |
| Bất biến an toàn | pass mọi tổ hợp cờ | `tests/test_output_gate.py` T1.8 |

Chú ý cách viết trung thực: hover "pass **dưới giả định A3/A5**" — không tuyên bố quá
mức. Đây là văn hóa kỹ thuật mà bạn nên học theo.

---

## 5. Quy trình làm việc thực tế

```
Sửa code
   │
   ├─► make test          (unit, vài giây) ──► fail? sửa ngay
   │
   ├─► make gate          (unit + SIL, vài phút)
   │
   ├─► chạy 1 phiên SIL có plot, nhìn bằng mắt
   │
   └─► nếu đổi wire/params: kiểm test_abi, test_param_parity
```

Với thay đổi lớn (ví dụ đổi gain PID):

```bash
# 1. Chạy SIL với scenario step, lưu log
PYTHONPATH=. .venv/bin/python -m plant.runner --mode rate --scenario rate_step_roll \
    --t-end 3 --log logs/before.csv

# 2. Sửa gain, chạy lại
PYTHONPATH=. .venv/bin/python -m plant.runner --mode rate --scenario rate_step_roll \
    --t-end 3 --log logs/after.csv

# 3. Overlay
PYTHONPATH=. .venv/bin/python tools/compare_logs.py logs/before.csv logs/after.csv
```

Đây chính là **A/B testing cho thuật toán điều khiển** — có seed cố định nên mọi khác
biệt là do code, không do ngẫu nhiên.

---

## 6. Checkpoint

1. Vì sao host bindings dùng `extern "C"`? Vì sao chỉ truyền mảng `float*` thay vì
   struct?
2. Sự khác biệt giữa `make test` và `make gate`? Khi nào dùng cái nào?
3. Vì sao test output gate dùng `itertools.product` thay vì test 2–3 trường hợp?
4. Log schema "append-only" nghĩa là gì? Vì sao không được đổi thứ tự cột?
5. `align_time` mode `arm` dùng khi nào? Nếu không căn, so sánh sẽ sai thế nào?
6. `compare_logs.py` trả exit code nào khi residual vượt tolerance?

<details>
<summary>Gợi ý đáp án</summary>

1. `extern "C"` để tắt name-mangling, Python tìm được symbol theo tên C. Mảng
   `float*` vì Python/ctypes không cần dựng lại layout struct — tránh sai ABI.
2. `make test` chỉ unit (nhanh, vòng lặp dev); `make gate` chạy cả SIL e2e (chậm,
   trước commit/merge).
3. Bất biến an toàn phải đúng cho **mọi** tổ hợp; test 2–3 trường hợp có thể bỏ sót
   tổ hợp nguy hiểm (ví dụ failsafe=1 nhưng armed=1).
4. Chỉ thêm cột mới vào cuối, tăng schema version. Đổi thứ tự sẽ làm log cũ đọc sai
   (cột này thành cột kia) mà không có lỗi rõ ràng.
5. Khi so sánh đáp ứng điều khiển giữa hai phiên: thời điểm arm khác nhau do
   boot/calibrate. Không căn thì hai đáp ứng lệch pha, residual vô nghĩa.
6. Exit 1 (gate fail).

</details>

---

*Tiếp theo: `13-lo-trinh-hoc-va-bai-tap.md` — lộ trình chi tiết và bài tập làm chủ.*
