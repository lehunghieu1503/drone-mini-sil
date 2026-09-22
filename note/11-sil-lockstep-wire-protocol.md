# 11 — SIL Lockstep và Wire Protocol

> Đọc xong bạn hiểu cỗ máy SIL hoạt động từng bước, định dạng nhị phân trao đổi giữa
> C++ và Python, cách chống lỗi/lệch/hiểm, và vì sao SIL phải **tất định**. Đây là file
> "hạ tầng" — ít bay bổng nhưng là thứ khiến mọi test đáng tin.

---

## 1. Lockstep — nguyên lý

### 1.1 Vấn đề của "chạy song song"

Nếu plant và firmware chạy tự do với tốc độ riêng:

- Firmware có thể chạy 3 vòng trong khi plant bước 1 bước → nhận cùng dữ liệu cảm biến
  cho nhiều tick → điều khiển sai.
- Hoặc plant chạy 10 bước trong khi firmware chạy 1 → firmware bỏ lỡ trạng thái.

Không cách nào tái lập kết quả. Debug là ác mộng.

### 1.2 Giải pháp: khóa bước

**Plant là master clock.** Mỗi tick:

```
PLANT (Python)                                FIRMWARE (C++)
──────────────                                ───────────────
1. plant.sense() đọc trạng thái
2. icm.sample() thêm nhiễu
3. batt.step() cập nhật pin
4. đóng gói STATE ─────────────────────────► 5. recvState()
                                              6. loop.tick(): imu → rc → failsafe
                                                 → estimator → attitude → rate
                                                 → mixer → output gate → led
                                              7. đóng gói FW_OUT
8. recv FW_OUT ◄──────────────────────────────┘
9. duty → motor bậc 1 → plant.step()
10. ghi log
11. lặp lại tick tiếp theo
```

Không bên nào chạy trước. Mỗi tick là một "giao dịch" hoàn chỉnh. Nhờ vậy:

- **Tái lập được**: cùng seed → cùng chuỗi tick → cùng log.
- **Debug được**: tick N luôn có thể xem lại chính xác.
- **Không cần đồng bộ thời gian thật**: `t_us` là **thời gian ảo** (tick × 1000).

---

## 2. Wire protocol — định dạng nhị phân

### 2.1 Header 14 byte

Mọi message bắt đầu bằng `Hdr` (`sil_wire.hpp:34`):

```cpp
struct Hdr {
  uint16_t magic;    // 0x534D = "MS" little-endian -> phát hiện stream rác
  uint8_t  ver;      // version protocol (hiện = 1)
  uint8_t  type;     // HELLO/ACK/STATE/FW_OUT/BYE/TELEM
  uint16_t len;      // độ dài payload
  uint32_t seq;      // số thứ tự, phải tăng đơn điệu
  uint32_t crc;      // CRC32 của (header với crc=0) + payload
};
```

| Trường | Vai trò | Nếu sai thì sao |
|---|---|---|
| `magic` | nhận diện frame | `unpackMsg` trả −1 |
| `ver` | tương thích protocol | trả −2 |
| `len` | độ dài payload | trả −3 (short body) |
| `seq` | phát hiện mất/thứ tự | kiểm monotonic ở tầng trên |
| `crc` | phát hiện hỏng bit | trả −4 |

### 2.2 CRC32 — phát hiện lỗi truyền

CRC (Cyclic Redundancy Check) là phép chia đa thức trên bit. Repo dùng CRC32 chuẩn
zlib (`sil_wire.cpp:23-40`) để Python (`zlib.crc32`) và C++ cho cùng kết quả:

```cpp
constexpr uint32_t kPoly = 0xEDB88320u;   // polynomial chuẩn CRC-32

uint32_t crc32(const void* data, int len) {
  uint32_t c = crc32Init();               // 0xFFFFFFFF
  c = crc32Update(c, data, len);
  return crc32Final(c);                   // XOR 0xFFFFFFFF
}
```

Cách tính CRC trong `packMsg` (`sil_wire.cpp:42`):

```cpp
h.crc = 0;
std::memcpy(buf, &h, sizeof(Hdr));
// ... copy payload
const uint32_t crc = crc32(buf, total);   // tính với crc = 0
std::memcpy(buf + offsetof(Hdr, crc), &crc, sizeof(crc));   // ghi lại
```

Và khi giải (`unpackMsg`): copy header, zero trường crc, tính lại, so với `hdr.crc`.
Nếu khác → frame hỏng. Chi tiết "zero trường crc trước khi tính" là quy ước hai bên
phải giống nhau — Python làm y hệt (`sil_proto.py:89`).

> CRC **không phải** bảo mật — nó chỉ phát hiện lỗi ngẫu nhiên. Repo nói rõ: "Trusted
> dev host only; no network port is opened."

### 2.3 Các payload — bảng kích thước

`sil_wire.hpp:43-97`:

| Struct | Size | Chứa gì | Chiều |
|---|---|---|---|
| `Hdr` | 14 | header | cả hai |
| `SilImu` | 49 | gyro, accel, mag, temp, t_us, valid | plant → fw |
| `SilRc` | 26 | 4 kênh + flags + t_us | (dự phòng) |
| `SilPwm` | 16 | 4 duty | fw → plant |
| `StateRc` | 87 | `SilImu` + vbat + raw SBUS 32 byte | plant → fw |
| `SilOut` | 40 | 4 duty + attitude est + flags + tick + sat_shift | fw → plant |
| `SilTelem` | 88 | state thật + est + duty + pin (cho log/HIL) | fw → plant |
| `SilHello` | 32 | dt_us + 7 sizes | handshake |

Chú ý `StateRc` chứa **raw SBUS** (`rc_raw[32]`) chứ không phải kênh đã giải mã — đúng
quyết định D5: HAL đưa byte thô, parser của firmware giải mã. Nhờ vậy SIL kiểm luôn cả
parser.

### 2.4 HELLO handshake — chống lệch ABI

Trước khi trao đổi dữ liệu, hai bên so "căn cước":

```cpp
void fillHello(SilHello& h) {
  h.dt_us = kSilDtUs;                          // 1000 us
  for (int i = 0; i < 7; ++i) h.sizes[i] = static_cast<uint32_t>(kHelloSizes[i]);
}
```

Client gửi HELLO, server so 7 kích thước + dt. Khác → từ chối chạy (exit 2).

**Vì sao cần?** Vì nếu một bên đổi struct (ví dụ thêm trường vào `SilOut`) mà bên kia
không đổi, dữ liệu sẽ bị đọc lệch byte — kiểu lỗi tệ nhất: không crash, chỉ sai âm
thầm. HELLO biến lỗi âm thầm thành lỗi ồn ào ngay từ đầu.

**Quy tắc bump version** (README repo): *"Any layout change must bump `kProtoVersion`
and is rejected by the HELLO size check (SIL and HIL alike)."*

### 2.5 Các message type

`sil_wire.hpp:16`:

| Type | Chiều | Khi nào |
|---|---|---|
| `kHello` | fw → plant | đầu phiên |
| `kAck` | plant → fw | trả lời HELLO |
| `kState` | plant → fw | mỗi tick |
| `kFwOut` | fw → plant | mỗi tick |
| `kBye` | fw → plant | kết thúc êm |
| `kTelem` | fw → plant | telemetry (HIL) |

### 2.6 Exit code — hợp đồng kết thúc

| Code | Nghĩa |
|---|---|
| 0 | sạch |
| 2 | vi phạm protocol (CRC, seq, size) |
| 3 | timeout |
| 4 | plant phân kỳ (NaN/Inf) |

Cả hai bên dùng cùng bảng. Test có thể khẳng định "chạy xong exit 0" — một tín hiệu
pass/fail rõ ràng, không cần parse log.

---

## 3. Transport — UDS socket

### 3.1 Vì sao Unix Domain Socket?

| Lựa chọn | Ưu | Nhược |
|---|---|---|
| TCP localhost | Quen thuộc | Qua network stack, chậm hơn, mở port |
| **UDS SEQPACKET** | Nhanh, giữ ranh giới message, không cần port | Chỉ local |
| Pipe/stdin | Đơn giản | Khó hai chiều, khó nhiều message |
| Shared memory | Nhanh nhất | Phức tạp, cần đồng bộ |

Repo chọn **AF_UNIX SOCK_SEQPACKET** (plan D3): nhanh, giữ ranh giới message (không
cần tự tách frame từ stream), và **không mở network port** — an toàn hơn.

### 3.2 Abstract namespace — không để lại file rác

```cpp
addr.sun_family = AF_UNIX;
addr.sun_path[0] = '\0';            // abstract namespace
std::memcpy(addr.sun_path + 1, name, namelen);
```

Abstract namespace (bắt đầu bằng byte NUL) nghĩa là socket **không tạo file** trên đĩa
— tự biến mất khi process kết thúc. Tên ngẫu nhiên (`runner.py:60`):

```python
name = f"dm-sil-{secrets.token_hex(8)}"
```

Tên ngẫu nhiên tránh xung đột khi chạy nhiều test song song, và tránh process lạ đoán
được tên để kết nối.

### 3.3 Bảo mật tối thiểu

`runner.py:89`:

```python
if peer_uid(conn) != os.getuid():
    sys.stderr.write("plant: SO_PEERCRED uid mismatch\n")
    return proto.EXIT_PROTO
```

`SO_PEERCRED` cho biết UID của process đầu kia. Chỉ chấp nhận cùng user. Đủ cho môi
trường dev; repo ghi rõ đây không phải cơ chế bảo mật mạnh.

### 3.4 Timeout — không treo vô hạn

```cpp
timeval tv{};
tv.tv_sec = timeout_s_;     // mặc định 30 s
::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
::setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
```

Timeout dài (30 s) vì SIL chạy nhanh hơn thời gian thật rất nhiều; chỉ dùng để thoát
khi bên kia chết hẳn. Timeout thật (100 ms) của failsafe là **thời gian ảo** —
`now_us` từ plant, không phải wall-clock.

---

## 4. Chống replay và đứng thời gian

### 4.1 `SilHal::recvState` (`hal_sil.cpp:126`)

```cpp
// Reject replay / frozen time so a stalled plant cannot hide a failsafe.
if (have_state_ && (h.seq <= last_state_seq_ || st.imu.t_us <= last_state_us_)) {
  return IoResult::kProto;
}
```

Hai điều kiện:

1. `h.seq` phải **tăng nghiêm ngặt** — packet trùng/ cũ bị từ chối.
2. `st.imu.t_us` phải **tăng nghiêm ngặt** — thời gian không được đứng yên.

**Kịch bản tấn công/lỗi:** nếu plant gửi lại packet cũ (hoặc clock đứng), firmware sẽ
nghĩ RC vẫn "mới" → failsafe không bao giờ kích → drone tiếp tục quay motor dù mất
điều khiển. Red team R10 gọi đây là rủi ro MED. Cách chặn: từ chối ngay ở tầng
transport (exit 2), không để lọt xuống logic.

### 4.2 Phía plant cũng kiểm

`runner.py:161`:

```python
if seq <= last_out_seq:
    sys.stderr.write(f"plant: non-monotonic out seq {seq}\n")
    return proto.EXIT_PROTO
```

Hai bên kiểm lẫn nhau — không tin bên nào.

### 4.3 Timestamp do HAL làm chủ

`sil_wire.hpp`/`hal_sil.cpp:174`:

```cpp
out.t_us = nowUs();  // re-stamp from the transport clock (RT#8)
```

`SilHal::imuRead` **không** dùng `t_us` từ payload mà lấy `state_.imu.t_us` (đã được
kiểm monotonic). Đây là "một nguồn thời gian duy nhất" (RT#8) — tránh trộn clock giữa
các trường.

---

## 5. Đọc `runner.py` — từng giai đoạn

`plant/runner.py:53`:

```python
def run(args, metrics: dict | None = None) -> int:
    n_ticks = max(1, int(round(args.t_end / P.DT)))          # 10 s -> 10000 tick
    plant = make_plant(args.plant, clean=args.clean_imu, seed=args.seed)
    icm = Icm20948(args.seed, clean=args.clean_imu)
    batt = Battery(P.BATTERY)
    scen = make_scenario(args.scenario, n_ticks)              # kịch bản RC

    name = f"dm-sil-{secrets.token_hex(8)}"
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    srv.bind("\0" + name)
    srv.listen(1)

    proc = subprocess.Popen(cmd, ...)                         # chạy sil_runner
    conn, _ = srv.accept()
    # ... SO_PEERCRED check
    # ... HELLO handshake
    writer = CsvWriter(args.log, meta={...})                  # mở log + metadata
```

Rồi vòng lặp chính (`runner.py:125`):

```python
    for tick in range(n_ticks):
        t_us = tick * P.DT_US
        gyro_true, accel_true = plant.sense()          # vật lý thật
        gyro, accel = icm.sample(gyro_true, accel_true) # thêm nhiễu
        vbat = batt.step(P.DT, float(np.sum(duty)))     # pin theo dòng
        raw = scen.raw(tick)                            # frame SBUS kịch bản
        state = proto.pack_state(t_us=t_us, gyro_rps=gyro, accel_mps2=accel,
                                 valid=valid, vbat=vbat, rc_raw=raw)
        conn.sendall(proto.pack_msg(proto.MsgType.STATE, tick, state))

        msg, err = _recv_msg(conn, args.transport_timeout)
        # ... kiểm tra lỗi, kiểm type/seq
        out = proto.unpack_out(body)
        duty = np.array(out["mot"], dtype=float)

        writer.write_row(...)                           # ghi log 23 cột
        plant.step(P.DT, duty, vbat)                    # bước vật lý
        if not plant.finite():
            return proto.EXIT_NUMERICAL
```

Thứ tự đáng chú ý:

- **`batt.step` trước khi gửi STATE**: firmware nhận điện áp của chính tick này.
- **`plant.step` sau khi nhận FW_OUT**: lệnh của firmware áp dụng cho bước tiếp theo
  (trễ 1 tick — đúng thực tế).
- **Log ghi giữa hai việc**: log phản ánh đúng trạng thái mà firmware vừa thấy.

### 5.1 Metadata log

`runner.py:109`:

```python
params_hash = hashlib.md5(repr((P.MASS_KG, P.K_T, P.K_Q, P.TAU_M_S,
                                P.OMEGA_MAX_RAD_S, P.INERTIA)).encode()).hexdigest()[:8]
writer = CsvWriter(args.log, meta={
    "kind": "SIL", "schema": 2, "seed": args.seed, "dt_us": P.DT_US,
    "plant": args.plant, "determinism": "rk4", "scenario": args.scenario,
    "mode": args.mode, "versions": f"numpy={_np.__version__}",
    "params_hash": params_hash, ...})
```

Log luôn tự mô tả: chạy với seed nào, tham số gì (hash), phiên bản thư viện nào. Khi
so sánh hai log, bạn biết chắc chúng cùng điều kiện hay không.

---

## 6. Determinism — vì sao và cách đảm bảo

### 6.1 Định nghĩa

> Cùng seed + cùng tham số + cùng phiên bản → **cùng log, bit-for-bit**.

Repo chỉ cam kết điều này cho **RK4 path** (plan D20). RotorPy dùng `solve_ivp` thích
nghi + RNG global nên chỉ tất định trong cùng phiên bản scipy/rotorpy.

### 6.2 Các yếu tố phá tất định và cách chặn

| Yếu tố | Cách chặn |
|---|---|
| RNG toàn cục | Mỗi model có `np.random.default_rng(seed)` riêng |
| Thời gian wall-clock | `t_us` là thời gian ảo (tick × 1000) |
| Thứ tự dict/set | Không dùng trong đường tính toán |
| Số luồng | Runner single-thread |
| Phiên bản thư viện | Ghi vào metadata; RK4 không phụ thuộc scipy |

### 6.3 Vì sao quan trọng?

- **Debug hồi quy**: test fail ở seed 7 → chạy lại seed 7, xem đúng tick sai.
- **So sánh A/B**: đổi gain PID, giữ nguyên seed → khác biệt log là do gain.
- **Test tự động**: assert trên giá trị cụ thể ổn định.

---

## 7. Băng thông — bài toán của HIL

Mỗi tick SIL trao đổi: `(14+87) + (14+40) = 155 byte`. Ở 1 kHz → **155 kB/s**.

Trên HIL dùng UART 2 Mbaud (≈200 kB/s thực dụng), 155 kB/s là ~78% tải — quá cao
(plan R11). Giải pháp HIL-1 (plan D18):

- Inject cảm biến **500 Hz** thay vì 1 kHz (firmware vẫn chạy 1 kHz, giữa các lần
  inject dùng dữ liệu cũ).
- TELEM **200 Hz**.
- Tổng ~71 kB/s = ~35% tải — còn dư cho jitter và burst.

Đây là bài học thiết kế: **băng thông là tài nguyên**, phải tính trước khi viết code.

---

## 8. Checkpoint

1. Lockstep là gì? Vì sao plant làm master clock?
2. Header wire protocol gồm những trường nào? CRC bảo vệ điều gì?
3. HELLO handshake so sánh gì? Vì sao cần nó?
4. Vì sao phải kiểm `seq` và `t_us` tăng đơn điệu? Kịch bản lỗi nếu không kiểm?
5. Tại sao log có `params_hash` và `seed`?
6. Tính băng thông một tick SIL. Vì sao HIL không chạy 1 kHz hai chiều?

<details>
<summary>Gợi ý đáp án</summary>

1. Plant và firmware bước từng tick khóa nhau, không bên nào chạy trước. Plant là
   master vì nó sở hữu thời gian ảo và mô hình vật lý.
2. magic, ver, type, len, seq, crc. CRC phát hiện lỗi truyền/hỏng bit.
3. So `dt_us` + 7 kích thước struct. Chống lệch ABI giữa C++ và Python — lỗi âm thầm
   nguy hiểm nhất.
4. Chống replay/đứng thời gian. Nếu không kiểm, packet cũ làm failsafe không kích,
   motor vẫn quay dù mất điều khiển.
5. Để biết hai log có cùng điều kiện so sánh được không; phục vụ tái lập và A/B test.
6. `(14+87)+(14+40) = 155 B/tick` → 155 kB/s @1 kHz. HIL UART 2 Mbaud ≈ 200 kB/s thực
   dụng → 78% tải, quá cao; giảm xuống 500 Hz + TELEM 200 Hz.

</details>

---

*Tiếp theo: `12-test-log-tools.md` — đo lường và bằng chứng.*
