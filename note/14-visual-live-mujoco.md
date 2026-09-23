# 14 — Live 3D view với MuJoCo (out-of-process)

> Đọc xong bạn biết cách xem drone bay 3D trong lúc chạy SIL, hiểu vì sao viewer là
> **process riêng**, và cách xử lý sự cố. Liên quan: `11-sil-lockstep-wire-protocol.md`
> (kiến trúc lockstep), `12-test-log-tools.md` (test/log).

---

## 1. Vấn đề: làm sao "thấy" drone bay?

Tool hiện có chỉ cho **plot 2D** từ log CSV (`tools/plot_sil.py`): gyro, attitude, duty,
pin. Muốn thấy **3D** (drone nghiêng, bay lên, xoay) thì cần render — và render cần
OpenGL.

Cách ngây thơ: nhúng viewer MuJoCo thẳng vào `plant/runner.py`. **Đã thử và loại** vì:

- Viewer dùng thread riêng của MuJoCo + `atexit`; khi process thoát, teardown GL crash
  native (**exit 1/124/139**) — không catch được ở Python.
- Không có display (SSH/CI) ⇒ process chết ngay, không có message.
- `make gate` (chạy **mọi** test) sẽ mở cửa sổ GUI và có thể đỏ vì crash lúc thoát.
- `viewer.sync()` **lock nội bộ** ⇒ nhúng `sync()` trong `viewer.lock()` sẽ deadlock.

## 2. Giải pháp: tách process + UDP

```
plant/runner.py (SIL lockstep — không GL, không import mujoco)
   │  UDP 127.0.0.1:45999, 36 B/datagram, cap 200 Hz
   │  (lỗi/không có viewer ⇒ backoff ~1 s rồi thử lại)
   ▼
tools/mujoco_view.py (process riêng — GL)
   drain latest-wins → qpos → viewer.sync(state_only=True)
```

**Vì sao hoạt động tốt:**

| Vấn đề | Cách giải |
|---|---|
| GL crash giết process | Viewer là process riêng → crash chỉ giết viewer |
| Không display | Viewer exit 5 + message; SIL run vẫn exit 0 |
| `make gate` mở GUI | Test không bao giờ mở GL; runner side thuần stdlib |
| Lockstep bị chặn | Runner chỉ `send()` UDP (µs), không `sync()` |
| Sync rate không kiểm soát | Runner cap `--visual-hz 200`; viewer render nhịp riêng |
| Viewer mở muộn | Sender backoff ~1 s rồi **tự thử lại** (không cần restart run) |
| Camera race | Viewer ghi `viewer.cam` trong `with viewer.lock():` |

**Frame quy ước:** plant world = ENU z-up, quat `wxyz`; MuJoCo world cũng z-up và
`qpos[3:7]` cũng `wxyz` ⇒ copy trực tiếp, chỉ thêm offset z **theo plant** (cosmetic):

| Plant | `pos.z` | Visual z |
|---|---|---|
| `mujoco` | COM đã ở `0.0125` do contact | `+0` |
| `rk4` | clamp tâm ở `0` | `+0.0125` |
| `rotorpy` | không có ground | `max(z,0) + 0.0125` |

## 3. Dùng

```sh
make venv-visual   # một lần: mujoco + glfw + PyOpenGL
make view          # terminal 1 — viewer (mở trước)
make visual        # terminal 2 — SIL run, publish pose, realtime 1x
```

Hoặc thủ công:

```sh
PYTHONPATH=. .venv/bin/python tools/mujoco_view.py --port 45999 --fps 60
PYTHONPATH=. .venv/bin/python -m plant.runner \
    --mode attitude --scenario att_hover --t-end 20 \
    --visual --visual-rate 1.0 --log logs/demo.csv
```

| Cờ (runner) | Mặc định | Ý nghĩa |
|---|---|---|
| `--visual` | off | bật publish pose (opt-in) |
| `--visual-port` | `45999` | port viewer lắng nghe |
| `--visual-hz` | `200` | cap số datagram/giây |
| `--visual-rate` | `0` | `0` = không throttle; `1.0` = realtime; `<1` = slow-mo |

| Cờ (viewer) | Mặc định | Ý nghĩa |
|---|---|---|
| `--port` | `45999` | phải khớp `--visual-port` |
| `--fps` | `60` | nhịp render |
| `--follow` / `--no-follow` | follow | camera bám drone (`--no-follow` để xem quỹ đạo) |

## 4. Xử lý sự cố

| Triệu chứng | Nguyên nhân | Cách xử lý |
|---|---|---|
| Không thấy gì trong viewer | Sai port | Cho `--port` / `--visual-port` khớp nhau |
| Viewer mở sau khi run chạy | Bình thường | Sender tự retry ~1 s; không cần restart |
| "cannot open a window" + exit 5 | Không có display | Chạy trên máy có GUI; macOS dùng `mjpython` |
| "cannot bind ... address already in use" | Viewer khác đang chạy | Đổi `--port`, hoặc tắt viewer cũ |
| Run chậm hơn bình thường | `--visual-rate 1.0` throttle về realtime | Dùng `--visual-rate 0` khi không cần xem |
| `kill -INT` không tắt viewer (chạy nền) | Shell non-interactive đặt `SIGINT=SIG_IGN` cho background job | Ctrl-C ở terminal thật, hoặc đóng cửa sổ; không phải bug của viewer |
| Drone lơ lửng trên sàn | Offset z theo plant | Xem bảng ở mục 2 — `mujoco` không cộng offset |

## 5. Vì sao log vẫn byte-identical?

`PoseSender.publish()` chỉ **đọc** `plant.pos`/`plant.quat` và gửi datagram; không ghi
log, không đụng plant. Có hẳn test always-on (không cần display) so **byte-identical**
phần CSV giữa run có và không có `--visual` (`tests/test_visual_integration.py`).

## 6. Checkpoint

1. Vì sao viewer phải là process riêng? (gợi ý: native crash + gate + lockstep)
2. Vì sao không gọi `viewer.sync()` bên trong `viewer.lock()`?
3. Vì sao `mujoco` không cộng offset z còn `rk4` thì cộng?
4. Viewer mở muộn thì chuyện gì xảy ra? (gợi ý: backoff + retry)
5. Nếu viewer chết giữa run, kết quả SIL có đổi không? Vì sao?
