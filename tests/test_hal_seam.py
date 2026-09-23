"""T1.3/T1.4/T1.5/T1.9/T1.15 — HAL seam, HELLO handshake, single tick owner."""

from __future__ import annotations

import os
import pathlib
import re
import socket
import subprocess
import threading
import time

from _bind import ROOT, sil_runner_path
from plant import sil_proto as sp

FLIGHT = ROOT / "firmware" / "main" / "flight"
HAL = ROOT / "firmware" / "main" / "hal"


def test_t1_3_no_idf_headers_in_flight():
    pattern = re.compile(r"\b(esp_[a-z0-9_]*\.h|driver/|freertos/|sdkconfig\.h)\b")
    offenders = []
    for path in list(FLIGHT.glob("*.hpp")) + list(FLIGHT.glob("*.cpp")):
        text = path.read_text()
        for i, line in enumerate(text.splitlines(), 1):
            if line.lstrip().startswith("//"):
                continue
            if pattern.search(line):
                offenders.append(f"{path.name}:{i}: {line.strip()}")
    assert offenders == [], offenders


def test_t1_9_pwmwrite_single_path():
    call = re.compile(r"(\.|->)pwmWrite\s*\(")
    offenders = []
    for path in list(FLIGHT.glob("*.hpp")) + list(FLIGHT.glob("*.cpp")):
        if path.name == "output.cpp":
            continue
        for i, line in enumerate(path.read_text().splitlines(), 1):
            if call.search(line):
                offenders.append(f"{path.name}:{i}")
    assert offenders == [], f"pwmWrite called outside output.cpp: {offenders}"


def test_t1_15_single_tick_owner():
    sim = (ROOT / "sim" / "main_sil.cpp").read_text()
    app = (ROOT / "firmware" / "main" / "app_main.cpp").read_text()
    assert "ControlLoop" in sim and "loop.tick" in sim
    assert "ControlLoop" in app and "loop.tick" in app
    # The tick order lives only in control_loop.cpp.
    assert "c.mixer.write" not in sim and "c.mixer.write" not in app


def _serve(name: str, handler):
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    srv.bind("\0" + name)
    srv.listen(1)
    errors = []

    def run():
        try:
            conn, _ = srv.accept()
            try:
                handler(conn)
            finally:
                conn.close()
        except Exception as exc:  # noqa: BLE001
            errors.append(exc)
        finally:
            srv.close()

    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t, errors


def _run_runner(name: str):
    return subprocess.run([str(sil_runner_path()), "--sock", name],
                          capture_output=True, timeout=30)


def _unique(tag: str) -> str:
    return f"dm-test-{tag}-{os.getpid()}-{time.time_ns()}"


def test_t1_4_hello_ok():
    name = _unique("hello")
    seen = {}

    def handler(conn):
        buf = conn.recv(4096)
        typ, seq, body = sp.unpack_msg(buf)
        seen["typ"] = typ
        conn.sendall(sp.pack_msg(sp.MsgType.ACK, 0, sp.pack_hello()))
        conn.sendall(sp.pack_msg(sp.MsgType.BYE, 0))

    t, errors = _serve(name, handler)
    r = _run_runner(name)
    t.join(timeout=5)
    assert not errors, errors
    assert seen.get("typ") == sp.MsgType.HELLO
    assert r.returncode == 0, r.stderr.decode()


def _run_runner_args(name: str, extra):
    return subprocess.run([str(sil_runner_path()), "--sock", name, *extra],
                          capture_output=True, timeout=30)


def test_t3_17_replay_frozen_time_rejected():
    name = _unique("replay")
    body = sp.pack_state(t_us=1000, gyro_rps=(0, 0, 0), accel_mps2=(0, 0, -9.81),
                         vbat=3.8, rc_raw=b"")

    def handler(conn):
        buf = conn.recv(4096)
        sp.unpack_msg(buf)
        conn.sendall(sp.pack_msg(sp.MsgType.ACK, 0, sp.pack_hello()))
        conn.sendall(sp.pack_msg(sp.MsgType.STATE, 0, body))
        conn.recv(4096)
        conn.sendall(sp.pack_msg(sp.MsgType.STATE, 1, body))  # frozen t_us

    t, errors = _serve(name, handler)
    r = _run_runner_args(name, [])
    t.join(timeout=5)
    assert not errors, errors
    assert r.returncode == 2, (r.returncode, r.stderr.decode())


def test_t3_20_transport_timeout():
    name = _unique("timeout")

    def handler(conn):
        buf = conn.recv(4096)
        sp.unpack_msg(buf)
        conn.sendall(sp.pack_msg(sp.MsgType.ACK, 0, sp.pack_hello()))
        time.sleep(3.0)  # stall past the 1 s transport timeout

    t, errors = _serve(name, handler)
    r = _run_runner_args(name, ["--transport-timeout", "1"])
    t.join(timeout=6)
    assert not errors, errors
    assert r.returncode == 3, (r.returncode, r.stderr.decode())


def test_t1_5_hello_size_mismatch():
    name = _unique("hellobad")

    def handler(conn):
        buf = conn.recv(4096)
        sp.unpack_msg(buf)
        bad = list(sp.HELLO_SIZES)
        bad[0] = 48
        conn.sendall(sp.pack_msg(sp.MsgType.ACK, 0, sp.pack_hello(sizes=bad)))

    t, errors = _serve(name, handler)
    r = _run_runner(name)
    t.join(timeout=5)
    assert not errors, errors
    assert r.returncode == 2, (r.returncode, r.stderr.decode())


def test_t1_16_fw_out_tick_increments():
    """The firmware, not the plant, owns SilOut.tick; it starts at 1 and grows."""
    name = _unique("tick")
    ticks = []

    def handler(conn):
        buf = conn.recv(4096)
        sp.unpack_msg(buf)
        conn.sendall(sp.pack_msg(sp.MsgType.ACK, 0, sp.pack_hello()))
        for i in range(5):
            state = sp.pack_state(t_us=(i + 1) * 1000, gyro_rps=(0, 0, 0),
                                  accel_mps2=(0, 0, -9.81), vbat=3.8, rc_raw=b"")
            conn.sendall(sp.pack_msg(sp.MsgType.STATE, i, state))
            typ, _seq, body = sp.unpack_msg(conn.recv(4096))
            if typ == sp.MsgType.FW_OUT:
                ticks.append(sp.unpack_out(body)["tick"])
        conn.sendall(sp.pack_msg(sp.MsgType.BYE, 0))

    t, errors = _serve(name, handler)
    r = _run_runner(name)
    t.join(timeout=5)
    assert not errors, errors
    assert r.returncode == 0, r.stderr.decode()
    assert ticks == [1, 2, 3, 4, 5]


def test_t3_21_transport_timeout_zero_rejected():
    """0 would mean 'no timeout' on Linux, so it must be rejected before connect."""
    name = _unique("tzero")
    seen = {"connected": False}

    def handler(conn):
        seen["connected"] = True

    t, errors = _serve(name, handler)
    r = _run_runner_args(name, ["--transport-timeout", "0"])
    t.join(timeout=1)
    assert r.returncode == 2, (r.returncode, r.stderr.decode())
    assert seen["connected"] is False


def test_t3_22_hello_timeout_exits_3():
    """A peer that never ACKs must time out (exit 3), not be a protocol error."""
    name = _unique("hellostall")

    def handler(conn):
        conn.recv(4096)
        time.sleep(3.0)

    t, errors = _serve(name, handler)
    r = _run_runner_args(name, ["--transport-timeout", "1"])
    t.join(timeout=6)
    assert not errors, errors
    assert r.returncode == 3, (r.returncode, r.stderr.decode())


def test_runner_rejects_transport_timeout_zero(tmp_path):
    from plant import runner

    args = runner.build_parser().parse_args(
        ["--transport-timeout", "0", "--t-end", "0.1", "--log", str(tmp_path / "x.csv")])
    assert runner.run(args) == 2
