"""SIL lockstep runner (plant = server, firmware = client).

AF_UNIX SOCK_SEQPACKET, abstract namespace with a random name, SO_PEERCRED UID
check, HELLO size handshake, seq/t_us monotonicity, CRC via sil_proto.

Exit codes: 0 clean, 2 protocol, 3 timeout, 4 plant numerical (the wire
contract), 5 host-side config/dependency error (missing optional plant dep).

Optional `--visual` publishes the plant pose as UDP datagrams for the external
viewer process (`tools/mujoco_view.py`). The sink is a pure consumer: it never
writes plant/log state, and a missing, late, or dead viewer cannot affect the
run.
"""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import secrets
import socket
import struct
import subprocess
import sys
import time

import numpy as np

from . import drone_mini_params as P
from . import sil_proto as proto
from .battery import Battery
from .icm20948 import Icm20948
from .log import CsvWriter
from .rc import make_scenario, sbus_to_stick, sbus_to_throttle, ARM_CHANNEL, sbus_decode
from .vehicle import make_plant
from .visual_link import DEFAULT_HZ, DEFAULT_PORT, PoseSender

ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_SIL_BIN = ROOT / "build" / "sim" / "sil_runner"

EXIT_CONFIG = 5  # host-side config/dependency error (additive; wire contract is 0/2/3/4)



def peer_uid(conn: socket.socket) -> int:
    raw = conn.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, struct.calcsize("3i"))
    _pid, uid, _gid = struct.unpack("3i", raw)
    return uid


def _recv_msg(conn, timeout_s):
    conn.settimeout(timeout_s)
    try:
        buf = conn.recv(4096)
    except socket.timeout:
        return None, "timeout"
    if not buf:
        return None, "closed"
    try:
        typ, seq, body = proto.unpack_msg(buf)
    except proto.ProtocolError as exc:
        return None, f"proto:{exc}"
    return (typ, seq, body), None


def _pace(t0_wall: float, t_virtual_s: float, rate: float,
          clock=time.perf_counter, sleep=time.sleep) -> None:
    """Sleep so wall-clock tracks virtual time at `rate`. rate <= 0 disables."""
    if not math.isfinite(rate) or rate <= 0.0:
        return
    deadline = t0_wall + t_virtual_s / rate
    while True:
        remaining = deadline - clock()
        if remaining <= 0.0:
            return  # late (or done): never accumulate debt
        sleep(min(remaining, 0.05))


def run(args, metrics: dict | None = None) -> int:
    n_ticks = max(1, int(round(args.t_end / P.DT)))
    try:
        plant = make_plant(args.plant, clean=args.clean_imu, seed=args.seed)
    except ImportError as exc:
        sys.stderr.write(
            f"plant: plant '{args.plant}' needs an optional dependency ({exc}).\n"
            "        Run `make venv-visual` for MuJoCo.\n"
        )
        return EXIT_CONFIG
    icm = Icm20948(args.seed, clean=args.clean_imu)
    batt = Battery(P.BATTERY)
    scen = make_scenario(args.scenario, n_ticks)

    name = f"dm-sil-{secrets.token_hex(8)}"
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    srv.bind("\0" + name)
    srv.listen(1)

    sil_bin = args.sil_bin or str(DEFAULT_SIL_BIN)
    cmd = [sil_bin, "--sock", name, "--mode", args.mode,
           "--transport-timeout", str(args.transport_timeout)]
    if args.arm_test:
        cmd.append("--arm-test")
    if args.ol_from_rc:
        cmd.append("--ol-from-rc")
    if args.ol_thr is not None:
        cmd += ["--thr", str(args.ol_thr)]
    if args.reset_reason:
        cmd += ["--reset-reason", str(args.reset_reason)]

    writer = None
    sender = None
    aborted = True
    proc = None
    conn = None
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        srv.settimeout(args.transport_timeout)
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            sys.stderr.write("plant: accept timeout\n")
            return proto.EXIT_TIMEOUT
        if peer_uid(conn) != os.getuid():
            sys.stderr.write("plant: SO_PEERCRED uid mismatch\n")
            return proto.EXIT_PROTO

        # HELLO handshake (plant is authoritative on sizes).
        msg, err = _recv_msg(conn, args.transport_timeout)
        if err or msg[0] != proto.MsgType.HELLO:
            sys.stderr.write(f"plant: bad HELLO ({err})\n")
            return proto.EXIT_PROTO
        dt_us, sizes = proto.unpack_hello(msg[2])
        if dt_us != P.DT_US or sizes != proto.HELLO_SIZES:
            sys.stderr.write("plant: HELLO size/version mismatch\n")
            conn.sendall(proto.pack_msg(proto.MsgType.ACK, 0,
                                        proto.pack_hello(sizes=proto.HELLO_SIZES)))
            return proto.EXIT_PROTO
        conn.sendall(proto.pack_msg(proto.MsgType.ACK, 0, proto.pack_hello()))

        import hashlib
        import numpy as _np

        params_hash = hashlib.md5(
            repr((P.MASS_KG, P.K_T, P.K_Q, P.TAU_M_S, P.OMEGA_MAX_RAD_S,
                  P.INERTIA)).encode()
        ).hexdigest()[:8]
        det = "rk4" if args.plant == "rk4" else f"{args.plant}-version-pinned"
        writer = CsvWriter(args.log, meta={
            "kind": "SIL", "schema": 2, "seed": args.seed, "dt_us": P.DT_US,
            "plant": args.plant, "determinism": det,
            "scenario": args.scenario, "mode": args.mode,
            "versions": f"numpy={_np.__version__}",
            "params_hash": params_hash,
            "reset_reason": args.reset_reason, "decimated": 0, "vbat_nan": 0,
        })
        if args.visual:
            sender = PoseSender(args.visual_port, hz=args.visual_hz)

        duty = np.zeros(4)
        vbat = P.BATTERY["vbat_nominal"]
        last_out_seq = -1
        t0_wall = time.perf_counter()
        for tick in range(n_ticks):
            t_us = tick * P.DT_US
            gyro_true, accel_true = plant.sense()
            gyro, accel = icm.sample(gyro_true, accel_true)
            duty_sum = float(np.sum(duty))
            vbat = batt.step(P.DT, duty_sum)

            raw = scen.raw(tick)
            ch = sbus_decode(raw)[0] if raw else [992] * 16
            valid = not (args.imu_invalid_after > 0.0
                         and t_us >= args.imu_invalid_after * 1e6)
            state = proto.pack_state(
                t_us=t_us, gyro_rps=gyro, accel_mps2=accel, valid=valid,
                vbat=vbat, rc_raw=raw,
            )
            conn.sendall(proto.pack_msg(proto.MsgType.STATE, tick, state))

            msg, err = _recv_msg(conn, args.transport_timeout)
            if err and err == "timeout":
                sys.stderr.write(f"plant: fw timeout at tick {tick}\n")
                return proto.EXIT_TIMEOUT
            if err and err.startswith("proto:"):
                sys.stderr.write(f"plant: {err} at tick {tick}\n")
                return proto.EXIT_PROTO
            if err and err == "closed":
                sys.stderr.write(f"plant: fw closed early at tick {tick}\n")
                if tick >= n_ticks - 1:
                    break
                return proto.EXIT_PROTO
            typ, seq, body = msg
            if typ == proto.MsgType.BYE:
                aborted = False
                break
            if typ != proto.MsgType.FW_OUT:
                sys.stderr.write(f"plant: unexpected msg {typ} at tick {tick}\n")
                return proto.EXIT_PROTO
            if seq <= last_out_seq:
                sys.stderr.write(f"plant: non-monotonic out seq {seq}\n")
                return proto.EXIT_PROTO
            last_out_seq = seq
            out = proto.unpack_out(body)
            duty = np.array(out["mot"], dtype=float)

            writer.write_row(
                t_us=t_us,
                roll_cmd=sbus_to_stick(ch[0]), pitch_cmd=sbus_to_stick(ch[1]),
                yaw_cmd=sbus_to_stick(ch[3]), thr_cmd=sbus_to_throttle(ch[2]),
                armed=out["armed"], failsafe=out["failsafe"],
                gyro_x=gyro[0], gyro_y=gyro[1], gyro_z=gyro[2],
                acc_x=accel[0], acc_y=accel[1], acc_z=accel[2],
                roll_est=out["roll_est"], pitch_est=out["pitch_est"], yaw_est=out["yaw_est"],
                mot0=out["mot"][0], mot1=out["mot"][1], mot2=out["mot"][2], mot3=out["mot"][3],
                vbat=vbat, led_mode=out["led_mode"], sat_shift=out["sat_shift"],
            )

            plant.step(P.DT, duty, vbat)
            if not plant.finite():
                sys.stderr.write("plant: NaN/Inf state\n")
                return proto.EXIT_NUMERICAL

            if sender is not None:
                sender.publish(t_us + P.DT_US, args.plant, plant.pos, plant.quat)
            _pace(t0_wall, (tick + 1) * P.DT, args.visual_rate)

        aborted = False
        conn.sendall(proto.pack_msg(proto.MsgType.BYE, 0))
        if metrics is not None:
            metrics.update({
                "alt": float(plant.pos[2]),
                "vz": float(plant.vel[2]),
                "motors": [float(m) for m in plant.motors],
                "ticks": n_ticks,
            })
        return proto.EXIT_CLEAN
    except FloatingPointError:
        sys.stderr.write("plant: numerical failure\n")
        return proto.EXIT_NUMERICAL
    except proto.ProtocolError as exc:
        sys.stderr.write(f"plant: protocol error: {exc}\n")
        return proto.EXIT_PROTO
    finally:
        if sender is not None:
            try:
                sender.close()
            except Exception:  # noqa: BLE001 - never mask writer/proc cleanup
                pass
        if writer is not None:
            writer.close(aborted=aborted)
        if conn is not None:
            try:
                conn.close()
            except OSError:
                pass
        srv.close()
        if proc is not None:
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()


def _rate_type(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value < 0.0:
        raise argparse.ArgumentTypeError("rate must be a finite number >= 0")
    return value


def _hz_type(text: str) -> float:
    value = float(text)
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("hz must be a finite number > 0")
    return value


def _port_type(text: str) -> int:
    value = int(text)
    if not (1 <= value <= 65535):
        raise argparse.ArgumentTypeError("port must be in 1..65535")
    return value


def build_parser():
    ap = argparse.ArgumentParser(prog="plant.runner")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--plant", default="rk4", choices=["rk4", "rotorpy", "mujoco"])
    ap.add_argument("--mode", default="open-loop", choices=["open-loop", "rate", "attitude"])
    ap.add_argument("--scenario", default="hover")
    ap.add_argument("--t-end", type=float, default=10.0)
    ap.add_argument("--log", default=str(ROOT / "logs" / "sil.csv"))
    ap.add_argument("--sil-bin", default=str(DEFAULT_SIL_BIN))
    ap.add_argument("--clean-imu", action="store_true")
    ap.add_argument("--arm-test", action="store_true")
    ap.add_argument("--ol-from-rc", action="store_true")
    ap.add_argument("--ol-thr", type=float, default=None)
    ap.add_argument("--reset-reason", type=int, default=0)
    ap.add_argument("--imu-invalid-after", type=float, default=0.0)
    ap.add_argument("--transport-timeout", type=int, default=P.TRANSPORT_TIMEOUT_S)
    ap.add_argument("--visual", action="store_true",
                    help="publish plant pose over UDP for tools/mujoco_view.py")
    ap.add_argument("--visual-port", type=_port_type, default=DEFAULT_PORT,
                    help="UDP port the viewer listens on (default 45999)")
    ap.add_argument("--visual-hz", type=_hz_type, default=DEFAULT_HZ,
                    help="cap on pose datagrams per second (default 200)")
    ap.add_argument("--visual-rate", type=_rate_type, default=0.0,
                    help="0 = no throttle (default), 1.0 = realtime, <1 = slow-mo")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    pathlib.Path(args.log).parent.mkdir(parents=True, exist_ok=True)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
