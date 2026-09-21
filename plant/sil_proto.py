"""SIL/HIL wire protocol v1 (mirror of firmware/main/hal/sil_wire.hpp).

Format strings and sizes must stay identical to the C++ side; the asserts below
are the Python half of the ABI test.
"""

from __future__ import annotations

import struct
import zlib

MAGIC = 0x534D
VERSION = 1
DT_US = 1000
TRANSPORT_TIMEOUT_S = 30


class MsgType:
    HELLO = 1
    ACK = 2
    STATE = 3
    FW_OUT = 4
    BYE = 5
    TELEM = 6


HDR_FMT = "<HBBHII"
IMU_FMT = "<3f3f3ffQ?"
RC_FMT = "<4fBBQ"
PWM_FMT = "<4f"
STATE_FMT = "<3f3f3ffQ?fBB32s"
FW_OUT_FMT = "<4f3f4BIf"
TELEM_FMT = "<Q4f3f3f3f4ff4Bf"
HELLO_FMT = "<8I"

HDR_SIZE = struct.calcsize(HDR_FMT)
IMU_SIZE = struct.calcsize(IMU_FMT)
RC_SIZE = struct.calcsize(RC_FMT)
PWM_SIZE = struct.calcsize(PWM_FMT)
STATE_SIZE = struct.calcsize(STATE_FMT)
FW_OUT_SIZE = struct.calcsize(FW_OUT_FMT)
TELEM_SIZE = struct.calcsize(TELEM_FMT)
HELLO_SIZE = struct.calcsize(HELLO_FMT)

# Order must match wire::kHelloSizes in sil_wire.hpp.
HELLO_SIZES = [IMU_SIZE, RC_SIZE, PWM_SIZE, STATE_SIZE, FW_OUT_SIZE, TELEM_SIZE, HDR_SIZE]

assert HDR_SIZE == 14, HDR_SIZE
assert IMU_SIZE == 49, IMU_SIZE
assert RC_SIZE == 26, RC_SIZE
assert PWM_SIZE == 16, PWM_SIZE
assert STATE_SIZE == 87, STATE_SIZE
assert FW_OUT_SIZE == 40, FW_OUT_SIZE
assert TELEM_SIZE == 88, TELEM_SIZE
assert HELLO_SIZE == 32, HELLO_SIZE

EXIT_CLEAN = 0
EXIT_PROTO = 2
EXIT_TIMEOUT = 3
EXIT_NUMERICAL = 4


class ProtocolError(Exception):
    """Raised on any framing / CRC / ordering violation (maps to exit 2)."""


def crc32(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF


def pack_msg(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    hdr = struct.pack(HDR_FMT, MAGIC, VERSION, msg_type, len(payload), seq, 0)
    crc = crc32(hdr + payload)
    hdr = struct.pack(HDR_FMT, MAGIC, VERSION, msg_type, len(payload), seq, crc)
    return hdr + payload


def unpack_msg(buf: bytes):
    if len(buf) < HDR_SIZE:
        raise ProtocolError("short header")
    magic, ver, typ, length, seq, crc = struct.unpack(HDR_FMT, buf[:HDR_SIZE])
    if magic != MAGIC:
        raise ProtocolError("bad magic")
    if ver != VERSION:
        raise ProtocolError(f"version mismatch: {ver} != {VERSION}")
    total = HDR_SIZE + length
    if total > len(buf):
        raise ProtocolError("short body")
    hdr_zero = struct.pack(HDR_FMT, magic, ver, typ, length, seq, 0)
    if crc32(hdr_zero + buf[HDR_SIZE:total]) != crc:
        raise ProtocolError("bad crc")
    return typ, seq, buf[HDR_SIZE:total]


def pack_hello(dt_us: int = DT_US, sizes=None) -> bytes:
    return struct.pack(HELLO_FMT, dt_us, *(sizes if sizes is not None else HELLO_SIZES))


def unpack_hello(body: bytes):
    dt_us, *sizes = struct.unpack(HELLO_FMT, body)
    return dt_us, sizes


def pack_state(
    *,
    t_us: int,
    gyro_rps,
    accel_mps2,
    mag_uT=(0.0, 0.0, 0.0),
    temp_c: float = 25.0,
    valid: bool = True,
    vbat: float,
    rc_proto: int = 0,
    rc_raw: bytes = b"",
) -> bytes:
    rc_padded = bytes(rc_raw[:32]).ljust(32, b"\x00")
    return struct.pack(
        STATE_FMT,
        *gyro_rps,
        *accel_mps2,
        *mag_uT,
        temp_c,
        t_us,
        1 if valid else 0,
        vbat,
        rc_proto,
        len(rc_raw),
        rc_padded,
    )


def unpack_state(body: bytes):
    fields = struct.unpack(STATE_FMT, body)
    return {
        "gyro_rps": fields[0:3],
        "accel_mps2": fields[3:6],
        "mag_uT": fields[6:9],
        "temp_c": fields[9],
        "t_us": fields[10],
        "valid": bool(fields[11]),
        "vbat": fields[12],
        "rc_proto": fields[13],
        "rc_len": fields[14],
        "rc_raw": fields[15],
    }


def pack_out(
    *,
    mot,
    roll_est=0.0,
    pitch_est=0.0,
    yaw_est=0.0,
    armed=0,
    failsafe=0,
    led_mode=0,
    imu_valid=1,
    tick=0,
    sat_shift=0.0,
) -> bytes:
    return struct.pack(
        FW_OUT_FMT, *mot, roll_est, pitch_est, yaw_est,
        armed, failsafe, led_mode, imu_valid, tick, sat_shift,
    )


def unpack_out(body: bytes):
    f = struct.unpack(FW_OUT_FMT, body)
    return {
        "mot": f[0:4],
        "roll_est": f[4],
        "pitch_est": f[5],
        "yaw_est": f[6],
        "armed": f[7],
        "failsafe": f[8],
        "led_mode": f[9],
        "imu_valid": f[10],
        "tick": f[11],
        "sat_shift": f[12],
    }
