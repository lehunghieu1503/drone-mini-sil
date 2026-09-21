"""T1.1/T1.2/T1.7 — wire ABI is frozen and identical on both sides."""

import ctypes
import struct

from _bind import bind
from plant import sil_proto as sp

F3 = ctypes.c_float * 3
F4 = ctypes.c_float * 4
B4 = ctypes.c_uint8 * 4
B32 = ctypes.c_uint8 * 32
U32x7 = ctypes.c_uint32 * 7


class CHdr(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("magic", ctypes.c_uint16), ("ver", ctypes.c_uint8), ("type", ctypes.c_uint8),
                ("len", ctypes.c_uint16), ("seq", ctypes.c_uint32), ("crc", ctypes.c_uint32)]


class CImu(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("gyro", F3), ("accel", F3), ("mag", F3), ("temp", ctypes.c_float),
                ("t_us", ctypes.c_uint64), ("valid", ctypes.c_bool)]


class CRc(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("roll", ctypes.c_float), ("pitch", ctypes.c_float), ("yaw", ctypes.c_float),
                ("throttle", ctypes.c_float), ("armed_switch", ctypes.c_uint8),
                ("frame_ok", ctypes.c_uint8), ("t_us", ctypes.c_uint64)]


class CPwm(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("mot", F4)]


class CState(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("imu", CImu), ("vbat", ctypes.c_float), ("rc_proto", ctypes.c_uint8),
                ("rc_len", ctypes.c_uint8), ("rc_raw", B32)]


class COut(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("mot", F4), ("roll_est", ctypes.c_float), ("pitch_est", ctypes.c_float),
                ("yaw_est", ctypes.c_float), ("armed", ctypes.c_uint8),
                ("failsafe", ctypes.c_uint8), ("led_mode", ctypes.c_uint8),
                ("imu_valid", ctypes.c_uint8), ("tick", ctypes.c_uint32),
                ("sat_shift", ctypes.c_float)]


class CTelem(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("t_us", ctypes.c_uint64), ("roll", ctypes.c_float), ("pitch", ctypes.c_float),
                ("yaw", ctypes.c_float), ("yaw_rate", ctypes.c_float), ("x", ctypes.c_float),
                ("y", ctypes.c_float), ("z", ctypes.c_float), ("vx", ctypes.c_float),
                ("vy", ctypes.c_float), ("vz", ctypes.c_float), ("roll_est", ctypes.c_float),
                ("pitch_est", ctypes.c_float), ("yaw_est", ctypes.c_float), ("mot", F4),
                ("vbat", ctypes.c_float), ("armed", ctypes.c_uint8),
                ("failsafe", ctypes.c_uint8), ("led_mode", ctypes.c_uint8),
                ("imu_valid", ctypes.c_uint8), ("sat_shift", ctypes.c_float)]


class CHello(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("dt_us", ctypes.c_uint32), ("sizes", U32x7)]


def test_t1_1_ctypes_sizes():
    assert ctypes.sizeof(CHdr) == 14
    assert ctypes.sizeof(CImu) == 49
    assert ctypes.sizeof(CRc) == 26
    assert ctypes.sizeof(CPwm) == 16
    assert ctypes.sizeof(CState) == 87
    assert ctypes.sizeof(COut) == 40
    assert ctypes.sizeof(CTelem) == 88
    assert ctypes.sizeof(CHello) == 32


def test_t1_2_struct_calcsize():
    assert sp.HDR_SIZE == 14
    assert sp.IMU_SIZE == 49
    assert sp.RC_SIZE == 26
    assert sp.PWM_SIZE == 16
    assert sp.STATE_SIZE == 87
    assert sp.FW_OUT_SIZE == 40
    assert sp.TELEM_SIZE == 88
    assert sp.HELLO_SIZE == 32
    assert struct.calcsize(sp.STATE_FMT) == 87


def test_t1_7_native_widths():
    assert ctypes.sizeof(ctypes.c_bool) == 1
    assert ctypes.sizeof(ctypes.c_float) == 4
    assert ctypes.sizeof(ctypes.c_uint64) == 8


def test_hello_sizes_match_cxx(lib):
    wire_hello_fill = bind(lib, "wire_hello_fill", ctypes.c_int,
                           [ctypes.POINTER(ctypes.c_uint32)])
    out = (ctypes.c_uint32 * 8)()
    n = wire_hello_fill(out)
    assert n == 8
    assert out[0] == 1000
    assert list(out[1:]) == sp.HELLO_SIZES
