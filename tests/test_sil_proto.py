"""T3.14 — wire framing roundtrip, CRC, version, HELLO."""

import pytest

from plant import sil_proto as sp


def _state_body():
    return sp.pack_state(t_us=1234, gyro_rps=(1.0, 2.0, 3.0), accel_mps2=(4.0, 5.0, 6.0),
                         vbat=3.8, rc_raw=b"\x0f" * 25)


def test_hello_roundtrip():
    msg = sp.pack_msg(sp.MsgType.HELLO, 7, sp.pack_hello())
    typ, seq, body = sp.unpack_msg(msg)
    assert typ == sp.MsgType.HELLO and seq == 7
    dt_us, sizes = sp.unpack_hello(body)
    assert dt_us == 1000
    assert sizes == sp.HELLO_SIZES


def test_state_and_out_roundtrip():
    typ, seq, body = sp.unpack_msg(sp.pack_msg(sp.MsgType.STATE, 11, _state_body()))
    st = sp.unpack_state(body)
    assert st["t_us"] == 1234
    assert st["gyro_rps"] == pytest.approx((1.0, 2.0, 3.0))
    assert st["rc_len"] == 25

    out_body = sp.pack_out(mot=[0.1, 0.2, 0.3, 0.4], armed=1, sat_shift=-0.2, tick=5)
    _, _, body = sp.unpack_msg(sp.pack_msg(sp.MsgType.FW_OUT, 12, out_body))
    out = sp.unpack_out(body)
    assert out["mot"] == pytest.approx([0.1, 0.2, 0.3, 0.4])
    assert out["armed"] == 1 and out["sat_shift"] == pytest.approx(-0.2)


def test_crc_corruption_raises():
    msg = bytearray(sp.pack_msg(sp.MsgType.STATE, 1, _state_body()))
    msg[-1] ^= 0xFF
    with pytest.raises(sp.ProtocolError):
        sp.unpack_msg(bytes(msg))


def test_version_mismatch_raises():
    msg = bytearray(sp.pack_msg(sp.MsgType.STATE, 1, _state_body()))
    msg[2] = 99  # version byte
    with pytest.raises(sp.ProtocolError):
        sp.unpack_msg(bytes(msg))


def test_short_and_bad_magic():
    with pytest.raises(sp.ProtocolError):
        sp.unpack_msg(b"\x00\x01")
    msg = bytearray(sp.pack_msg(sp.MsgType.BYE, 0))
    msg[0] ^= 0xFF
    with pytest.raises(sp.ProtocolError):
        sp.unpack_msg(bytes(msg))
