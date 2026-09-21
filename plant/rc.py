"""SBUS encode/decode + scenario RC sources.

SBUS: 25-byte frame, header 0x0F, 16 ch x 11 bits LSB-first, flags byte, footer.
Flags: bit2 = frame-lost (0x04), bit3 = failsafe (0x08). LSB-first numbering
(0x04/0x08), matching bolderflight/sbus and Betaflight."""

from __future__ import annotations

SBUS_MIN = 172
SBUS_MID = 992
SBUS_MAX = 1811
FLAG_FRAME_LOST = 0x04
FLAG_FAILSAFE = 0x08

# Channel map: 0 roll, 1 pitch, 2 throttle, 3 yaw, 4 arm switch.
ARM_CHANNEL = 4


def sbus_encode(channels, flags: int = 0) -> bytes:
    ch = list(channels) + [0] * (16 - len(channels))
    bits = 0
    for i in range(16):
        bits |= (int(ch[i]) & 0x7FF) << (11 * i)
    data = bytearray(25)
    data[0] = 0x0F
    for b in range(22):
        data[1 + b] = (bits >> (8 * b)) & 0xFF
    data[23] = flags & 0x0F
    data[24] = 0x00
    return bytes(data)


def sbus_decode(data):
    if len(data) != 25:
        raise ValueError("sbus frame must be 25 bytes")
    if data[0] != 0x0F:
        raise ValueError("bad sbus header")
    bits = int.from_bytes(data[1:23], "little")
    channels = [(bits >> (11 * i)) & 0x7FF for i in range(16)]
    return channels, data[23]


def stick_to_sbus(v: float) -> int:
    v = max(-1.0, min(1.0, float(v)))
    if v >= 0.0:
        return int(round(SBUS_MID + v * (SBUS_MAX - SBUS_MID)))
    return int(round(SBUS_MID + v * (SBUS_MID - SBUS_MIN)))


def throttle_to_sbus(t: float) -> int:
    t = max(0.0, min(1.0, float(t)))
    return int(round(SBUS_MIN + t * (SBUS_MAX - SBUS_MIN)))


def sbus_to_stick(v: int) -> float:
    if v >= SBUS_MID:
        return (v - SBUS_MID) / (SBUS_MAX - SBUS_MID)
    return (v - SBUS_MID) / (SBUS_MID - SBUS_MIN)


def sbus_to_throttle(v: int) -> float:
    return max(0.0, min(1.0, (v - SBUS_MIN) / (SBUS_MAX - SBUS_MIN)))


class RcScenario:
    """Deterministic stick scenario. Returns raw SBUS frames + normalized cmd."""

    def __init__(self, name: str, n_ticks: int):
        self.name = name
        self.n_ticks = n_ticks

    def _base_channels(self, roll=0.0, pitch=0.0, throttle=0.5, yaw=0.0, arm=True):
        ch = [0] * 16
        ch[0] = stick_to_sbus(roll)
        ch[1] = stick_to_sbus(pitch)
        ch[2] = throttle_to_sbus(throttle)
        ch[3] = stick_to_sbus(yaw)
        ch[ARM_CHANNEL] = SBUS_MAX if arm else SBUS_MIN
        return ch

    def cmd(self, tick: int):
        return self._cmd(tick)

    def raw(self, tick: int) -> bytes:
        frame = self._frame(tick)
        if frame is None or frame[0] is None:
            return b""  # no frame delivered this tick
        ch, flags = frame
        return sbus_encode(ch, flags)

    def _frame(self, tick: int):
        return self._base_channels(), 0

    def _cmd(self, tick: int):
        return 0.0, 0.0, 0.0, 0.5


class _Hover(RcScenario):
    def _cmd(self, tick):
        return 0.0, 0.0, 0.0, 0.5


class _Steps(RcScenario):
    def _cmd(self, tick):
        period = int(2.0 / 0.001)  # 2 s
        phase = (tick // period) % 3
        if phase == 1:
            return 0.3, 0.0, 0.0, 0.5
        if phase == 2:
            return 0.0, 0.3, 0.0, 0.5
        return 0.0, 0.0, 0.0, 0.5

    def _frame(self, tick):
        r, p, y, t = self._cmd(tick)
        return self._base_channels(r, p, t, y), 0


class _Cut(_Hover):
    """Throttle to zero after 3 s (PWM cut)."""

    def _cmd(self, tick):
        if tick * 0.001 >= 3.0:
            return 0.0, 0.0, 0.0, 0.0
        return 0.0, 0.0, 0.0, 0.5

    def _frame(self, tick):
        r, p, y, t = self._cmd(tick)
        return self._base_channels(r, p, t, y), 0


class _RxFailsafe(RcScenario):
    def _frame(self, tick):
        ch = self._base_channels()
        if tick * 0.001 >= 3.0:
            return ch, FLAG_FAILSAFE
        return ch, 0

    def _cmd(self, tick):
        return 0.0, 0.0, 0.0, 0.5


class _FrameLost(RcScenario):
    def _frame(self, tick):
        ch = self._base_channels()
        if tick * 0.001 >= 3.0:
            return ch, FLAG_FRAME_LOST
        return ch, 0

    def _cmd(self, tick):
        return 0.0, 0.0, 0.0, 0.5


class _RateStep(RcScenario):
    """Roll/pitch/yaw stick step at t=1 s (rate-mode setpoint)."""

    axis = 0
    amp = 0.25
    thr = 0.55

    def _frame(self, tick):
        # Pre-arm window: low throttle so the FSM can arm after gyro calibration.
        if tick * 0.001 < 0.4:
            return self._base_channels(0.0, 0.0, 0.02, 0.0), 0
        v = self.amp if tick * 0.001 >= 1.0 else 0.0
        sticks = [0.0, 0.0, 0.0]
        sticks[self.axis] = v
        return self._base_channels(sticks[0], sticks[1], self.thr, sticks[2]), 0

    def _cmd(self, tick):
        if tick * 0.001 < 0.4:
            return 0.0, 0.0, 0.0, 0.02
        v = self.amp if tick * 0.001 >= 1.0 else 0.0
        return (v, 0.0, 0.0, self.thr) if self.axis == 0 else (
            (0.0, v, 0.0, self.thr) if self.axis == 1 else (0.0, 0.0, v, self.thr))


class _RateStepRoll(_RateStep):
    axis = 0


class _RateStepPitch(_RateStep):
    axis = 1


class _RateStepYaw(_RateStep):
    axis = 2
    amp = 0.5


class _RateHover(_RateStep):
    axis = 0
    amp = 0.0


class _AttHover(_RateStep):
    """Attitude-mode hover: pre-arm low throttle, then hover throttle, sticks 0."""

    axis = 0
    amp = 0.0
    thr = 0.52


class _StickStep(_RateStep):
    """Attitude mode: roll stick 0 -> 0.05 (2.0–2.5 s) -> 0."""

    axis = 0
    thr = 0.52

    def _frame(self, tick):
        if tick * 0.001 < 0.4:
            return self._base_channels(0.0, 0.0, 0.02, 0.0), 0
        t = tick * 0.001
        roll = 0.05 if 2.0 <= t < 2.5 else 0.0
        return self._base_channels(roll, 0.0, self.thr, 0.0), 0

    def _cmd(self, tick):
        if tick * 0.001 < 0.4:
            return 0.0, 0.0, 0.0, 0.02
        t = tick * 0.001
        return (0.05 if 2.0 <= t < 2.5 else 0.0), 0.0, 0.0, self.thr


class _HoverSweep(_RateStep):
    """Throttle sweep {0.8,0.9,1.0,1.1,1.2} x nominal, 1.5 s per step."""

    axis = 0
    amp = 0.0

    def _frame(self, tick):
        if tick * 0.001 < 0.4:
            return self._base_channels(0.0, 0.0, 0.02, 0.0), 0
        factors = [0.8, 0.9, 1.0, 1.1, 1.2]
        idx = int((tick * 0.001 - 0.4) / 1.5) % len(factors)
        thr = min(1.0, factors[idx] * 0.5)
        return self._base_channels(0.0, 0.0, thr, 0.0), 0

    def _cmd(self, tick):
        if tick * 0.001 < 0.4:
            return 0.0, 0.0, 0.0, 0.02
        factors = [0.8, 0.9, 1.0, 1.1, 1.2]
        idx = int((tick * 0.001 - 0.4) / 1.5) % len(factors)
        return 0.0, 0.0, 0.0, min(1.0, factors[idx] * 0.5)


class _Saturation(_RateStep):
    axis = 0
    amp = 1.0
    thr = 0.30  # low collective so a large differential saturates the mixer


class _RcLoss(_RateStep):
    """Frames stop arriving after 3 s (parser sees nothing -> timeout)."""

    axis = 0
    amp = 0.0
    thr = 0.52

    def _frame(self, tick):
        if tick * 0.001 < 0.4:
            return self._base_channels(0.0, 0.0, 0.02, 0.0), 0
        if tick * 0.001 >= 3.0:
            return None, 0  # no frame this tick
        return self._base_channels(0.0, 0.0, self.thr, 0.0), 0

    def _cmd(self, tick):
        thr = self.thr if tick * 0.001 >= 0.4 else 0.02
        return 0.0, 0.0, 0.0, thr


def make_scenario(name: str, n_ticks: int) -> RcScenario:
    table = {
        "hover": _Hover,
        "steps": _Steps,
        "cut": _Cut,
        "rc_loss": _RcLoss,
        "rx_failsafe": _RxFailsafe,
        "frame_lost": _FrameLost,
        "rate_step_roll": _RateStepRoll,
        "rate_step_pitch": _RateStepPitch,
        "rate_step_yaw": _RateStepYaw,
        "rate_hover": _RateHover,
        "att_hover": _AttHover,
        "stick_step": _StickStep,
        "hover_sweep": _HoverSweep,
        "saturation": _Saturation,
    }
    if name not in table:
        raise ValueError(f"unknown scenario: {name}")
    return table[name](name, n_ticks)
