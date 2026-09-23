"""External attitude pilot. Firmware stays the controller; this only writes RC sticks.

The craft has no altitude loop. Throttle is collective, so a fixed stick climbs
forever. This pilot closes altitude and horizontal position *outside* the
firmware, the way a pilot (or a ground station) would move the sticks.

Frames: plant world is ENU, body is FLU. Measured on this plant, not from the
FRD comment alone: positive pitch stick accelerates world +x, positive roll
stick accelerates world -y. The hold law uses that.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import drone_mini_params as P
from .vehicle import quat_to_R

# Arm gate in the firmware is throttle <= 0.05 and centred sticks.
ARM_THROTTLE = 0.02

# Full stick is 30 deg. Keep the outer loop well inside that so the attitude
# loop is not asked to flip while it is still catching vertical speed.
MAX_TILT_STICK = 0.35
MAX_VZ = 0.7
ALT_SLEW = 0.4

KP_Z = 1.1
KP_VZ = 0.09
KI_VZ = 0.06
KP_XY = 0.55
KP_V = 0.40


@dataclass
class Manual:
    """Stick intent for one tick. Zero means "the pilot holds"."""

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    alt_rate: float = 0.0
    toggle_arm: bool = False
    land: bool = False
    quit: bool = False


@dataclass
class RcCmd:
    roll: float
    pitch: float
    yaw: float
    throttle: float
    arm: bool


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class HoverPilot:
    """Climb to `alt_sp`, then hold that height and the horizontal point of arming.

    While a manual roll/pitch stick is active the position hold lets go. The
    next centred tick recaptures the spot underneath the craft.
    """

    def __init__(self, alt_sp: float, hover_duty: float | None = None):
        if not np.isfinite(alt_sp) or alt_sp < 0.0:
            raise ValueError(f"alt_sp must be finite and >= 0, got {alt_sp}")
        self.alt_sp = float(alt_sp)
        self.hover_duty = float(P.hover_duty_nominal() if hover_duty is None else hover_duty)
        self.wants_arm = True
        self._vz_int = 0.0
        self._holding_xy = False
        self._x_sp = 0.0
        self._y_sp = 0.0
        self._was_armed = False
        self._landing = False

    def command(self, pos, vel, quat, vbat: float, armed: bool, manual: Manual, dt: float) -> RcCmd:
        if manual.toggle_arm:
            self.wants_arm = not self.wants_arm
            self._landing = False
        if manual.land:
            self._landing = True
            self.alt_sp = 0.0

        # Switch must rise while throttle is still idle, or the firmware refuses to arm.
        if not self.wants_arm or not armed:
            self._vz_int = 0.0
            self._holding_xy = False
            self._was_armed = False
            if self._landing and not self.wants_arm:
                self._landing = False
            return RcCmd(0.0, 0.0, 0.0, ARM_THROTTLE if self.wants_arm else 0.0, self.wants_arm)

        if not self._was_armed:
            # Capture the pad position at the arm edge so the hold does not
            # chase a setpoint from before the motors were alive.
            self._x_sp = float(pos[0])
            self._y_sp = float(pos[1])
            self._holding_xy = True
            self._was_armed = True

        if manual.alt_rate:
            self._landing = False
            self.alt_sp = max(0.0, self.alt_sp + manual.alt_rate * dt)

        # Touched the ground on a landing request: drop the switch.
        if self._landing and float(pos[2]) < 0.06 and abs(float(vel[2])) < 0.15:
            self.wants_arm = False
            self._landing = False
            self._vz_int = 0.0
            self._was_armed = False
            return RcCmd(0.0, 0.0, 0.0, 0.0, False)

        roll, pitch = self._tilt(pos, vel, quat, manual)
        throttle = self._collective(float(pos[2]), float(vel[2]), vbat, dt)
        return RcCmd(roll, pitch, _clip(manual.yaw, 1.0), throttle, True)

    def _tilt(self, pos, vel, quat, manual: Manual):
        manual_tilt = abs(manual.roll) > 1e-6 or abs(manual.pitch) > 1e-6
        if manual_tilt:
            self._holding_xy = False
            return _clip(manual.roll, 1.0), _clip(manual.pitch, 1.0)

        if not self._holding_xy:
            self._x_sp = float(pos[0])
            self._y_sp = float(pos[1])
            self._holding_xy = True

        # Body FLU. Positive pitch accelerates +x, so braking +x needs negative pitch.
        # Positive roll accelerates -y, so braking leftward +y needs positive roll.
        rotation = quat_to_R(quat)
        err_b = rotation.T @ np.array([self._x_sp - pos[0], self._y_sp - pos[1], 0.0])
        vel_b = rotation.T @ np.asarray(vel, dtype=float)
        v_sp_x = _clip(KP_XY * float(err_b[0]), 0.6)
        v_sp_y = _clip(KP_XY * float(err_b[1]), 0.6)
        pitch = _clip(KP_V * (v_sp_x - float(vel_b[0])), MAX_TILT_STICK)
        roll = _clip(KP_V * (float(vel_b[1]) - v_sp_y), MAX_TILT_STICK)
        return roll, pitch

    def _collective(self, z: float, vz: float, vbat: float, dt: float) -> float:
        # omega_max scales with pack voltage, so the same duty is less thrust when the cell sags.
        v = vbat if np.isfinite(vbat) and vbat > 0.5 else P.BATTERY["vbat_nominal"]
        hover = self.hover_duty * (P.BATTERY["vbat_nominal"] / v)

        vz_sp = _clip(KP_Z * (self.alt_sp - z), MAX_VZ)
        err = vz_sp - vz
        self._vz_int = _clip(self._vz_int + err * dt, 1.5)
        return min(0.82, max(0.08, hover + KP_VZ * err + KI_VZ * self._vz_int))
