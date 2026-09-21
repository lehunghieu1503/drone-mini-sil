"""6-DOF RK4 quad plant.

Simulation frames: world ENU (z up), body FLU (x fwd, y left, z up). The
firmware speaks body FRD, so `to_frd(v) = (vx, -vy, -vz)` is applied at the
sense boundary only. Sign contract (T3.10) is asserted in tests.

State x = [pos(3), vel(3), quat_wxyz(4), omega(3), Omega(4)] (17,)."""

from __future__ import annotations

import numpy as np

from . import drone_mini_params as P


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_to_R(q):
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - w * z), s * (x * z + w * y)],
        [s * (x * y + w * z), 1 - s * (x * x + z * z), s * (y * z - w * x)],
        [s * (x * z - w * y), s * (y * z + w * x), 1 - s * (x * x + y * y)],
    ])


def to_frd(v):
    return np.array([v[0], -v[1], -v[2]])


class Rk4Plant:
    def __init__(self, clean: bool = False, seed: int = 0):
        self.clean = clean
        self.rng = np.random.default_rng(seed)
        self.I = np.diag(np.array(P.INERTIA, dtype=float))
        self.Iinv = np.diag(1.0 / np.array(P.INERTIA, dtype=float))
        self.rotor_pos = np.array(P.ROTOR_POSITIONS, dtype=float)
        self.rotor_dir = np.array(P.ROTOR_DIRECTIONS, dtype=float)
        self.x = np.zeros(17)
        self.reset()

    def reset(self, alt: float = 0.0):
        self.x[:] = 0.0
        self.x[2] = alt
        self.x[6] = 1.0  # quaternion w

    def _deriv(self, x, duty, vbat):
        v = x[3:6]
        q = x[6:10]
        w = x[10:13]
        om = x[13:17]
        om_dot = (P.pwm_to_omega(duty, vbat) - om) / P.TAU_M_S
        thrust = P.K_T * om * om
        R = quat_to_R(q)
        fb = np.array([0.0, 0.0, thrust.sum()])
        acc = R @ fb / P.MASS_KG + np.array([0.0, 0.0, -P.GRAVITY])
        tau_x = float((self.rotor_pos[:, 1] * thrust).sum())
        tau_y = float(-(self.rotor_pos[:, 0] * thrust).sum())
        tau_z = float((self.rotor_dir * P.K_Q * om * om).sum())
        tau = np.array([tau_x, tau_y, tau_z])
        w_dot = self.Iinv @ (tau - np.cross(w, self.I @ w))
        acc = acc - np.asarray(P.LINEAR_DRAG) * v / P.MASS_KG
        w_dot = w_dot - np.asarray(P.ANGULAR_DRAG) * w / np.array(P.INERTIA)
        q_dot = 0.5 * quat_mul(q, np.array([0.0, w[0], w[1], w[2]]))
        return np.concatenate([v, acc, q_dot, w_dot, om_dot])

    def step(self, dt, duty, vbat):
        duty = np.clip(np.asarray(duty, dtype=float), 0.0, 1.0)
        k1 = self._deriv(self.x, duty, vbat)
        k2 = self._deriv(self.x + 0.5 * dt * k1, duty, vbat)
        k3 = self._deriv(self.x + 0.5 * dt * k2, duty, vbat)
        k4 = self._deriv(self.x + dt * k3, duty, vbat)
        self.x = self.x + dt / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        self.x[6:10] = self.x[6:10] / np.linalg.norm(self.x[6:10])
        self.x[13:17] = np.maximum(self.x[13:17], 0.0)
        if self.x[2] < 0.0:  # simple ground contact
            self.x[2] = 0.0
            if self.x[5] < 0.0:
                self.x[5] = 0.0
        if not np.all(np.isfinite(self.x)):
            raise FloatingPointError("plant state diverged")
        return self.x

    def finite(self) -> bool:
        return bool(np.all(np.isfinite(self.x)))

    def sense(self, dt=0.001):
        """Specific force + body rate in FRD (before IMU noise).

        f_body = R^T (a_world - g_world). At rest on the ground a_world = 0,
        which yields [0, 0, -g] in FRD (T3.10); the ground-clamp is what makes
        that true instead of the free-flight a_world = -g."""
        om = self.x[13:17]
        thrust = P.K_T * om * om
        fb = np.array([0.0, 0.0, thrust.sum()]) / P.MASS_KG
        R = quat_to_R(self.x[6:10])
        a_world = R @ fb + np.array([0.0, 0.0, -P.GRAVITY])
        on_ground = (self.x[2] <= 1e-9) and (self.x[5] <= 1e-9)
        if on_ground:
            a_world = np.zeros(3)
        f_body = R.T @ (a_world + np.array([0.0, 0.0, P.GRAVITY]))
        accel_frd = to_frd(f_body)
        gyro_frd = to_frd(self.x[10:13])
        return gyro_frd, accel_frd

    def rate_deriv(self, duty, vbat=P.BATTERY["vbat_nominal"]):
        """Body angular acceleration for a duty command (sign contract)."""
        return self._deriv(self.x, np.asarray(duty, dtype=float), vbat)[10:13]

    @property
    def pos(self):
        return self.x[0:3]

    @property
    def vel(self):
        return self.x[3:6]

    @property
    def quat(self):
        return self.x[6:10]

    @property
    def omega(self):
        return self.x[10:13]

    @property
    def motors(self):
        return self.x[13:17]

    def euler(self):
        """Roll, pitch (nose-down +), yaw (FRD-ish) in rad."""
        q = self.x[6:10]
        R = quat_to_R(q)
        # body FRD z is world -z; derive angles from the FRD rotation R_frd = R @ diag(1,1,-1)
        Rf = R @ np.diag([1.0, 1.0, -1.0])
        pitch = np.arcsin(np.clip(-Rf[2, 0], -1.0, 1.0))
        roll = np.arctan2(Rf[2, 1], Rf[2, 2])
        yaw = np.arctan2(Rf[1, 0], Rf[0, 0])
        return float(roll), float(pitch), float(yaw)


def make_plant(name: str = "rk4", clean: bool = False, seed: int = 0):
    if name == "rk4":
        return Rk4Plant(clean=clean, seed=seed)
    if name == "rotorpy":
        from .vehicle_rotorpy import RotorPyPlant  # P4

        return RotorPyPlant(seed=seed, clean=clean)
    raise ValueError(f"unknown plant: {name}")
