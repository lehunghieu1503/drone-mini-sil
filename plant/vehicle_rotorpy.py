"""RotorPy 3.0.0 plant adapter (P4, conditional; RK4 remains authoritative).

Same `Plant` interface as Rk4Plant. RotorPy param conventions are close to our
sim frame: body FLU, world z-up, Crazyflie mass/arm. Determinism is only
guaranteed within the same rotorpy/scipy version (D20) — the RK4 path is the
bit-for-bit path.

This adapter is opt-in (`--plant rotorpy`); the RK4 path is the committed one."""

from __future__ import annotations

import numpy as np

from . import drone_mini_params as P
from .vehicle import to_frd


class RotorPyPlant:
    def __init__(self, seed: int = 0, clean: bool = False):
        from rotorpy.vehicles.multirotor import Multirotor
        import rotorpy.vehicles.crazyflie_params as cf

        self.clean = clean
        self.rng = np.random.default_rng(seed)
        self.veh = Multirotor(cf.quad_params)
        self.k_eta = float(cf.quad_params["k_eta"])
        self.mass = float(cf.quad_params["mass"])
        self.reset()

    def reset(self, alt: float = 0.0):
        self.state = dict(self.veh.initial_state)
        self.state["x"] = np.array([0.0, 0.0, alt], dtype=float)
        self.state["v"] = np.zeros(3)
        self.state["w"] = np.zeros(3)
        self.state["rotor_speeds"] = np.zeros(4)

    def step(self, dt, duty, vbat):
        om = np.asarray(P.pwm_to_omega(duty, vbat), dtype=float)
        control = {"cmd_motor_speeds": om}
        self.state = self.veh.step(self.state, control, dt)
        if not np.all(np.isfinite(self.state["x"])) or not np.all(np.isfinite(self.state["v"])):
            raise FloatingPointError("rotorpy state diverged")
        return self.state

    def finite(self) -> bool:
        return bool(np.all(np.isfinite(self.state["x"])) and np.all(np.isfinite(self.state["v"])))

    def sense(self, dt=0.001):
        q = np.asarray(self.state["q"], dtype=float)  # (x,y,z,w) scalar-last
        w = np.asarray(self.state["w"], dtype=float)
        om = np.asarray(self.state.get("rotor_speeds", np.zeros(4)), dtype=float)
        thrust = self.k_eta * float(np.sum(om * om))
        fb = np.array([0.0, 0.0, thrust / self.mass])
        accel_frd = to_frd(fb)
        gyro_frd = to_frd(w)
        return gyro_frd, accel_frd

    def rate_deriv(self, duty, vbat=P.BATTERY["vbat_nominal"]):
        # Not exposed by RotorPy; return zeros (adapter is smoke-tested only).
        return np.zeros(3)

    @property
    def pos(self):
        x = np.asarray(self.state["x"], dtype=float)
        return x if len(x) == 3 else np.array([x[0], x[1], 0.0])

    @property
    def vel(self):
        v = np.asarray(self.state["v"], dtype=float)
        return v if len(v) == 3 else np.zeros(3)

    @property
    def omega(self):
        w = np.asarray(self.state["w"], dtype=float)
        return w if len(w) == 3 else np.zeros(3)

    @property
    def motors(self):
        return np.asarray(self.state.get("rotor_speeds", np.zeros(4)), dtype=float)

    @property
    def quat(self):
        q = np.asarray(self.state["q"], dtype=float)  # x,y,z,w -> w,x,y,z
        return np.array([q[3], q[0], q[1], q[2]])

    def euler(self):
        return (0.0, 0.0, 0.0)
