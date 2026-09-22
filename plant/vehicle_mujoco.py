"""MuJoCo plant adapter (Menagerie Crazyflie 2, MIT) — opt-in, sim-to-sim fidelity.

Third `Plant` implementation next to `Rk4Plant` and `RotorPyPlant`. It uses the
vendored Menagerie `bitcraze_crazyflie_2` MJCF for geometry, collision, contacts
and IMU sensors, and drives the rigid body with the **same propulsion model as
`Rk4Plant`** (duty -> first-order motor lag -> `k_t*omega^2` thrust + `k_q`
reaction torque), applied through `mjData.xfrc_applied` (world frame, at the
body COM). This keeps the controller/actuator path identical across engines so a
sim-to-sim comparison isolates the integrator/aero/contact difference.

Rigid-body parameters: by default the body mass/inertia are overwritten with
`drone_mini_params` (MASS_KG, INERTIA) so `--plant mujoco` is a drop-in engine
swap for the existing scenarios/tests. Pass `params="menagerie"` to keep the
Menagerie Crazyflie 2 values (mass 0.027 kg) instead.

Frames: MuJoCo world is z-up (ENU-like), body is FLU. Firmware is FRD, so
`to_frd` is applied at the `sense()` boundary only — same contract as Rk4Plant.

Determinism: not bit-for-bit across MuJoCo versions (same caveat as RotorPy,
plan D20). The RK4 path stays the authoritative reference. Run with
`--plant mujoco`.
"""

from __future__ import annotations

import pathlib

import numpy as np

from . import drone_mini_params as P
from .vehicle import quat_to_R, to_frd

_ASSET_DIR = pathlib.Path(__file__).resolve().parent / "assets" / "bitcraze_crazyflie_2"
_DEFAULT_XML = _ASSET_DIR / "scene.xml"
_BODY_NAME = "cf2"


class MuJoCoPlant:
    def __init__(self, seed: int = 0, clean: bool = False, xml: str | None = None,
                 params: str = "drone_mini"):
        import mujoco  # local import: opt-in dependency (see requirements-visual.txt)

        if params not in ("drone_mini", "menagerie"):
            raise ValueError(f"unknown params: {params}")

        self._mj = mujoco
        self.clean = clean
        self.params = params
        self.rng = np.random.default_rng(seed)
        self.m = mujoco.MjModel.from_xml_path(str(xml or _DEFAULT_XML))
        self.m.opt.timestep = P.DT  # 1 kHz virtual tick (model integrator is RK4)
        self.d = mujoco.MjData(self.m)
        self._bid = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, _BODY_NAME)

        if params == "drone_mini":
            self.m.body_mass[self._bid] = P.MASS_KG
            self.m.body_inertia[self._bid] = np.array(P.INERTIA, dtype=float)
        mujoco.mj_setConst(self.m, self.d)

        # Propulsion state, identical model to Rk4Plant._deriv.
        self.om = np.zeros(4)
        self.rotor_pos = np.array(P.ROTOR_POSITIONS, dtype=float)
        self.rotor_dir = np.array(P.ROTOR_DIRECTIONS, dtype=float)
        self._I = np.diag(np.array(P.INERTIA, dtype=float))
        self._Iinv = np.diag(1.0 / np.array(P.INERTIA, dtype=float))
        self.reset()

    def reset(self, alt: float = 0.0):
        mj = self._mj
        mj.mj_resetData(self.m, self.d)
        self.d.qpos[0:3] = [0.0, 0.0, alt]
        self.d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        self.d.xfrc_applied[:] = 0.0
        self.om[:] = 0.0
        mj.mj_forward(self.m, self.d)

    def _wrench_body(self, duty, vbat, dt):
        """Per-rotor thrust -> total body force (FLU, +z up) and body torque."""
        om_cmd = np.asarray(P.pwm_to_omega(duty, vbat), dtype=float)
        self.om = self.om + dt * (om_cmd - self.om) / P.TAU_M_S
        np.maximum(self.om, 0.0, out=self.om)
        thrust = P.K_T * self.om * self.om
        tau_x = float((self.rotor_pos[:, 1] * thrust).sum())
        tau_y = float(-(self.rotor_pos[:, 0] * thrust).sum())
        tau_z = float((self.rotor_dir * P.K_Q * self.om * self.om).sum())
        return np.array([0.0, 0.0, float(thrust.sum())]), np.array([tau_x, tau_y, tau_z])

    def step(self, dt, duty, vbat):
        duty = np.clip(np.asarray(duty, dtype=float), 0.0, 1.0)
        f_body, tau_body = self._wrench_body(duty, vbat, dt)
        R = quat_to_R(self.d.qpos[3:7])
        self.d.xfrc_applied[self._bid, 0:3] = R @ f_body
        self.d.xfrc_applied[self._bid, 3:6] = R @ tau_body
        self._mj.mj_step(self.m, self.d)
        if not self.finite():
            raise FloatingPointError("mujoco state diverged")
        return self.d.qpos

    def finite(self) -> bool:
        return bool(np.all(np.isfinite(self.d.qpos)) and np.all(np.isfinite(self.d.qvel)))

    def sense(self, dt=0.001):
        """Specific force + body rate in FRD (before IMU noise), from MuJoCo sensors."""
        gyro_frd = to_frd(np.asarray(self.d.sensor("body_gyro").data, dtype=float))
        accel_frd = to_frd(np.asarray(self.d.sensor("body_linacc").data, dtype=float))
        return gyro_frd, accel_frd

    def rate_deriv(self, duty, vbat=P.BATTERY["vbat_nominal"]):
        """Body angular acceleration for a duty command (sign contract)."""
        om = np.asarray(P.pwm_to_omega(duty, vbat), dtype=float)
        thrust = P.K_T * om * om
        tau = np.array([
            float((self.rotor_pos[:, 1] * thrust).sum()),
            float(-(self.rotor_pos[:, 0] * thrust).sum()),
            float((self.rotor_dir * P.K_Q * om * om).sum()),
        ])
        w = np.asarray(self.d.qvel[3:6], dtype=float)
        return self._Iinv @ (tau - np.cross(w, self._I @ w))

    @property
    def pos(self):
        return np.asarray(self.d.qpos[0:3], dtype=float)

    @property
    def vel(self):
        return np.asarray(self.d.qvel[0:3], dtype=float)

    @property
    def quat(self):
        return np.asarray(self.d.qpos[3:7], dtype=float)  # (w, x, y, z)

    @property
    def omega(self):
        return np.asarray(self.d.qvel[3:6], dtype=float)  # body FLU

    @property
    def motors(self):
        return self.om.copy()

    def euler(self):
        """Roll, pitch (nose-down +), yaw in rad (same convention as Rk4Plant)."""
        Rf = quat_to_R(self.d.qpos[3:7]) @ np.diag([1.0, 1.0, -1.0])
        pitch = np.arcsin(np.clip(-Rf[2, 0], -1.0, 1.0))
        roll = np.arctan2(Rf[2, 1], Rf[2, 2])
        yaw = np.arctan2(Rf[1, 0], Rf[0, 0])
        return float(roll), float(pitch), float(yaw)
