"""1S LiPo battery: Thevenin sag + 3V3 rail lag + brownout flags."""

from __future__ import annotations


class Battery:
    def __init__(self, cfg, initial_charge: float = 1.0):
        self.cfg = cfg
        self.charge = initial_charge
        self.i_rc = 0.0      # 1-RC branch current state (A*s)
        self.v3v3_lag = self._ocv()
        self.rail_low = False
        self.brownout = False

    def _ocv(self) -> float:
        lo, hi = self.cfg["ocv_empty"], self.cfg["ocv_full"]
        return lo + (hi - lo) * max(min(self.charge, 1.0), 0.0)

    def current(self, duty_sum: float) -> float:
        """Motor current from summed duty (>= 0). 1S whoop scale: ~0.3 A idle,
        ~1.2 A per unit of summed duty (~3 A at hover)."""
        return 0.3 + max(0.0, duty_sum) * 1.2

    def step(self, dt: float, duty_sum: float) -> float:
        i = self.current(duty_sum)
        self.i_rc += dt * (i - self.i_rc / (self.cfg["r1_ohm"] * self.cfg["c1_f"] + 1e-9))
        ocv = self._ocv()
        v = ocv - i * self.cfg["r0_ohm"] - self.i_rc * 0.0  # R0 dominant (A6)
        # 3V3 rail follows with an LDO lag (A10)
        tau = 3e-4
        self.v3v3_lag += (v - self.cfg["ldo_dropout_v"]) * (dt / (tau + dt)) - (
            self.v3v3_lag * (dt / (tau + dt))
        )
        self.v3v3_lag = min(self.v3v3_lag, v)
        self.rail_low = self.v3v3_lag < 3.0
        self.brownout = v < 2.9
        # drain charge from current draw
        self.charge = max(0.0, self.charge - dt * i / 3600.0 * 0.5)
        return v

    def voltage(self) -> float:
        return self._ocv() - self.current(0.0) * self.cfg["r0_ohm"]

    def rail3v3(self) -> float:
        return self.v3v3_lag
