"""1S LiPo battery: Thevenin sag + 3V3 rail lag + brownout flags."""

from __future__ import annotations

CAPACITY_AH = 2.0  # pack capacity; drain = dt * i / (CAPACITY_AH * 3600)


class Battery:
    """1S LiPo pack.

    `Battery(cfg)` with no `initial_charge` picks the SoC whose loaded voltage at
    hover duty is the nominal cell voltage (D9), so the config must define
    `hover_duty`. Pass `initial_charge` explicitly for any other pack state.
    """

    def __init__(self, cfg, initial_charge: float | None = None):
        self.cfg = cfg
        if initial_charge is None:
            if "hover_duty" not in cfg:
                raise KeyError(
                    "Battery: default SoC needs cfg['hover_duty']; "
                    "pass initial_charge explicitly otherwise"
                )
            initial_charge = self._hover_soc()
        self.charge = initial_charge
        self.v3v3_lag = self._ocv()
        self.rail_low = False
        self.brownout = False

    def _hover_soc(self) -> float:
        """SoC whose loaded voltage at hover duty is the nominal cell voltage.

        Keeps the open-loop runner honest: a fresh full pack starts high and
        climbs, so the default pack must sit at the hover operating point (D9).
        """
        i = self.current(4.0 * self.cfg["hover_duty"])
        ocv = self.cfg["vbat_nominal"] + i * self.cfg["r0_ohm"]
        span = self.cfg["ocv_full"] - self.cfg["ocv_empty"]
        return (ocv - self.cfg["ocv_empty"]) / span

    def _ocv(self) -> float:
        lo, hi = self.cfg["ocv_empty"], self.cfg["ocv_full"]
        return lo + (hi - lo) * max(min(self.charge, 1.0), 0.0)

    def current(self, duty_sum: float) -> float:
        """Motor current from summed duty (>= 0). 1S whoop scale: ~0.3 A idle,
        ~1.2 A per unit of summed duty (~3 A at hover)."""
        return 0.3 + max(0.0, duty_sum) * 1.2

    def step(self, dt: float, duty_sum: float) -> float:
        i = self.current(duty_sum)
        ocv = self._ocv()
        v = ocv - i * self.cfg["r0_ohm"]  # R0 dominant (A6)
        # 3V3 rail follows with an LDO lag (A10)
        tau = 3e-4
        self.v3v3_lag += (v - self.cfg["ldo_dropout_v"]) * (dt / (tau + dt)) - (
            self.v3v3_lag * (dt / (tau + dt))
        )
        self.v3v3_lag = min(self.v3v3_lag, v)
        self.rail_low = self.v3v3_lag < 3.0
        self.brownout = v < 2.9
        # drain charge from current draw
        self.charge = max(0.0, self.charge - dt * i / (CAPACITY_AH * 3600.0))
        return v

    def voltage(self) -> float:
        return self._ocv() - self.current(0.0) * self.cfg["r0_ohm"]

    def rail3v3(self) -> float:
        return self.v3v3_lag
