"""T3.6/T3.7 — battery sag and brownout flags."""

import pytest

from plant import drone_mini_params as P
from plant.battery import Battery


def test_t3_6_sag_vs_current():
    b = Battery(P.BATTERY)
    v0 = b._ocv()
    i = b.current(4.0)
    v_load = b.step(P.DT, 4.0)
    assert (v0 - v_load) == pytest.approx(i * b.cfg["r0_ohm"], rel=1e-6)


def test_t3_7_brownout_and_rail_flags():
    b = Battery(P.BATTERY, initial_charge=1.0)
    b.step(P.DT, 4.0)
    assert b.brownout is False
    # A worn pack (high internal resistance) at full load sags into brownout.
    config = dict(P.BATTERY)
    config["r0_ohm"] = 0.2
    b2 = Battery(config, initial_charge=0.0)
    b2.step(P.DT, 4.0)
    assert b2.brownout is True
    assert b2.rail_low is True
