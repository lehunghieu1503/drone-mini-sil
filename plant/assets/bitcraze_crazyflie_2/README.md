# Vendored model: Bitcraze Crazyflie 2 (MJCF)

Source: https://github.com/google-deepmind/mujoco_menagerie/tree/main/bitcraze_crazyflie_2
License: MIT (see `LICENSE` in this directory).
Vendored verbatim (`cf2.xml`, `scene.xml`, `assets/*.obj`) so the opt-in MuJoCo
plant is offline and reproducible.

Used by `plant/vehicle_mujoco.py` (`--plant mujoco`). The adapter overwrites the
body mass/inertia with `drone_mini_params` by default (drop-in engine swap) and
drives the body through `mjData.xfrc_applied` with the same propulsion model as
`Rk4Plant`. Pass `params="menagerie"` to keep the model's own values
(mass 0.027 kg, inertia 2.3951e-5/3.2347e-5).

Upstream note: the model exposes a CTBR abstraction (collective thrust + x/y/z
moments). Its moment actuators are ~300x weaker than the physical moments our
mixer produces, so the adapter does not use them; it applies the wrench directly.
