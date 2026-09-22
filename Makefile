# Drone mini SIL/HIL stack — developer entry points.
VENV := .venv
PY   := $(VENV)/bin/python

.PHONY: venv host sil fw test gate plot compare clean venv-visual sim2sim

venv:
	uv venv --python 3.13 $(VENV)
	uv pip install --python $(PY) -r requirements-dev.txt

# Optional fidelity/visual deps (MuJoCo). Not needed for `make gate`.
venv-visual:
	uv pip install --python $(PY) -r requirements-visual.txt

# Sim-to-sim comparison across engines (RK4 authoritative, MuJoCo opt-in).
sim2sim:
	PYTHONPATH=. $(PY) tools/sim_to_sim.py $(ARGS)

host:
	cmake -S firmware -B build/host -DDRONE_HOST_LIB=ON
	cmake --build build/host -j

sil:
	cmake -S sim -B build/sim
	cmake --build build/sim -j

# Chip build requires ESP-IDF v6.0.x in the environment (P9).
fw:
	cmake -S firmware -B build/fw
	cmake --build build/fw -j

# Unit tests only (excludes slow end-to-end simulations).
# PYTHONPATH is cleared so an inherited PYTHONPATH (e.g. a sourced ROS setup)
# cannot inject incompatible pytest plugins into the venv.
test:
	PYTHONPATH= $(PY) -m pytest -m "not slow" tests

# Headline gate: unit + slow simulation criteria. Nothing excluded.
gate:
	PYTHONPATH= $(PY) -m pytest tests

plot:
	PYTHONPATH=. $(PY) tools/plot_sil.py $(LOG) $(if $(OUT),--out $(OUT),--out logs/plot.png)

compare:
	PYTHONPATH=. $(PY) tools/compare_logs.py $(A) $(B)

clean:
	rm -rf build logs/*.csv logs/*.png
