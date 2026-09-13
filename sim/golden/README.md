# Parity oracles

Captured from the original rate-fed controller (commit `4277808`, before the
AXI-Stream event pipeline existed): `sim/agent_sim.py::sense` fed 2-bit
event-rate buckets straight into `fsm_controller` via a per-step `vvp`
invocation.

- `trajectory_default_ratefed.csv` — `make cosim` on the default course (45 steps)
- `stress_seed0_25_ratefed.csv` — `stress_test.py --episodes 25 --seed 0`

`sim/test_parity.py` re-runs both through the event pipeline (DVS events
in, hardware-side windowed counting and quantization) and asserts the
decisions and trajectories are identical. These files must never be
regenerated from the new pipeline: they are the independent reference.
