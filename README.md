# Neuromorphic Logic Controller

A simulation environment where a virtual agent is controlled by an
event-stream-processing "brain" implemented as a Finite State Machine (FSM)
in SystemVerilog. A Python model of the agent and its environment drives
the real, compiled RTL in a hardware-in-the-loop loop — the FSM isn't
mocked in software, it's the actual `.sv` design running under a
SystemVerilog simulator.

See [`docs/architecture.md`](docs/architecture.md) for the full design
writeup, including how state is carried between per-step hardware
invocations.

## Requirements

- [Icarus Verilog](http://iverilog.icarus.com/) (`iverilog`, `vvp`) — `brew install icarus-verilog`
- Python 3.10+ with `matplotlib` (see `sim/requirements.txt`)
- (optional) [GTKWave](http://gtkwave.sourceforge.net/) to view `build/*.vcd` waveforms

## Quickstart

```bash
# Run the self-checking RTL unit tests
make unit-tb

# Build the cosim testbench and run a full closed-loop episode
make cosim

# Run a randomized multi-episode verification sweep
make stress-test
```

`make cosim` writes `results/trajectory_log.csv` and `results/trajectory.png`
— the agent's path through a small obstacle field, driven step-by-step by
the SystemVerilog FSM.

`make stress-test` runs 25 independently-seeded random obstacle courses
through the same real hardware loop and reports the collision rate to
`results/stress_test_summary.csv`. Pass extra args through
`./scripts/run_stress_test.sh --episodes 100 --seed 42`; any failing seed
is reproducible on its own with `--episodes 1 --seed <n>`.

## Web simulation

**Live demo:** https://neuromorphic-logic-controller-srikar-velas-projects.vercel.app

`web/` is a browser visualization of the same controller — a React + Vite +
TypeScript app with a Canvas view, play/pause/step controls, and a
randomize-course button. It runs a TypeScript port of the FSM
(`web/engine/fsm.ts`), parity-tested against the exact same vectors as
`tb/tb_fsm_controller.sv` (`web/engine/fsm.test.ts`), rather than the real
`.sv` — see [`docs/architecture.md`](docs/architecture.md#web-simulation)
for why, and the planned path to running the actual RTL client-side via
WASM.

```bash
cd web
npm install
npm run dev     # http://localhost:5173
npm test        # parity tests
npm run build   # production bundle
```

## Layout

```
rtl/     fsm_controller.sv — the synthesizable FSM "brain"
tb/      unit testbench + per-step hardware-in-the-loop testbench
sim/     Python agent/environment model and the cosim driver
scripts/ shell wrappers around iverilog/vvp
docs/    architecture notes
web/     browser visualization (React + Vite + TS)
```
