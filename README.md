# Neuromorphic Logic Controller

[![Live Demo](https://img.shields.io/badge/Live-Demo-brightgreen)](https://neuromorphic-logic-controller-srikar-velas-projects.vercel.app)
[![CI](https://github.com/srikarvela/neuromorphic-logic-controller/actions/workflows/ci.yml/badge.svg)](https://github.com/srikarvela/neuromorphic-logic-controller/actions/workflows/ci.yml)

An insect-inspired reactive obstacle-avoidance controller. A virtual agent
moving through a 2D environment carries a simplified event-camera "sensor"
split into left/right hemispheres. A four-state finite state machine,
written in real SystemVerilog, reads the two event-rate readings and
reactively steers away from whichever side is busier — no path planning,
no map, no memory beyond one state register. Behavior emerges entirely
from the reflexive left/right coupling.

The FSM isn't mocked in software: a Python model of the agent and
environment drives the actual compiled `.sv` design, one control step at a
time, through a real SystemVerilog simulator (Icarus Verilog).

**[→ Try the live browser demo](https://neuromorphic-logic-controller-srikar-velas-projects.vercel.app)**

## How it thinks

| State | Trigger | Action |
|---|---|---|
| `FORWARD` | no obstacle detected | drive straight |
| `TURN_R` | obstacle on the left | steer right, away from it |
| `TURN_L` | obstacle on the right | steer left, away from it |
| `BRAKE` | obstacle on **both** sides at once | stop, hold for `BRAKE_CYCLES`, then re-evaluate |

Full transition logic, thresholds, and the one documented edge-case
"flicker" behavior are in [`rtl/fsm_controller.sv`](rtl/fsm_controller.sv)
and written up in [`docs/architecture.md`](docs/architecture.md).

## What's in the repo

- **RTL** — the synthesizable FSM, with a self-checking unit testbench
  covering every transition, including brake-preempts-turn edge cases
  (42/42 checks passing).
- **Hardware-in-the-loop cosim** — Python owns the agent physics; the real
  compiled RTL runs the control logic, invoked once per step via `vvp`.
- **Randomized verification sweep** — many seeded random obstacle courses
  run through the same real hardware loop, reporting a collision rate.
- **Browser visualization** — a React + Canvas app running a
  parity-tested TypeScript port of the FSM, with a documented path to
  swapping in the real RTL compiled to WASM.
- **CI** — unit tests, a cosim episode, a stress-test smoke check, and the
  web app's typecheck/tests/build all run on every push.

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
sim/     Python agent/environment model, cosim driver, and stress test
scripts/ shell wrappers around iverilog/vvp
docs/    architecture notes
web/     browser visualization (React + Vite + TS)
```

See [`docs/architecture.md`](docs/architecture.md) for the full design
writeup, including how state is carried between per-step hardware
invocations and the deliberate simplifications in the sensor model.
