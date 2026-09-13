# Neuromorphic Logic Controller

[![Live Demo](https://img.shields.io/badge/Live-Demo-brightgreen)](https://neuromorphic-logic-controller-srikar-velas-projects.vercel.app)
[![CI](https://github.com/srikarvela/neuromorphic-logic-controller/actions/workflows/ci.yml/badge.svg)](https://github.com/srikarvela/neuromorphic-logic-controller/actions/workflows/ci.yml)

An insect-inspired reactive obstacle-avoidance controller, built as an
FPGA event-processing pipeline with hardware-in-the-loop validation.

A virtual agent moving through a 2D environment carries a DVS-style event
camera (128 x 128 px, address-event output). Hardware counts the events on
the left and right halves of the field of view over fixed windows,
quantizes the counts into coarse rate buckets, and a four-state finite
state machine steers away from whichever side is busier — no path
planning, no map, no memory beyond one state register. Behavior emerges
entirely from the reflexive left/right coupling.

Nothing in the controller is mocked in software. The windowed event
counters, quantizer and FSM are synthesizable SystemVerilog behind two
AXI4-Stream ports; a Python model of the agent, environment and event
camera streams event packets in and reads decision words back — through a
persistent Icarus Verilog simulation on a laptop, or over AXI DMA to the
same RTL running on a PYNQ-Z2's XC7Z020. The harness is identical in both
cases, and a pure-Python golden model cross-checks every decision as it
comes back.

**[→ Try the live browser demo](https://neuromorphic-logic-controller-srikar-velas-projects.vercel.app)**

```
                Python (host)                       │            RTL (Icarus or XC7Z020)
                                                    │
  agent physics ──► event camera ──► packet of ─────┼──► s_axis ──► event_rate_window ──► fsm_controller
  (agent_sim)      (event_camera)   event words     │   (AXI-S)     windowed counters       4-state FSM
       ▲                                            │               + quantizer             one step / window
       │                                            │                                            │
       └──── apply_action ◄── decision word ◄───────┼──── m_axis ◄──── output register ◄─────────┘
                                                    │
                     transport:  Icarus = stdin/stdout pipe  ·  PYNQ-Z2 = AXI DMA
```

## How it thinks

| State | Trigger | Action |
|---|---|---|
| `FORWARD` | no obstacle detected | drive straight |
| `TURN_R` | obstacle on the left | steer right, away from it |
| `TURN_L` | obstacle on the right | steer left, away from it |
| `BRAKE` | obstacle on **both** sides at once | stop, hold for `BRAKE_CYCLES` windows, then re-evaluate |

"Obstacle" means the hemisphere's event count over one window crossed the
hardware quantizer's thresholds (25 / 55 / 85 events → buckets 1 / 2 / 3;
bucket 2 is an obstacle, bucket 3 on both sides is critical). Full
transition logic, the window/catch-up/stale/reset semantics of the counter,
and the one documented edge-case "flicker" behavior are in
[`docs/architecture.md`](docs/architecture.md).

## What's in the repo

- **RTL** — `fsm_controller.sv` (the FSM, unchanged transition logic from
  the original rate-fed project), `event_rate_window.sv` (per-hemisphere
  saturating window counters + quantizer, in-band reset, catch-up for
  skipped windows, stale-event drop), `nlc_axis_top.sv` (AXI4-Stream
  wrapper: one decision word per closed window, backpressure-safe).
- **Three self-checking unit testbenches** — 163 checks under Icarus: FSM
  transitions and brake preemption, counter mechanics, and the whole
  pipeline against an independent FSM instance under random backpressure.
- **Hardware-in-the-loop harness** — Python owns the agent physics and the
  event camera; decisions come from the RTL. `--engine icarus` talks to one
  persistent `vvp` process over a pipe; `--engine pynq` talks to the FPGA
  over AXI DMA. Same code path either way, every decision golden-checked.
- **Parity proof** — the event pipeline is decision-for-decision identical
  to the original rate-fed FSM: the default 45-step trajectory and the
  25-episode stress sweep reproduce the pre-pipeline CSVs exactly
  (`sim/golden/`, asserted by `sim/test_parity.py`).
- **Randomized verification sweep** and an optional Poisson-noise sensor
  mode for probing threshold robustness.
- **Single-packet replay benchmark** — a whole recorded episode streamed
  as one DMA packet, one decision per timestamp window, checked against the
  closed-loop decisions; the pure pipeline+DMA throughput number on the board.
- **Vivado flow for the PYNQ-Z2** — block design (PS7 + AXI DMA + the
  pipeline as an RTL module reference), bitstream build, OOC synthesis for
  Fmax/utilization, and a PYNQ driver in the same style as the crypto feed
  handler's.
- **Browser visualization** — React + Canvas app running a parity-tested
  TypeScript port of the FSM (unchanged).
- **CI** — unit testbenches, parity tests, a cosim episode, replay, stress
  and noise smoke checks, and the web app's typecheck/tests/build on every push.

## Requirements

- [Icarus Verilog](http://iverilog.icarus.com/) 12+ (`iverilog`, `vvp`) — `brew install icarus-verilog`
- Python 3.10+ with `matplotlib` and `pytest` (`sim/requirements.txt`)
- (FPGA) Vivado 2022.2+ with the TUL PYNQ-Z2 board files; a PYNQ-Z2 on the
  network for the board targets — see [`docs/pynq_bringup.md`](docs/pynq_bringup.md)
- (optional) [GTKWave](http://gtkwave.sourceforge.net/) to view `build/*.vcd`

## Quickstart

```bash
# RTL unit testbenches (FSM, windowed counter, AXI-Stream pipeline)
make unit-tb

# Parity + model tests: event pipeline == rate-fed FSM, RTL == golden model
make test

# Closed-loop episode with the RTL under Icarus in the loop
make cosim

# Randomized multi-episode sweep (add --noise 10 for background DVS events)
make stress-test

# Whole recorded episode as one packet, one decision per window
make replay-bench
```

`make cosim` writes `results/trajectory_log.csv` (now including per-step
event counts and packet sizes), `results/trajectory.png`, and a replay
stream `results/replay.bin` + `results/replay.expected.csv`.

`make stress-test` runs 25 independently-seeded random obstacle courses
and reports the collision rate to `results/stress_test_summary.csv`. Pass
extra args through `./scripts/run_stress_test.sh --episodes 100 --seed 42`;
any failing seed is reproducible on its own with `--episodes 1 --seed <n>`.

### On the PYNQ-Z2

```bash
make bitstream                       # Vivado: fpga/build/nlc.bit + nlc.hwh
make board-deploy BOARD_HOST=xilinx@pynq
make board-smoke                     # reset + two hand-built windows
make board-cosim                     # the same 45-step course, FPGA in the loop
make board-stress                    # the same 25 episodes on silicon
make board-bench                     # DMA replay throughput
```

Every `sim/*.py` script takes `--engine {icarus,pynq,golden}`; the board
targets just run them on the board with `--engine pynq`.

## Results

Verified in this checkout (Icarus Verilog 12, macOS):

| check | result |
|---|---|
| `tb_fsm_controller` / `tb_event_rate_window` / `tb_nlc_axis_top` | 53 / 28 / 82 checks pass |
| `sim/test_event_stream.py` + `sim/test_parity.py` | 21 tests pass |
| default course, event pipeline vs rate-fed baseline | 45 steps, identical `ev/state/timer/x/y/heading` |
| stress sweep, 25 episodes, seed 0 | 0 collisions, identical per-seed summary to baseline, every decision == golden |
| stress sweep with `--noise 10`, 5 episodes | 0 collisions, every decision == golden |
| single-packet replay of the recorded episode | 889 words / 45 windows, 0 mismatches |

Not yet run from this checkout: Vivado synthesis and the on-board run. The
flow is written (`fpga/`, `board/pynq/`) and follows the working feed
handler's conventions, but Fmax, utilization and DMA throughput numbers
are to be filled in after the first `make bitstream` / `make board-bench`
— [`docs/pynq_bringup.md`](docs/pynq_bringup.md) is the checklist.

## Event stream format

One 32-bit AXI-Stream beat per event:

```
EVENT  bit31=1   [30] polarity   [29:23] y   [22:16] x   [15:0] timestamp
CTRL   bit31=0   [30] reset      [29:16] 0               [15:0] timestamp
```

`x < 64` is the left hemisphere. The timestamp's upper 8 bits are the
window id (256-tick windows). A CTRL word with `reset=0` is a sync
heartbeat (closes an empty window); with `reset=1` it resets the FSM
in-band. Decisions come back one per closed window:

```
[31:24] window id  [23:22] ev_left  [21:20] ev_right  [19:18] state
[17:15] brake timer [14] stale      [13:7] count_left [6:0] count_right
```

## Web simulation

`web/` is a browser visualization of the same controller — a React + Vite +
TypeScript app with a Canvas view, play/pause/step controls, and a
randomize-course button. It runs a TypeScript port of the FSM
(`web/engine/fsm.ts`) fed by the rate-fed sensor model, parity-tested
against the same vectors as `tb/tb_fsm_controller.sv`. Because the event
pipeline is decision-for-decision identical to that model, the demo stays a
faithful picture of what the FPGA does.

```bash
cd web
npm install
npm run dev     # http://localhost:5173
npm test        # parity tests
npm run build   # production bundle
```

## Layout

```
rtl/        fsm_controller.sv, event_rate_window.sv, nlc_axis_top.sv (synthesizable)
tb/         three unit testbenches + tb_cosim_server.sv (persistent HIL server)
sim/        agent physics, event camera, wire format, golden model, engines,
            cosim driver, stress test, replay bench, pytest suites, golden CSVs
board/pynq/ PYNQ-Z2 engine (AXI DMA)
fpga/       Vivado block design, bitstream + OOC synthesis Tcl, XDC
scripts/    iverilog/vvp wrappers
docs/       architecture writeup, PYNQ bring-up checklist
web/        browser visualization (React + Vite + TS)
```

See [`docs/architecture.md`](docs/architecture.md) for the full design
writeup: the pipeline's cycle-level timing and backpressure story, the
window semantics, why the event pipeline reproduces the rate-fed controller
bit-for-bit, and the deliberate simplifications in the sensor model.
