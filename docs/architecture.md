# Architecture

## Concept

An insect-inspired reactive obstacle-avoidance controller. A virtual agent
moving through a 2D environment carries a simplified event-camera "sensor"
split into left and right hemispheres. Looming activity (an obstacle getting
closer) raises the event rate on the corresponding hemisphere. A finite
state machine implemented in SystemVerilog reads those two event-rate
readings and reactively steers away from whichever side is busier, or
brakes if both sides spike at once.

There is no path planning, no map, and no memory beyond one FSM state
register — behavior emerges entirely from the reflexive left/right coupling,
the way an insect's optic-flow avoidance works.

## RTL: `rtl/fsm_controller.sv`

Four states:

| State     | Meaning                                | Encoding |
|-----------|-----------------------------------------|----------|
| `FORWARD` | no obstacle detected, drive straight    | `2'b00`  |
| `TURN_L`  | obstacle on the right, steer left       | `2'b01`  |
| `TURN_R`  | obstacle on the left, steer right       | `2'b10`  |
| `BRAKE_S` | obstacle on both sides at once          | `2'b11`  |

Inputs `ev_left`/`ev_right` are 2-bit event-rate buckets (0-3). Thresholds:

- `OBSTACLE_THRESH` (default 2): either side alone triggers a turn away from it.
- `BRAKE_THRESH` (default 3): both sides at this level force `BRAKE_S`.
- `BRAKE_CYCLES` (default 4): how long `BRAKE_S` is held before re-evaluating.

Turning states have hysteresis: once turning, the FSM holds that state until
the triggering side's event rate drops back under `OBSTACLE_THRESH`, not
just for one clean cycle. `BRAKE_S` always takes priority and interrupts a
turn if a critical front reading appears mid-turn, reloading `brake_timer`
to `BRAKE_CYCLES` rather than carrying over a stale value.

**Known one-cycle flicker:** the countdown-expiry check
(`brake_timer == 0 -> FORWARD`) doesn't re-check `critical_front`. If the
obstacle is still critical exactly when the timer hits zero, the FSM spends
one cycle in `FORWARD` before `critical_front` sends it straight back into a
freshly-timed `BRAKE_S` the next cycle. This is intentional (keeping the
countdown-expiry case simple) and is covered by the "flicker re-entry" test
in `tb/tb_fsm_controller.sv`, not a bug to fix.

## Verification: `tb/tb_fsm_controller.sv`

Directed, self-checking testbench run under Icarus Verilog
(`scripts/run_unit_tb.sh`). Covers: reset, clear-path forward, left/right
avoidance and recovery, and the full brake/countdown/recover sequence.
Stimulus is driven on `negedge clk` and checked shortly after `posedge clk`
to avoid racing the DUT's own clocked always block — driving new inputs
immediately after a `posedge` is a classic same-edge race against whatever
else is also triggered by that edge.

## Hardware-in-the-loop cosim

`sim/cosim_driver.py` owns the agent's physics and the environment. On each
control step it:

1. Computes `(ev_left, ev_right)` from obstacle geometry (`sim/agent_sim.py::sense`).
2. Invokes the compiled `tb/tb_cosim_step.sv` testbench via `vvp`, passing
   the previous FSM state/brake-timer and this step's event readings as
   `+plusargs`.
3. Parses the single `RESULT ...` line the testbench prints and applies the
   resulting action to the agent (`apply_action`).

### Why a separate testbench for cosim

Icarus is invoked fresh per control step — there's no persistent simulation
process to hold register state between steps the way `cocotb` would. To
make each invocation "resume" from where the last one left off,
`tb_cosim_step.sv` forces the DUT's `state` and `brake_timer` registers to
the saved values right after reset, releases the force before the next
clock edge, and lets exactly one real `posedge` run the FSM's normal logic
from that point. This is a testbench-only backdoor (`force`/`release` on a
hierarchical reference) — not synthesizable, and not how the FSM would be
integrated on real hardware, where it would just run continuously. It's a
stand-in for a proper `cocotb`/`Verilator` persistent cosim, which was
skipped here since Icarus + one-shot `vvp` runs needed no extra toolchain.

### Engineering shortcuts worth knowing about

- `ev_left`/`ev_right` are a geometric proxy for real event-camera output
  (proximity² per hemisphere, bucketed to 2 bits), not a simulated pixel
  array. There's no dark current, refractory period, or per-pixel noise
  modeling here — see the `V2E Simulator` project idea for that.
- A single obstacle can only ever register on one hemisphere (bearing is
  strictly left-or-right of center), so a lone obstacle never triggers
  `BRAKE_S` by itself — braking only happens when the geometry puts
  meaningful looming on both sides simultaneously. That's a deliberate
  simplification, not a bug.
- The controller never re-centers heading after an avoidance turn; it just
  keeps going straight on the new heading. Obstacle placement in
  `sim/cosim_driver.py::build_default_environment` is tuned to sit on the
  agent's actual post-avoidance drift path so each one still gets a
  distinct reaction — moving obstacles around will change how many actually
  get "seen."

## Randomized verification: `sim/stress_test.py`

The unit testbench proves individual transitions are correct; it doesn't
say much about whether the reactive controller actually avoids obstacles in
general. `sim/stress_test.py` runs many independently-seeded random
obstacle courses through the same real `vvp`-compiled FSM used by
`cosim_driver.py` (no shortcuts — it calls `cosim_driver.run_fsm_step`
directly) and reports a collision rate plus how many episodes triggered a
turn or brake at all. Obstacle y-position is kept close to the agent's
nominal straight-line path deliberately — a wide spread mostly produces
obstacles that were never a threat, which would make the sweep meaningless.
Each episode is fully determined by its seed, so a failing seed can be
replayed in isolation.

Brake engagements are rare in this sweep by construction: the random
generator places at most one obstacle per x-band, and `BRAKE_S` requires
simultaneous bilateral criticality, which mostly needs two obstacles close
together on opposite sides. That case is covered directly by the unit
tests instead.

## Web simulation

`web/` is a browser-based visualization, built because real SystemVerilog
can't execute client-side without extra toolchain (WASM). Architecture:

- `web/engine/fsm.ts` — a TypeScript port of `rtl/fsm_controller.sv`'s
  transition logic. It is **not** the RTL itself; it's a reference model
  kept honest by `web/engine/fsm.test.ts`, which replays the exact same
  30-step vector sequence (directed + edge cases) as
  `tb/tb_fsm_controller.sv` and asserts identical results.
- `web/engine/agentSim.ts` — a TypeScript port of `sim/agent_sim.py`.
- `web/engine/FsmEngine.ts` — a small `FsmEngine` interface
  (`step(state, timer, evLeft, evRight)`) that the UI depends on instead of
  `fsm.ts` directly. `TsFsmEngine` is the only implementation today.
- `web/engine/simulationLoop.ts` — the real-time counterpart to
  `sim/cosim_driver.py::run_episode`, advancing one control tick per call
  instead of batch-running to completion.
- `web/frontend/` — React + Canvas UI (play/pause/step, randomize course,
  live event-rate bars and state badge).

**Planned: real RTL in the browser.** The `FsmEngine` interface exists so a
`WasmFsmEngine` can be added later without touching the UI — compile
`rtl/fsm_controller.sv` through Verilator to C++, then through Emscripten
to WebAssembly, wrapped behind the same `step()` signature. Neither
Verilator nor an Emscripten SDK is set up yet; this is a larger toolchain
lift than the TypeScript port and is deliberately deferred rather than
blocking the visualization on it.

## Directory layout

```
rtl/     synthesizable SystemVerilog (fsm_controller.sv)
tb/      testbenches (unit + per-step cosim)
sim/     Python agent/environment model, cosim driver, and stress test
scripts/ build/run helpers wrapping iverilog/vvp
build/   compiled .vvp binaries and .vcd waveforms (gitignored)
results/ CSV logs and trajectory plots from cosim runs (gitignored)
web/     browser visualization (React + Vite + TS), see above
```
