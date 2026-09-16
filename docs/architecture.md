# Architecture

## Concept

An insect-inspired reactive obstacle-avoidance controller, built as an
FPGA event-processing pipeline. A virtual agent moving through a 2D
environment carries a DVS-style event camera (128 x 128 px, address-event
output: x, y, polarity, timestamp). Looming activity — an obstacle getting
closer — raises the event rate on the corresponding half of the field of
view. Hardware counts those events per hemisphere over a fixed window,
quantizes the counts into coarse rate buckets, and a finite state machine
reactively steers away from whichever side is busier, or brakes if both
sides spike at once.

There is no path planning, no map, and no memory beyond one FSM state
register — behavior emerges entirely from the reflexive left/right coupling,
the way an insect's optic-flow avoidance works. And the controller never
sees a "rate": it sees events.

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

## Pipeline: `rtl/nlc_axis_top.sv`

The synthesizable top. Two AXI4-Stream ports (32-bit, TDATA/TVALID/TREADY/
TLAST) and nothing else, so it drops straight between the two halves of an
AXI DMA in a Zynq block design, and the same file is what the Icarus cosim
server wraps. Internally:

1. `event_rate_window` sinks event words and closes windows.
2. On every window close it pulses `step_en` on `fsm_controller` for one
   clock with the window's two buckets (or resets the FSM, for an in-band
   reset word).
3. The new FSM state and brake timer, plus the window's counts, buckets,
   id and flags, are packed into one decision word in a one-deep output
   register.

Per-window timing (A = the cycle the accumulator decides to close):

| cycle | what happens |
|---|---|
| A     | `event_rate_window` registers the `win_*` fields; a closing word that had to wait is held in the input register (`s_axis_tready` low) |
| A+1   | `win_close` high: the FSM takes one `step_en` (or its registered reset flop drops for a reset word) |
| A+2   | new state / timer captured into the output register |
| A+3   | `m_axis_tvalid` high |

The accumulator may not start another close until the output register has
drained (`close_allowed`), so at most one decision is ever in flight.
Under S2MM backpressure nothing is lost or duplicated: closes stall, the
closing word stays parked, and `s_axis_tready` backpressures the DMA in
turn. Words that *don't* close a window keep flowing while a close waits.
`tb/tb_nlc_axis_top.sv` scenario D drives exactly this.

`m_axis_tlast` is set on the decision produced by an `s_axis_tlast` word.
One input DMA packet therefore always yields exactly one output DMA
packet: a single decision in closed-loop mode, or one decision per window
when a whole recorded episode is replayed as one packet.

### Event word format

```
EVENT  bit31=1   [30] polarity   [29:23] y (7b)   [22:16] x (7b)   [15:0] timestamp
CTRL   bit31=0   [30] reset      [29:16] 0                         [15:0] timestamp
```

- `x < 64` is the left hemisphere, `x >= 64` the right. y and polarity are
  carried but not used by the controller (a real DVS emits them; a later
  filter stage might want them).
- The timestamp is 16 bits; its upper 8 bits are the **window id**, so a
  window is 256 ticks and ids wrap every 256 windows.
- A CTRL word with `reset=0` is a sync/heartbeat: it carries a timestamp
  (so it can open or close a window) but counts nothing. The host sends one
  to close a window in which the sensor produced no events at all — an
  AXI DMA can't move an empty packet.
- A CTRL word with `reset=1` returns the FSM to `FORWARD`, clears the open
  window and produces a decision of its own (state 0, counts 0). This is how
  the host resets between episodes without reloading the bitstream, and it
  is testable under Icarus, unlike a GPIO.

### Decision word format

```
[31:24] window id   [23:22] ev_left   [21:20] ev_right   [19:18] FSM state
[17:15] brake timer [14] stale        [13:7] count_left  [6:0]  count_right
```

`sim/event_stream.py` is the Python twin of both formats;
`sim/test_event_stream.py` and the RTL testbenches pin them together.

## Windowed counting: `rtl/event_rate_window.sv`

Two 7-bit saturating counters, one per hemisphere, and a window-id
register. The input has a one-deep register stage so `s_axis_tready` never
depends combinationally on `s_axis_tdata`. A window closes when:

1. **a word's window id differs from the open one** (mid-stream boundary,
   replay mode). The word is *not* consumed on that cycle: the open window
   closes first, then the parked word is re-evaluated against `cur_id + 1`.
   If ids were skipped, that loop closes each missing id as an empty window,
   so the FSM still ticks once per elapsed window — a silent window is a
   real `(0, 0)` input that lets a turn recover or a brake count down.
2. **the word carries TLAST** (end of a DMA packet, closed-loop mode). The
   host wants a decision now, whatever the timestamps say. After a TLAST
   close no window is open; the next word opens one at its own id with no
   catch-up, so closed-loop steps may reuse or skip ids freely.
3. **the word is a CTRL reset.**

A word whose id is *behind* the open window (modular compare: the upper half
of the 8-bit id space counts as behind) is stale. It is dropped and the
window's `stale` flag is set, so out-of-order host bugs are visible rather
than silently re-counted 255 windows later.

Quantizer: `count >= 85 → 3`, `>= 55 → 2`, `>= 25 → 1`, else `0`. These
three thresholds are parameters (`RATE_T1..T3`), chosen so that the event
pipeline is decision-for-decision identical to the original rate-fed model
— see "Parity" below.

## FSM: `rtl/fsm_controller.sv`

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

The transition logic is unchanged from the original rate-fed project. Two
ports were added for the pipeline: `step_en` (the register update is gated
so the FSM advances exactly once per closed window rather than every fabric
clock — `BRAKE_CYCLES` therefore counts windows) and `dbg_brake_timer`
(observability for the decision word; Vivado won't synthesize a
hierarchical reference into the FSM).

**Known one-cycle flicker:** the countdown-expiry check
(`brake_timer == 0 -> FORWARD`) doesn't re-check `critical_front`. If the
obstacle is still critical exactly when the timer hits zero, the FSM spends
one step in `FORWARD` before `critical_front` sends it straight back into a
freshly-timed `BRAKE_S` the next step. This is intentional (keeping the
countdown-expiry case simple) and is covered by the "flicker re-entry" test
in `tb/tb_fsm_controller.sv`, not a bug to fix.

## Parity: the event pipeline reproduces the rate-fed controller exactly

The original project fed `sim/agent_sim.py::sense()` — a looming intensity
per hemisphere, bucketed in software at 0.25 / 0.55 / 0.85 — straight into
the FSM. The event camera instead emits `floor(intensity * 100)` events per
hemisphere per window, and the hardware quantizes the *count* at 25 / 55 /
85. `sim/test_event_stream.py::test_count_quantization_matches_rate_model`
checks the two agree across a 100 000-point sweep including the exact
boundaries, and `sim/test_parity.py` re-runs the default course and the
25-episode stress sweep through the RTL and asserts the trajectories and
summaries equal the CSVs captured from the pre-pipeline project
(`sim/golden/*_ratefed.csv`, never regenerated). So every result the old
project reported still holds, now with the rate computation in hardware.

Event *placement* (which pixels, which timestamps, which polarity) is
random per step from a seeded generator, but only counts matter to the
controller, so parity is independent of the camera seed. With
`--noise N` the camera adds Poisson background events; that intentionally
breaks parity and is the sweep's way of probing threshold robustness (the
golden-model cross-check stays on).

## Verification

| what | how | checks |
|---|---|---|
| FSM transitions, brake preemption, flicker, `step_en` gating | `tb/tb_fsm_controller.sv` (directed, self-checking, whitebox timer checks) | 53 |
| window counting, thresholds, saturation, catch-up, stale, reset, backpressure, id wrap | `tb/tb_event_rate_window.sv` | 28 |
| whole pipeline vs an independent FSM instance: directed vectors packet-per-step, 40-window single-packet replay under random `m_axis` backpressure, reset, stalled output | `tb/tb_nlc_axis_top.sv` | 82 |
| wire format, count/bucket parity, golden model vs directed vectors, replay == closed-loop, camera model | `sim/test_event_stream.py` (no simulator) | 16 |
| RTL under Icarus == rate-fed baseline CSVs; RTL == golden on random multi-window streams with gaps, stale words and resets; noisy camera | `sim/test_parity.py` | 5 |

`make unit-tb` runs the three testbenches; `make test` the pytest suites.
All of it runs in CI on every push.

### The golden model's role

`sim/golden_model.py` is a pure-Python mirror of the whole pipeline
(windowed counter with identical id/catch-up/stale/reset semantics, plus a
port of the FSM). It never produces the project's results. The
hardware-in-the-loop harness feeds it the same packet it sent to the
hardware and compares the two decisions on every step, so a divergence
between silicon, simulation and the documented behaviour shows up on the
step it happens rather than as an unexplained collision three episodes
later. On the board this is the equivalent of the crypto feed handler's
hw-vs-golden diff, just done live.

## Hardware-in-the-loop, two transports, one harness

`sim/engines.py` defines the contract every "chip" speaks:

```
engine.reset()        -> Decision    in-band reset, FSM back to FORWARD
engine.step(words)    -> Decision    one closed-loop control step
engine.stream(words)  -> [Decision]  one packet, many windows (replay)
```

`sim/cosim_driver.py` and `sim/stress_test.py` only ever talk to that.

### Icarus: `tb/tb_cosim_server.sv` + `IcarusEngine`

One `vvp` process per session. Python writes one line per AXI-Stream beat
(`<8 hex> <last>`) to its stdin; the testbench drives the beat into
`nlc_axis_top` with proper TVALID/TREADY handshaking and prints every
`m_axis` beat back as `RESULT word=… last=…`. After a TLAST beat it keeps
clocking until the decision carrying `m_axis_tlast` has been printed, then
blocks on the next line (simulation time only advances while stimulus
flows). FSM state and brake timer persist between packets in the DUT's own
registers, exactly as on the FPGA. There is no `force`/`release`, no state
save/restore, no testbench backdoor of any kind — the original per-step
`vvp` invocation with forced registers is gone.

### PYNQ-Z2: AXI DMA + `board/pynq/nlc_pynq.py`

`fpga/tcl/bd_nlc.tcl` builds the block design: `processing_system7` (board
preset, 100 MHz `FCLK_CLK0`, `S_AXI_HP0` enabled) → `axi_dma_0` in simple
mode with 32-bit streams → the pipeline instantiated as an RTL module
reference. IP Integrator refuses a SystemVerilog file as a module-reference
top (`[filemgmt 56-195]`), so the reference points at
`rtl/nlc_axis_top_wrap.v`, a plain Verilog shell that re-declares the ports
with their `X_INTERFACE_INFO` attributes (which let Vivado infer the two
AXI4-Stream interfaces) and instantiates `nlc_axis_top` unchanged. Per control step the driver arms S2MM for one
word, sends the packet over MM2S, waits for both channels (with a timeout,
since a malformed packet that never produces a TLAST decision would
otherwise hang `wait()` forever) and unpacks the word. Replay mode arms
S2MM for one word per window and sends the whole episode as one packet.
Same `Overlay` / `allocate` / `sendchannel` / `recvchannel` conventions as
`fpga-crypto-feed-handler/board/pynq/pynq_driver.py`. See
`docs/pynq_bringup.md`.

### Replay mode: `sim/replay_bench.py`

`cosim_driver.py --record` writes every step's packet concatenated into one
stream (TLAST only at the very end) plus the decisions the closed loop
produced. The bench streams it as a single packet and checks the hardware
emits the identical decision sequence, one per window, closed on timestamp
boundaries alone. `tb/tb_nlc_axis_top.sv` scenario B and
`test_golden_replay_equals_closed_loop` cover the same property in
simulation. On the board this is the pure-pipeline throughput number (DMA
in, DMA out, no Python in the loop per window); under Icarus it only proves
the mechanism.

### FPGA implementation results

Vivado 2024.1, `xc7z020clg400-1`, 100 MHz. Reports in `fpga/prebuilt/`.

| | LUTs | FFs | BRAM | timing |
|---|---|---|---|---|
| `nlc_axis_top` | 106 | 108 | 0 | 4.08 ns slack out of context (Fmax ≈ 169 MHz, post-synthesis estimate) |
| whole overlay | 2756 | 3618 | 2 | 1.25 ns slack post-route; worst path is in the AXI interconnect's width upsizer (DMA 32-bit → HP0 64-bit) |

The remaining warnings are the usual ones from the generated interconnect
and DMA IP (AXI ID width truncation on HP0, empty CDC waivers, unused
clock/reset ports on pass-through couplers); none originate in the
pipeline RTL. The prebuilt overlay was produced without the TUL board
files, so its PS7 block carries Vivado's default DDR/MIO configuration —
see `docs/pynq_bringup.md`.

## Noise robustness: `sim/noise_sweep.py`

Adds Poisson background events (mean λ per hemisphere per window) and runs
100 random courses per λ with the RTL in the loop, measuring the fraction
of windows whose bucket differs from the noise-free sensor model, the
fraction of clear-path windows spent turning or braking, and the collision
rate. The fixed thresholds tolerate λ ≤ 10 (under 3% of buckets change, no
collisions). From λ ≈ 20 noise alone reaches bucket 1, from ≈ 50 bucket 2
(false turns), and from ≈ 80 both sides reach bucket 3 together (false
brakes; the one-window brake-expiry flicker then lets the agent creep into
obstacles, 34% collisions at λ = 100). A background-activity filter ahead of
the counters is the natural next pipeline stage.

## Sensor model: `sim/event_camera.py`

Engineering shortcuts worth knowing about:

- Looming intensity is still `proximity²` of the strongest obstacle per
  hemisphere (`agent_sim.sense_readings`), not a rendered pixel array. The
  camera scatters `floor(intensity * 100)` events around that obstacle's
  bearing (projected to a column: bearing `+fov/2 → x=0`, `0 → x=64`,
  `-fov/2 → x=127`) with a spread matching its apparent angular size,
  random polarity, timestamps spread across the window in order. There is
  no per-pixel contrast threshold, refractory period or dark current.
- A single obstacle can only ever register on one hemisphere (bearing is
  strictly left-or-right of center), so a lone obstacle never triggers
  `BRAKE_S` by itself — braking only happens when the geometry puts
  meaningful looming on both sides simultaneously.
- The controller never re-centers heading after an avoidance turn; it just
  keeps going straight on the new heading. Obstacle placement in
  `sim/cosim_driver.py::build_default_environment` is tuned to sit on the
  agent's actual post-avoidance drift path so each one still gets a
  distinct reaction.

## Randomized verification: `sim/stress_test.py`

The unit testbenches prove individual transitions and window mechanics are
correct; they don't say much about whether the reactive controller actually
avoids obstacles in general. `stress_test.py` runs many independently-seeded
random obstacle courses through the same engine (RTL under Icarus, or the
FPGA with `--engine pynq`) and reports a collision rate plus how many
episodes triggered a turn or brake at all. Obstacle y-position is kept
close to the agent's nominal straight-line path deliberately. Each episode
is fully determined by its seed, so a failing seed can be replayed in
isolation. Brake engagements are rare in this sweep by construction (at
most one obstacle per x-band); that case is covered directly by the unit
tests instead.

## Web simulation

`web/` is unchanged: a React + Canvas visualization running a TypeScript
port of the FSM fed by the rate-fed sensor model, parity-tested against the
same directed vectors as `tb/tb_fsm_controller.sv`. Because the event
pipeline is decision-for-decision identical to the rate-fed model (see
"Parity"), the browser demo remains a faithful picture of what the FPGA
does, without needing the event stream client-side. The `FsmEngine`
interface still leaves room for a WASM build of the RTL later.

## Directory layout

```
rtl/       fsm_controller.sv, event_rate_window.sv, nlc_axis_top.sv  (synthesizable)
           nlc_axis_top_wrap.v (Verilog shell for IP Integrator)
tb/        three unit testbenches + the persistent cosim server
sim/       agent physics, event camera, wire format, golden model, engines,
           cosim driver, stress test, noise sweep, replay bench, pytest suites,
           golden CSVs
board/pynq/ PYNQ-Z2 engine (AXI DMA driver)
fpga/      Vivado block design + build/OOC-synthesis Tcl, XDC, prebuilt overlay
scripts/   iverilog/vvp wrappers, Parallels Vivado runner
docs/      this file, docs/pynq_bringup.md, figure generator + images
build/     compiled .vvp and .vcd (gitignored)
results/   CSV logs, plots, replay streams (gitignored)
web/       browser visualization (React + Vite + TS)
```
