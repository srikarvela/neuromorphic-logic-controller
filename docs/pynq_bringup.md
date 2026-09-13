# PYNQ-Z2 bring-up

Everything below the "Board" line has not yet been run from this checkout:
the Vivado flow and the DMA driver follow the same conventions as the
crypto feed handler's, but the first `make bitstream` / `make board-smoke`
is still to do. Treat this as the checklist for that session and fill in
the numbers in the README when it's done.

## Prerequisites

- Vivado 2022.2 or newer (Vitis HLS not needed — the pipeline is plain RTL).
- TUL PYNQ-Z2 board files installed in Vivado (`tul.com.tw:pynq-z2:part0:1.0`).
  Without them `build_bitstream.tcl` warns and continues with the bare
  `xc7z020clg400-1`; the PS7 then needs the board's DDR/MIO preset applied
  by hand in the block design, so install the files.
- A PYNQ-Z2 running the PYNQ image (v2.7 or v3.x), reachable over ssh as
  `xilinx@pynq` (override with `BOARD_HOST=...` on every `make board-*`).

## 1. Bitstream

```bash
make synth-ooc    # optional, ~2 min: utilization + Fmax of nlc_axis_top alone
make bitstream    # full PS + DMA + pipeline design, ~10-15 min
```

`make bitstream` leaves `fpga/build/nlc.bit`, `fpga/build/nlc.hwh`,
`timing_summary.rpt` and `utilization.rpt`. The pipeline is a few hundred
LUTs and runs on the 100 MHz PS fabric clock; timing should close with
lots of margin — check `Worst setup slack` in the Vivado log.

What the block design contains (`fpga/tcl/bd_nlc.tcl`):

| block | config |
|---|---|
| `ps7` | PYNQ-Z2 preset, `FCLK_CLK0` = 100 MHz, `M_AXI_GP0` (DMA registers), `S_AXI_HP0` 64-bit (DMA data) |
| `axi_dma_0` | simple mode (no scatter-gather), MM2S + S2MM, 32-bit streams, 23-bit length register (8 MB max transfer) |
| `nlc_0` | `rtl/nlc_axis_top.sv` as a module reference; `s_axis` ← MM2S, `m_axis` → S2MM |
| automation | AXI interconnects and a `proc_sys_reset`; `nlc_0/aresetn` = its `peripheral_aresetn` |

## 2. Deploy

```bash
make board-deploy     # rsync repo (minus web/, build/, results/) + nlc.bit/.hwh to ~/nlc on the board
```

## 3. Smoke test, then the real thing

```bash
make board-smoke      # reset, 30 left events -> bucket 1 / FORWARD, 90 -> bucket 3 / TURN_R
make board-cosim      # the default 45-step course, FPGA in the loop; pulls results/board/
make board-stress     # 25 seeded episodes on the FPGA, golden-checked every step
make board-bench      # single-packet replay of the recorded episode, 20 passes, us/window
```

Expected outcomes, because the board runs the same RTL Icarus does:

- `board-cosim` prints `Every decision matched the golden model and the
  rate-fed sensor model`, and `results/board/trajectory_log.csv` is
  byte-identical to `sim/golden/trajectory_default_ratefed.csv` in the
  `ev_left, ev_right, state, brake_timer, x, y, heading_deg` columns.
- `board-stress` reports the same 0/25 collisions and per-seed step/turn/
  brake counts as `sim/golden/stress_seed0_25_ratefed.csv`.
- `board-bench` reports `0 mismatches` and the FPGA's decisions/s.

The scripts run with `sudo -E` because PYNQ's `Overlay`/`allocate` need
`/dev/mem` and the CMA allocator. Python packages needed on the board are
only `pynq` and `numpy` (already on the image); matplotlib is only imported
when a plot is written, so `cosim_driver.py` works without it.

## Troubleshooting

- **`TimeoutError: DMA S2MM did not complete`** — the pipeline never
  produced a TLAST decision. Usually the packet's last word didn't set
  TLAST (the driver always does), or the S2MM buffer was armed for fewer
  words than the hardware produced (replay of a stream with a gap in window
  ids: the hardware closes the skipped ids as empty windows, one decision
  each — `PynqEngine.stream` sizes the receive buffer by distinct ids, so
  keep the recorded stream gap-free, which `EventCamera` guarantees).
- **`need nlc.bit and nlc.hwh`** — PYNQ loads the `.hwh` next to the
  `.bit` with the same stem; `make board-deploy` copies both.
- **DMA error status** — check `axi_dma_0` is in simple mode and the S2MM
  buffer is 32-bit aligned (it is: `allocate(dtype=uint32)`). A non-zero
  `stale` bit in decisions means a word arrived with a timestamp behind the
  open window; the driver's timestamps are monotonic per episode, so that
  points at a corrupted packet.
- **Wrong decisions but no DMA error** — run `make board-smoke` first; then
  `sim/replay_bench.py --engine pynq` against a stream recorded under
  Icarus. A mismatch on the first window is a wire-format problem
  (endianness: the DMA moves little-endian u32, matching
  `event_stream.words_to_bytes`); a mismatch later is a state-persistence
  one (was the FSM reset between packets? `PynqEngine.reset()` sends the
  in-band reset word).
- **Bitstream builds but PYNQ can't find `axi_dma_0`** — the `.hwh` is
  from a different build than the `.bit`; re-run `make board-deploy`.

## Numbers to fill in after the first board run

In `README.md` under "Results":

- Fmax / worst slack and LUT/FF count for `nlc_axis_top` (from
  `fpga/build/ooc_timing.rpt`, `ooc_utilization.rpt`), and for the whole
  design (`timing_summary.rpt`, `utilization.rpt`).
- `make board-bench`: words/s, decisions/s and us/window through the DMA.
- `make board-cosim` / `board-stress`: the parity statements above, confirmed.
