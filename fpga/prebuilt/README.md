# Prebuilt PYNQ-Z2 overlay

Built from this repository with Vivado 2024.1 (Windows 11 on ARM, x86
emulation, Parallels) using `fpga/tcl/build_bitstream.tcl`, so the board can
be brought up without a Vivado install. `make board-deploy` uses these files
when `fpga/build/` has no fresher build.

| file | what |
|---|---|
| `nlc.bit` / `nlc.hwh` | overlay: PS7 + AXI DMA + `nlc_axis_top` (load both together) |
| `timing_summary.rpt` / `utilization.rpt` | full design, post-implementation |
| `ooc_timing.rpt` / `ooc_utilization.rpt` | `nlc_axis_top` alone, out-of-context synthesis |
| `SHA256SUMS` | checksums of the overlay files |

Summary:

| | LUTs | FFs | BRAM | worst setup slack @ 100 MHz |
|---|---|---|---|---|
| `nlc_axis_top` (event pipeline) | 106 | 108 | 0 | 4.08 ns OOC (Fmax ≈ 169 MHz) |
| whole overlay (with DMA + interconnect) | 2756 | 3618 | 2 × 36K | 1.25 ns (critical path inside the AXI interconnect) |

**Built without the TUL PYNQ-Z2 board files**, so the PS7 block carries
Vivado's default DDR/MIO settings rather than the board preset. On PYNQ the
processing system is configured at boot by the PYNQ image, and loading an
overlay only reprograms the fabric and applies the `.hwh` clock settings
(FCLK0 = 100 MHz here), so this is expected to be harmless — but rebuild with
the board files installed before treating this overlay as final. See
`docs/pynq_bringup.md`.
