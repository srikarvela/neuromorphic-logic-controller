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
```

`make cosim` writes `results/trajectory_log.csv` and `results/trajectory.png`
— the agent's path through a small obstacle field, driven step-by-step by
the SystemVerilog FSM.

## Layout

```
rtl/     fsm_controller.sv — the synthesizable FSM "brain"
tb/      unit testbench + per-step hardware-in-the-loop testbench
sim/     Python agent/environment model and the cosim driver
scripts/ shell wrappers around iverilog/vvp
docs/    architecture notes
```
