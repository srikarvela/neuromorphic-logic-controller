#!/usr/bin/env bash
# Compile the persistent hardware-in-the-loop cosim server used by
# sim/engines.py::IcarusEngine. It wraps the full AXI-Stream pipeline
# (rtl/nlc_axis_top.sv), the same RTL that is synthesized for the PYNQ-Z2.
set -euo pipefail
cd "$(dirname "$0")/.."

mkdir -p build
iverilog -g2012 -Wall -o build/tb_cosim_server.vvp \
  rtl/fsm_controller.sv rtl/event_rate_window.sv rtl/nlc_axis_top.sv \
  tb/tb_cosim_server.sv
echo "Built build/tb_cosim_server.vvp"
