#!/usr/bin/env bash
# Compile the single-step cosim testbench used by sim/cosim_driver.py.
set -euo pipefail
cd "$(dirname "$0")/.."

mkdir -p build
iverilog -g2012 -o build/tb_cosim_step.vvp rtl/fsm_controller.sv tb/tb_cosim_step.sv
echo "Built build/tb_cosim_step.vvp"
