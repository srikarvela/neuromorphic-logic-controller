#!/usr/bin/env bash
# Compile and run the self-checking FSM unit testbench under Icarus Verilog.
set -euo pipefail
cd "$(dirname "$0")/.."

mkdir -p build
iverilog -g2012 -o build/tb_fsm_controller.vvp rtl/fsm_controller.sv tb/tb_fsm_controller.sv
vvp build/tb_fsm_controller.vvp
