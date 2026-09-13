#!/usr/bin/env bash
# Compile and run every self-checking unit testbench under Icarus Verilog.
# Each testbench prints "ALL N CHECKS PASSED" on success; any other outcome
# (a FAIL line, a compile error, a missing summary) fails this script.
set -euo pipefail
cd "$(dirname "$0")/.."

mkdir -p build
RTL="rtl/fsm_controller.sv rtl/event_rate_window.sv rtl/nlc_axis_top.sv"
status=0

run_tb() {
  local tb="$1"
  echo "=================================================================="
  echo "== $tb"
  echo "=================================================================="
  iverilog -g2012 -Wall -o "build/$tb.vvp" $RTL "tb/$tb.sv"
  local out
  out="$(vvp -n "build/$tb.vvp")"
  echo "$out" | grep -v '^\[PASS\]' || true
  if ! echo "$out" | grep -q 'ALL [0-9]* CHECKS PASSED'; then
    echo "!! $tb FAILED"
    status=1
  fi
}

run_tb tb_fsm_controller
run_tb tb_event_rate_window
run_tb tb_nlc_axis_top

echo "=================================================================="
if [ "$status" -eq 0 ]; then
  echo "All unit testbenches passed."
else
  echo "Unit testbench failures above."
fi
exit "$status"
