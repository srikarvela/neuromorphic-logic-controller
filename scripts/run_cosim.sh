#!/usr/bin/env bash
# Build the cosim testbench and run a full closed-loop episode, writing
# results/trajectory_log.csv and results/trajectory.png.
set -euo pipefail
cd "$(dirname "$0")/.."

./scripts/build_cosim.sh
python3 sim/cosim_driver.py
