#!/usr/bin/env bash
# Build the cosim server and run a full closed-loop episode against the
# RTL under Icarus, writing results/trajectory_log.csv and
# results/trajectory.png. Extra args are forwarded to sim/cosim_driver.py,
# e.g. ./scripts/run_cosim.sh --engine pynq (on the board).
set -euo pipefail
cd "$(dirname "$0")/.."

./scripts/build_cosim.sh
python3 sim/cosim_driver.py "$@"
