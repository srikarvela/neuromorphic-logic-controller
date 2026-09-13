#!/usr/bin/env bash
# Build the cosim server and run the randomized multi-episode sweep.
# Extra args are forwarded to sim/stress_test.py, e.g.:
#   ./scripts/run_stress_test.sh --episodes 100 --seed 42
#   ./scripts/run_stress_test.sh --engine pynq --noise 5   (on the board)
set -euo pipefail
cd "$(dirname "$0")/.."

./scripts/build_cosim.sh
python3 sim/stress_test.py "$@"
