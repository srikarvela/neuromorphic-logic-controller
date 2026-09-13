# Neuromorphic Logic Controller: FPGA event-stream pipeline with
# hardware-in-the-loop validation.
#
#   Simulation (Icarus Verilog, this machine)      Silicon (PYNQ-Z2, over ssh)
#   ---------------------------------------------  -----------------------------
#   make unit-tb        all RTL unit testbenches   make bitstream    Vivado -> nlc.bit/.hwh
#   make cosim          closed-loop episode        make board-deploy copy repo + bitstream
#   make stress-test    randomized sweep           make board-cosim  same episode on the FPGA
#   make replay-bench   one-packet replay          make board-stress same sweep on the FPGA
#   make test           pytest (parity + models)   make board-bench  DMA replay throughput
#
# BOARD_HOST / BOARD_DIR select the PYNQ target; VIVADO the Vivado binary.

.PHONY: unit-tb build-cosim cosim stress-test replay-bench test check \
        synth-ooc bitstream board-deploy board-cosim board-stress board-bench board-smoke clean

PYTHON     ?= python3
VIVADO     ?= vivado
BOARD_HOST ?= xilinx@pynq
BOARD_DIR  ?= ~/nlc

# ── Simulation ────────────────────────────────────────────────────────────────

unit-tb:
	./scripts/run_unit_tb.sh

build-cosim:
	./scripts/build_cosim.sh

cosim: build-cosim
	$(PYTHON) sim/cosim_driver.py --record results/replay

stress-test: build-cosim
	$(PYTHON) sim/stress_test.py

replay-bench: cosim
	$(PYTHON) sim/replay_bench.py --repeat 3

test: build-cosim
	$(PYTHON) -m pytest sim -q

check: unit-tb test cosim stress-test replay-bench

# ── FPGA (needs Vivado; PYNQ-Z2 board files optional) ─────────────────────────

synth-ooc:
	$(VIVADO) -mode batch -nolog -nojournal -source fpga/tcl/synth_ooc.tcl
	@echo "=== fpga/build/ooc_utilization.rpt, fpga/build/ooc_timing.rpt ==="

bitstream:
	$(VIVADO) -mode batch -nolog -nojournal -source fpga/tcl/build_bitstream.tcl
	@echo "=== fpga/build/nlc.bit + nlc.hwh; timing_summary.rpt, utilization.rpt ==="

# ── Board (PYNQ-Z2 over ssh) ──────────────────────────────────────────────────

board-deploy:
	@test -f fpga/build/nlc.bit -a -f fpga/build/nlc.hwh || \
	  (echo "fpga/build/nlc.bit/.hwh missing -- run make bitstream first"; exit 1)
	rsync -az --delete --exclude web --exclude build --exclude results --exclude fpga/vivado \
	  --exclude '.git' --exclude '__pycache__' ./ $(BOARD_HOST):$(BOARD_DIR)/
	rsync -az fpga/build/nlc.bit fpga/build/nlc.hwh $(BOARD_HOST):$(BOARD_DIR)/board/pynq/

board-smoke:
	ssh $(BOARD_HOST) "cd $(BOARD_DIR) && sudo -E python3 board/pynq/nlc_pynq.py"

board-cosim:
	ssh $(BOARD_HOST) "cd $(BOARD_DIR) && sudo -E python3 sim/cosim_driver.py --engine pynq --record results/replay"
	rsync -az $(BOARD_HOST):$(BOARD_DIR)/results/ results/board/

board-stress:
	ssh $(BOARD_HOST) "cd $(BOARD_DIR) && sudo -E python3 sim/stress_test.py --engine pynq --episodes 25"
	rsync -az $(BOARD_HOST):$(BOARD_DIR)/results/ results/board/

board-bench:
	ssh $(BOARD_HOST) "cd $(BOARD_DIR) && sudo -E python3 sim/replay_bench.py --engine pynq --repeat 20"

# ── Clean ─────────────────────────────────────────────────────────────────────

clean:
	rm -rf build/*.vvp build/*.vcd results/*.csv results/*.png results/*.bin results/board
	rm -rf fpga/vivado fpga/build sim/__pycache__ sim/.pytest_cache .pytest_cache
