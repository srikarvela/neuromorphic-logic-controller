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
#   make noise-sweep    sensor-noise robustness
#   make figures        README charts              make vm-bitstream Vivado in a Parallels VM (macOS)
#
# BOARD_HOST / BOARD_DIR select the PYNQ target; VIVADO the Vivado binary.
# board-deploy ships fpga/build/nlc.bit if present, else fpga/prebuilt/.

.PHONY: unit-tb build-cosim cosim stress-test replay-bench test check noise-sweep figures \
        synth-ooc bitstream vm-ooc vm-bitstream \
        board-deploy board-cosim board-stress board-bench board-smoke clean

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

noise-sweep: build-cosim
	$(PYTHON) sim/noise_sweep.py --episodes 100

figures: build-cosim
	@test -f results/noise_sweep.csv || $(PYTHON) sim/noise_sweep.py --episodes 100
	$(PYTHON) docs/gen_figures.py

# ── FPGA (needs Vivado; PYNQ-Z2 board files optional) ─────────────────────────

synth-ooc:
	$(VIVADO) -mode batch -nolog -nojournal -source fpga/tcl/synth_ooc.tcl
	@echo "=== fpga/build/ooc_utilization.rpt, fpga/build/ooc_timing.rpt ==="

bitstream:
	$(VIVADO) -mode batch -nolog -nojournal -source fpga/tcl/build_bitstream.tcl
	@echo "=== fpga/build/nlc.bit + nlc.hwh; timing_summary.rpt, utilization.rpt ==="

# Apple Silicon: run the same Tcl inside a Parallels Windows VM (builds git HEAD).
vm-ooc:
	./scripts/vivado_in_parallels.sh ooc

vm-bitstream:
	./scripts/vivado_in_parallels.sh bitstream

# ── Board (PYNQ-Z2 over ssh) ──────────────────────────────────────────────────

OVERLAY_DIR = $(if $(wildcard fpga/build/nlc.bit),fpga/build,fpga/prebuilt)

board-deploy:
	@test -f $(OVERLAY_DIR)/nlc.bit -a -f $(OVERLAY_DIR)/nlc.hwh || \
	  (echo "no overlay found -- run make bitstream / make vm-bitstream"; exit 1)
	@echo "Deploying overlay from $(OVERLAY_DIR)/"
	rsync -az --delete --exclude web --exclude build --exclude results --exclude fpga/vivado \
	  --exclude '.git' --exclude '__pycache__' ./ $(BOARD_HOST):$(BOARD_DIR)/
	rsync -az $(OVERLAY_DIR)/nlc.bit $(OVERLAY_DIR)/nlc.hwh $(BOARD_HOST):$(BOARD_DIR)/board/pynq/

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
