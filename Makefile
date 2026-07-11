.PHONY: unit-tb build-cosim cosim stress-test clean

unit-tb:
	./scripts/run_unit_tb.sh

build-cosim:
	./scripts/build_cosim.sh

cosim: build-cosim
	python3 sim/cosim_driver.py

stress-test: build-cosim
	python3 sim/stress_test.py

clean:
	rm -rf build/*.vvp build/*.vcd results/*.csv results/*.png
