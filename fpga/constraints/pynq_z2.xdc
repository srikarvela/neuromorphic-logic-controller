# PYNQ-Z2 (XC7Z020-1CLG400C) constraints for the NLC event pipeline.
#
# The whole design is clocked from the PS (FCLK_CLK0, 100 MHz, configured
# on the processing_system7 block in fpga/tcl/bd_nlc.tcl); the block design
# generates the clock constraint, so nothing is needed here. No fabric pins
# are used: events arrive over AXI DMA from DDR, decisions leave the same way.
#
# Uncomment to bring a couple of state bits out on PMODA for a scope, after
# adding a matching output port to the block-design wrapper:
# set_property PACKAGE_PIN Y18 [get_ports {dbg_state[0]}]
# set_property PACKAGE_PIN Y19 [get_ports {dbg_state[1]}]
# set_property IOSTANDARD LVCMOS33 [get_ports {dbg_state[*]}]
