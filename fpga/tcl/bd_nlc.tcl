# Block design: Zynq PS  <-- AXI DMA -->  nlc_axis_top  (module reference)
# Sourced from build_bitstream.tcl after create_bd_design.
#
#   ps7.M_AXI_GP0  --> axi_dma_0.S_AXI_LITE            (control registers)
#   axi_dma_0.M_AXI_MM2S / M_AXI_S2MM --> ps7.S_AXI_HP0 (DDR access)
#   axi_dma_0.M_AXIS_MM2S --> nlc_0.s_axis              (event words)
#   nlc_0.m_axis --> axi_dma_0.S_AXIS_S2MM              (decision words)

set bd_name [current_bd_design]

# --- Processing system with the PYNQ-Z2 board preset, 100 MHz fabric clock,
#     one high-performance slave port for the DMA
create_bd_cell -type ip -vlnv xilinx.com:ip:processing_system7:5.5 ps7
apply_bd_automation -rule xilinx.com:bd_rule:processing_system7 \
    -config {make_external "FIXED_IO, DDR" apply_board_preset "1" Master "Disable" Slave "Disable"} \
    [get_bd_cells ps7]
set_property -dict [list \
    CONFIG.PCW_USE_M_AXI_GP0 {1} \
    CONFIG.PCW_USE_S_AXI_HP0 {1} \
    CONFIG.PCW_S_AXI_HP0_DATA_WIDTH {64} \
    CONFIG.PCW_FPGA0_PERIPHERAL_FREQMHZ {100} \
    CONFIG.PCW_EN_CLK0_PORT {1} \
    CONFIG.PCW_EN_RST0_PORT {1} \
] [get_bd_cells ps7]

# --- AXI DMA, simple (non-scatter-gather) mode, 32-bit streams both ways
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_dma:7.1 axi_dma_0
set_property -dict [list \
    CONFIG.c_include_sg {0} \
    CONFIG.c_sg_include_stscntrl_strm {0} \
    CONFIG.c_include_mm2s {1} \
    CONFIG.c_include_s2mm {1} \
    CONFIG.c_m_axis_mm2s_tdata_width {32} \
    CONFIG.c_s_axis_s2mm_tdata_width {32} \
    CONFIG.c_m_axi_mm2s_data_width {32} \
    CONFIG.c_m_axi_s2mm_data_width {32} \
    CONFIG.c_mm2s_burst_size {16} \
    CONFIG.c_s2mm_burst_size {16} \
    CONFIG.c_sg_length_width {23} \
] [get_bd_cells axi_dma_0]

# --- The event pipeline itself, instantiated straight from rtl/nlc_axis_top.sv.
#     Vivado infers the s_axis/m_axis AXI4-Stream interfaces from the
#     X_INTERFACE_INFO attributes on its ports.
create_bd_cell -type module -reference nlc_axis_top nlc_0

# --- Streams
connect_bd_intf_net [get_bd_intf_pins axi_dma_0/M_AXIS_MM2S] [get_bd_intf_pins nlc_0/s_axis]
connect_bd_intf_net [get_bd_intf_pins nlc_0/m_axis]          [get_bd_intf_pins axi_dma_0/S_AXIS_S2MM]

# --- Memory-mapped side: let the automation add the interconnects and the
#     processor system reset block, all on FCLK_CLK0
apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
    -config {Clk_master {/ps7/FCLK_CLK0 (100 MHz)} Clk_slave {Auto} Clk_xbar {Auto} \
             Master {/ps7/M_AXI_GP0} Slave {/axi_dma_0/S_AXI_LITE} intc_ip {New AXI Interconnect} master_apm {0}} \
    [get_bd_intf_pins axi_dma_0/S_AXI_LITE]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
    -config {Clk_master {Auto} Clk_slave {/ps7/FCLK_CLK0 (100 MHz)} Clk_xbar {Auto} \
             Master {/axi_dma_0/M_AXI_MM2S} Slave {/ps7/S_AXI_HP0} intc_ip {New AXI Interconnect} master_apm {0}} \
    [get_bd_intf_pins ps7/S_AXI_HP0]
apply_bd_automation -rule xilinx.com:bd_rule:axi4 \
    -config {Clk_master {Auto} Clk_slave {/ps7/FCLK_CLK0 (100 MHz)} Clk_xbar {Auto} \
             Master {/axi_dma_0/M_AXI_S2MM} Slave {/ps7/S_AXI_HP0} intc_ip {/axi_mem_intercon} master_apm {0}} \
    [get_bd_intf_pins axi_dma_0/M_AXI_S2MM]

# --- Clock and reset for the pipeline: same fabric clock, the automation's
#     peripheral (active-low, synchronous) reset
connect_bd_net [get_bd_pins ps7/FCLK_CLK0] [get_bd_pins nlc_0/aclk]
set psr [get_bd_cells -hierarchical -filter {VLNV =~ "xilinx.com:ip:proc_sys_reset:*"}]
if {[llength $psr] == 0} { error "no proc_sys_reset block was created by the automation" }
connect_bd_net [get_bd_pins [lindex $psr 0]/peripheral_aresetn] [get_bd_pins nlc_0/aresetn]

assign_bd_address
regenerate_bd_layout
puts "Block design $bd_name wired: ps7 <-> axi_dma_0 <-> nlc_0"
