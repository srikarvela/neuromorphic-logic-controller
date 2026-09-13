# Quick out-of-context synthesis of nlc_axis_top alone: utilization and
# Fmax for the pipeline itself, without the PS/DMA wrapper. Minutes, not
# tens of minutes.
#
#   vivado -mode batch -source fpga/tcl/synth_ooc.tcl     (from repo root)
#   make synth-ooc
#
# Writes fpga/build/ooc_utilization.rpt and fpga/build/ooc_timing.rpt.

set root    [file normalize [file join [file dirname [info script]] .. ..]]
set out_dir "$root/fpga/build"
set part    "xc7z020clg400-1"
set period_ns 10.0   ;# 100 MHz, the PS fabric clock used on the board

file mkdir $out_dir
read_verilog -sv [list \
    "$root/rtl/fsm_controller.sv" \
    "$root/rtl/event_rate_window.sv" \
    "$root/rtl/nlc_axis_top.sv" \
]
synth_design -top nlc_axis_top -part $part -mode out_of_context
create_clock -name aclk -period $period_ns [get_ports aclk]
report_utilization -file "$out_dir/ooc_utilization.rpt"
report_timing_summary -file "$out_dir/ooc_timing.rpt"
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts [format "nlc_axis_top OOC: worst setup slack %.3f ns at %.0f MHz -> Fmax ~ %.0f MHz" \
      $wns [expr {1000.0 / $period_ns}] [expr {1000.0 / ($period_ns - $wns)}]]
