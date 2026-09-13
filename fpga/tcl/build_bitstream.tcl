# Vivado batch flow: project -> block design -> synthesis -> implementation
# -> bitstream + hardware handoff for PYNQ.
#
#   vivado -mode batch -source fpga/tcl/build_bitstream.tcl        (from repo root)
#   make bitstream
#
# Outputs (fpga/build/, gitignored):
#   nlc.bit, nlc.hwh          copy both to the board next to board/pynq/nlc_pynq.py
#   timing_summary.rpt, utilization.rpt
#
# Tested target: PYNQ-Z2 (xc7z020clg400-1). Vivado 2022.2+ with the TUL
# PYNQ-Z2 board files installed; drop the board_part line if you don't
# have them (the PS7 preset is then applied from PCW defaults instead).

set root      [file normalize [file join [file dirname [info script]] .. ..]]
set proj_name "nlc_pynq"
set proj_dir  "$root/fpga/vivado/$proj_name"
set out_dir   "$root/fpga/build"
set part      "xc7z020clg400-1"
set board     "tul.com.tw:pynq-z2:part0:1.0"
set bd_name   "nlc_bd"

file mkdir $out_dir

create_project $proj_name $proj_dir -part $part -force
if {[catch {set_property board_part $board [current_project]} msg]} {
    puts "WARNING: PYNQ-Z2 board files not found ($msg); continuing with bare part $part"
}

# RTL: the same three files the Icarus testbenches use, nothing else.
add_files -norecurse [list \
    "$root/rtl/fsm_controller.sv" \
    "$root/rtl/event_rate_window.sv" \
    "$root/rtl/nlc_axis_top.sv" \
]
set_property file_type SystemVerilog [get_files *.sv]
add_files -fileset constrs_1 -norecurse "$root/fpga/constraints/pynq_z2.xdc"
update_compile_order -fileset sources_1

create_bd_design $bd_name
source "$root/fpga/tcl/bd_nlc.tcl"
validate_bd_design
save_bd_design

make_wrapper -files [get_files $bd_name.bd] -top
add_files -norecurse [glob $proj_dir/$proj_name.gen/sources_1/bd/$bd_name/hdl/${bd_name}_wrapper.v]
set_property top ${bd_name}_wrapper [current_fileset]
update_compile_order -fileset sources_1

launch_runs synth_1 -jobs 8
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] != "100%"} { error "synthesis failed" }

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} { error "implementation failed" }

open_run impl_1
report_timing_summary -file "$out_dir/timing_summary.rpt"
report_utilization -hierarchical -file "$out_dir/utilization.rpt"
set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "Worst setup slack: $wns ns at 100 MHz"

file copy -force "$proj_dir/$proj_name.runs/impl_1/${bd_name}_wrapper.bit" "$out_dir/nlc.bit"
file copy -force [glob $proj_dir/$proj_name.gen/sources_1/bd/$bd_name/hw_handoff/$bd_name.hwh] "$out_dir/nlc.hwh"
puts "=== Wrote $out_dir/nlc.bit and $out_dir/nlc.hwh ==="
