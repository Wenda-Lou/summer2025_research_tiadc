open_project "C:/thesis_v3/thesis_v3.xpr"
open_bd_design "C:/thesis_v3/thesis_v3.srcs/sources_1/bd/system/system.bd"
set_property CONFIG.FIFO_DATA_WIDTH 128 [get_bd_cells greedy_cup]
validate_bd_design
save_bd_design
generate_target all [get_files system.bd]
update_compile_order -fileset sources_1
reset_run synth_1
launch_runs synth_1 -jobs 4
wait_on_run synth_1
set synth_status [get_property STATUS [get_runs synth_1]]
puts "SKEW_ACTUATOR_SYNTH_STATUS=$synth_status"
if {![string match "*Complete*" $synth_status]} {
    error "Top-level synthesis did not complete"
}
close_project
