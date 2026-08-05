set project_path "C:/thesis_v3/thesis_v3.xpr"
set source_path "C:/thesis_v3/thesis_v3.srcs/sources_1/new/adc_channel_skew_actuator_gpio.v"

open_project $project_path
set existing_source [get_files -quiet $source_path]
if {[llength $existing_source] != 0} {
    remove_files $existing_source
}
add_files -norecurse -fileset sources_1 $source_path
update_compile_order -fileset sources_1
open_bd_design "C:/thesis_v3/thesis_v3.srcs/sources_1/bd/system/system.bd"
set_property CONFIG.FIFO_DATA_WIDTH 128 [get_bd_cells greedy_cup]

if {[llength [get_bd_cells -quiet adc_skew_actuator]] == 0} {
    create_bd_cell -type module -reference adc_channel_skew_actuator_gpio adc_skew_actuator
}
if {[llength [get_bd_cells -quiet skew_delay_gpio]] == 0} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:axi_gpio:2.0 skew_delay_gpio
}
set_property -dict [list \
    CONFIG.C_GPIO_WIDTH {10} \
    CONFIG.C_ALL_OUTPUTS {1} \
    CONFIG.C_DOUT_DEFAULT {0x00000100} \
    CONFIG.C_IS_DUAL {0}] [get_bd_cells skew_delay_gpio]

# Replace the direct TPL-to-capture connection with the skew actuator.
set old_net [get_bd_intf_nets -quiet -of_objects \
    [get_bd_intf_pins jesd204_tpl_core_0/m_axis]]
if {[llength $old_net] != 0 && \
    [llength [get_bd_intf_pins -quiet greedy_cup/s_axis]] != 0} {
    disconnect_bd_intf_net $old_net [get_bd_intf_pins greedy_cup/s_axis]
}
connect_bd_intf_net [get_bd_intf_pins jesd204_tpl_core_0/m_axis] \
    [get_bd_intf_pins adc_skew_actuator/s_axis]
connect_bd_intf_net [get_bd_intf_pins adc_skew_actuator/m_axis] \
    [get_bd_intf_pins greedy_cup/s_axis]

connect_bd_net [get_bd_ports rx_core_clk] \
    [get_bd_pins adc_skew_actuator/link_clk]
connect_bd_net [get_bd_pins jesd204_link/rx_aresetn] \
    [get_bd_pins adc_skew_actuator/link_resetn]
connect_bd_net [get_bd_pins skew_delay_gpio/gpio_io_o] \
    [get_bd_pins adc_skew_actuator/delay_q8_async]
connect_bd_net [get_bd_pins zynq_ultra_ps_e_0/pl_clk0] \
    [get_bd_pins skew_delay_gpio/s_axi_aclk]
connect_bd_net [get_bd_pins rst_ps8_0_99M/peripheral_aresetn] \
    [get_bd_pins skew_delay_gpio/s_axi_aresetn]

set_property CONFIG.NUM_MI 4 [get_bd_cells axi_smc_ctrl]
connect_bd_intf_net [get_bd_intf_pins axi_smc_ctrl/M03_AXI] \
    [get_bd_intf_pins skew_delay_gpio/S_AXI]

assign_bd_address -offset 0xA0030000 -range 64K \
    -target_address_space [get_bd_addr_spaces zynq_ultra_ps_e_0/Data] \
    [get_bd_addr_segs skew_delay_gpio/S_AXI/Reg] -force

validate_bd_design
save_bd_design
generate_target all [get_files system.bd]
update_compile_order -fileset sources_1
close_project
