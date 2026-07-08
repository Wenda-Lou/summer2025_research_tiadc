
cmake_minimum_required(VERSION 3.15)
project(bsp)

# Use CMAKE_LIBRARY_PATH in link_directories
link_directories(${CMAKE_LIBRARY_PATH})
        
set(DRIVER_TARGETS axidma;scugic;zdma;sysmonpsu;axipmon;canps;coresightps_dcc;resetps;clockps;csudma;ddrcpsu;dppsu;dpdma;emacps;gpiops;iicps;ipipsu;pciepsu;qspipsu;rtcpsu;sdps;spips;ttcps;uartps;usbpsu;wdtps)
set(DRIVER_LOCATIONS C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/axidma_v9_19;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/scugic_v5_5;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/zdma_v1_20;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/sysmonpsu_v3_1;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/axipmon_v6_12;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/canps_v3_10;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/coresightps_dcc_v1_12;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/resetps_v1_7;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/clockps_v1_6;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/csudma_v1_17;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/ddrcpsu_v1_8;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/dppsu_v1_7;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/dpdma_v1_6;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/emacps_v3_22;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/gpiops_v3_14;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/iicps_v3_21;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/ipipsu_v2_17;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/pciepsu_v2_1;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/qspipsu_v1_21;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/rtcpsu_v1_15;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/sdps_v4_4;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/spips_v3_12;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/ttcps_v3_21;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/uartps_v3_16;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/usbpsu_v1_17;C:/AMD/2025.1/Vitis/data/embeddedsw/XilinxProcessorIPLib/drivers/wdtps_v3_8)

list(LENGTH DRIVER_TARGETS no_of_drivers)
set(index 0)

while(${index} LESS ${no_of_drivers})
    list(GET DRIVER_TARGETS ${index} drv)
    list(GET DRIVER_LOCATIONS ${index} drv_dir)
    set(exp_dir "${drv_dir}/examples")
    add_custom_target(
        ${drv}_example ALL
        COMMAND lopper -f -O C:/TIDIAC/summer2025_research_tiadc/test_platform/dma_fix/psu_cortexa53_0/standalone_psu_cortexa53_0/bsp/libsrc/${drv} C:/TIDIAC/summer2025_research_tiadc/test_platform/dma_fix/psu_cortexa53_0/standalone_psu_cortexa53_0/bsp/hw_artifacts/psu_cortexa53_0_baremetal.dts -- bmcmake_metadata_xlnx psu_cortexa53_0 ${exp_dir} drvcmake_metadata
        BYPRODUCTS x${drv}_exlist.yaml)
    MATH(EXPR index "${index}+1")
endwhile()
