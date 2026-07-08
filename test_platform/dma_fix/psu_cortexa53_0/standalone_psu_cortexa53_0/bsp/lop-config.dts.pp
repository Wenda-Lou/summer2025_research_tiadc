# 1 "C:/TIDIAC/summer2025_research_tiadc/test_platform/dma_fix/psu_cortexa53_0/standalone_psu_cortexa53_0/bsp/lop-config.dts"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "C:/TIDIAC/summer2025_research_tiadc/test_platform/dma_fix/psu_cortexa53_0/standalone_psu_cortexa53_0/bsp/lop-config.dts"

/dts-v1/;
/ {
        compatible = "system-device-tree-v1,lop";
        lops {
                lop_0 {
                        compatible = "system-device-tree-v1,lop,load";
                        load = "assists/baremetal_validate_comp_xlnx.py";
                };

                lop_1 {
                    compatible = "system-device-tree-v1,lop,assist-v1";
                    node = "/";
                    outdir = "C:/TIDIAC/summer2025_research_tiadc/test_platform/dma_fix/psu_cortexa53_0/standalone_psu_cortexa53_0/bsp";
                    id = "module,baremetal_validate_comp_xlnx";
                    options = "psu_cortexa53_0 C:/AMD/2025.1/Vitis/data/embeddedsw/ThirdParty/sw_services/lwip220_v1_2/src C:/TIDIAC/summer2025_research_tiadc/test_platform/_ide/.wsdata/.repo.yaml";
                };

        };
    };
