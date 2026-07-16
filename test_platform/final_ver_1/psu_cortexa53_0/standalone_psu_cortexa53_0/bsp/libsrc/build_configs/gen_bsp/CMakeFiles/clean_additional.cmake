# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "D:\\TIADC\\summer2025_research_tiadc\\test_platform\\final_ver_1\\psu_cortexa53_0\\standalone_psu_cortexa53_0\\bsp\\include\\lwipopts.h"
  "D:\\TIADC\\summer2025_research_tiadc\\test_platform\\final_ver_1\\psu_cortexa53_0\\standalone_psu_cortexa53_0\\bsp\\include\\sleep.h"
  "D:\\TIADC\\summer2025_research_tiadc\\test_platform\\final_ver_1\\psu_cortexa53_0\\standalone_psu_cortexa53_0\\bsp\\include\\xemac_ieee_reg.h"
  "D:\\TIADC\\summer2025_research_tiadc\\test_platform\\final_ver_1\\psu_cortexa53_0\\standalone_psu_cortexa53_0\\bsp\\include\\xemacpsif_hw.h"
  "D:\\TIADC\\summer2025_research_tiadc\\test_platform\\final_ver_1\\psu_cortexa53_0\\standalone_psu_cortexa53_0\\bsp\\include\\xiltimer.h"
  "D:\\TIADC\\summer2025_research_tiadc\\test_platform\\final_ver_1\\psu_cortexa53_0\\standalone_psu_cortexa53_0\\bsp\\include\\xlwipconfig.h"
  "D:\\TIADC\\summer2025_research_tiadc\\test_platform\\final_ver_1\\psu_cortexa53_0\\standalone_psu_cortexa53_0\\bsp\\include\\xtimer_config.h"
  "D:\\TIADC\\summer2025_research_tiadc\\test_platform\\final_ver_1\\psu_cortexa53_0\\standalone_psu_cortexa53_0\\bsp\\lib\\liblwip220.a"
  "D:\\TIADC\\summer2025_research_tiadc\\test_platform\\final_ver_1\\psu_cortexa53_0\\standalone_psu_cortexa53_0\\bsp\\lib\\libxiltimer.a"
  )
endif()
