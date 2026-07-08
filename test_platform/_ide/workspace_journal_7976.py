# 2026-06-19T16:24:42.616217400
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

advanced_options = client.create_advanced_options_dict(dt_overlay="0")

platform = client.create_platform_component(name = "new_thesis_v3",hw_design = "$COMPONENT_LOCATION/../new_thesis_v3.xsa",os = "standalone",cpu = "psu_cortexa53_0",domain_name = "standalone_psu_cortexa53_0",generate_dtb = False,advanced_options = advanced_options,architecture = "64-bit",compiler = "gcc")

platform = client.get_component(name="new_thesis_v3")
domain = platform.get_domain(name="standalone_psu_cortexa53_0")

status = domain.set_lib(lib_name="lwip220", path="C:\AMD\2025.1\Vitis\data\embeddedsw\ThirdParty\sw_services\lwip220_v1_2")

status = platform.build()

comp = client.get_component(name="thesis_v3_500mhz_appl")
comp.build()

vitis.dispose()

