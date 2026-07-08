# 2026-07-03T14:45:58.973804600
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

comp = client.get_component(name="thesis_v3_500mhz_appl")
comp.build()

comp.build()

comp.build()

advanced_options = client.create_advanced_options_dict(dt_overlay="0")

platform = client.create_platform_component(name = "final_ver",hw_design = "$COMPONENT_LOCATION/../final_ver.xsa",os = "standalone",cpu = "psu_cortexa53_0",domain_name = "standalone_psu_cortexa53_0",generate_dtb = False,advanced_options = advanced_options,architecture = "64-bit",compiler = "gcc")

platform = client.get_component(name="final_ver")
domain = platform.get_domain(name="standalone_psu_cortexa53_0")

status = domain.set_lib(lib_name="lwip220", path="C:\AMD\2025.1\Vitis\data\embeddedsw\ThirdParty\sw_services\lwip220_v1_2")

status = domain.regenerate()

status = platform.build()

status = comp.clean()

comp.build()

comp.build()

comp.build()

advanced_options = client.create_advanced_options_dict(dt_overlay="0")

platform = client.create_platform_component(name = "final_ver_1",hw_design = "$COMPONENT_LOCATION/../final_ver_1.0.xsa",os = "standalone",cpu = "psu_cortexa53_0",domain_name = "standalone_psu_cortexa53_0",generate_dtb = False,advanced_options = advanced_options,architecture = "64-bit",compiler = "gcc")

platform = client.get_component(name="final_ver_1")
status = domain.set_lib(lib_name="lwip220", path="C:\AMD\2025.1\Vitis\data\embeddedsw\ThirdParty\sw_services\lwip220_v1_2")

status = domain.regenerate()

status = domain.regenerate()

status = platform.build()

comp.build()

comp.build()

vitis.dispose()

