# 2026-06-19T07:50:34.983901500
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

comp = client.get_component(name="thesis_v3_500mhz_appl")
comp.build()

comp.build()

comp.build()

platform = client.get_component(name="thesis_v3_500mhz_pf")
status = platform.update_hw(hw_design = "$COMPONENT_LOCATION/hw/thesis_v3_500mhz_pf.xsa")

vitis.dispose()

