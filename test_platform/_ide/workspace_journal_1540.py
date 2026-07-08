# 2026-06-18T14:34:46.073181600
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

platform = client.get_component(name="thesis_v3_500mhz_pf")
status = platform.build()

comp = client.get_component(name="thesis_v3_500mhz_appl")
comp.build()

comp.build()

comp.build()

comp.build()

comp.build()

comp.build()

vitis.dispose()

