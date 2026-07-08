# 2026-06-18T15:33:22.668896100
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

comp.build()

comp.build()

comp.build()

comp.build()

comp.build()

vitis.dispose()

