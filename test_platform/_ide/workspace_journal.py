# 2026-08-09T14:48:04.072448
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

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

comp.build()

vitis.dispose()

