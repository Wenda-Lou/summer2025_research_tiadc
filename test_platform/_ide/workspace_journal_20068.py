# 2026-06-23T16:26:16.979093500
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

comp = client.get_component(name="thesis_v3_500mhz_appl")
status = comp.clean()

comp.build()

comp.build()

comp.build()

