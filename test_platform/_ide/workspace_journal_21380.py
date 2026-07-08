# 2026-07-06T14:33:01.381187900
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

comp = client.get_component(name="thesis_v3_500mhz_appl")
comp.build()

vitis.dispose()

