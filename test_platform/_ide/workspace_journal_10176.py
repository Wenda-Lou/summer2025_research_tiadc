# 2026-06-19T00:28:24.423282900
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

comp = client.get_component(name="thesis_v3_500mhz_appl")
comp.build()

vitis.dispose()

