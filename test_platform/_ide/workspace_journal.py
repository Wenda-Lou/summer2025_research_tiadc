# 2026-07-17T03:55:48.515114200
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

platform = client.get_component(name="final_ver_1")
status = platform.build()

comp = client.get_component(name="thesis_v3_500mhz_appl")
comp.build()

