# 2026-07-17T04:54:59.690829700
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

comp = client.get_component(name="thesis_v3_500mhz_appl")
status = comp.clean()

comp.build()

comp.build()

comp.build()

status = comp.clean()

comp.build()

status = comp.clean()

comp.build()

status = comp.clean()

comp.build()

status = comp.clean()

comp.build()

comp.build()

comp.build()

