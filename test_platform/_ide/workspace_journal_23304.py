# 2026-07-07T13:32:59.475571200
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

vitis.dispose()

