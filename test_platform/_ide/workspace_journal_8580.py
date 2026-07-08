# 2026-06-20T15:23:06.126687900
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

vitis.dispose()

