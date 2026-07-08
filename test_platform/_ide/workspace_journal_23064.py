# 2026-06-22T16:26:08.607036200
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

vitis.dispose()

