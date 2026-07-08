# 2026-06-23T16:09:41.701179
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

vitis.dispose()

