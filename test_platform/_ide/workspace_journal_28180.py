# 2026-06-20T15:25:49.957677400
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

vitis.dispose()

