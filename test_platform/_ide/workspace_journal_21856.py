# 2026-08-08T14:49:27.746420100
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

vitis.dispose()

