# 2026-07-06T14:30:50.634981300
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

vitis.dispose()

