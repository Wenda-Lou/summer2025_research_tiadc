# 2026-07-06T15:46:32.341593800
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

vitis.dispose()

