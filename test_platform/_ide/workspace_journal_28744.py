# 2026-07-17T04:49:45.937659800
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

vitis.dispose()

