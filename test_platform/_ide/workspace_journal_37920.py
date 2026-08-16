# 2026-08-16T11:32:05.691092800
import vitis

client = vitis.create_client()
client.set_workspace(path="test_platform")

vitis.dispose()

