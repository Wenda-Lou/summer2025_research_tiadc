import struct
import numpy as np

from reference_upload import (
    REFERENCE_SAMPLE_COUNT,
    build_reference_packets,
)


def test_known_sequence_packets():
    reference = np.arange(REFERENCE_SAMPLE_COUNT, dtype=np.int16)
    packets = build_reference_packets(reference)

    assert packets[0][:4] == b"REFB"
    assert struct.unpack("<H", packets[0][4:6])[0] == REFERENCE_SAMPLE_COUNT
    assert packets[-1] == b"REFE"

    rebuilt = np.empty(REFERENCE_SAMPLE_COUNT, dtype="<i2")
    for packet in packets[1:-1]:
        assert packet[:4] == b"REFD"
        offset, count = struct.unpack("<HH", packet[4:8])
        values = np.frombuffer(packet[8:], dtype="<i2")
        assert values.size == count
        rebuilt[offset : offset + count] = values

    np.testing.assert_array_equal(rebuilt, reference)


if __name__ == "__main__":
    test_known_sequence_packets()
    print("Local packet-format test passed.")
    print("This does not verify reception by the FPGA; confirm on UART.")
