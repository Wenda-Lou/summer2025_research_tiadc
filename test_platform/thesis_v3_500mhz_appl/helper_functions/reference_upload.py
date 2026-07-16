import socket
import struct
import time

import numpy as np


FPGA_IP = "192.168.1.10"
FPGA_PORT = 6666

REFERENCE_SAMPLE_COUNT = 2032
REFERENCE_CHUNK_SAMPLES = 240


def prepare_reference_samples(reference_samples):
    """
    Convert an aligned/fitted ADC-domain reference into the exact format
    expected by the FPGA reference buffer.

    The input must already be:
      - aligned to the ADC capture,
      - expressed in ADC-code units,
      - no longer than REFERENCE_SAMPLE_COUNT.
    """
    reference = np.asarray(reference_samples, dtype=np.float64).reshape(-1)

    if reference.size == 0:
        raise ValueError("Reference is empty.")

    if reference.size > REFERENCE_SAMPLE_COUNT:
        raise ValueError(
            f"Reference contains {reference.size} samples; "
            f"maximum is {REFERENCE_SAMPLE_COUNT}."
        )

    if not np.all(np.isfinite(reference)):
        raise ValueError("Reference contains NaN or infinite values.")

    return np.clip(
        np.rint(reference),
        -32768,
        32767,
    ).astype("<i2")


def send_reference(
    reference_samples,
    fpga_ip=FPGA_IP,
    fpga_port=FPGA_PORT,
    chunk_samples=REFERENCE_CHUNK_SAMPLES,
):
    """
    Upload one aligned ADC-domain reference frame to the FPGA.

    Packet protocol:
      REFB + uint16 sample_count
      REFD + uint16 sample_offset + uint16 chunk_count + int16 payload
      REFE
    """
    reference = prepare_reference_samples(reference_samples)

    if chunk_samples <= 0:
        raise ValueError("chunk_samples must be positive.")

    destination = (fpga_ip, fpga_port)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        begin_packet = b"REFB" + struct.pack("<H", reference.size)
        sock.sendto(begin_packet, destination)
        time.sleep(0.02)

        for offset in range(0, reference.size, chunk_samples):
            chunk = reference[offset:offset + chunk_samples]

            header = b"REFD" + struct.pack(
                "<HH",
                offset,
                chunk.size,
            )

            sock.sendto(
                header + chunk.tobytes(),
                destination,
            )
            time.sleep(0.01)

        sock.sendto(b"REFE", destination)

    print(
        f"Sent {reference.size} aligned reference samples "
        f"to {fpga_ip}:{fpga_port}"
    )

    return int(reference.size)


def clear_reference(
    fpga_ip=FPGA_IP,
    fpga_port=FPGA_PORT,
):
    """Clear the FPGA reference buffer."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(b"REFC", (fpga_ip, fpga_port))

    print(f"Sent reference-clear command to {fpga_ip}:{fpga_port}")


if __name__ == "__main__":
    import numpy as np

    test_reference = np.arange(2032, dtype=np.int16)

    send_reference(test_reference)