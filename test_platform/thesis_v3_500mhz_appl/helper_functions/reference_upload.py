"""UDP uploader for the FPGA reference buffer.

Packet format (kept compatible with the existing firmware):
    REFB + uint16_le(total_samples)
    REFD + uint16_le(offset) + uint16_le(count) + int16_le samples
    REFE
    REFC

This module can validate packet construction locally. Actual FPGA reception
must still be verified from the FPGA UART output after any code change.
"""

from __future__ import annotations

import socket
import struct
import time
from typing import Iterable

import numpy as np

FPGA_IP = "192.168.1.10"
FPGA_PORT = 6666
REFERENCE_SAMPLE_COUNT = 2032
# Firmware accepts at most (512 - 8) / 2 = 252 samples per REFD packet.
# Keep 240 to match the previously used conservative packet size.
REFERENCE_CHUNK_SAMPLES = 240


def prepare_reference_samples(
    reference_samples: Iterable[float],
    *,
    require_full_buffer: bool = True,
) -> np.ndarray:
    """Convert reference values to little-endian signed 16-bit samples."""
    reference = np.asarray(reference_samples, dtype=np.float64).reshape(-1)

    if reference.size == 0:
        raise ValueError("Reference is empty.")
    if not np.all(np.isfinite(reference)):
        raise ValueError("Reference contains NaN or infinite values.")

    if require_full_buffer and reference.size != REFERENCE_SAMPLE_COUNT:
        raise ValueError(
            f"Expected exactly {REFERENCE_SAMPLE_COUNT} samples, "
            f"received {reference.size}."
        )
    if reference.size > REFERENCE_SAMPLE_COUNT:
        raise ValueError(
            f"Reference contains {reference.size} samples; "
            f"maximum is {REFERENCE_SAMPLE_COUNT}."
        )

    return np.clip(np.rint(reference), -32768, 32767).astype("<i2")


def build_reference_packets(
    reference_samples: Iterable[float],
    *,
    chunk_samples: int = REFERENCE_CHUNK_SAMPLES,
    require_full_buffer: bool = True,
) -> list[bytes]:
    """Build packets without sending them, for local inspection/testing."""
    if not 1 <= chunk_samples <= 252:
        raise ValueError("chunk_samples must be between 1 and 252.")

    reference = prepare_reference_samples(
        reference_samples,
        require_full_buffer=require_full_buffer,
    )

    packets = [b"REFB" + struct.pack("<H", reference.size)]

    for offset in range(0, reference.size, chunk_samples):
        chunk = reference[offset : offset + chunk_samples]
        packets.append(
            b"REFD"
            + struct.pack("<HH", offset, chunk.size)
            + chunk.tobytes()
        )

    packets.append(b"REFE")
    return packets


def send_reference(
    reference_samples: Iterable[float],
    fpga_ip: str = FPGA_IP,
    fpga_port: int = FPGA_PORT,
    chunk_samples: int = REFERENCE_CHUNK_SAMPLES,
    packet_delay_s: float = 0.01,
    require_full_buffer: bool = True,
) -> int:
    """Send one prepared reference using the existing FPGA UDP protocol."""
    packets = build_reference_packets(
        reference_samples,
        chunk_samples=chunk_samples,
        require_full_buffer=require_full_buffer,
    )

    destination = (fpga_ip, fpga_port)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        for index, packet in enumerate(packets):
            sock.sendto(packet, destination)
            # Give the bare-metal receive callback time to process packets.
            if index != len(packets) - 1 and packet_delay_s > 0:
                time.sleep(packet_delay_s)

    sample_count = struct.unpack("<H", packets[0][4:6])[0]
    print(
        f"Sent {sample_count} reference samples to "
        f"{fpga_ip}:{fpga_port}."
    )
    print(
        "Check the FPGA UART for 'Reference uploaded successfully' and "
        "the expected first/last sample values."
    )
    return int(sample_count)


def send_known_test_sequence(
    fpga_ip: str = FPGA_IP,
    fpga_port: int = FPGA_PORT,
) -> int:
    """Send 0..2031 for end-to-end FPGA/UART verification."""
    reference = np.arange(REFERENCE_SAMPLE_COUNT, dtype=np.int16)
    return send_reference(reference, fpga_ip=fpga_ip, fpga_port=fpga_port)


def clear_reference(
    fpga_ip: str = FPGA_IP,
    fpga_port: int = FPGA_PORT,
) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(b"REFC", (fpga_ip, fpga_port))
    print(
        f"Sent reference-clear command to {fpga_ip}:{fpga_port}. "
        "Confirm 'Reference cleared' on the FPGA UART."
    )
