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
from pathlib import Path
from typing import Iterable

import numpy as np

FPGA_IP = "192.168.1.10"
FPGA_PORT = 6666
REFERENCE_SAMPLE_COUNT = 2032
DAC_REFERENCE_SAMPLE_COUNT = 4064
DAC_SAMPLE_RATE_HZ = 2_600_000_000.0
ADC_SAMPLE_RATE_HZ = 1_450_000_000.0
DAC_BITS = 16
ADC_BITS = 14
# Firmware accepts at most (512 - 8) / 2 = 252 samples per REFD packet.
# Keep 240 to match the previously used conservative packet size.
REFERENCE_CHUNK_SAMPLES = 240
REFERENCE_FORMAT_ADC_RATE = 0
REFERENCE_FORMAT_DAC_RATE_2X = 1



def load_reference_txt(txt_file: str | Path) -> np.ndarray:
    """
    Load numeric reference samples from a TXT/DAT file.

    Supported input:
    - one value per line
    - whitespace- or comma-separated values
    - blank lines
    - comments beginning with #, //, or ;

    Values are rounded, clipped to signed int16, and returned as a
    little-endian NumPy array. Sample-count selection is handled by the GUI.
    """
    path = Path(txt_file)

    if not path.is_file():
        raise FileNotFoundError(f"Reference TXT file not found:\n{path}")

    values: list[float] = []

    with path.open("r", encoding="utf-8-sig") as handle:
        for line_number, original_line in enumerate(handle, start=1):
            line = original_line.strip()

            if not line:
                continue

            for marker in ("//", "#", ";"):
                marker_index = line.find(marker)
                if marker_index >= 0:
                    line = line[:marker_index].strip()

            if not line:
                continue

            for token in line.replace(",", " ").split():
                try:
                    values.append(float(token))
                except ValueError as exc:
                    raise ValueError(
                        f"Invalid numeric value '{token}' on line "
                        f"{line_number} of:\n{path}"
                    ) from exc

    if not values:
        raise ValueError(f"No numeric reference samples found in:\n{path}")

    reference = np.asarray(values, dtype=np.float64).reshape(-1)

    if not np.all(np.isfinite(reference)):
        raise ValueError("Reference contains NaN or infinite values.")

    return np.clip(
        np.rint(reference),
        -32768,
        32767,
    ).astype("<i2")


def reconstruct_adc_reference(
    dac_samples: Iterable[float],
    *,
    start_dac_index: float = 0.0,
    output_sample_count: int = REFERENCE_SAMPLE_COUNT,
    dac_sample_rate_hz: float = DAC_SAMPLE_RATE_HZ,
    adc_sample_rate_hz: float = ADC_SAMPLE_RATE_HZ,
) -> np.ndarray:
    """Reconstruct ideal signed 14-bit ADC samples from a periodic DAC file.

    The AD9164 TXT entries lie on the DAC time grid.  ADC sample positions are
    therefore spaced by ``dac_sample_rate_hz / adc_sample_rate_hz`` entries,
    not by one adjacent TXT entry.  Linear interpolation is periodic because
    the waveform file is replayed cyclically.

    This models sample rate and digital full-scale conversion only.  Unknown
    converter clock phase, analog-path gain/offset, delay, and bandwidth must
    still be handled by alignment/calibration against a real capture.
    """
    waveform = np.asarray(dac_samples, dtype=np.float64).reshape(-1)

    if waveform.size < 2:
        raise ValueError("At least two DAC samples are required.")
    if not np.all(np.isfinite(waveform)):
        raise ValueError("DAC waveform contains NaN or infinite values.")
    if output_sample_count <= 0:
        raise ValueError("output_sample_count must be positive.")
    if dac_sample_rate_hz <= 0.0 or adc_sample_rate_hz <= 0.0:
        raise ValueError("DAC and ADC sample rates must be positive.")
    if not 0.0 <= start_dac_index < waveform.size:
        raise ValueError(
            f"start_dac_index must be in [0, {waveform.size})."
        )

    dac_positions = (
        start_dac_index
        + np.arange(output_sample_count, dtype=np.float64)
        * (dac_sample_rate_hz / adc_sample_rate_hz)
    ) % waveform.size
    lower = np.floor(dac_positions).astype(np.int64)
    fraction = dac_positions - lower
    upper = (lower + 1) % waveform.size
    interpolated = (
        waveform[lower] * (1.0 - fraction)
        + waveform[upper] * fraction
    )

    adc_scale = float(1 << (DAC_BITS - ADC_BITS))
    adc_min = -(1 << (ADC_BITS - 1))
    adc_max = (1 << (ADC_BITS - 1)) - 1
    return np.clip(
        np.rint(interpolated / adc_scale),
        adc_min,
        adc_max,
    ).astype("<i2")


def prepare_reference_samples(
    reference_samples: Iterable[float],
    *,
    require_full_buffer: bool = True,
    reference_format: int = REFERENCE_FORMAT_ADC_RATE,
) -> np.ndarray:
    """Convert reference values to little-endian signed 16-bit samples."""
    reference = np.asarray(reference_samples, dtype=np.float64).reshape(-1)

    if reference.size == 0:
        raise ValueError("Reference is empty.")
    if not np.all(np.isfinite(reference)):
        raise ValueError("Reference contains NaN or infinite values.")

    expected_count = (
        DAC_REFERENCE_SAMPLE_COUNT
        if reference_format == REFERENCE_FORMAT_DAC_RATE_2X
        else REFERENCE_SAMPLE_COUNT
    )
    if reference_format not in (
        REFERENCE_FORMAT_ADC_RATE,
        REFERENCE_FORMAT_DAC_RATE_2X,
    ):
        raise ValueError("Unsupported reference format.")
    if require_full_buffer and reference.size != expected_count:
        raise ValueError(
            f"Expected exactly {expected_count} samples, "
            f"received {reference.size}."
        )
    if reference.size > expected_count:
        raise ValueError(
            f"Reference contains {reference.size} samples; "
            f"maximum is {expected_count}."
        )

    return np.clip(np.rint(reference), -32768, 32767).astype("<i2")


def build_reference_packets(
    reference_samples: Iterable[float],
    *,
    chunk_samples: int = REFERENCE_CHUNK_SAMPLES,
    require_full_buffer: bool = True,
    reference_format: int = REFERENCE_FORMAT_ADC_RATE,
) -> list[bytes]:
    """Build packets without sending them, for local inspection/testing."""
    if not 1 <= chunk_samples <= 252:
        raise ValueError("chunk_samples must be between 1 and 252.")

    reference = prepare_reference_samples(
        reference_samples,
        require_full_buffer=require_full_buffer,
        reference_format=reference_format,
    )

    packets = [
        b"REFB" + struct.pack("<HB", reference.size, reference_format)
    ]

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
    reference_format: int = REFERENCE_FORMAT_ADC_RATE,
) -> int:
    """Send one prepared reference using the existing FPGA UDP protocol."""
    packets = build_reference_packets(
        reference_samples,
        chunk_samples=chunk_samples,
        require_full_buffer=require_full_buffer,
        reference_format=reference_format,
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
