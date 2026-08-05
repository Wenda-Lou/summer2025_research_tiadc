import socket
import struct
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd

from .frame import reconstruct_adc_bytes

PROJECT_DIR = Path(__file__).resolve().parents[1]
SAVE_DIR = PROJECT_DIR / 'adc_data'
SAVE_DIR.mkdir(parents=True, exist_ok=True)

CALIBRATION_CSV_MAGIC = b"CALC"
CALIBRATION_CSV_HEADER_SIZE = 12


def receive_adc_data(
    bind_ip="0.0.0.0",
    port=6666,
    expected_packets=8,
    packet_size=512,
    timeout=15.0,
):
    save_dir = SAVE_DIR

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_ip, port))
    sock.settimeout(timeout)

    print(f"Listening on {bind_ip}:{port}")
    print(f"Waiting for {expected_packets} packets...")

    packets = []

    try:
        for i in range(expected_packets):
            data, addr = sock.recvfrom(2048)

            print(f"Packet {i+1}/{expected_packets}: {len(data)} bytes from {addr}")

            if len(data) != packet_size:
                print(f"Warning: expected {packet_size} bytes, got {len(data)} bytes")

            packets.append(data)

    except socket.timeout:
        print(f"\nTimeout: only received {len(packets)}/{expected_packets} packets.")
        print("This often happens on the first FPGA UDP run after boot.")
        print("Try running the UDP command again, or add a dummy warm-up transfer.")

        sock.close()
        return None

    sock.close()

    raw = np.frombuffer(b"".join(packets), dtype=np.uint8)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = save_dir / f"adc_capture_{timestamp}.csv"

    pd.DataFrame({"byte": raw}).to_csv(filename, index=False)

    print(f"\nSaved {len(raw)} bytes")
    print(f"File: {filename}")

    return filename


def receive_calibration_csv(
    bind_ip="0.0.0.0",
    port=6666,
    timeout=30.0,
):
    """Receive a framed, variable-length calibration CSV export."""
    save_dir = SAVE_DIR / "calibration_exports"
    save_dir.mkdir(parents=True, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_ip, port))
    deadline = time.monotonic() + timeout
    packets = {}
    expected_count = None
    expected_length = None
    sender = None

    print(f"Listening for calibration CSV on {bind_ip}:{port}")
    print("Run 'adc -cal export' in the FPGA UART terminal.")

    try:
        while expected_count is None or len(packets) < expected_count:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                received = len(packets)
                expected = expected_count if expected_count is not None else "unknown"
                raise TimeoutError(
                    f"Timed out receiving calibration CSV "
                    f"({received}/{expected} packets)."
                )
            sock.settimeout(remaining)
            try:
                data, address = sock.recvfrom(2048)
            except socket.timeout as exc:
                received = len(packets)
                expected = expected_count if expected_count is not None else "unknown"
                raise TimeoutError(
                    f"Timed out receiving calibration CSV "
                    f"({received}/{expected} packets)."
                ) from exc

            if len(data) < CALIBRATION_CSV_HEADER_SIZE or \
                    data[:4] != CALIBRATION_CSV_MAGIC:
                continue

            packet_index, packet_count, total_length = struct.unpack(
                "!HHI", data[4:CALIBRATION_CSV_HEADER_SIZE]
            )
            if packet_count == 0 or packet_index >= packet_count:
                continue
            if expected_count is None:
                expected_count = packet_count
                expected_length = total_length
                sender = address
            elif address != sender or packet_count != expected_count or \
                    total_length != expected_length:
                continue

            packets[packet_index] = data[CALIBRATION_CSV_HEADER_SIZE:]
            print(
                f"Calibration packet {packet_index + 1}/{packet_count} "
                f"from {address}"
            )
    finally:
        sock.close()

    payload = b"".join(packets[index] for index in range(expected_count))
    payload = payload[:expected_length]
    if len(payload) != expected_length:
        raise ValueError(
            f"Incomplete calibration CSV: expected {expected_length} bytes, "
            f"assembled {len(payload)} bytes."
        )
    try:
        payload.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError("Calibration export is not valid UTF-8 CSV data.") from exc

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = save_dir / f"calibration_performance_{timestamp}.csv"
    filename.write_bytes(payload)
    print(f"Saved calibration CSV: {filename}")
    return filename



def receive_adc_frame(
    bind_ip="0.0.0.0",
    port=6666,
    expected_packets=8,
    packet_size=512,
    timeout=15.0,
):
    """Receive one complete DMA frame and return reconstructed ADC samples."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_ip, port))
    sock.settimeout(timeout)
    packets = []

    try:
        for packet_index in range(expected_packets):
            data, addr = sock.recvfrom(max(2048, packet_size))
            if len(data) != packet_size:
                print(
                    f"Warning: packet {packet_index + 1}: expected "
                    f"{packet_size} bytes, got {len(data)} bytes from {addr}"
                )
            packets.append(data)
    except socket.timeout as exc:
        raise TimeoutError(
            f"Timed out after receiving {len(packets)}/{expected_packets} packets."
        ) from exc
    finally:
        sock.close()

    return reconstruct_adc_bytes(b"".join(packets))
