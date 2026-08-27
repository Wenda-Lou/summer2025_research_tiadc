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

CALIBRATION_CSV_DATASETS = {
    b"CALT": "calibration_timing_captures.csv",
    b"CALB": "calibration_baseline_captures.csv",
    b"CALO": "calibration_offset_captures.csv",
    b"CAOI": "calibration_offset_iterations.csv",
    b"CALG": "calibration_gain_captures.csv",
    b"CAGI": "calibration_gain_iterations.csv",
    b"CALS": "calibration_skew_captures.csv",
    b"CASI": "calibration_skew_iterations.csv",
    # Keep CALC assigned to performance for compatibility with existing FPGA
    # exports and receivers.
    b"CALC": "calibration_performance.csv",
}
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
    idle_timeout=1.5,
):
    """Receive all typed CSV datasets sent by one ``adc -cal export``.

    Each dataset uses the existing packet-index/count/length framing.  The
    four-byte magic identifies its stage, allowing incomplete calibration runs
    to return only the histories that actually exist.
    """
    save_dir = SAVE_DIR / "calibration_exports"
    save_dir.mkdir(parents=True, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_ip, port))
    deadline = time.monotonic() + timeout
    transfers = {}
    completed = {}
    last_packet_time = None

    print(f"Listening for calibration CSV on {bind_ip}:{port}")
    print("Run 'adc -cal export' in the FPGA UART terminal.")

    try:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                if completed:
                    break
                raise TimeoutError("Timed out before receiving a calibration CSV dataset.")
            if completed and last_packet_time is not None:
                idle_remaining = idle_timeout - (
                    time.monotonic() - last_packet_time
                )
                if idle_remaining <= 0.0:
                    break
                remaining = min(remaining, idle_remaining)
            sock.settimeout(remaining)
            try:
                data, address = sock.recvfrom(2048)
            except socket.timeout as exc:
                if completed:
                    break
                raise TimeoutError(
                    "Timed out before receiving a calibration CSV dataset."
                ) from exc

            if len(data) < CALIBRATION_CSV_HEADER_SIZE:
                continue
            magic = data[:4]
            if magic not in CALIBRATION_CSV_DATASETS:
                continue
            last_packet_time = time.monotonic()

            packet_index, packet_count, total_length = struct.unpack(
                "!HHI", data[4:CALIBRATION_CSV_HEADER_SIZE]
            )
            if packet_count == 0 or packet_index >= packet_count:
                continue
            transfer = transfers.setdefault(
                magic,
                {
                    "packet_count": packet_count,
                    "total_length": total_length,
                    "sender": address,
                    "packets": {},
                },
            )
            if address != transfer["sender"] or \
                    packet_count != transfer["packet_count"] or \
                    total_length != transfer["total_length"]:
                continue

            transfer["packets"][packet_index] = data[
                CALIBRATION_CSV_HEADER_SIZE:
            ]
            print(
                f"{CALIBRATION_CSV_DATASETS[magic]} packet "
                f"{packet_index + 1}/{packet_count} from {address}"
            )
            if len(transfer["packets"]) == packet_count:
                payload = b"".join(
                    transfer["packets"][index]
                    for index in range(packet_count)
                )[:total_length]
                if len(payload) != total_length:
                    raise ValueError(
                        f"Incomplete {CALIBRATION_CSV_DATASETS[magic]}: "
                        f"expected {total_length} bytes, assembled "
                        f"{len(payload)} bytes."
                    )
                try:
                    payload.decode("utf-8")
                except UnicodeDecodeError as exc:
                    raise ValueError(
                        f"{CALIBRATION_CSV_DATASETS[magic]} is not valid "
                        "UTF-8 CSV data."
                    ) from exc
                completed[magic] = payload
    finally:
        sock.close()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = save_dir / f"calibration_run_{timestamp}"
    run_dir.mkdir(parents=True, exist_ok=False)
    saved = {}
    for magic, filename in CALIBRATION_CSV_DATASETS.items():
        if magic not in completed:
            continue
        path = run_dir / filename
        path.write_bytes(completed[magic])
        saved[filename] = path
        print(f"Saved calibration CSV: {path}")
    if not saved:
        raise TimeoutError("No complete calibration CSV dataset was received.")
    return saved



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
