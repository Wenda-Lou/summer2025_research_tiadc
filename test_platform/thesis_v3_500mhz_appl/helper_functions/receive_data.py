import socket
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd

from .frame import reconstruct_adc_bytes

PROJECT_DIR = Path(__file__).resolve().parents[1]
SAVE_DIR = PROJECT_DIR / 'adc_data'
SAVE_DIR.mkdir(parents=True, exist_ok=True)


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
