import socket
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .frame import estimate_circular_lag, reconstruct_adc_bytes

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


def receive_timing_captures(
    frame_count=20,
    bind_ip="0.0.0.0",
    port=6666,
    packets_per_frame=8,
    packet_size=512,
    timeout=20.0,
):
    """
    Receive repeated frames produced by `adc -timing <frame_count>`.

    Each frame is assumed to contain packets_per_frame UDP packets. Captures
    are aligned to the first received ADC frame using integer circular
    cross-correlation. This verifies collection repeatability without requiring
    DAC/ADC sample-rate conversion yet.
    """
    if frame_count <= 0:
        raise ValueError("frame_count must be positive.")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    timing_dir = SAVE_DIR / f"timing_test_{timestamp}"
    timing_dir.mkdir(parents=True, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_ip, port))
    sock.settimeout(timeout)

    print(f"Listening on {bind_ip}:{port}")
    print(f"Waiting for {frame_count} timing frames")
    print(f"Packets per frame: {packets_per_frame}")
    print("Now run the FPGA command: adc -timing " + str(frame_count))

    captures = []
    summary = []

    try:
        for frame_index in range(1, frame_count + 1):
            packets = []
            print(f"\nReceiving timing frame {frame_index}/{frame_count}")

            for packet_index in range(1, packets_per_frame + 1):
                data, addr = sock.recvfrom(max(2048, packet_size))
                if len(data) != packet_size:
                    print(
                        f"Warning: frame {frame_index}, packet {packet_index}: "
                        f"expected {packet_size} bytes, got {len(data)}"
                    )
                packets.append(data)
                print(
                    f"  packet {packet_index}/{packets_per_frame}: "
                    f"{len(data)} bytes from {addr}"
                )

            raw_bytes = b"".join(packets)
            adc = reconstruct_adc_bytes(raw_bytes)
            captures.append(adc)

            pd.DataFrame({"byte": np.frombuffer(raw_bytes, dtype=np.uint8)}).to_csv(
                timing_dir / f"frame_{frame_index:03d}_raw.csv", index=False
            )
            pd.DataFrame({
                "sample_index": np.arange(adc.size, dtype=np.int32),
                "adc_code": adc,
            }).to_csv(timing_dir / f"frame_{frame_index:03d}_adc.csv", index=False)

    except socket.timeout as exc:
        raise TimeoutError(
            f"Timed out after receiving {len(captures)}/{frame_count} complete frames. "
            "Start this receiver before issuing the FPGA timing command."
        ) from exc
    finally:
        sock.close()

    reference = captures[0]
    aligned_frames = []

    for index, adc in enumerate(captures, start=1):
        lag, correlation = estimate_circular_lag(reference, adc)
        aligned = np.roll(adc, -lag)
        aligned_frames.append(aligned)

        n = min(reference.size, aligned.size)
        ref_float = reference[:n].astype(np.float64)
        aligned_float = aligned[:n].astype(np.float64)

        # Fit scale and offset so RMSE reflects timing/shape repeatability rather
        # than small gain or DC changes between frames.
        design = np.column_stack((aligned_float, np.ones(n)))
        scale, offset = np.linalg.lstsq(design, ref_float, rcond=None)[0]
        fitted = scale * aligned_float + offset
        rmse = float(np.sqrt(np.mean((fitted - ref_float) ** 2)))

        aligned_file = timing_dir / f"frame_{index:03d}_aligned.csv"
        pd.DataFrame({
            "sample_index": np.arange(n, dtype=np.int32),
            "adc_code": adc[:n],
            "aligned_adc_code": aligned[:n],
            "reference_frame_code": reference[:n],
        }).to_csv(aligned_file, index=False)

        summary.append({
            "frame": index,
            "sample_count": int(n),
            "lag_samples": int(lag),
            "correlation": correlation,
            "fitted_scale_to_frame1": float(scale),
            "fitted_offset_to_frame1": float(offset),
            "aligned_rmse_codes": rmse,
            "mean_code": float(np.mean(adc[:n])),
            "rms_code": float(np.sqrt(np.mean(adc[:n].astype(np.float64) ** 2))),
            "min_code": int(np.min(adc[:n])),
            "max_code": int(np.max(adc[:n])),
            "aligned_csv": str(aligned_file),
        })

    summary_df = pd.DataFrame(summary)
    summary_file = timing_dir / "timing_summary.csv"
    summary_df.to_csv(summary_file, index=False)

    # Save overlay figures without opening pop-up plots.
    max_plot_samples = min(reference.size, 1000)
    plt.figure(figsize=(14, 6))
    for aligned in aligned_frames:
        plt.plot(aligned[:max_plot_samples], linewidth=0.7, alpha=0.5)
    plt.xlabel("Sample index")
    plt.ylabel("ADC code")
    plt.title("Timing-aligned ADC captures")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(timing_dir / "aligned_overlay.png", dpi=150)
    plt.close()

    plt.figure(figsize=(10, 5))
    plt.plot(summary_df["frame"], summary_df["lag_samples"], marker="o")
    plt.xlabel("Frame")
    plt.ylabel("Detected lag (samples)")
    plt.title("Raw capture timing offset relative to frame 1")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(timing_dir / "detected_lag.png", dpi=150)
    plt.close()

    print("\nTiming capture complete")
    print(f"Folder: {timing_dir}")
    print(f"Summary: {summary_file}")
    print(summary_df[[
        "frame", "lag_samples", "correlation", "aligned_rmse_codes",
        "fitted_scale_to_frame1", "fitted_offset_to_frame1"
    ]].to_string(index=False))

    return timing_dir, summary_file, summary_df

