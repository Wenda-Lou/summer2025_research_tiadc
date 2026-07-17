import socket
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .frame import (
    align_frame_to_reference,
    reconstruct_adc_bytes,
)

PROJECT_DIR = Path(__file__).resolve().parents[1]
SAVE_DIR = PROJECT_DIR / 'adc_data'
SAVE_DIR.mkdir(parents=True, exist_ok=True)


DAC_SAMPLE_RATE_HZ = 2_457_600_000.0
ADC_SAMPLE_RATE_HZ = 1_300_000_000.0
EXPECTED_DAC_TXT_SAMPLES = 65_536


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





SELECTED_RECONSTRUCTION_MODE = "grouped_halves_interleave"


def receive_timing_captures(
    frame_count=20,
    reference_txt=None,
    bind_ip="0.0.0.0",
    port=6666,
    packets_per_frame=8,
    packet_size=512,
    timeout=20.0,
):
    """
    Receive repeated ADC captures and align every frame to frame 1.

    The DAC TXT is not used for this timing-repeatability test. Frame 1 becomes
    the master ADC reference, and later frames are circularly shifted to match
    its phase.
    """
    if frame_count <= 0:
        raise ValueError("frame_count must be positive.")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    timing_dir = SAVE_DIR / f"timing_test_frame1_{timestamp}"
    timing_dir.mkdir(parents=True, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_ip, port))
    sock.settimeout(timeout)

    print(f"Listening on {bind_ip}:{port}")
    print(f"Reconstruction mode: {SELECTED_RECONSTRUCTION_MODE}")
    print(f"Waiting for {frame_count} timing frames")
    print(f"Run on UART: adc -timing {frame_count}")

    captures = []

    try:
        for frame_index in range(1, frame_count + 1):
            packets = []
            print(f"Receiving frame {frame_index}/{frame_count}")

            for packet_index in range(1, packets_per_frame + 1):
                data, _addr = sock.recvfrom(max(2048, packet_size))

                if len(data) != packet_size:
                    print(
                        f"Warning: frame {frame_index}, "
                        f"packet {packet_index}: expected "
                        f"{packet_size} bytes, got {len(data)}"
                    )

                packets.append(data)

            raw_bytes = b"".join(packets)
            adc = reconstruct_adc_bytes(
                raw_bytes,
                mode=SELECTED_RECONSTRUCTION_MODE,
            )
            captures.append(adc)

            pd.DataFrame({
                "byte": np.frombuffer(raw_bytes, dtype=np.uint8)
            }).to_csv(
                timing_dir / f"frame_{frame_index:03d}_raw.csv",
                index=False,
            )

            pd.DataFrame({
                "sample_index": np.arange(adc.size, dtype=np.int32),
                "adc_code": adc,
            }).to_csv(
                timing_dir / f"frame_{frame_index:03d}_adc.csv",
                index=False,
            )

    except socket.timeout:
        print(
            f"Timeout after receiving "
            f"{len(captures)}/{frame_count} complete frames."
        )

        if not captures:
            raise TimeoutError("No complete timing frames were received.")

        print("Processing the complete frames already received.")

    finally:
        sock.close()

    frame1 = captures[0]
    results = []
    summary_rows = []

    for frame_index, adc in enumerate(captures, start=1):
        result = align_frame_to_reference(frame1, adc)
        results.append(result)

        aligned_file = timing_dir / f"frame_{frame_index:03d}_aligned.csv"

        pd.DataFrame({
            "sample_index": np.arange(
                result["aligned_adc"].size,
                dtype=np.int32,
            ),
            "frame1_adc_code": frame1[:result["aligned_adc"].size],
            "raw_adc_code": adc[:result["aligned_adc"].size],
            "aligned_adc_code": result["aligned_adc"],
            "frame1_zscore": result["reference_zscore"],
            "aligned_zscore": result["aligned_zscore"],
            "residual_error_codes": result["residual_error"],
        }).to_csv(aligned_file, index=False)

        summary_rows.append({
            "frame": frame_index,
            "reconstruction_mode": SELECTED_RECONSTRUCTION_MODE,
            "sample_count": int(result["aligned_adc"].size),
            "lag_samples_relative_to_frame1": result["lag_samples"],
            "correlation_to_frame1": result["correlation"],
            "fitted_gain_to_frame1": result["fitted_gain_to_frame1"],
            "fitted_offset_to_frame1": result["fitted_offset_to_frame1"],
            "aligned_rmse_codes": result["rmse_codes"],
            "mean_code": float(np.mean(adc)),
            "std_code": float(np.std(adc)),
            "min_code": int(np.min(adc)),
            "max_code": int(np.max(adc)),
            "aligned_csv": str(aligned_file),
        })

    summary_df = pd.DataFrame(summary_rows)
    summary_file = timing_dir / "timing_summary.csv"
    summary_df.to_csv(summary_file, index=False)

    max_plot_samples = min(frame1.size, 1000)

    plt.figure(figsize=(14, 6))
    for result in results:
        plt.plot(
            result["aligned_zscore"][:max_plot_samples],
            linewidth=0.7,
            alpha=0.45,
        )
    plt.xlabel("Sample index")
    plt.ylabel("Normalized amplitude")
    plt.title("ADC captures aligned to frame 1")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(
        timing_dir / "aligned_to_frame1_overlay.png",
        dpi=150,
    )
    plt.close()

    plt.figure(figsize=(10, 5))
    plt.plot(
        summary_df["frame"],
        summary_df["lag_samples_relative_to_frame1"],
        marker="o",
    )
    plt.xlabel("Frame")
    plt.ylabel("Lag relative to frame 1 (samples)")
    plt.title("Detected ADC capture lag")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(timing_dir / "lag_relative_to_frame1.png", dpi=150)
    plt.close()

    plt.figure(figsize=(10, 5))
    plt.plot(
        summary_df["frame"],
        summary_df["correlation_to_frame1"],
        marker="o",
    )
    plt.xlabel("Frame")
    plt.ylabel("Normalized correlation")
    plt.title("Frame-to-frame timing correlation")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(timing_dir / "correlation_to_frame1.png", dpi=150)
    plt.close()

    plt.figure(figsize=(10, 5))
    plt.plot(
        summary_df["frame"],
        summary_df["aligned_rmse_codes"],
        marker="o",
    )
    plt.xlabel("Frame")
    plt.ylabel("RMSE (ADC codes)")
    plt.title("Aligned residual relative to frame 1")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(timing_dir / "rmse_to_frame1.png", dpi=150)
    plt.close()

    print("\nFrame-1 timing alignment complete")
    print(f"Folder: {timing_dir}")
    print(f"Summary: {summary_file}")
    print(
        summary_df[[
            "frame",
            "lag_samples_relative_to_frame1",
            "correlation_to_frame1",
            "fitted_gain_to_frame1",
            "fitted_offset_to_frame1",
            "aligned_rmse_codes",
        ]].to_string(index=False)
    )

    return timing_dir, summary_file, summary_df
