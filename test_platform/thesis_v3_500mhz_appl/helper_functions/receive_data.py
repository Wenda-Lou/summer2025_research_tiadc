import socket
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from .frame import align_adc_to_periodic_reference, load_dac_reference_txt, reconstruct_adc_bytes, resample_periodic_dac_reference

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



def receive_timing_captures(
    frame_count=20,
    reference_txt=None,
    bind_ip="0.0.0.0",
    port=6666,
    packets_per_frame=8,
    packet_size=512,
    timeout=20.0,
):
    if frame_count <= 0:
        raise ValueError("frame_count must be positive.")
    if not reference_txt:
        raise ValueError("reference_txt is required.")

    dac_reference = load_dac_reference_txt(reference_txt)
    reference_period, reference_metadata = resample_periodic_dac_reference(
        dac_reference,
        DAC_SAMPLE_RATE_HZ,
        ADC_SAMPLE_RATE_HZ,
        expected_txt_samples=EXPECTED_DAC_TXT_SAMPLES,
    )

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    timing_dir = SAVE_DIR / f"timing_test_{timestamp}"
    timing_dir.mkdir(parents=True, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((bind_ip, port))
    sock.settimeout(timeout)

    print(f"Listening on {bind_ip}:{port}")
    print(f"Loaded DAC reference: {dac_reference.size} samples")
    print(
        "Exact joint reference period: "
        f"{reference_metadata['dac_repetitions']} DAC TXT repetitions, "
        f"{reference_metadata['joint_dac_samples']} DAC samples, "
        f"{reference_metadata['joint_adc_samples']} ADC samples"
    )
    print(f"Waiting for {frame_count} timing frames")
    print("Now run the FPGA command: adc -timing " + str(frame_count))

    captures = []
    try:
        for frame_index in range(1, frame_count + 1):
            packets = []
            print(f"Receiving frame {frame_index}/{frame_count}")
            for packet_index in range(1, packets_per_frame + 1):
                data, addr = sock.recvfrom(max(2048, packet_size))
                if len(data) != packet_size:
                    print(
                        f"Warning: frame {frame_index}, packet {packet_index}: "
                        f"expected {packet_size}, got {len(data)}"
                    )
                packets.append(data)

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

    except socket.timeout:
        print(f"Timeout after {len(captures)}/{frame_count} complete frames.")
        if not captures:
            raise TimeoutError("No complete timing frames were received.")
        print("Processing the complete frames already received.")
    finally:
        sock.close()

    summary = []
    fitted_frames = []
    timing_results = []

    for index, adc in enumerate(captures, start=1):
        result = align_adc_to_periodic_reference(adc, reference_period)
        fitted_frames.append(result["fitted_adc"])
        timing_results.append(result)

        aligned_file = timing_dir / f"frame_{index:03d}_aligned.csv"
        pd.DataFrame({
            "sample_index": np.arange(adc.size, dtype=np.int32),
            "adc_code": adc,
            "aligned_dac_reference": result["aligned_reference"],
            "fitted_adc_from_reference": result["fitted_adc"],
            "residual_error_codes": result["error"],
        }).to_csv(aligned_file, index=False)

        summary.append({
            "frame": index,
            "sample_count": int(adc.size),
            "reference_start_index": result["reference_start_index"],
            "reference_period_samples": result["reference_period_samples"],
            "dac_txt_samples": reference_metadata["txt_samples"],
            "dac_txt_repetitions": reference_metadata["dac_repetitions"],
            "joint_dac_samples": reference_metadata["joint_dac_samples"],
            "joint_adc_samples": reference_metadata["joint_adc_samples"],
            "dac_sample_rate_hz": reference_metadata["dac_sample_rate_hz"],
            "adc_sample_rate_hz": reference_metadata["adc_sample_rate_hz"],
            "correlation": result["correlation"],
            "fitted_scale_dac_to_adc": result["scale"],
            "fitted_offset_codes": result["offset"],
            "reference_rmse_codes": result["rmse_codes"],
            "mean_code": float(np.mean(adc)),
            "rms_code": float(np.sqrt(np.mean(adc.astype(np.float64) ** 2))),
            "min_code": int(np.min(adc)),
            "max_code": int(np.max(adc)),
            "aligned_csv": str(aligned_file),
        })

    summary_df = pd.DataFrame(summary)
    summary_file = timing_dir / "timing_summary.csv"
    summary_df.to_csv(summary_file, index=False)

    max_plot_samples = min(captures[0].size, 1000)
    plt.figure(figsize=(14, 6))
    for fitted in fitted_frames:
        plt.plot(fitted[:max_plot_samples], linewidth=0.7, alpha=0.5)
    plt.xlabel("Sample index")
    plt.ylabel("ADC code")
    plt.title("ADC captures aligned to DAC TXT reference")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(timing_dir / "aligned_overlay.png", dpi=150)
    plt.close()

    plt.figure(figsize=(10, 5))
    plt.plot(
        summary_df["frame"],
        summary_df["reference_start_index"],
        marker="o",
    )
    plt.xlabel("Frame")
    plt.ylabel("Reference start index")
    plt.title("ADC capture phase relative to DAC TXT reference")
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(timing_dir / "detected_lag.png", dpi=150)
    plt.close()

    print(summary_df[[
        "frame",
        "reference_start_index",
        "correlation",
        "reference_rmse_codes",
        "fitted_scale_dac_to_adc",
        "fitted_offset_codes",
    ]].to_string(index=False))

    # Select the timing frame with the highest correlation.
    best_result_index = int(
        np.argmax([
            result["correlation"]
            for result in timing_results
        ])
    )

    best_result = timing_results[best_result_index]
    best_reference = np.clip(
        np.rint(best_result["fitted_adc"]),
        -32768,
        32767,
    ).astype(np.int16)

    if best_reference.size != captures[best_result_index].size:
        raise RuntimeError(
            "Best reference length does not match its ADC capture length."
        )

    best_reference_file = timing_dir / "best_fitted_reference.csv"
    pd.DataFrame({
        "sample_index": np.arange(
            best_reference.size,
            dtype=np.int32,
        ),
        "reference_adc_code": best_reference,
    }).to_csv(best_reference_file, index=False)

    print(
        f"Best timing frame: {best_result_index + 1}, "
        f"correlation={best_result['correlation']:.6f}"
    )
    print(f"Saved FPGA reference: {best_reference_file}")

    return (
        timing_dir,
        summary_file,
        summary_df,
        best_reference,
    )
