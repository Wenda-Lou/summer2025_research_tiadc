from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
from tkinter import filedialog, messagebox

from .receive_data import receive_adc_data
from .frame import reconstruct_adc_bytes

PROJECT_DIR = Path(__file__).resolve().parents[1]
SAVE_DIR = PROJECT_DIR / 'adc_data'
SAVE_DIR.mkdir(parents=True, exist_ok=True)


def receive_ifc_sweep(
    bind_ip="0.0.0.0",
    port=6666,
    expected_packets=8,
    packet_size=512,
    timeout=15.0,
    reconstruct=True,
    offset_threshold_codes=2.0,
):
    """
    Receive seven IFC sweep captures and calculate DC-offset metrics.

    Metrics:
        sample_sum:
            Sum of all reconstructed ADC samples.

        mean:
            Signed average ADC code.

        absolute_offset:
            Absolute value of the mean.

        pos_mean / neg_mean:
            Mean values of the positive and reconstructed-negative
            branches before interleaving. These help determine whether
            the observed offset is caused by branch asymmetry.

        needs_calibration:
            True when absolute_offset exceeds offset_threshold_codes.
    """

    ifc_values = [
        "2.04",
        "1.93",
        "1.81",
        "1.70",
        "1.59",
        "1.47",
        "1.36",
    ]

    sweep_dir = SAVE_DIR / (
        f"ifc_sweep_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    sweep_dir.mkdir(parents=True, exist_ok=True)

    results = []
    saved_files = []

    def empty_result(index, ifc, status):
        return {
            "index": index,
            "ifc_vpp": float(ifc),
            "sample_count": None,
            "sample_sum": None,
            "mean": None,
            "absolute_offset": None,
            "pos_mean": None,
            "neg_mean": None,
            "branch_mean_difference": None,
            "needs_calibration": None,
            "min": None,
            "max": None,
            "peak": None,
            "rms": None,
            "csv_file": status,
        }

    for index, ifc in enumerate(ifc_values, start=1):
        print(
            f"\nWaiting for sweep frame {index}/{len(ifc_values)}, "
            f"IFC = {ifc} Vpp"
        )

        csv_file = receive_adc_data(
            bind_ip=bind_ip,
            port=port,
            expected_packets=expected_packets,
            packet_size=packet_size,
            timeout=timeout,
        )

        if csv_file is None:
            results.append(
                empty_result(index, ifc, "TIMEOUT")
            )
            continue

        try:
            raw_df = pd.read_csv(csv_file)

            if "byte" not in raw_df.columns:
                raise ValueError(
                    f"Raw capture does not contain a 'byte' column: {csv_file}"
                )

            raw = raw_df["byte"].to_numpy(dtype=np.uint8)

            if raw.size < 2:
                raise ValueError(
                    f"Capture for IFC {ifc} Vpp is too short."
                )

            pos_mean = np.nan
            neg_mean = np.nan
            branch_mean_difference = np.nan

            if reconstruct:
                adc = reconstruct_adc_bytes(raw.tobytes())

                # Recover branch statistics from the same validated word layout.
                raw_even = raw[:-1] if raw.size % 2 else raw
                words14 = (raw_even.view("<i2") >> 2).astype(np.int16)
                if words14.size <= 8:
                    raise ValueError(
                        f"Capture for IFC {ifc} Vpp contains too few samples."
                    )
                words14 = words14[:-8]
                words14 = words14[: words14.size - (words14.size % 8)]
                grouped = words14.reshape(-1, 8)

                pos = grouped[:, :4].reshape(-1).astype(np.int32)
                neg = (-grouped[:, 4:]).reshape(-1).astype(np.int32)

                pos_mean = float(np.mean(pos))
                neg_mean = float(np.mean(neg))
                branch_mean_difference = float(pos_mean - neg_mean)
            else:
                raw_even = raw[:-1] if raw.size % 2 else raw
                adc = (raw_even.view("<i2") >> 2).astype(np.int32)

            if adc.size == 0:
                raise ValueError(
                    f"No ADC samples available for IFC {ifc} Vpp."
                )

            adc_float = adc.astype(np.float64)

            sample_count = int(adc.size)
            sample_sum = float(np.sum(adc_float))
            mean_val = float(np.mean(adc_float))
            absolute_offset = float(abs(mean_val))

            min_val = int(np.min(adc))
            max_val = int(np.max(adc))
            peak_val = float((max_val - min_val) / 2.0)

            rms_val = float(
                np.sqrt(np.mean(adc_float ** 2))
            )

            needs_calibration = bool(
                absolute_offset > offset_threshold_codes
            )

            out_csv = sweep_dir / (
                f"sweep_{index:02d}_"
                f"ifc_{ifc.replace('.', 'p')}_vpp.csv"
            )

            pd.DataFrame({
                "sample_index": np.arange(
                    adc.size,
                    dtype=np.int32,
                ),
                "adc_code": adc,
                "ifc_vpp": float(ifc),
                "sweep_index": index,
            }).to_csv(out_csv, index=False)

            saved_files.append(out_csv)

            results.append({
                "index": index,
                "ifc_vpp": float(ifc),
                "sample_count": sample_count,
                "sample_sum": sample_sum,
                "mean": mean_val,
                "absolute_offset": absolute_offset,
                "pos_mean": pos_mean,
                "neg_mean": neg_mean,
                "branch_mean_difference": branch_mean_difference,
                "needs_calibration": needs_calibration,
                "min": min_val,
                "max": max_val,
                "peak": peak_val,
                "rms": rms_val,
                "csv_file": str(out_csv),
            })

            status_text = (
                "CALIBRATION NEEDED"
                if needs_calibration
                else "PASS"
            )

            print(
                f"IFC {ifc} Vpp: "
                f"sum={sample_sum:.1f}, "
                f"mean={mean_val:.6f}, "
                f"|offset|={absolute_offset:.6f}, "
                f"pos mean={pos_mean:.6f}, "
                f"neg mean={neg_mean:.6f}, "
                f"status={status_text}"
            )

        except Exception as exc:
            print(
                f"Processing failed for IFC {ifc} Vpp: {exc}"
            )

            results.append(
                empty_result(index, ifc, "PROCESSING_ERROR")
            )

        finally:
            # Remove the temporary raw-byte CSV.
            try:
                Path(csv_file).unlink()
            except OSError:
                pass

    summary_df = pd.DataFrame(results)

    summary_file = sweep_dir / "ifc_sweep_summary.csv"
    summary_df.to_csv(summary_file, index=False)

    return (
        sweep_dir,
        summary_file,
        summary_df,
        saved_files,
    )

