"""Generate coherent pure-sine vectors for a DAC frequency-response sweep.

The vectors are intended for the AD9164 DPG path used by this repository:

* 2.600 GSPS DAC sample rate
* 16,640 samples per repeating vector (65 x 256)
* signed 16-bit text, one integer per line
* no dither and no DC offset

Every default vector has the same 32,729-code peak amplitude (-0.010 dBFS),
so differences in measured output voltage are frequency-response differences,
not differences in the commanded waveform.  Frequencies are placed on the
nearest coherent DFT bin and the exact generated frequency is recorded in the
JSON metadata and CSV manifest.  The default 90-degree phase places the first
sample at +32,729 for every frequency, making the peak code easy to verify.

Run from the repository root:

    python -m calibration_loop.sweep_frequency

Choose a different set of frequencies or amplitude with, for example:

    python -m calibration_loop.sweep_frequency --peak-code 16384 --out waveforms/frequency_sweep_half_scale
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


FS_DAC_HZ = 2_600_000_000.0
N_DAC_POINTS = 16_640
DPG_ALIGNMENT_SAMPLES = 256
INT16_MAX = 32_767
DEFAULT_PEAK_CODE = 32_729
DEFAULT_PHASE_DEG = 90.0

# All defaults are exact coherent bins for the default clock and vector length.
DEFAULT_FREQUENCIES_MHZ = [
    1.25,
    2.5,
    5.0,
    7.5,
    10.0,
    12.5,
    15.0,
    20.0,
    25.0,
    35.0,
    50.0,
    75.0,
    100.0,
    200.0,
    300.0,
    400.0,
    500.0,
    625.0,
    750.0,
    1000.0,
    1200.0,
]


def frequency_label(frequency_mhz: float) -> str:
    """Return a compact filename-safe frequency label."""
    return f"{frequency_mhz:.9f}".rstrip("0").rstrip(".").replace(".", "p")


def coherent_tone(
    requested_hz: float,
    fs_dac_hz: float,
    n_dac_points: int,
    peak_code: int,
    phase_deg: float = DEFAULT_PHASE_DEG,
) -> tuple[np.ndarray, dict]:
    """Generate one sine on the nearest coherent bin and return its metadata."""
    if not 0.0 < requested_hz < fs_dac_hz / 2.0:
        raise ValueError(
            f"frequency must be above 0 Hz and below Nyquist ({fs_dac_hz / 2e6:g} MHz)"
        )
    if not 1 <= peak_code <= INT16_MAX:
        raise ValueError(f"peak_code must be in 1..{INT16_MAX}")
    if n_dac_points <= 0 or n_dac_points % DPG_ALIGNMENT_SAMPLES:
        raise ValueError(
            f"n_dac_points must be a positive multiple of {DPG_ALIGNMENT_SAMPLES}"
        )

    tone_bin = int(round(requested_hz * n_dac_points / fs_dac_hz))
    if not 1 <= tone_bin < n_dac_points // 2:
        raise ValueError("requested frequency maps outside the valid DFT bins")

    actual_hz = tone_bin * fs_dac_hz / n_dac_points
    n = np.arange(n_dac_points, dtype=np.float64)
    phase_rad = math.radians(phase_deg)
    ideal = peak_code * np.sin(2.0 * np.pi * tone_bin * n / n_dac_points + phase_rad)
    waveform = np.rint(ideal).astype(np.int16)

    meta = {
        "requested_frequency_hz": requested_hz,
        "actual_frequency_hz": actual_hz,
        "frequency_error_hz": actual_hz - requested_hz,
        "coherent_tone_bin": tone_bin,
        "bin_spacing_hz": fs_dac_hz / n_dac_points,
        "fs_dac_hz": fs_dac_hz,
        "n_dac_samples": n_dac_points,
        "peak_code_commanded": peak_code,
        "peak_dbfs": 20.0 * math.log10(peak_code / INT16_MAX),
        "phase_deg": phase_deg,
        "minimum_code": int(waveform.min()),
        "maximum_code": int(waveform.max()),
        "rms_codes": float(np.sqrt(np.mean(waveform.astype(np.float64) ** 2))),
        "mean_code": float(np.mean(waveform)),
        "clipping": bool(np.max(np.abs(waveform.astype(np.int32))) >= INT16_MAX),
        "dither_enabled": False,
        "format": "signed 16-bit integer, one sample per line",
    }
    return waveform, meta


def write_vector(out_dir: Path, waveform: np.ndarray, meta: dict) -> dict:
    """Write one DPG TXT vector and its JSON sidecar."""
    actual_mhz = meta["actual_frequency_hz"] / 1e6
    peak_code = meta["peak_code_commanded"]
    rate_label = (
        f"{meta['fs_dac_hz'] / 1e9:.9f}"
        .rstrip("0")
        .rstrip(".")
        .replace(".", "p")
    )
    stem = f"sine_{frequency_label(actual_mhz)}MHz_{rate_label}GSPS_amp{peak_code}"
    txt_path = out_dir / f"{stem}.txt"
    json_path = out_dir / f"{stem}.json"

    np.savetxt(txt_path, waveform, fmt="%d")
    json_path.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")

    return {
        "requested_frequency_mhz": meta["requested_frequency_hz"] / 1e6,
        "actual_frequency_mhz": actual_mhz,
        "frequency_error_hz": meta["frequency_error_hz"],
        "tone_bin": meta["coherent_tone_bin"],
        "peak_code": peak_code,
        "peak_dbfs": meta["peak_dbfs"],
        "minimum_code": meta["minimum_code"],
        "maximum_code": meta["maximum_code"],
        "txt": str(txt_path),
        "json": str(json_path),
    }


def generate_sweep(
    frequencies_mhz: list[float],
    out_dir: str | Path,
    peak_code: int = DEFAULT_PEAK_CODE,
    fs_dac_hz: float = FS_DAC_HZ,
    n_dac_points: int = N_DAC_POINTS,
    phase_deg: float = DEFAULT_PHASE_DEG,
) -> list[dict]:
    """Generate all requested vectors and return their manifest rows."""
    output = Path(out_dir)
    output.mkdir(parents=True, exist_ok=True)

    rows = []
    for frequency_mhz in frequencies_mhz:
        waveform, meta = coherent_tone(
            frequency_mhz * 1e6,
            fs_dac_hz,
            n_dac_points,
            peak_code,
            phase_deg,
        )
        row = write_vector(output, waveform, meta)
        rows.append(row)
        print(
            f"{frequency_mhz:9.3f} MHz requested -> "
            f"{row['actual_frequency_mhz']:9.6f} MHz, "
            f"bin {row['tone_bin']:4d}, codes "
            f"{row['minimum_code']:+6d}..{row['maximum_code']:+6d}"
        )

    manifest = {
        "purpose": "AD9164 full-scale output-amplitude versus frequency sweep",
        "fs_dac_hz": fs_dac_hz,
        "n_dac_points": n_dac_points,
        "bin_spacing_hz": fs_dac_hz / n_dac_points,
        "peak_code": peak_code,
        "peak_dbfs": 20.0 * math.log10(peak_code / INT16_MAX),
        "phase_deg": phase_deg,
        "dither_enabled": False,
        "termination_ohms": 50,
        "waveforms": rows,
    }
    (output / "frequency_sweep_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
    )

    fieldnames = [
        "requested_frequency_mhz",
        "actual_frequency_mhz",
        "frequency_error_hz",
        "tone_bin",
        "peak_code",
        "peak_dbfs",
        "minimum_code",
        "maximum_code",
        "txt",
        "json",
    ]
    with (output / "frequency_sweep_manifest.csv").open(
        "w", newline="", encoding="utf-8"
    ) as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    measurement_fields = [
        "actual_frequency_mhz",
        "peak_code",
        "measured_vpp",
        "fundamental_dbm",
        "relative_db",
        "instrument",
        "notes",
    ]
    with (output / "frequency_sweep_measurements.csv").open(
        "w", newline="", encoding="utf-8"
    ) as fh:
        writer = csv.DictWriter(fh, fieldnames=measurement_fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(
                {
                    "actual_frequency_mhz": row["actual_frequency_mhz"],
                    "peak_code": row["peak_code"],
                }
            )

    return rows


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--frequencies-mhz",
        nargs="+",
        type=float,
        default=DEFAULT_FREQUENCIES_MHZ,
        help="requested output frequencies in MHz",
    )
    parser.add_argument(
        "--peak-code",
        type=int,
        default=DEFAULT_PEAK_CODE,
        help="sine peak amplitude in signed DAC codes (default: %(default)s)",
    )
    parser.add_argument(
        "--out",
        default="waveforms/frequency_sweep_full_scale",
        help="output directory (default: %(default)s)",
    )
    parser.add_argument("--fs-dac", type=float, default=FS_DAC_HZ, help="DAC sample rate in Hz")
    parser.add_argument(
        "--n-dac-points",
        type=int,
        default=N_DAC_POINTS,
        help="samples in each repeating DPG vector",
    )
    parser.add_argument(
        "--phase-deg",
        type=float,
        default=DEFAULT_PHASE_DEG,
        help="initial sine phase in degrees (default: %(default)s)",
    )
    args = parser.parse_args(argv)

    if len(set(args.frequencies_mhz)) != len(args.frequencies_mhz):
        parser.error("--frequencies-mhz contains duplicates")

    try:
        rows = generate_sweep(
            args.frequencies_mhz,
            args.out,
            peak_code=args.peak_code,
            fs_dac_hz=args.fs_dac,
            n_dac_points=args.n_dac_points,
            phase_deg=args.phase_deg,
        )
    except ValueError as exc:
        parser.error(str(exc))

    print(f"\nGenerated {len(rows)} pure-sine vectors in {Path(args.out).resolve()}")
    print("All vectors use the same commanded amplitude; dither is disabled.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
