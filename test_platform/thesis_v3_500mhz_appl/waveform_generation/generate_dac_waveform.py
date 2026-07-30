#!/usr/bin/env python3
"""
Generate a seamless periodic TXT waveform for AD9164 DPG Downloader.

Edit only the USER SETTINGS section below, then run:

    python generate_dac_waveform.py

The output uses the AD9164-compatible plain-text format: one signed 16-bit
integer per line, with a record length divisible by 256.
"""

from __future__ import annotations

import json
import math
import argparse
from pathlib import Path

import numpy as np


# =====================================================================
# USER SETTINGS — edit these values
# =====================================================================

# Desired main tone frequency in Hz. The generator selects the nearest
# coherent bin for the configured output length.
TONE_FREQUENCY_HZ = 100_000_000.0

# Sine-wave peak level in dBFS.
# Examples:
#   -0.5 dBFS = very close to full scale
#   -1.5 dBFS = recommended starting point
#   -3.0 dBFS = more headroom
#   -6.0 dBFS = half-scale peak amplitude
SINE_PEAK_DBFS = -1.5

# Initial phase of the sine wave.
SINE_PHASE_DEG = 0.0

# Enable or disable periodic dither.
ENABLE_DITHER = False

# Dither RMS level in dBFS. Used only when ENABLE_DITHER is True.
DITHER_RMS_DBFS = -35.0

# Dither frequency band in Hz.
DITHER_LOW_HZ = 5e6
DITHER_HIGH_HZ = 600e6

# Fixed seed makes the dither exactly reproducible.
DITHER_SEED = 1234

# Output TXT filename.
OUTPUT_FILE = Path("sine_100MHz_2p6GSPS.txt")

# Reserve a small number of codes to protect against rounding/clipping.
HEADROOM_CODES = 16


# =====================================================================
# FIXED PROJECT SETTINGS — normally do not change
# =====================================================================

ADC_SAMPLE_RATE_HZ = 1_450_000_000.0
DAC_SAMPLE_RATE_HZ = 2_600_000_000.0
ADC_FRAME_SAMPLES = 2_032
DAC_TO_ADC_RATE_RATIO = DAC_SAMPLE_RATE_HZ / ADC_SAMPLE_RATE_HZ

# The ADC and DAC rates are not an integer ratio. Generate a periodic file
# whose length is compatible with both the analysis-frame bookkeeping and the
# AD9164 downloader's 256-sample alignment requirement.
DAC_FILE_ALIGNMENT_SAMPLES = 256
NUM_SAMPLES = math.lcm(ADC_FRAME_SAMPLES, DAC_FILE_ALIGNMENT_SAMPLES)
INT16_MIN = -32_768
INT16_MAX = 32_767


def nearest_coherent_bin(
    requested_frequency_hz: float,
    sample_rate_hz: float,
    num_samples: int,
) -> int:
    if not 0.0 < requested_frequency_hz < sample_rate_hz / 2.0:
        raise ValueError(
            "TONE_FREQUENCY_HZ must be above 0 Hz and below DAC Nyquist."
        )

    tone_bin = int(
        round(requested_frequency_hz * num_samples / sample_rate_hz)
    )

    if not 1 <= tone_bin < num_samples // 2:
        raise ValueError("Requested tone maps outside the valid DFT bins.")

    return tone_bin


def generate_coherent_sine(
    num_samples: int,
    tone_bin: int,
    peak_amplitude: float,
    phase_deg: float,
) -> np.ndarray:
    n = np.arange(num_samples, dtype=np.float64)
    phase_rad = np.deg2rad(phase_deg)

    return peak_amplitude * np.sin(
        2.0 * np.pi * tone_bin * n / num_samples + phase_rad
    )


def dbfs_rms_to_codes(dbfs: float) -> float:
    full_scale_sine_rms = INT16_MAX / math.sqrt(2.0)
    return full_scale_sine_rms * 10.0 ** (dbfs / 20.0)


def generate_periodic_bandlimited_dither(
    num_samples: int,
    sample_rate_hz: float,
    rms_amplitude: float,
    low_frequency_hz: float,
    high_frequency_hz: float,
    seed: int,
) -> np.ndarray:
    if rms_amplitude <= 0.0:
        return np.zeros(num_samples, dtype=np.float64)

    nyquist_hz = sample_rate_hz / 2.0

    if low_frequency_hz < 0.0:
        raise ValueError("DITHER_LOW_HZ cannot be negative.")

    if not low_frequency_hz < high_frequency_hz <= nyquist_hz:
        raise ValueError(
            "DITHER_HIGH_HZ must exceed DITHER_LOW_HZ and not exceed Nyquist."
        )

    bin_spacing_hz = sample_rate_hz / num_samples
    first_bin = max(1, int(math.ceil(low_frequency_hz / bin_spacing_hz)))
    last_bin = min(
        num_samples // 2 - 1,
        int(math.floor(high_frequency_hz / bin_spacing_hz)),
    )

    if first_bin > last_bin:
        raise ValueError("The selected dither band contains no coherent bins.")

    rng = np.random.default_rng(seed)
    spectrum = np.zeros(num_samples // 2 + 1, dtype=np.complex128)

    bin_count = last_bin - first_bin + 1
    spectrum[first_bin:last_bin + 1] = (
        rng.normal(size=bin_count)
        + 1j * rng.normal(size=bin_count)
    )

    spectrum[0] = 0.0
    spectrum[-1] = 0.0

    dither = np.fft.irfft(spectrum, n=num_samples)
    dither -= np.mean(dither)

    current_rms = float(np.sqrt(np.mean(dither**2)))
    if current_rms <= 0.0:
        raise RuntimeError("Generated dither has zero RMS.")

    return dither * (rms_amplitude / current_rms)


def scale_to_avoid_clipping(
    waveform: np.ndarray,
    headroom_codes: int,
) -> tuple[np.ndarray, float]:
    allowed_peak = INT16_MAX - headroom_codes

    if allowed_peak <= 0:
        raise ValueError("HEADROOM_CODES is too large.")

    measured_peak = float(np.max(np.abs(waveform)))

    if measured_peak == 0.0:
        return waveform.copy(), 1.0

    scale = min(1.0, allowed_peak / measured_peak)
    return waveform * scale, scale


def main() -> None:
    tone_bin = nearest_coherent_bin(
        requested_frequency_hz=TONE_FREQUENCY_HZ,
        sample_rate_hz=DAC_SAMPLE_RATE_HZ,
        num_samples=NUM_SAMPLES,
    )

    actual_tone_hz = (
        tone_bin * DAC_SAMPLE_RATE_HZ / NUM_SAMPLES
    )

    if NUM_SAMPLES % DAC_FILE_ALIGNMENT_SAMPLES != 0:
        raise RuntimeError(
            f"DAC file length must be divisible by "
            f"{DAC_FILE_ALIGNMENT_SAMPLES}."
        )

    sine_peak_codes = (
        INT16_MAX * 10.0 ** (SINE_PEAK_DBFS / 20.0)
    )

    sine = generate_coherent_sine(
        num_samples=NUM_SAMPLES,
        tone_bin=tone_bin,
        peak_amplitude=sine_peak_codes,
        phase_deg=SINE_PHASE_DEG,
    )

    if ENABLE_DITHER:
        dither_rms_codes = dbfs_rms_to_codes(DITHER_RMS_DBFS)

        dither = generate_periodic_bandlimited_dither(
            num_samples=NUM_SAMPLES,
            sample_rate_hz=DAC_SAMPLE_RATE_HZ,
            rms_amplitude=dither_rms_codes,
            low_frequency_hz=DITHER_LOW_HZ,
            high_frequency_hz=DITHER_HIGH_HZ,
            seed=DITHER_SEED,
        )
    else:
        dither = np.zeros(NUM_SAMPLES, dtype=np.float64)

    combined = sine + dither
    combined, applied_scale = scale_to_avoid_clipping(
        combined,
        HEADROOM_CODES,
    )

    waveform = np.rint(combined).astype(np.int16)

    if waveform.size != NUM_SAMPLES:
        raise RuntimeError("Generated waveform has the wrong sample count.")

    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(OUTPUT_FILE, waveform, fmt="%d")

    metadata = {
        "output_file": str(OUTPUT_FILE),
        "dac_sample_rate_hz": DAC_SAMPLE_RATE_HZ,
        "adc_sample_rate_hz": ADC_SAMPLE_RATE_HZ,
        "dac_to_adc_rate_ratio": DAC_TO_ADC_RATE_RATIO,
        "adc_frame_samples": ADC_FRAME_SAMPLES,
        "periodic_dac_file_samples": NUM_SAMPLES,
        "dac_file_alignment_samples": DAC_FILE_ALIGNMENT_SAMPLES,
        "adc_tone_bin": actual_tone_hz * ADC_FRAME_SAMPLES /
        ADC_SAMPLE_RATE_HZ,
        "num_samples": NUM_SAMPLES,
        "requested_tone_hz": TONE_FREQUENCY_HZ,
        "actual_tone_hz": actual_tone_hz,
        "tone_bin": tone_bin,
        "bin_spacing_hz": DAC_SAMPLE_RATE_HZ / NUM_SAMPLES,
        "sine_peak_dbfs": SINE_PEAK_DBFS,
        "sine_phase_deg": SINE_PHASE_DEG,
        "dither_enabled": ENABLE_DITHER,
        "dither_rms_dbfs": DITHER_RMS_DBFS if ENABLE_DITHER else None,
        "dither_low_hz": DITHER_LOW_HZ if ENABLE_DITHER else None,
        "dither_high_hz": DITHER_HIGH_HZ if ENABLE_DITHER else None,
        "dither_seed": DITHER_SEED if ENABLE_DITHER else None,
        "anti_clipping_scale": applied_scale,
        "minimum_code": int(waveform.min()),
        "maximum_code": int(waveform.max()),
        "mean_code": float(np.mean(waveform.astype(np.float64))),
        "rms_code": float(
            np.sqrt(np.mean(waveform.astype(np.float64) ** 2))
        ),
        "periodic_by_construction": True,
    }

    metadata_file = OUTPUT_FILE.with_suffix(
        OUTPUT_FILE.suffix + ".json"
    )
    metadata_file.write_text(
        json.dumps(metadata, indent=2),
        encoding="utf-8",
    )

    print("Waveform generated successfully")
    print(f"TXT file            : {OUTPUT_FILE}")
    print(f"Metadata file       : {metadata_file}")
    print(f"Number of samples   : {NUM_SAMPLES}")
    print(f"Requested frequency : {TONE_FREQUENCY_HZ / 1e6:.9f} MHz")
    print(f"Actual frequency    : {actual_tone_hz / 1e6:.9f} MHz")
    print(f"Coherent tone bin   : {tone_bin}")
    print(f"Sine peak level     : {SINE_PEAK_DBFS:.3f} dBFS")
    print(f"Minimum code        : {waveform.min()}")
    print(f"Maximum code        : {waveform.max()}")
    print(f"Mean code           : {np.mean(waveform):.6f}")
    print(f"Scale applied       : {applied_scale:.9f}")
    print(f"Dither enabled      : {ENABLE_DITHER}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate one or more AD9164 sine-wave TXT files."
    )
    parser.add_argument(
        "--frequencies-mhz",
        nargs="+",
        type=float,
        help="Tone frequencies in MHz; defaults to the configured tone.",
    )
    args = parser.parse_args()

    try:
        if args.frequencies_mhz:
            for frequency_mhz in args.frequencies_mhz:
                TONE_FREQUENCY_HZ = frequency_mhz * 1e6
                frequency_label = f"{frequency_mhz:g}".replace(".", "p")
                OUTPUT_FILE = Path(
                    f"sine_{frequency_label}MHz_2p6GSPS.txt"
                )
                main()
        else:
            main()
    except Exception as exc:
        raise SystemExit(f"Error: {exc}") from exc
