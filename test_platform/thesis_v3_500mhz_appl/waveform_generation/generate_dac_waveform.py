#!/usr/bin/env python3
"""
Generate a seamless periodic TXT waveform for AD9164 DPG Downloader.

Edit only the USER SETTINGS section below, then run:

    python generate_dac_waveform.py

The output uses the AD9164-compatible plain-text format: one signed 16-bit
integer per line, with a record length divisible by 256.
"""

from __future__ import annotations

import argparse
import json
import math
from fractions import Fraction
from pathlib import Path

import numpy as np


# =====================================================================
# USER SETTINGS — edit these values
# =====================================================================

# Desired main tone frequency in Hz. The generator selects the nearest
# coherent bin for the configured output length.
TONE_FREQUENCY_HZ = 200_000_000.0

# Sine-wave peak level in dBFS.
# Examples:
#   -0.5 dBFS = very close to full scale
#   -1.5 dBFS = recommended starting point
#   -3.0 dBFS = more headroom
#   -6.0 dBFS = half-scale peak amplitude
SINE_PEAK_DBFS = -1.5

# Initial phase of the sine wave.
SINE_PHASE_DEG = 0.0

# The only switch needed to select the output waveform:
#   False = coherent sine only
#   True  = coherent sine plus balanced impulse dither
ENABLE_DITHER = True

# Impulse-dither settings copied from calibration_loop/dither.py. The event
# interval is a physical time; pulse geometry remains in DAC samples.
DITHER_EVENT_PERIOD_SECONDS = 100.0e-9
DITHER_POSITION_DAC = 96
DITHER_EDGE_DAC = 16
DITHER_TOP_DAC = 32
DITHER_SCALE_LSB = 2_000.0
DITHER_SEED = 20_260_725

# Reserve a small number of codes to protect against rounding/clipping.
HEADROOM_CODES = 16


# =====================================================================
# FIXED PROJECT SETTINGS — normally do not change
# =====================================================================

ADC_SAMPLE_RATE_HZ = 1_300_000_000.0
DAC_SAMPLE_RATE_HZ = 2_600_000_000.0
# Each 2032-word DMA payload reconstructs to 1016 chronological samples per
# ADC channel; FFT-bin metadata is expressed on that per-channel axis.
ADC_FRAME_SAMPLES = 1_016
DAC_TO_ADC_RATE_RATIO = DAC_SAMPLE_RATE_HZ / ADC_SAMPLE_RATE_HZ
GENERATED_SAMPLES_DIR = Path(__file__).resolve().parent / "generated_samples"

# Derive the DAC- and ADC-domain event spacing from the physical interval so a
# future clock update cannot leave a stale sample count behind.
DITHER_PERIOD_DAC = int(round(
    DITHER_EVENT_PERIOD_SECONDS * DAC_SAMPLE_RATE_HZ
))
DITHER_PERIOD_ADC = int(round(
    DITHER_EVENT_PERIOD_SECONDS * ADC_SAMPLE_RATE_HZ
))
if not math.isclose(
    DITHER_PERIOD_DAC / DAC_SAMPLE_RATE_HZ,
    DITHER_EVENT_PERIOD_SECONDS,
    rel_tol=0.0,
    abs_tol=1.0e-18,
) or not math.isclose(
    DITHER_PERIOD_ADC / ADC_SAMPLE_RATE_HZ,
    DITHER_EVENT_PERIOD_SECONDS,
    rel_tol=0.0,
    abs_tol=1.0e-18,
):
    raise RuntimeError(
        "Dither event period must map to whole DAC and ADC samples."
    )
DAC_ADC_RATIO_FRACTION = Fraction(
    int(round(DAC_SAMPLE_RATE_HZ)),
    int(round(ADC_SAMPLE_RATE_HZ)),
)
DAC_RATE_NUMERATOR = DAC_ADC_RATIO_FRACTION.numerator
DAC_RATE_DENOMINATOR = DAC_ADC_RATIO_FRACTION.denominator
event_period_adc_numerator = DITHER_PERIOD_DAC * DAC_RATE_DENOMINATOR
if event_period_adc_numerator % DAC_RATE_NUMERATOR != 0:
    raise RuntimeError(
        "Dither period must map to a whole number of ADC samples."
    )
if DITHER_PERIOD_ADC != event_period_adc_numerator // DAC_RATE_NUMERATOR:
    raise RuntimeError("Dither spacing derivations disagree.")
DAC_FILE_ALIGNMENT_SAMPLES = 256
# The loop length is aligned for DPG download, contains whole dither periods,
# and closes on an ADC sample boundary. Double it only if polarity balancing
# would otherwise leave an odd event count.
NUM_SAMPLES = math.lcm(
    DAC_FILE_ALIGNMENT_SAMPLES,
    DITHER_PERIOD_DAC,
    DAC_RATE_NUMERATOR,
)
DITHER_NUM_SAMPLES = (
    NUM_SAMPLES
    if (NUM_SAMPLES // DITHER_PERIOD_DAC) % 2 == 0
    else 2 * NUM_SAMPLES
)
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


def raised_cosine_impulse(
    sample_offsets: np.ndarray,
    edge_samples: int,
    top_samples: int,
) -> np.ndarray:
    """Return the calibration-loop raised-cosine impulse on the DAC grid."""
    t = np.asarray(sample_offsets, dtype=np.float64)
    total = 2 * edge_samples + top_samples
    impulse = np.zeros_like(t)

    rise = (t >= 0.0) & (t < edge_samples)
    top = (t >= edge_samples) & (t < edge_samples + top_samples)
    fall = (t >= edge_samples + top_samples) & (t < total)

    impulse[rise] = 0.5 * (
        1.0 - np.cos(np.pi * t[rise] / edge_samples)
    )
    impulse[top] = 1.0
    impulse[fall] = 0.5 * (
        1.0 - np.cos(np.pi * (total - t[fall]) / edge_samples)
    )
    return impulse


def balanced_polarity_sequence(
    event_count: int,
    seed: int,
) -> np.ndarray:
    """Return a deterministic sequence with equal numbers of +1 and -1."""
    if event_count <= 0 or event_count % 2:
        raise ValueError(
            "The impulse-dither event count must be positive and even."
        )

    half = event_count // 2
    polarity = np.concatenate([np.ones(half), -np.ones(half)])
    rng = np.random.RandomState(seed)
    rng.shuffle(polarity)
    return polarity.astype(np.float64)


def generate_periodic_impulse_dither(
    num_samples: int,
    period_samples: int,
    position_samples: int,
    edge_samples: int,
    top_samples: int,
    amplitude_codes: float,
    seed: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Build the calibration-loop balanced impulse dither at the DAC rate."""
    if period_samples <= 0 or num_samples % period_samples:
        raise ValueError(
            "Dither waveform length must contain whole dither periods."
        )
    if edge_samples <= 0 or top_samples <= 0:
        raise ValueError("Dither edge and flat-top lengths must be positive.")
    if position_samples < 0:
        raise ValueError("Dither position cannot be negative.")

    pulse_samples = 2 * edge_samples + top_samples
    if position_samples + pulse_samples > period_samples:
        raise ValueError("The impulse must fit completely inside its period.")
    if amplitude_codes < 0.0:
        raise ValueError("Dither amplitude cannot be negative.")

    event_count = num_samples // period_samples
    polarity = balanced_polarity_sequence(event_count, seed)
    pulse_shape = raised_cosine_impulse(
        np.arange(pulse_samples, dtype=np.float64),
        edge_samples,
        top_samples,
    )

    dither = np.zeros(num_samples, dtype=np.float64)
    for event_index, sign in enumerate(polarity):
        start = event_index * period_samples + position_samples
        stop = start + pulse_samples
        dither[start:stop] = sign * amplitude_codes * pulse_shape

    return dither, polarity


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


def frequency_filename_label(frequency_hz: float) -> str:
    """Return a filesystem-friendly MHz label from the requested frequency."""
    frequency_mhz = frequency_hz / 1e6
    label = f"{frequency_mhz:.9f}".rstrip("0").rstrip(".")
    return label.replace("-", "minus").replace(".", "p")


def output_path_for_frequency(
    frequency_hz: float,
    dither_enabled: bool,
) -> Path:
    """Return a frequency-named path without replacing an existing bundle."""
    frequency_label = frequency_filename_label(frequency_hz)
    sample_rate_label = (
        f"{DAC_SAMPLE_RATE_HZ / 1e9:.9f}".rstrip("0").rstrip(".")
        .replace(".", "p")
    )
    mode_suffix = "impulse_dither" if dither_enabled else "non_dither"
    stem = f"sine_{frequency_label}MHz_{sample_rate_label}GSPS_{mode_suffix}"

    sequence_number = 0
    while True:
        numbered_suffix = (
            "" if sequence_number == 0 else f"_{sequence_number:03d}"
        )
        txt_file = GENERATED_SAMPLES_DIR / (
            f"{stem}{numbered_suffix}.txt"
        )
        companion_files = (
            txt_file,
            txt_file.with_suffix(".png"),
            txt_file.with_suffix(txt_file.suffix + ".json"),
        )
        if not any(path.exists() for path in companion_files):
            return txt_file
        sequence_number += 1


def plot_generated_txt(
    txt_file: Path,
    sample_rate_hz: float,
    dither_enabled: bool,
) -> Path:
    """Read the generated TXT and save a full-record and detail PNG plot."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    samples = np.loadtxt(txt_file, dtype=np.int16, ndmin=1)
    if samples.size == 0:
        raise RuntimeError(f"Cannot plot empty waveform file: {txt_file}")

    sample_indices = np.arange(samples.size)
    time_us = sample_indices / sample_rate_hz * 1e6
    detail_count = min(samples.size, 1_024)
    mode_label = "Impulse dither enabled" if dither_enabled else "No dither"

    figure, (full_axis, detail_axis) = plt.subplots(
        2,
        1,
        figsize=(12, 7),
        constrained_layout=True,
    )
    full_axis.plot(time_us, samples, linewidth=0.5)
    full_axis.set_title(f"Generated DAC waveform - {mode_label}")
    full_axis.set_xlabel("Time (us)")
    full_axis.set_ylabel("DAC code")
    full_axis.grid(True, alpha=0.3)

    detail_axis.plot(
        sample_indices[:detail_count],
        samples[:detail_count],
        linewidth=0.8,
    )
    detail_axis.set_title(f"First {detail_count} samples")
    detail_axis.set_xlabel("Sample index")
    detail_axis.set_ylabel("DAC code")
    detail_axis.grid(True, alpha=0.3)

    png_file = txt_file.with_suffix(".png")
    figure.savefig(png_file, dpi=150)
    plt.close(figure)
    return png_file


def main() -> None:
    num_samples = DITHER_NUM_SAMPLES if ENABLE_DITHER else NUM_SAMPLES

    tone_bin = nearest_coherent_bin(
        requested_frequency_hz=TONE_FREQUENCY_HZ,
        sample_rate_hz=DAC_SAMPLE_RATE_HZ,
        num_samples=num_samples,
    )

    actual_tone_hz = (
        tone_bin * DAC_SAMPLE_RATE_HZ / num_samples
    )
    output_file = output_path_for_frequency(
        TONE_FREQUENCY_HZ,
        ENABLE_DITHER,
    )

    if num_samples % DAC_FILE_ALIGNMENT_SAMPLES != 0:
        raise RuntimeError(
            f"DAC file length must be divisible by "
            f"{DAC_FILE_ALIGNMENT_SAMPLES}."
        )

    sine_peak_codes = (
        INT16_MAX * 10.0 ** (SINE_PEAK_DBFS / 20.0)
    )

    sine = generate_coherent_sine(
        num_samples=num_samples,
        tone_bin=tone_bin,
        peak_amplitude=sine_peak_codes,
        phase_deg=SINE_PHASE_DEG,
    )

    if ENABLE_DITHER:
        dither, polarity = generate_periodic_impulse_dither(
            num_samples=num_samples,
            period_samples=DITHER_PERIOD_DAC,
            position_samples=DITHER_POSITION_DAC,
            edge_samples=DITHER_EDGE_DAC,
            top_samples=DITHER_TOP_DAC,
            amplitude_codes=DITHER_SCALE_LSB,
            seed=DITHER_SEED,
        )
    else:
        dither = np.zeros(num_samples, dtype=np.float64)
        polarity = np.empty(0, dtype=np.float64)

    combined = sine + dither
    combined, applied_scale = scale_to_avoid_clipping(
        combined,
        HEADROOM_CODES,
    )

    waveform = np.rint(combined).astype(np.int16)

    if waveform.size != num_samples:
        raise RuntimeError("Generated waveform has the wrong sample count.")

    output_file.parent.mkdir(parents=True, exist_ok=True)
    np.savetxt(output_file, waveform, fmt="%d")
    plot_file = plot_generated_txt(
        txt_file=output_file,
        sample_rate_hz=DAC_SAMPLE_RATE_HZ,
        dither_enabled=ENABLE_DITHER,
    )

    metadata = {
        "output_file": str(output_file),
        "plot_file": str(plot_file),
        "dac_sample_rate_hz": DAC_SAMPLE_RATE_HZ,
        "adc_sample_rate_hz": ADC_SAMPLE_RATE_HZ,
        "dac_to_adc_rate_ratio": DAC_TO_ADC_RATE_RATIO,
        "dac_adc_ratio_fraction": (
            f"{DAC_RATE_NUMERATOR}/{DAC_RATE_DENOMINATOR}"
        ),
        "dither_period_adc": DITHER_PERIOD_ADC if ENABLE_DITHER else None,
        "dither_event_period_seconds": (
            DITHER_EVENT_PERIOD_SECONDS if ENABLE_DITHER else None
        ),
        "adc_frame_samples": ADC_FRAME_SAMPLES,
        "periodic_dac_file_samples": num_samples,
        "dac_file_alignment_samples": DAC_FILE_ALIGNMENT_SAMPLES,
        "adc_tone_bin": actual_tone_hz * ADC_FRAME_SAMPLES /
        ADC_SAMPLE_RATE_HZ,
        "num_samples": num_samples,
        "requested_tone_hz": TONE_FREQUENCY_HZ,
        "actual_tone_hz": actual_tone_hz,
        "tone_bin": tone_bin,
        "bin_spacing_hz": DAC_SAMPLE_RATE_HZ / num_samples,
        "sine_peak_dbfs": SINE_PEAK_DBFS,
        "sine_phase_deg": SINE_PHASE_DEG,
        "dither_enabled": ENABLE_DITHER,
        "dither_type": (
            "balanced random-polarity raised-cosine impulse"
            if ENABLE_DITHER else None
        ),
        "dither_period_dac": DITHER_PERIOD_DAC if ENABLE_DITHER else None,
        "dither_position_dac": (
            DITHER_POSITION_DAC if ENABLE_DITHER else None
        ),
        "dither_edge_dac": DITHER_EDGE_DAC if ENABLE_DITHER else None,
        "dither_top_dac": DITHER_TOP_DAC if ENABLE_DITHER else None,
        "dither_scale_lsb": DITHER_SCALE_LSB if ENABLE_DITHER else None,
        "dither_seed": DITHER_SEED if ENABLE_DITHER else None,
        "dither_event_count": int(polarity.size),
        "dither_polarity_sum": float(np.sum(polarity)),
        "dither_polarity": polarity.astype(int).tolist(),
        "anti_clipping_scale": applied_scale,
        "minimum_code": int(waveform.min()),
        "maximum_code": int(waveform.max()),
        "mean_code": float(np.mean(waveform.astype(np.float64))),
        "rms_code": float(
            np.sqrt(np.mean(waveform.astype(np.float64) ** 2))
        ),
        "periodic_by_construction": True,
    }

    metadata_file = output_file.with_suffix(
        output_file.suffix + ".json"
    )
    metadata_file.write_text(
        json.dumps(metadata, indent=2),
        encoding="utf-8",
    )

    print("Waveform generated successfully")
    print(f"TXT file            : {output_file}")
    print(f"PNG plot            : {plot_file}")
    print(f"Metadata file       : {metadata_file}")
    print(f"Number of samples   : {num_samples}")
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
                main()
        else:
            main()
    except Exception as exc:
        raise SystemExit(f"Error: {exc}") from exc
