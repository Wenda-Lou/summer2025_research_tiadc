"""
Offline replay of board DMA captures against a generated DAC waveform.

Reproduces the firmware dither fine-skew analysis chain in Python so the
rising/falling edge estimates can be diagnosed without reflashing:

    python -m calibration_loop.dither_replay \
        <waveform.txt> <capture.csv> [capture.csv ...]

Layout and estimators mirror the firmware:

- adc_reconstruct_channels()  (adc_frame.c): w0..w3 = A, w4..w7 = B,
  little-endian, signed 14-bit left-aligned, arithmetic >> 2.
- least-squares tone fit at the fitted bench frequency, residual after
  subtraction (calibration_fit_tone_refined equivalent).
- impulse cross-correlation event detection, polarity-weighted profile
  aggregation, Channel-A profile as local template, masked derivative
  projection for all/rising/falling skew (estimate_profile_skew).

The pulse-shape diagnosis is the primary output; the skew numbers are a
cross-check against the firmware "Dither rising/falling-edge" report.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

ADC_RATE_HZ = 1300000000.0
DAC_RATE_HZ = 2600000000.0
RATIO = DAC_RATE_HZ / ADC_RATE_HZ

FRAME_BYTES = 4096
VALID_WORDS = 2032
CHANNEL_SAMPLES = 1016
BEAT_WORDS = 8
SAMPLES_PER_BEAT = 4
TRAILING_WORDS = 8

# Dither geometry from sine_199p375MHz_2p6GSPS_impulse_dither.txt.json
# (identical to the 200 MHz bundle): 260 DAC samples period, raised-cosine
# impulse edge 16 / top 32 DAC samples.
DITHER_PERIOD_ADC = 130
DITHER_EDGE_ADC = 8
DITHER_TOP_ADC = 16
DITHER_PULSE_LEN = 2 * DITHER_EDGE_ADC + DITHER_TOP_ADC

PROFILE_HALF = 64


def load_capture(path: str) -> tuple[np.ndarray, np.ndarray]:
    """Mirror adc_reconstruct_channels() for a 4096-byte DMA CSV capture."""
    byte_values = np.loadtxt(path, dtype=np.uint16, skiprows=1)
    if byte_values.size != FRAME_BYTES:
        raise ValueError(
            f"{path}: expected {FRAME_BYTES} byte values, got {byte_values.size}")
    words = (byte_values[0::2].astype(np.uint16) |
             (byte_values[1::2].astype(np.uint16) << 8))
    words = words[:VALID_WORDS]
    samples = (words.astype(np.int16).astype(np.int32)) >> 2
    channel_a = np.empty(CHANNEL_SAMPLES, dtype=np.int32)
    channel_b = np.empty(CHANNEL_SAMPLES, dtype=np.int32)
    for beat in range(VALID_WORDS // BEAT_WORDS):
        base = beat * BEAT_WORDS
        out = beat * SAMPLES_PER_BEAT
        channel_a[out:out + SAMPLES_PER_BEAT] = samples[base:base + 4]
        channel_b[out:out + SAMPLES_PER_BEAT] = samples[base + 4:base + 8]
    return channel_a.astype(np.float64), channel_b.astype(np.float64)


def load_dac_waveform(path: str) -> np.ndarray:
    """Load the DAC TXT and downsample 2:1 onto the ADC grid, phase 0."""
    dac = np.loadtxt(path, dtype=np.float64)
    return dac[0::2]


def raised_cosine_impulse() -> np.ndarray:
    """Ideal dither impulse on the ADC grid (edge 8 / top 16 / edge 8)."""
    edge = DITHER_EDGE_ADC
    top = DITHER_TOP_ADC
    n = 2 * edge + top
    t = np.arange(n, dtype=np.float64)
    pulse = np.zeros(n)
    rise = t < edge
    fall = t >= edge + top
    middle = ~rise & ~fall
    pulse[rise] = 0.5 * (1.0 - np.cos(np.pi * t[rise] / edge))
    pulse[middle] = 1.0
    pulse[fall] = 0.5 * (1.0 - np.cos(np.pi * (n - 1 - t[fall]) / edge))
    return pulse


def fit_tone(samples: np.ndarray, frequency_hz: float,
             sample_rate_hz: float = ADC_RATE_HZ):
    """Least-squares DC + cosine + sine at a fixed frequency; returns model."""
    n = samples.size
    t = np.arange(n, dtype=np.float64) / sample_rate_hz
    design = np.column_stack((
        np.ones(n),
        np.cos(2.0 * np.pi * frequency_hz * t),
        np.sin(2.0 * np.pi * frequency_hz * t)))
    coef, *_ = np.linalg.lstsq(design, samples, rcond=None)
    return design @ coef


def refine_tone_frequency(samples: np.ndarray, coarse_hz: float,
                          span_hz: float = 200000.0, steps: int = 401,
                          sample_rate_hz: float = ADC_RATE_HZ):
    """Grid-refine the tone frequency that best explains the capture."""
    freqs = coarse_hz + np.linspace(-span_hz, span_hz, steps)
    best_f = coarse_hz
    best_r = np.inf
    n = samples.size
    t = np.arange(n, dtype=np.float64) / sample_rate_hz
    ones = np.ones(n)
    for f in freqs:
        design = np.column_stack((
            ones, np.cos(2.0 * np.pi * f * t), np.sin(2.0 * np.pi * f * t)))
        coef, *_ = np.linalg.lstsq(design, samples, rcond=None)
        residual = samples - design @ coef
        rms = float(np.sqrt(np.mean(residual * residual)))
        if rms < best_r:
            best_r = rms
            best_f = f
    return best_f, best_r


def detect_events(residual: np.ndarray, template: np.ndarray,
                  minimum_peak: float = 0.0,
                  window_half: int = PROFILE_HALF):
    """Cross-correlate the residual with the ideal impulse; return
    (center, polarity, peak_score) for complete, spaced events.

    One event per 130-sample slot is selected (strongest absolute
    correlation), so the picker is phase-agnostic and cannot invent more
    events than the dither period allows.
    """
    corr = np.correlate(residual, template, mode="same")
    corr = np.nan_to_num(corr, nan=0.0)
    scores = np.abs(corr)
    half = template.size // 2
    centers = []
    for slot_start in range(0, residual.size, DITHER_PERIOD_ADC):
        lo = slot_start + half
        hi = min(slot_start + DITHER_PERIOD_ADC + half, scores.size)
        if lo >= hi:
            continue
        local = int(np.argmax(scores[lo:hi]))
        peak_index = lo + local
        if scores[peak_index] <= minimum_peak:
            continue
        center = peak_index - half + 1
        if center - window_half < 0 or center + window_half > residual.size:
            continue
        polarity = 1.0 if corr[peak_index] >= 0.0 else -1.0
        centers.append((center, polarity, float(corr[peak_index])))
    return centers


def aggregate_profiles(residual_a, residual_b, events,
                       window_half: int = PROFILE_HALF):
    """Polarity-weighted event average around each detected impulse."""
    profiles_a = []
    profiles_b = []
    for center, polarity, _score in events:
        start = center - window_half
        stop = center + window_half
        if start < 0 or stop > residual_a.size:
            continue
        profiles_a.append(polarity * residual_a[start:stop])
        profiles_b.append(polarity * residual_b[start:stop])
    if not profiles_a:
        raise RuntimeError("no complete dither events in capture")
    profile_a = np.mean(np.asarray(profiles_a), axis=0)
    profile_b = np.mean(np.asarray(profiles_b), axis=0)
    return profile_a, profile_b


def edge_skew(profile, template, derivative, mask, gain,
              amplitude_fraction: float = 0.0):
    """Mirror estimate_profile_skew(): masked derivative projection with
    an optional amplitude gate on the local template."""
    select = np.ones(derivative.size, dtype=bool)
    if mask > 0:
        select = derivative > 0.0
    elif mask < 0:
        select = derivative < 0.0
    if amplitude_fraction > 0.0:
        threshold = amplitude_fraction * float(np.max(np.abs(template)))
        select &= np.abs(template) >= threshold
    num = float(np.sum((profile[select] - gain * template[select]) *
                       derivative[select]))
    den = float(np.sum(derivative[select] * derivative[select]))
    if den <= 1e-12:
        return float("nan")
    return num / (gain * den)


def analyze(path: str, waveform_path: str, tone_hz: float, plot: bool,
            window_half: int = PROFILE_HALF,
            gate_fraction: float = 0.0):
    a, b = load_capture(path)
    reference = load_dac_waveform(waveform_path)
    print(f"\n{path}")
    print(f"  reference length (ADC grid): {reference.size}")

    fit_a_hz, rms_a = refine_tone_frequency(a, tone_hz)
    fit_b_hz, rms_b = refine_tone_frequency(b, tone_hz)
    print(f"  tone fit: A {fit_a_hz:.1f} Hz (rms {rms_a:.2f}), "
          f"B {fit_b_hz:.1f} Hz (rms {rms_b:.2f})")

    residual_a = a - fit_tone(a, fit_a_hz)
    residual_b = b - fit_tone(b, fit_b_hz)

    template = raised_cosine_impulse()
    events_a = detect_events(residual_a, template, 0.0, window_half)
    print(f"  dither events detected: {len(events_a)} "
          f"(window +-{window_half}, gate {gate_fraction:g})")

    profile_a, profile_b = aggregate_profiles(residual_a, residual_b, events_a,
                                              window_half)
    gain_b = float(np.sum(profile_b * profile_a) /
                   np.sum(profile_a * profile_a))
    derivative = np.gradient(profile_a)

    skew_all = edge_skew(profile_b, profile_a, derivative, 0, gain_b,
                         gate_fraction)
    skew_rise = edge_skew(profile_b, profile_a, derivative, 1, gain_b,
                          gate_fraction)
    skew_fall = edge_skew(profile_b, profile_a, derivative, -1, gain_b,
                          gate_fraction)

    def ps(value):
        return value * 1.0e12 / ADC_RATE_HZ if np.isfinite(value) else float("nan")

    print(f"  relative skew : {skew_all:+.6f} samples = {ps(skew_all):+9.2f} ps")
    print(f"  rising skew   : {skew_rise:+.6f} samples = {ps(skew_rise):+9.2f} ps")
    print(f"  falling skew  : {skew_fall:+.6f} samples = {ps(skew_fall):+9.2f} ps")
    print(f"  edge disagreement: {abs(skew_rise - skew_fall):.6f} samples = "
          f"{ps(abs(skew_rise - skew_fall)):9.2f} ps")

    # Threshold-based edge diagnostics independent of the projection model.
    def crossing(signal, target):
        idx = np.where(np.diff(np.sign(signal - target)) != 0)[0]
        return idx

    peak = float(np.max(profile_a))
    lo = 0.1 * peak
    hi = 0.9 * peak
    rise_a = crossing(profile_a, hi)[0]
    fall_a = crossing(profile_a, hi)[-1] + 1
    rise_a_lo = crossing(profile_a, lo)[0]
    fall_a_lo = crossing(profile_a, lo)[-1] + 1
    rise_b = crossing(profile_b, hi * gain_b)[0]
    fall_b = crossing(profile_b, hi * gain_b)[-1] + 1
    rise_b_lo = crossing(profile_b, lo * gain_b)[0]
    fall_b_lo = crossing(profile_b, lo * gain_b)[-1] + 1
    width_a = fall_a_lo - rise_a_lo
    width_b = fall_b_lo - rise_b_lo
    print(f"  A pulse: rise@90% {rise_a}, fall@90% {fall_a}, width(10-90) {width_a}")
    print(f"  B pulse: rise@90% {rise_b}, fall@90% {fall_b}, width(10-90) {width_b}")
    print(f"  threshold edge shifts: rise {rise_b - rise_a:+g} samples, "
          f"fall {fall_b - fall_a:+g} samples")

    if plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(1, 2, figsize=(12, 4))
            x = np.arange(profile_a.size) - PROFILE_HALF
            for a_ in ax:
                a_.plot(x, profile_a, label="A")
                a_.plot(x, profile_b / gain_b, label="B (gain-normalized)")
                a_.grid(True, alpha=0.3)
                a_.legend()
            ax[0].set_xlim(rise_a - 8, rise_a + 8)
            ax[0].set_title("rising edge")
            ax[1].set_xlim(fall_a - 8, fall_a + 8)
            ax[1].set_title("falling edge")
            fig.tight_layout()
            out = Path(path).with_suffix(".dither_edges.png")
            fig.savefig(out, dpi=140)
            print(f"  plot: {out}")
        except Exception as exc:  # matplotlib is optional
            print(f"  (plot skipped: {exc})")

    return {
        "relative_samples": skew_all,
        "rising_samples": skew_rise,
        "falling_samples": skew_fall,
        "gain_b": gain_b,
        "events": len(events_a),
        "tone_a_hz": fit_a_hz,
        "tone_b_hz": fit_b_hz,
    }


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("waveform", help="DAC waveform TXT "
                        "(e.g. sine_199p375MHz_2p6GSPS_impulse_dither.txt)")
    parser.add_argument("captures", nargs="+", help="DMA capture CSV(s)")
    parser.add_argument("--tone-hz", type=float, default=199374935.0,
                        help="coarse tone frequency for the grid refinement "
                             "(default: 199.375 MHz scaled by the fitted "
                             "200 MHz bench clock ratio)")
    parser.add_argument("--window-half", type=int, default=PROFILE_HALF,
                        help="aggregation half-window in ADC samples "
                             "(firmware uses 64)")
    parser.add_argument("--gate", type=float, default=0.0,
                        help="amplitude gate fraction on |template| (firmware "
                             "uses 0.15; 0 disables)")
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args(argv)

    rows = []
    for capture in args.captures:
        rows.append(analyze(capture, args.waveform, args.tone_hz, args.plot,
                            args.window_half, args.gate))

    print("\nSummary")
    print(f"{'capture':<42} {'rel ps':>9} {'rise ps':>9} "
          f"{'fall ps':>9} {'disagree ps':>12} {'events':>6}")
    for capture, row in zip(args.captures, rows):
        scale = 1.0e12 / ADC_RATE_HZ
        rel = row["relative_samples"] * scale
        rise = row["rising_samples"] * scale
        fall = row["falling_samples"] * scale
        print(f"{Path(capture).name:<42} {rel:>9.2f} {rise:>9.2f} "
              f"{fall:>9.2f} {abs(rise - fall):>12.2f} {row['events']:>6}")


if __name__ == "__main__":
    main()
