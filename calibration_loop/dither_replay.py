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
# impulse edge 16 / top 32 DAC samples.  The doubled-period bundle
# (sine_*_p260.txt) uses 520 DAC samples = 260 ADC samples per event;
# pass --period-adc 260 and --window-half 100 for those captures.
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


def pulse_template(edge_dac: int, top_dac: int, shape: str) -> np.ndarray:
    """Ideal dither impulse on the ADC grid for a DAC-grid geometry.

    Local mirror of dither_geometry_scan.pulse_waveform() to avoid the
    circular import (that module imports this one for the replay pipeline).
    """
    length = 2 * edge_dac + top_dac
    t = np.arange(length, dtype=np.float64)
    pulse = np.zeros(length)
    rise = t < edge_dac
    fall = t >= edge_dac + top_dac
    middle = ~rise & ~fall
    if shape == "raised_cosine":
        pulse[rise] = 0.5 * (1.0 - np.cos(np.pi * t[rise] / edge_dac))
        pulse[middle] = 1.0
        pulse[fall] = 0.5 * (
            1.0 - np.cos(np.pi * (length - 1 - t[fall]) / edge_dac))
    elif shape == "linear":
        pulse[rise] = (t[rise] + 1.0) / edge_dac
        pulse[middle] = 1.0
        pulse[fall] = (length - t[fall]) / edge_dac
    else:
        raise ValueError(f"unknown dither shape {shape!r}")
    return pulse[0::2]


def parabolic_peak(scores: np.ndarray, index: int) -> float:
    """Sub-sample peak position via 3-point parabolic interpolation."""
    if index <= 0 or index >= scores.size - 1:
        return float(index)
    y0 = scores[index - 1]
    y1 = scores[index]
    y2 = scores[index + 1]
    denom = y0 - 2.0 * y1 + y2
    if abs(denom) <= 1e-12:
        return float(index)
    offset = 0.5 * (y0 - y2) / denom
    if not np.isfinite(offset) or abs(offset) > 1.0:
        return float(index)
    return float(index) + offset


def centroid_peak(scores: np.ndarray, index: int, radius: int = 4) -> float:
    """Robust sub-sample peak position: |score|-weighted centroid of the
    peak neighborhood.  Less biased than parabolic interpolation when the
    template is much narrower than the dispersed real pulse."""
    lo = max(0, index - radius)
    hi = min(scores.size, index + radius + 1)
    w = np.abs(scores[lo:hi])
    denom = float(np.sum(w))
    if denom <= 1e-12:
        return float(index)
    return float(np.sum(np.arange(lo, hi) * w) / denom)


def profile_delay(profile_a: np.ndarray, profile_b: np.ndarray) -> float:
    """Frame-level B-A delay from the aggregated A/B profiles via
    cross-correlation peak (parabolic sub-sample).  Shape-insensitive:
    both windows are measured data, so pulse-shape differences bias this
    far less than template-correlation peak positions."""
    xcorr = np.correlate(profile_b, profile_a, mode="full")
    peak = int(np.argmax(np.abs(xcorr)))
    delay = parabolic_peak(xcorr, peak)
    return delay - (profile_a.size - 1)


def interpolate(samples: np.ndarray, position: float) -> float:
    """Linear interpolation at a fractional sample position (mirrors the
    firmware adc_cal_dither_interpolate used by the aggregation)."""
    i0 = int(np.floor(position))
    frac = position - i0
    if i0 < 0 or i0 + 1 >= samples.size:
        return float("nan")
    return (1.0 - frac) * samples[i0] + frac * samples[i0 + 1]


def aggregate_profiles_interp(residual_a: np.ndarray, residual_b: np.ndarray,
                              centers, window_half: int = PROFILE_HALF):
    """Polarity-weighted event average with fractional (interpolated) event
    centers, mirroring the firmware aggregation.  centers is a list of
    (center, polarity) tuples with float center."""
    positions = np.arange(-window_half, window_half + 1)
    n = positions.size
    pa = np.zeros(n)
    pb = np.zeros(n)
    used = 0
    for center, polarity in centers:
        if center - window_half < 0.0 or center + window_half > residual_a.size - 1.0:
            continue
        va = np.array([interpolate(residual_a, center + p) for p in positions])
        vb = np.array([interpolate(residual_b, center + p) for p in positions])
        if np.isnan(va).any() or np.isnan(vb).any():
            continue
        pa += polarity * va
        pb += polarity * vb
        used += 1
    if used == 0:
        raise RuntimeError("no complete dither events in capture")
    return pa / used, pb / used


def refine_centers(residual: np.ndarray, template: np.ndarray, events_a,
                   search: int = 15):
    """Re-locate each coarse event center against the measured A profile
    (self-calibrated template) with parabolic sub-sample peak."""
    corr = np.correlate(residual, template, mode="same")
    corr = np.nan_to_num(corr, nan=0.0)
    refined = []
    for center, polarity, _score in events_a:
        c = int(round(center))
        lo = max(0, c - search)
        hi = min(corr.size, c + search)
        if lo >= hi:
            continue
        peak = lo + int(np.argmax(np.abs(corr[lo:hi])))
        refined.append((parabolic_peak(corr, peak), polarity))
    return refined


def event_ba_delta(residual_a: np.ndarray, residual_b: np.ndarray,
                   center: float, radius: int = 24) -> float:
    """Per-event B-A delay: cross-correlate the B window against the A
    window around one event.  Both windows are measured data, so the
    pulse-shape bias that corrupts template-peak positions cancels."""
    lo = int(round(center)) - radius
    hi = int(round(center)) + radius + 1
    if lo < 0 or hi > residual_a.size:
        return float("nan")
    wa = residual_a[lo:hi]
    wb = residual_b[lo:hi]
    xc = np.correlate(wb, wa, mode="full")
    peak = int(np.argmax(np.abs(xc)))
    return parabolic_peak(xc, peak) - (wa.size - 1)


def analyze_selfcalibrated(residual_a: np.ndarray, residual_b: np.ndarray,
                           template: np.ndarray, events_a,
                           window_half: int = PROFILE_HALF,
                           gate_fraction: float = 0.0,
                           align_b: bool = True) -> dict:
    """Self-calibrated dither skew estimate:

    1. coarse detection with the DAC template, aggregate the measured A
       profile (this is the per-frame 'local template');
    2. re-locate every event center against that measured A profile;
    3. aggregate with interpolated centers, and (optionally) align each B
       event window to its A window by the per-event cross-correlation
       delay before aggregation;
    4. run the same derivative-projection edge estimates on the result.
    """
    profile_a0, profile_b0 = aggregate_profiles(residual_a, residual_b,
                                                events_a, window_half)
    centers = refine_centers(residual_a, profile_a0, events_a)
    if len(centers) < 2:
        return {}

    deltas = np.asarray([event_ba_delta(residual_a, residual_b, c)
                         for c, _p in centers])
    deltas = deltas[np.isfinite(deltas)]

    if align_b:
        profile_a, _ = aggregate_profiles_interp(residual_a, residual_b,
                                                 centers, window_half)
        # re-aggregate B with each window shifted by its own delta
        positions = np.arange(-window_half, window_half + 1)
        n = positions.size
        pb = np.zeros(n)
        used = 0
        for (center, polarity), delta in zip(centers, deltas):
            if center - window_half < 0.0 or center + window_half > residual_b.size - 1.0:
                continue
            vb = np.array([interpolate(residual_b, center + delta + p)
                           for p in positions])
            if np.isnan(vb).any():
                continue
            pb += polarity * vb
            used += 1
        if used == 0:
            return {}
        profile_b = pb / used
    else:
        profile_a, profile_b = aggregate_profiles_interp(
            residual_a, residual_b, centers, window_half)

    gain_b = float(np.sum(profile_b * profile_a) /
                   np.sum(profile_a * profile_a))
    derivative = np.gradient(profile_a)
    skew_all = edge_skew(profile_b, profile_a, derivative, 0, gain_b,
                         gate_fraction)
    skew_rise = edge_skew(profile_b, profile_a, derivative, 1, gain_b,
                          gate_fraction)
    skew_fall = edge_skew(profile_b, profile_a, derivative, -1, gain_b,
                          gate_fraction)
    return {
        "relative_samples": skew_all,
        "rising_samples": skew_rise,
        "falling_samples": skew_fall,
        "gain_b": gain_b,
        "events": len(centers),
        "delta_mean": float(deltas.mean()) if deltas.size else np.nan,
        "delta_std": float(deltas.std()) if deltas.size else np.nan,
        "delta_min": float(deltas.min()) if deltas.size else np.nan,
        "delta_max": float(deltas.max()) if deltas.size else np.nan,
    }


def diagnose_events(residual_a: np.ndarray, residual_b: np.ndarray,
                    template: np.ndarray, events_a,
                    window_half: int = PROFILE_HALF,
                    period_adc: int = DITHER_PERIOD_ADC) -> dict:
    """Per-event alignment-grid diagnostics for one capture frame.

    Separates the three candidate sources of the frame-to-frame dither
    disagreement:

    - slot offset jitter: how far each A event center sits from the nominal
      130-sample DAC grid (per-event alignment scatter);
    - spacing drift: the mean A event spacing minus 130 samples (ADC/DAC
      clock mismatch accumulates across the frame and clips edge events);
    - A-vs-B center delta: the per-event B-A delay measured directly from
      the correlation peaks, with its scatter (event-level skew noise).
    """
    half = template.size // 2
    corr_a = np.correlate(residual_a, template, mode="same")
    corr_b = np.correlate(residual_b, template, mode="same")
    corr_a = np.nan_to_num(corr_a, nan=0.0)
    corr_b = np.nan_to_num(corr_b, nan=0.0)

    rows = []
    for k, (center, polarity, score) in enumerate(events_a):
        nominal = k * period_adc + half
        idx_a = int(round(center))
        ca_par = parabolic_peak(corr_a, idx_a)
        ca_cen = centroid_peak(corr_a, idx_a)
        lo = max(0, idx_a - 8)
        hi = min(corr_b.size, idx_a + 8)
        band = np.abs(corr_b[lo:hi])
        if band.size == 0:
            continue
        idx_b = lo + int(np.argmax(band))
        cb_par = parabolic_peak(corr_b, idx_b)
        cb_cen = centroid_peak(corr_b, idx_b)
        rows.append((k, ca_par, ca_cen, cb_par, cb_cen, cb_par - ca_par,
                     cb_cen - ca_cen, ca_par - nominal, polarity, score))

    ca_par = np.asarray([r[1] for r in rows])
    ca_cen = np.asarray([r[2] for r in rows])
    delta_par = np.asarray([r[5] for r in rows])
    delta_cen = np.asarray([r[6] for r in rows])
    offset_par = np.asarray([r[7] for r in rows])

    def fmt(values, width=7):
        return ", ".join(f"{v:+{width}.2f}" for v in values)

    print(f"  event grid (window +-{window_half}):")
    print(f"    centers(A) parabola    : {fmt(ca_par)}")
    print(f"    centers(A) centroid    : {fmt(ca_cen)}")
    print(f"    slot offset(A) parabola: {fmt(offset_par)}  "
          f"[mean {offset_par.mean():+.2f}, std {offset_par.std():.2f}]")
    spacing = np.diff(ca_cen)
    if spacing.size > 0:
        drift = float(spacing.mean() - period_adc)
        print(f"    spacing(A) centroid   : {fmt(spacing)}  "
              f"[mean {spacing.mean():.2f}, std {spacing.std():.2f}, "
              f"drift {drift:+.3f}/event]")
    print(f"    B-A delta parabola    : {fmt(delta_par)}  "
          f"[mean {delta_par.mean():+.3f}, std {delta_par.std():.3f} samples]")
    print(f"    B-A delta centroid    : {fmt(delta_cen)}  "
          f"[mean {delta_cen.mean():+.3f}, std {delta_cen.std():.3f} samples]")
    print(f"    polarity(A)           : "
          f"{', '.join(f'{int(r[8]):+d}' for r in rows)}")
    return {
        "slot_offset_mean": float(offset_par.mean()) if offset_par.size else np.nan,
        "slot_offset_std": float(offset_par.std()) if offset_par.size else np.nan,
        "spacing_mean": float(spacing.mean()) if spacing.size else np.nan,
        "spacing_std": float(spacing.std()) if spacing.size else np.nan,
        "spacing_drift": float(spacing.mean() - period_adc) if spacing.size else np.nan,
        "ab_delta_mean": float(delta_cen.mean()) if delta_cen.size else np.nan,
        "ab_delta_std": float(delta_cen.std()) if delta_cen.size else np.nan,
    }


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
                  window_half: int = PROFILE_HALF,
                  period_adc: int = DITHER_PERIOD_ADC):
    """Cross-correlate the residual with the ideal impulse; return
    (center, polarity, peak_score) for complete, spaced events.

    One event per period-adc slot is selected (strongest absolute
    correlation), so the picker is phase-agnostic and cannot invent more
    events than the dither period allows.  period_adc defaults to the
    doubled-period geometry (260); pass 130 for legacy captures.
    """
    corr = np.correlate(residual, template, mode="same")
    corr = np.nan_to_num(corr, nan=0.0)
    scores = np.abs(corr)
    half = template.size // 2
    centers = []
    for slot_start in range(0, residual.size, period_adc):
        lo = slot_start + half
        hi = min(slot_start + period_adc + half, scores.size)
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
                       window_half: int = PROFILE_HALF,
                       detrend: bool = False):
    """Polarity-weighted event average around each detected impulse.

    detrend=True removes each event window's linear trend (DC + slope)
    before aggregation, mirroring the firmware profile_window_detrend
    flag.  Board evidence (2026-08-19): tone-residual DC/ramp inside the
    windows biases the rising/falling projections oppositely (100-240 ps
    edge disagreement); per-event detrending drops replay frames under
    the 23.1 ps gate.  Only meaningful with a widened window.
    """
    profiles_a = []
    profiles_b = []
    for center, polarity, _score in events:
        start = center - window_half
        stop = center + window_half
        if start < 0 or stop > residual_a.size:
            continue
        wa = polarity * residual_a[start:stop]
        wb = polarity * residual_b[start:stop]
        if detrend:
            x = np.arange(wa.size)
            wa = wa - np.polyval(np.polyfit(x, wa, 1), x)
            wb = wb - np.polyval(np.polyfit(x, wb, 1), x)
        profiles_a.append(wa)
        profiles_b.append(wb)
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
            gate_fraction: float = 0.0,
            template: np.ndarray | None = None,
            diagnose: bool = False,
            selfcalibrate: bool = False,
            detrend_windows: bool = False,
            period_adc: int = DITHER_PERIOD_ADC):
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

    if template is None:
        template = raised_cosine_impulse()
    events_a = detect_events(residual_a, template, 0.0, window_half,
                             period_adc)
    print(f"  dither events detected: {len(events_a)} "
          f"(window +-{window_half}, gate {gate_fraction:g}, "
          f"period {period_adc} ADC samples)")

    if diagnose:
        diagnose_events(residual_a, residual_b, template, events_a,
                        window_half, period_adc)

    profile_a, profile_b = aggregate_profiles(residual_a, residual_b, events_a,
                                              window_half, detrend_windows)
    if detrend_windows:
        print(f"  per-event window detrend: ON (mirrors firmware "
              f"profile_window_detrend)")
    gain_b = float(np.sum(profile_b * profile_a) /
                   np.sum(profile_a * profile_a))
    derivative = np.gradient(profile_a)

    profile_delay_samples = profile_delay(profile_a, profile_b)

    skew_all = edge_skew(profile_b, profile_a, derivative, 0, gain_b,
                         gate_fraction)
    skew_rise = edge_skew(profile_b, profile_a, derivative, 1, gain_b,
                          gate_fraction)
    skew_fall = edge_skew(profile_b, profile_a, derivative, -1, gain_b,
                          gate_fraction)

    def ps(value):
        return value * 1.0e12 / ADC_RATE_HZ if np.isfinite(value) else float("nan")

    print(f"  profile-level B-A delay: {profile_delay_samples:+.4f} samples = "
          f"{ps(profile_delay_samples):+9.2f} ps")

    if selfcalibrate:
        sc = analyze_selfcalibrated(residual_a, residual_b, template,
                                    events_a, window_half, gate_fraction)
        if sc:
            dis = abs(sc["rising_samples"] - sc["falling_samples"])
            print(f"  [self-calibrated] centers: {sc['events']} | "
                  f"B-A delta mean {sc['delta_mean']:+.3f} "
                  f"std {sc['delta_std']:.3f} "
                  f"[{sc['delta_min']:+.3f}, {sc['delta_max']:+.3f}] samples")
            print(f"  [self-calibrated] rel {ps(sc['relative_samples']):+9.2f} ps | "
                  f"rise {ps(sc['rising_samples']):+9.2f} ps | "
                  f"fall {ps(sc['falling_samples']):+9.2f} ps | "
                  f"disagreement {ps(dis):+9.2f} ps")

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
    parser.add_argument("--template-shape", choices=("raised_cosine", "linear"),
                        default="raised_cosine",
                        help="ideal impulse geometry (default raised_cosine)")
    parser.add_argument("--template-edge", type=int, default=16,
                        help="impulse edge length in DAC samples "
                             "(default 16 = legacy raised_cosine 8 ADC)")
    parser.add_argument("--template-top", type=int, default=32,
                        help="impulse flat-top length in DAC samples "
                             "(default 32 = legacy raised_cosine 16 ADC)")
    parser.add_argument("--period-adc", type=int, default=DITHER_PERIOD_ADC,
                        help="dither event spacing in ADC samples "
                             "(default 260 doubled-period; pass 130 for "
                             "legacy captures)")
    parser.add_argument("--diagnose", action="store_true",
                        help="print per-event alignment-grid diagnostics "
                             "(slot offsets, spacing drift, A-B center deltas)")
    parser.add_argument("--selfcalibrate", action="store_true",
                        help="two-pass estimate: re-locate events against the "
                             "measured A profile and align each B window to A "
                             "before the edge projections")
    parser.add_argument("--detrend-windows", action="store_true",
                        help="remove each event window's linear trend before "
                             "aggregation (mirrors firmware "
                             "profile_window_detrend; pairs with a widened "
                             "window)")
    parser.add_argument("--plot", action="store_true")
    args = parser.parse_args(argv)

    template = pulse_template(args.template_edge, args.template_top,
                              args.template_shape)
    rows = []
    for capture in args.captures:
        rows.append(analyze(capture, args.waveform, args.tone_hz, args.plot,
                            args.window_half, args.gate, template,
                            args.diagnose, args.selfcalibrate,
                            args.detrend_windows, args.period_adc))

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
