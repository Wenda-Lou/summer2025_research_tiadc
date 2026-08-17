"""
Synthetic dither-geometry scanner for the ZCU102 bench.

Mirrors the production estimator chain (event detection -> polarity-weighted
profile aggregation -> Channel-A-profile template -> masked derivative
projection) on the exact bench rates, so pulse geometries can be ranked in
seconds instead of one bench session each:

    python -m calibration_loop.dither_geometry_scan

The virtual board model follows the VB-project recipe that solved the same
dither cross-check problem: ideal DAC code pattern, linear-interpolation
sampling with a sub-sample shift on Channel B, least-squares tone fit and
residual subtraction before dither analysis.
"""

from __future__ import annotations

import numpy as np

from .dither_replay import (
    ADC_RATE_HZ,
    DAC_RATE_HZ,
    aggregate_profiles,
    detect_events,
    edge_skew,
    fit_tone,
)

TONE_HZ = 199_375_000.0
NUM_DAC = 16_640
PERIOD_DAC = 260
POSITION_DAC = 96
SCALE_LSB = 2_000.0
SINE_PEAK_CODES = 29_369.0
SKEW_TRUTH_ADC_SAMPLES = 0.12          # ~92 ps, near the observed bench skew
WINDOW_ADC = 1_040                     # 8 complete events at 130-sample period
GATE_EDGE_DISAGREEMENT_PS = 23.0769    # 0.03 samples at 1.3 GSPS


def tone_waveform() -> np.ndarray:
    n = np.arange(NUM_DAC, dtype=np.float64)
    return SINE_PEAK_CODES * np.sin(
        2.0 * np.pi * TONE_HZ * n / DAC_RATE_HZ)


def pulse_waveform(edge: int, top: int, shape: str) -> np.ndarray:
    """Raised-cosine or linear-triangle impulse of edge/top/edge DAC samples."""
    length = 2 * edge + top
    t = np.arange(length, dtype=np.float64)
    pulse = np.zeros(length)
    if shape == "raised_cosine":
        rise = t < edge
        fall = t >= edge + top
        middle = ~rise & ~fall
        pulse[rise] = 0.5 * (1.0 - np.cos(np.pi * t[rise] / edge))
        pulse[middle] = 1.0
        pulse[fall] = 0.5 * (
            1.0 - np.cos(np.pi * (length - 1 - t[fall]) / edge))
    elif shape == "linear":
        rise = t < edge
        fall = t >= edge + top
        middle = ~rise & ~fall
        pulse[rise] = (t[rise] + 1.0) / edge
        pulse[middle] = 1.0
        pulse[fall] = (length - t[fall]) / edge
    else:
        raise ValueError(f"unknown dither shape {shape!r}")
    return pulse


def build_dac_waveform(edge: int, top: int, shape: str) -> np.ndarray:
    dac = tone_waveform()
    pulse = pulse_waveform(edge, top, shape)
    for event in range(NUM_DAC // PERIOD_DAC):
        start = event * PERIOD_DAC + POSITION_DAC
        sign = 1.0 if event % 2 == 0 else -1.0
        dac[start:start + pulse.size] += sign * SCALE_LSB * pulse
    return dac


def sample_channel(dac: np.ndarray, offset_dac_samples: float) -> np.ndarray:
    """2:1 downsample with linear-interpolation sub-sample offset (VB model)."""
    positions = 2.0 * np.arange(WINDOW_ADC, dtype=np.float64) + offset_dac_samples
    base = np.floor(positions).astype(np.int64)
    frac = positions - base
    base = np.clip(base, 0, NUM_DAC - 2)
    return dac[base] * (1.0 - frac) + dac[base + 1] * frac


def estimate(edge: int, top: int, shape: str) -> dict:
    dac = build_dac_waveform(edge, top, shape)
    a = sample_channel(dac, 0.0)
    b = sample_channel(dac, 2.0 * SKEW_TRUTH_ADC_SAMPLES)
    residual_a = a - fit_tone(a, TONE_HZ)
    residual_b = b - fit_tone(b, TONE_HZ)
    template = pulse_waveform(edge, top, shape)[0::2]
    events = detect_events(residual_a, template)
    profile_a, profile_b = aggregate_profiles(residual_a, residual_b, events)
    gain_b = float(np.sum(profile_b * profile_a) /
                   np.sum(profile_a * profile_a))
    derivative = np.gradient(profile_a)
    rel = edge_skew(profile_b, profile_a, derivative, 0, gain_b)
    rise = edge_skew(profile_b, profile_a, derivative, 1, gain_b)
    fall = edge_skew(profile_b, profile_a, derivative, -1, gain_b)

    def ps(value):
        return value * 1.0e12 / ADC_RATE_HZ

    truth_ps = ps(SKEW_TRUTH_ADC_SAMPLES)
    disagreement = abs(rise - fall)
    return {
        "edge": edge,
        "top": top,
        "shape": shape,
        "events": len(events),
        "gain_b": gain_b,
        "rel_ps": ps(rel),
        "rise_ps": ps(rise),
        "fall_ps": ps(fall),
        "rel_error_ps": ps(rel) - truth_ps,
        "disagreement_ps": ps(disagreement),
        "passes": bool(ps(disagreement) < GATE_EDGE_DISAGREEMENT_PS and
                       abs(ps(rel) - truth_ps) < GATE_EDGE_DISAGREEMENT_PS),
    }


def main() -> None:
    print(f"truth: B shifted {SKEW_TRUTH_ADC_SAMPLES} ADC samples = "
          f"{SKEW_TRUTH_ADC_SAMPLES * 1e12 / ADC_RATE_HZ:.1f} ps; "
          f"gate {GATE_EDGE_DISAGREEMENT_PS:.1f} ps")
    print()
    header = (f"{'geom':<22} {'events':>6} {'gain_b':>8} {'rel ps':>9} "
              f"{'err ps':>8} {'rise ps':>9} {'fall ps':>9} {'dis ps':>8}  ok")
    print(header)
    rows = []
    for shape in ("raised_cosine", "linear"):
        for edge in (8, 12, 16, 24, 32, 48):
            for top in (0, 16, 32, 64):
                if 2 * edge + top > 200:
                    continue
                rows.append(estimate(edge, top, shape))
    rows.sort(key=lambda r: (not r["passes"], r["disagreement_ps"],
                             abs(r["rel_error_ps"])))
    for r in rows:
        name = f"{r['shape'][:8]:<8} e{r['edge']:>2}/t{r['top']:>2}"
        print(f"{name:<22} {r['events']:>6} {r['gain_b']:>8.4f} "
              f"{r['rel_ps']:>9.2f} {r['rel_error_ps']:>+8.2f} "
              f"{r['rise_ps']:>9.2f} {r['fall_ps']:>9.2f} "
              f"{r['disagreement_ps']:>8.2f}  {'PASS' if r['passes'] else ''}")


if __name__ == "__main__":
    main()
