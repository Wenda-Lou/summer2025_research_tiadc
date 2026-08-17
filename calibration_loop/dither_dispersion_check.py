"""
Dispersion/truncation experiment for the dither fine-skew estimator.

Models the measured bench analog chain (ideal 32-sample pulse arrives as a
~75-125 sample 10-90% blob) and compares the firmware's template-sized
profile window against a widened window on the new linear-triangle vector:

    python -m calibration_loop.dither_dispersion_check
"""

from __future__ import annotations

import numpy as np

from .dither_replay import (
    ADC_RATE_HZ,
    aggregate_profiles,
    detect_events,
    edge_skew,
    fit_tone,
)
from .dither_geometry_scan import (
    build_dac_waveform,
    pulse_waveform,
    sample_channel,
    SKEW_TRUTH_ADC_SAMPLES,
    TONE_HZ,
)

# Measured bench dispersion: ideal 32-ADC-sample pulse -> 10-90% width
# ~75-125 ADC samples, so the chain's own 10-90% step width is ~87 samples.
# A Gaussian with sigma ~34 ADC samples reproduces that scale.
SIGMA_ADC = 34.0


def disperse(dac: np.ndarray) -> np.ndarray:
    sigma_dac = 2.0 * SIGMA_ADC
    half = int(5 * sigma_dac)
    kernel = np.exp(-0.5 * (np.arange(-half, half + 1) / sigma_dac) ** 2)
    kernel /= kernel.sum()
    return np.convolve(dac, kernel, mode="same")


def run_case(edge, top, shape, window_half, truth_ps):
    dac = build_dac_waveform(edge, top, shape)
    dac = disperse(dac)
    a = sample_channel(dac, 0.0)
    b = sample_channel(dac, 2.0 * SKEW_TRUTH_ADC_SAMPLES)
    residual_a = a - fit_tone(a, TONE_HZ)
    residual_b = b - fit_tone(b, TONE_HZ)
    template = pulse_waveform(edge, top, shape)[0::2]
    events = detect_events(residual_a, template, 0.0, window_half)
    pa, pb = aggregate_profiles(residual_a, residual_b, events, window_half)
    gain_b = float(np.sum(pb * pa) / np.sum(pa * pa))
    derivative = np.gradient(pa)
    rel = edge_skew(pb, pa, derivative, 0, gain_b)
    rise = edge_skew(pb, pa, derivative, 1, gain_b)
    fall = edge_skew(pb, pa, derivative, -1, gain_b)

    def ps(v):
        return v * 1e12 / ADC_RATE_HZ

    return {
        "rel_ps": ps(rel),
        "rise_ps": ps(rise),
        "fall_ps": ps(fall),
        "err_ps": ps(rel) - truth_ps,
        "dis_ps": ps(abs(rise - fall)),
        "gain_b": gain_b,
    }


def main() -> None:
    truth_ps = SKEW_TRUTH_ADC_SAMPLES * 1e12 / ADC_RATE_HZ
    print(f"truth {truth_ps:.1f} ps, dispersion sigma {SIGMA_ADC} ADC samples")
    print()
    header = (f"{'geometry':<24} {'window':>8} {'rel ps':>9} {'err ps':>8} "
              f"{'rise ps':>9} {'fall ps':>9} {'dis ps':>8}")
    print(header)
    for edge, top, shape in (
        (16, 32, "raised_cosine"),   # previous bench vector
        (48, 0, "linear"),           # new bench vector
        (64, 0, "linear"),           # wider fallback
    ):
        template_half = (2 * edge + top) // 2  # firmware window = template/2
        for window_half in (template_half, 64):
            r = run_case(edge, top, shape, window_half, truth_ps)
            name = f"{shape[:5]} e{edge}/t{top}"
            print(f"{name:<24} {window_half:>8} {r['rel_ps']:>9.2f} "
                  f"{r['err_ps']:>+8.2f} {r['rise_ps']:>9.2f} "
                  f"{r['fall_ps']:>9.2f} {r['dis_ps']:>8.2f}")


if __name__ == "__main__":
    main()
