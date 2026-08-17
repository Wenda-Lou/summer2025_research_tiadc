"""
Grid scan of (window-half, amplitude gate) for the dither replay.

Fits the tone once per capture, then sweeps the two firmware macros the
bench can set (CAL_SKEW_DITHER_PROFILE_WINDOW_HALF and
CAL_SKEW_DITHER_PROFILE_MASK_FRACTION) and ranks the combinations by mean
edge disagreement across the captured frames:

    python -m calibration_loop.dither_param_scan <waveform.txt> <capture.csv>...
"""

from __future__ import annotations

import sys

import numpy as np

from .dither_geometry_scan import pulse_waveform
from .dither_replay import (
    ADC_RATE_HZ,
    aggregate_profiles,
    detect_events,
    edge_skew,
    fit_tone,
    load_capture,
    refine_tone_frequency,
)

WINDOWS = (16, 24, 32, 48, 64)
GATES = (0.0, 0.05, 0.10, 0.15, 0.25)
COARSE_TONE_HZ = 199_374_935.0


def main() -> None:
    waveform = sys.argv[1]
    captures = sys.argv[2:]
    template = pulse_waveform(48, 0, "linear")[0::2]
    prepared = []
    for path in captures:
        a, b = load_capture(path)
        fa, _ = refine_tone_frequency(a, COARSE_TONE_HZ)
        fb, _ = refine_tone_frequency(b, COARSE_TONE_HZ)
        prepared.append((path, a - fit_tone(a, fa), b - fit_tone(b, fb)))
    rows = []
    for window in WINDOWS:
        for gate in GATES:
            dis_vals = []
            rel_vals = []
            for _path, ra, rb in prepared:
                events = detect_events(ra, template, 0.0, window)
                if not events:
                    continue
                pa, pb = aggregate_profiles(ra, rb, events, window)
                gain = float(np.sum(pb * pa) / np.sum(pa * pa))
                derivative = np.gradient(pa)
                rise = edge_skew(pb, pa, derivative, 1, gain, gate)
                fall = edge_skew(pb, pa, derivative, -1, gain, gate)
                rel = edge_skew(pb, pa, derivative, 0, gain, gate)
                if np.isfinite(rise) and np.isfinite(fall):
                    dis_vals.append(abs(rise - fall) * 1e12 / ADC_RATE_HZ)
                    rel_vals.append(rel * 1e12 / ADC_RATE_HZ)
            if dis_vals:
                rows.append((window, gate,
                             float(np.mean(dis_vals)),
                             float(np.mean(rel_vals)),
                             float(np.std(dis_vals)),
                             len(dis_vals)))
    rows.sort(key=lambda r: r[2])
    print(f"{'window':>7} {'gate':>5} {'mean dis ps':>12} "
          f"{'std dis ps':>11} {'mean rel ps':>12} {'frames':>7}")
    for window, gate, dis, dis_std, rel, n in rows:
        print(f"{window:>7} {gate:>5.2f} {dis:>12.2f} {dis_std:>11.2f} "
              f"{rel:>12.2f} {n:>7}")


if __name__ == "__main__":
    main()
