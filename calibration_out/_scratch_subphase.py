"""Scratch: does a drifting sub-sample clock phase reproduce the bench scatter?

The bench shows a ~+-10 % common-mode wander in gainA/gainB with a few per cent
left over in the ratio, on a timescale of tens of seconds -- while the simulator
reports 0.0008.  The simulator pins ``BenchModel.subsample_phase`` (the ADC clock
phase against the DPG loop) to a constant; on the bench nothing pins it.  This
script drifts it between captures and measures what that does to the same
statistic the probe gates on (half-vs-half of the joint gain ratio).
"""

from __future__ import annotations

import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.dither import DitherConfig          # noqa: E402
from calibration_loop.estimator import (                    # noqa: E402
    estimate_block, estimate_block_joint, polarity_anchor, prepare_capture,
)
from calibration_loop.simulate import BenchModel            # noqa: E402


def run(cfg, frames=10, drift_per_frame=0.0, skew_b_ps=150.0, noise=3.0, seed=7):
    bench = BenchModel(cfg=cfg, skew_b_ps=skew_b_ps, noise_rms_codes=noise, seed=seed)
    batch, gains = [], []
    pol = None
    for _ in range(frames):
        raw = bench.capture(n_words=2048)
        prep = prepare_capture(raw, cfg)
        if pol is None:
            pol = polarity_anchor(prep)
        est = estimate_block(prep["ch_a"], prep["ch_b"], cfg, n0=prep["n0"],
                             polarity_sign=pol, pin_polarity=True)
        gains.append((est.ch_a.gain_codes, est.ch_b.gain_codes, est.gain_ratio))
        batch.append((prep["ch_a"], prep["ch_b"], prep["n0"]))
        # the drift under test: the ADC clock slides against the DPG loop
        bench.subsample_phase = (bench.subsample_phase + drift_per_frame) % 1.0

    ga = np.array([g[0] for g in gains])
    gb = np.array([g[1] for g in gains])
    gr = np.array([g[2] for g in gains])
    half = len(batch) // 2
    j1 = estimate_block_joint(batch[:half], cfg, polarity_sign=pol, pin_polarity=True)
    j2 = estimate_block_joint(batch[half:], cfg, polarity_sign=pol, pin_polarity=True)
    scatter = abs(j1.gain_ratio - j2.gain_ratio) / 2.0
    return dict(ga=ga, gb=gb, gr=gr, scatter=scatter,
                joint=estimate_block_joint(batch, cfg, polarity_sign=pol,
                                           pin_polarity=True).gain_ratio)


def main() -> int:
    cfg = DitherConfig()
    cfg.validate()
    print(f"{'subsample drift':>16}{'gainA spread':>14}{'gainB spread':>14}"
          f"{'ratio spread':>14}{'half-vs-half':>14}{'rand err':>10}")
    for drift in (0.0, 0.01, 0.02, 0.05, 0.10):
        r = run(cfg, drift_per_frame=drift)
        print(f"{drift:>16.2f}"
              f"{np.std(r['ga']) / np.mean(r['ga']) * 100:>13.2f}%"
              f"{np.std(r['gb']) / np.mean(r['gb']) * 100:>13.2f}%"
              f"{np.std(r['gr']):>14.5f}"
              f"{r['scatter']:>14.5f}"
              f"{abs(r['joint'] - 1.021):>10.5f}")
    print("\nbench probe (10 frames) observed: gainA/gainB wander ~10 %, "
          "ratio std ~0.023-0.028, half-vs-half 0.012-0.029")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
