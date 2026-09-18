"""Scratch: how much does an integer loop-alignment offset bias the gain estimate?

The bench probe shows low ``margin`` frames carrying ~10-13 % lower gainA *and*
gainB (common mode), with a few per cent left over in the ratio -- and the ratio
scatter gate (half-vs-half) is what fails.  The tone amplitude is rock stable
across the same frames (<1 %), so the fluctuation is in the dither processing,
not in the analog path.

``estimate_block`` takes ``n0`` and slices both channels' dither windows from it.
This sweeps an injected offset on identical data to measure the sensitivity --
something the simulator can do exactly and the bench cannot.
"""

from __future__ import annotations

import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.dither import DitherConfig                 # noqa: E402
from calibration_loop.estimator import (                          # noqa: E402
    align_to_loop, estimate_block, fit_tone, polarity_anchor, prepare_capture,
)
from calibration_loop.simulate import BenchModel                  # noqa: E402


def main() -> int:
    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    bench = BenchModel(cfg=cfg, skew_b_ps=150.0, noise_rms_codes=3.0, seed=7)
    raw = bench.capture(n_words=2048)
    prep = prepare_capture(raw, cfg)
    pol = polarity_anchor(prep)
    n0 = prep["n0"]

    # margin at each offset, for context: this is what the probe gates on
    print(f"true n0 = {n0}")
    print(f"{'n0 offset':>10}{'margin':>9}{'gainA':>10}{'gainB':>10}{'gB/gA':>10}")
    base = None
    for delta in range(-5, 6):
        al = align_to_loop(fit_tone(prep["ch_a"], f0, refine=True)["residual"], cfg)
        est = estimate_block(prep["ch_a"], prep["ch_b"], cfg, n0=n0 + delta,
                             polarity_sign=pol, pin_polarity=True)
        if delta == 0:
            base = est.gain_ratio
        print(f"{delta:>10d}{al['margin']:>9.1f}{est.ch_a.gain_codes:>10.2f}"
              f"{est.ch_b.gain_codes:>10.2f}{est.gain_ratio:>10.5f}")
    print(f"\nratio at offset 0 = {base:.5f}")
    print("a few per cent of swing over a couple of samples would explain the")
    print("bench's half-vs-half wander, since each frame picks its own n0.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
