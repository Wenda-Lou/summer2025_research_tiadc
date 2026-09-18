"""Why does the channel polarity flip between frames?

Tests the hypothesis that the recovered dither profile sign is decided by the
LOCAL polarity balance of the ~8 events visible in a capture, not by the true
channel sense.  Over the full 64-event loop the polarity sequence is balanced,
but any 8-event window can be strongly one-sided, which inverts V and therefore
the gain sign.

For each frame this prints:
  n0, the visible event indices k, the mean polarity pbar = mean(signs[k]),
  the raw profile peak sign, the resulting gain sign, and the channel signs
  returned by align_to_loop.

Usage:
    python tools/polarity_diag.py --uart COM5 --frames 10
"""

from __future__ import annotations

import argparse
import sys

import numpy as np


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--frames", type=int, default=10)
    args = ap.parse_args(argv)

    from calibration_loop.capture import HardwareBench
    from calibration_loop.dither import DitherConfig, adc_templates, polarity_sequence
    from calibration_loop.estimator import (
        align_to_loop, estimate_channel, fit_tone, prepare_capture, visible_events,
    )

    cfg = DitherConfig()
    cfg.validate()
    signs = polarity_sequence(cfg)
    m, _, _ = adc_templates(cfg)
    m_lo, m_hi = int(m[0]), int(m[-1])
    f0 = cfg.sig_cycles / cfg.n_adc_period

    bench = HardwareBench(uart_port=args.uart)
    print(f"{'fr':>3} {'n0':>6} {'k range':>10} {'nEv':>4} {'pbar':>7} "
          f"{'peakA':>8} {'signA':>6} {'peakB':>8} {'signB':>6} "
          f"{'gainA':>8} {'gainB':>8} {'swap':>5}")
    print("-" * 100)
    try:
        sig = None
        for i in range(args.frames):
            raw = bench.capture()
            if raw is None:
                continue
            prep = prepare_capture(raw, cfg, signature=sig)
            if sig is None:
                sig = prep["signature"]
            n0 = prep["n0"]
            ks, starts = visible_events(n0, prep["ch_a"].size, cfg, m_lo, m_hi)
            pbar = float(signs[ks].mean()) if ks.size else float("nan")

            # channel signs exactly as prepare_capture sees them
            alA = align_to_loop(fit_tone(prep["ch_a"], f0, refine=True)["residual"], cfg)
            alB = align_to_loop(fit_tone(prep["ch_b"], f0, refine=True)["residual"], cfg)

            ea = estimate_channel(prep["ch_a"], cfg, n0=n0)
            eb = estimate_channel(prep["ch_b"], cfg, n0=n0)
            kr = f"{ks.min()}..{ks.max()}" if ks.size else "-"
            print(f"{i:>3} {n0:>6} {kr:>10} {ks.size:>4} {pbar:>+7.3f} "
                  f"{alA['peak']:>+8.3f} {alA['sign']:>+6.0f} "
                  f"{alB['peak']:>+8.3f} {alB['sign']:>+6.0f} "
                  f"{ea.gain_codes:>+8.2f} {eb.gain_codes:>+8.2f} "
                  f"{str(prep['swapped']):>5}")
    finally:
        bench.close()
    print("-" * 100)
    print("If gain sign tracks pbar, the local polarity balance is the cause and")
    print("an absolute polarity anchor is required (the loop index k alone is not")
    print("enough when only a few events are visible).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
