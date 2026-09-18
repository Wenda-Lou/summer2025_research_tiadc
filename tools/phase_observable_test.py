"""Find a dither-immune tone-phase estimator on live hardware frames.

The block estimator's phase observable is very repeatable but carries a large
constant offset, which is the signature of the dither pulse train leaking into
the tone fit.  This compares three phase-extraction strategies on the SAME frames
and reports, for each, the per-frame B-A skew in ps:

  raw        : fit the tone on y as-is
  twopass    : subtract a synthesized dither (from the profile estimate), refit
  masked     : fit the tone with the dither event windows excluded entirely

The winner is the one whose frame-to-frame scatter is smallest AND whose mean is
physically plausible (the firmware measures this pair at -148.8 ps raw -> 0.67 ps
calibrated).

Usage:
    python tools/phase_observable_test.py --uart COM5 --frames 6
"""

from __future__ import annotations

import argparse
import statistics as st
import sys

import numpy as np

TS_PS = 1e12 / 1300e6


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--frames", type=int, default=6)
    args = ap.parse_args(argv)

    from calibration_loop.capture import HardwareBench
    from calibration_loop.dither import DitherConfig
    from calibration_loop.estimator import (
        estimate_channel, fit_tone, prepare_capture,
    )

    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period

    bench = HardwareBench(uart_port=args.uart)
    res = {"raw": [], "twopass": [], "masked": []}
    signs = []
    try:
        sig = None
        for i in range(args.frames):
            raw = bench.capture()
            if raw is None:
                print(f"frame {i}: capture FAILED")
                continue
            prep = prepare_capture(raw, cfg, signature=sig)
            if sig is None:
                sig = prep["signature"]
            a, b = prep["ch_a"], prep["ch_b"]
            n = a.size

            # common frequency: refine once on channel A, hold for both
            fA = fit_tone(a, f0, refine=True)["f0"]
            fB = fit_tone(b, f0, refine=True)["f0"]
            fsh = 0.5 * (fA + fB)

            phase = {}
            # --- raw ---
            phase["raw"] = (fit_tone(a, fsh, refine=False)["phase"],
                            fit_tone(b, fsh, refine=False)["phase"])

            # --- two-pass: remove the profile-synthesized dither first ---
            ea = estimate_channel(a, cfg, n0=prep["n0"])
            eb = estimate_channel(b, cfg, n0=prep["n0"])
            da = _synth(n, prep["n0"], cfg, ea.gain_codes)
            db = _synth(n, prep["n0"], cfg, eb.gain_codes)
            phase["twopass"] = (fit_tone(a - da, fsh, refine=False)["phase"],
                                fit_tone(b - db, fsh, refine=False)["phase"])

            # --- masked: exclude every dither event window from the fit ---
            phase["masked"] = (_fit_masked(a, fsh, cfg, prep["n0"]),
                               _fit_masked(b, fsh, cfg, prep["n0"]))

            line = f"frame {i:2d}: "
            for k, (pa, pb) in phase.items():
                d = float(np.arctan2(np.sin(pb - pa), np.cos(pb - pa)))
                sk = d / (2 * np.pi * fsh) * TS_PS
                res[k].append(sk)
                line += f"{k}={sk:+9.2f}ps  "
            line += (f"| rot={prep['rotation']} swap={str(prep['swapped']):<5} "
                     f"dcA={a.mean():+7.2f} dcB={b.mean():+7.2f} "
                     f"sum={a.mean()+b.mean():+7.2f}")
            print(line)
            signs.append((i, prep["rotation"], prep["swapped"],
                          float(a.mean()), float(b.mean())))
    finally:
        bench.close()

    print("\n=== per-method B-A skew [ps] ===")
    print(f"{'method':<10} {'mean':>10} {'std':>9} {'min':>10} {'max':>10} {'n':>4}")
    for k, v in res.items():
        if not v:
            continue
        print(f"{k:<10} {st.mean(v):+10.2f} {st.pstdev(v):9.2f} "
              f"{min(v):+10.2f} {max(v):+10.2f} {len(v):4d}")
    print("\nReference: firmware measures this same pair at -148.8 ps raw.")
    print("Winner = smallest std with a plausible mean.")
    return 0


def _synth(n_samples, n0, cfg, gain_codes):
    from calibration_loop.estimator import synthesize_dither
    if not np.isfinite(gain_codes):
        return np.zeros(n_samples)
    return synthesize_dither(n_samples, n0, cfg, gain_codes)


def _fit_masked(y, f0, cfg, n0):
    """Fit the tone with all dither event windows zero-weighted."""
    n = y.size
    t = np.arange(n, dtype=np.float64)
    keep = np.ones(n, dtype=bool)
    n_loop = cfg.n_adc_period
    half = int(cfg.edge_r + cfg.top_w + cfg.edge_r) + 8
    for k in range(cfg.n_events):
        pos = k * cfg.slot_period + cfg.pulse_offset
        nk = int((pos - n0) % n_loop)
        lo = max(0, nk - 6)
        hi = min(n, nk + half)
        if lo < hi:
            keep[lo:hi] = False
    if keep.sum() < 64:
        return np.nan
    w = 2 * np.pi * f0 * t[keep]
    design = np.column_stack([np.cos(w), np.sin(w), np.ones_like(w)])
    coef, *_ = np.linalg.lstsq(design, y[keep], rcond=None)
    a, b, _ = coef
    return float(np.arctan2(-b, a))


if __name__ == "__main__":
    sys.exit(main())
