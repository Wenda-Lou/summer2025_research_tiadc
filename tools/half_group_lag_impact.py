"""Quantify what a half-group de-frame error does to the estimates.

The pre-fix ``prepare_capture`` re-split at ``rot + 4`` whenever it decided the
converters were in the wrong order.  That re-split does not interleave the two
streams -- both stay spectrally clean -- so the damage is invisible in the tone
fit.  What it does is slide the two streams four samples apart, and that breaks
the assumption :func:`~calibration_loop.estimator.estimate_block` is built on:
one loop position ``n0`` is used to slice **both** channels' dither windows.

This tool runs both extractions on the *same* bytes and reports, per channel,
the tone residual, the recovered dither gain, and the resulting gain ratio, so
the mechanism can be tied to the bench symptoms (channel B residual and a gain
ratio pulled off truth) rather than asserted.

Usage:
    python tools/half_group_lag_impact.py [--frames 8] [--noise 3]
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def aligned_and_lagged(raw, cfg):
    """The two adjacent splits: the correct one and the half-group one."""
    from calibration_loop.estimator import prepare_capture, split_at, unpack_words

    words = unpack_words(raw)
    prep = prepare_capture(raw, cfg)
    rot = prep["rotation"]
    lag = split_at(words, (rot + 4) % 8)
    if prep["corr"] < 0:
        lag_b = -lag["ch_b"]
    else:
        lag_b = lag["ch_b"]
    return prep, (lag["ch_a"], lag_b)


def summarise(tag, a, b, cfg, f0):
    from calibration_loop.estimator import estimate_block, fit_tone

    ra = float(np.std(fit_tone(a, f0, refine=True)["residual"]))
    rb = float(np.std(fit_tone(b, f0, refine=True)["residual"]))
    c = float(np.corrcoef(a - a.mean(), b - b.mean())[0, 1]) if a.size == b.size else np.nan
    est = estimate_block(a, b, cfg, n0=None)
    # The estimator's own residual: pass 2 re-fits the tone after subtracting a
    # dither synthesised from the *shared* n0, so a channel that sits four
    # samples off that n0 has error injected into it here -- the plain tone fit
    # above cannot see that, because both streams stay spectrally clean.
    ea = est.ch_a.residual_rms
    eb = est.ch_b.residual_rms
    print(f"  {tag:<22} corr(A,B)={c:+.4f}  tone-resid A/B={ra:6.2f}/{rb:6.2f}  "
          f"est-resid A/B={ea:6.2f}/{eb:6.2f}")
    print(f"  {'':<22} gainA={est.ch_a.gain_codes:+8.3f} gainB={est.ch_b.gain_codes:+8.3f}"
          f"  g_B/g_A={est.gain_ratio:+.4f}  dSkew={est.skew_mismatch_ps:+8.2f} ps")
    return dict(corr=c, resid_a=ra, resid_b=rb, est_resid_a=ea, est_resid_b=eb,
                gain_a=est.ch_a.gain_codes, gain_b=est.ch_b.gain_codes,
                ratio=est.gain_ratio)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--frames", type=int, default=8)
    ap.add_argument("--words", type=int, default=2048)
    ap.add_argument("--noise", type=float, default=3.0)
    args = ap.parse_args(argv)

    sys.path.insert(0, REPO)
    from calibration_loop.dither import DitherConfig
    from calibration_loop.simulate import BenchModel

    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    bench = BenchModel(cfg=cfg, noise_rms_codes=args.noise, seed=7)
    truth = bench.gain_b / bench.gain_a
    print(f"model truth: g_B/g_A = {truth:+.5f}   "
          f"(invert_b={bench.invert_b}, noise={args.noise:g} codes, "
          f"frames={args.frames})")

    rows_a, rows_l = [], []
    for i in range(args.frames):
        raw = bench.capture(n_words=args.words)
        prep, (la, lb) = aligned_and_lagged(raw, cfg)
        print(f"\nframe {i}  (rotation={prep['rotation']}, lagged={prep['swapped']})")
        rows_a.append(summarise("correct (aligned)", prep["ch_a"], prep["ch_b"], cfg, f0))
        rows_l.append(summarise("half-group lagged", la, lb, cfg, f0))

    def agg(rows, key):
        v = [r[key] for r in rows if np.isfinite(r[key])]
        return (float(np.mean(v)), float(np.std(v))) if v else (np.nan, np.nan)

    print("\n" + "=" * 78)
    print(f"{'metric':<24}{'correct':>18}{'half-group lagged':>22}")
    for key, label, fmt in (("corr", "corr(A,B)", "{:+.4f}"),
                            ("resid_a", "tone resid chA", "{:.2f}"),
                            ("resid_b", "tone resid chB", "{:.2f}"),
                            ("est_resid_a", "est resid chA", "{:.2f}"),
                            ("est_resid_b", "est resid chB", "{:.2f}"),
                            ("gain_a", "gainA", "{:+.4f}"),
                            ("gain_b", "gainB", "{:+.4f}"),
                            ("ratio", "g_B/g_A", "{:+.4f}")):
        ma, sa = agg(rows_a, key)
        ml, sl = agg(rows_l, key)
        print(f"{label:<24}{fmt.format(ma) + f' +-{sa:<8.4f}':>18}"
              f"{fmt.format(ml) + f' +-{sl:<8.4f}':>22}")
    print(f"\n  truth g_B/g_A = {truth:+.5f}")
    print("\nReading: the plain tone fit is blind to the lag (both streams stay")
    print("spectrally clean), but the estimator's own residual and the recovered")
    print("dither gains are not -- pass 2 subtracts a dither synthesised from the")
    print("shared n0, so the lagged channel has error injected into it.  A frame")
    print("can therefore look clean on a tone fit and still be corrupted.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
