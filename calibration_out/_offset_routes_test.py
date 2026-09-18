"""Scratch: which route to the channel DC does the offset loop actually close on?

The loop integrates ``ChannelEstimate.offset_codes``: the flat-top mean of the
*polarity-decoupled* pulse-window profile, measured on the tone-removed residual
(``estimator.py:526-546, 595``).  Three other routes to the same DC exist in the data:

  * ``offset_record_codes`` -- the DC term of the whole-record tone fit;
  * the plain record mean of each prepared channel;
  * ``cal_dc_difference_codes`` (in the run CSV) -- ``mean(cal_a) - mean(cal_b)`` of
    the *dither-subtracted but tone-retaining* corrected records.

The run CSVs show the loop's reported residual and that last quantity are strongly
ANTI-correlated (-0.69 to -0.93) with both means ~0, which cannot happen if both are
measuring one DC mismatch up to noise.  This measures all routes on the same archived
bench frames (real hardware data, no bench time) to find out which is contaminated.

    python calibration_out/_offset_routes_test.py
"""

from __future__ import annotations

import glob
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.dither import DitherConfig                        # noqa: E402
from calibration_loop.estimator import (                                # noqa: E402
    estimate_block, fit_tone, polarity_anchor, prepare_capture,
)

FRAMES = os.path.join(REPO, "calibration_out", "dither_vs_tone_frames")


def _dither_sub_mismatch(prep, est, cfg, polarity) -> float:
    """``mean(cal_b - d_b) - mean(cal_a - d_a)``: the run's cal_dc_difference_codes.

    The synthesized train is built from the *estimated* dither amplitude, so whatever
    that estimate gets wrong is left in the record as a residue -- and the window
    route explicitly decouples exactly that term (``estimator.py:541-546``).
    """
    from calibration_loop.estimator import synthesize_dither

    out = []
    for ch, e in ((prep["ch_a"], est.ch_a), (prep["ch_b"], est.ch_b)):
        d = synthesize_dither(ch.size, prep["n0"], cfg, e.gain_codes,
                              polarity_sign=polarity)
        out.append((ch - d).mean())
    return out[1] - out[0]


def main() -> int:
    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period

    files = sorted(glob.glob(os.path.join(FRAMES, "frame_*.bin")))
    if not files:
        print(f"no frames in {FRAMES}")
        return 1

    rows = []
    signature = polarity = None
    for path in files:
        raw = open(path, "rb").read()
        prep = prepare_capture(raw, cfg, signature=signature)
        if signature is None:
            signature = prep["signature"]
        if polarity is None:
            polarity = polarity_anchor(prep)
        est = estimate_block(prep["ch_a"], prep["ch_b"], cfg, cancel_signal=True,
                             n0=prep["n0"], polarity_sign=polarity, pin_polarity=True)
        # whole-record tone fit per channel: gives dc and amplitude
        fa = fit_tone(prep["ch_a"], f0, refine=False)
        fb = fit_tone(prep["ch_b"], f0, refine=False)
        rows.append({
            "win": est.ch_b.offset_codes - est.ch_a.offset_codes,
            "fit": fb["dc"] - fa["dc"],
            "rec": prep["ch_b"].mean() - prep["ch_a"].mean(),
            "win_a": est.ch_a.offset_codes, "fit_a": fa["dc"], "rec_a": prep["ch_a"].mean(),
            "amp_a": abs(fa["amplitude"]), "amp_b": abs(fb["amplitude"]),
            "n0": prep["n0"], "n_ev": est.ch_a.n_events_used,
            # the run's own route: subtract the *estimated* dither train (scaled by the
            # state gain, exactly as loop._measure does), then take the record means
            "rec_sub": _dither_sub_mismatch(prep, est, cfg, polarity),
        })

    def col(key):
        return np.array([r[key] for r in rows], dtype=float)

    n = len(rows)
    print(f"{n} archived bench frames (converged state, real hardware)\n")
    print(f"{'route (B - A)':>28}{'mean':>10}{'sd':>9}   what it is")
    print(f"{'pulse-window (loop)':>28}{col('win').mean():>10.3f}{col('win').std(ddof=1):>9.3f}"
          f"   flat-top of the decoupled window profile, tone removed")
    print(f"{'whole-record tone fit dc':>28}{col('fit').mean():>10.3f}"
          f"{col('fit').std(ddof=1):>9.3f}   fit_tone(...)['dc'], tone removed")
    print(f"{'plain record mean':>28}{col('rec').mean():>10.3f}"
          f"{col('rec').std(ddof=1):>9.3f}   mean() of the record, tone NOT removed")

    print("\nper-channel DC by each route (codes):")
    for ch in ("a",):
        print(f"   channel {ch.upper()}: window {col('win_'+ch).mean():+8.3f}  "
              f"fit {col('fit_'+ch).mean():+8.3f}  record {col('rec_'+ch).mean():+8.3f}")

    print("\npairwise correlations across frames:")
    for x, y in (("win", "fit"), ("win", "rec"), ("fit", "rec")):
        print(f"   {x:>4} vs {y:<4}: {np.corrcoef(col(x), col(y))[0, 1]:+.3f}")

    sub = col("rec_sub")
    print(f"\nthe run's own route (dither subtracted before the record mean):")
    print(f"   mean {sub.mean():+.3f}  sd {sub.std(ddof=1):.3f} codes"
          f"   (unsubtracted record route sd {col('rec').std(ddof=1):.3f})")
    print(f"   corr with the window route : {np.corrcoef(col('win'), sub)[0, 1]:+.3f}"
          f"   <- the run CSVs showed -0.69 to -0.93 for exactly this pair")
    print(f"   corr with the record route : {np.corrcoef(col('rec'), sub)[0, 1]:+.3f}")
    print(f"   in-band: the dither subtraction moved the mismatch by "
          f"{(sub - col('rec')).mean():+.3f} +- {(sub - col('rec')).std(ddof=1):.3f} codes")

    amp = 0.5 * (col("amp_a") + col("amp_b"))
    print(f"\ntone amplitude ~{amp.mean():.1f} codes; the record mean of a tone that is "
          f"not a whole number of cycles carries a leakage term")
    print(f"   cycles per 1020-sample capture: {f0 * 1020:.4f}"
          f"  (fractional part {f0 * 1020 % 1:.4f} -> up to "
          f"{amp.mean() * 2 * np.pi * 0.5 * abs(np.sin(np.pi * (f0 * 1020 % 1))) / (2 * np.pi * f0 * 1020):.2f}"
          f" codes of DC from a half-cycle-halved sine)")
    print(f"   corr(record mean, tone amplitude): "
          f"{np.corrcoef(col('rec'), amp)[0, 1]:+.3f}")

    # does the record-mean route track |A-B| tone difference (i.e. leakage)?
    d_ab = col("rec") - col("fit")
    print(f"\nrecord route minus fit route: {d_ab.mean():+.3f} +- "
          f"{d_ab.std(ddof=1):.3f} codes -- the part of the record mean that is not DC")
    print(f"   corr(that part, frames with n0 change): "
          f"{np.corrcoef(d_ab, np.diff(np.r_[col('n0')[0], col('n0')]))[0, 1]:+.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
