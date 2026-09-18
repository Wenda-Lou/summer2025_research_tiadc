"""Scratch: is the margin>=6 acceptance threshold buying anything?

The neutral initialization (enabling fine-delay mode with a proper 0x0114 code)
moved the operating point, and the frame acceptance rate fell from ~75 % to
~55 %: more frames now land below the margin 6 threshold.  Before touching the
threshold, answer three questions with data:

  Q1  what does each candidate threshold cost and buy?
  Q2  are the frames it rejects *biased*, or merely noisier?  If their mean sits
      inside the accepted set's confidence interval, rejecting them is pure loss.
  Q3  how many acquisitions per point does each choice need for 80/90/95 %
      detection power of one 13.8 ps control code?

Compares the pre-initialization batch with the neutral-state batch, so the effect
of the operating point is visible rather than assumed.
"""

from __future__ import annotations

import glob
import math
import os
import statistics as st
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.dither import DitherConfig                       # noqa: E402
from calibration_loop.estimator import (                               # noqa: E402
    estimate_block, fit_tone, prepare_capture,
)

STEP_PS = 13.8
THRESHOLDS = (4.5, 5.0, 5.5, 6.0, 6.5)
POWERS = (0.80, 0.90, 0.95)


def load(folder, cfg, f0):
    files = sorted(glob.glob(os.path.join(folder, "frame_*.bin")))
    rows = []
    for path in files:
        with open(path, "rb") as fh:
            raw = fh.read()
        prep = prepare_capture(raw, cfg)
        a, b = prep["ch_a"], prep["ch_b"]
        est = estimate_block(a, b, cfg, n0=prep["n0"])
        fit = fit_tone(a, f0, refine=True)
        rows.append({
            "margin": prep["align_margin"],
            "phase_ps": est.skew_phase_ps,
            "diff_route_ps": est.skew_diff_route_ps,
            "source": est.skew_source,
            "tone": fit["amplitude"],
            "resid": float(np.std(fit["residual"])),
        })
    return rows


def healthy(rows):
    """The acceptance filter minus the margin criterion (margin handled separately)."""
    tone = np.median([r["tone"] for r in rows])
    resid = np.median([r["resid"] for r in rows])
    out = []
    for r in rows:
        ok = (r["source"] != "profile-fallback"
              and np.isfinite(r["phase_ps"])
              and (not np.isfinite(r["diff_route_ps"])
                   or abs(r["phase_ps"] - r["diff_route_ps"]) <= 5.0)
              and abs(r["tone"] - tone) <= 0.10 * tone
              and 0.5 * resid < r["resid"] < 2.0 * resid)
        out.append(ok)
    return np.array(out)


def study(tag, rows):
    from calibration_loop.estimator import skew_batch

    n = len(rows)
    ok = healthy(rows)
    margins = np.array([r["margin"] for r in rows])
    phase = np.array([r["phase_ps"] for r in rows], dtype=float)
    print(f"\n=== {tag}: {n} frames ===")
    print(f"  healthy (all but margin): {int(ok.sum())}/{n}")
    print(f"  margin: min {np.nanmin(margins):.2f}  p10 {np.nanpercentile(margins,10):.2f}  "
          f"median {np.nanmedian(margins):.2f}  p90 {np.nanpercentile(margins,90):.2f}  "
          f"max {np.nanmax(margins):.2f}")

    print(f"\n  {'thr':>5}{'accept':>9}{'yield':>8}{'mean ps':>10}{'sigma':>8}{'SE':>7}"
          f"{'frames@80%':>12}{'@90%':>7}{'@95%':>7}")
    for thr in THRESHOLDS:
        keep = ok & (margins >= thr) & np.isfinite(phase)
        k = int(keep.sum())
        if k < 5:
            print(f"  {thr:>5.1f}{k:>9}{'--':>8}")
            continue
        vals = phase[keep]
        sigma = float(vals.std(ddof=1))
        se = sigma / math.sqrt(k)
        yield_frac = k / n
        row = f"  {thr:>5.1f}{k:>9}{yield_frac:>8.2f}{vals.mean():>10.2f}{sigma:>8.2f}{se:>7.2f}"
        for p in POWERS:
            z = {0.80: 2.802, 0.90: 3.242, 0.95: 3.605}[p]
            n_valid = (z * sigma / STEP_PS) ** 2
            row += f"{n_valid / yield_frac:>12.0f}" if p == 0.80 else f"{n_valid / yield_frac:>7.0f}"
        print(row)

    # Q2: are the rejected-by-margin frames biased?
    print("\n  bias test (frames the margin 6 threshold rejects):")
    keep6 = ok & (margins >= 6.0) & np.isfinite(phase)
    low = ok & (margins < 6.0) & np.isfinite(phase)
    if keep6.sum() >= 5 and low.sum() >= 5:
        a, b = phase[keep6], phase[low]
        se_a, se_b = a.std(ddof=1) / math.sqrt(a.size), b.std(ddof=1) / math.sqrt(b.size)
        diff = b.mean() - a.mean()
        comb = math.hypot(se_a, se_b)
        print(f"    accepted  n={a.size:>3}  mean {a.mean():+8.2f} +-{se_a:.2f}  sigma {a.std(ddof=1):.2f}")
        print(f"    rejected  n={b.size:>3}  mean {b.mean():+8.2f} +-{se_b:.2f}  sigma {b.std(ddof=1):.2f}")
        print(f"    difference {diff:+.2f} ps  ({abs(diff)/comb:.2f} sigma of the difference)")
        print(f"    -> {'BIASED: the threshold is removing real bias' if abs(diff) > 2*comb else 'no detectable bias: the threshold is costing frames for nothing'}")
    else:
        print(f"    not enough frames on one side (accepted {int(keep6.sum())}, "
              f"rejected {int(low.sum())})")
    batch = skew_batch(rows)  # the production filter, for reference
    print(f"\n  production filter at threshold 6.0: used {batch.n_used}/{batch.n_total}  "
          f"mean {batch.mean_ps:+.2f} +-{batch.ci95_half_ps:.2f}")


def main() -> int:
    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    folders = [
        ("pre-initialization (mode 0x00)", os.path.join(REPO, "calibration_out", "frames100")),
        ("neutral (mode 0x04 + 0x0114=0x60)", os.path.join(REPO, "calibration_out", "frames_neutral")),
    ]
    found = False
    for tag, folder in folders:
        if not os.path.isdir(folder) or not glob.glob(os.path.join(folder, "frame_*.bin")):
            print(f"(skipping {tag}: no frames in {folder})")
            continue
        found = True
        study(tag, load(folder, cfg, f0))
    if not found:
        return 1
    print("\nreading: if the rejected frames are unbiased, lowering the threshold to")
    print("5.5 or 5.0 buys frames at no accuracy cost; if they are biased, the")
    print("threshold is doing its job and the only lever is more acquisitions.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
