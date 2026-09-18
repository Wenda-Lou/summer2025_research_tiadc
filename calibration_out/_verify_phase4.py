"""Scratch: verify phase-4 (batch skew acceptance + arithmetic mean) on the 100
saved bench frames, through the production code path.

Checks, in order:
  1. the new estimator field skew_diff_route_ps agrees with the phase route
     (the sign convention is the thing most likely to be wrong);
  2. skew_batch's acceptance filter reproduces the offline analysis: ~75 % yield
     and the 19 recovered "observables-disagree" frames;
  3. SE of an N-frame batch equals sigma/sqrt(N) on the accepted population;
  4. the median/trimmed mean really are worse, as the docstring claims.
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
    estimate_block, fit_tone, prepare_capture, skew_batch,
)

DIR = os.path.join(REPO, "calibration_out", "frames100")


def main() -> int:
    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    files = sorted(glob.glob(os.path.join(DIR, "frame_*.bin")))
    if not files:
        print(f"no frames in {DIR}")
        return 1

    frames = []
    for path in files:
        with open(path, "rb") as fh:
            raw = fh.read()
        prep = prepare_capture(raw, cfg)
        a, b = prep["ch_a"], prep["ch_b"]
        est = estimate_block(a, b, cfg, n0=prep["n0"])
        fit = fit_tone(a, f0, refine=True)
        frames.append({
            "phase_ps": est.skew_phase_ps,
            "diff_route_ps": est.skew_diff_route_ps,
            "centroid_ps": est.skew_centroid_ps,
            "source": est.skew_source,
            "tone": fit["amplitude"],
            "resid": float(np.std(fit["residual"])),
            "margin": prep["align_margin"],
        })

    ph = np.array([f["phase_ps"] for f in frames], dtype=float)
    df = np.array([f["diff_route_ps"] for f in frames], dtype=float)
    ok = np.isfinite(ph) & np.isfinite(df)
    print(f"1. route agreement (production fields): n={int(ok.sum())}/{len(frames)}")
    print(f"   corr = {np.corrcoef(ph[ok], df[ok])[0,1]:+.4f}   "
          f"rms(diff-phase) = {np.sqrt(np.mean((ph[ok]-df[ok])**2)):.3f} ps   "
          f"max |d| = {np.max(np.abs(ph[ok]-df[ok])):.3f} ps")

    batch = skew_batch(frames)
    print(f"\n2. skew_batch: used {batch.n_used}/{batch.n_total} "
          f"({100.0*batch.n_used/batch.n_total:.0f} %)  mean {batch.mean_ps:+.2f} ps  "
          f"se {batch.se_ps:.2f} ps  ci95 +-{batch.ci95_half_ps:.2f} ps")
    src = [f["source"] for f in frames]
    kept_src = {}
    for f in frames:
        pass
    disagree = sum(1 for f in frames if f["source"] == "observables-disagree")
    print(f"   rejected reasons: {batch.rejected}")
    print(f"   frames whose source was 'observables-disagree': {disagree} "
          f"(the filter is allowed to keep these when phase and diff agree)")

    kept = np.array([f["phase_ps"] for f in frames
                     if f["source"] != "profile-fallback" and np.isfinite(f["phase_ps"])],
                    dtype=float)
    pop = np.array([f["phase_ps"] for f in frames], dtype=float)
    acc_mask = np.isfinite(pop)
    accepted = pop[acc_mask]
    sigma = float(accepted.std(ddof=1))
    print(f"\n3. accepted population: n={accepted.size}  mean {accepted.mean():+.2f} ps  "
          f"sigma {sigma:.2f} ps")

    rng = np.random.default_rng(20260917)
    B = 20000
    print(f"\n4. batch statistics vs N (bootstrap {B} draws on the accepted population)")
    print(f"{'N':>4}{'mean SE':>10}{'sigma/sqrtN':>13}{'median SE':>11}{'trim10 SE':>11}")
    for n in (5, 10, 20, 30, 50):
        s = accepted[rng.integers(0, accepted.size, size=(B, n))]
        m = s.mean(axis=1)
        md = np.median(s, axis=1)
        srt = np.sort(s, axis=1)
        k = max(1, int(n * 0.10))
        tm = srt[:, k:n - k].mean(axis=1) if n - 2 * k > 0 else srt.mean(axis=1)
        print(f"{n:>4}{m.std(ddof=1):>10.2f}{sigma/math.sqrt(n):>13.2f}"
              f"{md.std(ddof=1):>11.2f}{tm.std(ddof=1):>11.2f}")
    print("\nreading: mean SE must match sigma/sqrt(N) and must beat median/trimmed;")
    print("both are the claim the SkewBatch docstring makes.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
