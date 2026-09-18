"""Scratch: finish the story on the +-19 ps per-capture skew scatter.

The phase-route and the difference-signal route agree at corr -0.9995, so the
variation is in the samples, not in the fit.  Three questions remain:

1. Is the agreement real (sign-flip residual ~1 ps) or an artefact?
2. Does the scatter depend on anything in the frame metadata -- n0 (loop phase),
   event count, margin?  A dependence on n0 would mean a reconstruction artefact.
3. Is it white in capture order (hardware jitter) or slow (drift)?  The frames
   were captured back to back, about a second apart.
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

DIR = os.path.join(REPO, "calibration_out", "frames100")


def main() -> int:
    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    ts = 1e12 / cfg.fs_adc
    ps_per_rad = ts / (2 * math.pi * f0)
    files = sorted(glob.glob(os.path.join(DIR, "frame_*.bin")))
    if not files:
        print(f"no frames in {DIR}")
        return 1

    phase, diff, n0s, evs, mgs = [], [], [], [], []
    for path in files:
        with open(path, "rb") as fh:
            raw = fh.read()
        prep = prepare_capture(raw, cfg)
        a, b = prep["ch_a"], prep["ch_b"]
        est = estimate_block(a, b, cfg, n0=prep["n0"])
        f_c = 0.5 * (fit_tone(a, f0, refine=True)["f0"] + fit_tone(b, f0, refine=True)["f0"])
        pa = fit_tone(a, f_c, refine=False)["phase"]
        pb = fit_tone(b, f_c, refine=False)["phase"]
        dphi = math.atan2(math.sin(pb - pa), math.cos(pb - pa))
        d = a - b
        amp_ab = abs(fit_tone(d - d.mean(), f_c, refine=False)["amplitude"])
        amp_a = fit_tone(a, f_c, refine=False)["amplitude"]
        phase.append(dphi * ps_per_rad)
        diff.append(2.0 * math.asin(min(1.0, amp_ab / (2.0 * amp_a))) * ps_per_rad)
        n0s.append(prep["n0"])
        evs.append(est.ch_a.n_events_used)
        mgs.append(prep["align_margin"])

    phase = np.array(phase)
    diff = -np.array(diff)            # opposite sign convention (b-a vs a-b)
    print(f"phase route : {phase.mean():+8.2f} +- {phase.std():5.2f} ps")
    print(f"diff  route : {diff.mean():+8.2f} +- {diff.std():5.2f} ps")
    resid = phase - (phase.mean() - diff.mean()) - diff
    print(f"agreement residual after aligning means: rms = {resid.std():.2f} ps  "
          f"(=> the two routes measure the same thing)")
    print(f"corr(phase, diff) = {np.corrcoef(phase, diff)[0,1]:+.4f}")

    def cc(a1, b1):
        if np.std(a1) == 0 or np.std(b1) == 0:
            return float("nan")
        return float(np.corrcoef(a1, b1)[0, 1])

    n0 = np.array(n0s, dtype=float)
    ev = np.array(evs, dtype=float)
    mg = np.array(mgs, dtype=float)
    print("\ndependence on frame metadata:")
    print(f"  corr(skew, n0)            {cc(phase, n0):+.3f}")
    print(f"  corr(skew, n0 mod 8)      {cc(phase, n0 % 8):+.3f}")
    print(f"  corr(skew, n0 mod 8320/64){cc(phase, n0 % 130):+.3f}")
    print(f"  corr(skew, events_used)   {cc(phase, ev):+.3f}")
    print(f"  corr(skew, margin)        {cc(phase, mg):+.3f}")
    print(f"  n0 range {int(n0.min())}..{int(n0.max())}, unique {len(set(n0s))}/{len(n0s)}")

    print("\ntime structure in capture order (frames ~1 s apart):")
    for lag in (1, 2, 3, 5, 10):
        if len(phase) > lag:
            print(f"  lag-{lag:<2} autocorrelation {cc(phase[:-lag], phase[lag:]):+.3f}")

    # how big a step can a single frame resolve?  (2-sigma on one frame)
    print(f"\nper-frame 1-sigma scatter = {phase.std():.2f} ps")
    print(f"  a 13.8 ps control step is {13.8 / phase.std():.2f} sigma on ONE frame")
    for n in (1, 5, 10, 17, 22, 40):
        se = phase.std() / math.sqrt(n)
        print(f"  n={n:>3} valid frames -> SE {se:5.2f} ps -> "
              f"2-sigma resolves {2 * se:5.2f} ps")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
