"""Scratch: is the +-19 ps frame-to-frame skew scatter physical, or an artefact of
dither leakage into the tone-phase estimate?

Both the fitted phase difference and the A-B difference-signal power are linear
functionals of the same two tones, so agreement between them cannot decide the
question on its own.  What can decide it is changing *how the tone is fitted*
while leaving the samples untouched:

  raw       fit both channels with the dither still in the record
  twopass   subtract the dither synthesised from the per-frame gain, then fit
  masked    weighted fit that ignores every dither-event window

If the scatter collapses under `masked`/`twopass`, the +-19 ps lives in the
estimator (dither leakage), and a better phase estimate makes one 13.8 ps control
step resolvable with far fewer frames.  If all three variants scatter the same
AND a fit-free observable agrees, the variation is in the data itself.

Run offline on frames saved by _capture_frames.py:
    python calibration_out/_phase_variants.py
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
    adc_templates, estimate_block, fit_tone, prepare_capture, synthesize_dither,
    visible_events,
)

DIR = os.path.join(REPO, "calibration_out", "frames100")


def fit_weighted(y, f0, w):
    """Weighted a*cos + b*sin + dc fit at a fixed frequency."""
    n = np.arange(y.size, dtype=np.float64)
    c, s = np.cos(2 * np.pi * f0 * n), np.sin(2 * np.pi * f0 * n)
    design = np.column_stack([c, s, np.ones_like(n)])
    wd = design * w[:, None]
    coef, *_ = np.linalg.lstsq(design.T @ wd, wd.T @ y, rcond=None)
    a, b, dc = coef
    return math.atan2(-b, a), math.hypot(a, b), y - design @ coef


def phase_pair(a, b, f0, wa=None, wb=None):
    if wa is None:
        pa, aa, ra = (lambda f: (f["phase"], f["amplitude"], f["residual"]))(fit_tone(a, f0))
        pb, ab, rb = (lambda f: (f["phase"], f["amplitude"], f["residual"]))(fit_tone(b, f0))
    else:
        pa, aa, ra = fit_weighted(a, f0, wa)
        pb, ab, rb = fit_weighted(b, f0, wb)
    d = math.atan2(math.sin(pb - pa), math.cos(pb - pa))
    return d, aa, ab, float(np.std(ra))


def main() -> int:
    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    ts = 1e12 / cfg.fs_adc
    ps_per_rad = ts / (2 * math.pi * f0)
    m, _, _ = adc_templates(cfg)
    m_lo, m_hi = int(m[0]), int(m[-1])
    files = sorted(glob.glob(os.path.join(DIR, "frame_*.bin")))
    if not files:
        print(f"no frames in {DIR}")
        return 1

    out = {k: [] for k in ("raw", "twopass", "masked", "diffmetric")}
    for path in files:
        with open(path, "rb") as fh:
            raw = fh.read()
        prep = prepare_capture(raw, cfg)
        a, b, n0 = prep["ch_a"], prep["ch_b"], prep["n0"]
        est = estimate_block(a, b, cfg, n0=n0)
        da = synthesize_dither(a.size, n0, cfg, est.ch_a.gain_codes)
        db = synthesize_dither(b.size, n0, cfg, est.ch_b.gain_codes)

        f_common = 0.5 * (fit_tone(a, f0, refine=True)["f0"]
                          + fit_tone(b, f0, refine=True)["f0"])

        out["raw"].append(phase_pair(a, b, f_common)[0] * ps_per_rad)
        out["twopass"].append(phase_pair(a - da, b - db, f_common)[0] * ps_per_rad)

        # mask out every dither event window, plus a margin either side
        mask = np.ones(a.size)
        ks, starts = visible_events(n0, a.size, cfg, m_lo, m_hi)
        for s0 in starts:
            lo, hi = max(0, s0 - 2), min(a.size, s0 + (m_hi - m_lo) + 3)
            mask[lo:hi] = 0.0
        if mask.sum() > a.size // 3:
            out["masked"].append(phase_pair(a, b, f_common, mask, mask)[0] * ps_per_rad)
        else:
            out["masked"].append(float("nan"))

        # fit-free route: the A-B difference signal's tone amplitude gives the
        # relative shift directly,  amp = 2*A*|sin(dphi/2)|
        d = a - b
        d = d - d.mean()
        amp_ab = abs(fit_tone(d, f_common, refine=False)["amplitude"])
        amp_a = fit_tone(a, f_common, refine=False)["amplitude"]
        ratio = min(1.0, amp_ab / (2.0 * amp_a)) if amp_a > 0 else 0.0
        out["diffmetric"].append(2.0 * math.asin(ratio) * ps_per_rad)

    def rep(key):
        v = [x for x in out[key] if math.isfinite(x)]
        mean = st.mean(v)
        # scatter about the *mean phase* is what matters, not frame-to-frame jumps
        return mean, st.pstdev(v), len(v), np.mean(np.abs(np.diff(v))) if len(v) > 1 else float("nan")

    print(f"{'variant':<12}{'mean ps':>10}{'std ps':>9}{'n':>5}{'mean |jump|':>13}")
    for key in ("raw", "twopass", "masked", "diffmetric"):
        mean, sd, n, jump = rep(key)
        print(f"{key:<12}{mean:>10.2f}{sd:>9.2f}{n:>5}{jump:>13.2f}")

    r = np.array([x for x in out["raw"]], dtype=float)
    dm = np.array(out["diffmetric"], dtype=float)
    ok = np.isfinite(r) & np.isfinite(dm)
    if ok.sum() > 3:
        c = float(np.corrcoef(r[ok], dm[ok])[0, 1])
        print(f"\ncorr(fitted phase diff, difference-signal phase) = {c:+.4f}  "
              f"(n={int(ok.sum())})")
        print(f"  rms of (phase - difference metric) = {np.sqrt(np.mean((r[ok]-dm[ok])**2)):.2f} ps")
    print("\nreading: if 'masked' collapses to ~1-2 ps the scatter is dither leakage")
    print("in the tone fit; if all variants agree at ~19 ps it is in the samples.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
