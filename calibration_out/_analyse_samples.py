"""Scratch: two open questions, answered offline from saved frames.

Q1  The closed loop prints SNDR ~37 dB but a few per cent of frames print ~29-30.
    Does the *raw* capture degrade too, or only the dither-subtracted metric?
    (raw flat + cal dropping => the dither estimate/subtraction is the culprit;
     both dropping => the capture or the analog input is.)

Q2  The skew observable scatters ~20-35 ps frame to frame, yet a 1020-sample tone
    fit with A~564 codes and 10.6-code noise should resolve the phase difference
    to well under 1 ps.  Where do the other ~30x come from?  Decomposed here by
    re-fitting both channels on one common frequency and comparing the per-frame
    phase difference against its own noise floor.
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
    estimate_block, fit_tone, prepare_capture, synthesize_dither,
)
from calibration_loop.metrics import analyse                           # noqa: E402

DIR = os.path.join(REPO, "calibration_out", "frames100")


def one(raw, cfg, f0):
    prep = prepare_capture(raw, cfg)
    a, b = prep["ch_a"], prep["ch_b"]
    est = estimate_block(a, b, cfg, n0=prep["n0"])
    fa = fit_tone(a, f0, refine=True)
    fb = fit_tone(b, f0, refine=True)
    da = synthesize_dither(a.size, prep["n0"], cfg, est.ch_a.gain_codes)
    db = synthesize_dither(b.size, prep["n0"], cfg, est.ch_b.gain_codes)
    sndr_raw = analyse(a, cfg.fs_adc)["sndr_db"]
    sndr_cal = analyse(a - da, cfg.fs_adc)["sndr_db"]

    # Q2: phase difference with each channel's own refined frequency versus one
    # common frequency (the estimator's own choice is the mean of the two).
    f_common = 0.5 * (fa["f0"] + fb["f0"])
    pa = fit_tone(a, f_common, refine=False)["phase"]
    pb = fit_tone(b, f_common, refine=False)["phase"]
    dphi_common = math.atan2(math.sin(pb - pa), math.cos(pb - pa))
    dphi_own = math.atan2(math.sin(fb["phase"] - fa["phase"]),
                          math.cos(fb["phase"] - fa["phase"]))
    ts = 1e12 / cfg.fs_adc
    ps_per_rad = 1.0 / (2.0 * math.pi * f_common) * ts

    # expected phase noise floor for this record (Cramer-Rao-ish): sigma_phase ~
    # sigma_noise / (A * sqrt(N/2)), converted to ps
    n = a.size
    sig_phase = float(np.std(fa["residual"])) / (fa["amplitude"] * math.sqrt(n / 2.0))
    floor_ps = sig_phase * math.sqrt(2.0) * ps_per_rad

    return {
        "margin": prep["align_margin"], "events": est.ch_a.n_events_used,
        "sndr_raw": sndr_raw, "sndr_cal": sndr_cal,
        "gain_a": est.ch_a.gain_codes, "gain_b": est.ch_b.gain_codes,
        "ratio": est.gain_ratio, "skew": est.skew_mismatch_ps,
        "src": est.skew_source,
        "phase_ps": est.skew_phase_ps,
        "dphi_common_ps": dphi_common * ps_per_rad,
        "dphi_own_ps": dphi_own * ps_per_rad,
        "df_milli": (fb["f0"] - fa["f0"]) * 1e6,      # frequency mismatch, 1e-9 cyc/smp
        "floor_ps": floor_ps,
        "resid_a": float(np.std(fa["residual"])),
    }


def stats(vals):
    v = [x for x in vals if isinstance(x, float) and math.isfinite(x)]
    return (st.mean(v), st.pstdev(v), len(v)) if v else (float("nan"), float("nan"), 0)


def main() -> int:
    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    files = sorted(glob.glob(os.path.join(DIR, "frame_*.bin")))
    if not files:
        print(f"no frames in {DIR}")
        return 1
    rows = []
    for path in files:
        with open(path, "rb") as fh:
            rows.append(one(fh.read(), cfg, f0))
    print(f"analysed {len(rows)} frames\n")

    # ---- Q1 -----------------------------------------------------------------
    m, s, n = stats([r["sndr_cal"] for r in rows])
    mr, sr, nr = stats([r["sndr_raw"] for r in rows])
    print(f"SNDR (loop metric): mean={m:.2f} +-{s:.2f}   raw: mean={mr:.2f} +-{sr:.2f}")
    low = [(i, r) for i, r in enumerate(rows) if r["sndr_cal"] < m - 3.0]
    print(f"frames >3 dB below the loop-metric mean: {len(low)}")
    print(f"{'idx':>4}{'SNDRcal':>9}{'SNDRraw':>9}{'margin':>8}{'ev':>4}{'residA':>8}"
          f"{'gainA':>8}{'ratio':>9}{'worst?':>8}")
    for i, r in low:
        print(f"{i:>4}{r['sndr_cal']:>9.2f}{r['sndr_raw']:>9.2f}{r['margin']:>8.1f}"
              f"{r['events']:>4}{r['resid_a']:>8.2f}{r['gain_a']:>8.2f}{r['ratio']:>9.5f}"
              f"{'':>8}")
    if low:
        idx = {i for i, _ in low}
        good = [r for i, r in enumerate(rows) if i not in idx]
        for key in ("sndr_raw", "margin", "events", "resid_a", "gain_a"):
            mg, _, _ = stats([float(r[key]) for r in good])
            ml, _, _ = stats([float(r[key]) for r in low])
            print(f"  {key:<11} low frames {ml:>9.3f}   rest {mg:>9.3f}   "
                  f"delta {ml - mg:+.3f}")

    # ---- Q2 -----------------------------------------------------------------
    print()
    for key in ("skew", "phase_ps", "dphi_common_ps", "dphi_own_ps", "df_milli", "floor_ps"):
        mm, ss, nn = stats([r[key] for r in rows])
        print(f"  {key:<16} mean={mm:>9.2f}  std={ss:>8.2f}  (n={nn}/{len(rows)})")
    srcs = {}
    for r in rows:
        srcs[r["src"]] = srcs.get(r["src"], 0) + 1
    print(f"  skew_source counts: {srcs}")


if __name__ == "__main__":
    raise SystemExit(main())
