"""Does the converged skew state hold, with nothing being written?

Read-only: this captures frames and measures them, nothing else.  It exists because
the closed loop's own log stops at the end of the run, so nothing in a bench
session answers "is the state the run left behind still there five minutes later,
or is a periodic re-convergence needed?".

Each round measures a batch of frames through the production path and reports the
batch skew mean +- CI and the A-B difference spur, then pauses.  Two things are
compared at the end:

  * the skew drift over the whole window, as a rate in ps/minute, with its slope
    uncertainty -- a drift smaller than the uncertainty means "stable";
  * the measured spur against ``20*log10|2 sin(pi f dt)|`` for the final skew,
    which is what the skew residual actually costs.  If the two agree, the state
    is skew-limited and nothing else is in the way.

Measured on 2026-09-17 at the converged code 31 (7 rounds, 340 s): drift
+0.01 ps/minute (slope uncertainty +-0.20), spur spread 0.57 dB, and the prediction
matched to 0.68 dB -- so no periodic re-convergence was needed on a minutes
timescale.  The state was also left alone by this tool, which is its point.

    python tools/skew_stability_check.py --uart COM5
    python tools/skew_stability_check.py --uart COM5 --rounds 3 --pause 20   # quick
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from skew_step_characterize import frame_record          # noqa: E402


def diff_dbc(a: np.ndarray, b: np.ndarray, f0: float) -> float:
    """A-B difference spur in dBc, without subtracting the dither."""
    from calibration_loop.estimator import fit_tone

    d = np.asarray(a, dtype=float) - np.asarray(b, dtype=float)
    d = d - d.mean()
    a_amp = fit_tone(np.asarray(a, dtype=float), f0, refine=False)["amplitude"]
    d_amp = abs(fit_tone(d, f0, refine=False)["amplitude"])
    return 20.0 * math.log10(max(d_amp, 1e-9) / max(abs(a_amp), 1e-9))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--rounds", type=int, default=7)
    ap.add_argument("--frames", type=int, default=20, help="frames per round")
    ap.add_argument("--pause", type=float, default=40.0, help="seconds between rounds")
    ap.add_argument("--out", default=os.path.join(REPO, "calibration_out"))
    args = ap.parse_args(argv)

    from calibration_loop.capture import HardwareBench
    from calibration_loop.dither import DitherConfig
    from calibration_loop.estimator import prepare_capture, skew_batch

    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period

    bench = HardwareBench(uart_port=args.uart)   # read-only: no writes allowed
    rows: list[dict] = []
    try:
        print(f"{'t s':>6}{'used':>7}{'mean ps':>10}{'ci95':>7}{'A-B dBc':>10}")
        t0 = time.time()
        for r in range(args.rounds):
            recs, dbcs = [], []
            for _ in range(args.frames):
                raw = bench.capture()
                if not raw:
                    continue
                prep = prepare_capture(raw, cfg)
                recs.append(frame_record(raw, cfg, f0))
                dbcs.append(diff_dbc(prep["ch_a"], prep["ch_b"], f0))
            batch = skew_batch(recs)
            rows.append({
                "round": r, "t_s": time.time() - t0,
                "n_used": batch.n_used, "n_total": batch.n_total,
                "mean_ps": batch.mean_ps, "ci95_half_ps": batch.ci95_half_ps,
                "se_ps": batch.se_ps,
                "spur_dbc": float(np.mean(dbcs)) if dbcs else float("nan"),
                "rejected": dict(batch.rejected),
            })
            print(f"{rows[-1]['t_s']:>6.0f}{batch.n_used:>4}/{batch.n_total:<2}"
                  f"{batch.mean_ps:>10.2f}{batch.ci95_half_ps:>7.2f}"
                  f"{rows[-1]['spur_dbc']:>10.2f}")
            if r + 1 < args.rounds:
                time.sleep(args.pause)
    finally:
        bench.close()

    if not rows:
        print("no frames captured")
        return 1

    means = np.array([r["mean_ps"] for r in rows], dtype=float)
    ses = np.array([r["se_ps"] for r in rows], dtype=float)
    dbcs = np.array([r["spur_dbc"] for r in rows], dtype=float)
    t = np.array([r["t_s"] for r in rows], dtype=float)

    slope, intercept = np.polyfit(t, means, 1) if len(rows) > 1 else (0.0, means[0])
    resid = means - (slope * t + intercept)
    slope_se = (np.sqrt((resid ** 2).sum() / max(len(rows) - 2, 1)
                        / ((t - t.mean()) ** 2).sum())
                if len(rows) > 2 else float("nan"))

    print(f"\nskew: first {means[0]:+.2f}  last {means[-1]:+.2f}  "
          f"range {means.max() - means.min():.2f} ps over {t[-1]:.0f} s")
    print(f"      batch spread {means.std(ddof=1):.2f} ps "
          f"(per-round SE {np.nanmean(ses):.2f} ps)")
    print(f"      linear drift {slope * 60:+.2f} +- {slope_se * 60:.2f} ps/minute")
    print(f"A-B : first {dbcs[0]:.2f}  last {dbcs[-1]:.2f}  "
          f"spread {dbcs.std(ddof=1):.2f} dB")
    f_hz = f0 * cfg.fs_adc
    pred = 20.0 * math.log10(abs(2.0 * math.sin(math.pi * f_hz * means[-1] * 1e-12))
                             + 1e-15)
    print(f"      prediction from the final skew ({means[-1]:+.2f} ps at "
          f"{f_hz / 1e6:.1f} MHz): {pred:.2f} dBc, measured {dbcs[-1]:.2f} dBc "
          f"(gap {dbcs[-1] - pred:+.2f} dB)")

    deadband = 10.0
    if np.isfinite(slope_se) and abs(slope * 60) <= max(3.0 * slope_se * 60, 1.0):
        print(f"  -> drift is not distinguishable from zero; the converged point "
              f"holds (deadband {deadband:.0f} ps)")
    elif abs(slope * 60) > 6.0:
        print(f"  -> drifting faster than the {deadband:.0f} ps deadband per minute; "
              f"plan a periodic re-convergence")
    else:
        print(f"  -> slow drift; fine for a minutes-long run, re-check before a long one")

    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, "skew_stability.csv")
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"round history: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
