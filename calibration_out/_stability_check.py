"""Scratch: does the converged skew state hold, with nothing being written?

Read-only. Seven rounds, ~50 s apart, each round a batch of frames through the
production path: batch skew mean +- CI, and the A-B difference spur
(20*log10|2 sin(pi f dt)|), which is what an improved skew actually buys.

Answers: is the converged point stable enough to trust, how far does it drift over
minutes, and is a periodic re-convergence worth planning for.
"""

from __future__ import annotations

import math
import os
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "tools"))

from skew_step_characterize import frame_record          # noqa: E402

ROUNDS = 7
FRAMES = 20
PAUSE_S = 40.0


def diff_dbc(a, b, f0):
    from calibration_loop.estimator import fit_tone
    d = a - b
    d = d - d.mean()
    A = fit_tone(a, f0, refine=False)["amplitude"]
    D = abs(fit_tone(d, f0, refine=False)["amplitude"])
    return 20.0 * math.log10(max(D, 1e-9) / max(A, 1e-9))


def main() -> int:
    from calibration_loop.capture import HardwareBench
    from calibration_loop.dither import DitherConfig
    from calibration_loop.estimator import prepare_capture, skew_batch

    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period

    bench = HardwareBench(uart_port="COM5")   # read-only: no writes allowed
    rows = []
    try:
        print(f"{'t s':>6}{'used':>7}{'mean ps':>10}{'ci95':>7}{'A-B dBc':>10}")
        t0 = time.time()
        for r in range(ROUNDS):
            recs, dbcs = [], []
            for _ in range(FRAMES):
                raw = bench.capture()
                if not raw:
                    continue
                prep = prepare_capture(raw, cfg)
                recs.append(frame_record(raw, cfg, f0))
                dbcs.append(diff_dbc(prep["ch_a"], prep["ch_b"], f0))
            b = skew_batch(recs)
            rows.append((time.time() - t0, b, float(np.mean(dbcs)) if dbcs else float("nan")))
            print(f"{rows[-1][0]:>6.0f}{b.n_used:>4}/{b.n_total:<2}"
                  f"{b.mean_ps:>10.2f}{b.ci95_half_ps:>7.2f}{rows[-1][2]:>10.2f}")
            if r + 1 < ROUNDS:
                time.sleep(PAUSE_S)
    finally:
        bench.close()

    means = np.array([r[1].mean_ps for r in rows])
    ses = np.array([r[1].se_ps for r in rows])
    dbcs = np.array([r[2] for r in rows])
    print(f"\nskew: first {means[0]:+.2f}  last {means[-1]:+.2f}  "
          f"drift {means[-1] - means[0]:+.2f} ps over {rows[-1][0]:.0f} s")
    print(f"      spread across rounds {means.std(ddof=1):.2f} ps  "
          f"(per-round SE {ses.mean():.2f} ps -> expected spread {ses.mean():.2f})")
    print(f"A-B : first {dbcs[0]:.2f}  last {dbcs[-1]:.2f}  drift {dbcs[-1] - dbcs[0]:+.2f} dB")
    print(f"      spread {dbcs.std(ddof=1):.2f} dB")
    # a slow monotonic walk would show up as a trend against time
    t = np.array([r[0] for r in rows])
    slope = float(np.polyfit(t, means, 1)[0])
    print(f"linear drift rate: {slope * 60:+.2f} ps/minute")
    f_hz = f0 * cfg.fs_adc
    pred = 20.0 * math.log10(abs(2.0 * math.sin(math.pi * f_hz * means[-1] * 1e-12)) + 1e-15)
    print(f"predicted A-B spur from the final skew ({means[-1]:+.2f} ps at "
          f"{f_hz / 1e6:.1f} MHz): {pred:.2f} dBc  (measured {dbcs[-1]:.2f} dBc, "
          f"gap {dbcs[-1] - pred:+.2f} dB)")
    if abs(slope * 60) > 6.0:
        print("  -> drifting faster than the 10 ps deadband per minute; periodic "
              "re-convergence will be needed")
    else:
        print("  -> stable enough that the converged point holds on a minutes timescale")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
