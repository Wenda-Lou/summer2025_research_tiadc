"""Scratch: what limits the loop's *corrected* A-B spur on run1?

Post-convergence run1 measures -38.62 dBc on the raw channel difference but only
-35.07 dBc on the corrected one, while the skew prediction for that state is
-38.84 dBc.  So the correction is adding ~3.5 dB of difference spur.  Two
candidates, both computable from the logged state:

  * the applied gain correction's own ratio error, 1 - gain_corr_b/gain_corr_a,
    which puts (1 - ratio) * A_tone straight into cal_a - cal_b at f0;
  * the per-frame gain estimate wobble, |g_B/g_A - 1|, which is what the state
    chases and which a session-averaged correction would not carry.

Under the loop's own convention cal_x = gain_corr_x * (raw_x - offset_x), so the
only channel-dependent scaling in the corrected difference is that ratio; the
offset only moves DC (checked against cal_dc_difference_codes).
"""

from __future__ import annotations

import csv
import math
import sys

import numpy as np

FS_ADC = 1300e6
F0 = 1276 / 8320 * FS_ADC


def load(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return [r for r in csv.DictReader(fh)]


def num(rows, key):
    out = []
    for r in rows:
        try:
            out.append(float(r[key]))
        except (KeyError, TypeError, ValueError):
            out.append(np.nan)
    return np.array(out)


def db(x):
    x = np.abs(np.asarray(x, dtype=float))
    return 20.0 * np.log10(np.maximum(x, 1e-15))


def main(path):
    rows = [r for r in load(path) if not r.get("rejected")]
    conv = next(int(r["iteration"]) for r in rows
                if r.get("skew_action") == "inside-deadband")
    post = [r for r in rows if int(r["iteration"]) >= conv]
    n = len(post)
    print(f"post-convergence window: {n} accepted captures "
          f"(iterations {conv}..{post[-1]['iteration']})\n")

    g = num(post, "gain_ratio")
    skew = num(post, "skew_mismatch_ps")
    raw = num(post, "raw_difference_dbc")
    cal = num(post, "cal_difference_dbc")
    dc = num(post, "cal_dc_difference_codes")
    r_state = num(post, "gain_corr_b") / num(post, "gain_corr_a")

    pred = 20 * np.log10(np.abs(2 * np.sin(np.pi * F0 * skew * 1e-12)) + 1e-15)

    print(f"residual dither gain ratio g_B/g_A: mean {np.nanmean(g):+.5f} "
          f"sd {np.nanstd(g):.5f}   (mean-1 = {np.nanmean(g) - 1:+.2e}, "
          f"t = {(np.nanmean(g) - 1) / (np.nanstd(g) / math.sqrt(n)):+.2f})")
    print(f"applied state ratio g_corr_b/g_corr_a: mean {np.nanmean(r_state):.5f} "
          f"sd {np.nanstd(r_state):.5f}"
          f"   -> 1-ratio = {1 - np.nanmean(r_state):+.2e} = "
          f"{db(1 - np.nanmean(r_state)):.2f} dBc")
    print(f"per-frame wobble |g_B/g_A - 1|: mean {np.nanmean(np.abs(g - 1)):.3e} = "
          f"{db(np.nanmean(np.abs(g - 1))):.2f} dBc")
    print(f"cal DC difference: mean {np.nanmean(dc):+.3f} codes "
          f"sd {np.nanstd(dc):.3f} (only DC -- cannot explain an f0 spur)\n")

    print(f"measured: raw {np.nanmean(raw):.2f} dBc   cal {np.nanmean(cal):.2f} dBc"
          f"   skew prediction {np.nanmean(pred):.2f} dBc")
    print(f"           cal - raw {np.nanmean(cal) - np.nanmean(raw):+.2f} dB,"
          f" cal - pred {np.nanmean(cal) - np.nanmean(pred):+.2f} dB")

    # do the two candidate terms power-combine to what is measured?
    p_skew = 10 ** (np.nanmean(pred) / 10)
    p_state = 10 ** (db(1 - np.nanmean(r_state)) / 10)
    p_sum = 10 * math.log10(p_skew + p_state)
    print(f"\npower sum of (skew term {np.nanmean(pred):.2f} dBc + applied-ratio "
          f"term {db(1 - np.nanmean(r_state)):.2f} dBc) = {p_sum:.2f} dBc"
          f"   vs measured cal {np.nanmean(cal):.2f} dBc"
          f"   -> {p_sum - np.nanmean(cal):+.2f} dB")

    # per-frame: does cal track the prediction plus the ratio term?
    model = 10 * np.log10(10 ** (pred / 10) + 10 ** (db(1 - r_state) / 10))
    print(f"per-frame model vs cal: mean gap {np.nanmean(model - cal):+.2f} dB, "
          f"sd of gap {np.nanstd(model - cal):.2f} dB, "
          f"corr {np.corrcoef(model, cal)[0, 1]:+.3f}")
    print(f"per-frame raw vs pred : mean gap {np.nanmean(raw - pred):+.2f} dB, "
          f"sd of gap {np.nanstd(raw - pred):.2f} dB")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "calibration_out/closed_loop/run1.csv")
