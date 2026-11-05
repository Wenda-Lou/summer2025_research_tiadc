"""Scratch: why does the loop's corrected A-B spur sit below its raw one?

The read-only check measures the A-B difference spur on *prepared raw* frames and
gets -38.6 dBc at the converged point.  The loop logs two versions of the same
quantity: ``raw_difference_dbc`` (raw records, dither subtracted) and
``cal_difference_dbc`` (host-corrected records, dither subtracted).  On run1 they
disagree by ~4.5 dB after convergence, so one of them is not measuring skew.

Also compares both against the analytic prediction 20log10|2 sin(pi f dt)|, which
is the number that decides whether the residual is skew-limited at all.
"""

from __future__ import annotations

import csv
import math
import sys

import numpy as np

FS_ADC = 1300e6
F0 = 1276 / 8320 * FS_ADC          # sig_cycles / n_adc_period, from run1_meta.json


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


def spur(dt_ps):
    return 20.0 * math.log10(abs(2.0 * math.sin(math.pi * F0 * dt_ps * 1e-12)) + 1e-15)


def main(path):
    rows = load(path)
    acc = [r for r in rows if not r.get("rejected")]
    print(f"{path}: {len(rows)} captures, {len(acc)} accepted")
    print("columns:", ", ".join(rows[0].keys()))
    print(f"f0 = {F0 / 1e6:.3f} MHz")

    # convergence = first batch decision that found itself inside the deadband
    conv = next(int(r["iteration"]) for r in acc
                if r.get("skew_action") == "inside-deadband")
    pre = [r for r in acc if int(r["iteration"]) < conv]
    post = [r for r in acc if int(r["iteration"]) >= conv]
    print(f"converged at iteration {conv}: {len(pre)} accepted before, "
          f"{len(post)} after\n")

    for label, seg in (("pre-convergence", pre), ("post-convergence", post)):
        raw = num(seg, "raw_difference_dbc")
        cal = num(seg, "cal_difference_dbc")
        skew = num(seg, "skew_mismatch_ps")
        phase = num(seg, "skew_phase_ps")
        diff = num(seg, "skew_diff_route_ps")
        pred = np.array([spur(s) for s in skew])
        pred_phase = np.array([spur(s) for s in phase])
        print(f"== {label} ({len(seg)} rows) ==")
        print(f"  skew mismatch   {np.nanmean(skew):+7.2f} +- {np.nanstd(skew):5.2f} ps"
              f"   (phase {np.nanmean(phase):+7.2f}, diff-route "
              f"{np.nanmean(diff):+7.2f})")
        print(f"  raw A-B spur    {np.nanmean(raw):7.2f} +- {np.nanstd(raw):5.2f} dBc")
        print(f"  cal A-B spur    {np.nanmean(cal):7.2f} +- {np.nanstd(cal):5.2f} dBc"
              f"   (cal - raw = {np.nanmean(cal) - np.nanmean(raw):+.2f} dB)")
        print(f"  predicted       {np.nanmean(pred):7.2f} +- {np.nanstd(pred):5.2f} dBc"
              f"   (raw - pred = {np.nanmean(raw) - np.nanmean(pred):+.2f} dB,"
              f" cal - pred = {np.nanmean(cal) - np.nanmean(pred):+.2f} dB)")
        print(f"  rows with raw within 1 dB of prediction: "
              f"{int(np.sum(np.abs(raw - pred) < 1.0))}/{len(seg)}")
        print(f"  raw/cal correlation with |g_meas-1|: "
              f"{np.corrcoef(np.abs(num(seg, 'gain_ratio') - 1), raw)[0, 1]:+.3f} / "
              f"{np.corrcoef(np.abs(num(seg, 'gain_ratio') - 1), cal)[0, 1]:+.3f}")
        print(f"  raw/cal correlation with |offset mismatch|: "
              f"{np.corrcoef(np.abs(num(seg, 'offset_b_codes') - num(seg, 'offset_a_codes')), raw)[0, 1]:+.3f} / "
              f"{np.corrcoef(np.abs(num(seg, 'offset_b_codes') - num(seg, 'offset_a_codes')), cal)[0, 1]:+.3f}")
        print(f"  state gain_corr_a {np.nanmean(num(seg, 'gain_corr_a')):.5f}"
              f" +- {np.nanstd(num(seg, 'gain_corr_a')):.5f}"
              f"   gain_corr_b {np.nanmean(num(seg, 'gain_corr_b')):.5f}"
              f" +- {np.nanstd(num(seg, 'gain_corr_b')):.5f}")
        print(f"  measured g_B/g_A wobble |g-1| mean "
              f"{np.nanmean(np.abs(num(seg, 'gain_ratio') - 1)):.2e}"
              f"  -> -20log10 = {20 * math.log10(max(np.nanmean(np.abs(num(seg, 'gain_ratio') - 1)), 1e-12)):.1f} dBc if that were the whole difference")
        print(f"  cal SNDR {np.nanmean(num(seg, 'cal_a_sndr_db')):.2f} dB   "
              f"raw SNDR {np.nanmean(num(seg, 'raw_a_sndr_db')):.2f} dB")
        print()


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "calibration_out/closed_loop/run1.csv")
