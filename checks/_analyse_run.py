"""Scratch: summarize a closed-loop bench run (run1 / run2_nocancel)."""

from __future__ import annotations

import csv
import sys
from collections import Counter

import numpy as np


def load(path):
    with open(path, newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def fnum(row, key, default=np.nan):
    try:
        return float(row[key])
    except (KeyError, TypeError, ValueError):
        return default


def block(rows, label):
    n = len(rows)
    gain = np.array([fnum(r, "gain_ratio") for r in rows])
    off = np.array([fnum(r, "offset_b_codes") - fnum(r, "offset_a_codes") for r in rows])
    skew = np.array([fnum(r, "skew_mismatch_ps") for r in rows])
    raw = np.array([fnum(r, "raw_difference_dbc") for r in rows])
    cal = np.array([fnum(r, "cal_difference_dbc") for r in rows])
    head, tail = slice(0, max(1, n // 10)), slice(n - max(1, n // 10), n)
    print(f"\n== {label}  ({n} iterations) ==")
    print(f"{'':22}{'first 10%':>12}{'last 10%':>12}{'all mean':>12}{'all sd':>10}")
    for name, arr in (("|gain-1| [1e-5]", np.abs(gain - 1.0) * 1e5),
                      ("|offset mismatch| LSB", np.abs(off)),
                      ("|skew| ps", np.abs(skew)),
                      ("raw A-B spur dBc", raw),
                      ("cal A-B spur dBc", cal)):
        print(f"{name:22}{np.nanmean(arr[head]):>12.4f}{np.nanmean(arr[tail]):>12.4f}"
              f"{np.nanmean(arr):>12.4f}{np.nanstd(arr):>10.4f}")
    print(f"  skew mismatch: first {skew[0]:+.2f} ps -> last {skew[-1]:+.2f} ps "
          f"(min {np.nanmin(skew):+.1f}, max {np.nanmax(skew):+.1f})")
    print(f"  A-B spur     : first {raw[0]:.2f} dBc -> last {cal[-1]:.2f} dBc "
          f"(best {np.nanmin(cal):.1f})")
    print(f"  rejected rows: {sum(1 for r in rows if r.get('rejected'))}")
    actions = Counter(r.get("skew_action", "") for r in rows if r.get("skew_action"))
    print(f"  skew_action  : {dict(actions)}")
    codes = sorted({r.get("skew_code", "") for r in rows if r.get("skew_code")})
    print(f"  skew codes seen in ACKs: {codes}")
    errs = [r.get("skew_error", "") for r in rows if r.get("skew_error")]
    print(f"  skew_error rows: {len(errs)} {errs[:2]}")
    batches = [r for r in rows if r.get("skew_batch_mean_ps")]
    if batches:
        print(f"  skew decisions: {len(batches)}")
        for r in batches:
            print(f"    it {r['iteration']:>4}  n={r.get('skew_batch_used')}"
                  f"  mean {fnum(r, 'skew_batch_mean_ps'):+7.2f} ps"
                  f"  se {fnum(r, 'skew_batch_se_ps'):5.2f}"
                  f"  err {fnum(r, 'skew_batch_error_ps'):+7.2f} ps"
                  f"  {r.get('skew_action')}")
    mg = np.array([fnum(r, "align_margin") for r in rows])
    print(f"  align margin: min {np.nanmin(mg):.1f}  mean {np.nanmean(mg):.1f}")
    if "frame_sha1" in rows[0]:
        shas = {r.get("frame_sha1") for r in rows}
        print(f"  distinct frames: {len(shas)}/{len(rows)}"
              f"{'  <-- REPEATED FRAMES (frozen buffer!)' if len(shas) < len(rows) else ''}")
    else:
        print("  distinct frames: n/a (run predates the frame_sha1 column)")
    rows_ = [r for r in rows if fnum(r, "cal_sndr_db") == fnum(r, "cal_sndr_db")]
    if rows_:
        print(f"  SNDR cal: mean {np.nanmean([fnum(r, 'cal_sndr_db') for r in rows_]):.2f} dB")


if __name__ == "__main__":
    for path in sys.argv[1:]:
        block(load(path), path)
