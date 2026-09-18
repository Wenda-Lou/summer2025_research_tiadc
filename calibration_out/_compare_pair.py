"""Scratch: matched-pair comparison of two closed-loop bench runs.

Both runs must start from the same actuator state (code 24, ~-44 ps) — that is what
``tools/skew_park.py`` is for.  With cancellation on (run1) versus off
(run2_nocancel) the *digital* gain/offset loop is the thing under test: the skew
actuator moves at most one code per 20-frame batch either way, so its trajectory is
set by the batch size, while the gain and offset estimates are what the removed
main tone was protecting.

    python calibration_out/_compare_pair.py run1.csv run2_nocancel.csv
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


def first_below(x, thresh):
    idx = np.where(np.abs(x) < thresh)[0]
    return int(idx[0]) if idx.size else None


def summarize(path, label):
    rows = load(path)
    acc = [r for r in rows if not r.get("rejected")]
    gain = np.abs(num(acc, "gain_ratio") - 1.0)
    off = np.abs(num(acc, "offset_b_codes") - num(acc, "offset_a_codes"))
    skew = np.abs(num(acc, "skew_mismatch_ps"))
    spur = num(acc, "raw_difference_dbc")
    n = len(acc)
    tail = slice(n - max(1, n // 10), n)
    dec = [r for r in acc if r.get("skew_action")]
    moves = [r for r in dec if str(r.get("skew_action", "")).startswith("move")]
    conv = next((i for i, r in enumerate(dec)
                 if r.get("skew_action") == "inside-deadband"), None)
    return {
        "label": label, "path": path, "captures": len(rows), "accepted": n,
        "rejected": len(rows) - n,
        "gain_first": float(gain[:max(1, n // 10)].mean()),
        "gain_tail": float(gain[tail].mean()),
        "off_tail": float(off[tail].mean()),
        "skew_first": float(np.abs(num(acc, "skew_mismatch_ps"))[:1].mean()),
        "skew_tail": float(skew[tail].mean()),
        "spur_first": float(spur[:max(1, n // 10)].mean()),
        "spur_tail": float(spur[tail].mean()),
        "moves": len(moves), "decisions": len(dec),
        "conv_decision": conv,
        "conv_iteration": int(dec[conv]["iteration"]) if conv is not None else None,
        "final_code": (moves[-1].get("skew_code") if moves else None),
        "gain_iter_to_1pct": first_below(gain, 0.01),
        "off_iter_to_1lsb": first_below(off, 1.0),
        "abort": None,
    }


def main(paths):
    runs = [summarize(p, p.split("/")[-1]) for p in paths]
    w = 30
    print(f"{'':>{w}}" + "".join(f"{r['label']:>26}" for r in runs))
    rows = [
        ("captures / accepted / rejected",
         lambda r: f"{r['captures']}/{r['accepted']}/{r['rejected']}"),
        ("|gain-1| first 10% [1e-5]", lambda r: f"{r['gain_first'] * 1e5:.0f}"),
        ("|gain-1| last 10% [1e-5]", lambda r: f"{r['gain_tail'] * 1e5:.0f}"),
        ("iters to |gain-1| < 0.01", lambda r: f"{r['gain_iter_to_1pct']}"),
        ("|offset mismatch| last 10% LSB", lambda r: f"{r['off_tail']:.2f}"),
        ("iters to |offset| < 1 LSB", lambda r: f"{r['off_iter_to_1lsb']}"),
        ("|skew| first capture ps", lambda r: f"{r['skew_first']:.1f}"),
        ("|skew| last 10% ps", lambda r: f"{r['skew_tail']:.1f}"),
        ("skew decisions / moves", lambda r: f"{r['decisions']}/{r['moves']}"),
        ("first inside-deadband at iter",
         lambda r: f"{r['conv_iteration']}"),
        ("final control code", lambda r: f"{r['final_code']}"),
        ("raw A-B spur first 10% dBc", lambda r: f"{r['spur_first']:.2f}"),
        ("raw A-B spur last 10% dBc", lambda r: f"{r['spur_tail']:.2f}"),
    ]
    for name, fn in rows:
        print(f"{name:>{w}}" + "".join(f"{fn(r):>26}" for r in runs))


if __name__ == "__main__":
    main(sys.argv[1:] or ["calibration_out/closed_loop/run1.csv",
                          "calibration_out/closed_loop/run2_nocancel.csv"])
