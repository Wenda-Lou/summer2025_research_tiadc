"""Scratch: one figure for the professor update.

Left: the corrected A-B spur before and after the gain-observable change, on the two
runs that are comparable — both cancellation on, both parked at code 24 first, both
converging to the same skew residual (-9.70 vs -9.72 ps), so the only thing that
changed is which observable the gain loop integrates.

Right: run3's gain observables through the run.  The tone route is the controlled one
and goes to 1.0000; the dither route is no longer nulled and parks at ~1.012, which is
the design cost of the choice, shown rather than hidden.

    python calibration_out/_professor_figure.py
"""

from __future__ import annotations

import csv
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

import matplotlib                                        # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt                          # noqa: E402

OLD = os.path.join(REPO, "calibration_out", "closed_loop", "run1_300_from_log.csv")
NEW = os.path.join(REPO, "calibration_out", "closed_loop", "run3_gainfix.csv")
OUT = os.path.join(REPO, "calibration_out", "professor_update")

F0 = 1276 / 8320 * 1300e6


def load(path):
    return [r for r in csv.DictReader(open(path, newline="", encoding="utf-8"))
            if not r.get("rejected")]


def col(rows, key):
    out = []
    for r in rows:
        try:
            out.append(float(r[key]))
        except (KeyError, TypeError, ValueError):
            out.append(np.nan)
    return np.array(out, dtype=float)


def post(rows, conv=159):
    return [r for r in rows if int(float(r["iteration"])) >= conv]


def main() -> int:
    old, new = post(load(OLD)), post(load(NEW))
    # the recovered 300-sample curve carries cal_image_spur_dbc, which in the
    # non-interleaved branch of loop._measure *is* cal_difference_dbc
    old_cal = col(old, "cal_image_spur_dbc")
    new_cal = col(new, "cal_difference_dbc")
    new_raw = col(new, "raw_difference_dbc")

    fig, ax = plt.subplots(1, 2, figsize=(12.5, 4.6))

    bars = ax[0].bar(["impulse route\n(old default)", "tone route\n(current default)"],
                     [old_cal.mean(), new_cal.mean()],
                     yerr=[old_cal.std(ddof=1), new_cal.std(ddof=1)],
                     color=["#b0b0b0", "#3060a0"], capsize=6, width=0.55)
    ax[0].axhline(new_raw.mean(), ls="--", lw=1.2, color="k",
                  label=f"skew limit (raw, {new_raw.mean():.1f} dBc)")
    ax[0].axhline(20 * math.log10(abs(2 * math.sin(math.pi * F0 * 9.72e-12))),
                  ls=":", lw=1.2, color="r",
                  label="analytic $20\\log_{10}|2\\sin(\\pi f\\Delta t)|$")
    ax[0].set_ylabel("corrected A-B difference spur [dBc]")
    ax[0].set_title("Gain observable decides the corrected metric\n"
                    "(both runs: cancellation on, parked at code 24, "
                    "residual skew -9.70 vs -9.72 ps)", fontsize=10)
    ax[0].legend(fontsize=8, loc="lower right")
    ax[0].grid(True, axis="y", alpha=0.3)
    for b, v in zip(bars, [old_cal.mean(), new_cal.mean()]):
        ax[0].annotate(f"{v:.1f}", (b.get_x() + b.get_width() / 2, v),
                       textcoords="offset points", xytext=(0, -16),
                       ha="center", fontsize=9)

    it = col(new, "iteration")
    ax[1].plot(it, col(new, "tone_ratio"), lw=1.2, label="tone ratio (controlled)")
    ax[1].plot(it, col(new, "gain_ratio"), ls=":", lw=1.0,
               label="impulse ratio (no longer nulled)")
    ax[1].axhline(1.0, ls="--", lw=1, color="k")
    ax[1].set_ylim(0.99, 1.03)
    ax[1].set_xlabel("qualified sample")
    ax[1].set_ylabel("gain ratio $g_B/g_A$")
    ax[1].set_title("Run 3: the controlled observable reaches 1.0000\n"
                    "while the impulse route parks at ~1.012 by design", fontsize=10)
    ax[1].legend(fontsize=8, loc="upper right")
    ax[1].grid(True, alpha=0.3)

    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, "loop_summary.png")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    print(f"wrote {path}")
    print(f"  old observable: corrected {old_cal.mean():.2f} +- {old_cal.std(ddof=1):.2f} "
          f"dBc over {len(old)} samples")
    print(f"  new observable: corrected {new_cal.mean():.2f} +- {new_cal.std(ddof=1):.2f} "
          f"dBc, raw {new_raw.mean():.2f} +- {new_raw.std(ddof=1):.2f} over {len(new)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
