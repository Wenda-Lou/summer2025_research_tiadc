"""Scratch: redraw a saved run's learning curves from qualified samples only.

Runs saved before the driver counted qualified samples were plotted from every row in
the log, so their PNGs show the acceptance filter's rejects as if they were loop
behaviour.  The CSV still carries the `rejected` column, so the clean figure can be
rebuilt offline -- no bench time, no re-run.

``--min-margin`` additionally drops qualified frames whose dither alignment margin is
below it.  Those frames are fine for the *loop* -- its observables are window and phase
integrals, and their scatter is flat against margin -- but they wreck the *spectral*
score.  In run3, all 16 frames more than 3 dB below the median SNDR sit at margin
6.08-6.8, and the SNDR 5th percentile jumps from 30.1 to 36.6 dB when the floor moves
from 6.0 to 6.5 (mean moves only 0.25 dB).  That is a scoring filter, not a loop-gate
change: raising the loop's own `min_align_margin` would cost ~40 % more captures for no
benefit to the loop.

    python calibration_out/_replot_qualified.py calibration_out/closed_loop/run1.csv
    python calibration_out/_replot_qualified.py --min-margin 6.5 <run>.csv
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from pathlib import Path

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.dither import DitherConfig                 # noqa: E402
from calibration_loop.loop import CalibrationLoop                # noqa: E402
from calibration_loop.simulate import BenchModel                 # noqa: E402

# every key the four panels read, so a partial log (e.g. one recovered from a
# console log, which has only the printed columns) still plots what it has
PLOT_KEYS = ("iteration", "gain_ratio", "offset_a_codes", "offset_b_codes",
             "skew_mismatch_ps", "cal_sndr_db", "cal_sfdr_db", "raw_sndr_db")


def replot(path: str, min_margin: float | None = None, out_dir: str | None = None) -> int:
    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
    qualified = [r for r in rows if not r.get("rejected")]
    n_qualified = len(qualified)
    if min_margin is not None:
        qualified = [r for r in qualified
                     if float(r.get("align_margin") or 0.0) >= min_margin]
    if not qualified:
        print(f"{path}: no rows left after filtering")
        return 1

    for r in qualified:
        for key in PLOT_KEYS:
            try:
                r[key] = float(r[key])
            except (KeyError, TypeError, ValueError):
                r[key] = float("nan")

    cfg = DitherConfig()
    cfg.validate()
    # A loop object is only needed as a carrier for the log and the plot code, so it
    # is built over the bench model: no hardware, no writes.
    loop = CalibrationLoop(BenchModel(cfg=cfg), cfg)
    loop.log = qualified

    out = Path(out_dir) if out_dir else Path(path).parent
    stem = Path(path).stem + ("_scored" if min_margin is not None else "_from_csv")
    png = loop.plot(out, stem=stem)

    note = (f"; align margin >= {min_margin:g} kept {len(qualified)} of {n_qualified} "
            f"qualified" if min_margin is not None else "")
    print(f"{path}: {len(qualified)} drawn of {len(rows)} rows "
          f"({len(rows) - n_qualified} rejected excluded{note}) -> {Path(str(png)).name}")

    if min_margin is not None:
        s = np.array([r["cal_sndr_db"] for r in qualified], dtype=float)
        sf = np.array([r["cal_sfdr_db"] for r in qualified], dtype=float)
        print(f"    scored subset: SNDR {np.nanmean(s):.2f} +- {np.nanstd(s, ddof=1):.2f} dB"
              f" (p5 {np.nanpercentile(s, 5):.2f}), SFDR {np.nanmean(sf):.2f} +- "
              f"{np.nanstd(sf, ddof=1):.2f} dB")

    missing = [k for k in PLOT_KEYS if all(not np.isfinite(r[k]) for r in qualified)]
    if missing:
        print(f"    panels missing data (left blank): {', '.join(missing)}")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("paths", nargs="+")
    ap.add_argument("--min-margin", dest="min_margin", type=float, default=None,
                    help="drop qualified frames whose dither alignment margin is below "
                         "this before plotting (6.5 removes every SNDR outlier in run3 "
                         "while keeping 184 of 300 frames)")
    ap.add_argument("--out", dest="out_dir", default=None,
                    help="directory for the PNG (default: next to the CSV)")
    args = ap.parse_args()
    sys.exit(max(replot(p, args.min_margin, args.out_dir) for p in args.paths))
