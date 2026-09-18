"""Scratch: redraw a saved run's learning curves from its qualified samples only.

Runs saved before the driver counted qualified samples were plotted from every row in
the log, so their PNGs show the acceptance filter's rejects as if they were loop
behaviour.  The CSV still carries the `rejected` column, so the clean figure can be
rebuilt offline -- no bench time, no re-run.

Written to `<stem>_learning_qualified.png` so the original stays put for comparison.

    python calibration_out/_replot_qualified.py calibration_out/closed_loop/run1.csv
"""

from __future__ import annotations

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


def replot(path: str) -> int:
    rows = list(csv.DictReader(open(path, newline="", encoding="utf-8")))
    qualified = [r for r in rows if not r.get("rejected")]
    for r in qualified:
        for key in PLOT_KEYS:
            try:
                r[key] = float(r[key])
            except (KeyError, TypeError, ValueError):
                r[key] = float("nan")
    if not qualified:
        print(f"{path}: no qualified rows")
        return 1

    cfg = DitherConfig()
    cfg.validate()
    # A loop object is only needed as a carrier for the log and the plot code, so it
    # is built over the bench model: no hardware, no writes.
    loop = CalibrationLoop(BenchModel(cfg=cfg), cfg)
    loop.log = rows

    out = Path(path).parent
    stem = Path(path).stem + "_from_csv"
    png = loop.plot(out, stem=stem)
    missing = [k for k in PLOT_KEYS
               if all(not np.isfinite(r[k]) for r in qualified)]
    print(f"{path}: {len(qualified)} qualified of {len(rows)} rows "
          f"({len(rows) - len(qualified)} rejected excluded) -> {Path(str(png)).name}")
    if missing:
        print(f"    panels missing data (left blank): {', '.join(missing)}")
    return 0


if __name__ == "__main__":
    sys.exit(max(replot(p) for p in sys.argv[1:]))
