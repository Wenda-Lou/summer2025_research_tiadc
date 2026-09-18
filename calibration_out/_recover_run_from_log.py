"""Scratch: rebuild a learning curve from a run's console log.

The console log survives things the CSV does not: the CSV/JSON/plot are written only
when the run finishes, and a later run reusing the same `--stem` overwrites them
(that happened on 2026-09-17, when a 100-iteration run replaced the 300-capture
`run1.csv`).  The log still holds one line per accepted iteration, so the curve can
be recovered.

Recovered columns are exactly what the loop prints -- iteration, gain ratio, offset
mismatch, skew mismatch, calibrated SNDR and the corrected image spur.  Everything
else in the original CSV (raw spur, per-row batch statistics, frame hashes) is gone.

    python calibration_out/_recover_run_from_log.py calibration_out/closed_loop_run1.txt \
        --out calibration_out/closed_loop/run1_300_from_log.csv
"""

from __future__ import annotations

import argparse
import csv
import re
import sys

LINE = re.compile(
    r"^\s*it\s+(?P<it>\d+)\s+"
    r"g_B/g_A=(?P<gain>[+-][\d.]+)\s+"
    r"dOffset=\s*(?P<offset>[+-][\d.]+)\s*LSB\s+"
    r"dSkew=\s*(?P<skew>[+-][\d.]+)\s*ps\s+"
    r"SNDR=(?P<sndr>[\d.]+)\s*dB\s+"
    r"image=\s*(?P<image>[+-][\d.]+)\s*dBc"
)
REJECT = re.compile(r"^\s*it\s+(?P<it>\d+)\s+rejected:\s*(?P<why>.+?)\s*$")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("log")
    ap.add_argument("--out", required=True)
    args = ap.parse_args(argv)

    rows, rejections, header = [], 0, False
    # PowerShell's `>` and `Tee-Object` write UTF-16LE with a BOM on Windows
    # (PowerShell 5.1), so a redirect is not the plain text it looks like in the
    # terminal.  Sniff the BOM instead of assuming.
    with open(args.log, "rb") as fh:
        head = fh.read(4)
    if head[:2] in (b"\xff\xfe", b"\xfe\xff"):
        encoding = "utf-16"
    elif head[:3] == b"\xef\xbb\xbf":
        encoding = "utf-8-sig"
    else:
        encoding = "utf-8"
    with open(args.log, encoding=encoding, errors="replace") as fh:
        for line in fh:
            if "Running hardware loop" in line or "Running simulation" in line:
                header = True
                continue
            m = LINE.match(line)
            if m:
                rows.append({
                    "iteration": int(m["it"]),
                    "gain_ratio": float(m["gain"]),
                    "offset_mismatch_lsb": float(m["offset"]),
                    "skew_mismatch_ps": float(m["skew"]),
                    "cal_sndr_db": float(m["sndr"]),
                    "cal_image_spur_dbc": float(m["image"]),
                })
                continue
            if REJECT.match(line):
                rejections += 1

    if not rows:
        print(f"no iteration lines found in {args.log}")
        return 1
    if not header:
        print("note: no 'Running ... for N iterations' line -- is this a run log?")

    iterations = [r["iteration"] for r in rows]
    print(f"recovered {len(rows)} accepted iterations, {rejections} rejection lines")
    print(f"iteration counter {iterations[0]}..{iterations[-1]} "
          f"(gaps mean rejected captures)")

    with open(args.out, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
