"""Scratch: capture raw frames to disk for offline analysis (read-only on the bench).

Only ``HardwareBench.capture()`` is used -- the same read-only path as ``probe``;
no register is written.  Each frame is stored verbatim so every later question can
be answered offline without touching the bench again.

Usage:
    python calibration_out/_capture_frames.py --uart COM5 --frames 20
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--out", default=os.path.join(REPO, "calibration_out", "frames"))
    args = ap.parse_args(argv)

    from calibration_loop.capture import HardwareBench

    os.makedirs(args.out, exist_ok=True)
    bench = HardwareBench(uart_port=args.uart)
    index = []
    try:
        for i in range(args.frames):
            raw = bench.capture()
            if raw is None:
                print(f"frame {i:>3}: capture FAILED")
                continue
            name = f"frame_{i:02d}.bin"
            with open(os.path.join(args.out, name), "wb") as fh:
                fh.write(raw)
            digest = hashlib.sha1(raw).hexdigest()[:12]
            index.append({"i": i, "file": name, "bytes": len(raw), "sha1_12": digest})
            print(f"frame {i:>3}: {len(raw)} bytes  sha1={digest}")
    finally:
        bench.close()

    with open(os.path.join(args.out, "index.json"), "w", encoding="utf-8") as fh:
        json.dump(index, fh, indent=2)
    print(f"\nsaved {len(index)} frames to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
