"""Characterise a serial port: echo test vs a real board console.

Sends a distinctive marker.  If the port returns exactly that marker and
nothing else, it is a loopback / local echo, not a command console.  A real
firmware console answers with a help menu or a prompt.

    python tools/classify_port.py --port COM7
    python tools/classify_port.py --port COM5          # console candidate
"""

from __future__ import annotations

import argparse
import sys
import time

MARKER = "zzq-marker-9182736"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--wait", type=float, default=2.5)
    args = ap.parse_args(argv)

    import serial

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.5)
    except Exception as exc:  # noqa: BLE001
        print(f"{args.port}: OPEN FAILED: {exc}")
        return 2

    try:
        time.sleep(0.3)
        ser.reset_input_buffer()
        ser.write((MARKER + "\r").encode())
        ser.flush()

        end = time.time() + args.wait
        buf = []
        while time.time() < end:
            chunk = ser.read(4096)
            if chunk:
                buf.append(chunk)
        text = b"".join(buf).decode(errors="replace")
    finally:
        ser.close()

    print(f"{args.port}: {len(text)} chars back")
    print(f"  raw: {text!r}")

    stripped = text.strip()
    if stripped == MARKER:
        print("  VERDICT: local echo / loopback - NOT a command console")
        return 1
    if MARKER in text and len(stripped) > len(MARKER) + 4:
        print("  VERDICT: echo plus extra traffic - inspect raw above")
        return 0
    if not stripped:
        print("  VERDICT: silent - not the firmware console")
        return 1
    print("  VERDICT: replied with something other than our marker - "
          "likely a real console")
    return 0


if __name__ == "__main__":
    sys.exit(main())
