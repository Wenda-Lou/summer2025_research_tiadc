"""Find which CP210x interface is the ZCU102 firmware console.

Sends a bare CR and a `help` to each candidate port and reports which one talks
back.  Read-only with respect to the board: `help` changes no state.

Usage:
    python tools/find_console.py
    python tools/find_console.py --ports COM4,COM5,COM6,COM7
"""

from __future__ import annotations

import argparse
import sys
import time

BAUD = 115200


def probe(port: str, baud: int) -> tuple[bool, str]:
    import serial
    try:
        ser = serial.Serial(port, baud, timeout=0.6)
    except Exception as exc:  # noqa: BLE001
        return False, f"OPEN FAILED: {exc}"

    chunks = []
    try:
        time.sleep(0.3)
        ser.reset_input_buffer()
        ser.write(b"\r")
        ser.flush()
        time.sleep(1.0)
        chunks.append(ser.read(8192))

        ser.write(b"help\r")
        ser.flush()
        time.sleep(2.0)
        chunks.append(ser.read(32768))
    except Exception as exc:  # noqa: BLE001
        return False, f"IO ERROR: {exc}"
    finally:
        ser.close()

    text = b"".join(chunks).decode(errors="replace")
    printable = "".join(c if c.isprintable() or c in "\r\n\t" else "." for c in text)
    return len(text) > 0, printable


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ports", default="COM4,COM5,COM6,COM7,COM3,COM8")
    ap.add_argument("--baud", type=int, default=BAUD)
    args = ap.parse_args(argv)

    ports = [p.strip() for p in args.ports.split(",") if p.strip()]
    winners = []
    for p in ports:
        ok, text = probe(p, args.baud)
        if ok:
            head = text.strip()[:400].replace("\r", "")
            print(f"[{p}] RESPONDED ({len(text)} chars)")
            for line in head.splitlines()[:12]:
                print(f"      {line}")
            winners.append(p)
        else:
            print(f"[{p}] silent ({text})")

    print()
    if not winners:
        print("NO PORT RESPONDED.")
        print("Board-side checks: SD boot mode, power, and that the ELF was "
              "actually programmed; press the POR button and retry.")
        return 1
    print(f"Console port(s): {', '.join(winners)}")
    if len(winners) > 1:
        print("More than one replied - pick the one printing the adc/dma menu.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
