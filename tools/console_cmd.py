"""Send one line to the board console and print whatever comes back.

Read-only: this only types a command, it never pokes a register.  Used to ask the
board about its own state when the host-side capture looks wrong -- notably after
an AD9695 restart, when the firmware's link/bring-up state and the chip's register
state can disagree with what the host assumes.

Usage:
    python tools/console_cmd.py "help"
    python tools/console_cmd.py "adc -cal status" --wait 3
    python tools/console_cmd.py "help" "adc -cal status"     # several, in order
"""

from __future__ import annotations

import argparse
import os
import sys
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("commands", nargs="+", help="console lines to send, in order")
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--wait", type=float, default=2.0,
                    help="seconds to collect the reply to each command")
    args = ap.parse_args(argv)

    from calibration_loop.capture import UartConsole

    console = UartConsole(args.uart)
    try:
        time.sleep(0.2)
        console.read_all()  # drop anything left over from an earlier session
        for cmd in args.commands:
            print(f"\n>>> {cmd}")
            console.send(cmd, settle=0.0)
            deadline = time.time() + args.wait
            seen = ""
            while time.time() < deadline:
                chunk = console.read_all()
                if chunk:
                    seen += chunk
                    deadline = time.time() + 0.4   # keep reading while it talks
                time.sleep(0.05)
            text = seen.replace("\r", "")
            print(text.rstrip() if text.strip() else "(no output)")
    finally:
        console.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
