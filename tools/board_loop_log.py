"""Run one ``adc -cal loop`` command on the board and save the console output.

The on-board loop (firmware/thesis_v3_loop_appl) prints one ``LOOP`` line per
capture; a 300-sample run is too long to copy out of a terminal.  This sends the
command over the UART, echoes every line, writes them to a log file and stops at
``LOOP-STATUS`` (printed right after ``LOOP-END``).  The terminal on that port must
be disconnected first.

    python tools/board_loop_log.py --uart COM7 --cmd "adc -cal loop open 20"
    python tools/board_loop_log.py --uart COM7 --cmd "adc -cal loop 300" \
        --out calibration_out/closed_loop/run7_board_w06.log
    python tools/board_loop_log.py --uart COM7 --wave 1 --cmd "adc -cal loop 300"

``--wave K`` first sends ``adc -cal loop wave K`` (the row of loop_waveform.h the
DPG is playing) and checks the firmware's LOOP-WAVE answer.  A ``Command Error``
line from the firmware ends the log at once instead of waiting for the timeout.
"""

from __future__ import annotations

import argparse
import os
import sys
import time


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uart", default="COM7")
    ap.add_argument("--cmd", required=True, help='e.g. "adc -cal loop 300"')
    ap.add_argument("--wave", type=int, default=None,
                    help="select this loop_waveform.h row (adc -cal loop wave K) before --cmd")
    ap.add_argument("--out", default=None, help="log file (default: calibration_out/board_loop_<time>.log)")
    ap.add_argument("--timeout", type=float, default=1800.0, help="seconds without LOOP-STATUS before giving up")
    args = ap.parse_args(argv)

    if not args.cmd.strip().startswith("adc -cal loop"):
        print("refusing: --cmd must be an 'adc -cal loop ...' command")
        return 2
    try:
        import serial  # noqa: PLC0415
    except ImportError:
        print("pyserial is required: pip install pyserial")
        return 2

    out = args.out or os.path.join("calibration_out", time.strftime("board_loop_%Y%m%d_%H%M%S.log"))
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)

    ser = serial.Serial(args.uart, 115200, timeout=0.5)
    time.sleep(0.2)
    ser.reset_input_buffer()

    def send(line: str) -> None:
        ser.write((line + "\r").encode())
        ser.flush()

    def read_until(log, marker: str, timeout: float) -> str:
        """Echo and log lines until one starts with ``marker``; returns how it ended."""
        deadline = time.time() + timeout
        buf = ""
        while time.time() < deadline:
            chunk = ser.read(ser.in_waiting or 1).decode(errors="replace")
            if not chunk:
                continue
            buf += chunk
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.rstrip("\r")
                print(line)
                log.write(line + "\n")
                log.flush()
                if line.startswith(marker):
                    return "ok"
                if "Command Error" in line:
                    return "error"
        return "timeout"

    with open(out, "w", encoding="utf-8", newline="\n") as log:
        log.write(f"# cmd: {args.cmd}\n# port: {args.uart}\n# start: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        ended = "ok"
        if args.wave is not None:
            log.write(f"# wave: {args.wave}\n")
            send(f"adc -cal loop wave {args.wave}")
            ended = read_until(log, "LOOP-WAVE", 10.0)
            if ended == "timeout":
                print("no LOOP-WAVE answer: is the ELF with 'adc -cal loop wave' loaded?")
        if ended == "ok":
            send(args.cmd)
            ended = read_until(log, "LOOP-STATUS", args.timeout)
        if ended != "ok":
            log.write(f"# STOPPED: {ended}\n")
    ser.close()
    print(f"\nlog: {out}" + ("" if ended == "ok" else f"   (stopped: {ended})"))
    return 0 if ended == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
