"""
Capture N consecutive raw DMA frames from the bench into CSV files.

    python -m calibration_loop.capture_frames --uart COM3 --frames 10

Each frame runs one full reset-settle-transfer cycle over the existing
console/UDP path (``dma -d`` -> ``dma -w`` -> ``udp``), exactly like the
calibration loop does, so the AD9695 register state stays fixed across the
batch.  That fixed-register batch is what the cross-frame joint dither
analysis needs (dither_replay.py consumes these files): all frames share
one skew working point, which the 2026-08-19 manual 3-frame grabs could not
guarantee.

Output files follow the recorded convention ``adc_capture_YYYYMMDD_HHMMSS.csv``
(``byte`` header + 4096 byte values, one per line), written into the
firmware ``adc_data`` directory by default.

Usage:
    python -m calibration_loop.capture_frames --uart COM3 --frames 10
    python -m calibration_loop.capture_frames --uart COM3 --frames 30 \
        --out F:/captures --prefix skew_reg35
"""

from __future__ import annotations

import argparse
import datetime
import sys
import time
from pathlib import Path

import numpy as np

from .capture import BOARD_IP, PACKETS_PER_FRAME, PACKET_SIZE, UDP_PORT
from .capture import UartConsole, UdpFrameReceiver

DEFAULT_OUT_DIR = (
    Path(__file__).resolve().parent.parent
    / "test_platform"
    / "thesis_v3_500mhz_appl"
    / "adc_data"
)

# dither_replay.load_capture() expects exactly this many byte values.
FRAME_BYTES = 4096


def capture_one_frame(rx: UdpFrameReceiver, console: UartConsole,
                      timeout: float = 3.0) -> bytes | None:
    """One DMA reset-settle-transfer cycle; returns the full 4096 bytes
    shipped by the board (8 x 512-byte UDP datagrams)."""
    rx.drain()
    console.send("dma -d", settle=0.15)
    console.send("dma -w", settle=0.20)
    console.send("udp", settle=0.0)
    return receive_one_frame(rx, timeout)


def receive_one_frame(rx: UdpFrameReceiver, timeout: float = 3.0) -> bytes | None:
    """Collect one 8 x 512-byte UDP frame (used by both the single-frame
    loop and the firmware burst mode)."""
    rx.sock.settimeout(timeout)
    chunks = []
    try:
        for _ in range(PACKETS_PER_FRAME):
            data, _ = rx.sock.recvfrom(2048)
            chunks.append(data)
    except OSError:
        return None
    if len(chunks) != PACKETS_PER_FRAME:
        return None
    return b"".join(chunks)[:FRAME_BYTES]


def save_frame(frame: bytes | None, index: int, total: int, prefix: str,
               out_dir: Path, ok_so_far: int) -> int:
    """Persist one frame as adc_capture_<prefix><timestamp>.csv; returns
    the updated success count."""
    if frame is None or len(frame) != FRAME_BYTES:
        print(f"frame {index}/{total}: FAILED (timeout/short frame, "
              f"{0 if frame is None else len(frame)} bytes)")
        return ok_so_far
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    name = f"adc_capture_{prefix}{stamp}.csv"
    path = out_dir / name
    arr = np.frombuffer(frame, dtype=np.uint8)
    np.savetxt(path, arr, fmt="%d", header="byte", comments="")
    print(f"frame {index}/{total}: OK -> {path.name} ({arr.size} bytes)")
    return ok_so_far + 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uart", required=True,
                        help="board UART port, e.g. COM3")
    parser.add_argument("--frames", type=int, default=10,
                        help="number of frames to capture (default 10)")
    parser.add_argument("--out", default=None,
                        help="output directory (default: firmware adc_data)")
    parser.add_argument("--prefix", default="",
                        help="optional file-name prefix, e.g. skew_reg35")
    parser.add_argument("--interframe-delay", type=float, default=0.0,
                        help="extra seconds between frames")
    parser.add_argument("--no-burst", action="store_true",
                        help="use the slow single-frame UART loop instead of "
                             "the firmware `dma -burst N` command (requires "
                             "firmware with the -burst option)")
    args = parser.parse_args(argv)

    if args.frames <= 0:
        print("--frames must be positive", file=sys.stderr)
        return 2
    out_dir = Path(args.out) if args.out else DEFAULT_OUT_DIR
    out_dir.mkdir(parents=True, exist_ok=True)

    rx = UdpFrameReceiver()
    console = UartConsole(args.uart)
    try:
        if args.no_burst:
            ok = 0
            for i in range(1, args.frames + 1):
                frame = capture_one_frame(rx, console)
                ok = save_frame(frame, i, args.frames, args.prefix,
                                out_dir, ok)
                if args.interframe_delay > 0.0 and i < args.frames:
                    time.sleep(args.interframe_delay)
        else:
            rx.drain()
            console.send(f"dma -burst {args.frames}", settle=0.5)
            ok = 0
            for i in range(1, args.frames + 1):
                frame = receive_one_frame(rx)
                ok = save_frame(frame, i, args.frames, args.prefix,
                                out_dir, ok)
    finally:
        console.close()
        rx.close()
    print(f"captured {ok}/{args.frames} frames into {out_dir}")
    return 0 if ok > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
