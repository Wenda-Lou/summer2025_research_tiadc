"""Dump the first raw 16-bit words of a live DMA frame, plus per-channel stats.

No layout assumption: prints raw hex/dec words so the true packing is visible,
then evaluates every plausible (a_word, b_word) pairing inside the 8-word group
and reports tone purity + A/B correlation for each, so the physical layout can be
read directly instead of inferred.

Usage:
    python tools/dump_raw_words.py --uart COM5
"""

from __future__ import annotations

import argparse
import socket
import sys
import time

import numpy as np

UDP_PORT = 6666
PACKETS = 8


def capture(ser, sock, listen=2.0):
    sock.settimeout(0.02)
    t = time.time() + 0.4
    while time.time() < t:
        try:
            sock.recvfrom(65535)
        except OSError:
            pass
    for cmd, settle in (("dma -d", 0.15), ("dma -w", 0.20), ("udp", 0.0)):
        ser.write((cmd + "\r").encode())
        ser.flush()
        if settle:
            time.sleep(settle)
    sock.settimeout(listen)
    ch = []
    try:
        for _ in range(PACKETS):
            d, _ = sock.recvfrom(65535)
            ch.append(d)
    except OSError:
        return None
    return b"".join(ch)


def purity(x):
    if x.size < 16 or np.allclose(x, x.mean()):
        return 0.0
    ac = x - x.mean()
    sp = np.abs(np.fft.rfft(ac * np.hanning(ac.size))) ** 2
    return float(sp[1:].max() / (sp[1:].sum() + 1e-30))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--fs-adc", type=float, default=1300e6)
    args = ap.parse_args(argv)

    import serial
    ser = serial.Serial(args.uart, 115200, timeout=1.0)
    time.sleep(0.3)
    ser.reset_input_buffer()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind(("0.0.0.0", UDP_PORT))

    try:
        raw = capture(ser, sock)
    finally:
        ser.close()
        sock.close()
    if raw is None:
        print("capture FAILED")
        return 1

    print(f"frame = {len(raw)} bytes = {len(raw)//4} x 32-bit words "
          f"= {len(raw)//2} x 16-bit words")

    w16 = np.frombuffer(raw[: (len(raw) // 2) * 2], dtype="<i2").astype(np.int32)
    print(f"\n=== first 48 raw 16-bit words ===")
    for i in range(0, min(48, w16.size), 8):
        grp = w16[i:i + 8]
        print("  " + "  ".join(f"{v:7d}" for v in grp)
              + "   | raw " + " ".join(f"{v & 0xFFFF:04X}" for v in grp))

    # shift-by-2 view (raw 16-bit -> 14-bit codes)
    codes = w16 >> 2
    print(f"\n=== shift-2 view, first 24 ===")
    print("  " + "  ".join(f"{v:6d}" for v in codes[:24]))

    print("\n=== per-position statistics over the whole frame ===")
    print("  idx   count      mean       std      min      max")
    for p in range(8):
        col = codes[p::8]
        if col.size:
            print(f"   {p}  {col.size:7d}  {col.mean():9.1f}  {col.std():8.1f} "
                  f"{col.min():8.0f} {col.max():8.0f}")

    print("\n=== pairing candidates: which (A positions, B positions) "
          "gives parallel channels? ===")
    best = []
    for k in range(1, 8):
        a = codes[np.arange(codes.size) % 8 < k]
        b = codes[np.arange(codes.size) % 8 >= k]
        if a.size == b.size and a.size > 32:
            c = float(np.corrcoef(a - a.mean(), b - b.mean())[0, 1])
            best.append((abs(c), c, f"split<{k} vs >={k}", a.size, purity(a), purity(b)))
    for lags in [(0, 1), (0, 2), (1, 2), (0, 3)]:
        pass
    # also test even/odd word pairing
    a = codes[0::2][: codes.size // 2]
    b = codes[1::2][: codes.size // 2]
    if a.size == b.size and a.size > 32:
        c = float(np.corrcoef(a - a.mean(), b - b.mean())[0, 1])
        best.append((abs(c), c, "even words vs odd words", a.size, purity(a), purity(b)))
    for k in range(1, 7):
        a = codes[k::8][: codes.size // 8]
        b = codes[(k + 4) % 8::8][: codes.size // 8]
        if a.size == b.size and a.size > 32:
            c = float(np.corrcoef(a - a.mean(), b - b.mean())[0, 1])
            best.append((abs(c), c, f"pos{k} vs pos{(k+4)%8}", a.size, purity(a), purity(b)))

    best.sort(reverse=True)
    for _, c, name, n, pa, pb in best[:12]:
        print(f"  {name:<34} n={n:5d} corr={c:+.4f} purityA={pa:.3f} purityB={pb:.3f}")
    print("\nCorrect layout: BOTH purities high AND corr strongly positive.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
