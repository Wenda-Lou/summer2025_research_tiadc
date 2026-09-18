"""Test whether the DMA frame needs even/odd de-interleaving per channel.

Hypothesis: within the 4-word-per-sample channel groups, consecutive entries are
NOT consecutive time samples; the two channels may be interleaved differently
than assumed.  Three extraction models are compared on the SAME raw frame:

  model 0 "plain"      : A = idx 0..3, B = idx 4..7 of each 8-word group
  model 1 "deint-even" : model 0, then take every 2nd sample of each channel
  model 2 "deint-cross": A = even entries of idx0..3 + even of idx4..7, etc.

For each model the discriminator is corr(chA, chB) -- for TRULY parallel
channels it must be strongly positive -- plus the tone frequency.

Usage:
    python tools/deint_test.py --uart COM5 --frames 2
"""

from __future__ import annotations

import argparse
import socket
import sys
import time

import numpy as np

UDP_PORT = 6666
PACKETS = 8


def capture(ser, sock, listen: float):
    sock.settimeout(0.05)
    try:
        while True:
            sock.recvfrom(2048)
    except OSError:
        pass
    for cmd, settle in (("dma -d", 0.15), ("dma -w", 0.20), ("udp", 0.0)):
        ser.write((cmd + "\r").encode())
        ser.flush()
        if settle:
            time.sleep(settle)
    sock.settimeout(listen)
    chunks = []
    try:
        for _ in range(PACKETS):
            d, _ = sock.recvfrom(2048)
            chunks.append(d)
    except OSError:
        return None
    return b"".join(chunks)[:4096] if len(chunks) == PACKETS else None


def codes(words):
    return (words >> 16).astype(np.uint16).view(np.int16).astype(float)


def tone(x, fs):
    ac = x - x.mean()
    if np.allclose(ac, 0):
        return 0.0
    w = np.hanning(ac.size)
    sp = np.abs(np.fft.rfft(ac * w))
    k = int(np.argmax(sp[1:]) + 1)
    return k * fs / ac.size


def report(tag, a, b, fs):
    ra, rb = np.std(a), np.std(b)
    line = f"  {tag:<34} n={a.size:4d}  rmsA={ra/4:8.1f} rmsB={rb/4:8.1f}"
    if ra > 0 and rb > 0 and a.size == b.size:
        c = float(np.corrcoef(a - a.mean(), b - b.mean())[0, 1])
        line += f"  corr={c:+7.4f}"
        line += f"  fA={tone(a, fs)/1e6:8.3f} fB={tone(b, fs)/1e6:8.3f} MHz"
    print(line)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--fs-adc", type=float, default=1300e6)
    ap.add_argument("--listen", type=float, default=3.0)
    ap.add_argument("--frames", type=int, default=2)
    args = ap.parse_args(argv)

    import serial
    ser = serial.Serial(args.uart, 115200, timeout=1.0)
    time.sleep(0.3)
    ser.reset_input_buffer()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))

    try:
        for fi in range(args.frames):
            raw = capture(ser, sock, args.listen)
            if raw is None:
                print(f"frame {fi}: FAILED")
                continue
            w = np.frombuffer(raw, dtype="<u4")
            idx = np.arange(w.size)
            grp = idx % 8

            # base groups
            a_words = w[grp < 4]
            b_words = w[grp >= 4]
            a0, b0 = codes(a_words), codes(b_words)

            print(f"\n===== frame {fi} =====")
            report("plain (A=0..3, B=4..7)", a0, b0, args.fs_adc)

            # model 1: each channel's stream split even/odd in time
            report("A even vs B even", a0[0::2], b0[0::2], args.fs_adc)
            report("A odd  vs B odd ", a0[1::2], b0[1::2], args.fs_adc)
            report("A even vs B odd ", a0[0::2], b0[1::2], args.fs_adc)
            report("A odd  vs B even", a0[1::2], b0[0::2], args.fs_adc)

            # model 2: A from idx0..3 even + idx4..7 even (cross regrouping)
            a2 = codes(w[(grp < 4) | (grp >= 6)])
            b2 = codes(w[(grp >= 4) & (grp < 6)])
            report("cross: A=(0,1,2,3,6,7) B=(4,5)", a2, b2, args.fs_adc)

            # model 3: 2-sample-per-word layout -- even words are one sample
            report("A=sample-even B=sample-odd", a0[0::2], a0[1::2], args.fs_adc)
            report("B vs itself split", b0[0::2], b0[1::2], args.fs_adc)
    finally:
        ser.close()
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
