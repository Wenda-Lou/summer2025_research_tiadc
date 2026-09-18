"""Settle the DMA frame layout empirically: how many ADC samples per channel?

Compares two candidate de-framings of the same raw capture and reports which one
yields a clean tone on BOTH channels with high A/B correlation (the channels are
physically parallel, so a correct layout must correlate strongly).

  model "4+4"  : words [0..3] -> A, [4..7] -> B   (4 samples/channel per 8-word
                 group; this is what estimator.split_at does today)
  model "2+2"  : two words per sample, [A B] alternating, packed
                 (4 samples/channel per 8-word group, adjacent-word form)

Reports sample counts, tone purity, and cross-correlation for each, for every
8-word rotation, so the winner is unambiguous.

Usage:
    python tools/frame_layout_test.py --uart COM5 --frames 3
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
            sock.recvfrom(2048)
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


def purity(x: np.ndarray) -> float:
    if x.size < 8 or np.allclose(x, x.mean()):
        return 0.0
    ac = x - x.mean()
    sp = np.abs(np.fft.rfft(ac * np.hanning(ac.size))) ** 2
    return float(sp[1:].max() / (sp[1:].sum() + 1e-30))


def model_4x4(words, rot):
    body = words[rot:]
    body = body[: (body.size // 8) * 8].reshape(-1, 8)
    return (body[:, :4].reshape(-1).astype(np.float64),
            body[:, 4:].reshape(-1).astype(np.float64))


def model_2x2(words, rot):
    """Two words per sample: pairs (A,B),(A,B)... 4 samples per 8-word group."""
    body = words[rot:]
    body = body[: (body.size // 8) * 8].reshape(-1, 8)
    a = body[:, 0:8:2].reshape(-1).astype(np.float64)
    b = body[:, 1:8:2].reshape(-1).astype(np.float64)
    return a, b


def model_1x1(words, rot):
    """One word per sample: first half of the group -> A, second -> B."""
    body = words[rot:]
    body = body[: (body.size // 8) * 8].reshape(-1, 8)
    return (body[:, :4].reshape(-1)[::1].astype(np.float64),
            body[:, 4:].reshape(-1)[::1].astype(np.float64))


def report(tag, a, b, fs):
    out = f"  {tag:<22} nA={a.size:5d} nB={b.size:5d}"
    pa, pb = purity(a), purity(b)
    out += f"  purityA={pa:.3f} purityB={pb:.3f}"
    if a.size == b.size and a.size > 8:
        c = float(np.corrcoef(a - a.mean(), b - b.mean())[0, 1])
        out += f"  corr(A,B)={c:+.4f}"
        if not np.allclose(a, a.mean()):
            sp = np.abs(np.fft.rfft((a - a.mean()) * np.hanning(a.size)))
            k = int(np.argmax(sp[1:]) + 1)
            out += f"  fA={k*fs/a.size/1e6:8.3f}MHz"
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--fs-adc", type=float, default=1300e6)
    ap.add_argument("--frames", type=int, default=3)
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
        for fi in range(args.frames):
            raw = capture(ser, sock)
            if raw is None:
                print(f"frame {fi}: FAILED")
                continue
            print(f"\n===== frame {fi}: {len(raw)} bytes = "
                  f"{len(raw)//4} words =====")
            words = np.frombuffer(raw[: (len(raw) // 4) * 4], dtype="<i2")
            words = (words >> 2).astype(np.int32)
            for rot in range(8):
                a4, b4 = model_4x4(words, rot)
                a2, b2 = model_2x2(words, rot)
                print(report(f"rot{rot} model 4+4", a4, b4, args.fs_adc))
                print(report(f"rot{rot} model 2+2", a2, b2, args.fs_adc))
    finally:
        ser.close()
        sock.close()
    print("\nWinner = the model with high purity on BOTH channels AND strong "
          "positive corr(A,B); parallel channels must correlate.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
