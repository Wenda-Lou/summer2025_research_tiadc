"""Read-only verification of the raw-frame packing, using an independent method.

For each candidate packing of the 8 x 16-bit words in a beat, splits the frame
into two channel streams and scores them on FOUR independent criteria:

  1. tone purity              - each channel must be a clean single tone
  2. A/B correlation          - parallel channels must correlate (sign noted)
  3. pulse periodicity        - envelope must repeat with the known 130-sample
                                dither period (lock-in / fold score)
  4. event count              - n_samples / 130, should be ~7.8 for 1020 samples

A packing is correct only if ALL FOUR agree.  The dither period is the key
discriminator: only a correct split reproduces the 130-sample impulse train.

Usage:
    python tools/verify_packing.py --uart COM5 --frames 2
"""

from __future__ import annotations

import argparse
import socket
import sys
import time

import numpy as np

UDP_PORT = 6666
PACKETS = 8
SLOT = 130


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
    if x.size < 32 or np.allclose(x, x.mean()):
        return 0.0
    ac = x - x.mean()
    sp = np.abs(np.fft.rfft(ac * np.hanning(ac.size))) ** 2
    return float(sp[1:].max() / (sp[1:].sum() + 1e-30))


def fold_score(res, p):
    """Strength of a period-p pulse train in the envelope (tone removed)."""
    env = np.abs(res - res.mean())
    env = env - env.mean()
    n = env.size
    m = (n // p) * p
    if m < 2 * p:
        return 0.0
    prof = env[:m].reshape(-1, p).mean(axis=0)
    return float(prof.std() / (env.std() + 1e-30))


def remove_tone(x):
    n = x.size
    t = np.arange(n)
    X = x - x.mean()
    k = int(np.argmax(np.abs(np.fft.rfft(X))[1:]) + 1)
    w = 2 * np.pi * k / n
    M = np.column_stack([np.cos(w * t), np.sin(w * t), np.ones(n)])
    coef, *_ = np.linalg.lstsq(M, x, rcond=None)
    return x - M @ coef


def score(name, a, b, out):
    if a.size < 64 or a.size != b.size:
        out.append((name, a.size, None))
        return
    pa, pb = purity(a), purity(b)
    c = float(np.corrcoef(a - a.mean(), b - b.mean())[0, 1])
    fa = fold_score(remove_tone(a), SLOT)
    fb = fold_score(remove_tone(b), SLOT)
    ne = a.size / SLOT
    out.append((name, a.size, dict(purity_a=pa, purity_b=pb, corr=c,
                                   fold_a=fa, fold_b=fb, events=ne)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--frames", type=int, default=2)
    args = ap.parse_args(argv)

    import serial
    ser = serial.Serial(args.uart, 115200, timeout=1.0)
    time.sleep(0.3)
    ser.reset_input_buffer()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind(("0.0.0.0", UDP_PORT))

    frames = []
    try:
        for _ in range(args.frames):
            r = capture(ser, sock)
            if r:
                frames.append(r)
    finally:
        ser.close()
        sock.close()
    if not frames:
        print("no capture")
        return 1

    agg = {}
    for raw in frames:
        w = np.frombuffer(raw[: (len(raw) // 2) * 2], dtype="<i2").astype(np.float64) / 4.0

        # ---- H1: even/odd 16-bit words are A/B (1 word = 1 sample) ----
        score("H1 even=A odd=B", w[0::2], w[1::2], [])
        # ---- H2: first4/last4 of each 8-word group (current split_at) ----
        g = w[: (w.size // 8) * 8].reshape(-1, 8)
        a2, b2 = g[:, :4].reshape(-1), g[:, 4:].reshape(-1)
        # ---- H3: 32-bit sample: even 32-bit word = A, odd = B ----
        w32 = np.frombuffer(raw[: (len(raw) // 4) * 4], dtype="<i4").astype(np.float64)
        a3, b3 = w32[0::2], w32[1::2]
        # ---- H4: within each 8-word group, even positions = A, odd = B ----
        a4, b4 = g[:, 0::2].reshape(-1), g[:, 1::2].reshape(-1)

        for nm, a, b in (("H1 even=A odd=B", w[0::2], w[1::2]),
                         ("H2 split<4 vs >=4", a2, b2),
                         ("H3 32bit even/odd", a3, b3),
                         ("H4 gpos even/odd", a4, b4)):
            tmp = []
            score(nm, a, b, tmp)
            if tmp and tmp[0][2]:
                agg.setdefault(nm, []).append(tmp[0][2])

    print(f"frames analysed: {len(frames)}   (dither slot = {SLOT} ADC samples)")
    print(f"\n{'hypothesis':<22} {'n/chan':>7} {'purA':>6} {'purB':>6} "
          f"{'corr':>8} {'foldA':>6} {'foldB':>6} {'events':>7}")
    print("-" * 78)
    for nm, rows in agg.items():
        n = rows[0]["events"] * SLOT if "events" in rows[0] else 0
        mean = {k: float(np.mean([r[k] for r in rows])) for k in rows[0]}
        print(f"{nm:<22} {int(round(mean['events']*SLOT)):7d} "
              f"{mean['purity_a']:6.3f} {mean['purity_b']:6.3f} "
              f"{mean['corr']:+8.4f} {mean['fold_a']:6.3f} {mean['fold_b']:6.3f} "
              f"{mean['events']:7.2f}")
    print("-" * 78)
    print("Correct packing needs: high purity on BOTH, |corr| near 1, and a")
    print("clear 130-sample fold peak on BOTH channels.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
