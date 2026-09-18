"""Rigorous channel analysis of raw DMA frames.

Answers, per channel and per frame:
  * proper channel extraction at the best 8-word pattern shift
  * DC in raw 14-bit codes (data is left-aligned, so >>2)
  * coherent-FFT tone frequency with parabolic interpolation
  * chA-vs-chB correlation (parallel channels MUST correlate if both alive)
  * lock-in periodicity search for the impulse pulse train, in ADC samples

Usage:
    python tools/chan_analysis.py --uart COM5 --frames 3
    python tools/chan_analysis.py --uart COM5 --fs-adc 1300e6 --search-max 200
"""

from __future__ import annotations

import argparse
import socket
import sys
import time

import numpy as np

UDP_PORT = 6666
FRAME_BYTES = 4096
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
    return b"".join(chunks)[:FRAME_BYTES] if len(chunks) == PACKETS else None


def extract(raw: bytes, shift: int, high: bool = True):
    words = np.frombuffer(raw, dtype="<u4")
    idx = np.arange(words.size)
    pat = ((idx + shift) % 8) < 4
    def codes(w):
        v = (w >> 16) if high else (w & 0xFFFF)
        return v.astype(np.uint16).view(np.int16).astype(float)
    return codes(words[pat]), codes(words[~pat])


def tone_freq(x: np.ndarray, fs: float) -> tuple[float, float]:
    ac = x - x.mean()
    if np.allclose(ac, 0):
        return 0.0, 0.0
    win = np.hanning(ac.size)
    spec = np.abs(np.fft.rfft(ac * win))
    k = int(np.argmax(spec[1:]) + 1)
    d = 0.0
    if 0 < k < spec.size - 1:
        y0, y1, y2 = spec[k - 1], spec[k], spec[k + 1]
        den = y0 - 2 * y1 + y2
        if den != 0:
            d = 0.5 * (y0 - y2) / den
    kk = k + d
    f = kk * fs / ac.size
    if f > fs / 2:
        f = fs - f
    return abs(f), spec[k]


def lockin(x: np.ndarray, fs: float, lo: int, hi: int) -> tuple[int, float]:
    """Search pulse period p in [lo,hi] ADC samples by folding the envelope."""
    ac = x - x.mean()
    env = np.abs(ac)
    env = env - env.mean()
    best_p, best_s = 0, -2.0
    n = env.size
    for p in range(lo, hi + 1):
        # fold onto p phases and take the variance of the phase means
        trimmed = env[: (n // p) * p].reshape(-1, p)
        means = trimmed.mean(axis=0)
        s = float(means.std() / (env.std() + 1e-30))
        if s > best_s:
            best_s, best_p = s, p
    return best_p, best_s


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--fs-adc", type=float, default=1300e6)
    ap.add_argument("--listen", type=float, default=3.0)
    ap.add_argument("--frames", type=int, default=3)
    ap.add_argument("--search-max", type=int, default=200)
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
                print(f"frame {fi}: capture FAILED")
                continue

            # pick the shift whose A/B rms are most unequal-free (both alive)
            best = None
            for sh in range(8):
                a, b = extract(raw, sh)
                ra, rb = np.std(a), np.std(b)
                score = min(ra, rb)          # want BOTH channels alive
                if best is None or score > best[0]:
                    best = (score, sh, a, b)
            _, sh, a, b = best

            print(f"\n===== frame {fi}  (best 8-word shift = {sh}) =====")
            for nm, ch in (("A", a), ("B", b)):
                f, mag = tone_freq(ch, args.fs_adc)
                print(f"  ch{nm}: DC={ch.mean()/4:9.1f} codes   "
                      f"rms={np.std(ch)/4:8.2f} codes   "
                      f"min/max={ch.min()/4:9.1f}/{ch.max()/4:9.1f}   "
                      f"tone={f/1e6:8.3f} MHz")

            if np.std(a) > 0 and np.std(b) > 0:
                c = float(np.corrcoef(a - a.mean(), b - b.mean())[0, 1])
                print(f"  corr(A,B) = {c:+.4f}   "
                      f"<- parallel channels should be strongly +correlated")

            for nm, ch in (("A", a), ("B", b)):
                p, s = lockin(ch, args.fs_adc, 4, args.search_max)
                print(f"  ch{nm} pulse-period search: best lag = {p} ADC samples "
                      f"(fold score {s:.3f}, 1.00 = pure impulse train)")
    finally:
        ser.close()
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
