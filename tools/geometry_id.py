"""Identify the DPG waveform geometry actually loaded, from captured frames.

No configuration assumptions.  Procedure per frame:
  1. extract chA/chB at the best 8-word pattern shift
  2. estimate and remove the main tone (coherent sine fit at the peak bin)
  3. fold the residual envelope over candidate periods and score how strongly
     a pulse train repeats at that period

A high fold score at lag L means the impulse period really is L ADC samples.
Candidate lags cover every waveform file we know about, so the winner names the
loaded file.  Pulse width is then measured at the winning lag.

Usage:
    python tools/geometry_id.py --uart COM5 --frames 4
"""

from __future__ import annotations

import argparse
import socket
import sys
import time

import numpy as np

UDP_PORT = 6666
PACKETS = 8

# candidate dither periods, in ADC samples (DAC period / adc_ratio=2)
CANDIDATES = {
    32: "dither_period_dac 64",
    50: "pulse_p100? / period 100 DAC",
    64: "dither_period_dac 128  (waveforms/impulse_dither.txt)",
    130: "dither_period_dac 260 (firmware sweep geometry)",
    260: "dither_period_dac 520",
    520: "dither_period_dac 1040",
}


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
            d, _ = sock.recvfrom(2048)
            ch.append(d)
    except OSError:
        return None
    if len(ch) != PACKETS:
        return None
    w = np.frombuffer(b"".join(ch)[:4096], dtype="<u4")
    grp = np.arange(w.size) % 8
    a = (w[grp < 4] >> 16).astype(np.uint16).view(np.int16).astype(float)
    b = (w[grp >= 4] >> 16).astype(np.uint16).view(np.int16).astype(float)
    return a, b


def remove_tone(x):
    """Coherently fit and subtract the dominant sinusoid; return residual."""
    n = x.size
    t = np.arange(n)
    X = x - x.mean()
    sp = np.abs(np.fft.rfft(X))
    k = int(np.argmax(sp[1:]) + 1)
    w = 2 * np.pi * k / n
    M = np.column_stack([np.cos(w * t), np.sin(w * t), np.ones(n)])
    coef, *_ = np.linalg.lstsq(M, x, rcond=None)
    return x - M @ coef, k


def fold_score(res, p):
    """How strongly does the residual envelope repeat with period p?"""
    env = np.abs(res)
    env = env - env.mean()
    n = env.size
    m = (n // p) * p
    if m < 2 * p:
        return 0.0
    prof = env[:m].reshape(-1, p).mean(axis=0)
    return float(prof.std() / (env.std() + 1e-30))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--fs-adc", type=float, default=1300e6)
    ap.add_argument("--frames", type=int, default=4)
    args = ap.parse_args(argv)

    import serial
    ser = serial.Serial(args.uart, 115200, timeout=1.0)
    time.sleep(0.3)
    ser.reset_input_buffer()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind(("0.0.0.0", UDP_PORT))

    scores = {}
    try:
        for fi in range(args.frames):
            got = capture(ser, sock)
            if got is None:
                print(f"frame {fi}: capture FAILED")
                continue
            a, b = got
            resA, kA = remove_tone(a)
            resB, kB = remove_tone(b)
            fA = kA * args.fs_adc / a.size
            fB = kB * args.fs_adc / b.size
            print(f"\nframe {fi}: toneA={fA/1e6:8.3f} MHz  toneB={fB/1e6:8.3f} MHz")
            print(f"   rmsA={np.std(a)/4:8.2f} rmsB={np.std(b)/4:8.2f} "
                  f"| residual rmsA={np.std(resA)/4:8.2f} "
                  f"rmsB={np.std(resB)/4:8.2f}")

            for p in CANDIDATES:
                sa, sb = fold_score(resA, p), fold_score(resB, p)
                s = max(sa, sb)
                scores.setdefault(p, []).append(s)
                print(f"   period {p:4d} ADC ({CANDIDATES[p]:<52}) "
                      f"foldA={sa:.3f} foldB={sb:.3f}")
    finally:
        ser.close()
        sock.close()

    if scores:
        print("\n===== mean fold score per candidate period =====")
        ranked = sorted(((np.mean(v), p) for p, v in scores.items()), reverse=True)
        for m, p in ranked:
            bar = "#" * int(max(0.0, m) * 40)
            print(f"  {p:4d} ADC  mean={m:.3f}  {bar}")
        best = ranked[0]
        print(f"\nBEST: period {best[1]} ADC samples "
              f"({CANDIDATES[best[1]]})  score={best[0]:.3f}")
        print("A clear winner identifies the loaded waveform geometry.")
        print("If all scores are low and flat, there is no repeating pulse "
              "train in the capture at all.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
