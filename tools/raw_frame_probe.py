"""Capture one raw DMA frame and measure what the board is actually sending.

Independent of the calibration loop: no config assumptions, no estimator.  It
binds 0.0.0.0:6666, runs the board's proven reset->capture->udp sequence, then
analyses the 4096 bytes:

  * word layout / byte alignment check (bit-15 aligned 14-bit data?)
  * per-channel DC and RMS
  * FFT of each channel -> dominant tone frequency (assuming fs_adc)
  * autocorrelation -> pulse repetition spacing in ADC samples

Usage:
    python tools/raw_frame_probe.py --uart COM5
    python tools/raw_frame_probe.py --uart COM5 --fs-adc 1300e6
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
PACKET_LEN = 512


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
            data, _ = sock.recvfrom(2048)
            chunks.append(data)
    except OSError:
        return None
    if len(chunks) != PACKETS:
        return None
    return b"".join(chunks)[:FRAME_BYTES]


def parse_words(raw: bytes) -> np.ndarray:
    """4096 bytes -> 1024 little-endian 32-bit words."""
    return np.frombuffer(raw, dtype="<u4")


def _channel_words(words: np.ndarray, shift: int) -> tuple[np.ndarray, np.ndarray]:
    """Split 1024 words into the 4-of-8 A group and the 4-of-8 B group,
    starting the 8-word pattern at `shift`."""
    idx = np.arange(words.size)
    pat = ((idx + shift) % 8) < 4
    return words[pat], words[~pat]


def _as_codes(w: np.ndarray, high: bool) -> np.ndarray:
    v = (w >> 16) if high else (w & 0xFFFF)
    return v.astype(np.uint16).view(np.int16).astype(float)


def score_alignment(words: np.ndarray, shift: int, high: bool = True):
    """Return (rms_a, rms_b) when the 8-word pattern starts at `shift`."""
    a_words, b_words = _channel_words(words, shift)
    a = _as_codes(a_words, high)
    b = _as_codes(b_words, high)
    return float(np.std(a)), float(np.std(b))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--fs-adc", type=float, default=1300e6)
    ap.add_argument("--listen", type=float, default=3.0)
    ap.add_argument("--frames", type=int, default=3)
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
            words = parse_words(raw)
            print(f"\n===== frame {fi} ({len(raw)} bytes, {words.size} words) =====")

            # 1. which 4-of-8 word pattern gives the most ADC-like data?
            print("  alignment sweep (rms of chA/chB by 8-word pattern start):")
            for hi in (True, False):
                tag = "high16" if hi else "low16 "
                row = []
                for sh in range(8):
                    ra, rb = score_alignment(words, sh, hi)
                    row.append(f"sh{sh}:{ra:7.1f}/{rb:7.1f}")
                print(f"    {tag} " + "  ".join(row))

            pat = (np.arange(words.size) % 8) < 4
            aw, bw = words[pat], words[~pat]
            a = _as_codes(aw, True)
            b = _as_codes(bw, True)

            for name, ch in (("A", a), ("B", b)):
                dc = ch.mean()
                rms = ch.std()
                ac = ch - dc
                win = np.hanning(ac.size)
                spec = np.abs(np.fft.rfft(ac * win))
                k = int(np.argmax(spec[1:]) + 1)
                freq = k * args.fs_adc / ac.size
                # parabolic peak interpolation for a better frequency read
                if 0 < k < spec.size - 1:
                    y0, y1, y2 = spec[k - 1], spec[k], spec[k + 1]
                    d = 0.5 * (y0 - y2) / (y0 - 2 * y1 + y2 + 1e-30)
                else:
                    d = 0.0
                freq_i = (k + d) * args.fs_adc / ac.size
                print(f"  ch{name}: DC={dc:9.2f}  rms={rms:8.2f}  "
                      f"peak bin={k:4d}  f={freq/1e6:9.3f} MHz "
                      f"(interp {freq_i/1e6:9.3f} MHz)")

                # pulse spacing via autocorrelation of the envelope
                env = np.abs(ac)
                env = env - env.mean()
                acorr = np.correlate(env, env, mode="full")[env.size - 1:]
                acorr /= acorr[0] + 1e-30
                lo, hi = 4, min(400, acorr.size - 1)
                pk = lo + int(np.argmax(acorr[lo:hi]))
                print(f"        envelope autocorr peak at lag={pk} ADC samples "
                      f"(corr={acorr[pk]:.3f})")

                codes = ch / 4.0
                print(f"        as 16-bit codes: min={codes.min():8.1f} "
                      f"max={codes.max():8.1f} p2p={codes.max()-codes.min():8.1f}")
    finally:
        ser.close()
        sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
