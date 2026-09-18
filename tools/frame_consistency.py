"""Check whether consecutive captured frames are self-consistent.

If the host reassembles 8 UDP packets that do not belong to the same DMA
transfer, every downstream number is garbage.  This tool captures N frames
back-to-back with a strict drain-before and drain-after, then reports per-frame
statistics so inconsistency is obvious:

  * per-frame rms of each channel (should be stable frame to frame)
  * inter-frame correlation of chA (consecutive frames of the same tone are
    NOT expected to be identical, but the amplitude should be stable)
  * MD5 of each raw frame (all-identical => frozen DMA buffer)

Usage:
    python tools/frame_consistency.py --uart COM5 --frames 6
"""

from __future__ import annotations

import argparse
import hashlib
import socket
import sys
import time

import numpy as np

UDP_PORT = 6666
PACKETS = 8
PACKET_LEN = 512


def drain(sock, seconds=0.4):
    sock.settimeout(0.02)
    end = time.time() + seconds
    n = 0
    while time.time() < end:
        try:
            sock.recvfrom(2048)
            n += 1
        except OSError:
            pass
    return n


def grab(ser, sock, listen=2.0):
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
        return None, len(chunks)
    return b"".join(chunks), len(chunks)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--frames", type=int, default=6)
    ap.add_argument("--listen", type=float, default=2.0)
    args = ap.parse_args(argv)

    import serial
    ser = serial.Serial(args.uart, 115200, timeout=1.0)
    time.sleep(0.3)
    ser.reset_input_buffer()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    sock.bind(("0.0.0.0", UDP_PORT))

    rms_a, rms_b, md5s = [], [], []
    prev_a = None
    try:
        pre = drain(sock, 0.5)
        print(f"drained {pre} stale packets before starting\n")
        for i in range(args.frames):
            raw, got = grab(ser, sock, args.listen)
            if raw is None or len(raw) < 4096:
                print(f"frame {i}: INCOMPLETE ({got}/{PACKETS} packets)")
                drain(sock, 0.3)
                continue
            extra = drain(sock, 0.3)

            w = np.frombuffer(raw[:4096], dtype="<u4")
            grp = np.arange(w.size) % 8
            a = (w[grp < 4] >> 16).astype(np.uint16).view(np.int16).astype(float)
            b = (w[grp >= 4] >> 16).astype(np.uint16).view(np.int16).astype(float)
            ra, rb = np.std(a) / 4, np.std(b) / 4
            rms_a.append(ra)
            rms_b.append(rb)
            m = hashlib.md5(raw[:4096]).hexdigest()[:10]
            md5s.append(m)

            line = (f"frame {i}: rmsA={ra:8.2f} rmsB={rb:8.2f} "
                    f"dcA={a.mean()/4:8.1f} dcB={b.mean()/4:8.1f} md5={m}")
            if prev_a is not None and prev_a.size == a.size:
                c = float(np.corrcoef(prev_a - prev_a.mean(),
                                      a - a.mean())[0, 1])
                line += f" corr(prevA,A)={c:+.3f}"
            line += f" packets={got} extra_after={extra}"
            print(line)
            prev_a = a
    finally:
        ser.close()
        sock.close()

    if rms_a:
        print(f"\nrmsA: mean={np.mean(rms_a):.1f} std={np.std(rms_a):.1f} "
              f"spread={max(rms_a)-min(rms_a):.1f}")
        print(f"rmsB: mean={np.mean(rms_b):.1f} std={np.std(rms_b):.1f} "
              f"spread={max(rms_b)-min(rms_b):.1f}")
        uniq = len(set(md5s))
        print(f"unique frame hashes: {uniq}/{len(md5s)}"
              + ("  <-- FROZEN DMA BUFFER" if uniq == 1 else ""))
        if np.std(rms_a) > 0.2 * np.mean(rms_a):
            print("rmsA varies >20% frame-to-frame -> capture is NOT reliable")
        else:
            print("rmsA stable -> capture reassembly looks clean")
    return 0


if __name__ == "__main__":
    sys.exit(main())
