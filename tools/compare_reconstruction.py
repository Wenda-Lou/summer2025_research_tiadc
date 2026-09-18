"""Compare the firmware's frame reconstruction against calibration_loop's.

Both consume the SAME raw DMA bytes but produce different channel streams:

  firmware        : 8 x 16-bit words per beat -> 4 samples/channel (M=2 assumption)
                    imported as ADC_CHANNEL_SAMPLE_COUNT = 1016 samples/channel
  calibration_loop: split_at(group=8, half=4) -> 1 sample per word, 4 words per
                    channel per beat

The firmware reports tone_A_rmse ~= tone_B_rmse ~= 10.6 codes on this bench, so
if the loop's channel B is 3x noisier than its channel A, the fault is in the
loop's extraction, not in the hardware.  This tool runs both on identical bytes
and reports rms / tone amplitude / residual rms per channel for each.

Usage:
    python tools/compare_reconstruction.py
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np


def loop_view(raw: bytes, cfg):
    from calibration_loop.estimator import prepare_capture
    pr = prepare_capture(raw, cfg)
    return pr["ch_a"], pr["ch_b"], pr


def firmware_fixture_bytes(path: str) -> bytes:
    arr = np.loadtxt(path, delimiter=",", skiprows=1, dtype=np.uint8)
    return arr.astype(np.uint8).tobytes()


def report(tag, a, b, cfg, f0):
    from calibration_loop.estimator import fit_tone
    print(f"\n--- {tag} ---")
    for nm, y in (("A", a), ("B", b)):
        ft = fit_tone(y, f0, refine=True)
        print(f"  ch{nm}: n={y.size:5d}  dc={y.mean():+9.2f}  rms={np.std(y):8.2f}  "
              f"tone_amp={ft['amplitude']:8.2f}  resid_rms={np.std(ft['residual']):8.2f}")
    if a.size == b.size:
        c = float(np.corrcoef(a - a.mean(), b - b.mean())[0, 1])
        print(f"  corr(A,B) = {c:+.4f}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--live-frames", type=int, default=3)
    args = ap.parse_args(argv)

    from calibration_loop.dither import DitherConfig
    from calibration_loop.estimator import prepare_capture

    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period

    base = os.path.join("firmware", "thesis_v3_500mhz_appl", "adc_data")
    if not os.path.isdir(base):
        print(f"fixture dir not found: {base}")
        return 1
    files = sorted(f for f in os.listdir(base) if f.startswith("adc_capture_"))
    if not files:
        print("no fixture captures")
        return 1

    for name in files:
        raw = firmware_fixture_bytes(os.path.join(base, name))
        print("=" * 78)
        print(f"{name}   raw = {len(raw)} bytes = {len(raw)//2} x 16-bit words")

        a, b, pr = loop_view(raw, cfg)
        report(f"calibration_loop prepare_capture "
               f"(rot={pr['rotation']}, swap={pr['swapped']})", a, b, cfg, f0)

        # Naive 4+4 split with NO rotation search and NO sign normalisation, to
        # isolate how much of the asymmetry is the split itself.
        w = (np.frombuffer(raw[: (len(raw) // 2) * 2], dtype="<i2")
             .astype(np.int32) >> 2).astype(np.float64)
        g = w[: (w.size // 8) * 8].reshape(-1, 8)
        report("raw 4+4 split, no rotation/sign handling",
               g[:, :4].reshape(-1), g[:, 4:].reshape(-1), cfg, f0)

    print("\nFirmware reference on the same session: tone_A_rmse = 10.66, "
          "tone_B_rmse = 10.60 codes.")
    print("If the loop view shows B ~3x noisier while the raw split does not, the")
    print("fault is in prepare_capture's rotation/sign handling, not the hardware.")

    # ---- live frames, both extraction stages -----------------------------
    try:
        from calibration_loop.capture import HardwareBench
        bench = HardwareBench(uart_port=args.uart)
    except Exception as exc:  # noqa: BLE001
        print(f"\n(live comparison skipped: {exc})")
        return 0
    print("\n" + "=" * 78)
    print(f"LIVE frames from {args.uart}")
    try:
        sig = None
        for i in range(args.live_frames):
            raw = bench.capture()
            if raw is None:
                continue
            pr = prepare_capture(raw, cfg, signature=sig) if sig is not None else \
                prepare_capture(raw, cfg)
            if sig is None:
                sig = pr["signature"]
            print(f"\n-- live frame {i} --")
            report("calibration_loop view", pr["ch_a"], pr["ch_b"], cfg, f0)
            w = (np.frombuffer(raw[: (len(raw) // 2) * 2], dtype="<i2")
                 .astype(np.int32) >> 2).astype(np.float64)
            g = w[: (w.size // 8) * 8].reshape(-1, 8)
            report("raw 4+4 split", g[:, :4].reshape(-1), g[:, 4:].reshape(-1), cfg, f0)
    finally:
        bench.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
