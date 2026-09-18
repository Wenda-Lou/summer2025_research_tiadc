"""Diagnose the calibration_loop estimator against real board frames.

Runs the production estimator path (HardwareBench + prepare_capture +
estimate_block) on live captures and dumps the per-channel internals that the
`probe` summary hides: alignment n0/margin, events used, tone amplitude,
gain, and the raw skew of each channel.

Also reports the arithmetic that decides how many dither events can fit:
    n_events_visible ~ floor(frame_samples / slot_period)

Usage:
    python tools/loop_estimator_diag.py --uart COM5 --frames 6
"""

from __future__ import annotations

import argparse
import statistics as st
import sys

import numpy as np


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--frames", type=int, default=6)
    args = ap.parse_args(argv)

    from calibration_loop.capture import HardwareBench
    from calibration_loop.dither import DitherConfig
    from calibration_loop.estimator import estimate_block, prepare_capture

    cfg = DitherConfig()
    cfg.validate()

    n_adc_per_loop = cfg.n_adc_period
    print("=== configuration (new hardware-matched defaults) ===")
    print(f"  n_dac_points       : {cfg.n_dac_points}")
    print(f"  slot_period (ADC)  : {cfg.slot_period}")
    print(f"  n_events per loop  : {cfg.n_events}")
    print(f"  n_adc_per_loop     : {n_adc_per_loop}")
    print(f"  pulse_offset (ADC) : {cfg.pulse_offset}")
    print(f"  edge/top (ADC)     : {cfg.edge_r}/{cfg.top_w}")
    print(f"  f_sig              : {cfg.f_sig/1e6:.4f} MHz")

    bench = HardwareBench(uart_port=args.uart)
    try:
        rows = []
        sig = None
        for i in range(args.frames):
            raw = bench.capture()
            if raw is None:
                print(f"\nframe {i}: capture FAILED")
                continue
            n_words = len(raw) // 4
            prep = prepare_capture(raw, cfg, signature=sig)
            if sig is None:
                sig = prep["signature"]
            est = estimate_block(prep["ch_a"], prep["ch_b"], cfg, n0=prep["n0"])

            frame_samples = prep["ch_a"].size
            theo = int(np.floor(frame_samples / cfg.slot_period))
            if i == 0:
                print("\n=== frame geometry ===")
                print(f"  raw bytes          : {len(raw)}")
                print(f"  32-bit words       : {n_words}")
                print(f"  ADC samples/channel: {frame_samples}")
                print(f"  visible slots in frame: floor({frame_samples}/{cfg.slot_period})"
                      f" = {theo}")
                print(f"  firmware analysis window for comparison: 1016 samples "
                      f"-> {1016 // cfg.slot_period} slots")

            a, b = est.ch_a, est.ch_b
            rows.append((prep, est))
            print(f"\n--- frame {i} ---")
            print(f"  rotation={prep['rotation']}  swapped={prep['swapped']}  "
                  f"n0={prep['n0']}")
            for nm, c in (("A", a), ("B", b)):
                print(f"  ch{nm}: align_margin={c.align_margin:6.2f}  "
                      f"n0={c.align_n0:6d}  events={c.n_events_used:3d}  "
                      f"tone_amp={c.tone_amplitude:9.2f}  "
                      f"gain={c.gain_codes:9.2f}  offset={c.offset_codes:8.2f}  "
                      f"resid_rms={c.residual_rms:8.2f}")
                print(f"       skew_samples={c.skew_samples:+.6f}  "
                      f"skew_ps={c.skew_ps:+10.2f}")
            print(f"  BLOCK: gB/gA={est.gain_ratio:+.5f}  "
                  f"offset_mismatch={est.offset_mismatch_codes:+8.3f} LSB  "
                  f"skew_mismatch={est.skew_mismatch_ps:+11.2f} ps")
            print(f"         skew_source={est.skew_source:<22} "
                  f"phase_ps={est.skew_phase_ps:+11.2f}  "
                  f"centroid_ps={est.skew_centroid_ps:+11.2f}")

        if not rows:
            print("\nno usable frames")
            return 1

        print("\n=== summary over frames ===")
        for label, vals in (
            ("margin(A)", [p["align_margin"] for p, _ in rows]),
            ("margin(B)", [e.ch_b.align_margin for _, e in rows]),
            ("events(A)", [float(e.ch_a.n_events_used) for _, e in rows]),
            ("gainA", [e.ch_a.gain_codes for _, e in rows]),
            ("gainB", [e.ch_b.gain_codes for _, e in rows]),
            ("gB/gA", [e.gain_ratio for _, e in rows]),
            ("skewA_ps", [e.ch_a.skew_ps for _, e in rows]),
            ("skewB_ps", [e.ch_b.skew_ps for _, e in rows]),
            ("dSkew_ps", [e.skew_mismatch_ps for _, e in rows]),
        ):
            f = [v for v in vals if np.isfinite(v)]
            if f:
                print(f"  {label:<10} mean={st.mean(f):+12.4f}  std={st.pstdev(f):11.4f}  "
                      f"min={min(f):+12.4f}  max={max(f):+12.4f}  n={len(f)}")
        print("\n  note: probe requires margin>6, pooled events>=8, "
              "joint gB/gA uncertainty<0.02")
    finally:
        bench.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
