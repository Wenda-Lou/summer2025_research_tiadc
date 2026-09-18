"""Test the calibration_loop estimator against BenchModel ground truth.

Uses the same frame byte-length the hardware actually returns (4095 bytes) so
the visible-event count matches the bench.  If the estimator recovers the known
skew/gain here but not on hardware, the fault is signal/model mismatch; if it
fails here too, the estimator itself is broken.

Usage:
    python tools/estimator_truth_test.py
    python tools/estimator_truth_test.py --skew-b-ps 0 --frames 8
"""

from __future__ import annotations

import argparse
import statistics as st
import sys

import numpy as np


def run(cfg, skew_b_ps: float, frames: int, words: int, noise: float):
    from calibration_loop.estimator import estimate_block, prepare_capture
    from calibration_loop.simulate import BenchModel

    bench = BenchModel(cfg=cfg, skew_b_ps=skew_b_ps, noise_rms_codes=noise)
    truth_gain = bench.gain_b / bench.gain_a
    sig = None
    ga, gb, gm, sk, dsk, ev, mg = [], [], [], [], [], [], []
    for _ in range(frames):
        raw = bench.capture(n_words=words)
        prep = prepare_capture(raw, cfg, signature=sig)
        if sig is None:
            sig = prep["signature"]
        est = estimate_block(prep["ch_a"], prep["ch_b"], cfg, n0=prep["n0"])
        ga.append(est.ch_a.gain_codes)
        gb.append(est.ch_b.gain_codes)
        gm.append(est.gain_ratio)
        sk.append(est.ch_a.skew_ps)
        sk.append(est.ch_b.skew_ps)
        dsk.append(est.skew_mismatch_ps)
        ev.append(est.ch_a.n_events_used)
        mg.append(prep["align_margin"])
    return dict(truth_gain=truth_gain, ga=ga, gb=gb, gm=gm, dsk=dsk,
                sk=sk, ev=ev, mg=mg)


def run_joint(cfg, skew_b_ps: float, frames: int, words: int, noise: float):
    """Same kind of frames, but ONE joint estimate pooled across all of them."""
    from calibration_loop.estimator import (
        estimate_block, estimate_block_joint, prepare_capture,
    )
    from calibration_loop.simulate import BenchModel

    bench = BenchModel(cfg=cfg, skew_b_ps=skew_b_ps, noise_rms_codes=noise)
    sig = None
    batch, per_frame = [], []
    for _ in range(frames):
        raw = bench.capture(n_words=words)
        prep = prepare_capture(raw, cfg, signature=sig)
        if sig is None:
            sig = prep["signature"]
        batch.append((prep["ch_a"], prep["ch_b"], prep["n0"], prep["rotation"]))
        per_frame.append(estimate_block(prep["ch_a"], prep["ch_b"], cfg,
                                        n0=prep["n0"]))
    joint = estimate_block_joint([(a, b, n0) for a, b, n0, _ in batch], cfg)
    return bench, joint, per_frame


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--frames", type=int, default=6)
    ap.add_argument("--skew-b-ps", type=float, default=3.6)
    ap.add_argument("--hw-bytes", type=int, default=4095,
                    help="frame byte length; 4095 matches the board")
    ap.add_argument("--words", type=int, default=2048,
                    help="16-bit JESD words per frame; board sends 2048 (4096 B)")
    ap.add_argument("--noise", type=float, default=3.0)
    args = ap.parse_args(argv)

    from calibration_loop.dither import DitherConfig

    cfg = DitherConfig()
    cfg.validate()
    print("=== config ===")
    print(f"  slot_period={cfg.slot_period}  n_events={cfg.n_events}  "
          f"f_sig={cfg.f_sig/1e6:.4f} MHz  amp_dbfs={cfg.amp_dbfs}")
    print(f"  frame = {args.words} words = {args.words*2} bytes "
          f"= {args.words//8*4} samples/channel")

    for sk_truth in (args.skew_b_ps, 0.0):
        r = run(cfg, sk_truth, args.frames, args.words, args.noise)
        print(f"\n=== BenchModel ground truth: skew_b = {sk_truth:+.2f} ps, "
              f"gain ratio = {r['truth_gain']:.5f} ===")

        def fin(vals):
            return [v for v in vals if np.isfinite(v)]

        def line(label, vals, fmt="{:9.3f}", extra="", truth=None, terr=False):
            f = fin(vals)
            if not f:
                print(f"  {label:<17}: ALL NaN")
                return
            s = (f"  {label:<17}: mean=" + fmt.format(st.mean(f))
                 + f" std={st.pstdev(f):9.4f} n={len(f)}/{len(vals)}")
            if truth is not None:
                s += f"  (truth {truth:+.2f}, err {st.mean(f)-truth:+.2f})"
            print(s + extra)

        print(f"  events/frame     : {r['ev']}")
        line("align margin", r["mg"], "{:9.3f}")
        line("gainA", r["ga"])
        line("gainB", r["gb"])
        line("gB/gA", r["gm"], "{:9.5f}", truth=r["truth_gain"])
        line("dSkew (B-A) ps", r["dsk"], "{:+11.2f}", truth=sk_truth)
        f = fin(r["sk"])
        if f:
            print(f"  per-channel skew : min={min(f):+10.2f} max={max(f):+10.2f} ps "
                  f"(n={len(f)}/{len(r['sk'])})")
        else:
            print("  per-channel skew : ALL NaN")

    # ---- joint vs per-frame: the whole point of pooling --------------------
    print("\n" + "=" * 70)
    print("JOINT aggregation vs PER-FRAME (same frames, pooled estimate)")
    print("=" * 70)
    for sk_truth in (args.skew_b_ps, 0.0):
        bench, joint, pf = run_joint(cfg, sk_truth, args.frames, args.words,
                                     args.noise)
        truth = bench.gain_b / bench.gain_a
        pf_g = [e.gain_ratio for e in pf if np.isfinite(e.gain_ratio)]
        pf_s = [e.skew_mismatch_ps for e in pf if np.isfinite(e.skew_mismatch_ps)]
        print(f"\ntruth: skew_b={sk_truth:+.2f} ps  gain={truth:.5f}  "
              f"frames={args.frames}")
        if pf_g:
            print(f"  per-frame gB/gA : mean={st.mean(pf_g):+.5f} "
                  f"scatter(std)={st.pstdev(pf_g):.5f}")
            print(f"  JOINT   gB/gA   : {joint.gain_ratio:+.5f}  "
                  f"err={joint.gain_ratio - truth:+.5f}")
        if pf_s:
            print(f"  per-frame skew  : mean={st.mean(pf_s):+11.2f} "
                  f"scatter(std)={st.pstdev(pf_s):9.2f}")
        print(f"  JOINT   skew    : {joint.skew_mismatch_ps:+11.2f} ps  "
              f"err={joint.skew_mismatch_ps - sk_truth:+.2f}  "
              f"[{joint.skew_source}]")
        print(f"  JOINT   events  : {joint.ch_a.n_events_used} per channel "
              f"pooled from {args.frames} frames")

    print("\nInterpretation: if dSkew err is ~0 here, the estimator recovers "
          "known truth and the hardware discrepancy is signal/model mismatch.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
