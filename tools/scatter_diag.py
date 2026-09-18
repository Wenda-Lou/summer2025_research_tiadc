"""Decompose the gain-ratio scatter: is it noise, or a swap-dependent bias?

The probe's `gain ratio scatter < 0.02` threshold is what fails.  Scatter that is
independent per frame must be beaten down with SNR (more events, longer frames).
Scatter that correlates with `swap` / `rotation` is a *systematic* anchoring
error and is fixable directly.

Prints per frame: swap, rotation, n0, events, gainA, gainB, gB/gA, and the two
channels' residual RMS, then the grouped means so the two explanations separate.

Usage:
    python tools/scatter_diag.py --uart COM5 --frames 16
"""

from __future__ import annotations

import argparse
import statistics as st
import sys

import numpy as np


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--frames", type=int, default=16)
    args = ap.parse_args(argv)

    from calibration_loop.capture import HardwareBench
    from calibration_loop.dither import DitherConfig
    from calibration_loop.estimator import estimate_block, prepare_capture

    cfg = DitherConfig()
    cfg.validate()
    bench = HardwareBench(uart_port=args.uart)

    rows = []
    print(f"{'fr':>3} {'swap':>5} {'rot':>4} {'n0':>6} {'nEv':>4} {'mg':>5} "
          f"{'gainA':>8} {'gainB':>8} {'gB/gA':>8} {'rmsA':>7} {'rmsB':>7}")
    print("-" * 84)
    try:
        sig = None
        for i in range(args.frames):
            raw = bench.capture()
            if raw is None:
                continue
            prep = prepare_capture(raw, cfg, signature=sig)
            if sig is None:
                sig = prep["signature"]
            est = estimate_block(prep["ch_a"], prep["ch_b"], cfg, n0=prep["n0"])
            r = dict(i=i, swap=bool(prep["swapped"]), rot=prep["rotation"],
                     n0=prep["n0"], nev=est.ch_a.n_events_used,
                     mg=prep["align_margin"], ga=est.ch_a.gain_codes,
                     gb=est.ch_b.gain_codes, gr=est.gain_ratio,
                     ra=est.ch_a.residual_rms, rb=est.ch_b.residual_rms,
                     sk=est.skew_mismatch_ps)
            rows.append(r)
            print(f"{i:>3} {str(r['swap']):>5} {r['rot']:>4} {r['n0']:>6} "
                  f"{r['nev']:>4} {r['mg']:>5.1f} {r['ga']:>8.2f} {r['gb']:>8.2f} "
                  f"{r['gr']:>8.4f} {r['ra']:>7.2f} {r['rb']:>7.2f}")
    finally:
        bench.close()

    if not rows:
        print("no frames")
        return 1

    def fin(v):
        return [x for x in v if np.isfinite(x)]

    print("\n=== grouped by swap ===")
    for flag in (False, True):
        g = [r for r in rows if r["swap"] is flag]
        if not g:
            continue
        gr = fin([r["gr"] for r in g])
        print(f"  swap={str(flag):<5} n={len(g):3d}  "
              f"gB/gA mean={st.mean(gr):+.5f} std={st.pstdev(gr):.5f}  "
              f"gainA={st.mean(fin([r['ga'] for r in g])):8.2f}  "
              f"gainB={st.mean(fin([r['gb'] for r in g])):8.2f}  "
              f"rmsB={st.mean(fin([r['rb'] for r in g])):7.2f}")

    gr = fin([r["gr"] for r in rows])
    print(f"\n  overall gB/gA: mean={st.mean(gr):+.5f} std={st.pstdev(gr):.5f} "
          f"(threshold 0.02)")

    # correlation of the ratio with swap, and pure-noise expectation
    sw = np.array([1.0 if r["swap"] else 0.0 for r in rows])
    rr = np.array([r["gr"] for r in rows], dtype=float)
    ok = np.isfinite(rr)
    if ok.sum() > 3 and np.std(sw[ok]) > 0:
        print(f"  corr(gB/gA, swap)      = {np.corrcoef(sw[ok], rr[ok])[0,1]:+.4f}")
        # how much of the variance is explained by swap?
        A = np.column_stack([sw[ok], np.ones(ok.sum())])
        coef, *_ = np.linalg.lstsq(A, rr[ok], rcond=None)
        resid = rr[ok] - A @ coef
        print(f"  swap-explained std     = {abs(coef[0])/2:8.5f}")
        print(f"  residual std after swap= {np.std(resid, ddof=1):8.5f}")

    # per-channel gain scatter vs its residual noise
    for nm, k, rk in (("gainA", "ga", "ra"), ("gainB", "gb", "rb")):
        v = fin([abs(r[k]) for r in rows])
        rms = fin([r[rk] for r in rows])
        print(f"  {nm}: mean={st.mean(v):8.2f} std={st.pstdev(v):7.3f} "
              f"({100*st.pstdev(v)/st.mean(v):5.2f} %)   "
              f"residual rms={st.mean(rms):7.2f}")

    print("\nInterpretation: a large corr(gB/gA, swap) or a big drop in residual")
    print("std after removing the swap term means the scatter is a systematic")
    print("anchoring error (fixable).  Otherwise it is SNR-limited and needs more")
    print("events per capture.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
