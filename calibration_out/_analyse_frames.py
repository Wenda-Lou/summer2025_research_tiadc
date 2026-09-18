"""Scratch: offline analysis of the frames saved by _capture_frames.py.

Answers four questions without touching the bench again:

1. Is the current bench state as clean as the known-good state?  (correlate the
   two channels, compare with a synthetic BenchModel frame.)
2. Which frames are the low-SNDR ones, and does the loop's own metric (dither
   removed) explain them?
3. Is the damage localised -- e.g. one bad 512-byte datagram inside the frame?
4. Does anything else obvious differ (clipping, stuck words, event count, margin)?
"""

from __future__ import annotations

import csv
import glob
import json
import math
import os
import statistics as st
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.dither import DitherConfig                      # noqa: E402
from calibration_loop.estimator import (                              # noqa: E402
    estimate_block, fit_tone, prepare_capture, synthesize_dither, unpack_words,
)
from calibration_loop.metrics import analyse                          # noqa: E402
from calibration_loop.simulate import BenchModel                      # noqa: E402

FRAME_DIR = os.path.join(REPO, "calibration_out", "frames")


def chunk_residuals(words, rotation, f0, cfg, n_chunks=8):
    """Per-datagram residual RMS about each datagram's own tone fit.

    Localised corruption (one bad datagram, torn transfer) shows up as one chunk
    standing out; a uniformly noisy frame leaves all chunks equal.
    """
    out = []
    per = 256  # 512 bytes = 256 words = 32 beats
    for c in range(n_chunks):
        seg = words[c * per:(c + 1) * per]
        if seg.size < 64:
            break
        body = seg[rotation:]
        body = body[:(body.size // 8) * 8]
        if body.size < 32:
            out.append((np.nan, np.nan))
            continue
        g = body.reshape(-1, 8)
        ra = fit_tone(g[:, :4].reshape(-1).astype(float), f0)["residual"]
        rb = fit_tone(g[:, 4:].reshape(-1).astype(float), f0)["residual"]
        out.append((float(np.std(ra)), float(np.std(rb))))
    return out


def corr(x, y):
    xs, ys = x - x.mean(), y - y.mean()
    d = math.sqrt(float(xs @ xs) * float(ys @ ys))
    return float(xs @ ys) / d if d > 0 else 0.0


def analyse_frame(raw, cfg, f0):
    prep = prepare_capture(raw, cfg)
    a, b = prep["ch_a"], prep["ch_b"]
    est = estimate_block(a, b, cfg, n0=prep["n0"])
    fa = fit_tone(a, f0, refine=True)
    fb = fit_tone(b, f0, refine=True)
    # the loop's own metric: subtract the dither it thinks it saw, then score
    da = synthesize_dither(a.size, prep["n0"], cfg, est.ch_a.gain_codes)
    db = synthesize_dither(b.size, prep["n0"], cfg, est.ch_b.gain_codes)
    words = unpack_words(raw)
    chunks = chunk_residuals(words, prep["rotation"], f0, cfg)
    return {
        "n0": prep["n0"], "rot": prep["rotation"], "margin": prep["align_margin"],
        "events": est.ch_a.n_events_used,
        "gain_a": est.ch_a.gain_codes, "gain_b": est.ch_b.gain_codes,
        "ratio": est.gain_ratio,
        "skew": est.skew_mismatch_ps, "src": est.skew_source,
        "corr_ab": corr(a, b),
        "tone_a": fa["amplitude"], "tone_b": fb["amplitude"],
        "resid_a": float(np.std(fa["residual"])), "resid_b": float(np.std(fb["residual"])),
        "sndr_raw_a": analyse(a, cfg.fs_adc)["sndr_db"],
        "sndr_cal_a": analyse(a - da, cfg.fs_adc)["sndr_db"],
        "sndr_cal_b": analyse(b - db, cfg.fs_adc)["sndr_db"],
        "chunks_a": [c[0] for c in chunks], "chunks_b": [c[1] for c in chunks],
        "clip": int(np.sum(np.abs(a) >= 8191)) + int(np.sum(np.abs(b) >= 8191)),
    }


def sim_reference(cfg, f0, frames=3):
    bench = BenchModel(cfg=cfg, noise_rms_codes=3.0, seed=7)
    vals = []
    for _ in range(frames):
        raw = bench.capture(n_words=2048)
        vals.append(analyse_frame(raw, cfg, f0))
    return vals


def main() -> int:
    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    files = sorted(glob.glob(os.path.join(FRAME_DIR, "frame_*.bin")))
    if not files:
        print(f"no frames in {FRAME_DIR}; run _capture_frames.py first")
        return 1

    rows = []
    for path in files:
        with open(path, "rb") as fh:
            rows.append((os.path.basename(path), analyse_frame(fh.read(), cfg, f0)))

    print(f"{'frame':>12}{'margin':>7}{'ev':>4}{'|corr|':>8}{'toneA':>8}{'residA':>8}"
          f"{'SNDRraw':>9}{'SNDRcal':>9}{'g_ratio':>9}{'dSkew':>8}{'clip':>6}")
    for name, r in rows:
        print(f"{name:>12}{r['margin']:>7.1f}{r['events']:>4}{abs(r['corr_ab']):>8.4f}"
              f"{r['tone_a']:>8.1f}{r['resid_a']:>8.2f}{r['sndr_raw_a']:>9.2f}"
              f"{r['sndr_cal_a']:>9.2f}{r['ratio']:>9.5f}"
              f"{r['skew']:>8.1f}{r['clip']:>6}")

    def col(key):
        v = [r[key] for _, r in rows]
        v = [x for x in v if isinstance(x, float) and math.isfinite(x)]
        return (st.mean(v), st.pstdev(v), len(v)) if v else (float("nan"), float("nan"), 0)

    print("\nframe-to-frame (mean +- std, n):")
    for key in ("corr_ab", "tone_a", "resid_a", "sndr_raw_a", "sndr_cal_a",
                "gain_a", "gain_b", "ratio", "skew"):
        m, s, n = col(key)
        print(f"  {key:<12}{m:>10.4f} +-{s:>9.4f}  (n={n})")

    # per-datagram spread, averaged over frames
    ca = np.array([r["chunks_a"] for _, r in rows], dtype=float)
    cb = np.array([r["chunks_b"] for _, r in rows], dtype=float)
    if ca.size:
        print("\nper-512B-datagram residual RMS (mean over frames):")
        print("  chA: " + "  ".join(f"{x:6.2f}" for x in np.nanmean(ca, axis=0)))
        print("  chB: " + "  ".join(f"{x:6.2f}" for x in np.nanmean(cb, axis=0)))
        worst = np.nanmax(ca, axis=1) / np.maximum(np.nanmean(ca, axis=1), 1e-9)
        print(f"  worst-chunk / mean-chunk ratio per frame: "
              + "  ".join(f"{x:.2f}" for x in worst))

    ref = sim_reference(cfg, f0)
    print("\nBenchModel reference (healthy, 3-code noise):")
    for key in ("corr_ab", "tone_a", "resid_a", "sndr_raw_a", "sndr_cal_a", "ratio"):
        v = [r[key] for r in ref]
        print(f"  {key:<12}{st.mean(v):>10.4f}")

    low = [(n, r) for n, r in rows if r["sndr_cal_a"] < st.mean([x["sndr_cal_a"] for _, x in rows]) - 3]
    print(f"\nframes more than 3 dB below the batch mean (loop metric): {len(low)}")
    for n, r in low:
        print(f"  {n}: SNDRcal={r['sndr_cal_a']:.2f} margin={r['margin']:.1f} ev={r['events']}"
              f" residA={r['resid_a']:.2f} gainA={r['gain_a']:.2f} ratio={r['ratio']:.5f}"
              f" worstchunk={max(r['chunks_a']) / np.mean(r['chunks_a']):.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
