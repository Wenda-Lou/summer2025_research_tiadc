"""Run the pre-fix and current prepare_capture on the SAME live frames.

The de-frame change is the only difference -- both variants are graded with the
current estimate_block, so any difference in the numbers comes from the front
end alone.  This is the experiment that settles which of the two front ends
produces a stable channel identity, an aligned pair, and a physical gain ratio
on real hardware.

The pre-fix implementation is imported from ``git show HEAD:...`` and is never
run against the board itself: it only sees the bytes the current code captured.

Usage:
    python tools/live_before_after.py --uart COM5 --frames 8
"""

from __future__ import annotations

import argparse
import importlib.util
import os
import subprocess
import sys
import tempfile

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_old_estimator():
    try:
        src = subprocess.run(
            ["git", "-C", REPO, "show", "HEAD:calibration_loop/estimator.py"],
            capture_output=True, check=True,
        ).stdout
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        print(f"pre-fix estimator unavailable: {exc}")
        return None
    tmp = os.path.join(tempfile.mkdtemp(prefix="old_estimator_"), "estimator.py")
    with open(tmp, "wb") as fh:
        fh.write(src)
    spec = importlib.util.spec_from_file_location("calibration_loop._old_estimator", tmp)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["calibration_loop._old_estimator"] = mod
    spec.loader.exec_module(mod)
    return mod


def row(tag, prep, cfg, f0, est_mod, align_mod):
    """Metrics for one front end's output, measured with the current estimator."""
    a, b = prep["ch_a"], prep["ch_b"]
    corr = float(np.corrcoef(a - a.mean(), b - b.mean())[0, 1]) if a.size == b.size else np.nan
    estim = est_mod.estimate_block(a, b, cfg, n0=prep["n0"])
    al_a = align_mod.align_to_loop(align_mod.fit_tone(a, f0, refine=True)["residual"], cfg)
    al_b = align_mod.align_to_loop(align_mod.fit_tone(b, f0, refine=True)["residual"], cfg)
    return (f"  {tag:<16} rot={prep['rotation']:<2} "
            f"corr={corr:+.4f}  alignSign A/B={al_a['sign']:+.0f}/{al_b['sign']:+.0f}  "
            f"gainA={estim.ch_a.gain_codes:+8.2f} gainB={estim.ch_b.gain_codes:+8.2f}  "
            f"gB/gA={estim.gain_ratio:+8.5f}  offA={estim.ch_a.offset_codes:+7.2f} "
            f"offB={estim.ch_b.offset_codes:+7.2f}  nEv={estim.ch_a.n_events_used}/{estim.ch_b.n_events_used}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--frames", type=int, default=8)
    ap.add_argument("--bind-ip", dest="bind_ip", default=None)
    args = ap.parse_args(argv)

    sys.path.insert(0, REPO)
    from calibration_loop.capture import HardwareBench
    from calibration_loop.dither import DitherConfig
    from calibration_loop import estimator as new

    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    old = load_old_estimator()
    if old is None:
        return 1

    bench_kwargs = {"uart_port": args.uart}
    if args.bind_ip:
        bench_kwargs["bind_ip"] = args.bind_ip
    bench = HardwareBench(**bench_kwargs)
    ratios_old, ratios_new, corrs_old, corrs_new = [], [], [], []
    print(f"Live frames from {args.uart}: pre-fix vs current front end, "
          f"identical bytes, identical estimator ({args.frames} frames)\n")
    try:
        for i in range(args.frames):
            raw = bench.capture()
            if raw is None:
                print(f"  frame {i}: capture failed")
                continue
            print(f"-- frame {i}  ({len(raw)} bytes) --")
            prep_old = old.prepare_capture(raw, cfg)
            prep_new = new.prepare_capture(raw, cfg)
            print(row("PRE-FIX (HEAD)", prep_old, cfg, f0, new, new))
            print(row("CURRENT (fixed)", prep_new, cfg, f0, new, new))

            for prep, ratios, corrs in ((prep_old, ratios_old, corrs_old),
                                        (prep_new, ratios_new, corrs_new)):
                a, b = prep["ch_a"], prep["ch_b"]
                corrs.append(float(np.corrcoef(a - a.mean(), b - b.mean())[0, 1]))
                ratios.append(new.estimate_block(a, b, cfg, n0=prep["n0"]).gain_ratio)
    finally:
        bench.close()

    print("\n" + "=" * 100)

    def agg(vals, label):
        v = [x for x in vals if np.isfinite(x)]
        if not v:
            print(f"  {label:<22} all NaN")
            return
        print(f"  {label:<22} mean={np.mean(v):+.5f}  std={np.std(v):.5f}  "
              f"min={min(v):+.5f}  max={max(v):+.5f}  n={len(v)}/{len(vals)}")

    agg(corrs_old, "PRE-FIX |corr(A,B)|")
    agg(corrs_new, "CURRENT |corr(A,B)|")
    agg(ratios_old, "PRE-FIX gB/gA")
    agg(ratios_new, "CURRENT gB/gA")
    print("\n  |corr(A,B)| ~ 1 means the two streams share their instants;")
    print("  ~0.756 means they are four samples apart (the half-group split).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
