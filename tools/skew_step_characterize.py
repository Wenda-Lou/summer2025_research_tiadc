"""Paired characterization of one actuator control code (bench procedure stage 6).

Answers the one question every later conversion depends on: how many picoseconds
does one control code actually move, and with which sign?  Same session, paired,
so drift cancels:

    code 24 (baseline) -> 25 (stepped) -> 24 (restore) -> 25 (stepped) -> 24 (restore)

Each point is a batch of frames measured through the production path; the batch
estimator (`calibration_loop.estimator.skew_batch`) applies the acceptance filter
and the arithmetic mean, because a single frame resolves 0.73 sigma of one code
and the per-frame scatter is +-19 ps.

Safety: the tool only writes when `--execute` is given AND every transaction's ACK
comes back `result=OK`.  A failed transaction aborts the run -- writing again is
never a recovery strategy.  Frame counts default to 40 per point (~80 % detection
power at the 75 % frame yield measured on this bench); use 55 for ~90 %.

    python tools/skew_step_characterize.py --uart COM5 --self-test   # offline
    python tools/skew_step_characterize.py --uart COM5 --frames 40 --execute
"""

from __future__ import annotations

import argparse
import csv
import os
import statistics as st
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

SEQUENCE = (("baseline", 24), ("stepped", 25), ("restore", 24),
            ("stepped", 25), ("restore", 24))


def frame_record(raw, cfg, f0):
    """One frame through the production path, in the shape skew_batch wants."""
    from calibration_loop.estimator import estimate_block, fit_tone, prepare_capture

    prep = prepare_capture(raw, cfg)
    a, b = prep["ch_a"], prep["ch_b"]
    est = estimate_block(a, b, cfg, n0=prep["n0"])
    fit = fit_tone(a, f0, refine=True)
    return {
        "phase_ps": est.skew_phase_ps,
        "diff_route_ps": est.skew_diff_route_ps,
        "centroid_ps": est.skew_centroid_ps,
        "source": est.skew_source,
        "tone": fit["amplitude"],
        "resid": float(np.std(fit["residual"])),
        "margin": prep["align_margin"],
    }


def summarise(points):
    """points: list of dicts with label, code, batch (SkewBatch). Returns verdicts."""
    from calibration_loop.estimator import skew_batch

    base = [p for p in points if p["code"] == 24]
    step = [p for p in points if p["code"] != 24]
    out = {"points": points}

    def pool(sel):
        frames = [f for p in sel for f in p["frames"]]
        return skew_batch(frames), len(frames)

    b_batch, b_n = pool(base)
    s_batch, s_n = pool(step)
    out["baseline"] = b_batch
    out["stepped"] = s_batch
    out["n_baseline_frames"] = b_n
    out["n_stepped_frames"] = s_n

    if b_batch.n_used and s_batch.n_used:
        delta = s_batch.mean_ps - b_batch.mean_ps
        se = float(np.hypot(b_batch.se_ps, s_batch.se_ps))
        out["step_ps"] = delta
        out["step_se_ps"] = se
        out["step_ci95_ps"] = 1.96 * se
        out["step_significant"] = abs(delta) > 2.0 * se
        out["ps_per_code"] = delta  # one code in this sequence
    else:
        out["step_ps"] = float("nan")
        out["step_significant"] = False

    # per-repeat consistency: each stepped point against the baseline point before it
    signs = []
    for i, p in enumerate(points):
        if p["code"] == 24 or i == 0:
            continue
        prev = points[i - 1]
        if prev["batch"].n_used and p["batch"].n_used:
            signs.append(np.sign(p["batch"].mean_ps - prev["batch"].mean_ps))
    out["repeat_signs"] = signs
    out["repeats_agree"] = bool(signs) and all(s == signs[0] for s in signs)

    # restore consistency: every code-24 point against the first one
    first = base[0]["batch"] if base else None
    restore_ok = True
    for p in base[1:]:
        if first is not None and first.n_used and p["batch"].n_used:
            diff = abs(p["batch"].mean_ps - first.mean_ps)
            limit = 2.0 * float(np.hypot(first.se_ps, p["batch"].se_ps))
            if diff > max(limit, 1e-9):
                restore_ok = False
    out["restore_ok"] = restore_ok
    return out


def report(res) -> bool:
    print(f"\n{'point':<10}{'code':>5}{'used':>7}{'mean ps':>10}{'se ps':>8}{'ci95':>8}")
    for p in res["points"]:
        b = p["batch"]
        print(f"{p['label']:<10}{p['code']:>5}{b.n_used:>4}/{b.n_total:<3}"
              f"{b.mean_ps:>10.2f}{b.se_ps:>8.2f}{b.ci95_half_ps:>8.2f}")
    print(f"\nbaseline  mean {res['baseline'].mean_ps:+.2f} ps "
          f"(n={res['baseline'].n_used} accepted of {res['n_baseline_frames']})")
    print(f"stepped   mean {res['stepped'].mean_ps:+.2f} ps "
          f"(n={res['stepped'].n_used} accepted of {res['n_stepped_frames']})")
    print(f"step      {res['step_ps']:+.2f} ps +-{res['step_ci95_ps']:.2f} (95 %)  "
          f"se {res['step_se_ps']:.2f} -> {res['step_ps']:.2f} ps per control code")
    print(f"repeat signs {res['repeat_signs']}  agree={res['repeats_agree']}")
    print(f"restore returns to baseline: {res['restore_ok']}")
    ok = bool(res["step_significant"] and res["repeats_agree"] and res["restore_ok"])
    print(f"\nSTAGE 6 VERDICT: {'PASS' if ok else 'FAIL'}")
    if not ok:
        print("  do not proceed to stage 7; record the table and stop")
    return ok


def self_test(args) -> int:
    """Exercise the verdict logic offline: a real 13.8 ps step must PASS, and a
    zero step must be reported as not significant."""
    import glob as _glob

    from calibration_loop.estimator import skew_batch  # noqa: F401  (used in summarise)
    from calibration_loop.dither import DitherConfig

    cfg = DitherConfig()
    cfg.validate()
    pop = None
    saved = sorted(_glob.glob(os.path.join(REPO, "calibration_out", "frames100",
                                           "frame_*.bin")))
    if saved:
        recs = []
        for path in saved:
            with open(path, "rb") as fh:
                recs.append(frame_record(fh.read(), cfg, cfg.sig_cycles / cfg.n_adc_period))
        pop = np.array([r["phase_ps"] for r in recs], dtype=float)
        pop = pop[np.isfinite(pop)]
        print(f"self-test population: {pop.size} measured frames (sigma "
              f"{pop.std(ddof=1):.2f} ps)")
    else:
        pop = np.random.default_rng(7).normal(-100.0, 19.0, size=100)
        print("self-test population: synthetic (no saved frames found)")

    rng = np.random.default_rng(20260917)

    def fake_points(step_ps):
        pts = []
        for label, code in SEQUENCE:
            shift = 0.0 if code == 24 else step_ps
            vals = pop[rng.integers(0, pop.size, size=args.frames)] + shift
            frames = [{"phase_ps": float(v), "diff_route_ps": float(v),
                       "source": "phase", "tone": 564.0, "resid": 10.6, "margin": 7.0}
                      for v in vals]
            from calibration_loop.estimator import skew_batch
            pts.append({"label": label, "code": code, "frames": frames,
                        "batch": skew_batch(frames)})
        return pts

    real = summarise(fake_points(13.8))
    zero = summarise(fake_points(0.0))
    ok_real = report(real)
    print("\n--- control: a zero step must NOT be reported as real ---")
    zero_sig = zero["step_significant"]
    print(f"zero step: {zero['step_ps']:+.2f} ps, significant={zero_sig} "
          f"-> {'FAIL (false positive)' if zero_sig else 'correctly rejected'}")
    verdict = ok_real and not zero_sig
    print(f"\nSELF-TEST: {'PASS' if verdict else 'FAIL'} "
          f"(real step detected and zero step rejected)")
    return 0 if verdict else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--frames", type=int, default=40, help="frames per point")
    ap.add_argument("--execute", action="store_true",
                    help="actually move the actuator (otherwise measure only)")
    ap.add_argument("--self-test", dest="self_test", action="store_true",
                    help="run the verdict logic offline on saved/synthetic frames")
    ap.add_argument("--out", default=os.path.join(REPO, "calibration_out", "skew_step"))
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test(args)

    from calibration_loop.capture import HardwareBench
    from calibration_loop.dither import DitherConfig
    from calibration_loop.estimator import skew_batch

    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    os.makedirs(args.out, exist_ok=True)

    bench = HardwareBench(uart_port=args.uart, allow_skew_writes=args.execute)
    points = []
    try:
        if args.execute:
            # Learn the current code before anything else: the actuator refuses to
            # move until it has seen one ACK, and a zero-step transaction is also
            # the neutral verification (it must report 24 -> 24).
            txn = bench.actuator.request_steps(0)
            if not txn.get("ok"):
                print(f"neutral verification failed: {txn}")
                print("ABORT: not retrying the write; record and stop")
                return 2
            print(f"neutral verify: code {txn.get('code_before')} -> {txn.get('code_after')}"
                  f"  gen {txn.get('previous_generation')} -> {txn.get('capture_generation')}"
                  f"  neutral_initialized={txn.get('neutral_initialized')}")
            if txn.get("code_after") != 24:
                print(f"ABORT: actuator is at code {txn.get('code_after')}, not the "
                      f"neutral 24 -- the sequence below assumes it starts neutral")
                return 2

        for label, code in SEQUENCE:
            if args.execute:
                txn = bench.actuator.set_code(code)
                if not txn.get("ok"):
                    print(f"transaction failed before '{label}': {txn}")
                    print("ABORT: not retrying the write; record and stop")
                    return 2
                print(f"[{label}] code={txn.get('code_after')} "
                      f"gen={txn.get('capture_generation')} ok")
            else:
                print(f"[{label}] measure-only (no --execute)")
            frames = []
            for _ in range(args.frames):
                raw = bench.capture()
                if raw is None:
                    continue
                frames.append(frame_record(raw, cfg, f0))
            batch = skew_batch(frames)
            print(f"   used {batch.n_used}/{batch.n_total}  mean "
                  f"{batch.mean_ps:+.2f} ps  se {batch.se_ps:.2f}  rejected {batch.rejected}")
            points.append({"label": label, "code": code, "frames": frames,
                           "batch": batch})
    finally:
        bench.close()

    res = summarise(points)
    ok = report(res)

    path = os.path.join(args.out, "skew_step_points.csv")
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["point", "code", "frame", "phase_ps", "diff_route_ps",
                    "centroid_ps", "source", "tone", "resid", "margin"])
        for i, p in enumerate(points):
            for j, f in enumerate(p["frames"]):
                w.writerow([p["label"], p["code"], j, f["phase_ps"], f["diff_route_ps"],
                            f["centroid_ps"], f["source"], f["tone"], f["resid"],
                            f["margin"]])
    print(f"\nper-frame data: {path}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
