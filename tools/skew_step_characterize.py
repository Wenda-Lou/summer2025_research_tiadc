"""Paired characterization of one actuator control code (bench procedure stage 6).

Answers the one question every later conversion depends on: how many picoseconds
does one control code actually move, and with which sign?  Same session, paired,
so drift cancels:

    code N (baseline) -> N+1 (stepped) -> N (restore) -> N+1 (stepped) -> N (restore)

``N`` defaults to 24 (the neutral code) and is set with ``--code``.  **Characterize the code
you actually work at.**  Measured 2026-09-20: a tone-mode cross-check at the code a dither-only
run had parked on moved one control code and the tone-phase residual went -16.90 -> +2.63 ps,
i.e. **19.5 ps per code** there, against the 4.8-7.4 ps/code this bench was characterized at near
neutral -- and the paired characterization then measured **+19.08 +- 0.06 ps** for that same code
pair, so `SkewActuator.HOST_STEP_PS` (now 19.1) is the working-point value while the neutral
region really does move ~7 ps per code.  The actuator's step is not uniform; a figure from one
code says nothing about another.

Each point is a batch of frames measured through the production path; the batch
estimator (`calibration_loop.estimator.skew_batch`) applies the acceptance filter
and the arithmetic mean, because a single frame resolves 0.73 sigma of one code
and the per-frame scatter is +-19 ps.  The timing comes from the tone-phase route,
which needs a tone in the loaded waveform (load ``waveforms/impulse_dither.txt``).

Safety: the tool only writes when `--execute` is given AND every transaction's ACK
comes back `result=OK`.  A failed transaction aborts the run -- writing again is
never a recovery strategy.  Frame counts default to 40 per point (~80 % detection
power at the 75 % frame yield measured on this bench); use 55 for ~90 %, and note
that the batch mean's standard error is what decides a 7 ps step, not the frames'
per-frame scatter alone.

    python tools/skew_step_characterize.py --self-test            # offline
    python tools/skew_step_characterize.py --uart COM5 --frames 40 --execute
    python tools/skew_step_characterize.py --uart COM5 --code 34 --frames 40 --execute
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

STEP_PATTERN = (("baseline", 0), ("stepped", +1), ("restore", 0),
                ("stepped", +1), ("restore", 0))


def sequence(base: int):
    """The paired point list for one baseline code."""
    return tuple((label, base + offset) for label, offset in STEP_PATTERN)


def frame_record(raw, cfg, f0, route: str = "tone"):
    """One frame through the production path, in the shape the batch filters want.

    Every route is measured from the same capture: the tone-phase route needs a tone in the
    waveform, the two dither routes do not.  Measured 2026-09-20 on the bench, running this with
    the tone-free waveform loaded cost ten minutes and five JESD-resetting transactions for a
    verdict that could not exist -- so the route is now chosen up front and checked, not assumed.
    """
    from calibration_loop.dither_raw import measure as measure_dither
    from calibration_loop.estimator import estimate_block, fit_tone, prepare_capture

    prep = prepare_capture(raw, cfg)
    a, b = prep["ch_a"], prep["ch_b"]
    rec = {"margin": prep["align_margin"], "route": route}
    dith = measure_dither(a, b, cfg, min_margin=0.0)
    rec["dither_phase_ps"] = dith.skew_slope_ps
    rec["dither_fold_ps"] = dith.skew_fold_ps
    if route == "tone":
        est = estimate_block(a, b, cfg, cancel_signal=True, n0=prep["n0"],
                             skew_prior_samples=max(_TONEPRIOR_PS, 1e-3) * 1e-12 * cfg.fs_adc)
        fit = fit_tone(a, f0, refine=True)
        rec.update({
            "phase_ps": est.skew_phase_ps,
            "diff_route_ps": est.skew_diff_route_ps,
            "centroid_ps": est.skew_centroid_ps,
            "source": est.skew_source,
            "tone": fit["amplitude"],
            "resid": float(np.std(fit["residual"])),
        })
    else:
        rec.update({"phase_ps": dith.skew_slope_ps, "tone": float("nan"), "resid": float("nan")})
    return rec


_TONEPRIOR_PS = 0.0
"""Prior for the tone route's half-period branch; set from --tone-prior-ps."""


def tone_amplitude(bench, cfg, f0, frames: int = 6):
    """Mean fitted tone amplitude over a few frames -- 0.2 codes means there is no tone."""
    from calibration_loop.estimator import fit_tone, prepare_capture

    amps = []
    for _ in range(frames):
        raw = bench.capture()
        if raw is None:
            continue
        prep = prepare_capture(raw, cfg)
        amps.append(float(fit_tone(prep["ch_a"], f0, refine=True)["amplitude"]))
    return float(np.mean(amps)) if amps else float("nan")


def batch_of(frames, route: str, margin_min: float = 6.0):
    """Batch the frames with the filter that belongs to the route.

    Both filters take the same alignment-margin floor (the loop's 6.0), so a frame the closed loop
    would refuse never enters a step measurement either.
    """
    from calibration_loop.estimator import skew_batch, skew_batch_dither

    if route == "tone":
        return skew_batch(frames, margin_min=margin_min)
    return skew_batch_dither([{"used_ps": f.get(f"{route}_ps"), "margin": f.get("margin")}
                              for f in frames], margin_min=margin_min)


def goto(bench, target: int, label: str, max_moves: int = 60) -> dict:
    """Drive the actuator to ``target``, one ACK-checked code per transaction.

    ``SkewActuator.set_code`` moves at most one code per call and returns the ACK, so an absolute
    position is a loop on the ACK -- the same discipline `tools/skew_park.py` uses.  Every move
    resets the JESD link, so the caller sees how many transactions a base code costs.
    """
    moves = 0
    txn = bench.actuator.set_code(target)
    while txn.get("ok") and not txn.get("skipped") and txn.get("code_after") != target:
        moves += 1
        if moves >= max_moves:
            return {"ok": False, "stage": "did-not-converge", "code_after": txn.get("code_after")}
        txn = bench.actuator.set_code(target)
    if txn.get("ok"):
        moves += 0 if txn.get("skipped") else 1
    if moves:
        print(f"[{label}] drove to code {txn.get('code_after')} in {moves} "
              f"ACK-checked transaction(s)")
    return txn

def summarise(points, base_code: int = 24):
    """points: list of dicts with label, code, batch (SkewBatch). Returns verdicts."""
    from calibration_loop.estimator import skew_batch

    base = [p for p in points if p["code"] == base_code]
    step = [p for p in points if p["code"] != base_code]
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
        if p["code"] == base_code or i == 0:
            continue
        prev = points[i - 1]
        if prev["batch"].n_used and p["batch"].n_used:
            signs.append(np.sign(p["batch"].mean_ps - prev["batch"].mean_ps))
    out["repeat_signs"] = signs
    out["repeats_agree"] = bool(signs) and all(s == signs[0] for s in signs)

    # restore consistency: every baseline point against the first one
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
    # A real measured population beats a synthetic one for this verdict test, and the
    # neutral-state captures are the documented ones (they are also the "before" half of
    # the professor data package); frames100 was the older pre-initialization set.
    saved = []
    for name in ("frames_neutral", "frames100"):
        saved = sorted(_glob.glob(os.path.join(REPO, "calibration_out", name,
                                              "frame_*.bin")))
        if saved:
            break
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

    def fake_points(step_ps, base=24):
        pts = []
        for label, code in sequence(base):
            shift = 0.0 if code == base else step_ps
            vals = pop[rng.integers(0, pop.size, size=args.frames)] + shift
            frames = [{"phase_ps": float(v), "diff_route_ps": float(v),
                       "source": "phase", "tone": 564.0, "resid": 10.6, "margin": 7.0}
                      for v in vals]
            from calibration_loop.estimator import skew_batch
            pts.append({"label": label, "code": code, "frames": frames,
                        "batch": skew_batch(frames)})
        return pts

    real = summarise(fake_points(13.8), 24)
    zero = summarise(fake_points(0.0), 24)
    ok_real = report(real)
    print("\n--- control: a zero step must NOT be reported as real ---")
    zero_sig = zero["step_significant"]
    print(f"zero step: {zero['step_ps']:+.2f} ps, significant={zero_sig} "
          f"-> {'FAIL (false positive)' if zero_sig else 'correctly rejected'}")
    # A non-neutral base code must be handled by the same logic (the field that broke when the
    # sequence was hardcoded was the `code == 24` split).
    shifted = summarise(fake_points(13.8, base=34), 34)
    shifted_ok = (abs(shifted["step_ps"] - 13.8) < 3.0
                  and shifted["repeats_agree"] and shifted["restore_ok"])
    print(f"\n--- base code 34: step {shifted['step_ps']:+.2f} ps, "
          f"repeats_agree={shifted['repeats_agree']}, restore_ok={shifted['restore_ok']} ---")
    verdict = ok_real and not zero_sig and shifted_ok
    print(f"\nSELF-TEST: {'PASS' if verdict else 'FAIL'} "
          f"(real step detected, zero step rejected, base code 34 handled)")
    return 0 if verdict else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--frames", type=int, default=40, help="frames per point")
    ap.add_argument("--code", type=int, default=24,
                    help="baseline control code to characterize (the paired sequence steps "
                         "+1 from it); measure the code you work at, because the step is not "
                         "uniform -- 19.5 ps/code at code 34/35 against 4.8-7.4 near neutral")
    ap.add_argument("--route", choices=("auto", "tone", "dither_phase", "dither_fold"),
                    default="auto",
                    help="timing route for the measurement.  'auto' probes the captures for a "
                         "tone and uses the tone-phase route when one is there, else "
                         "dither_phase (whose scale was cross-checked against the tone route, "
                         "18.2 against 19.5 ps/code, and against a closed loop's own 11-move "
                         "history, 17.87).  The tone route needs the tone+dither waveform "
                         "(waveforms/impulse_dither.txt); the dither routes work on the tone-free "
                         "ladder waveforms.")
    ap.add_argument("--tone-min-codes", type=float, default=5.0,
                    help="a fitted tone amplitude below this means the captures have no tone")
    ap.add_argument("--tone-prior-ps", type=float, default=0.0,
                    help="prior skew for the tone route's half-period branch resolution")
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

    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period
    os.makedirs(args.out, exist_ok=True)
    points_plan = sequence(args.code)

    bench = HardwareBench(uart_port=args.uart, allow_skew_writes=args.execute)
    points = []
    try:
        # Route first: a measurement on the wrong route is not a weak result, it is no result.
        route = args.route
        if route == "auto":
            amp = tone_amplitude(bench, cfg, f0)
            route = "tone" if amp >= args.tone_min_codes else "dither_phase"
            print(f"route probe: fitted tone amplitude {amp:.2f} codes -> "
                  f"{'tone route (a real tone)' if route == 'tone' else 'no tone: using dither_phase'}")
            if route == "dither_phase":
                print("  (the tone-free path.  Its scale is anchored by the 2026-09-20 cross-check "
                      "-- phase +18.2 vs tone +19.5 ps/code, and a closed loop's own 11-move "
                      "history at 17.87 -- not by an independent tone reference in this run.)")
        elif route == "tone":
            amp = tone_amplitude(bench, cfg, f0)
            if amp < args.tone_min_codes:
                print(f"ABORT: --route tone asked for, but the captures hold no tone (fitted "
                      f"amplitude {amp:.2f} codes).  Load waveforms/impulse_dither.txt (the "
                      f"tone+dither waveform), or let --route auto pick dither_phase.  Nothing "
                      f"was captured beyond the probe and no register was written.")
                return 2
        global _TONEPRIOR_PS
        _TONEPRIOR_PS = args.tone_prior_ps
        print(f"timing route: {route}")

        if args.execute:
            # Learn the current code before anything else: the actuator refuses to
            # move until it has seen one ACK, and a zero-step transaction is also
            # informative (neutral_initialized reports the action taken, not the state).
            txn = bench.actuator.request_steps(0)
            if not txn.get("ok"):
                print(f"zero-step verification failed: {txn}")
                print("ABORT: not retrying the write; record and stop")
                return 2
            print(f"current code {txn.get('code_before')} -> {txn.get('code_after')}"
                  f"  gen {txn.get('previous_generation')} -> {txn.get('capture_generation')}"
                  f"  neutral_initialized={txn.get('neutral_initialized')}")
            print(f"paired sequence about code {args.code}: "
                  + " -> ".join(f"{lbl}:{code}" for lbl, code in points_plan))

        for label, code in points_plan:
            if args.execute:
                txn = goto(bench, code, label)
                if not txn.get("ok") or txn.get("code_after") != code:
                    print(f"transaction failed before '{label}': {txn}")
                    print("ABORT: not retrying the write; record and stop")
                    return 2
                print(f"[{label}] code={txn.get('code_after')} "
                      f"gen={txn.get('capture_generation')} ok")
            else:
                print(f"[{label}] measure-only (no --execute); assuming code {code}")
            frames = []
            for _ in range(args.frames):
                raw = bench.capture()
                if raw is None:
                    continue
                frames.append(frame_record(raw, cfg, f0, route=route))
            batch = batch_of(frames, route)
            print(f"   used {batch.n_used}/{batch.n_total}  mean "
                  f"{batch.mean_ps:+.2f} ps  se {batch.se_ps:.2f}  rejected {batch.rejected}")
            points.append({"label": label, "code": code, "frames": frames,
                           "batch": batch})
    finally:
        bench.close()

    res = summarise(points, args.code)
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
