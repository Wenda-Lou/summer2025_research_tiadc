"""Offline proof for the tone-free (dither-only) closed loop.

The loop's default timing observable is the phase of a reference tone.  A dither-only session
has no tone, and the work of 2026-09-20 was to give it a route that works without one: fit each
channel's folded impulse replica against the known pulse template at a fractional sampling phase
and difference the two phases (``calibration_loop.dither_raw``).  Two things went wrong on the
way and both are checked here, because both are invisible in a "did it converge" summary:

1. **Sign.**  The first version of the route measured the shift of channel B's *index* sequence
   against channel A's, which is the negative of the sampling-instant error the loop integrates.
   The batch error then had the wrong sign: in the model the commanded delay ran 0 -> 390 ps
   while the true residual grew to -371 ps, and the actuator was being walked off the rail one
   acknowledged transaction at a time.  Checked by putting known delays on channel B and
   requiring the route to read them back, sign and scale.
2. **Which observable is judged.**  The acceptance filter gated on the estimator's *tone* fields,
   which on a tone-free capture are a fit of noise: 68 of 84 captures (81 %) were rejected while
   the loop's own route was fine.  Checked by running the loop and counting what the old gate
   would have thrown away.

    python calibration_out/_tone_free_loop_test.py
"""

from __future__ import annotations

import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.dither import DitherConfig                     # noqa: E402
from calibration_loop.dither_raw import measure                      # noqa: E402
from calibration_loop.estimator import prepare_capture               # noqa: E402
from calibration_loop.loop import CalibrationLoop, LoopOptions       # noqa: E402
from calibration_loop.simulate import BenchModel                     # noqa: E402

CFG_KW = dict(amp_dbfs=-120.0, dither_edge_dac=4, dither_top_dac=4, dither_scale_lsb=16000.0)
"""The bench's best dither-only excitation (2026-09-19): 6-sample pulse, 16000 LSB, no tone."""

BENCH_SCALE = 400.0 / 32000.0
"""``BenchModel.adc_scale`` that puts the replica at ~200 codes pk-pk, i.e. bench-like: the
measured bench replica at amp16000 was 391 codes pk-pk with ~1 code of per-event noise."""

TS_PS = 1e12 / 1.3e9


def tone_free_options(**kw) -> LoopOptions:
    kw.setdefault("cancel_signal", False)
    kw.setdefault("gain_observable", "dither_mag")
    kw.setdefault("skew_observable", "dither_phase")
    return LoopOptions(**kw)


def route_truth(delays_ps, frames: int = 20, scale: float = BENCH_SCALE,
                noise: float = 3.0, seed: int = 7):
    """Recover known sampling-instant delays on channel B, from the impulses alone."""
    cfg = DitherConfig(**CFG_KW)
    cfg.validate()
    bench = BenchModel(cfg=cfg, adc_scale=scale, noise_rms_codes=noise, seed=seed)
    out = []
    for d in delays_ps:
        bench.command_skew(d)
        slope, fold, mag, dc, phase_a = [], [], [], [], []
        for _ in range(frames):
            prep = prepare_capture(bench.capture(), cfg)
            m = measure(prep["ch_a"], prep["ch_b"], cfg)
            slope.append(m.skew_slope_ps)
            fold.append(m.skew_fold_ps)
            mag.append(m.gain_mag_ratio)
            dc.append(m.offset_codes)
            phase_a.append(m.phase_a)
        truth = bench.truth()["skew_mismatch_ps"]
        out.append({
            "cmd_ps": d,
            "truth_ps": truth,
            "slope_ps": float(np.nanmean(slope)),
            "slope_sd": float(np.nanstd(slope)),
            "fold_ps": float(np.nanmean(fold)),
            "mag_ratio": float(np.nanmean(mag)),
            "dc_codes": float(np.nanmean(dc)),
            "phase_a": float(np.nanmean(phase_a)),
            "truth_dc": bench.offset_b_codes - bench.offset_a_codes,
        })
    return out


def run_loop(skew_ps: float, iterations: int = 40, tone_free: bool = True,
             cfg: DitherConfig | None = None, route: str = "dither_fold"):
    cfg = cfg or DitherConfig(**CFG_KW)
    cfg.validate()
    bench = BenchModel(cfg=cfg, skew_b_ps=skew_ps,
                       adc_scale=BENCH_SCALE if tone_free else 0.25,
                       noise_rms_codes=3.0)
    opts = tone_free_options(close_skew_loop=True)
    if tone_free:
        opts.skew_observable = route
    loop = CalibrationLoop(bench, cfg, options=opts if tone_free
                           else LoopOptions(close_skew_loop=True))
    loop.run(iterations, verbose=False)
    return loop, bench


def old_gate_rejections(loop) -> tuple[int, int]:
    """What the pre-2026-09-20 acceptance filter would have refused.

    The old filter judged the estimator's own fields: a non-finite per-channel gain/skew, and
    ``skew_mismatch_ps`` inside +-0.25 samples.  On a tone-free capture both are properties of a
    noise fit, not of the frame.  Counted here over the frames the loop *accepted*, which is the
    number that matters: every one of them is a frame the loop would have had to retry.
    """
    rows = [r for r in loop.log if not r.get("rejected")]
    old = 0
    for r in rows:
        bad = (not np.isfinite(r.get("gain_a_codes", np.nan))
               or not np.isfinite(r.get("gain_b_codes", np.nan))
               or not np.isfinite(r.get("skew_a_ps", np.nan)))
        residual = (r.get("skew_mismatch_ps", np.nan)
                    - loop.state.skew_target_ps) * 1e-12 * loop.cfg.fs_adc
        if bad or abs(residual) > 0.25:
            old += 1
    return old, len(rows)


def main() -> int:
    checks: list[tuple[str, bool, str]] = []

    # ---- 1. the route against known delays ---------------------------------
    print("tone-free timing route against known sampling-instant delays (channel B, model truth)")
    print(f"  replica ~{16000 * BENCH_SCALE:.0f} codes pk-pk, 3 codes rms noise, "
          f"20 frames per point")
    print(f"  {'true ps':>9}{'phase route':>13}{'sd':>7}{'fold route':>12}"
          f"{'mag ratio':>11}{'DC(B-A)':>10}{'phase A':>9}")
    cases = route_truth([0.0, 25.0, 50.0, 100.0, 200.0, -100.0])
    for c in cases:
        print(f"  {c['truth_ps']:>9.1f}{c['slope_ps']:>13.1f}{c['slope_sd']:>7.1f}"
              f"{c['fold_ps']:>12.1f}{c['mag_ratio']:>11.4f}{c['dc_codes']:>10.2f}"
              f"{c['phase_a']:>9.3f}")

    errs = [c["slope_ps"] - c["truth_ps"] for c in cases]
    checks.append(("the phase route reads a late channel B as positive",
                   all((c["slope_ps"] > 0) == (c["truth_ps"] > 0) for c in cases
                       if c["truth_ps"] != 0),
                   "signs: " + ", ".join(f"{c['truth_ps']:+.0f}->{c['slope_ps']:+.0f}"
                                         for c in cases)))
    checks.append(("and it is unbiased over the range the loop works in (|err| < 3 ps)",
                   all(abs(e) < 3.0 for e in errs),
                   "errors " + ", ".join(f"{e:+.1f}" for e in errs) + " ps"))
    case50 = [c for c in cases if c["cmd_ps"] == 50.0][0]
    case100 = [c for c in cases if c["cmd_ps"] == 100.0][0]
    checks.append(("it is linear where the fold projection is not (50->100 ps doubles)",
                   abs((case100["slope_ps"] - case50["slope_ps"]) - 50.0) < 4.0,
                   "phase 50 -> 100 ps reads "
                   f"{case50['slope_ps']:.1f} -> {case100['slope_ps']:.1f} ps, "
                   "the fold cross-check reads "
                   f"{case50['fold_ps']:.1f} -> {case100['fold_ps']:.1f} ps"))
    checks.append(("gain and offset carry the right sign and scale too",
                   all(abs(c["mag_ratio"] - 1.021) < 0.005 for c in cases)
                   and all(abs(c["dc_codes"] - c["truth_dc"]) < 1.0 for c in cases),
                   f"mag ratio {cases[0]['mag_ratio']:.4f} against a true 1.021000, "
                   f"DC {cases[0]['dc_codes']:.2f} against {cases[0]['truth_dc']:.2f} codes"))
    checks.append(("the fitted phase is shared by both channels' clock path, not per frame",
                   np.std([c["phase_a"] for c in cases]) < 0.02,
                   f"channel A phase {np.mean([c['phase_a'] for c in cases]):.3f} "
                   f"+- {np.std([c['phase_a'] for c in cases]):.3f} samples across states"))

    # ---- 2. the closed loop, on both tone-free routes ------------------------
    # Both routes have to converge: the fold projection is what --tone-free selects (it is the
    # quieter one on real bench captures) and the fractional-phase fit is the linear alternative,
    # so a change that breaks either one has to fail here rather than on the bench.
    closed = {}
    for route in ("dither_fold", "dither_phase"):
        loop, bench = run_loop(skew_ps=3.6, route=route)
        rows = [r for r in loop.log if not r.get("rejected")]
        tail = rows[-max(1, len(rows) // 4):]

        def avg(key, rows=tail):
            vals = [r[key] for r in rows if np.isfinite(r.get(key, np.nan))]
            return float(np.mean(vals)) if vals else float("nan")

        closed[route] = {
            "loop": loop, "rows": rows, "tail": tail, "avg": avg,
            "gain": avg("gain_mag_ratio"),
            "offset": avg("offset_b_codes") - avg("offset_a_codes"),
            "skew": avg("skew_used_ps"),
        }
        print(f"\ntone-free closed loop, {route} route over the model "
              f"({len(rows)} qualified samples from {loop.captures} captures)")
        print(f"  gain magnitude ratio  {closed[route]['gain']:+.6f}   "
              f"(true mismatch 1.021000)")
        print(f"  offset mismatch       {closed[route]['offset']:+.4f} LSB")
        print(f"  skew ({route})   {closed[route]['skew']:+.4f} ps   (true 3.600 ps)")
        print(f"  commanded delay       {loop.state.skew_cmd_ps:+.2f} ps")
        print(f"  the estimator's tone route on the same frames: "
              f"{avg('skew_mismatch_ps'):+.2f} ps, source "
              f"{tail[-1].get('skew_source')}  (no tone: this is a noise fit)")

        checks.append((f"[{route}] the gain loop nulls the magnitude ratio",
                       abs(closed[route]["gain"] - 1.0) < 0.002,
                       f"{closed[route]['gain']:+.6f}"))
        checks.append((f"[{route}] the offset loop nulls the DC mismatch",
                       abs(closed[route]["offset"]) < 0.5,
                       f"{closed[route]['offset']:+.4f} LSB"))
        checks.append((f"[{route}] the skew loop nulls the timing mismatch",
                       abs(closed[route]["skew"]) < 1.5,
                       f"{closed[route]['skew']:+.4f} ps against a true 3.600 ps"))
        checks.append((f"[{route}] the commanded delay moves B the right way and stays in range",
                       -8.0 < loop.state.skew_cmd_ps < 0.5,
                       f"commanded {loop.state.skew_cmd_ps:+.2f} ps for a +3.6 ps mismatch "
                       "(the model adds the command to B's delay)"))
        used_key = "skew_used_ps"
        source_key = {"dither_fold": "skew_fold_ps", "dither_phase": "skew_slope_ps"}[route]
        checks.append((f"[{route}] every frame reports which observable it integrated",
                       all(r.get("skew_observable") == route
                           and abs(r.get(used_key, np.nan) - r.get(source_key, 0.0)) < 1e-9
                           for r in rows),
                       f"skew_used_ps == {source_key} on all accepted rows"))

    loop = closed["dither_fold"]["loop"]
    rows = closed["dither_fold"]["rows"]
    old, total = old_gate_rejections(loop)
    checks.append(("the old acceptance filter would have thrown these frames away",
                   old / max(total, 1) > 0.2,
                   f"the pre-2026-09-20 gate would have refused {old} of {total} accepted "
                   f"frames ({100 * old / max(total, 1):.0f} %) -- it judged the estimator's "
                   "tone fields, which on a tone-free capture are a noise fit"))
    checks.append(("and the gate that judges the live route keeps them",
                   loop.rejected / max(loop.captures, 1) < 0.1,
                   f"this run rejected {loop.rejected}/{loop.captures} captures "
                   f"({100 * loop.rejected / max(loop.captures, 1):.0f} %)"))

    # ---- 3. a large mismatch must be acquired, not run away ------------------
    far, far_bench = run_loop(skew_ps=200.0, iterations=40)
    far_rows = [r for r in far.log if not r.get("rejected")]
    far_tail = far_rows[-max(1, len(far_rows) // 4):]
    far_res = float(np.mean([r["skew_used_ps"] for r in far_tail]))
    print(f"\nacquisition from a large mismatch (+200 ps, outside one actuator code's range)")
    print(f"  residual after the run  {far_res:+.2f} ps   commanded "
          f"{far.state.skew_cmd_ps:+.1f} ps   abort {far._skew_abort!r}")
    checks.append(("a large mismatch is driven towards zero, not away from it",
                   abs(far_res) < 5.0 and far.state.skew_cmd_ps < 0,
                   f"residual {far_res:+.2f} ps, command {far.state.skew_cmd_ps:+.1f} ps"))
    checks.append(("and the sim run does not latch the actuator",
                   far._skew_abort is None, f"abort={far._skew_abort!r}"))

    # ---- 4. the tone route, for contrast ------------------------------------
    tone_loop, _ = run_loop(skew_ps=3.6, tone_free=False, cfg=DitherConfig())
    tone_rows = [r for r in tone_loop.log if not r.get("rejected")]
    print("\nsame model, tone route, waveform WITH the tone (the default configuration):")
    print(f"  {len(tone_rows)} qualified samples from {tone_loop.captures} captures, "
          f"skew source {tone_rows[-1].get('skew_source')}, "
          f"residual {np.mean([r['skew_used_ps'] for r in tone_rows[-10:]]):+.3f} ps")
    checks.append(("the tone route still works, so the tone-free fix is additive",
                   len(tone_rows) >= 30
                   and abs(np.mean([r["skew_used_ps"] for r in tone_rows[-10:]])) < 2.0,
                   f"residual "
                   f"{np.mean([r['skew_used_ps'] for r in tone_rows[-10:]]):+.3f} ps over "
                   f"{len(tone_rows)} samples"))

    print()
    ok = True
    for name, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name} -- {detail}")
        ok = ok and passed
    print(f"\nTONE-FREE LOOP: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
