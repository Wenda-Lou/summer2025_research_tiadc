"""Which sub-sample timing route to trust, checked against the actuator's known codes.

Two tone-free routes claim to measure the same delay, and they disagree about *how* they do it:

  * ``fold``  -- project the folded A-B difference onto the pulse's derivative.  This is the route
                 every ladder number in ``BENCH_SESSION_DITHER_ONLY.md`` was measured with, and
                 it is the loop's ``--skew-observable dither_fold``.
  * ``phase`` -- fit each channel's folded replica against the template at a fractional sampling
                 phase and difference the phases (the loop's ``dither_phase``, and what
                 ``--tone-free`` selects by default).

Their trade-off is not the same in the model as on the bench, and the bench is what matters:

  * in the bench *model* the phase route wins outright -- linear to better than 2 % over +-400 ps
    with 3-7 ps of per-frame scatter, where the fold projection compresses (86.6 ps read for a
    true 103.6) and scatters 3-4x worse;
  * on the *bench* the analog path reshapes the impulse (measured replica FWHM 3.5 samples
    against an ideal 4.0, and 17.2 against 24 for the 32-sample pulse), and a shape mismatch
    hurts the route that fits the whole template.  Measured on the archived 2026-09-19 captures:
    at the recommended short, large dither (``w06``, 16000 LSB) the fold route scatters 15.3 ps
    per frame against 26.0 for the phase route, and on the 32-sample ladder pulse the phase route
    degrades to ~100 ps (a 15 % template residual drives the fit) where the fold route holds
    46 ps.

**A first live run on 2026-09-20 settled it, and the answer was not the one the noise figures
suggested.**  ``w06`` 16000 LSB, 200 frames at code 24 and 200 at code 32, per-frame scatter 3.1 ps
(fold) and 4.8 ps (phase) -- both far quieter than the archived session, so the bench was in good
shape:

    fold  : +6.46 ps/code   (51.7 ps over the 8-code step)
    phase : +18.19 ps/code  (145.5 ps over the same step)

The fold route is the *quieter* one -- and it is the one with the wrong scale.  The tone route,
measured the next hour on a tone+dither waveform at the same working point, read **+19.5 ps/code**
(one code, so 19.5 ps for the step), agreeing with the phase route to 7 % and disagreeing with the
fold route by 2.8x.  The consequence is not academic: a dither-only run driving the fold route
parked at code 34 reading **-7.5 ps**, i.e. inside its 10 ps deadband, while the tone route
measured **-16.9 ps** there and the raw A-B spur at f_in -- after removing the known 1.5 % gain
term -- implied **17.7 ps**.  A scale-biased route stops with a real residual ~2.8x its own
deadband.  The zero points, by contrast, agreed to 0.3 code throughout (fold 35.2, phase 35.2,
tone 34.9), so the fault is scale, not offset.

That is why this tool's verdict is about **agreement**, not about a fixed ps/code band: the band
this file used to check (4.85-7.4 ps/code, from the 2026-09-17 sessions) was itself measured with
the tone route (`tools/skew_step_characterize.py` takes ``est.skew_phase_ps``) and marked the
*correct* route FAIL.  What it does now:

  * fits each route's ps/code against the known codes and checks the *sign* of every one;
  * when the captures hold a tone, measures the tone-phase route too and uses it as the anchor
    (with a guard: if the fitted tone amplitude is below ``--tone-min-codes`` the captures have no
    tone -- the loaded waveform is not the one whose JSON was passed -- and the column is dropped
    rather than used as an anchor);
  * otherwise requires the two dither routes to agree within 30 %, and says plainly that without a
    tone it cannot tell which of them is wrong;
  * and only then picks the quieter of the routes whose scale is trustworthy, reporting whether a
    20-frame batch is precise enough for the loop's 10 ps deadband.

So the choice is per-waveform and is measured, not assumed.  It is read-only (no register writes)
and runs offline on any archived or fresh pair of states whose actuator codes differ by a known
amount.

The ground truth is the actuator's own characterization: ~4.8-4.9 ps per control code end to end
between codes 24 and 31 (two independent traversals, 2026-09-17), against 7.4 ps/code on a
single-step measurement near neutral -- the step is *not* uniform, so this check reports the
measured ps/code and compares it with both numbers rather than asserting one.

    # the archived stage-C ladder (32-sample pulse)
    python tools/timing_route_check.py \
        --state 24=calibration_out/response/ladder/raw/code24_frame_*.bin \
        --state 25=calibration_out/response/ladder/raw/code25_frame_*.bin \
        --state 32=calibration_out/response/ladder/raw/code32_frame_*.bin \
        --waveform-json waveforms/pulse_ladder/w32adc_e16_t32_amp2000.json

Frames are stride-sampled across each state (``--frames``, default 200), never taken from its
head: every sample-clock write resets the JESD link, so the first captures of a state can sit
inside the re-sync window -- measured on the archived ladder, the head of a state read 25.8 ps
for an 8-code step whose full-state number is 39.5 ps.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.dither import DitherConfig                      # noqa: E402
from calibration_loop.dither_raw import measure                       # noqa: E402
from calibration_loop.estimator import estimate_block, prepare_capture       # noqa: E402

CHARACTERIZED_PS_PER_CODE = 4.85
"""End-to-end step between codes 24 and 31, measured two ways in 2026-09-17 runs."""

SINGLE_STEP_PS_PER_CODE = 7.4
"""Single-step measurement near neutral.  The step is not uniform; both are reported."""


def resolve_state(pattern: str) -> list[str]:
    """Frames for one state, from a directory or a glob.

    A directory is accepted because that is what `dither_response_test.py --out` names and what
    people type; refusing it produced a bare ``PermissionError`` from ``open()`` on the directory
    itself, which is a bad way to learn the calling convention.  A directory holding more than one
    capture tag is refused rather than mixed: states must not be averaged together.
    """
    if os.path.isdir(pattern):
        files = sorted(glob.glob(os.path.join(pattern, "*.bin")))
        if not files:
            raise SystemExit(f"no *.bin frames in directory {pattern!r} -- point at its raw/ "
                             f"subdirectory, e.g. {pattern.rstrip('/')}/raw")
        tags = sorted({os.path.basename(f).split("_frame_")[0] for f in files})
        if len(tags) > 1:
            raise SystemExit(
                f"{pattern!r} holds {len(tags)} capture tags ({', '.join(tags)}) and they would "
                f"be averaged together -- pass one state per code, e.g. "
                f"--state 24={pattern}/{tags[0]}_frame_*.bin")
        return files
    files = [f for f in sorted(glob.glob(pattern)) if os.path.isfile(f)]
    if not files:
        raise SystemExit(f"no frames matched {pattern!r}"
                         + (" (that is a directory -- pass it directly, or its raw/ "
                            "subdirectory)" if os.path.isdir(pattern.rstrip("/*")) else ""))
    return files


def measure_state(files, cfg: DitherConfig, frames: int, min_margin: float,
                  tone_reference: bool = False, tone_prior_ps: float = 0.0):
    """Measure one state from frames spread across it, not from its head.

    The head is where a delay transaction's side effect lives: every sample-clock write resets
    the JESD link, and `dither_response_test.py` starts capturing at the next command, so the
    first frames of a state can sit inside the re-sync window.  Taking `files[:frames]` measured
    exactly those, and on the archived 2026-09-19 ladder it read 25.8 ps for an 8-code step the
    full-state number puts at 39.5 ps.  Stride-sampling removes the question.

    Returns ``(phase, fold, tone)`` -- the last is empty unless a tone reference was asked for.
    """
    if frames >= len(files):
        picked = files
    else:
        idx = np.linspace(0, len(files) - 1, frames).round().astype(int)
        picked = [files[i] for i in sorted(set(idx))]
    phase, fold, tone, amps = [], [], [], []
    for path in picked:
        prep = prepare_capture(open(path, "rb").read(), cfg)
        m = measure(prep["ch_a"], prep["ch_b"], cfg, min_margin=min_margin)
        if np.isfinite(m.skew_slope_ps) and np.isfinite(m.skew_fold_ps):
            phase.append(m.skew_slope_ps)
            fold.append(m.skew_fold_ps)
            if tone_reference:
                # `skew_prior_samples` must be non-zero or the estimator resolves the tone phase's
                # half-period ambiguity (1254 ps here) against the *centroid*, which is documented
                # as +-600 ps of noise -- measured on the archived tone-free ladder, that made the
                # tone column hop by a full half period frame to frame (sd 1246 ps).  A prior of
                # anything inside +-627 ps picks the right branch, and the true residual is tens
                # of ps.
                prior = tone_prior_ps if abs(tone_prior_ps) > 1e-9 else 0.001
                est = estimate_block(prep["ch_a"], prep["ch_b"], cfg, cancel_signal=True,
                                     n0=prep["n0"],
                                     skew_prior_samples=prior * 1e-12 * cfg.fs_adc)
                tone.append(est.skew_phase_ps)
                amps.append(est.ch_a.tone_amplitude)
    return np.asarray(phase), np.asarray(fold), np.asarray(tone), np.asarray(amps)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--state", action="append", required=True, metavar="CODE=PATH",
                    help="actuator control code and the frames captured at it: a directory of "
                         "*_frame_*.bin files, or a glob")
    ap.add_argument("--waveform-json", required=True)
    ap.add_argument("--frames", type=int, default=200,
                    help="frames per state, stride-sampled across the state")
    ap.add_argument("--min-margin", type=float, default=6.0)
    ap.add_argument("--tone-reference", dest="tone_reference", action="store_true", default=None,
                    help="also measure the tone-phase route (needs a tone in the waveform) and "
                         "use it as the scale anchor; default: on whenever the waveform's tone "
                         "is above -60 dBFS")
    ap.add_argument("--no-tone-reference", dest="tone_reference", action="store_false",
                    help="skip the tone column even if the waveform has a tone")
    ap.add_argument("--tone-prior-ps", type=float, default=0.0,
                    help="prior skew for the tone route's half-period branch resolution (any value "
                         "inside +-627 ps works; the residual is tens of ps)")
    ap.add_argument("--tone-min-codes", type=float, default=5.0,
                    help="refuse the tone anchor when the fitted tone amplitude is below this, "
                         "i.e. the loaded waveform has no tone and the column would be a noise fit")
    args = ap.parse_args(argv)

    cfg = DitherConfig(**json.loads(open(args.waveform_json, encoding="utf-8")
                                    .read())["config"])
    cfg.validate()
    if args.tone_reference is None:
        args.tone_reference = cfg.amp_dbfs > -60.0
    if args.tone_reference:
        print(f"tone reference: ON (waveform tone {cfg.amp_dbfs:+.1f} dBFS) -- the tone-phase "
              f"route is shape-independent, so it is the scale anchor here.")

    states = []
    for item in args.state:
        code_s, pattern = item.split("=", 1)
        files = resolve_state(pattern)
        states.append((int(code_s), files))
    states.sort()

    print(f"timing routes against known actuator codes ({args.frames} frames/state, "
          f"waveform {os.path.basename(args.waveform_json)})")
    head = (f"  {'code':>5}{'frames':>8}{'phase ps':>11}{'sd':>7}{'se':>6}"
            f"{'fold ps':>10}{'sd':>7}{'se':>6}")
    if args.tone_reference:
        head += f"{'tone ps':>10}{'sd':>7}{'se':>6}"
    print(head)
    table = []
    anchor_amps = []
    for code, files in states:
        ph, fo, to, amps = measure_state(files, cfg, args.frames, args.min_margin,
                                         tone_reference=args.tone_reference,
                                         tone_prior_ps=args.tone_prior_ps)
        if ph.size < 2:
            print(f"  {code:>5}{0:>8}   no measurable frame")
            continue
        se = lambda v: float(np.std(v, ddof=1) / np.sqrt(v.size))
        line = (f"  {code:>5}{ph.size:>8}{np.mean(ph):>11.1f}{np.std(ph, ddof=1):>7.1f}"
                f"{se(ph):>6.1f}{np.mean(fo):>10.1f}{np.std(fo, ddof=1):>7.1f}{se(fo):>6.1f}")
        tone_mean = tone_se = float("nan")
        if args.tone_reference and to.size >= 2:
            tone_mean, tone_se = float(np.mean(to)), se(to)
            line += f"{tone_mean:>10.1f}{np.std(to, ddof=1):>7.1f}{tone_se:>6.1f}"
            anchor_amps.append(float(np.mean(amps)))
        print(line)
        table.append((code, float(np.mean(ph)), se(ph), float(np.mean(fo)), se(fo),
                      tone_mean, tone_se))

    # A tone column is only an anchor if there is a tone.  The fitted tone amplitude says so: the
    # tone-free ladder files fit ~0.2 codes (noise), a real 555-code tone fits ~555.  Without this
    # guard the "anchor" is a fit of noise and both dither routes get marked wrong against it --
    # measured on the archived tone-free ladder before the guard existed.
    if args.tone_reference and anchor_amps:
        amp = float(np.mean(anchor_amps))
        if amp < args.tone_min_codes:
            print(f"\ntone reference REFUSED: the captures hold no tone (fitted amplitude "
                  f"{amp:.2f} codes < {args.tone_min_codes:.1f}) -- the waveform loaded when they "
                  f"were taken is not the one whose JSON was passed, or it is tone-free.  The "
                  f"column is dropped rather than used as an anchor.")
            args.tone_reference = False
            table = [row[:5] + (float("nan"), float("nan")) for row in table]
        else:
            print(f"\ntone reference: fitted tone amplitude {amp:.1f} codes -- a real tone.")

    if len({row[0] for row in table}) < 2:
        print("\nneed at least two distinct codes to fit a step")
        return 1

    # A weighted fit would be dominated by the smallest error bars and would hide a nonlinear
    # route, so this is a plain least-squares line through the state means: the question is the
    # step's sign and magnitude, not its last decimal.
    codes = np.array([row[0] for row in table], dtype=float)
    fits = {}
    for tag, idx in (("phase", 1), ("fold", 3), ("tone", 5)):
        vals = np.array([row[idx] for row in table], dtype=float)
        if not np.isfinite(vals).all() or vals.size < 2:
            continue
        slope, intercept = np.polyfit(codes, vals, 1)
        resid = vals - (slope * codes + intercept)
        fits[tag] = {"slope": float(slope), "intercept": float(intercept),
                     "rms": float(np.sqrt(np.mean(resid ** 2))),
                     "values": vals}

    print("\nstep per control code.  All routes use the loop's convention -- positive means "
          "channel B samples later -- so a *larger* code must read more positive: the "
          "firmware's `skew step +N` adds N codes of B delay.")
    print(f"  (the 2026-09-17 characterizations -- {CHARACTERIZED_PS_PER_CODE:.2f} end-to-end, "
          f"{SINGLE_STEP_PS_PER_CODE:.2f} single-step -- were both measured with the tone route, "
          f"so they are context, not an independent scale reference)")
    checks: list[tuple[str, bool, str]] = []
    for tag in ("phase", "fold", "tone"):
        if tag not in fits:
            continue
        s = fits[tag]["slope"]
        print(f"  {tag:>6}: {s:+7.2f} ps/code   (spread about the line {fits[tag]['rms']:.2f} ps)")
        checks.append((f"[{tag}] the route recovers a step of the right sign", s > 0,
                       f"{s:+.2f} ps/code"))

    # The verdict is now about *agreement*, not about a fixed band.  Measured 2026-09-20: the two
    # dither routes disagreed by 2.8x on an 8-code step (fold 6.46, phase 18.19 ps/code), the tone
    # route -- measured the next hour at the same working point, on a tone+dither waveform --
    # read 19.5 ps/code for a single code and put the zero crossing within 0.3 code of both dither
    # routes'.  A fixed band anchored on the 2026-09-17 tone-route figures marked the *correct*
    # route FAIL and the scale-biased one PASS, so agreement is what gets checked here.
    anchor = fits.get("tone", {}).get("slope")
    agree = None
    if "phase" in fits and "fold" in fits and fits["fold"]["slope"]:
        agree = fits["phase"]["slope"] / fits["fold"]["slope"]
    for tag in ("phase", "fold"):
        if tag not in fits:
            continue
        s = fits[tag]["slope"]
        if anchor is not None:
            checks.append((
                f"[{tag}] agrees with the tone route's scale (within 30 %)",
                abs(s / anchor - 1.0) < 0.30,
                f"{s:.2f} against the tone route's {anchor:.2f} ps/code "
                f"({100 * (s / anchor - 1):+.0f} %)"))
    if anchor is None and agree is not None:
        checks.append((
            "[phase vs fold] the two dither routes agree on the scale (within 30 %)",
            abs(agree - 1.0) < 0.30,
            f"phase {fits['phase']['slope']:.2f} / fold {fits['fold']['slope']:.2f} = "
            f"{agree:.2f}x -- a disagreement means one of them carries a scale error, and "
            f"without a tone in the waveform this run cannot say which"))

    # Per-frame scatter decides only whether the *correctly scaled* route is precise enough to
    # drive the loop's 10 ps deadband -- and a route whose scale is unresolved is not a candidate
    # however quiet it is (measured 2026-09-20: the quieter route was the one with the 2.8x scale
    # error, so "pick the quiet one" would have picked wrong).
    scatters = {"phase": np.mean([row[2] for row in table]) * np.sqrt(args.frames),
                "fold": np.mean([row[4] for row in table]) * np.sqrt(args.frames)}
    print(f"\nper-frame scatter: phase {scatters['phase']:.1f} ps, fold {scatters['fold']:.1f} ps")
    print(f"  a 20-frame batch (the loop's default) then has {scatters['phase'] / np.sqrt(20):.1f} "
          f"ps / {scatters['fold'] / np.sqrt(20):.1f} ps of standard error against the 10 ps "
          f"deadband")
    if anchor is not None:
        scaled = [t for t in ("phase", "fold")
                  if t in fits and abs(fits[t]["slope"] / anchor - 1) < 0.30]
    elif agree is not None and abs(agree - 1.0) < 0.30:
        scaled = ["phase", "fold"]
    else:
        scaled = []
    if scaled:
        best = min(scaled, key=lambda t: scatters[t])
        print(f"  -> of the routes whose scale is trustworthy, '{best}' is the quieter one: use it")
        checks.append((f"[{best}] the trustworthy route is precise enough for the deadband",
                       scatters[best] / np.sqrt(20) < 3.0,
                       f"{scatters[best] / np.sqrt(20):.1f} ps of batch standard error"))
    else:
        print("  -> no route's scale is trustworthy on this waveform: load a tone+dither "
              "waveform and re-run with it (the tone-phase route is shape-independent and "
              "settles the scale), or accept that the deadband's meaning in ps is unknown")

    print()
    ok = True
    for name, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name} -- {detail}")
        ok = ok and passed
    print(f"\nTIMING ROUTES: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
