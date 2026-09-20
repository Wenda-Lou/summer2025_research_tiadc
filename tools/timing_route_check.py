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

**A first live run on 2026-09-20 settled it, and the answer was not the one the model predicted.**
``w06`` 16000 LSB, 200 frames at code 24 and 200 at code 32, per-frame scatter 3.1 ps (fold) and
4.8 ps (phase) -- both far quieter than the archived session, so the bench was in good shape:

    fold  : +6.46 ps/code   (51.7 ps over the 8-code step)
    phase : +18.19 ps/code  (145.5 ps over the same step)

The fold route agrees with the actuator's single-step figure (7.4 ps/code) and with the register
arithmetic (four 1.725 ps fine steps = 6.9 ps/code).  The phase route reads ~3x that, with a
per-frame scatter of only 4.8 ps -- a *systematic* scale error, not noise, and it is the failure
this check exists to catch.  The likely mechanism is the template mismatch: the bench's replica is
not the ideal raised cosine, so the fitted phase is partly driven by the replica's *shape*, and
the actuator code itself moves the amplitude readout (+0.21 %/code, documented) -- over 8 codes
that is a 1.7 % amplitude change for an ill-conditioned fit to convert into apparent timing.  In
the model both effects are absent, which is why the model cannot see this.

So: **the fold route drives a tone-free loop on this bench**, and the phase route is a cross-check
whose absolute scale must be verified per waveform before it is trusted for anything.

So the choice is per-waveform and has to be measured, not assumed -- which is what this tool
does.  It is read-only (no register writes) and runs offline on any archived or fresh pair of
states whose actuator codes differ by a known amount.

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
from calibration_loop.estimator import prepare_capture                # noqa: E402

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


def measure_state(files, cfg: DitherConfig, frames: int, min_margin: float):
    """Measure one state from frames spread across it, not from its head.

    The head is where a delay transaction's side effect lives: every sample-clock write resets
    the JESD link, and `dither_response_test.py` starts capturing at the next command, so the
    first frames of a state can sit inside the re-sync window.  Taking `files[:frames]` measured
    exactly those, and on the archived 2026-09-19 ladder it read 25.8 ps for an 8-code step the
    full-state number puts at 39.5 ps.  Stride-sampling removes the question.
    """
    if frames >= len(files):
        picked = files
    else:
        idx = np.linspace(0, len(files) - 1, frames).round().astype(int)
        picked = [files[i] for i in sorted(set(idx))]
    phase, fold = [], []
    for path in picked:
        prep = prepare_capture(open(path, "rb").read(), cfg)
        m = measure(prep["ch_a"], prep["ch_b"], cfg, min_margin=min_margin)
        if np.isfinite(m.skew_slope_ps) and np.isfinite(m.skew_fold_ps):
            phase.append(m.skew_slope_ps)
            fold.append(m.skew_fold_ps)
    return np.asarray(phase), np.asarray(fold)


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
    ap.add_argument("--step-ps", type=float, default=CHARACTERIZED_PS_PER_CODE,
                    help="expected ps per control code (default: the end-to-end figure)")
    args = ap.parse_args(argv)

    cfg = DitherConfig(**json.loads(open(args.waveform_json, encoding="utf-8")
                                    .read())["config"])
    cfg.validate()

    states = []
    for item in args.state:
        code_s, pattern = item.split("=", 1)
        files = resolve_state(pattern)
        states.append((int(code_s), files))
    states.sort()

    print(f"timing routes against known actuator codes ({args.frames} frames/state, "
          f"waveform {os.path.basename(args.waveform_json)})")
    print(f"  {'code':>5}{'frames':>8}{'phase ps':>11}{'sd':>7}{'se':>6}"
          f"{'fold ps':>10}{'sd':>7}{'se':>6}")
    table = []
    for code, files in states:
        ph, fo = measure_state(files, cfg, args.frames, args.min_margin)
        if ph.size < 2:
            print(f"  {code:>5}{0:>8}   no measurable frame")
            continue
        se = lambda v: float(np.std(v, ddof=1) / np.sqrt(v.size))
        print(f"  {code:>5}{ph.size:>8}{np.mean(ph):>11.1f}{np.std(ph, ddof=1):>7.1f}"
              f"{se(ph):>6.1f}{np.mean(fo):>10.1f}{np.std(fo, ddof=1):>7.1f}{se(fo):>6.1f}")
        table.append((code, float(np.mean(ph)), se(ph), float(np.mean(fo)), se(fo)))

    if len({c for c, *_ in table}) < 2:
        print("\nneed at least two distinct codes to fit a step")
        return 1

    # A weighted fit would be dominated by the smallest error bars and would hide a nonlinear
    # route, so this is a plain least-squares line through the state means: the question is the
    # step's sign and magnitude, not its last decimal.
    codes = np.array([row[0] for row in table], dtype=float)
    fits = {}
    for tag, idx in (("phase", 1), ("fold", 3)):
        vals = np.array([row[idx] for row in table], dtype=float)
        slope, intercept = np.polyfit(codes, vals, 1)
        resid = vals - (slope * codes + intercept)
        fits[tag] = {"slope": float(slope), "intercept": float(intercept),
                     "rms": float(np.sqrt(np.mean(resid ** 2))),
                     "values": vals}

    print("\nstep per control code.  Both routes use the loop's convention -- positive means "
          "channel B samples later -- so a *larger* code must read more positive: the "
          "firmware's `skew step +N` adds N codes of B delay.")
    checks: list[tuple[str, bool, str]] = []
    for tag in ("fold", "phase"):
        s = fits[tag]["slope"]
        print(f"  {tag:>6}: {s:+7.2f} ps/code   (spread about the line {fits[tag]['rms']:.2f} ps)")
        checks.append((
            f"[{tag}] the route recovers a step of the right sign",
            s > 0,
            f"{s:+.2f} ps/code"))
        # The step is not uniform -- 4.8-4.9 ps/code end to end against 7.4 near neutral -- so
        # either figure is acceptable and anything in between is too.
        lo = min(args.step_ps, SINGLE_STEP_PS_PER_CODE) / 1.6
        hi = max(args.step_ps, SINGLE_STEP_PS_PER_CODE) * 1.6
        checks.append((
            f"[{tag}] and lands between the two characterized steps "
            f"({lo:.1f}-{hi:.1f} ps/code)",
            lo < s < hi,
            f"{s:.2f} ps/code against {args.step_ps:.2f} end-to-end / "
            f"{SINGLE_STEP_PS_PER_CODE:.2f} single-step"))

    # Which route should drive the loop here: the one with the smaller per-frame scatter, since
    # the loop's deadband and direction tolerance are in ps and its step is in codes.
    sd_phase = np.mean([row[2] for row in table]) * np.sqrt(args.frames)
    sd_fold = np.mean([row[4] for row in table]) * np.sqrt(args.frames)
    print(f"\nper-frame scatter: phase {sd_phase:.1f} ps, fold {sd_fold:.1f} ps")
    print("  one single-step code (7.4 ps) is "
          f"{7.4 / sd_phase:.2f} sigma/frame on the phase route, "
          f"{7.4 / sd_fold:.2f} on the fold route")
    print(f"  a {20}-frame batch (the loop's default) then has "
          f"{sd_phase / np.sqrt(20):.1f} ps / {sd_fold / np.sqrt(20):.1f} ps of standard error "
          f"against the 10 ps deadband")
    quieter = "fold" if sd_fold < sd_phase else "phase"
    print(f"  -> the quieter route on this waveform is '{quieter}'; drive the loop with it unless "
          f"its ps/code above shows the input scale is wrong")

    print()
    ok = True
    for name, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name} -- {detail}")
        ok = ok and passed
    print(f"\nTIMING ROUTES: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
