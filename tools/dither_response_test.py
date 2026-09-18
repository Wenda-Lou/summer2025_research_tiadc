"""The professor's experiment: can the dither estimates recover KNOWN changes?

Two stages, both designed so the answer is a number rather than an opinion:

  stage 1  repeated captures at one fixed actuator state: how repeatable is each estimate
           frame to frame, with the raw A/B waveform saved for every capture;
  stage 2  known timing steps (the delay actuator, ACK-verified, one code at a time, with
           frames at each code): does each route *recover* the known change, and how much
           averaging does that need?

Why this is the right test for a calibration loop: an estimate with a constant scale or
offset error can still close a loop, whereas one whose noise exceeds the change it is meant
to detect cannot.  Offline versions of both halves already exist on archived data
(`calibration_out/_dither_response_test.py`): injected gain changes are recovered with slope
1.0000 by both routes, but the dither route carries a +1.16 % offset and 8x the noise, and
for one actuator code the tone-phase route sees 2.55 sigma per frame against 0.115 sigma for
the dither/pulse-centroid route.  This script reproduces that on the bench and adds the raw
waveform archive.

Read-only by default: no actuator writes unless ``--allow-skew-writes`` is given, and then
every move goes through the firmware transaction and requires its ACK.

    python tools/dither_response_test.py --uart COM5 --frames 40
    python tools/dither_response_test.py --uart COM5 --frames 40 --steps -1,-2,+3 \
        --allow-skew-writes
"""

from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

ROUTES = ("phase_ps", "diff_route_ps", "centroid_ps", "margin", "dither_offset",
          "dither_gain", "tone_gain", "tone_amp", "resid")


def measure(bench, cfg, n_frames: int, frames_dir, tag: str, polarity):
    """Capture n_frames, save the raw bytes, return (records, polarity)."""
    from calibration_loop.estimator import estimate_block, polarity_anchor, prepare_capture

    recs = []
    for i in range(n_frames):
        raw = bench.capture()
        if not raw:
            continue
        if frames_dir:
            with open(os.path.join(frames_dir, f"{tag}_frame_{i:03d}.bin"), "wb") as fh:
                fh.write(raw)
        prep = prepare_capture(raw, cfg)
        if polarity is None:
            # the session constant, measured once from channel A (see the loop)
            polarity = polarity_anchor(prep)
        est = estimate_block(prep["ch_a"], prep["ch_b"], cfg, cancel_signal=True,
                             n0=prep["n0"], polarity_sign=polarity, pin_polarity=True)
        recs.append({
            "phase_ps": est.skew_phase_ps,
            "diff_route_ps": est.skew_diff_route_ps,
            "centroid_ps": est.skew_centroid_ps,
            "margin": prep["align_margin"],
            "dither_offset": est.offset_mismatch_codes,
            "dither_gain": est.gain_ratio,
            "tone_gain": est.tone_ratio,
            "tone_amp": est.ch_a.tone_amplitude,
            "resid": est.ch_a.residual_rms,
        })
    return recs, polarity


def summarize(recs) -> dict:
    out = {}
    for key in ROUTES:
        v = np.array([r[key] for r in recs], dtype=float)
        v = v[np.isfinite(v)]
        out[key] = (float(v.mean()), float(v.std(ddof=1)), int(v.size)) if v.size \
            else (float("nan"), float("nan"), 0)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--frames", type=int, default=40, help="captures per state")
    ap.add_argument("--steps", default="", help="comma-separated code offsets from the "
                    "starting code, e.g. -1,-2,+3 (empty = stage 1 only)")
    ap.add_argument("--allow-skew-writes", dest="allow", action="store_true",
                    help="permit the stage-2 actuator moves (each needs result=OK)")
    ap.add_argument("--out", default=os.path.join(REPO, "calibration_out", "response"))
    args = ap.parse_args(argv)

    from calibration_loop.capture import HardwareBench
    from calibration_loop.dither import DitherConfig

    cfg = DitherConfig()
    cfg.validate()
    os.makedirs(args.out, exist_ok=True)
    frames_dir = os.path.join(args.out, "raw")
    os.makedirs(frames_dir, exist_ok=True)

    steps = [int(s) for s in args.steps.split(",") if s.strip()] if args.steps else []
    if steps and not args.allow:
        print("stage 2 moves the actuator, so it needs --allow-skew-writes")
        return 2

    bench = HardwareBench(uart_port=args.uart, allow_skew_writes=args.allow)
    rows = []
    polarity = None
    try:
        # Learn the real code first: the actuator's local belief starts at the neutral code
        # and would otherwise be used to compute targets.  This zero-step transaction is
        # also the neutral initialization, so the operating point may move (~57 ps).
        read = bench.actuator.request_steps(0)
        print(f"read: {read.get('result')} code {read.get('code_before')} -> "
              f"{read.get('code_after')} neutral_initialized="
              f"{read.get('neutral_initialized')}")
        if not read.get("ok") or read.get("code_after") is None:
            print(f"FAILED: result={read.get('result')} stage={read.get('stage')} "
                  f"code_before={read.get('code_before')} "
                  f"code_after={read.get('code_after')} generation={read.get('generation')}")
            print(f"raw ACK: {read.get('raw', '').strip()!r}")
            print("Nothing is retried.  Check the capture path read-only first, and re-run "
                  "the board bring-up if the link is down.")
            return 2
        base = int(bench.actuator.code)

        targets = [base] + [base + s for s in steps]
        for target in targets:
            while bench.actuator.code != target:
                step = int(np.sign(target - bench.actuator.code))
                ack = bench.actuator.request_steps(step)
                print(f"  move {step:+d}: {ack.get('result')} "
                      f"{ack.get('code_before')} -> {ack.get('code_after')}")
                if not ack.get("ok"):
                    print(f"ABORT: transaction failed ({ack.get('result')} "
                          f"stage={ack.get('stage')}); nothing is retried")
                    return 2
            tag = f"code{bench.actuator.code:02d}"
            print(f"state {tag}: capturing {args.frames} frames")
            recs, polarity = measure(bench, cfg, args.frames, frames_dir, tag, polarity)
            s = summarize(recs)
            rows.append((bench.actuator.code, tag, len(recs), s))
            print(f"  n={len(recs):3d}  phase {s['phase_ps'][0]:+8.2f}+-{s['phase_ps'][1]:5.2f} ps"
                  f"  centroid {s['centroid_ps'][0]:+9.1f}+-{s['centroid_ps'][1]:6.1f} ps"
                  f"  dither gain {s['dither_gain'][0]:.5f}+-{s['dither_gain'][1]:.5f}"
                  f"  tone gain {s['tone_gain'][0]:.5f}+-{s['tone_gain'][1]:.5f}")
    finally:
        bench.close()

    path = os.path.join(args.out, "response_states.csv")
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["code", "tag", "n"] + [f"{k}_{s}" for k in ROUTES
                                           for s in ("mean", "sd", "n")])
        for code, tag, n, s in rows:
            w.writerow([code, tag, n] + [v for k in ROUTES
                                         for v in (s[k][0], s[k][1], s[k][2])])
    print(f"\nstates : {path}")
    print(f"raw    : {frames_dir}")

    if len(rows) > 1:
        codes = np.array([r[0] for r in rows], dtype=float)
        print("\nresponse to the KNOWN code changes:")
        for key in ("phase_ps", "diff_route_ps", "centroid_ps"):
            means = np.array([r[3][key][0] for r in rows], dtype=float)
            scat = float(np.mean([r[3][key][1] for r in rows]))
            ok = np.isfinite(means)
            if ok.sum() < 2:
                continue
            slope = float(np.polyfit(codes[ok], means[ok], 1)[0])
            print(f"  {key:>14}: {slope:+7.2f} ps/code, per-frame scatter {scat:7.1f} ps"
                  f"  -> 1 code is {abs(slope) / scat:.2f} sigma per frame, "
                  f"{abs(slope) / (scat / np.sqrt(args.frames)):.1f} sigma over "
                  f"{args.frames} frames")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
