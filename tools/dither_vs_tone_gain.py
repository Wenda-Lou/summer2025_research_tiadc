"""Do the dither amplitude ratio and the main-tone amplitude ratio agree?

The closed loop equalises the *dither* amplitude between the two channels: every
``CalibrationState.update`` drives ``mean_g / g_x -> 1`` from the pulse-window
amplitudes.  On the bench that leaves a corrected A-B difference spur 3.6 dB
(run1) and 5.5 dB (run2) worse than the skew-limited raw one, and the gap closes to
0.3 dB in both runs if it is attributed to the applied gain ratio
``1 - gain_corr_b/gain_corr_a`` (1.26 % and 2.05 %).  That attribution is only
possible if the two routes to the gain mismatch disagree -- the dither route that
the loop nulls, and the tone route that the *normalised* difference spur actually
sees.

This settles it read-only, on the same frames:

  * dither route -- ``BlockEstimate.gain_ratio``, from the pulse-window amplitudes;
  * tone route   -- ``ChannelEstimate.tone_amplitude``, a coherent fit of the main
                    tone over the whole record, which never touches the pulse
                    windows.

Both use the same records and the same session polarity anchor, so a difference
between them is a property of the bench (a narrow pulse and a 199 MHz tone do not
see the same channel gain), not of the loop's bookkeeping.

    python tools/dither_vs_tone_gain.py --uart COM5 --frames 40
"""

# Note on the skew column, which is context here and not the subject: ~7 % of
# frames drop out of the estimator's phase route entirely (nan), so summarise it
# with nanmean and never mean -- one nan poisons the average, which is exactly how
# the first run of this tool printed "+nan ps".  One run also read skew +1.74 ps
# where two later runs and an offline replay of the archived frames read
# -9.4 / -9.8 / -9.74 ps, matching the closed-loop runs (-9.3 / -9.4 ps) and the
# read-only stability check (-8.5 to -11.6 ps); that batch was internally
# inconsistent with its own -38 dBc difference spur, which a +1.7 ps skew would
# have put near -54 dBc.  Unresolved, and it is why --save-frames exists: the
# captures of a suspicious run can then be replayed without bench time.

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)


def db(x: float) -> float:
    return 20.0 * math.log10(max(abs(x), 1e-15))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--frames", type=int, default=40)
    ap.add_argument("--out", default=os.path.join(REPO, "calibration_out"))
    ap.add_argument("--save-frames", dest="save_frames", action="store_true",
                    help="write the raw captures next to the CSV so later analysis "
                         "of this state needs no bench time")
    ap.add_argument("--state-meta", dest="state_meta", default=None,
                    help="a *_meta.json from a closed-loop run; its final_state is "
                         "applied to compare the skew estimate on raw vs corrected "
                         "records")
    args = ap.parse_args(argv)

    from calibration_loop.capture import HardwareBench
    from calibration_loop.dither import DitherConfig
    from calibration_loop.estimator import (
        CalibrationState, estimate_block, polarity_anchor, prepare_capture,
    )

    cfg = DitherConfig()
    cfg.validate()

    state = None
    if args.state_meta:
        with open(args.state_meta, encoding="utf-8") as fh:
            state = CalibrationState(**json.load(fh)["final_state"])
        print(f"applying final_state from {args.state_meta} for the corrected path")

    frames_dir = os.path.join(args.out, "dither_vs_tone_frames")
    if args.save_frames:
        os.makedirs(frames_dir, exist_ok=True)

    bench = HardwareBench(uart_port=args.uart)   # read-only: no writes allowed
    rows: list[dict] = []
    signature = None
    polarity = None
    try:
        for _ in range(args.frames):
            raw = bench.capture()
            if not raw:
                continue
            prep = prepare_capture(raw, cfg, signature=signature)
            if signature is None:
                signature = prep["signature"]
            if polarity is None:
                polarity = polarity_anchor(prep)
            est = estimate_block(
                prep["ch_a"], prep["ch_b"], cfg,
                cancel_signal=True, n0=prep["n0"],
                polarity_sign=polarity, pin_polarity=True,
            )
            ta, tb = est.ch_a.tone_amplitude, est.ch_b.tone_amplitude
            r_dither = est.gain_ratio
            r_tone = abs(tb) / abs(ta) if np.isfinite(ta) and abs(ta) > 0 else np.nan
            row = {
                "frame": len(rows),
                "align_margin": prep["align_margin"],
                "r_dither": r_dither,
                "r_tone": r_tone,
                "tone_a": ta, "tone_b": tb,
                "dither_a": est.ch_a.gain_codes, "dither_b": est.ch_b.gain_codes,
                "offset_a": est.ch_a.offset_codes, "offset_b": est.ch_b.offset_codes,
                "skew_ps": est.skew_mismatch_ps,
            }
            if state is not None:
                # The same estimator on the *corrected* records, which is what the
                # closed loop itself sees.  If the two disagree, the skew estimate
                # depends on the applied gain correction -- which matters, because
                # the loop's whole learning curve is recorded in that basis.
                cal_a, cal_b = state.apply(prep["ch_a"], prep["ch_b"])
                est_c = estimate_block(
                    cal_a, cal_b, cfg, cancel_signal=True, n0=prep["n0"],
                    polarity_sign=polarity, pin_polarity=True,
                )
                row["skew_ps_corrected"] = est_c.skew_mismatch_ps
                row["gain_ratio_corrected"] = est_c.gain_ratio
            if args.save_frames:
                with open(os.path.join(frames_dir, f"frame_{row['frame']:03d}.bin"),
                          "wb") as fh:
                    fh.write(raw)
            rows.append(row)
    finally:
        bench.close()

    good = [r for r in rows
            if np.isfinite(r["r_dither"]) and np.isfinite(r["r_tone"])
            and r["align_margin"] >= 6.0]
    if not good:
        print("no usable frames")
        return 1

    d = np.array([r["r_dither"] for r in good])
    t = np.array([r["r_tone"] for r in good])
    n = len(good)
    print(f"{n} usable frames of {len(rows)} captured\n")
    print(f"  dither route  r = g_B/g_A : {d.mean():.5f} +- {d.std(ddof=1) / np.sqrt(n):.5f}"
          f"   (scatter {d.std(ddof=1):.5f})   -> mismatch {db(d.mean() - 1):.2f} dBc")
    print(f"  tone   route  r = |A_B|/|A_A| : {t.mean():.5f} "
          f"+- {t.std(ddof=1) / np.sqrt(n):.5f}   (scatter {t.std(ddof=1):.5f})"
          f"   -> mismatch {db(t.mean() - 1):.2f} dBc")

    diff = t - d
    ratio = t.mean() / d.mean()
    print(f"\n  tone - dither              : {diff.mean():+.5f} "
          f"+- {diff.std(ddof=1) / np.sqrt(n):.5f}   (paired)")
    print(f"  tone / dither              : {ratio:.5f}"
          f"   -> |1 - ratio| = {abs(1 - ratio):.5f} = {db(1 - ratio):.2f} dBc")
    print(f"  t-statistic on the difference: "
          f"{diff.mean() / (diff.std(ddof=1) / math.sqrt(n)):+.2f}")

    print("\n  What the loop leaves behind: it nulls the dither ratio, so the tone")
    print("  ratio of the corrected data is r_tone/r_dither, i.e. the normalised")
    print(f"  A-B difference spur the correction cannot remove is {db(1 - ratio):.2f} dBc")
    print("  (run1 and run2 measured the correction leaving -38.0 and -33.8 dBc terms,")
    print("   i.e. 1.26 % and 2.05 %.)")

    print(f"\n  tone amplitudes: |A| = {np.mean([abs(r['tone_a']) for r in good]):.1f} / "
          f"{np.mean([abs(r['tone_b']) for r in good]):.1f} codes")
    print(f"  dither amplitudes: {np.mean([r['dither_a'] for r in good]):.1f} / "
          f"{np.mean([r['dither_b'] for r in good]):.1f} codes")

    # Skew is only context here, but it must not be summarised with a plain mean:
    # a single dropped frame turns the whole average into nan (that is exactly what
    # happened on the first run of this tool).  Report the dropout rate instead --
    # it is also the reason LoopOptions.skew_min_yield exists.
    sk = np.array([r["skew_ps"] for r in good], dtype=float)
    n_fin = int(np.isfinite(sk).sum())
    print(f"  skew, raw records     : {np.nanmean(sk):+.2f} ps over {n_fin}/{len(good)} "
          f"frames ({len(good) - n_fin} dropped out, "
          f"{100.0 * (len(good) - n_fin) / len(good):.1f} %)")
    if "skew_ps_corrected" in good[0]:
        skc = np.array([r["skew_ps_corrected"] for r in good], dtype=float)
        n_finc = int(np.isfinite(skc).sum())
        print(f"  skew, corrected records: {np.nanmean(skc):+.2f} ps over "
              f"{n_finc}/{len(good)} frames")
        both = np.isfinite(sk) & np.isfinite(skc)
        if both.sum() > 2:
            print(f"  corrected - raw        : "
                  f"{(skc[both] - sk[both]).mean():+.2f} +- "
                  f"{(skc[both] - sk[both]).std(ddof=1) / math.sqrt(both.sum()):.2f} ps "
                  f"(paired, n={int(both.sum())})")

    os.makedirs(args.out, exist_ok=True)
    path = os.path.join(args.out, "dither_vs_tone.csv")
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\n  per-frame data: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
