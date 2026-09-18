"""Scratch: does the gain observable fix the bench failure?  Offline proof.

Bench finding (2026-09-17): the dither amplitude ratio and the main-tone amplitude
ratio are two estimates of one gain mismatch that disagree by ~1.2 %, the loop
integrated the *dither* one, and the corrected A-B difference spur was therefore
capped at -35 dBc while the raw one reached -38.6 dBc.  The residual dither ratio
stayed unbiased, so the loop converged happily -- to the wrong point.

The bench model can reproduce that, because ``pulse_gain_a`` / ``pulse_gain_b`` scale
the dither pulses only.  This runs the same closed loop twice over a model whose
pulse gains make the dither ratio ~1.2 % higher than the tone ratio, once per
observable, and compares what each one leaves behind:

  * ``dither``  -- the old behaviour: the tone ratio is left ~1.2 % out;
  * ``tone``    -- the fix: the tone ratio is what gets nulled.

The number that matters is the last one: the corrected A-B difference spur, which is
the metric the calibration is supposed to improve.

    python calibration_out/_gain_loop_test.py
"""

from __future__ import annotations

import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.dither import DitherConfig                    # noqa: E402
from calibration_loop.loop import CalibrationLoop, LoopOptions      # noqa: E402
from calibration_loop.simulate import BenchModel                    # noqa: E402

SAMPLES = 120
DISPERSION = 1.012
"""Pulse-gain offset that makes the dither ratio 1.2 % above the tone ratio,
matching the measured bench disagreement (1.1-1.4 % over three runs)."""


def run(observable: str) -> dict:
    cfg = DitherConfig()
    cfg.validate()
    bench = BenchModel(cfg=cfg, pulse_gain_a=1.0, pulse_gain_b=DISPERSION)
    options = LoopOptions(gain_observable=observable, close_skew_loop=False)
    loop = CalibrationLoop(bench, cfg, options=options)
    loop.run(SAMPLES, verbose=False)

    rows = [r for r in loop.log if not r.get("rejected")]
    tail = rows[-max(1, len(rows) // 4):]

    def mean(key):
        vals = [r[key] for r in tail if np.isfinite(r[key])]
        return float(np.mean(vals)) if vals else float("nan")

    return {
        "observable": observable,
        "tone_ratio": mean("tone_ratio"),
        "dither_ratio": mean("gain_ratio"),
        "raw_spur": mean("raw_difference_dbc"),
        "cal_spur": mean("cal_difference_dbc"),
        "state_ratio": loop.state.gain_corr_b / loop.state.gain_corr_a,
        "source": tail[-1].get("gain_source"),
        "samples": len(rows),
    }


def main() -> int:
    results = [run("dither"), run("tone")]

    print(f"model: tone ratio {1.021:.4f}, dither ratio "
          f"{1.021 * DISPERSION:.4f} (disagreement "
          f"{(DISPERSION - 1) * 100:.1f} %), {SAMPLES} samples each\n")
    header = f"{'observable':>11}{'controlled':>12}{'tone resid':>12}" \
             f"{'dither resid':>14}{'raw spur':>10}{'cal spur':>10}{'source':>9}"
    print(header)
    for r in results:
        controlled = r["tone_ratio"] if r["observable"] == "tone" else r["dither_ratio"]
        print(f"{r['observable']:>11}{controlled:>12.5f}"
              f"{r['tone_ratio'] - 1:>+12.4f}{r['dither_ratio'] - 1:>+14.4f}"
              f"{r['raw_spur']:>10.1f}{r['cal_spur']:>10.1f}{str(r['source']):>9}")

    old, new = results
    # Sign, stated once: the pulse gains put the *dither* ratio above the tone ratio,
    # so nulling the dither under-corrects the tone and the residual comes out
    # negative.  That is the same direction the bench showed (tone/dither = 0.987-0.989).
    checks = [
        ("the dither route reproduces the bench failure (tone left ~1.2 % out, low)",
         old["tone_ratio"] < 1.0
         and abs(abs(old["tone_ratio"] - 1) - (DISPERSION - 1)) < 0.004,
         f"tone residual {old['tone_ratio'] - 1:+.4f} against a "
         f"{DISPERSION - 1:+.4f} disagreement"),
        ("the tone route nulls the tone ratio",
         abs(new["tone_ratio"] - 1) < 0.004,
         f"tone residual {new['tone_ratio'] - 1:+.4f}"),
        # more negative dBc is better, so the improvement is old - new
        ("so the corrected A-B spur improves, and improves a lot",
         old["cal_spur"] - new["cal_spur"] > 4.0,
         f"{old['cal_spur']:.1f} -> {new['cal_spur']:.1f} dBc "
         f"({new['cal_spur'] - old['cal_spur']:+.1f} dB)"),
        ("the tone route leaves the correction gain-limited by nothing",
         abs(new["cal_spur"] - new["raw_spur"]) > 10.0,
         f"corrected {new['cal_spur']:.1f} dBc against a raw {new['raw_spur']:.1f} dBc "
         f"that is dominated by the 2.1 % gain mismatch itself"),
        ("the raw spur is untouched by the choice (it never had the correction)",
         abs(new["raw_spur"] - old["raw_spur"]) < 1.0,
         f"{old['raw_spur']:.1f} vs {new['raw_spur']:.1f} dBc"),
        ("each run reports which observable it actually used",
         old["source"] == "dither" and new["source"] == "tone",
         f"{old['source']} / {new['source']}"),
    ]

    print()
    ok = True
    for name, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name} -- {detail}")
        ok = ok and passed
    print(f"\nGAIN LOOP: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
