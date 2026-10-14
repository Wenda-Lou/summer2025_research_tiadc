"""Record the Python calibration loop as a golden file for the C port.

Runs ``calibration_loop.CalibrationLoop`` against ``BenchModel`` and writes, for
every capture, the raw DMA bytes it consumed plus what the Python loop made of
them: de-framing, the block estimate, the controller state after the update and
the skew decision.  ``adc_cal_sim --run-loop-golden FILE`` replays the same bytes
through ``adc_calibration_loop.c`` and compares field by field.

Four cases:
  model_nominal   model branch (per-frame skew integrator), BenchModel defaults
  model_mismatch  model branch, larger gain / offset / skew mismatch
  batch_honest    hardware branch (batch decision) with an honest stub actuator
  batch_lying     hardware branch with an actuator that moves the wrong way,
                  so the direction latch must fire

    python calibration_loop_c/export_golden.py [--out FILE]

Needs numpy.  The output (~1 MB) is a generated artefact; do not commit it.
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

import calibration_loop.loop as loop_mod                    # noqa: E402
from calibration_loop.capture import SkewActuator           # noqa: E402
from calibration_loop.dither import DitherConfig, polarity_sequence   # noqa: E402
from calibration_loop.loop import CalibrationLoop, LoopOptions        # noqa: E402
from calibration_loop.simulate import BenchModel            # noqa: E402

SOURCE_CODES = {
    "none": 0, "phase": 1, "phase+branch(prior)": 2, "phase+branch(centroid)": 3,
    "profile-fallback": 4, "observables-disagree": 5,
}


def action_code(action: str) -> int:
    if not action:
        return 0
    for prefix, code in (("measure-only", 1), ("aborted", 2), ("no-usable-frames", 3),
                         ("low-yield", 4), ("abort:direction", 5), ("inside-deadband", 6),
                         ("at-range-limit", 7)):
        if action.startswith(prefix):
            return code
    if action.startswith("move"):
        return 8 if action.endswith(":OK") else 9
    raise ValueError(f"unknown skew action {action!r}")


class StubActuator:
    """Same stub as tools/loop_direction_check.py: production decision logic,
    a bench whose sign may be wrong."""

    HOST_STEP_PS = SkewActuator.HOST_STEP_PS
    NEUTRAL_CODE = SkewActuator.NEUTRAL_CODE
    CODE_MIN = SkewActuator.CODE_MIN
    CODE_MAX = SkewActuator.CODE_MAX

    def __init__(self, bench, direction: float):
        self.bench = bench
        self.direction = float(direction)
        self.code = self.NEUTRAL_CODE
        self.code_known = True
        self.deadband_ps = SkewActuator.deadband_ps
        self.commanded_ps = 0.0

    def error_to_steps(self, error_ps: float) -> int:
        return SkewActuator.error_to_steps(self, error_ps)

    def request_steps(self, delta: int, max_step: int = 1) -> dict:
        delta = int(np.clip(delta, -abs(int(max_step)), abs(int(max_step))))
        self.code = int(np.clip(self.code + delta, self.CODE_MIN, self.CODE_MAX))
        self.commanded_ps += self.direction * delta * self.HOST_STEP_PS
        self.bench.command_skew(self.commanded_ps)
        return {"ok": True, "result": "OK", "code_after": self.code}


class Recorder:
    """Bench wrapper that keeps every buffer handed to the loop."""

    def __init__(self, bench):
        self.bench = bench
        self.frames: list[bytes] = []

    def capture(self, *args, **kwargs):
        raw = self.bench.capture(*args, **kwargs)
        self.frames.append(raw)
        return raw

    def command_skew(self, delay_ps: float) -> None:
        self.bench.command_skew(delay_ps)


def run_case(name: str, bench: BenchModel, options: LoopOptions, qualified: int,
             actuator_direction: float | None):
    rec = Recorder(bench)
    actuator = None
    if actuator_direction is not None:
        actuator = StubActuator(bench, actuator_direction)
        rec.actuator = actuator
    loop = CalibrationLoop(rec, bench.cfg, options=options)

    captured = {}
    orig_estimate = loop_mod.estimate_block
    orig_prepare = loop_mod.prepare_capture

    def spy_estimate(*a, **k):
        captured["est"] = orig_estimate(*a, **k)
        return captured["est"]

    def spy_prepare(*a, **k):
        captured["prep"] = orig_prepare(*a, **k)
        return captured["prep"]

    loop_mod.estimate_block = spy_estimate
    loop_mod.prepare_capture = spy_prepare
    records = []
    try:
        n_ok = 0
        streak = 0
        while n_ok < qualified:
            row = loop.step()
            est, prep = captured["est"], captured["prep"]
            st = loop.state
            rejected = bool(row.get("rejected"))
            records.append({
                "raw": rec.frames[-1],
                "reason": row.get("rejected", ""),
                "values": [
                    1 if rejected else 0,
                    prep["rotation"], prep["n0"], 1 if prep["swapped"] else 0,
                    prep["align_margin"], loop._polarity,
                    est.ch_a.offset_codes, est.ch_b.offset_codes,
                    est.ch_a.gain_codes, est.ch_b.gain_codes,
                    est.gain_ratio, est.tone_ratio,
                    est.ch_a.tone_amplitude, est.ch_a.residual_rms,
                    est.skew_phase_ps, est.skew_diff_route_ps, est.skew_centroid_ps,
                    est.skew_mismatch_ps, SOURCE_CODES[est.skew_source],
                    st.offset_a, st.offset_b, st.gain_corr_a, st.gain_corr_b, st.skew_cmd_ps,
                    action_code(row.get("skew_action", "")),
                    int(row.get("skew_step_codes", 0)),
                ],
            })
            if rejected:
                streak += 1
                if streak >= options.max_consecutive_rejects:
                    break
            else:
                streak = 0
                n_ok += 1
    finally:
        loop_mod.estimate_block = orig_estimate
        loop_mod.prepare_capture = orig_prepare

    truth = bench.truth()
    n_rej = sum(1 for r in records if r["values"][0])
    print(f"  {name:15s} {len(records):3d} captures, {n_rej} rejected; truth "
          f"gain {truth['gain_ratio']:.4f} skew {truth['skew_mismatch_ps']:+.2f} ps; "
          f"final gain_corr_b/a {loop.state.gain_corr_b / loop.state.gain_corr_a:.5f}")
    return {
        "name": name, "cfg": bench.cfg, "options": options, "records": records,
        "batch": actuator is not None,
        "step_ps": StubActuator.HOST_STEP_PS, "deadband_ps": SkewActuator.deadband_ps,
        "code0": StubActuator.NEUTRAL_CODE,
    }


def fmt(x) -> str:
    x = float(x)
    if not np.isfinite(x):
        return "nan"
    return repr(x)


def write(cases, path: str) -> None:
    with open(path, "w", encoding="ascii", newline="\n") as fh:
        fh.write("LOOPGOLDEN 1\n")
        fh.write(f"cases {len(cases)}\n")
        for case in cases:
            cfg: DitherConfig = case["cfg"]
            opt: LoopOptions = case["options"]
            pol = polarity_sequence(cfg).astype(int)
            fh.write(f"case {case['name']}\n")
            fh.write("config {} {} {} {} {} {} {} {} {} {} {}\n".format(
                fmt(cfg.fs_dac), cfg.n_dac_points, cfg.adc_ratio, cfg.sig_cycles,
                fmt(cfg.amp_dbfs), cfg.dither_period_dac, cfg.dither_position_dac,
                cfg.dither_edge_dac, cfg.dither_top_dac, fmt(cfg.dither_scale_lsb),
                cfg.n_events))
            fh.write("polarity " + " ".join(str(p) for p in pol) + "\n")
            fh.write("options {} {} {} {} {} {} {} {} {} {} {}\n".format(
                1 if case["batch"] else 0, 1 if opt.cancel_signal else 0,
                1 if opt.close_skew_loop else 0, opt.skew_batch_frames,
                fmt(opt.min_align_margin), fmt(opt.max_skew_samples),
                fmt(opt.max_gain_deviation), fmt(opt.skew_direction_tol_ps),
                fmt(case["step_ps"]), fmt(case["deadband_ps"]), case["code0"]))
            fh.write(f"frames {len(case['records'])}\n")
            for rec in case["records"]:
                fh.write(f"frame {len(rec['raw'])}\n")
                fh.write(rec["raw"].hex() + "\n")
                fh.write("expect " + " ".join(fmt(v) for v in rec["values"]) + "\n")
                fh.write(f"# {rec['reason'] or 'accepted'}\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", default=os.path.join(REPO, "calibration_sim", "output",
                                                  "loop_golden.txt"))
    args = ap.parse_args()

    cfg = DitherConfig()
    cfg.validate()
    cases = [
        run_case("model_nominal", BenchModel(cfg=cfg), LoopOptions(), 30, None),
        run_case("model_mismatch",
                 BenchModel(cfg=cfg, gain_b=1.05, offset_b_codes=-40.0, skew_b_ps=60.0,
                            seed=11),
                 LoopOptions(), 30, None),
        run_case("batch_honest",
                 BenchModel(cfg=cfg, skew_b_ps=45.0, noise_rms_codes=2.0, seed=7),
                 LoopOptions(skew_batch_frames=5), 40, +1.0),
        run_case("batch_lying",
                 BenchModel(cfg=cfg, skew_b_ps=45.0, noise_rms_codes=2.0, seed=7),
                 LoopOptions(skew_batch_frames=5), 40, -1.0),
    ]
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    write(cases, args.out)
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
