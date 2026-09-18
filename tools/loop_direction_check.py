"""Offline proof that CalibrationLoop stops actuating when the bench disagrees.

The skew loop is the only part of the calibration that *writes* hardware, and its
failure mode is quiet.  A move in the wrong direction pushes the measured error
further out, the next batch cheerfully asks for another code, and every single
transaction answers ``result=OK`` while the actuator walks away from the target.
``CalibrationLoop._skew_decision`` guards that with a direction check
(``LoopOptions.skew_direction_tol_ps``); this file proves the guard works, using
the bench *model* instead of the bench.

Two cases over the same code path, differing only in how the actuator responds:

  * honest bench -- each code shifts the measured skew by the calibrated
    +7.4 ps, so the loop walks the code towards the target and never latches.
    Without this case a check that always fired would look identical to one that
    works;
  * lying bench -- the same code moves the skew the other way.  The loop must
    latch after the batch that confirms the disagreement and never write again.

The controller's sign logic is not re-implemented here: both stubs delegate to
``capture.SkewActuator.error_to_steps``, so this tests the production decision
path (including the deadband and the one-code-per-batch clamp).

    python tools/loop_direction_check.py
"""

from __future__ import annotations

import os
import sys
from types import SimpleNamespace

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.capture import SkewActuator          # noqa: E402
from calibration_loop.dither import DitherConfig           # noqa: E402
from calibration_loop.loop import CalibrationLoop, LoopOptions   # noqa: E402
from calibration_loop.simulate import BenchModel           # noqa: E402

BATCHES = 8
FRAMES_PER_BATCH = 5
INITIAL_SKEW_PS = 45.0
"""Far outside the 10 ps deadband, so an honest bench *must* walk several codes
before it may stop -- otherwise the positive control would never move at all and
would prove nothing."""


class StubActuator:
    """The firmware transaction, with a bench that may be lying about the sign.

    ``direction=+1`` reproduces the bench-measured behaviour (a code moves the
    measured B-A skew by +HOST_STEP_PS); ``direction=-1`` is the fault this test
    is about.  Decision logic comes from production code, never from here.
    """

    HOST_STEP_PS = SkewActuator.HOST_STEP_PS
    NEUTRAL_CODE = SkewActuator.NEUTRAL_CODE
    CODE_MIN = SkewActuator.CODE_MIN
    CODE_MAX = SkewActuator.CODE_MAX

    def __init__(self, bench, direction: float = 1.0):
        self.bench = bench
        self.direction = float(direction)
        self.code = self.NEUTRAL_CODE
        self.code_known = True
        self.deadband_ps = SkewActuator.deadband_ps
        self.moves: list[int] = []
        self.commanded_ps = 0.0

    def error_to_steps(self, error_ps: float) -> int:
        return SkewActuator.error_to_steps(self, error_ps)

    def request_steps(self, delta: int, max_step: int = 1) -> dict:
        delta = int(np.clip(delta, -abs(int(max_step)), abs(int(max_step))))
        if delta == 0:
            return {"ok": True, "result": "OK", "code_after": self.code,
                    "applied_steps": 0}
        self.moves.append(delta)
        self.code = int(np.clip(self.code + delta, self.CODE_MIN, self.CODE_MAX))
        self.commanded_ps += self.direction * delta * self.HOST_STEP_PS
        self.bench.command_skew(self.commanded_ps)
        return {"ok": True, "result": "OK", "code_after": self.code,
                "applied_steps": delta}


def run_case(direction: float):
    cfg = DitherConfig()
    cfg.validate()
    bench = BenchModel(cfg=cfg, skew_a_ps=0.0, skew_b_ps=INITIAL_SKEW_PS,
                       noise_rms_codes=2.0)
    actuator = StubActuator(bench, direction=direction)
    bench.actuator = actuator
    """Attaching an actuator is what puts CalibrationLoop on its hardware branch
    (per-frame gain/offset, batch skew), so this exercises the real code path."""

    options = LoopOptions(
        close_skew_loop=True,
        skew_batch_frames=FRAMES_PER_BATCH,
        skew_deadband_ps=actuator.deadband_ps,
    )
    loop = CalibrationLoop(bench, cfg, options=options)
    loop.run(BATCHES * FRAMES_PER_BATCH, verbose=False)
    return loop, actuator


def decisions(loop) -> list[dict]:
    return [r for r in loop.log if r.get("skew_action")]


def yield_case(kept: int, total: int = 20):
    """One batch decision driven directly, to test the yield floor.

    ``CalibrationLoop._skew_decision`` needs only a log row and a small estimate
    object, so the acceptance yield can be set exactly: ``skew_batch`` rejects a
    frame whose alignment margin is below 6.0, so frames with margin 8.0 survive
    and frames with margin 5.0 do not.
    """
    cfg = DitherConfig()
    cfg.validate()
    bench = BenchModel(cfg=cfg)
    actuator = StubActuator(bench, direction=+1.0)
    bench.actuator = actuator
    loop = CalibrationLoop(bench, cfg, options=LoopOptions(
        close_skew_loop=True, skew_batch_frames=total))

    row: dict = {}
    for i in range(total):
        # a fresh row per frame, exactly as CalibrationLoop.step() does
        row = {"align_margin": 8.0 if i < kept else 5.0}
        loop._skew_decision(
            SimpleNamespace(
                skew_phase_ps=-43.0, skew_diff_route_ps=-43.0,
                skew_centroid_ps=-43.0, skew_source="pulse-window",
                ch_a=SimpleNamespace(tone_amplitude=2000.0, residual_rms=30.0),
            ),
            row,
        )
    return row, actuator


def main() -> int:
    checks: list[tuple[str, bool, str]] = []

    honest, honest_act = run_case(+1.0)
    print(f"honest bench ({INITIAL_SKEW_PS:.0f} ps mismatch, "
          f"{FRAMES_PER_BATCH} frames/batch)")
    for i, r in enumerate(decisions(honest)):
        print(f"  batch {i}  code {r.get('skew_code', honest_act.NEUTRAL_CODE):>2}  "
              f"mean {r['skew_batch_mean_ps']:+7.2f} ps  "
              f"error {r['skew_batch_error_ps']:+7.2f} ps  {r['skew_action']}")

    checks.append(("honest bench never latches the actuator",
                   honest._skew_abort is None, f"abort={honest._skew_abort!r}"))
    checks.append(("honest bench actually moves the code",
                   len(honest_act.moves) >= 2, f"{len(honest_act.moves)} moves"))
    checks.append(("honest bench moves towards the target",
                   all(m < 0 for m in honest_act.moves),
                   f"moves {honest_act.moves}"))
    tail = decisions(honest)[-1] if decisions(honest) else None
    checks.append(("honest bench ends inside the deadband",
                   tail is not None
                   and abs(tail["skew_batch_error_ps"]) <= honest_act.deadband_ps,
                   f"final error {tail['skew_batch_error_ps']:+.2f} ps"
                   if tail else "no decision recorded"))

    lying, lying_act = run_case(-1.0)
    print(f"\nlying bench (same code path, actuator sign inverted)")
    for i, r in enumerate(decisions(lying)):
        print(f"  batch {i}  code {r.get('skew_code', lying_act.NEUTRAL_CODE):>2}  "
              f"mean {r['skew_batch_mean_ps']:+7.2f} ps  "
              f"error {r['skew_batch_error_ps']:+7.2f} ps  {r['skew_action']}")

    checks.append(("lying bench latches on direction",
                   bool(lying._skew_abort) and "direction" in (lying._skew_abort or ""),
                   f"abort={lying._skew_abort!r}"))
    checks.append(("lying bench stops after the first move",
                   len(lying_act.moves) == 1, f"{len(lying_act.moves)} moves"))
    after = [r for r in decisions(lying) if r["skew_action"] == "aborted"]
    checks.append(("every later batch reports the latch instead of moving",
                   len(after) == len(decisions(lying)) - 2,
                   f"{len(after)} latched batches of {len(decisions(lying))}"))
    checks.append(("the fault is written into the run metadata",
                   lying.save(os.path.join(REPO, "calibration_out", "_direction_check"),
                              stem="lying")["meta"].read_text(encoding="utf-8")
                   .find('"skew_abort": "direction') >= 0,
                   "skew_abort recorded in the meta JSON"))

    healthy_row, healthy_act = yield_case(kept=18)
    thin_row, thin_act = yield_case(kept=2)
    print("acceptance yield (2 of 20 frames survive vs 18 of 20)")
    print(f"  18/20 -> {healthy_row.get('skew_action')}   "
          f"2/20 -> {thin_row.get('skew_action')}")
    checks.append(("a healthy batch may move the actuator",
                   len(healthy_act.moves) == 1, f"{healthy_act.moves}"))
    checks.append(("a thin batch may not, whatever its mean says",
                   not thin_act.moves and "low-yield" in thin_row.get("skew_action", ""),
                   f"{thin_row.get('skew_action')}"))

    print()
    ok = True
    for name, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name} -- {detail}")
        ok = ok and passed
    print(f"\nDIRECTION CHECK: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
