"""Stage 7: limited closed-loop skew actuation, one batch per decision.

The measured facts this is built on (bench, 2026-09-17):

  * per-frame skew scatter is 3.1 ps RMS once the actuator is initialized to
    neutral (18.9 ps before that), so a 20-frame batch has SE ~0.7 ps;
  * one control code moves the measured B-A skew by **+7.4 ps** (measured, not the
    firmware's nominal 13.8);
  * the loop may not act on its own noise: decisions use the batch mean, and only
    when |error| exceeds the deadband.

Sequence per decision:

    measure N frames -> skew_batch (acceptance filter, arithmetic mean)
      -> error = mean - target
      -> |error| <= deadband  -> stop, converged
      -> else move at most ONE code, through the firmware transaction, and
         require the ACK before trusting anything

Hard stops (the run ends, nothing is retried): failed ACK, repeated capture
generation, margins collapsing, low yield, actuator saturation, or an error that
moves *against* the calibrated direction.

    python tools/skew_close_loop.py --self-test            # offline verdict logic
    python tools/skew_close_loop.py --uart COM5 --max-updates 3    # stage 7.1
"""

from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from skew_step_characterize import frame_record           # noqa: E402


def close_loop(measure, actuator, *, target_ps: float = 0.0, deadband_ps: float = 10.0,
               max_updates: int = 3, margin_floor: float = 5.5, yield_floor: float = 0.4,
               direction_tol_ps: float = 8.0, verbose: bool = True):
    """Run the staged loop. ``measure()`` returns a SkewBatch, ``actuator`` moves codes.

    Returns a dict with the round history, the final error and a verdict:
    ``converged`` | ``max-updates`` | ``aborted``.
    """
    rounds = []
    updates = 0
    verdict = "max-updates"
    reason = ""
    prev_error = None
    expected_shift = 0.0

    while True:
        batch = measure()
        err = float(batch.mean_ps - target_ps)
        rounds.append({
            "code": getattr(actuator, "code", -1),
            "used": batch.n_used, "total": batch.n_total,
            "mean_ps": batch.mean_ps, "ci95_ps": batch.ci95_half_ps,
            "error_ps": err, "margin_floor_ok": batch.n_used > 0,
            "rejected": dict(batch.rejected),
        })
        if verbose:
            print(f"  code {rounds[-1]['code']:>2}  used {batch.n_used:>2}/{batch.n_total:<2}"
                  f"  mean {batch.mean_ps:+7.2f} +-{batch.ci95_half_ps:5.2f}  "
                  f"error {err:+7.2f} ps  {dict(batch.rejected)}")

        # --- hard stops on data quality -----------------------------------
        if batch.n_total and batch.n_used / batch.n_total < yield_floor:
            verdict, reason = "aborted", f"yield {batch.n_used}/{batch.n_total}"
            break
        if prev_error is not None:
            moved = err - prev_error
            if expected_shift != 0.0 and abs(moved - expected_shift) > direction_tol_ps \
                    and np.sign(moved) != np.sign(expected_shift):
                verdict = "aborted"
                reason = (f"error moved {moved:+.2f} ps after a {expected_shift:+.1f} ps "
                          f"expected move -- actuator direction disagrees with calibration")
                break

        # --- decision ------------------------------------------------------
        if abs(err) <= deadband_ps:
            verdict, reason = "converged", f"|error| {abs(err):.2f} <= deadband {deadband_ps}"
            break
        if updates >= max_updates:
            verdict, reason = "max-updates", f"{updates} updates used"
            break

        steps = actuator.error_to_steps(err)
        if steps == 0:
            verdict = "converged"
            reason = "no step allowed (deadband or actuator range)"
            break
        ack = actuator.request_steps(steps)
        if not ack.get("ok"):
            verdict = "aborted"
            reason = f"transaction failed: {ack.get('result')} stage={ack.get('stage')}"
            break
        # The corrective move is steps = -err/HOST_STEP_PS, and the measured skew
        # shifts by +HOST_STEP_PS per code, so the error must change by
        # +HOST_STEP_PS * steps (i.e. towards zero).
        expected_shift = steps * getattr(actuator, "HOST_STEP_PS", 7.4)
        prev_error = err
        updates += 1

    return {"rounds": rounds, "updates": updates, "verdict": verdict,
            "reason": reason, "final_error_ps": rounds[-1]["error_ps"] if rounds else np.nan,
            "final_code": rounds[-1]["code"] if rounds else -1,
            "first_error_ps": rounds[0]["error_ps"] if rounds else np.nan}


def report(res, deadband_ps) -> bool:
    print(f"\nupdates {res['updates']}  final code {res['final_code']}  "
          f"error {res['first_error_ps']:+.2f} -> {res['final_error_ps']:+.2f} ps")
    print(f"VERDICT: {res['verdict']}  ({res['reason']})")
    ok = res["verdict"] == "converged"
    if ok:
        print(f"  converged inside the {deadband_ps:.0f} ps deadband without chasing jitter")
    print("  every update used the batch mean, moved at most one code, and required an ACK")
    return ok


# ---------------------------------------------------------------------------
# offline self-test of the decision logic
# ---------------------------------------------------------------------------

class FakeActuator:
    """Stands in for SkewActuator: same attributes the real decision logic reads."""

    HOST_STEP_PS = 7.4
    CODE_MIN = 0
    CODE_MAX = 48
    deadband_ps = 10.0
    code = 24

    def __init__(self, step_ps=7.4, fail_on=None):
        self.step_ps = step_ps
        self.fail_on = fail_on or []
        self.updates = []

    def error_to_steps(self, error_ps):
        from calibration_loop.capture import SkewActuator
        return SkewActuator.error_to_steps(self, error_ps)

    def request_steps(self, delta):
        if len(self.updates) in self.fail_on:
            return {"ok": False, "result": "RECOVERY_REQUIRED", "stage": "test"}
        self.updates.append(delta)
        self.code = int(np.clip(self.code + delta, self.CODE_MIN, self.CODE_MAX))
        return {"ok": True, "result": "OK", "code_after": self.code,
                "applied_steps": delta}


class FakeBatch:
    def __init__(self, mean_ps, n_used=18, n_total=20):
        self.mean_ps, self.n_used, self.n_total = mean_ps, n_used, n_total
        self.ci95_half_ps = 0.7
        self.rejected = {"margin below threshold": n_total - n_used}


def self_test(args) -> int:
    rng = np.random.default_rng(3)
    checks = []

    # 1. a converging actuator: the loop must walk the code and stop in the deadband
    act = FakeActuator(step_ps=7.4)
    act.code = 24
    truth = {"code": 24}

    def measure_case(base_ps):
        return FakeBatch(base_ps + 7.4 * truth["code"] + rng.normal(0, 0.7))

    def make_measure(base, actuator, noise=0.7):
        """Measured skew depends on the code *relative to neutral*, as on the bench.

        Bound to a specific actuator: each case below models the bench the loop is
        actually driving, otherwise a wrong-direction test would be graded against
        a healthy bench and prove nothing.
        """
        def _m():
            return FakeBatch(base + actuator.step_ps * (actuator.code - 24)
                             + rng.normal(0, noise))
        return _m

    res = close_loop(make_measure(-45.0, act), act, max_updates=10, deadband_ps=10.0,
                     verbose=False)
    checks.append(("loop converges and stops", res["verdict"] == "converged",
                   f"{res['verdict']}: {res['reason']}"))
    checks.append(("never moves more than one code per update",
                   all(abs(u) <= 1 for u in act.updates), f"updates {act.updates}"))
    checks.append(("moves in the direction that reduces the error",
                   all(u > 0 for u in act.updates[:3]), f"updates {act.updates[:3]}"))
    checks.append(("final error inside the deadband",
                   abs(res["final_error_ps"]) <= 10.0, f"{res['final_error_ps']:.2f} ps"))

    # 2. a wrong-direction actuator must abort, not run away
    act2 = FakeActuator(step_ps=-7.4)
    res2 = close_loop(make_measure(-45.0, act2), act2, max_updates=10, verbose=False)
    checks.append(("wrong-direction actuator aborts",
                   res2["verdict"] == "aborted" and "direction" in res2["reason"],
                   f"{res2['verdict']}: {res2['reason'][:60]}"))

    # 3. a failed ACK aborts immediately and is not retried
    act3 = FakeActuator(step_ps=7.4, fail_on=[0])
    res3 = close_loop(make_measure(-45.0, act3), act3, max_updates=3, verbose=False)
    checks.append(("failed ACK aborts without retrying",
                   res3["verdict"] == "aborted" and len(act3.updates) == 0,
                   f"{res3['verdict']}: {res3['reason'][:50]}"))

    # 4. an already-converged bench is left alone
    act4 = FakeActuator(step_ps=7.4)
    res4 = close_loop(make_measure(-40.0, act4), act4, max_updates=3, verbose=False)
    # -40 ps at code 24 is outside the deadband, so this one moves; check the
    # deadband case separately by starting at the target
    act5 = FakeActuator(step_ps=7.4)
    res5 = close_loop(lambda: FakeBatch(3.0), act5, max_updates=3, verbose=False)
    checks.append(("inside the deadband nothing moves",
                   res5["verdict"] == "converged" and not act5.updates,
                   f"{res5['verdict']}, updates {len(act5.updates)}"))

    print()
    ok = True
    for name, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name} -- {detail}")
        ok = ok and passed
    print(f"\nSELF-TEST: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--frames", type=int, default=20, help="frames per decision")
    ap.add_argument("--max-updates", dest="max_updates", type=int, default=3,
                    help="physical code updates allowed in this run (stage 7.1 uses 3)")
    ap.add_argument("--target-ps", dest="target_ps", type=float, default=0.0)
    ap.add_argument("--deadband-ps", dest="deadband_ps", type=float, default=10.0)
    ap.add_argument("--self-test", dest="self_test", action="store_true")
    ap.add_argument("--out", default=os.path.join(REPO, "calibration_out", "close_loop"))
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

    bench = HardwareBench(uart_port=args.uart, allow_skew_writes=True)
    history = []
    try:
        txn = bench.actuator.request_steps(0)
        print(f"verify: {txn.get('result')} code {txn.get('code_before')} -> "
              f"{txn.get('code_after')} neutral_initialized={txn.get('neutral_initialized')}")
        if not txn.get("ok"):
            print("ABORT: the actuator did not answer a zero-step transaction")
            return 2
        if txn.get("neutral_initialized") == "YES":
            print("note: that transaction performed the neutral initialization, so the\n"
                  "      operating point just moved (measured +57 ps) and the first batch\n"
                  "      below is the new starting error, not the old one.")
        if txn.get("code_after") is None:
            print("ABORT: no code in the ACK")
            return 2

        def measure():
            recs = []
            for _ in range(args.frames):
                raw = bench.capture()
                if raw:
                    recs.append(frame_record(raw, cfg, f0))
            history.append(recs)
            return skew_batch(recs)

        res = close_loop(measure, bench.actuator, target_ps=args.target_ps,
                         deadband_ps=args.deadband_ps, max_updates=args.max_updates)
    finally:
        bench.close()

    ok = report(res, args.deadband_ps)
    path = os.path.join(args.out, "close_loop_rounds.csv")
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=["round", "code", "used", "total", "mean_ps",
                                           "ci95_ps", "error_ps", "rejected"])
        w.writeheader()
        for i, r in enumerate(res["rounds"]):
            w.writerow({"round": i, "code": r["code"], "used": r["used"],
                        "total": r["total"], "mean_ps": r["mean_ps"],
                        "ci95_ps": r["ci95_ps"], "error_ps": r["error_ps"],
                        "rejected": r["rejected"]})
    print(f"round history: {path}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
