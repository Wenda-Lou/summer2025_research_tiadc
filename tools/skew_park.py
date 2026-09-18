"""Park the skew actuator at a chosen control code, through the sanctioned path.

The counterpart of ``tools/reset_skew.py``: that one drives the frozen raw-UDP
delay write and refuses by default.  This one uses the firmware transaction
(``adc -cal skew step +/-N``) -- the only route with an ACK, a readback and one
JESD reset per move -- and loops one code at a time until the reported code is the
target, which ``capture.SkewActuator.set_code`` deliberately does not do by itself.

Why park at all: the pair of runs BENCH_GUIDE section 6 calls for is only a matched
pair if both start from the same operating point.  A run that inherits the previous
run's converged code (31, ~-10 ps) cannot be compared with one that started from
the neutral code (24, ~-44 ps) -- the convergence curve would differ for a reason
that has nothing to do with the flag under test.

    python tools/skew_park.py --uart COM5                 # park at code 24 (neutral)
    python tools/skew_park.py --uart COM5 --code 31       # back to the converged code
    python tools/skew_park.py --uart COM5 --verify-frames 0   # no measurement

Every move requires ``result=OK``; a failed transaction stops the run and is never
retried (writing the actuator again is not a recovery strategy).
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from skew_step_characterize import frame_record           # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--code", type=int, default=24,
                    help="target control code (24 is neutral)")
    ap.add_argument("--verify-frames", dest="verify_frames", type=int, default=20,
                    help="frames measured afterwards to confirm the state; 0 to skip")
    ap.add_argument("--max-moves", dest="max_moves", type=int, default=48)
    args = ap.parse_args(argv)

    from calibration_loop.capture import HardwareBench
    from calibration_loop.dither import DitherConfig
    from calibration_loop.estimator import skew_batch

    cfg = DitherConfig()
    cfg.validate()
    f0 = cfg.sig_cycles / cfg.n_adc_period

    bench = HardwareBench(uart_port=args.uart, allow_skew_writes=True)
    try:
        txn = bench.actuator.request_steps(0)
        print(f"read: {txn.get('result')} code {txn.get('code_before')} -> "
              f"{txn.get('code_after')} "
              f"neutral_initialized={txn.get('neutral_initialized')}")
        if not txn.get("ok") or txn.get("code_after") is None:
            # Never guess why: the firmware's early-bail ACK carries a stage and only
            # that stage's fields, so print all of them.  `code_before` appears at
            # stage=write, `code_after` at stage=warmup-capture (i.e. the write
            # succeeded and the *capture* failed), `generation` at
            # stage=fresh-frame, and nothing at stage=neutral-init.
            print(f"FAILED: result={txn.get('result')} stage={txn.get('stage')} "
                  f"code_before={txn.get('code_before')} "
                  f"code_after={txn.get('code_after')} "
                  f"generation={txn.get('generation')}")
            print("Nothing is retried -- re-running a failed transaction is not a "
                  "recovery strategy.\nCheck the capture path read-only first "
                  "(`python -m calibration_loop.run_calibration probe --uart ...`); "
                  "if the DMA link is down, re-run the board bring-up.")
            print(f"raw ACK: {txn.get('raw', '').strip()!r}")
            return 2
        if txn.get("neutral_initialized") == "YES":
            print("note: that transaction performed the neutral initialization, so the\n"
                  "      operating point just moved (measured +57 ps)")

        target = int(np.clip(args.code, bench.actuator.CODE_MIN, bench.actuator.CODE_MAX))
        moves = 0
        while bench.actuator.code != target:
            if moves >= args.max_moves:
                print(f"ABORT: still at {bench.actuator.code} after {moves} moves")
                return 2
            step = int(np.sign(target - bench.actuator.code))
            ack = bench.actuator.request_steps(step)
            print(f"  move {step:+d} -> {ack.get('result')} "
                  f"code {ack.get('code_before')} -> {ack.get('code_after')} "
                  f"nominal {ack.get('nominal_differential_ps')} ps   "
                  f"generation {ack.get('capture_generation')}")
            if not ack.get("ok"):
                print(f"ABORT: transaction failed ({ack.get('result')}, "
                      f"stage={ack.get('stage')}) -- not retrying")
                return 2
            moves += 1
        print(f"parked at code {target} after {moves} moves, "
              f"{bench.actuator.transactions} transactions")

        if args.verify_frames:
            recs = []
            for _ in range(args.verify_frames):
                raw = bench.capture()
                if raw:
                    recs.append(frame_record(raw, cfg, f0))
            batch = skew_batch(recs)
            print(f"measured after parking: {batch.mean_ps:+.2f} +- "
                  f"{batch.ci95_half_ps:.2f} ps over {batch.n_used}/{batch.n_total} "
                  f"frames  {dict(batch.rejected)}")
    finally:
        bench.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
