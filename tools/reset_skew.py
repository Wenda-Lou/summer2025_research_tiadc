"""Park the AD9695 sample-clock delay back at its bias (0 ps differential).

A run that ends with the skew loop at its limit leaves channel B sitting at full
delay, and the *next* run inherits that offset: with ``--open-skew`` nothing ever
writes the register, so the loop starts ~200 ps out and rejects frame after frame
on ``skew mismatch residual out of range``.  Closing a loop resets it at
iteration 0 (the state starts at 0 ps), but a measurement-only run does not.

Usage:
    python tools/reset_skew.py --uart COM5 [--bias-ps 165] [--super-fine]
"""

from __future__ import annotations

import argparse
import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM5")
    ap.add_argument("--bias-ps", dest="bias_ps", type=float, default=165.0)
    ap.add_argument("--super-fine", dest="super_fine", action="store_true")
    ap.add_argument("--force-unsafe", dest="force_unsafe", action="store_true",
                    help="override the refusal and write the delay registers anyway")
    args = ap.parse_args(argv)

    from calibration_loop.capture import CH_A, CH_B, HardwareBench

    if not args.force_unsafe:
        print("REFUSING: this tool writes the ADC delay registers over the raw UDP path.\n"
              "That path is frozen: it never programs the 0x0114 neutral code, moves\n"
              "only channel B instead of the complementary 0x0112 pair, verifies\n"
              "nothing, and resets the JESD link twice per update -- both attempts\n"
              "against this bench left it half-synced and needing a chip bring-up.\n"
              "\n"
              "Recovery instead: re-run the board bring-up (the firmware reprograms\n"
              "the AD9695 and re-establishes the link).  To program the actuator use\n"
              "the firmware's transactional skew command once it exists.\n"
              "\n"
              "--force-unsafe overrides this refusal and is for lab use only.")
        return 2

    bench = HardwareBench(uart_port=args.uart, skew_bias_ps=args.bias_ps,
                          allow_super_fine=args.super_fine, allow_skew_writes=True)
    try:
        applied = bench.command_skew(0.0)
    finally:
        bench.close()

    a, b = applied["ch_a"], applied["ch_b"]
    print(f"channel A: fine={a['fine']:>3} super={a['super_fine']:>3} "
          f"-> {a['actual_ps']:7.2f} ps")
    print(f"channel B: fine={b['fine']:>3} super={b['super_fine']:>3} "
          f"-> {b['actual_ps']:7.2f} ps")
    print(f"differential: {applied['differential_ps']:+.2f} ps "
          f"(bias {args.bias_ps:.1f} ps held on channel A)")
    print("note: each delay write makes the firmware reset the JESD204C link; "
          "the link may need a chip bring-up before captures are clean again")
    if abs(applied["differential_ps"]) > 2.0:
        print("note: differential is not ~0 -- check the bias and the register step")
        return 1
    print("dither-bias reset: safe to run a measurement-only pass now")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
