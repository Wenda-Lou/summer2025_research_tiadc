"""Regression tests for the skew actuator transaction client (phase 3).

The host no longer writes delay registers over UDP.  It asks the firmware for one
transaction and believes only the ACK, so the things worth testing are exactly
the rules that keep that safe:

  * the ACK block is parsed, including the failure case;
  * a request never asks for more than one control code;
  * a no-op request sends nothing at all;
  * local state is updated only from an ACK, never optimistically;
  * a failed transaction leaves the stored code untouched and is not retried;
  * the deadband and the code-range clip stop the controller acting on the
    +-19 ps per-frame jitter and winding up against the actuator limits.

No bench, no pyserial: the console is faked.

Usage:
    python tools/skew_actuator_test.py
"""

from __future__ import annotations

import os
import re
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop.capture import HardwareBench, SkewActuator  # noqa: E402

OK_BLOCK = (
    "\r\nSKEW-TXN 7 requested_steps=+1\r\n"
    "SKEW-TXN 7 code_before=24 code_after=25 applied_steps=1\r\n"
    "SKEW-TXN 7 nominal_differential_ps_x10=138 saturated=NO "
    "neutral_initialized=YES\r\n"
    "SKEW-TXN 7 capture_generation=412 previous_generation=411\r\n"
    "SKEW-TXN 7 result=OK\r\n"
)
FAIL_BLOCK = (
    "\r\nSKEW-TXN 8 requested_steps=+1\r\n"
    "SKEW-TXN 8 result=RECOVERY_REQUIRED stage=warmup-capture code_after=25\r\n"
)


class FakeConsole:
    def __init__(self, reply: str):
        self.reply = reply
        self.sent: list[str] = []
        self.drained = 0

    def send(self, line, settle=0.05):
        self.sent.append(line)

    def read_all(self):
        self.drained += 1
        return ""

    def read_until(self, pattern, timeout=12.0, poll=0.05):
        return self.reply, bool(re.search(pattern, self.reply))


def result(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f" -- {detail}" if detail else ""))
    return bool(ok)


def main() -> int:
    passed = []
    res = []

    ack = SkewActuator.parse_ack(OK_BLOCK)
    res.append(result("ACK parsed", ack.get("ok") is True and ack.get("transaction") == 7,
                      f"result={ack.get('result')}"))
    res.append(result("ACK carries the readback pair",
                      ack.get("code_before") == 24 and ack.get("code_after") == 25,
                      f"{ack.get('code_before')} -> {ack.get('code_after')}"))
    res.append(result("ACK carries generation and step size",
                      ack.get("capture_generation") == 412
                      and abs(ack.get("nominal_differential_ps", 0) - 13.8) < 1e-9,
                      f"gen={ack.get('capture_generation')} "
                      f"ps={ack.get('nominal_differential_ps')}"))

    fail = SkewActuator.parse_ack(FAIL_BLOCK)
    res.append(result("failure ACK is not mistaken for success",
                      fail.get("ok") is False and fail.get("stage") == "warmup-capture",
                      f"stage={fail.get('stage')}"))

    console = FakeConsole(OK_BLOCK)
    act = SkewActuator(console, neutral_code=24)
    ack = act.request_steps(5)
    res.append(result("a request is clamped to one control code",
                      ack.get("requested_steps") == 1, f"asked {ack.get('requested_steps')}"))
    res.append(result("the command sent is the firmware transaction",
                      console.sent and console.sent[-1] == "adc -cal skew step +1",
                      repr(console.sent[-1]) if console.sent else "nothing sent"))
    res.append(result("local code follows the ACK", act.code == 25, f"code={act.code}"))

    console2 = FakeConsole(OK_BLOCK)
    act2 = SkewActuator(console2, neutral_code=25)
    before = len(console2.sent)
    noop = act2.set_code(25)
    res.append(result("a no-op request sends nothing",
                      noop.get("skipped") is True and len(console2.sent) == before))

    console3 = FakeConsole(FAIL_BLOCK)
    act3 = SkewActuator(console3, neutral_code=24)
    bad = act3.set_code(25)
    res.append(result("failed transaction reported and not retried",
                      bad.get("ok") is False and len(console3.sent) == 1,
                      f"{len(console3.sent)} commands sent"))
    res.append(result("failed transaction does not move the stored code",
                      act3.code == 24, f"code={act3.code}"))

    # an unknown actuator position is read, never assumed
    console5 = FakeConsole(OK_BLOCK)
    act5 = SkewActuator(console5)
    act5.set_code(24)
    res.append(result("unknown position is learned with a zero-step transaction",
                      console5.sent and console5.sent[0] == "adc -cal skew step +0",
                      f"first command {console5.sent[:1]}"))
    res.append(result("the learned code comes from the ACK, not from the request",
                      act5.code == 25, f"code={act5.code}"))
    res.append(result("the follow-up move is the real delta (-1)",
                      len(console5.sent) > 1 and console5.sent[1] == "adc -cal skew step -1",
                      f"second command {console5.sent[1:2]}"))
    console6 = FakeConsole(OK_BLOCK)
    act6 = SkewActuator(console6)
    nope = act6.set_code(24, learn=False)
    res.append(result("learn=False refuses instead of assuming neutral",
                      nope.get("stage") == "code-unknown" and not console6.sent))

    act4 = SkewActuator(FakeConsole(OK_BLOCK), neutral_code=24)
    res.append(result("deadband swallows the jitter floor",
                      act4.error_to_steps(9.0) == 0 and act4.error_to_steps(-9.9) == 0))
    # sign check against the measurement: +1 code raises the measured skew, so a
    # negative error (B early) must be cancelled by moving UP
    res.append(result("error sign: a negative error moves the code up",
                      act4.error_to_steps(-20.0) == 1, f"{act4.error_to_steps(-20.0)}"))
    res.append(result("error sign: a positive error moves the code down",
                      act4.error_to_steps(20.0) == -1, f"{act4.error_to_steps(20.0)}"))
    res.append(result("one code at a time even for a large error",
                      act4.error_to_steps(-200.0) == 1 and act4.error_to_steps(200.0) == -1))
    act4.code = SkewActuator.CODE_MAX
    res.append(result("anti-windup: no step past the code range",
                      act4.error_to_steps(-50.0) == 0))
    act4.code = SkewActuator.CODE_MIN
    res.append(result("anti-windup at the low end", act4.error_to_steps(50.0) == 0))

    # command_skew converts ps -> code around the neutral point and goes through
    # the transaction, without constructing a serial port
    bench = object.__new__(HardwareBench)
    bench.console = FakeConsole(OK_BLOCK)
    bench.actuator = SkewActuator(bench.console, neutral_code=24)
    out = bench.command_skew(SkewActuator.HOST_STEP_PS)
    res.append(result("command_skew maps one measured step onto one code",
                      out.get("requested_steps") == 1 and out.get("ok") is True,
                      f"requested={out.get('requested_steps')}"))
    res.append(result("command_skew records the command it sent",
                      abs(bench.skew_cmd_b_ps - SkewActuator.HOST_STEP_PS) < 1e-9))

    print(f"\n{sum(res)}/{len(res)} passed")
    return 0 if all(res) else 1


if __name__ == "__main__":
    raise SystemExit(main())
