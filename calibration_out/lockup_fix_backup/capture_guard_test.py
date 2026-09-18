"""Regression tests for the capture guards that keep a run from silently freezing.

A stalled DMA does not look like a failure from the host: ``udp`` ships whatever
is in the board's ``RxBufferPtr``, so if ``dma -w`` timed out the host receives a
well-formed frame of *stale* samples.  Every downstream metric then repeats
exactly, which reads like convergence -- a 296-iteration bench run reached gain
ratio ``1.00000`` and offset mismatch ``0.000`` that way while its skew
integrator walked the delay actuator to the rail.  See
``calibration_out/frozen_run_regression_296.csv``.

These tests drive ``HardwareBench.capture`` with fake console/socket objects, so
they need no bench and no pyserial.

Usage:
    python tools/capture_guard_test.py     # exits 0 only if every case passes
"""

from __future__ import annotations

import os
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from calibration_loop import capture as C  # noqa: E402
from calibration_loop.estimator import (  # noqa: E402
    BlockEstimate,
    CalibrationState,
    ChannelEstimate,
)

OK_ECHO = ("reset completed!\r\nDMA Finished Successfully.\r\n"
           "dma -w complete.\r\n")
TIMEOUT_ECHO = ("reset completed!\r\nDMA was still busy and timed out.\r\n"
                "dma -w complete.\r\n")

FRAME_A = b"A" * 4095
FRAME_B = b"B" * 4095


class FakeRx:
    def __init__(self, frames):
        self.frames = list(frames)
        self.drains = 0

    def drain(self):
        self.drains += 1
        return 0

    def receive(self, timeout=5.0):
        return self.frames.pop(0) if self.frames else None


class FakeConsole:
    def __init__(self, echoes):
        self.echoes = list(echoes)
        self.sent: list[str] = []

    def send(self, line, settle=0.05):
        self.sent.append(line)

    def read_all(self):
        return self.echoes.pop(0) if self.echoes else ""


def fake_bench(frames, echoes, last=None):
    """A HardwareBench without __init__, so no serial port is opened."""
    bench = object.__new__(C.HardwareBench)
    bench.rx = FakeRx(frames)
    bench.console = FakeConsole(echoes)
    bench._last_frame = last
    return bench


class FakeSock:
    def __init__(self):
        self.sent: list[bytes] = []

    def sendto(self, payload, addr):
        self.sent.append(bytes(payload))


def main() -> int:
    results: list[tuple[bool, str]] = []

    def check(name, ok, detail=""):
        results.append((bool(ok), f"{name}{(' -- ' + detail) if detail else ''}"))

    # 1. the normal path still returns the frame
    b = fake_bench([FRAME_B], [OK_ECHO])
    check("fresh frame accepted", b.capture() == FRAME_B)

    # 2. a repeated frame must not be handed downstream as data
    b = fake_bench([FRAME_A, FRAME_A, FRAME_A], [OK_ECHO] * 3, last=FRAME_A)
    check("stale frame rejected (returns None)", b.capture() is None)

    # 3. the firmware's own verdict is honoured: skip the udp ask, retry, recover
    b = fake_bench([FRAME_B], [TIMEOUT_ECHO, OK_ECHO])
    got = b.capture()
    check("DMA timeout retried, then recovered", got == FRAME_B)
    check("no udp request after a reported timeout",
          b.console.sent.count("udp") == 1, f"udp sent {b.console.sent.count('udp')}x")

    # 4. persistent timeout gives up instead of feeding stale data
    b = fake_bench([FRAME_B], [TIMEOUT_ECHO] * 3)
    check("persistent timeout returns None", b.capture() is None)

    # 5. a fresh frame that merely follows a rejection is still accepted
    b = fake_bench([FRAME_B], [OK_ECHO], last=None)
    check("first capture has nothing to compare against", b.capture() == FRAME_B)

    # 6. the skew integrator cannot walk past the actuator's authority
    st = CalibrationState(skew_limit_ps=165.0)
    est = BlockEstimate(ch_a=ChannelEstimate(), ch_b=ChannelEstimate(),
                        skew_mismatch_ps=-2.603)
    for _ in range(400):
        st.update(est)
    check("skew command clamped to its limit",
          abs(st.skew_cmd_ps) <= 165.0 + 1e-9, f"{st.skew_cmd_ps:.2f} ps")
    check("saturation is recorded", st.skew_saturated > 0,
          f"{st.skew_saturated} updates")

    # 7. a delay command that changes nothing must not reach the board, because
    #    every write makes the firmware reset the JESD204C link.
    sock = FakeSock()
    delay = C.AdcClockDelay(receiver_sock=sock, bias_ps=165.0, console=None,
                            link_settle_s=0.0)
    first = delay.apply_skew(0.0)
    second = delay.apply_skew(0.0)
    check("first delay command writes both channels",
          first["wrote"] and len(sock.sent) == 2, f"{len(sock.sent)} datagrams")
    check("repeating the same command writes nothing",
          not second["wrote"] and len(sock.sent) == 2,
          f"{len(sock.sent)} datagrams after 2 calls")
    third = delay.apply_skew(2.0 * C.FINE_STEP_PS)   # two register steps
    check("a two-step change does write", third["wrote"] and len(sock.sent) > 2,
          f"{len(sock.sent)} datagrams")
    sub = delay.apply_skew(2.0 * C.FINE_STEP_PS + 0.5)
    check("a sub-step change writes nothing",
          not sub["wrote"], f"{len(sock.sent)} datagrams")
    delay.set(C.CH_B, delay.bias_ps, force=True)
    settled = delay.apply_skew(0.0)
    check("already-applied state is not re-sent", not settled["wrote"])
    check("differential reported from the encoding",
          abs(first["differential_ps"]) < 1e-9, f"{first['differential_ps']:+.2f} ps")

    width = max(len(name) for _, name in results)
    failures = 0
    for ok, name in results:
        print(f"  [{'PASS' if ok else 'FAIL'}] {name:<{width}}")
        failures += not ok
    print(f"\n{'all capture guards hold' if not failures else str(failures) + ' FAILED'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
