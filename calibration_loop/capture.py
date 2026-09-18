"""
Hardware I/O: trigger a DMA capture over the UART console, collect the frame over
UDP, and drive the AD9695 sample-clock delay.

Nothing here requires a firmware change.  The existing console already exposes
everything the loop needs::

    dma -d      reset the S2MM channel
    dma -w      capture DMA_CMD_BUF_SIZE (4095) bytes into DDR
    udp         ship the buffer to 192.168.1.100:6666 as 8 x 512-byte datagrams

and the UDP receive callback in ``ethernet.c`` already reprograms the clock
delay from a 4-byte packet.

Two traps in the current firmware, both handled here:

1. ``udp_update()`` is called only *after* ``uart_get_line()`` returns, so lwIP
   only services its receive queue once a UART line has been entered.  A clock
   delay packet sent over UDP therefore sits unprocessed until the next console
   line arrives.  :meth:`AdcClockDelay.set` sends a bare newline afterwards to
   flush it.

2. ``ad9695_adc_super_fine_delay()`` in ``ad9695_api.c`` writes to
   ``AD9695_CLK_FINE_DELAY_REG`` (0x0112) instead of
   ``AD9695_CLK_SUPER_FINE_DELAY_REG`` (0x0111), so the super-fine field is
   never programmed and the fine field gets clobbered.  Until that is fixed, run
   with ``allow_super_fine=False`` and accept the 1.725 ps quantisation.
"""

from __future__ import annotations

import socket
import re
import time
from dataclasses import dataclass

import numpy as np

BOARD_IP = "192.168.1.10"
HOST_IP = "0.0.0.0"
UDP_PORT = 6666

PACKETS_PER_FRAME = 8
PACKET_SIZE = 512
DMA_BUF_BYTES = 4095  # DMA_CMD_BUF_SIZE in baxidma.h

FINE_STEP_PS = 1.725
SUPER_FINE_STEP_PS = 0.25
FINE_MAX = 192
SUPER_FINE_MAX = 128

CLK_DELAY_OFF = 0x00
CLK_DELAY_FINE_192 = 0x04
CLK_DELAY_SUPER_FINE = 0x06

CH_A, CH_B, CH_BOTH = 1, 2, 3


class UdpFrameReceiver:
    """Collects one 8 x 512-byte frame from the board."""

    def __init__(self, bind_ip: str = HOST_IP, port: int = UDP_PORT, timeout: float = 5.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind((bind_ip, port))
        self.sock.settimeout(timeout)

    def drain(self) -> int:
        """Throw away anything left from a previous frame."""
        self.sock.settimeout(0.01)
        dropped = 0
        try:
            while True:
                self.sock.recvfrom(2048)
                dropped += 1
        except socket.timeout:
            pass
        return dropped

    def receive(self, timeout: float = 5.0) -> bytes | None:
        self.sock.settimeout(timeout)
        chunks = []
        try:
            for _ in range(PACKETS_PER_FRAME):
                data, _ = self.sock.recvfrom(2048)
                chunks.append(data)
        except socket.timeout:
            return None
        return b"".join(chunks)[:DMA_BUF_BYTES]

    def close(self) -> None:
        self.sock.close()


class UartConsole:
    """Drives the board's UART command prompt (needs ``pyserial``)."""

    def __init__(self, port: str, baud: int = 115200, timeout: float = 2.0):
        try:
            import serial  # noqa: PLC0415
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("pyserial is required: pip install pyserial") from exc
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def send(self, line: str, settle: float = 0.05) -> None:
        self.ser.write((line + "\r").encode())
        self.ser.flush()
        time.sleep(settle)

    def read_all(self) -> str:
        return self.ser.read(self.ser.in_waiting or 0).decode(errors="replace")

    def read_until(self, pattern, timeout: float = 12.0,
                   poll: float = 0.05) -> tuple[str, bool]:
        """Accumulate console output until ``pattern`` (a compiled regex) matches.

        Returns ``(text, matched)``.  The firmware's actuator transaction prints
        a bounded block ending in ``result=...`` *after* it has finished, so
        waiting for that marker is a real completion signal rather than a sleep.
        """
        deadline = time.time() + timeout
        seen = ""
        while time.time() < deadline:
            chunk = self.read_all()
            if chunk:
                seen += chunk
                if pattern.search(seen):
                    return seen, True
            time.sleep(poll)
        return seen, bool(pattern.search(seen))

    def close(self) -> None:
        self.ser.close()


@dataclass
class AdcClockDelay:
    """Maps a wanted delay in picoseconds onto the AD9695 delay registers.

    The hardware delay is one-sided (0 .. ~331 ps fine, plus 0 .. 32 ps super
    fine), so a *bias* is programmed into both channels first.  Channel B can
    then be moved either side of channel A, which is what a skew loop needs.
    """

    receiver_sock: socket.socket
    bias_ps: float = 165.0
    allow_super_fine: bool = False
    board_ip: str = BOARD_IP
    port: int = UDP_PORT
    console: UartConsole | None = None
    allow_writes: bool = False
    """Refuse to program the ADC delay registers unless explicitly enabled.

    Every write down here ends in ``jesdlink_reset()`` on the board, and this path
    does not follow the firmware's own actuator convention: it never programs the
    common digital-delay code ``0x0114`` (the firmware's readiness check requires
    ``0x0114 = 0x60`` on both channels), it moves channel B while holding A instead
    of using the complementary ``0x0112`` pair with ``raw_a + raw_b == 0xC0``, it
    verifies nothing, and it resets the link twice per update instead of once.
    Both host writes tried against this bench so far coincided with the link
    going half-synced and needing a chip bring-up to recover, so the path is
    frozen until the firmware exposes one transactional skew command.
    """

    @staticmethod
    def _encode(delay_ps: float, allow_super_fine: bool) -> tuple[int, int, int]:
        delay_ps = max(0.0, delay_ps)
        fine = int(round(delay_ps / FINE_STEP_PS))
        fine = int(np.clip(fine, 0, FINE_MAX))
        if not allow_super_fine:
            return CLK_DELAY_FINE_192, fine, 0
        rest = delay_ps - fine * FINE_STEP_PS
        if rest < 0:
            fine = max(0, fine - 1)
            rest = delay_ps - fine * FINE_STEP_PS
        super_fine = int(np.clip(round(rest / SUPER_FINE_STEP_PS), 0, SUPER_FINE_MAX))
        return CLK_DELAY_SUPER_FINE, fine, super_fine

    def set(self, channel: int, delay_ps: float) -> dict:
        if not self.allow_writes:
            raise RuntimeError(
                "ADC delay writes are frozen (AdcClockDelay.allow_writes=False): "
                "this path resets the JESD link and has twice left the bench "
                "half-synced.  Use the firmware transactional skew command, or "
                "run with bench --allow-skew-writes to re-enable it deliberately."
            )
        mode, fine, super_fine = self._encode(delay_ps, self.allow_super_fine)
        payload = bytes([mode, fine, super_fine, channel]) + bytes(60)
        self.receiver_sock.sendto(payload, (self.board_ip, self.port))
        # See module docstring, trap 1: the board only services lwIP after a
        # console line, so nudge the prompt.
        if self.console is not None:
            self.console.send("")
        time.sleep(0.05)
        return {
            "channel": channel,
            "mode": mode,
            "fine": fine,
            "super_fine": super_fine,
            "actual_ps": fine * FINE_STEP_PS + super_fine * SUPER_FINE_STEP_PS,
        }

    def apply_skew(self, skew_cmd_ps: float) -> dict:
        """Hold channel A at the bias and put channel B at bias + command."""
        a = self.set(CH_A, self.bias_ps)
        b = self.set(CH_B, self.bias_ps + skew_cmd_ps)
        return {"ch_a": a, "ch_b": b, "differential_ps": b["actual_ps"] - a["actual_ps"]}


class SkewActuator:
    """The only sanctioned way to move the ADC sample-clock delay.

    Speaks the firmware's transaction command (``adc -cal skew step +/-N``,
    routed to ``handle_adc_skew_transaction_cmd()``) and insists on its ACK:

        REQUEST -> WAIT_ACK -> (firmware does its own readback, one JESD reset,
        warm-up capture and freshness check) -> VERIFY_ACK -> update local state

    Nothing local is updated before the ACK, no write is ever repeated
    automatically, and a failed transaction is reported as a hard stop -- writing
    the actuator again is never a recovery strategy, because the host cannot tell
    a half-synced link from a healthy one without measuring again.

    The controller works in *control codes* (0..48, neutral 24), not picoseconds
    and never raw per-channel register codes.  ``HOST_STEP_PS`` is the measured
    7.4 ps per code (the firmware's nominal 13.8 ps is ~1.9x larger; see the
    constant for the measurements).  ``deadband_ps`` keeps the loop from acting on
    measurement noise rather than signal.
    """

    NEUTRAL_CODE = 24
    CODE_MIN = 0
    CODE_MAX = 48
    HOST_STEP_PS = 7.4
    """Measured differential skew per control code, in picoseconds.

    Bench-measured 2026-09-17 with the actuator initialized to neutral, paired
    same-session runs: code 25 vs 24 gave +7.17 +- 1.21 ps (95 % CI), a repeat of
    the same step +7.6 ps, code 23 vs 24 gave -8.0 ps (opposite sign), and a
    two-code move gave +15.36 ps, i.e. 7.68 ps per code -- linear, and about half
    the firmware's documented 13.8 ps (4 raw 0x0112 taps per channel in opposite
    directions at 1.725 ps each).  Either only one channel's taps move the
    measured relation, or the tap is nearer 0.9 ps than 1.725 ps; the empirical
    value is used here and the discrepancy is recorded for the firmware team.
    """
    deadband_ps = 10.0
    ack_timeout_s = 20.0
    _RESULT_RE = re.compile(r"SKEW-TXN\s+(\d+)\s+result=(\w+)")
    _LINE_RE = re.compile(r"SKEW-TXN\s+(\d+)\s+(.*)")

    def __init__(self, console: UartConsole, neutral_code: int | None = None):
        self.console = console
        self.code = int(neutral_code if neutral_code is not None else self.NEUTRAL_CODE)
        self.code_known = neutral_code is not None
        self.transactions = 0
        self.last_ack: dict = {}
        self.history: list[dict] = []

    # -- transport ----------------------------------------------------------
    def request_steps(self, delta: int, max_step: int = 1) -> dict:
        """Run one firmware transaction and return its parsed ACK.

        ``delta`` is in control codes and is clamped to ``max_step`` codes, which
        defaults to one: the closed loop is not allowed to move further until the
        step response has been measured on the bench.  Characterization passes a
        larger ``max_step`` deliberately, to check that the response is linear.
        """
        delta = int(np.clip(delta, -abs(int(max_step)), abs(int(max_step))))
        self.transactions += 1
        return self._transaction(delta)

    def _transaction(self, delta: int) -> dict:
        self.console.read_all()            # drop anything stale
        self.console.send(f"adc -cal skew step {delta:+d}", settle=0.0)
        text, matched = self.console.read_until(self._RESULT_RE, self.ack_timeout_s)
        ack = self.parse_ack(text)
        if not matched or not ack:
            ack = {"ok": False, "stage": "no-ack", "raw": text[-400:]}
        ack["requested_steps"] = delta
        ack["raw"] = text[-400:]
        self.last_ack = ack
        self.history.append(ack)
        if ack.get("ok") and "code_after" in ack:
            self.code = int(ack["code_after"])
            self.code_known = True
        return ack

    @classmethod
    def parse_ack(cls, text: str) -> dict:
        """Parse the ``SKEW-TXN`` block; the last transaction in the text wins."""
        ack: dict = {}
        txn = None
        for line in text.replace("\r", "").split("\n"):
            m = cls._LINE_RE.search(line)
            if not m:
                continue
            txn = int(m.group(1))
            for token in m.group(2).split():
                if "=" in token:
                    key, _, value = token.partition("=")
                    ack[key] = value
        if txn is None:
            return {}
        for key in ("code_before", "code_after", "applied_steps", "capture_generation",
                    "previous_generation"):
            if key in ack:
                try:
                    ack[key] = int(ack[key])
                except ValueError:
                    pass
        if "nominal_differential_ps_x10" in ack:
            try:
                ack["nominal_differential_ps"] = int(ack["nominal_differential_ps_x10"]) / 10.0
            except ValueError:
                pass
        ack["transaction"] = txn
        ack["ok"] = ack.get("result") == "OK"
        return ack

    # -- controller-side helpers -------------------------------------------
    def set_code(self, code: int, learn: bool = True) -> dict:
        """Step *at most one code* towards ``code`` and return the transaction ACK.

        One code per call is deliberate -- the closed loop may not jump further
        until each step has been measured -- so this is a single-step request and
        not absolute positioning: asking for code 24 while at 31 returns
        ``code_after = 30``.  A caller that needs an absolute position (parking the
        actuator for a repeat run) must loop on the ACK, as ``tools/skew_park.py``
        does.  ``delta == 0`` is a real no-op and writes nothing.

        With ``learn`` (the default) a not-yet-known code is read first through a
        zero-step transaction, which is also the neutral verification.  Without it
        the call refuses rather than guessing -- treating an unknown actuator
        position as "probably neutral" is how a differential skew gets introduced
        by accident.
        """
        code = int(np.clip(code, self.CODE_MIN, self.CODE_MAX))
        if not self.code_known and learn:
            self.request_steps(0)
        if not self.code_known:
            return {"ok": False, "stage": "code-unknown"}
        delta = code - self.code
        if delta == 0:
            return {"ok": True, "skipped": True, "stage": "no-op",
                    "code_after": self.code}
        return self.request_steps(delta)

    def error_to_steps(self, error_ps: float) -> int:
        """Codes to move to cancel a measured error, with deadband and anti-windup.

        Sign, from the bench measurement rather than from intuition: one code step
        *increases* the measured B-A skew by ~7.4 ps (code 25 read +7.17 ps against
        code 24, and code 23 read -8.0 ps).  To remove an error ``e`` the measured
        skew must change by ``-e``, so the move is ``-e / HOST_STEP_PS`` codes.

        Returns 0 when the error is inside the deadband, which is the point: the
        loop should not act on its own measurement noise.  The result is never
        larger than one code, and it is clipped again by the code range so the
        command cannot wind up against the actuator limits.
        """
        if not np.isfinite(error_ps) or abs(error_ps) < self.deadband_ps:
            return 0
        steps = int(round(-error_ps / self.HOST_STEP_PS))
        steps = int(np.clip(steps, -1, 1))
        if steps > 0 and self.code >= self.CODE_MAX:
            return 0
        if steps < 0 and self.code <= self.CODE_MIN:
            return 0
        return steps


class HardwareBench:
    """Same ``capture()`` / ``command_skew()`` interface as the simulator."""

    def __init__(
        self,
        uart_port: str,
        bind_ip: str = HOST_IP,
        skew_bias_ps: float = 165.0,
        allow_super_fine: bool = False,
        allow_skew_writes: bool = False,
    ):
        self.rx = UdpFrameReceiver(bind_ip=bind_ip)
        self.console = UartConsole(uart_port)
        self.delay = AdcClockDelay(
            receiver_sock=self.rx.sock,
            bias_ps=skew_bias_ps,
            allow_super_fine=allow_super_fine,
            console=self.console,
            allow_writes=allow_skew_writes,
        )
        self.skew_cmd_b_ps = 0.0
        # The sanctioned actuator path: firmware transaction + ACK.  The frozen
        # AdcClockDelay above is kept only as the documented unsafe alternative.
        self.actuator = SkewActuator(self.console)

    def command_skew(self, delay_ps: float) -> dict:
        """Move the sample-clock delay through the firmware transaction.

        The command arrives in picoseconds because that is what the estimator
        measures; it is converted with the *nominal* 13.8 ps per control code and
        executed as at most one code of movement, with the firmware's readback in
        the ACK as the only source of truth.  A failed transaction is returned,
        never retried: writing the actuator again is not a recovery strategy.
        """
        self.skew_cmd_b_ps = float(delay_ps)
        if not self.actuator.code_known:
            # Learn the current code from a zero-step transaction: the ACK carries
            # code_before/code_after even when nothing moves.
            self.actuator.request_steps(0)
        target = int(round(SkewActuator.NEUTRAL_CODE
                           + delay_ps / SkewActuator.HOST_STEP_PS))
        return self.actuator.set_code(target)

    def capture(self, n_words: int | None = None, retries: int = 3) -> bytes | None:
        """Reset DMA, grab a frame, ship it, collect it.

        The reset-settle-transfer order mirrors ``adc_capture_frame()`` in
        ``butils.c``, which is the sequence already proven on this board.
        """
        for _ in range(retries):
            self.rx.drain()
            self.console.send("dma -d", settle=0.15)
            self.console.send("dma -w", settle=0.20)
            self.console.send("udp", settle=0.0)
            frame = self.rx.receive(timeout=3.0)
            if frame is not None:
                return frame
        return None

    def close(self) -> None:
        self.rx.close()
        self.console.close()
