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


class HardwareBench:
    """Same ``capture()`` / ``command_skew()`` interface as the simulator."""

    def __init__(
        self,
        uart_port: str,
        bind_ip: str = HOST_IP,
        skew_bias_ps: float = 165.0,
        allow_super_fine: bool = False,
    ):
        self.rx = UdpFrameReceiver(bind_ip=bind_ip)
        self.console = UartConsole(uart_port)
        self.delay = AdcClockDelay(
            receiver_sock=self.rx.sock,
            bias_ps=skew_bias_ps,
            allow_super_fine=allow_super_fine,
            console=self.console,
        )
        self.skew_cmd_b_ps = 0.0

    def command_skew(self, delay_ps: float) -> dict:
        self.skew_cmd_b_ps = float(delay_ps)
        return self.delay.apply_skew(self.skew_cmd_b_ps)

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
