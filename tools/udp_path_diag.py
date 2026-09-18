"""One-shot bench UDP/UART path diagnostic.

Answers, in a single run:
  1. Which COM port is the board (by USB descriptor)?
  2. Does the board's console answer at 115200?
  3. What does the firmware say for `dma -d` / `dma -w` / `udp`?
  4. Do any UDP datagrams arrive on 0.0.0.0:6666 while `udp` runs?

Usage:
    python tools/udp_path_diag.py --uart COM4
    python tools/udp_path_diag.py --uart COM4 --no-commands   # listen only
"""

from __future__ import annotations

import argparse
import socket
import sys
import time

UDP_PORT = 6666
PACKETS_PER_FRAME = 8
PACKET_SIZE = 512
BAUD = 115200


def list_ports() -> None:
    try:
        from serial.tools import list_ports
    except ImportError:
        print("pyserial missing: pip install pyserial")
        return
    print("=== serial ports ===")
    for p in list_ports.comports():
        print(f"  {p.device:<8} {p.description}")
        if p.hwid:
            print(f"           hwid={p.hwid}")
    print()


def open_console(port: str):
    import serial
    ser = serial.Serial(port, BAUD, timeout=1.0)
    time.sleep(0.3)
    ser.reset_input_buffer()
    return ser


def pump(ser, seconds: float) -> str:
    """Read whatever the console emits for `seconds`."""
    end = time.time() + seconds
    out = []
    while time.time() < end:
        try:
            chunk = ser.read(4096)
        except Exception as exc:  # noqa: BLE001
            out.append(f"<read error: {exc}>")
            break
        if chunk:
            out.append(chunk.decode(errors="replace"))
    return "".join(out)


def collect(sock: socket.socket, seconds: float):
    """Return (packet_count, total_bytes, first_payload_len, senders)."""
    sock.settimeout(seconds)
    n, total, first, senders = 0, 0, None, set()
    deadline = time.time() + seconds
    while time.time() < deadline:
        try:
            data, addr = sock.recvfrom(2048)
        except socket.timeout:
            break
        except OSError as exc:
            print(f"  <recv error: {exc}>")
            break
        n += 1
        total += len(data)
        senders.add(addr[0])
        if first is None:
            first = len(data)
    return n, total, first, senders


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--uart", default="COM4", help="board UART port")
    ap.add_argument("--listen", type=float, default=3.0,
                    help="seconds to listen for UDP after `udp` (default 3)")
    ap.add_argument("--no-commands", action="store_true",
                    help="listen only; do not send the DMA sequence")
    args = ap.parse_args(argv)

    list_ports()

    print("=== UDP socket ===")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    try:
        sock.bind(("0.0.0.0", UDP_PORT))
    except OSError as exc:
        print(f"  BIND FAILED on {UDP_PORT}: {exc}")
        print("  -> another process holds the port (close any running "
              "udp_receiver.py / probe run)")
        return 2
    print(f"  bound 0.0.0.0:{UDP_PORT} OK")

    if args.no_commands:
        print(f"\n=== listening {args.listen:.0f}s (no commands sent) ===")
        n, total, first, senders = collect(sock, args.listen)
        print(f"  packets={n} bytes={total} first_len={first} senders={senders or '{}'}")
        sock.close()
        return 0 if n else 1

    print("\n=== UART console ===")
    try:
        ser = open_console(args.uart)
    except Exception as exc:  # noqa: BLE001
        print(f"  OPEN FAILED on {args.uart}: {exc}")
        sock.close()
        return 2
    print(f"  opened {args.uart} @ {BAUD} OK")

    ser.write(b"\r")
    banner = pump(ser, 1.5)
    print(f"  banner/newline response ({len(banner)} chars):")
    print("    " + (banner.strip().replace("\n", "\n    ") or "<nothing>"))

    total_packets = 0
    for cmd in ("dma -d", "dma -w", "udp"):
        ser.reset_input_buffer()
        ser.write((cmd + "\r").encode())
        ser.flush()
        resp = pump(ser, 0.8)
        n, nbytes, first, senders = collect(sock, args.listen if cmd == "udp" else 0.05)
        total_packets += n
        print(f"\n  >>> {cmd!r}")
        print(f"      uart reply ({len(resp)} chars): "
              f"{resp.strip().replace(chr(10), ' | ') or '<nothing>'}")
        if cmd == "udp":
            print(f"      udp packets={n} bytes={nbytes} "
                  f"first_len={first} senders={senders or '{}'}")

    print("\n=== verdict ===")
    if total_packets == 0:
        print("  NO UDP arrived. Board-side or network-side fault.")
        print("  - did the console reply to `udp` at all? if 'nothing', the "
              "UART port is wrong or firmware is not running")
        print("  - if console replied but no packets: check the Windows "
              "firewall for inbound UDP 6666 (profiles Domain/Private are ON)")
    else:
        print(f"  UDP path WORKS ({total_packets} packets). "
              f"expect {PACKETS_PER_FRAME} x {PACKET_SIZE} B per frame")

    ser.close()
    sock.close()
    return 0 if total_packets else 1


if __name__ == "__main__":
    sys.exit(main())
