"""Capture the ADC waveform after a board calibration run, for the run viewer.

board_loop_log.py calls this after a run that ended cleanly and *before*
--park-after moves the actuator, so the frames show the calibrated state.  It
uses the console/UDP path the host tools already use (``dma -d``, ``dma -w``,
``udp`` -> 192.168.1.100:6666 as 8 x 512-byte datagrams; the same sequence as
``calibration_loop.capture.HardwareBench.capture``).  When the park succeeds it
captures again at the parked code, for a before/after comparison.  Read-only:
no register or actuator writes.

Frames are split into channels A and B with the board's rule
(``calibration_loop.estimator.deframe``: |corr(A, B)| plus the half-group lag
test; B is negated when corr < 0, as ``adc_calibration_loop.c`` does).

``<stem>_waveform.csv`` columns: state,code,frame,sample,time_ns,a_raw,b_raw,a_cal,b_cal
  state ``final``  actuator at the run's final code; ``*_cal`` is the run's final
                   digital correction applied on the PC, (raw - offset) * gain.
                   The board applies it only to its estimates, never to the ADC
                   output, so the raw frames do not contain it.
  state ``parked`` after --park-after; ``*_cal`` equals the raw codes.
"""
from __future__ import annotations

import csv
from pathlib import Path
import socket
import sys
import time

REPO = Path(__file__).resolve().parents[2]
FS_ADC_HZ = 1.3e9
UDP_PORT = 6666
PACKETS_PER_FRAME = 8
DMA_BUF_BYTES = 4095
COLUMNS = ['state', 'code', 'frame', 'sample', 'time_ns', 'a_raw', 'b_raw', 'a_cal', 'b_cal']


class UdpReceiver:
    """Collects one 8 x 512-byte frame from the board."""

    def __init__(self, port=UDP_PORT, timeout=3.0):
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.bind(('0.0.0.0', port))

    def drain(self):
        self.sock.settimeout(0.01)
        try:
            while True:
                self.sock.recvfrom(2048)
        except (socket.timeout, OSError):
            pass

    def receive(self):
        self.sock.settimeout(self.timeout)
        chunks = []
        try:
            for _ in range(PACKETS_PER_FRAME):
                chunks.append(self.sock.recvfrom(2048)[0])
        except socket.timeout:
            return None
        return b''.join(chunks)[:DMA_BUF_BYTES]

    def close(self):
        self.sock.close()


def open_receiver():
    """Bind UDP 6666; raises OSError when the port is taken."""
    return UdpReceiver()


def _console(ser, log, command, settle):
    """Send one console command, then keep whatever it printed in the human log."""
    ser.write((command + '\r').encode())
    ser.flush()
    time.sleep(settle)
    text = ser.read(ser.in_waiting or 0).decode(errors='replace')
    for line in text.replace('\r', '').split('\n'):
        if line.strip():
            log.write(line + '\n')
    log.flush()


def capture(ser, log, receiver, n_frames, retries=3):
    """Capture up to n_frames raw DMA frames; returns the list of byte strings."""
    frames = []
    for _ in range(n_frames):
        for _ in range(retries):
            receiver.drain()
            _console(ser, log, 'dma -d', 0.15)
            _console(ser, log, 'dma -w', 0.20)
            _console(ser, log, 'udp', 0.0)
            raw = receiver.receive()
            if raw:
                frames.append(raw)
                break
    # keep the console output of the last command in the log
    time.sleep(0.1)
    text = ser.read(ser.in_waiting or 0).decode(errors='replace')
    for line in text.replace('\r', '').split('\n'):
        if line.strip():
            log.write(line + '\n')
    log.flush()
    return frames


def channels(raw):
    """Raw DMA bytes -> (ch_a, ch_b) in ADC codes, split the way the board splits them."""
    if str(REPO) not in sys.path:
        sys.path.insert(0, str(REPO))
    from calibration_loop.estimator import deframe, unpack_words
    split = deframe(unpack_words(raw), tone_free=True)
    a, b = split['ch_a'], split['ch_b']
    if split['corr'] < 0:
        b = -b
    return a, b


def rows_for(state, code, frames, correction=None):
    """CSV rows for one state; correction = the run's final values, or None for raw."""
    c = correction or {}
    ga, gb = c.get('gain_corr_a', 1.0), c.get('gain_corr_b', 1.0)
    oa, ob = c.get('offset_a_codes', 0.0), c.get('offset_b_codes', 0.0)
    rows = []
    for f, raw in enumerate(frames):
        a, b = channels(raw)
        for i, (va, vb) in enumerate(zip(a, b)):
            rows.append({'state': state, 'code': code, 'frame': f, 'sample': i,
                         'time_ns': round(i / FS_ADC_HZ * 1e9, 4),
                         'a_raw': int(va), 'b_raw': int(vb),
                         'a_cal': round((va - oa) * ga, 3), 'b_cal': round((vb - ob) * gb, 3)})
    return rows


def rms_difference(rows, state, raw=False):
    """RMS of A - B over every sample of one state (cal columns unless raw)."""
    ka, kb = ('a_raw', 'b_raw') if raw else ('a_cal', 'b_cal')
    d = [float(r[ka]) - float(r[kb]) for r in rows if r['state'] == state]
    return (sum(v * v for v in d) / len(d)) ** 0.5 if d else None


def save(out_log, captures, correction):
    """Write <stem>_waveform.csv and <stem>_waveform.png beside the run log.

    captures: [(state, code, [raw, ...]), ...]; only the 'final' state is corrected.
    Returns (csv_path, png_path, rows); png_path is None when matplotlib fails.
    """
    rows = []
    for state, code, frames in captures:
        rows.extend(rows_for(state, code, frames, correction if state == 'final' else None))
    csv_path = out_log.with_name(out_log.stem + '_waveform.csv')
    with csv_path.open('w', encoding='utf-8', newline='') as fh:
        writer = csv.DictWriter(fh, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    png_path = csv_path.with_suffix('.png')
    try:
        _plot(rows, png_path)
    except Exception as exc:  # the CSV is the record; a figure failure must not lose it
        print(f'waveform figure not written: {exc}')
        png_path = None
    return csv_path, png_path, rows


def _plot(rows, path, samples=520):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    def series(state, key):
        pts = [r for r in rows if r['state'] == state and r['frame'] == 0 and r['sample'] < samples]
        return [r['time_ns'] for r in pts], [float(r[key]) for r in pts], (pts[0]['code'] if pts else None)

    fig, (top, bottom) = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
    t, a, code = series('final', 'a_cal')
    _, b, _ = series('final', 'b_cal')
    top.plot(t, a, lw=1.1, label='A (corrected)')
    top.plot(t, b, lw=1.1, label='B (corrected)')
    top.set_ylabel('ADC codes')
    top.set_title(f'Final state: actuator code {code}, final gain/offset correction applied')
    top.legend(loc='upper right')
    for state, key_a, key_b, label in (('parked', 'a_raw', 'b_raw', 'parked, no correction'),
                                        ('final', 'a_cal', 'b_cal', 'final, corrected')):
        ts, va, c = series(state, key_a)
        _, vb, _ = series(state, key_b)
        if ts:
            bottom.plot(ts, [x - y for x, y in zip(va, vb)], lw=1.1, label=f'code {c}, {label}')
    bottom.set_ylabel('A - B (codes)')
    bottom.set_xlabel('time (ns), frame 0')
    bottom.legend(loc='upper right')
    for ax in (top, bottom):
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)
