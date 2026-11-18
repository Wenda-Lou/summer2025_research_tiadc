"""Download the final signal snapshot after `adc -cal loop N`.

    python tools/board_loop/board_signal_output.py --uart COM5 --out calibration_out/signal.csv

No waveform parameters or references are supplied. This reads saved RAM only:
it does not capture, recalibrate, actuate, or stop the physical dither source.
A non-cancelled snapshot is retained and explicitly reported with exit code 1.
CSV/JSON are embedded in an HTML waveform viewer and opened after downloading;
--no-viewer skips generating/opening the page.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import time


def fields(line):
    return dict(word.split('=', 1) for word in line.split() if '=' in word)


class SignalDecoder:
    def __init__(self):
        self.meta = None
        self.rows = []
        self.done = False

    def feed(self, line):
        line = line.strip()
        if line.startswith('uart-cmd$:'):
            line = line.split(':', 1)[1].strip()
        if self.done:
            raise ValueError('data after signal trailer')
        if line.startswith('LOOP-SIGNAL-BEGIN '):
            if self.meta is not None:
                raise ValueError('duplicate signal header')
            self.meta = fields(line)
            if self.meta.get('version') != '1' or self.meta.get('coefficients') != 'frozen':
                raise ValueError('unsupported signal format')
            if not 512 <= int(self.meta['samples']) <= 4096:
                raise ValueError('invalid signal length')
            rate = float(self.meta['sample_rate_hz'])
            if not math.isfinite(rate) or rate <= 0:
                raise ValueError('invalid sample rate')
        elif line.startswith('LOOP-SIGNAL-DATA '):
            if self.meta is None:
                raise ValueError('signal data without header')
            index, a, b = (int(word) for word in line.split()[1].split(','))
            if index != len(self.rows) or index >= int(self.meta['samples']):
                raise ValueError('missing, duplicate or excess signal sample')
            self.rows.append((index, a / 1000.0, b / 1000.0))
        elif line.startswith('LOOP-SIGNAL-END '):
            end = fields(line)
            if end.get('result') != 'OK':
                raise ValueError('board refused signal export: ' + end.get('reason', 'unknown'))
            if self.meta is None or int(end['samples']) != len(self.rows) or \
                    int(self.meta['samples']) != len(self.rows) or end.get('status') != self.meta['status']:
                raise ValueError('incomplete or inconsistent signal snapshot')
            self.done = True

    def save(self, path):
        if not self.done:
            raise ValueError('signal snapshot is incomplete')
        path = Path(path)
        # Exclusive creation keeps a previous signal snapshot intact.
        if path.exists() or path.with_suffix('.json').exists():
            raise FileExistsError('signal CSV or JSON already exists')
        with path.open('x', encoding='utf-8', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(('sample', 'time_s', 'output_a_codes', 'output_b_codes'))
            rate = float(self.meta['sample_rate_hz'])
            writer.writerows((i, i / rate, a, b) for i, a, b in self.rows)
        meta = {**self.meta, 'samples': len(self.rows),
                'dither_cancelled': self.meta['status'] == 'cancelled',
                'source': 'frozen_board_signal_snapshot'}
        with path.with_suffix('.json').open('x', encoding='utf-8') as stream:
            stream.write(json.dumps(meta, indent=2) + '\n')
        return meta


def read_saved_signal(port, timeout):
    """Read the saved snapshot on an already-open console; no capture/writes."""
    decoder = SignalDecoder()
    port.write(b'adc -cal loop output\r')
    port.flush()
    deadline = time.monotonic() + timeout
    while not decoder.done and time.monotonic() < deadline:
        line = port.readline().decode('ascii', errors='replace').strip()
        if 'Command Error' in line:
            raise ValueError('firmware rejected loop output; rebuild/download the matching ELF')
        decoder.feed(line)
    if not decoder.done:
        raise ValueError('timed out; no complete signal snapshot received')
    return decoder


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--uart', required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--no-viewer', action='store_true', help='do not write/open the HTML waveform viewer')
    args = ap.parse_args(argv)
    if args.out.suffix.lower() != '.csv' or args.timeout <= 0:
        ap.error('--out must be a .csv path; --timeout must be positive')
    if args.out.exists() or args.out.with_suffix('.json').exists():
        ap.error('output CSV or JSON already exists; choose a new name')
    try:
        import serial
        with serial.Serial(args.uart, 115200, timeout=0.5) as port:
            time.sleep(0.2)
            port.reset_input_buffer()
            decoder = read_saved_signal(port, args.timeout)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        meta = decoder.save(args.out)
        print(f"{args.out}: {meta['samples']} samples, status={meta['status']}")
        if not args.no_viewer:
            if __package__:
                from .loop_viewer_export import write_viewer
            else:
                from loop_viewer_export import write_viewer
            try:
                print(f'viewer: {write_viewer(args.out, open_browser=True)}')
            except (OSError, ValueError) as exc:
                print(f'viewer not written: {exc}; CSV/JSON retained')
        return 0 if meta['dither_cancelled'] else 1
    except (ImportError, OSError, ValueError, KeyError) as exc:
        print(f'signal export failed: {exc}')
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
