"""Run dither calibration; save all frame records, plots and a compact UART log.

Close the serial terminal first. One PC command calibrates and downloads:
    python tools/board_loop_log.py --uart COM7 --out calibration_out/dither_run.log
Download the last run without recalibrating:
    python tools/board_loop_log.py --uart COM7 --cmd "adc -cal loop export"

Each run gets its own folder: --out calibration_out/dither_run.log saves
calibration_out/dither_run/dither_run.log, .csv, .json, .png, .svg and .html
(a reused name gets dither_run_<timestamp>/; --flat keeps the old flat layout).
The .html is tools/loop_viewer.html with this run embedded, opened in the
browser after the export; --no-viewer skips it.
--park-after CODE returns the skew actuator to CODE after a run that ended
cleanly, through the firmware's ACK-checked `adc -cal loop test park CODE`;
it is never sent after a timeout, error or RECOVERY_REQUIRED.
PC mode streams every record into CSV and PNG/SVG curves. Data packets are
kept out of the human UART log. Requires the matching firmware and matplotlib.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys
import time

if __package__:
    from .loop_result_export import CSV_COLUMNS, WIRE_COLUMNS, canonical, fields, row_to_units, save_results
else:
    from loop_result_export import CSV_COLUMNS, WIRE_COLUMNS, canonical, fields, row_to_units, save_results


def read_until(ser, log, marker, timeout, lines, writer=None, data_file=None):
    deadline = time.monotonic() + timeout
    buf = ''
    data_rows = 0
    test_rows = 0
    header_seen = False
    while time.monotonic() < deadline:
        chunk = ser.read(ser.in_waiting or 1).decode(errors='replace')
        if not chunk:
            continue
        buf += chunk
        while '\n' in buf:
            raw, buf = buf.split('\n', 1)
            raw = raw.rstrip('\r')
            line = canonical(raw)
            lines.append(line)
            is_test_row = line.startswith('LOOP-TEST-ROW ')
            is_test_capture = line.startswith('LOOP-TEST-CAPTURE ')
            if writer is not None and (is_test_row or is_test_capture):
                test_rows += 1
                row = fields(line)
                row['capture'] = str(test_rows)
                writer.writerow(row_to_units(row))
                data_file.flush()
            if line.startswith('LOOP-EXPORT-HEADER '):
                header_seen = line[len('LOOP-EXPORT-HEADER '):].split(',') == WIRE_COLUMNS
                if not header_seen:
                    raise ValueError('unsupported export schema')
            if (not line.startswith('LOOP-EXPORT-') or 'result=ERROR' in line) and not is_test_row:
                log.write(raw + '\n')
                log.flush()
            if line.startswith('LOOP-EXPORT-DATA '):
                if writer is not None:
                    if not header_seen:
                        raise ValueError('data arrived without its export header')
                    values = next(csv.reader([line[len('LOOP-EXPORT-DATA '):]]))
                    if len(values) != len(WIRE_COLUMNS):
                        raise ValueError('damaged export row')
                    writer.writerow(row_to_units(dict(zip(WIRE_COLUMNS, values))))
                    data_file.flush()
                data_rows += 1
                if data_rows % 200 == 0:
                    print(f'Export: {data_rows} rows received')
            elif (not line.startswith('LOOP-EXPORT-') or 'result=ERROR' in line) and not is_test_row:
                row = fields(line)
                if not line.startswith('LOOP it=') or row.get('st') != 'ok' or (
                        int(row.get('it', '0')) + 1) % 20 == 0:
                    print(raw)
            if line.startswith(marker):
                return 'error' if 'result=ERROR' in line else 'ok'
            if 'Command Error' in line:
                return 'error'
    if buf and not canonical(buf).startswith('LOOP-EXPORT-'):
        log.write(buf + '\n')
        log.flush()
    return 'timeout'


def available_output(path):
    """Keep previous scientific runs when an output stem is reused."""
    def exists(p):
        return any(p.with_suffix(suffix).exists() for suffix in ('.log', '.csv', '.json', '.png', '.svg'))
    if not path.exists() and not exists(path):
        return path
    stem = path.stem + time.strftime('_%Y%m%d_%H%M%S')
    candidate = path.with_name(stem + (path.suffix or '.log'))
    index = 1
    while candidate.exists() or exists(candidate):
        candidate = path.with_name(f'{stem}_{index}' + (path.suffix or '.log'))
        index += 1
    return candidate


def run_folder_output(path):
    """Give each run its own folder: calibration_out/<stem>/<stem>.log plus .csv/.json/.png/.svg/.html.

    A path already inside a folder of its own stem is used as is.  A folder that already holds a
    run is never reused: the new run gets <stem>_<timestamp>/ instead.
    """
    suffix = path.suffix or '.log'
    folder = path.parent if path.parent.name == path.stem else path.parent / path.stem
    if folder.exists() and any(folder.iterdir()):
        stamp = folder.name + time.strftime('_%Y%m%d_%H%M%S')
        candidate = folder.with_name(stamp)
        index = 1
        while candidate.exists() and any(candidate.iterdir()):
            candidate = folder.with_name(f'{stamp}_{index}')
            index += 1
        folder = candidate
    return folder / (folder.name + suffix)


def park_actuator(ser, log, target, ended, lines, timeout=120.0):
    """Return the skew actuator to `target` after a run; never after a fault.

    Uses only the firmware's ACK-checked park (one code per transaction) and
    believes it only if LOOP-TEST-END is OK, every LOOP-SKEW-TXN is result=OK
    and the last one reports code_after=target.  Returns (ok, message).
    """
    if ended != 'ok':
        return False, f'skipped: the run ended with {ended}, so the board may still be busy'
    if any('RECOVERY_REQUIRED' in line for line in lines):
        return False, 'skipped: the run reported RECOVERY_REQUIRED; restart the firmware first'
    command = f'adc -cal loop test park {target}'
    log.write(f'# park-after: {command}\n')
    ser.write((command + '\r').encode())
    ser.flush()
    park_lines = []
    result = read_until(ser, log, 'LOOP-TEST-END', timeout, park_lines)
    txns = [fields(line) for line in park_lines if line.startswith('LOOP-SKEW-TXN ')]
    if result != 'ok':
        return False, f'failed: park ended with {result}'
    if not txns or any(t.get('result') != 'OK' for t in txns):
        return False, 'failed: a skew transaction did not ACK result=OK'
    if txns[-1].get('code_after') != str(target):
        return False, f"failed: code_after={txns[-1].get('code_after')}, wanted {target}"
    moves = sum(1 for t in txns if t.get('steps') != '0')
    return True, f'code={target} after {moves} moves, every transaction result=OK'


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--uart', default='COM7')
    ap.add_argument('--cmd', default='adc -cal loop 300',
                    help='default: adc -cal loop 300; use adc -cal loop export to download only')
    ap.add_argument('--wave', type=int, choices=(2, 3),
                    help='legacy firmware compatibility only; omit for reference-free calibration')
    ap.add_argument('--out', help='run name as a .log path; the run is saved in a folder of that name '
                                  '(calibration_out/x.log -> calibration_out/x/x.log, .csv, .json, .png, .svg, .html)')
    ap.add_argument('--flat', action='store_true',
                    help='old layout: save the files directly beside --out instead of in a run folder')
    ap.add_argument('--timeout', type=float, default=1800.0)
    ap.add_argument('--no-export', action='store_true', help='UART progress only; no CSV or plots')
    ap.add_argument('--no-viewer', action='store_true',
                    help='do not write/open <stem>.html (loop_viewer.html with this run embedded)')
    ap.add_argument('--park-after', type=int, metavar='CODE',
                    help='after a run that ended cleanly, return the skew actuator to CODE (0..48) '
                         'with adc -cal loop test park CODE; every move must ACK result=OK')
    args = ap.parse_args(argv)
    if args.park_after is not None and not 0 <= args.park_after <= 48:
        print('--park-after must be a control code from 0 to 48')
        return 2
    tokens = args.cmd.split()
    if tokens[:3] != ['adc', '-cal', 'loop'] or any(c in args.cmd for c in '\r\n'):
        print("refusing: --cmd must be one 'adc -cal loop ...' command")
        return 2
    if args.timeout <= 0:
        print('--timeout must be positive')
        return 2
    out = Path(args.out or Path('calibration_out') / time.strftime('board_loop_%Y%m%d_%H%M%S.log'))
    if out.suffix.lower() in ('.csv', '.json', '.png', '.svg'):
        print('--out must name a UART log, not a data/image file')
        return 2
    chosen = available_output(out) if args.flat else run_folder_output(out)
    if chosen != out:
        print(f'Saving new run as: {chosen}')
    out = chosen
    explicit_stream = tokens[-1:] == ['stream']
    base = tokens[:-1] if explicit_stream else tokens
    regular = (len(base) == 4 and base[3].isdigit() or
               len(base) == 5 and base[3] == 'open' and base[4].isdigit())
    test_capture = tokens[3:4] == ['test'] and tokens[4:5] in (['baseline'], ['run'], ['step'])
    test_data = test_capture and not args.no_export
    streaming = regular and not args.no_export
    if args.park_after is not None and not regular:
        print('--park-after only follows a calibration run (adc -cal loop [open] N)')
        return 2
    if explicit_stream and (not regular or args.no_export):
        print('stream requires a calibration run with export enabled')
        return 2
    if streaming:
        args.cmd = ' '.join(base + ['stream'])
    if test_data and tokens[-1:] != ['full']:
        args.cmd += ' full'
    exporting = tokens[3:] == ['export']
    try:
        import serial
        if streaming or exporting or test_data:
            import matplotlib
            matplotlib.use('Agg')
    except ImportError:
        print('pyserial and matplotlib are required: pip install pyserial matplotlib')
        return 2
    marker = ('LOOP-EXPORT-END' if exporting else 'LOOP-TEST-END' if
              tokens[3:4] == ['test'] else 'LOOP-WAVE' if
              tokens[3:4] == ['wave'] else 'LOOP-STATUS')
    ser = None
    data_file = None
    lines = []
    try:
        out.parent.mkdir(parents=True, exist_ok=True)
        ser = serial.Serial(args.uart, 115200, timeout=0.5)
        time.sleep(0.2)
        ser.reset_input_buffer()

        def send(command):
            ser.write((command + '\r').encode())
            ser.flush()

        with out.open('w', encoding='utf-8', newline='\n') as log:
            writer = None
            if streaming or exporting or test_data:
                data_file = out.with_suffix('.csv').open('w', encoding='utf-8', newline='')
                writer = csv.DictWriter(data_file, fieldnames=CSV_COLUMNS)
                writer.writeheader()
                data_file.flush()
            log.write(f'# cmd: {args.cmd}\n# port: {args.uart}\n'
                      f'# start: {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
            ended = 'ok'
            if args.wave is not None:
                send(f'adc -cal loop wave {args.wave}')
                ended = read_until(ser, log, 'LOOP-WAVE', 10.0, lines)
            if ended == 'ok':
                send(args.cmd)
                ended = read_until(ser, log, marker, args.timeout, lines, writer, data_file)
            if ended != 'ok':
                log.write(f'# STOPPED: {ended}\n')
            if args.park_after is not None:
                park_ok, park_message = park_actuator(ser, log, args.park_after, ended, lines)
                log.write(f'# park-after: {park_message}\n')
        if data_file is not None:
            data_file.close()
            data_file = None
        print(f'log: {out}')
        if args.park_after is not None:
            print(f'park-after {args.park_after}: {park_message}')
            if not park_ok and ended == 'ok':
                print('WARNING: the actuator was NOT parked; check LOOP-SKEW-TXN in the log '
                      'before the next run')
        park_failed = args.park_after is not None and not park_ok
        test_ended = test_data and any(line.startswith('LOOP-TEST-END ') for line in lines)
        if ended != 'ok' and not test_ended:
            print(f'stopped: {ended}; UART log retained')
            return 1
        if exporting or regular and not args.no_export or test_data:
            if test_data:
                if __package__:
                    from .axis_result_export import save_test_results
                else:
                    from axis_result_export import save_test_results
                csv_path, json_path, summary = save_test_results(lines, out)
            else:
                csv_path, json_path, summary = save_results(lines, out)
            print(f"RESULT qualified={summary['qualified']}/{summary['requested']} "
                  f"rejected={summary['rejected']} code={summary['actuator_code']} "
                  f"tail={summary['tail']['window']} (feedback {summary['tail']['feedback']})")
            for key, label, scale in (('gain_error_ppm', 'gain (%)', 1e4),
                                      ('offset_error_codes', 'offset (codes)', 1),
                                      ('skew_error_ps', 'skew (ps)', 1)):
                stats = summary['tail'][key]
                mean = f"{stats['mean']/scale:.5g}" if stats['mean'] is not None else 'n/a'
                sd = f"{stats['sd']/scale:.5g}" if stats['sd'] is not None else 'n/a'
                print(f'  {label}: mean={mean} SD={sd}')
            print(f'csv: {csv_path}\njson: {json_path}')
            print(f"png: {summary['plots']['png']}\nsvg: {summary['plots']['svg']}")
            if not args.no_viewer:
                if __package__:
                    from .loop_viewer_export import write_viewer
                else:
                    from loop_viewer_export import write_viewer
                try:
                    # The run's own page: loop_viewer.html with this CSV embedded, opened now.
                    print(f'viewer: {write_viewer(csv_path, open_browser=True)}')
                except (OSError, ValueError) as exc:
                    print(f'viewer not written: {exc}')
            if not summary['history_complete']:
                print(f"History incomplete: {summary['dropped_rows']} earlier rows unavailable")
            if summary.get('assessment') is not None:
                print('HOLD assessment: ' + ('PASS' if summary['assessment']['pass'] else 'FAIL'))
            elif not summary['run_healthy']:
                print('Run incomplete, aborted or missing finite residuals; see JSON/log')
            return 0 if summary['run_healthy'] and summary['history_complete'] and not park_failed else 1
        return 1 if park_failed else 0
    except (OSError, ValueError) as exc:
        print(f'error: {exc}; UART log: {out}')
        return 1
    except KeyboardInterrupt:
        print(f'interrupted; UART log: {out}')
        return 130
    finally:
        if data_file is not None:
            data_file.close()
        if ser is not None:
            ser.close()
        # A run that failed before writing anything (e.g. the COM port was busy)
        # must not leave an empty run folder behind.
        if out.parent.is_dir() and not any(out.parent.iterdir()):
            out.parent.rmdir()


if __name__ == '__main__':
    sys.exit(main())
