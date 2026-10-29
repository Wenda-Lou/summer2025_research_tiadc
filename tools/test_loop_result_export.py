"""Offline export/serial protocol checks. No board or serial port is opened."""
import contextlib
import csv
import io
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from board_loop_log import main
from loop_result_export import WIRE_COLUMNS, decode_results
from loop_result_plot import build_figure


def packet(n=3, run_id=7):
    lines = [f'LOOP-EXPORT-BEGIN version=1 run_id={run_id} rows={n} total_rows={n} dropped=0',
             f'LOOP-EXPORT-META run_id={run_id} mode=closed waveform=dither_only requested={n} '
             f'qualified={n} captures={n} rejected=0 code=12 skew_abort=0 '
             'corr_a_ppm=10000 corr_b_ppm=-10000 offa_mc=3000 offb_mc=-5000 '
             'gsrc=dither_mag skew_route=dither_phase feedback=active',
             'LOOP-EXPORT-HEADER ' + ','.join(WIRE_COLUMNS)]
    for i in range(n):
        error = 1000 if i % 2 else -1000
        lines.append(f'LOOP-EXPORT-DATA {i+1},{i},ok,{error},{error},-209,'
                     '10000,-10000,3000,-5000,12,none,0,0,0,nan,0,44,0,1700')
    return lines + [f'LOOP-EXPORT-END result=OK run_id={run_id} rows={n}']


def stream_packet(n=3, run_id=7):
    saved = packet(n, run_id)
    return [f'LOOP-EXPORT-BEGIN version=2 run_id={run_id} mode=stream', saved[2],
            *saved[3:-1], saved[1],
            f'LOOP-EXPORT-END result=OK run_id={run_id} rows={n} total_rows={n} dropped=0']


class Port:
    instances = []
    fail = False
    silent = False
    incomplete = False
    partial = False

    def __init__(self, *args, **kwargs):
        self.output = b''
        self.commands = []
        self.closed = False
        self.n = 3
        self.instances.append(self)

    @property
    def in_waiting(self):
        return min(11, len(self.output))

    def write(self, data):
        command = data.decode().strip()
        self.commands.append(command)
        if self.silent:
            return
        if self.fail:
            reply = 'Command Error: unavailable\r\n'
        elif command == 'adc -cal loop export':
            lines = packet(self.n)
            if self.incomplete:
                lines[1] = lines[1].replace(f'requested={self.n}', f'requested={self.n+1}')
            reply = '\r\n'.join(lines) + '\r\n'
        elif command.startswith('adc -cal loop test run offset '):
            lines = []
            index = 0
            for phase, axes in (('pre', 0), ('train', 1), ('hold', 0)):
                lines.append(f'LOOP-TEST-PHASE phase={phase} axes={axes} budget=4 code=12')
                for attempt in range(4):
                    state = 0 if phase == 'pre' else 32000
                    error = 32000 if phase == 'pre' else 0
                    lines.append(f'LOOP-TEST-ROW phase={phase} attempt={attempt} LOOP it={index} st=ok '
                                 f'used_gain_ppm=0 doff_mc={error} skew_cps=0 code=12 '
                                 f'corr_a_ppm=0 corr_b_ppm=0 offa_mc=0 offb_mc={state} '
                                 'clips=0 gsrc=dither_mag skew_route=dither_phase')
                    index += 1
                lines.append(f'LOOP-TEST-SUMMARY phase={phase} attempts=4 accepted=4 code=12')
            reply = '\r\n'.join(lines + ['LOOP-TEST-END result=OK']) + '\r\n'
        elif command.startswith('adc -cal loop ') and command.split()[-1] == 'stream':
            self.n = int(command.split()[-2])
            if self.partial:
                self.output += ('LOOP-START stream=1 run_id=7\r\n' +
                                '\r\n'.join(stream_packet(10)[:7]) + '\r\n').encode()
                return
            reply = (f'LOOP-START samples={self.n} export=1 stream=1 run_id=7\r\n'
                     + '\r\n'.join(stream_packet(self.n)) + '\r\n' +
                     f'LOOP-PROGRESS accepted={self.n}/{self.n}\r\n'
                     f'LOOP-END qualified={self.n}/{self.n} captures={self.n} rejected=0\r\n'
                     f'LOOP-STATUS qualified={self.n} captures={self.n} rejected=0 code=12 skew_abort=0\r\n')
        else:
            reply = 'Command Error: unexpected test command\r\n'
        self.output += reply.encode()

    def read(self, count):
        result, self.output = self.output[:count], self.output[count:]
        return result

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.output = b''

    def close(self):
        self.closed = True


class ExportTests(unittest.TestCase):
    def setUp(self):
        Port.instances.clear()
        Port.fail = Port.silent = Port.incomplete = Port.partial = False

    def call_main(self, args):
        with patch.dict(sys.modules, serial=types.SimpleNamespace(Serial=Port)), \
                patch('board_loop_log.time.sleep'), contextlib.redirect_stdout(io.StringIO()):
            return main(args)

    def test_units_signed_statistics_and_coefficients(self):
        rows, result = decode_results(packet(20))
        self.assertEqual(rows[0]['offset_error_codes'], -1)
        self.assertEqual(rows[0]['skew_error_ps'], -2.09)
        self.assertEqual(rows[0]['gain_error_ppm'], -1000)
        self.assertEqual(rows[0]['gain_corr_a'], 1.01)
        self.assertEqual(rows[0]['gain_corr_b'], .99)
        self.assertEqual(rows[0]['offset_b_codes'], -5)
        self.assertIsNone(rows[0]['batch_skew_ps'])
        self.assertEqual(result['tail']['offset_error_codes']['mean'], 0)
        self.assertAlmostEqual(result['tail']['offset_error_codes']['sd'], math.sqrt(20/19))
        self.assertEqual(result['tail']['gain_error_ppm']['mean'], 0)
        self.assertEqual(result['correction']['gain_corr_a'], 1.01)
        self.assertTrue(result['history_complete'])

    def test_rejects_damaged_mixed_and_truncated_packets(self):
        cases = [packet()[:-1], packet()[:3] + packet()[4:],
                 [s.replace('run_id=7 rows=3', 'run_id=8 rows=3')
                  if s.startswith('LOOP-EXPORT-END') else s for s in packet()],
                 ['LOOP-START run_id=8'] + packet(),
                 [s.replace('version=1', 'version=2') for s in packet()],
                 [s + ',bad' if s.startswith('LOOP-EXPORT-DATA') else s for s in packet()]]
        for lines in cases:
            with self.subTest(lines=lines[-1]), self.assertRaises(ValueError):
                decode_results(lines)

    def test_duplicate_capture_rejected(self):
        lines = [s.replace('LOOP-EXPORT-DATA 2,', 'LOOP-EXPORT-DATA 1,') for s in packet()]
        with self.assertRaisesRegex(ValueError, 'capture indices'):
            decode_results(lines)

    def test_bounded_history_reports_lost_rows(self):
        lines = packet()
        lines[0] = 'LOOP-EXPORT-BEGIN version=1 run_id=7 rows=3 total_rows=5 dropped=2'
        lines[1] = lines[1].replace('requested=3 qualified=3 captures=3',
                                  'requested=5 qualified=5 captures=5')
        for i in range(3):
            lines[3+i] = lines[3+i].replace(f'DATA {i+1},', f'DATA {i+3},')
        rows, result = decode_results(lines)
        self.assertEqual(len(rows), 3)
        self.assertFalse(result['history_complete'])
        self.assertEqual(result['dropped_rows'], 2)
        self.assertEqual(result['tail']['window'], 3)

    def test_nonfinite_fields_remain_missing_and_json_safe(self):
        lines = [s.replace(',1000,1000,-209,', ',nan,1000,-209,') for s in packet()]
        rows, result = decode_results(lines)
        self.assertIsNone(rows[1]['gain_error_ppm'])
        self.assertEqual(result['tail']['gain_error_ppm']['n'], 2)
        self.assertFalse(result['run_healthy'])
        json.dumps(result, allow_nan=False)

    def test_one_pc_command_runs_and_downloads(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'run.log'
            self.assertEqual(self.call_main(['--uart', 'TEST', '--out', str(path)]), 0)
            port = Port.instances[-1]
            self.assertEqual(port.commands, ['adc -cal loop 300 stream'])
            self.assertTrue(port.closed)
            result = json.loads(path.with_suffix('.json').read_text())
            self.assertEqual(result['exported_rows'], 300)
            self.assertEqual(result['tail']['window'], 20)
            self.assertTrue(result['run_healthy'])
            with path.with_suffix('.csv').open(newline='') as stream:
                self.assertEqual(len(list(csv.DictReader(stream))), 300)
            self.assertNotIn('LOOP-EXPORT-DATA', path.read_text())
            self.assertTrue(path.with_suffix('.png').exists())
            self.assertTrue(path.with_suffix('.svg').exists())
            self.assertEqual(result['source'], 'firmware_export_v2')

    def test_download_only_never_requests_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'last.log'
            self.assertEqual(self.call_main(['--out', str(path), '--cmd', 'adc -cal loop export']), 0)
            self.assertEqual(Port.instances[-1].commands, ['adc -cal loop export'])

    def test_single_axis_capture_is_full_data_with_compact_log(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'offset.log'
            command = 'adc -cal loop test run offset 4 4 4'
            self.assertEqual(self.call_main(['--out', str(path), '--cmd', command]), 0)
            self.assertEqual(Port.instances[-1].commands, [command + ' full'])
            result = json.loads(path.with_suffix('.json').read_text())
            self.assertTrue(result['assessment']['pass'])
            self.assertEqual(result['tail']['feedback'], 'frozen')
            self.assertEqual(result['phase_statistics']['hold']['accepted'], 4)
            with path.with_suffix('.csv').open(newline='') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(len(rows), 12)
            self.assertEqual({r['phase'] for r in rows}, {'pre', 'train', 'hold'})
            self.assertNotIn('LOOP-TEST-ROW ', path.read_text())
            self.assertTrue(path.with_suffix('.png').exists())

    def test_partial_stream_is_already_saved_when_uart_times_out(self):
        Port.partial = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'partial.log'
            self.assertEqual(self.call_main(['--out', str(path), '--timeout', '.05']), 1)
            with path.with_suffix('.csv').open(newline='') as stream:
                self.assertEqual(len(list(csv.DictReader(stream))), 5)
            self.assertNotIn('LOOP-EXPORT-DATA', path.read_text())
            self.assertFalse(path.with_suffix('.json').exists())
            self.assertFalse(path.with_suffix('.png').exists())

    def test_incomplete_calibration_still_exports_without_retry(self):
        Port.incomplete = True
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'incomplete.log'
            self.assertEqual(self.call_main(['--out', str(path), '--cmd', 'adc -cal loop export']), 1)
            self.assertEqual(Port.instances[-1].commands, ['adc -cal loop export'])
            self.assertFalse(json.loads(path.with_suffix('.json').read_text())['run_completed'])
            self.assertTrue(path.with_suffix('.csv').exists())

    def test_uart_error_and_timeout_close_without_retry(self):
        for failure in ('fail', 'silent'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                setattr(Port, failure, True)
                path = Path(directory) / 'failed.log'
                self.assertEqual(self.call_main(['--out', str(path), '--timeout', '.01']), 1)
                self.assertEqual(Port.instances[-1].commands, ['adc -cal loop 300 stream'])
                self.assertTrue(Port.instances[-1].closed)
                self.assertFalse(path.with_suffix('.json').exists())
                self.assertIn('STOPPED', path.read_text())
                setattr(Port, failure, False)

    def test_legacy_rows_need_no_carrier_fields(self):
        lines = [f'LOOP it={i} st=ok used_gain_ppm=0 doff_mc={value} skew_cps=-209 '
                 'corr_a_ppm=10000 corr_b_ppm=-10000 offa_mc=3000 offb_mc=-5000 '
                 'clips=0 gsrc=dither_mag skew_route=dither_phase'
                 for i, value in enumerate((-1000, 1000))]
        lines += ['LOOP-END qualified=2/2 captures=2 rejected=0',
                  'LOOP-STATUS qualified=2 captures=2 rejected=0 code=12 skew_abort=0']
        rows, result = decode_results(lines)
        self.assertEqual(result['source'], 'legacy_uart_rows')
        self.assertEqual(result['tail']['offset_error_codes']['mean'], 0)
        self.assertIsNone(rows[0]['capture'])
        failed_capture = ['LOOP-CAPTURE failed'] + [s.replace('captures=2', 'captures=3') for s in lines]
        _, result = decode_results(failed_capture)
        self.assertTrue(result['history_complete'])
        self.assertEqual(result['capture_failures_without_measurement'], 1)

    def test_bad_command_refused_before_serial_open(self):
        for args in (['--cmd', 'adc -cal loop 3\radc -cal loop 4'], ['--out', 'run.csv'],
                     ['--out', 'run.png'], ['--out', 'run.svg']):
            self.assertEqual(self.call_main(args), 2)
        self.assertFalse(Port.instances)

    def test_stream_has_all_rows_without_ram_history_limit(self):
        rows, result = decode_results(stream_packet(4010))
        self.assertEqual(len(rows), 4010)
        self.assertTrue(result['history_complete'])
        self.assertEqual(result['dropped_rows'], 0)
        damaged = stream_packet(30)
        del damaged[10]
        with self.assertRaisesRegex(ValueError, 'row count'):
            decode_results(damaged)

    def test_plot_uses_entire_trace_and_marks_rejects(self):
        rows, result = decode_results(stream_packet(50))
        rows[5]['status'] = 'margin'
        fig = build_figure(rows, result)
        import matplotlib.pyplot as plt
        try:
            left = fig.axes[0]
            raw = next(line for line in left.lines if line.get_label() == 'Accepted measurements')
            self.assertEqual(len(raw.get_xdata()), 49)
            self.assertEqual(raw.get_xdata()[0], 1)
            self.assertEqual(raw.get_xdata()[-1], 50)
            self.assertNotIn(6, raw.get_xdata())
            rejected = next(line for line in left.lines if line.get_label() == 'Rejected captures')
            self.assertEqual(list(rejected.get_xdata()), [6])
            # Coefficients retain all records, including the rejected one.
            self.assertEqual(len(fig.axes[1].lines[0].get_xdata()), 50)
        finally:
            plt.close(fig)

    def test_reused_output_stem_preserves_previous_run(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'run.log'
            path.with_suffix('.json').write_text('previous run')
            self.assertEqual(self.call_main(['--out', str(path)]), 0)
            self.assertEqual(path.with_suffix('.json').read_text(), 'previous run')
            self.assertEqual(len(list(Path(directory).glob('run_*.png'))), 1)

    @unittest.skipUnless(os.environ.get('LOOP_C_EXPORT_FIXTURE'), 'native C fixture not specified')
    def test_actual_production_c_export(self):
        lines = Path(os.environ['LOOP_C_EXPORT_FIXTURE']).read_text().splitlines()
        rows, result = decode_results(lines)
        self.assertEqual(result['qualified'], 24)
        self.assertEqual(result['captures'], 26)
        self.assertEqual(result['capture_failures_without_measurement'], 1)
        self.assertEqual(len(rows), 25)
        self.assertEqual(rows[0]['status'], 'clipping')
        self.assertEqual(result['tail']['window'], 20)
        self.assertEqual(result['tail']['gain_error_ppm']['mean'], 0)
        self.assertEqual(result['tail']['offset_error_codes']['mean'], 0)
        self.assertAlmostEqual(result['tail']['skew_error_ps']['mean'], -2.09)
        self.assertEqual(result['correction']['offset_a_codes'], 3)


if __name__ == '__main__':
    unittest.main()
