"""Offline serial-protocol coverage for the three-axis automation; no board I/O."""
import contextlib
import csv
import io
import json
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import patch

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from board_axis_test import BenchFault, parser, run, suite_points


class FirmwarePort:
    def __init__(self, code=12, ifc=12, valid=True, fault=None, bad_hold=None,
                 signal_status='cancelled', corrupt_signal=False, missing_snapshot=False,
                 rejected_axis=None):
        self.code = self.base_code = code
        self.ifc = self.base_ifc = ifc
        self.valid = valid
        self.fault = fault
        self.bad_hold = bad_hold
        self.signal_status = signal_status
        self.corrupt_signal = corrupt_signal
        self.missing_snapshot = missing_snapshot
        self.rejected_axis = rejected_axis
        self.features = 'spectral_line_rejection=1 gain_freeze=training_mean acquisition_phase_gate=1'
        self.acquire_phase_mrad = 150
        self.config = '200000 500 1000 1910 20'
        self.commands = []
        self.output = b''
        self.offsets = {'A': 0, 'B': 0}
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True

    def reset_input_buffer(self):
        pass

    @property
    def in_waiting(self):
        return min(137, len(self.output))

    def read(self, count):
        data, self.output = self.output[:count], self.output[count:]
        return data

    def flush(self):
        pass

    def status(self):
        values = self.config.split()
        config = ' '.join(f'{k}={v}' for k, v in zip(
            ('gain_ppm', 'skew_milli', 'deadband_cps', 'step_cps', 'batch'), values))
        lines = [f'LOOP-TEST-QUALITY acquisition_phase_mrad={self.acquire_phase_mrad} hold_phase_mrad=150',
                 f'LOOP-TEST-BASELINE valid={int(self.valid)} snapshot=1 code={self.base_code} fault=0',
                 f'LOOP-TEST-CURRENT code={self.code}', f'LOOP-TEST-CONFIG {config}',
                 'LOOP-TEST-ROUTES gain=dither_spectrum skew=dither_cross_spectrum '
                 'tone_free=1 reference_free=1 ' + self.features]
        if not self.missing_snapshot:
            lines += [f'LOOP-TEST-SNAPSHOT channel={c} ifc={self.base_ifc} dc1=2 dc2=191'
                      for c in (0, 1)]
        return lines

    def phases(self, axis, budgets):
        baseline = axis == 'baseline'
        names = ('baseline_train', 'baseline_hold') if baseline else ('pre', 'train', 'hold')
        rejected = axis == self.rejected_axis
        if rejected:
            names = ('pre',)
        lines = []
        for phase, budget in zip(names, budgets):
            train = phase.endswith('train')
            bits = {'baseline': 7, 'offset': 1, 'gain': 2, 'skew': 4}[axis] if train else 0
            if axis == 'skew' and train:
                self.code = self.base_code
            lines.append(f'LOOP-TEST-PHASE phase={phase} axes={bits} budget={budget} code={self.code}')
            for attempt in range(budget):
                corrected = phase != 'pre'
                offa = self.offsets['A'] * 1000 if axis == 'offset' and corrected else 0
                offb = self.offsets['B'] * 1000 if axis == 'offset' and corrected else 0
                gain_state = 70000 if axis == 'gain' and corrected else 0
                error = 2000 if axis == self.bad_hold and phase == 'hold' else 0
                if phase == 'pre' and axis == 'offset':
                    error = (self.offsets['B'] - self.offsets['A'])*1000
                status = 'gain_range' if rejected else 'ok'
                lines.append(f'LOOP-TEST-ROW phase={phase} attempt={attempt} code={self.code} '
                             f'LOOP it={attempt} st={status} doff_mc={error} used_gain_ppm=0 '
                             f'skew_cps=0 corr_a_ppm=0 corr_b_ppm={gain_state} '
                             f'offa_mc={offa} offb_mc={offb} clips=0 '
                             'gsrc=dither_spectrum skew_route=dither_cross_spectrum')
            lines.append(f'LOOP-TEST-SUMMARY phase={phase} attempts={budget} accepted={0 if rejected else budget} '
                         f'code={self.code} abort=0')
        return lines

    def write(self, data):
        command = data.decode().strip()
        self.commands.append(command)
        words = command.split()
        reply = [f'uart-cmd$: {command}']
        if command == self.fault:
            reply += ['LOOP-SKEW-TXN result=RECOVERY_REQUIRED', 'LOOP-TEST-END result=ERROR']
        elif command == 'adc -cal loop test status':
            reply += self.status() + ['LOOP-TEST-END result=OK']
        elif words[3:5] == ['test', 'config']:
            self.config = ' '.join(words[5:])
            reply += self.status() + ['LOOP-TEST-END result=OK']
        elif words[3:5] == ['test', 'quality']:
            self.acquire_phase_mrad = int(words[5])
            reply += self.status() + ['LOOP-TEST-END result=OK']
        elif words[3:5] == ['test', 'offset']:
            self.offsets[words[5]] = int(words[6])
            reply += ['LOOP-TEST-END result=OK']
        elif words[3:5] == ['test', 'baseline']:
            reply += self.phases('baseline', [int(v) for v in words[5:7]])
            reply += self.status() + [f'LOOP-TEST-END result={"OK" if self.valid else "ERROR"}']
        elif words[3:5] == ['test', 'restore']:
            self.code = self.base_code
            self.ifc = self.base_ifc
            self.offsets = {'A': 0, 'B': 0}
            reply += ['LOOP-TEST-END result=OK']
        elif words[3:5] == ['test', 'ifc']:
            self.ifc = int(words[6])
            reply += ['LOOP-TEST-END result=OK']
        elif words[3:5] == ['test', 'park']:
            previous = self.code
            self.code = int(words[5])
            reply += [f'LOOP-SKEW-TXN result=OK code_before={previous} code_after={self.code}',
                      'LOOP-TEST-END result=OK']
        elif words[3:5] == ['test', 'run']:
            axis = words[5]
            reply += self.phases(axis, [int(v) for v in words[6:9]])
            if axis == self.rejected_axis:
                reply += ['LOOP-TEST-END result=ERROR']
            else:
                reply += [f'LOOP-OUTPUT status={self.signal_status} samples=512 coefficients=frozen',
                          'LOOP-TEST-END result=OK']
        elif command == 'adc -cal loop output':
            status = 'ambiguous' if self.corrupt_signal else self.signal_status
            reply += [f'LOOP-SIGNAL-BEGIN version=1 samples=512 status={status} '
                      'coefficients=frozen sample_rate_hz=1300000000']
            reply += [f'LOOP-SIGNAL-DATA {i},123456,-789123' for i in range(512)]
            reply += [f'LOOP-SIGNAL-END result=OK samples=512 status={status}']
        else:
            raise AssertionError(f'Unexpected command: {command}')
        self.output += ('\n'.join(reply) + '\n').encode()


class SuiteTests(unittest.TestCase):
    def tearDown(self):
        plt.close('all')

    def arguments(self, out, *extra):
        return parser().parse_args(['suite', '--uart', 'COM5', '--pre', '4', '--train', '4',
                                    '--hold', '4', '--baseline-train', '4', '--baseline-hold', '4',
                                    '--offset-points=32', '--gain-points=13', '--skew-points=1',
                                    '--skew-mode=relative',
                                    '--gain-gate-ppm=200000',
                                    '--out', str(out), '--execute', *extra])

    def execute(self, port, args, plots=False):
        serial = types.SimpleNamespace(Serial=lambda *a, **kw: port)
        with contextlib.ExitStack() as stack:
            stack.enter_context(patch.dict('sys.modules', serial=serial))
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            browser = stack.enter_context(patch('loop_viewer_export.webbrowser.open'))
            if not plots:
                stack.enter_context(patch('axis_result_export.save_plots', return_value={}))
            result = run(args)
            return result, browser.call_count

    def test_resume_remaining_axes_skips_offset_and_completed_gain_points(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / 'remaining'
            port = FirmwarePort()
            args = self.arguments(out, '--axes=gain,skew', '--gain-points=0',
                                  '--skew-mode=absolute', '--skew-points=0,32')
            self.assertEqual(self.execute(port, args)[0], 0)
            runs = [c for c in port.commands if c.startswith('adc -cal loop test run ')]
            self.assertEqual(len(runs), 3)
            self.assertFalse(any('run offset ' in c for c in runs))
            self.assertIn('adc -cal loop test ifc B 0', port.commands)
            self.assertNotIn('adc -cal loop test ifc B 13', port.commands)
            meta = json.loads((out / 'meta.json').read_text())
            self.assertEqual(meta['points'], {'offset': [], 'gain': [0], 'skew': [0, 32]})
            self.assertEqual(meta['requested_tests'], 3)
            self.assertTrue(meta['completed'])
            self.assertTrue(meta['restored'])

    def test_invalid_axes_are_rejected(self):
        for axes in ('', 'gain,gain', 'unknown'):
            args = parser().parse_args(['suite', '--axes=' + axes])
            with self.assertRaisesRegex(ValueError, 'axes'):
                suite_points(args)

    def test_relaxed_acquisition_is_explicit_and_hold_tolerance_unchanged(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / 'relaxed'
            port = FirmwarePort()
            args = self.arguments(out, '--axes=skew', '--acquire-phase-mrad=750')
            self.assertEqual(self.execute(port, args)[0], 0)
            self.assertIn('adc -cal loop test quality 750', port.commands)
            meta = json.loads((out / 'meta.json').read_text())
            self.assertEqual(meta['board_quality'], {'acquisition_phase_mrad': '750', 'hold_phase_mrad': '150'})
            self.assertEqual(meta['tolerance']['skew'], 10.0)

    def test_relaxed_acquisition_requires_new_firmware_before_writes(self):
        with tempfile.TemporaryDirectory() as folder:
            port = FirmwarePort()
            port.features = 'spectral_line_rejection=1 gain_freeze=training_mean'
            args = self.arguments(Path(folder) / 'old', '--acquire-phase-mrad=750')
            with self.assertRaisesRegex(ValueError, 'updated ELF'):
                self.execute(port, args)
            self.assertEqual(port.commands, ['adc -cal loop test status'])

    def test_old_firmware_keeps_strict_default(self):
        with tempfile.TemporaryDirectory() as folder:
            port = FirmwarePort()
            port.features = 'spectral_line_rejection=1 gain_freeze=training_mean'
            self.assertEqual(self.execute(port, self.arguments(Path(folder) / 'old'))[0], 0)
            self.assertFalse(any('test quality' in c for c in port.commands))

    def test_fresh_baseline_all_axes_waveforms_and_viewer(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'suite'
            port = FirmwarePort()
            result, opened = self.execute(port, self.arguments(out), plots=True)
            self.assertEqual(result, 0)
            self.assertEqual(opened, 1)
            self.assertTrue(port.closed)
            self.assertEqual(port.commands.count('adc -cal loop test baseline 4 4 full'), 1)
            self.assertEqual(port.commands.count('adc -cal loop test restore'), 4)
            self.assertEqual(port.commands[-1], 'adc -cal loop test restore')
            self.assertIn('adc -cal loop test ifc B 13', port.commands)
            self.assertIn('adc -cal loop test park 13', port.commands)
            run_positions = [i for i, c in enumerate(port.commands) if c.startswith('adc -cal loop test run')]
            for position in run_positions:
                self.assertEqual(port.commands[position+1], 'adc -cal loop output')
                self.assertEqual(port.commands[position+2], 'adc -cal loop test restore')
            for axis, point in (('offset', 32), ('gain', 13), ('skew', 1)):
                stem = out/axis/f'{axis}_point{point}_repeat1'
                for suffix in ('.csv', '.json', '.png', '.svg', '.html'):
                    self.assertTrue(stem.with_suffix(suffix).exists())
                meta = json.loads(stem.with_suffix('.json').read_text())
                self.assertTrue(meta['assessment']['pass'])
                self.assertTrue(meta['assessment']['signal_downloaded'])
                signal = stem.with_name(stem.name+'_signal.json')
                self.assertTrue(json.loads(signal.read_text())['dither_cancelled'])
            self.assertTrue((out/'baseline.csv').exists())
            self.assertTrue((out/'viewer.html').exists())
            log = (out/'console.log').read_text()
            self.assertNotIn('LOOP-SIGNAL-DATA', log)
            self.assertNotIn('LOOP-TEST-ROW', log)
            meta = json.loads((out/'meta.json').read_text())
            self.assertTrue(meta['completed'])
            self.assertTrue(meta['restored'])
            self.assertEqual(meta['points'], {'offset': [32], 'gain': [13], 'skew': [1]})
            self.assertEqual((port.code, port.ifc, port.offsets), (12, 12, {'A': 0, 'B': 0}))

    def test_reuse_boundary_code_and_upper_ifc_bits(self):
        with tempfile.TemporaryDirectory() as folder:
            port = FirmwarePort(code=48, ifc=0x8C)
            args = self.arguments(Path(folder)/'suite', '--reuse-baseline', '--no-viewer', '--skew-points=-1,0,1')
            self.assertEqual(self.execute(port, args)[0], 0)
            self.assertFalse(any('test config' in c or 'test baseline' in c for c in port.commands))
            self.assertIn('adc -cal loop test park 47', port.commands)
            self.assertIn('adc -cal loop test ifc B 13', port.commands)
            self.assertNotIn('adc -cal loop test park 49', port.commands)
            skipped = json.loads((Path(args.out)/'meta.json').read_text())['skipped_points']
            self.assertEqual([(p['point'], p['target_code']) for p in skipped], [(1, 49)])

    def test_default_ranges_and_ifc_full_scale_order(self):
        args = parser().parse_args(['suite'])
        points = suite_points(args, 12, 12)
        self.assertEqual(points['offset'], list(range(32, -33, -1)))
        self.assertEqual(points['gain'], [10, 11, 12, 13, 14, 15, 0])
        self.assertEqual(points['skew'], list(range(33)))
        self.assertEqual(args.skew_mode, 'absolute')
        self.assertEqual(args.gain_gate_ppm, 600000)

    def test_bad_or_old_baseline_stops_before_writes(self):
        with tempfile.TemporaryDirectory() as folder:
            for kind in ('invalid', 'old', 'snapshot', 'config'):
                port = FirmwarePort(valid=kind != 'invalid', missing_snapshot=kind == 'snapshot')
                if kind == 'old':
                    port.features = ''
                if kind == 'config':
                    port.config = '200000 250 1000 1910 20'
                args = self.arguments(Path(folder)/kind, '--reuse-baseline', '--no-viewer')
                with self.assertRaises(ValueError):
                    self.execute(port, args)
                self.assertEqual(port.commands, ['adc -cal loop test status'])

    def test_failed_new_baseline_is_saved_and_never_injected(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'suite'
            port = FirmwarePort(valid=False)
            with self.assertRaisesRegex(BenchFault, 'Baseline admission failed'):
                self.execute(port, self.arguments(out, '--no-viewer'))
            self.assertTrue((out/'baseline.json').exists())
            self.assertFalse(any('test run' in c or 'test restore' in c or 'test ifc' in c for c in port.commands))
            self.assertFalse(json.loads((out/'meta.json').read_text())['completed'])

    def test_fault_stops_sequence_without_automatic_restore(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'suite'
            port = FirmwarePort(fault='adc -cal loop test ifc B 13')
            with self.assertRaises(BenchFault):
                self.execute(port, self.arguments(out, '--reuse-baseline', '--no-viewer'))
            self.assertEqual(port.commands[-1], 'adc -cal loop test ifc B 13')
            self.assertEqual(port.commands.count('adc -cal loop test restore'), 2)
            self.assertTrue((out/'offset/offset_point32_repeat1.csv').exists())
            self.assertTrue((out/'frames.csv').exists())
            self.assertFalse(json.loads((out/'meta.json').read_text())['restored'])

    def test_skew_ack_must_confirm_target_code(self):
        class WrongAckPort(FirmwarePort):
            def write(self, data):
                super().write(data)
                if data.decode().strip().startswith('adc -cal loop test park '):
                    self.output = self.output.replace(b'code_after=13', b'code_after=14')

        with tempfile.TemporaryDirectory() as folder:
            port = WrongAckPort()
            out = Path(folder)/'suite'
            with self.assertRaisesRegex(BenchFault, 'expected final ACK code'):
                self.execute(port, self.arguments(out, '--reuse-baseline', '--no-viewer'))
            self.assertEqual(port.commands[-1], 'adc -cal loop test park 13')
            self.assertEqual(sum('test run' in c for c in port.commands), 2)

    def test_failed_test_end_keeps_partial_export_and_stops(self):
        class FailedRunPort(FirmwarePort):
            def write(self, data):
                super().write(data)
                if data.decode().strip().startswith('adc -cal loop test run gain '):
                    self.output = self.output.replace(b'st=ok doff_mc=0', b'st=clipping doff_mc=0')
                    self.output = self.output.replace(b'accepted=4', b'accepted=0')
                    self.output = self.output.replace(b'LOOP-TEST-END result=OK', b'LOOP-TEST-END result=ERROR')

        with tempfile.TemporaryDirectory() as folder:
            port = FailedRunPort()
            out = Path(folder)/'suite'
            with self.assertRaises(BenchFault):
                self.execute(port, self.arguments(out, '--reuse-baseline', '--no-viewer'))
            self.assertTrue(port.commands[-1].startswith('adc -cal loop test run gain '))
            failed = json.loads((out/'gain/gain_point13_repeat1.json').read_text())
            self.assertFalse(failed['assessment']['pass'])
            self.assertTrue((out/'frames.csv').exists())
            self.assertFalse(json.loads((out/'meta.json').read_text())['restored'])

    def test_absolute_zero_skew_still_requires_a_matching_move_ack(self):
        class WrongAckPort(FirmwarePort):
            def write(self, data):
                super().write(data)
                if data.decode().strip() == 'adc -cal loop test park 0':
                    self.output = self.output.replace(b'code_after=0', b'code_after=1')

        with tempfile.TemporaryDirectory() as folder:
            port = WrongAckPort()
            args = self.arguments(Path(folder)/'suite', '--reuse-baseline', '--no-viewer',
                                  '--skew-mode=absolute', '--skew-points=0')
            with self.assertRaisesRegex(BenchFault, 'expected final ACK code'):
                self.execute(port, args)
            self.assertEqual(port.commands[-1], 'adc -cal loop test park 0')

    def test_hold_failure_is_not_retrained_and_other_axes_continue(self):
        with tempfile.TemporaryDirectory() as folder:
            port = FirmwarePort(bad_hold='offset', signal_status='no-tone')
            out = Path(folder)/'suite'
            self.assertEqual(self.execute(port, self.arguments(out, '--reuse-baseline', '--no-viewer'))[0], 1)
            self.assertEqual(sum('test run' in c for c in port.commands), 3)
            self.assertFalse(any('test baseline' in c for c in port.commands))
            meta = json.loads((out/'meta.json').read_text())
            self.assertTrue(meta['completed'])
            self.assertTrue(meta['restored'])
            point = json.loads((out/'offset/offset_point32_repeat1.json').read_text())['assessment']
            self.assertFalse(point['pass'])
            self.assertEqual(point['signal_status'], 'no-tone')

    def test_acquisition_range_failure_is_saved_and_next_axis_continues(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'suite'
            port = FirmwarePort(rejected_axis='gain')
            self.assertEqual(self.execute(port, self.arguments(out, '--reuse-baseline', '--no-viewer'))[0], 1)
            self.assertEqual(sum('test run' in c for c in port.commands), 3)
            self.assertEqual(port.commands[-1], 'adc -cal loop test restore')
            point = json.loads((out/'gain/gain_point13_repeat1.json').read_text())
            self.assertEqual(point['axis'], 'gain')
            self.assertFalse(point['assessment']['pass'])
            self.assertTrue(point['assessment']['measurement_rejected'])
            self.assertTrue(json.loads((out/'meta.json').read_text())['completed'])

    def test_full_default_scan_uses_absolute_skew_0_through_32(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'suite'
            port = FirmwarePort()
            args = self.arguments(out, '--no-viewer', '--no-signal')
            args.offset_points = args.gain_points = args.skew_points = None
            args.skew_mode = 'absolute'
            args.gain_gate_ppm = 600000
            self.assertEqual(self.execute(port, args)[0], 0)
            offsets = [int(c.split()[-1]) for c in port.commands if c.startswith('adc -cal loop test offset B ')][1:]
            ifcs = [int(c.split()[-1]) for c in port.commands if c.startswith('adc -cal loop test ifc B ')]
            parks = [int(c.split()[-1]) for c in port.commands if c.startswith('adc -cal loop test park ')]
            self.assertEqual(offsets, list(range(32, -33, -1)))
            self.assertEqual(ifcs, [10, 11, 12, 13, 14, 15, 0])
            self.assertEqual(parks, list(range(33)))
            self.assertEqual(sum('test run' in c for c in port.commands), 105)
            meta = json.loads((out/'meta.json').read_text())
            self.assertEqual(meta['requested_tests'], 105)
            self.assertEqual(meta['skipped_tests'], 0)
            self.assertTrue(meta['coverage_complete'])
            self.assertEqual(meta['skew_point_mode'], 'absolute')
            self.assertFalse((out/'skipped.csv').exists())
            control = json.loads((out/'skew/skew_point12_repeat1.json').read_text())['assessment']
            zero = json.loads((out/'skew/skew_point0_repeat1.json').read_text())['assessment']
            self.assertTrue(control['is_control'])
            self.assertFalse(zero['is_control'])
            self.assertEqual(zero['target_code'], 0)
            self.assertEqual(zero['relative_code_offset'], -12)
            self.assertEqual(port.commands[-1], 'adc -cal loop test restore')

    def test_unknown_error_end_is_not_treated_as_a_range_failure(self):
        class ErrorPort(FirmwarePort):
            def write(self, data):
                super().write(data)
                if data.decode().strip().startswith('adc -cal loop test run gain '):
                    self.output = self.output.replace(b'LOOP-TEST-END result=OK', b'LOOP-TEST-END result=ERROR')

        with tempfile.TemporaryDirectory() as folder:
            port = ErrorPort()
            out = Path(folder)/'suite'
            with self.assertRaisesRegex(BenchFault, 'Test ended with ERROR'):
                self.execute(port, self.arguments(out, '--reuse-baseline', '--no-viewer', '--no-signal'))
            self.assertEqual(sum('test run' in c for c in port.commands), 2)
            self.assertTrue(port.commands[-1].startswith('adc -cal loop test run gain '))
            self.assertFalse(json.loads((out/'meta.json').read_text())['restored'])

    def test_signal_failure_retains_calibration_and_stops_before_restore(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'suite'
            port = FirmwarePort(corrupt_signal=True)
            with self.assertRaisesRegex(ValueError, 'differs from advertised'):
                self.execute(port, self.arguments(out, '--reuse-baseline', '--no-viewer'))
            self.assertEqual(port.commands[-1], 'adc -cal loop output')
            point = json.loads((out/'offset/offset_point32_repeat1.json').read_text())['assessment']
            self.assertTrue(point['pass'])
            self.assertFalse(point['signal_downloaded'])
            self.assertIn('signal_error', point)
            with (out/'summary.csv').open() as stream:
                self.assertEqual(len(list(csv.DictReader(stream))), 1)

    def test_non_cancelled_signal_does_not_change_passing_hold_grade(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'suite'
            port = FirmwarePort(signal_status='ambiguous')
            self.assertEqual(self.execute(port, self.arguments(out, '--reuse-baseline', '--no-viewer'))[0], 0)
            point = json.loads((out/'offset/offset_point32_repeat1.json').read_text())['assessment']
            signal = json.loads((out/'offset/offset_point32_repeat1_signal.json').read_text())
            self.assertTrue(point['pass'])
            self.assertEqual(point['signal_status'], 'ambiguous')
            self.assertFalse(signal['dither_cancelled'])

    def test_custom_points_repeats_and_signal_skip(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'suite'
            port = FirmwarePort()
            args = self.arguments(out, '--reuse-baseline', '--no-viewer', '--no-signal',
                                  '--offset-points=-16,16', '--gain-points=0x0D,0x0E',
                                  '--skew-points=-1,1', '--repeats', '2')
            self.assertEqual(self.execute(port, args)[0], 0)
            self.assertEqual(sum('test run' in c for c in port.commands), 12)
            self.assertEqual(port.commands.count('adc -cal loop test restore'), 13)
            self.assertNotIn('adc -cal loop output', port.commands)
            self.assertFalse(list(out.rglob('*.html')))
            self.assertFalse(list(out.rglob('*_signal.csv')))

    def test_plan_and_invalid_points_do_not_open_serial(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)/'plan'
            args = self.arguments(out)
            args.execute = False
            with patch.dict('sys.modules', serial=types.SimpleNamespace(Serial=lambda *a, **kw: self.fail('serial opened'))):
                with contextlib.redirect_stdout(io.StringIO()):
                    self.assertEqual(run(args), 0)
                self.assertFalse(out.exists())
                args.execute = True
                args.gain_points = '9'
                with self.assertRaises(ValueError):
                    run(args)
                self.assertFalse(out.exists())
                args.gain_points = None
                args.offset_points = '32,32'
                with self.assertRaisesRegex(ValueError, 'unique'):
                    run(args)
                args.offset_points = '32'
                args.skew_mode = 'absolute'
                args.skew_points = '-1'
                with self.assertRaisesRegex(ValueError, 'absolute skew points'):
                    run(args)


if __name__ == '__main__':
    unittest.main()
