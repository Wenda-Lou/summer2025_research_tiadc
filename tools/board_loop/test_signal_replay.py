"""Offline checks of the production-C snapshot replay tool; no bench access."""
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[2]
DEFAULT_EXE = REPO / 'calibration_sim/build-dither' / ('adc_signal_replay.exe' if os.name == 'nt' else 'adc_signal_replay')
EXE = Path(os.environ.get('ADC_SIGNAL_REPLAY', DEFAULT_EXE)).resolve()
FIXTURE = REPO / 'calibration_sim/fixtures/output_wide_impulse.csv'


@unittest.skipUnless(EXE.is_file(), 'build adc_signal_replay or set ADC_SIGNAL_REPLAY')
class SignalReplayTests(unittest.TestCase):
    def replay(self, source, out):
        return subprocess.run([str(EXE), str(source), str(out), '1300000000'],
                              capture_output=True, text=True, timeout=30)

    def test_real_frame_cancelled_and_existing_results_never_overwritten(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / 'clean.csv'
            result = self.replay(FIXTURE, out)
            self.assertEqual(result.returncode, 0, result.stderr)
            meta = json.loads(out.with_suffix('.json').read_text())
            self.assertTrue(meta['dither_cancelled'])
            self.assertEqual(meta['source'], 'offline_production_c_replay')
            self.assertGreater(meta['validation_db_a'], 20)
            self.assertGreater(meta['validation_db_b'], 20)
            self.assertEqual(meta['samples'], 1020)
            saved = out.read_bytes(), out.with_suffix('.json').read_bytes()
            self.assertEqual(self.replay(FIXTURE, out).returncode, 2)
            self.assertEqual(saved, (out.read_bytes(), out.with_suffix('.json').read_bytes()))
            out.unlink()  # test sidecar collision separately; do not replace it
            self.assertEqual(self.replay(FIXTURE, out).returncode, 2)
            self.assertFalse(out.exists())
            self.assertEqual(saved[1], out.with_suffix('.json').read_bytes())

    def test_bad_time_indices_and_nonfinite_data_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / 'bad.csv'
            out = Path(folder) / 'clean.csv'
            lines = FIXTURE.read_text().splitlines()
            for replacement in ('0,0,nan,0', '1,0,0,0', '0,0.1,0,0'):
                source.write_text('\n'.join([lines[0], replacement, *lines[2:]]) + '\n')
                self.assertEqual(self.replay(source, out).returncode, 2)
                self.assertFalse(out.exists())
                self.assertFalse(out.with_suffix('.json').exists())

    def test_new_board_snapshot_with_33_sample_ringing_is_cancelled(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder) / 'clean.csv'
            source = FIXTURE.with_name('output_ringing_boundary.csv')
            result = self.replay(source, out)
            self.assertEqual(result.returncode, 0, result.stderr)
            meta = json.loads(out.with_suffix('.json').read_text())
            self.assertEqual(meta['output_revision'], 2)
            self.assertEqual(meta['reason'], 'validated')
            self.assertEqual(meta['core_samples'], 4)
            self.assertEqual(meta['support_samples'], 33)
            self.assertGreater(meta['validation_db_a'], 25)
            self.assertGreater(meta['validation_db_b'], 25)

    def test_clean_tone_passes_through_without_cancellation_claim(self):
        with tempfile.TemporaryDirectory() as folder:
            source, out = Path(folder) / 'tone.csv', Path(folder) / 'clean.csv'
            with source.open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(('sample', 'time_s', 'output_a_codes', 'output_b_codes'))
                for i in range(1020):
                    y = 300 * math.sin(2*math.pi*0.15337*i)
                    writer.writerow((i, i/1.3e9, y, y))
            self.assertEqual(self.replay(source, out).returncode, 1)
            self.assertFalse(json.loads(out.with_suffix('.json').read_text())['dither_cancelled'])
            with source.open() as a, out.open() as b:
                before, after = list(csv.DictReader(a)), list(csv.DictReader(b))
            self.assertEqual([float(r['output_a_codes']) for r in before],
                             [float(r['output_a_codes']) for r in after])


if __name__ == '__main__':
    unittest.main()
