"""Offline signal protocol checks; never opens a serial port."""
import csv
import json
import os
from pathlib import Path
import tempfile
import unittest

from board_signal_output import SignalDecoder


def snapshot(status='cancelled'):
    return [f'LOOP-SIGNAL-BEGIN version=1 samples=512 status={status} '
            'sample_rate_hz=1300000000 coefficients=frozen',
            *(f'LOOP-SIGNAL-DATA {i},123456,-789123' for i in range(512)),
            f'LOOP-SIGNAL-END result=OK samples=512 status={status}']


class SignalTests(unittest.TestCase):
    def test_output_revision_and_rejection_diagnostics_are_preserved(self):
        lines = snapshot('ambiguous')
        lines[0] += ' output_revision=2 reason=ringing-support core_samples=4 support_samples=50'
        decoder = SignalDecoder()
        for line in lines:
            decoder.feed(line)
        with tempfile.TemporaryDirectory() as folder:
            meta = decoder.save(Path(folder)/'signal.csv')
        self.assertFalse(meta['dither_cancelled'])
        self.assertEqual(meta['output_revision'], '2')
        self.assertEqual(meta['reason'], 'ringing-support')
        self.assertEqual(meta['support_samples'], '50')

    def test_samples_units_status_and_no_overwrite(self):
        for status in ('cancelled', 'ambiguous', 'no-tone', 'no-dither'):
            decoder = SignalDecoder()
            for line in snapshot(status):
                decoder.feed(line)
            with tempfile.TemporaryDirectory() as directory:
                path = Path(directory)/'signal.csv'
                meta = decoder.save(path)
                with path.open() as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual(len(rows), 512)
                self.assertEqual(float(rows[-1]['output_a_codes']), 123.456)
                self.assertEqual(float(rows[-1]['output_b_codes']), -789.123)
                self.assertAlmostEqual(float(rows[-1]['time_s']), 511/1.3e9)
                self.assertEqual(meta['dither_cancelled'], status == 'cancelled')
                self.assertEqual(json.loads(path.with_suffix('.json').read_text())['status'], status)
                with self.assertRaises(FileExistsError):
                    decoder.save(path)

    def test_missing_duplicate_or_corrupt_packets_are_rejected(self):
        for mutate in ('missing', 'duplicate', 'mismatch', 'refused', 'no_header', 'order'):
            lines = snapshot()
            if mutate == 'missing': del lines[20]
            if mutate == 'duplicate': lines.insert(20, lines[19])
            if mutate == 'order': lines[20], lines[21] = lines[21], lines[20]
            if mutate == 'mismatch': lines[-1] = lines[-1].replace('cancelled', 'ambiguous')
            if mutate == 'refused': lines = ['LOOP-SIGNAL-END result=ERROR reason=no-output']
            if mutate == 'no_header': del lines[0]
            decoder = SignalDecoder()
            with self.subTest(mutate=mutate), self.assertRaises(ValueError):
                for line in lines: decoder.feed(line)

    def test_incomplete_snapshot_cannot_be_saved(self):
        decoder = SignalDecoder()
        for line in snapshot()[:-1]: decoder.feed(line)
        with self.assertRaises(ValueError): decoder.save('unused.csv')

    def test_uart_prompt_prefix_does_not_drop_the_signal_header(self):
        decoder = SignalDecoder()
        for line in snapshot():
            decoder.feed('uart-cmd$: ' + line)
        self.assertTrue(decoder.done)
        self.assertEqual(len(decoder.rows), 512)

    @unittest.skipUnless(os.environ.get('LOOP_C_SIGNAL_FIXTURE'), 'native C fixture not specified')
    def test_actual_production_c_signal(self):
        decoder = SignalDecoder()
        for line in Path(os.environ['LOOP_C_SIGNAL_FIXTURE']).read_text().splitlines():
            if line.startswith('LOOP-SIGNAL-'):
                decoder.feed(line)
        self.assertTrue(decoder.done)
        self.assertEqual(decoder.meta['status'], 'cancelled')
        self.assertEqual(decoder.rows[-1], (511, 123.456, -789.123))


if __name__ == '__main__':
    unittest.main()
