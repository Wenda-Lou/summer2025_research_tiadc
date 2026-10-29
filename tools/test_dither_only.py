"""Strict dither calibration audit. No UART or bench access."""
from contextlib import ExitStack
from dataclasses import replace
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from calibration_loop.dither import DitherConfig, build_dac_waveform, write_dac_files
from calibration_loop.loop import CalibrationLoop, LoopOptions
from calibration_loop.simulate import BenchModel


def config():
    return DitherConfig(sig_cycles=0, amp_dbfs=-120, dither_edge_dac=4,
                        dither_top_dac=4, dither_scale_lsb=5190)


def options(**kwargs):
    return LoopOptions(gain_observable="dither_mag", skew_observable="dither_phase", **kwargs)


class Replay:
    def __init__(self, raw):
        self.raw = raw

    def capture(self):
        return self.raw


class StrictDitherTests(unittest.TestCase):
    def test_full_scale_waveform_has_legal_peaks_without_overflow(self):
        cfg = replace(config(), dither_scale_lsb=32767)
        with tempfile.TemporaryDirectory() as out:
            result = write_dac_files(cfg, out, stem="full_scale")
            derived = result["meta"]["derived"]
            wave = np.loadtxt(result["txt"], dtype=np.int64)
        self.assertEqual((wave.min(), wave.max(), wave.sum()), (-32767, 32767, 0))
        self.assertEqual(wave.size, 16640)
        self.assertFalse(derived["clipping"])
        self.assertTrue(derived["touches_dac_rails"])
        self.assertEqual(derived["dac_rail_samples"], 320)
        self.assertEqual(derived["main_tone_amplitude_lsb"], 0)
        with self.assertRaises(ValueError):
            build_dac_waveform(replace(cfg, dither_scale_lsb=32768))

    def forbid_tone(self):
        stack = ExitStack()
        for target in ("calibration_loop.estimator.fit_tone",
                       "calibration_loop.dither.DitherConfig.tone_phase_coherence",
                       "calibration_loop.estimator._tone_purity",
                       "calibration_loop.loop.analyse",
                       "calibration_loop.loop.channel_difference_dbc",
                       "calibration_loop.loop.mismatch_spurs"):
            stack.enter_context(patch(target, side_effect=AssertionError("Tone dependency: " + target)))
        return stack

    def test_closed_loop_with_every_tone_function_forbidden(self):
        cfg = config()
        cfg.validate()
        bench = BenchModel(cfg=cfg, noise_rms_codes=1.0)
        with self.forbid_tone():
            loop = CalibrationLoop(bench, cfg, options=options())
            loop.run(60, verbose=False)
        self.assertEqual((loop.qualified, loop.rejected), (60, 0))
        self.assertFalse(loop.opt.cancel_signal)
        tail = loop.log[-12:]
        self.assertLess(abs(np.mean([r["gain_mag_ratio"] for r in tail]) - 1), .001)
        self.assertLess(abs(np.mean([r["offset_b_codes"] - r["offset_a_codes"] for r in tail])), .1)
        self.assertLess(abs(np.mean([r["skew_used_ps"] for r in tail])), 1.0)
        self.assertTrue(all(np.isnan(r["tone_ratio"]) and np.isnan(r["skew_phase_ps"]) for r in tail))

    def test_unused_tone_metadata_cannot_change_results(self):
        cfg = config()
        raw = BenchModel(cfg=cfg, noise_rms_codes=1.0, seed=17).capture()
        rows = []
        with self.forbid_tone():
            for cycles in (0, 1280, 2556):
                candidate = replace(cfg, sig_cycles=cycles)
                candidate.validate()
                rows.append(CalibrationLoop(Replay(raw), candidate,
                                            options=options(close_skew_loop=False)).step())
        for row in rows:
            self.assertFalse(row["rejected"])
            for key in ("rotation", "align_n0", "align_margin", "gain_mag_ratio",
                        "offset_a_codes", "offset_b_codes", "skew_used_ps",
                        "gain_corr_a", "gain_corr_b", "offset_a_state", "offset_b_state"):
                self.assertEqual(row[key], rows[0][key], key)

    def test_missing_dither_never_falls_back_to_tone(self):
        cfg = config()
        carrier_only = BenchModel(cfg=DitherConfig(dither_scale_lsb=0),
                                  noise_rms_codes=0.0).capture()
        with self.forbid_tone():
            for raw in (bytes(4096), carrier_only):
                loop = CalibrationLoop(Replay(raw), cfg, options=options())
                loop.run(1, verbose=False)
                self.assertEqual(loop.qualified, 0)
                self.assertEqual(loop.state.gain_corr_a, 1)
                self.assertEqual(loop.state.offset_b, 0)

    def test_mixed_tone_and_dither_control_is_refused(self):
        for opts in (LoopOptions(gain_observable="tone", skew_observable="dither_phase"),
                     LoopOptions(gain_observable="dither_mag", skew_observable="phase")):
            with self.assertRaises(ValueError):
                CalibrationLoop(Replay(bytes(4096)), config(), options=opts)
        wave, _ = build_dac_waveform(config())
        self.assertEqual(config().a_sine, 0)
        self.assertEqual((wave.min(), wave.max(), wave.sum()), (-5190, 5190, 0))


if __name__ == "__main__":
    unittest.main()
