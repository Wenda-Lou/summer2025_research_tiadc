import json
import struct
import tempfile
import unittest
from pathlib import Path

from .reference_upload import (
    ADC_SAMPLE_RATE_HZ,
    DAC_SAMPLE_RATE_HZ,
    REFERENCE_FORMAT_DAC_RATE_2X,
    build_reference_packets,
    load_and_validate_waveform_metadata,
    validate_waveform_metadata,
)


class WaveformMetadataRateTests(unittest.TestCase):
    def test_hardware_rate_metadata_passes(self):
        rates = validate_waveform_metadata({
            "adc_sample_rate_hz": 1_300_000_000.0,
            "dac_sample_rate_hz": 2_600_000_000.0,
            "dac_to_adc_rate_ratio": 2.0,
            "dither_period_dac": 260,
            "dither_period_adc": 130,
            "dither_event_period_seconds": 100e-9,
        })
        self.assertEqual(rates["adc_sample_rate_hz"], ADC_SAMPLE_RATE_HZ)
        self.assertEqual(rates["dac_sample_rate_hz"], DAC_SAMPLE_RATE_HZ)
        self.assertEqual(rates["dac_to_adc_rate_ratio"], 2.0)

    def test_old_1450_metadata_fails(self):
        with self.assertRaisesRegex(ValueError, "ADC rate does not match"):
            validate_waveform_metadata({
                "adc_sample_rate_hz": 1_450_000_000.0,
                "dac_sample_rate_hz": 2_600_000_000.0,
                "dac_to_adc_rate_ratio": 2_600.0 / 1_450.0,
            })

    def test_inconsistent_dither_spacing_fails(self):
        with self.assertRaisesRegex(ValueError, "dither spacing"):
            validate_waveform_metadata({
                "adc_sample_rate_hz": 1_300_000_000.0,
                "dac_sample_rate_hz": 2_600_000_000.0,
                "dac_to_adc_rate_ratio": 2.0,
                "dither_period_dac": 260,
                "dither_period_adc": 145,
            })

    def test_reference_begin_packet_carries_rates(self):
        packets = build_reference_packets(
            range(8),
            require_full_buffer=False,
            reference_format=REFERENCE_FORMAT_DAC_RATE_2X,
        )
        count, format_id, adc_rate, dac_rate = struct.unpack(
            "<HBII", packets[0][4:]
        )
        self.assertEqual(count, 8)
        self.assertEqual(format_id, REFERENCE_FORMAT_DAC_RATE_2X)
        self.assertEqual(adc_rate, 1_300_000_000)
        self.assertEqual(dac_rate, 2_600_000_000)

    def test_calibration_loop_sidecar_name_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            txt_path = Path(directory) / "impulse_dither.txt"
            metadata_path = txt_path.with_suffix(".json")
            metadata_path.write_text(json.dumps({
                "derived": {
                    "fs_adc_hz": 1_300_000_000.0,
                    "fs_dac_hz": 2_600_000_000.0,
                    "adc_ratio": 2,
                    "slot_period_adc_samples": 64,
                },
                "config": {"dither_period_dac": 128},
            }), encoding="utf-8")
            rates = load_and_validate_waveform_metadata(txt_path)
            self.assertEqual(rates["dac_to_adc_rate_ratio"], 2.0)


if __name__ == "__main__":
    unittest.main()
