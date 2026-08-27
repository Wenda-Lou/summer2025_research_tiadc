import unittest

from .generate_dac_waveform import (
    ADC_FRAME_SAMPLES,
    ADC_SAMPLE_RATE_HZ,
    DAC_SAMPLE_RATE_HZ,
    DAC_TO_ADC_RATE_RATIO,
    DITHER_EVENT_PERIOD_SECONDS,
    DITHER_PERIOD_ADC,
    DITHER_PERIOD_DAC,
)


class WaveformRateModelTests(unittest.TestCase):
    def test_authoritative_rates_and_ratio(self):
        self.assertEqual(ADC_SAMPLE_RATE_HZ, 1_300_000_000.0)
        self.assertEqual(DAC_SAMPLE_RATE_HZ, 2_600_000_000.0)
        self.assertEqual(DAC_TO_ADC_RATE_RATIO, 2.0)

    def test_dither_spacing_comes_from_physical_period(self):
        self.assertEqual(
            DITHER_PERIOD_DAC,
            round(DITHER_EVENT_PERIOD_SECONDS * DAC_SAMPLE_RATE_HZ),
        )
        self.assertEqual(
            DITHER_PERIOD_ADC,
            round(DITHER_EVENT_PERIOD_SECONDS * ADC_SAMPLE_RATE_HZ),
        )
        self.assertEqual(DITHER_PERIOD_DAC, 260)
        self.assertEqual(DITHER_PERIOD_ADC, 130)

    def test_diagnostic_fft_bin_frequency(self):
        self.assertEqual(ADC_FRAME_SAMPLES, 1016)
        frequency_hz = 156 * ADC_SAMPLE_RATE_HZ / 1016
        self.assertAlmostEqual(frequency_hz / 1e6, 199.6062992125984)


if __name__ == "__main__":
    unittest.main()
