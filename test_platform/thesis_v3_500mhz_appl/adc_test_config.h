#ifndef ADC_TEST_CONFIG_H
#define ADC_TEST_CONFIG_H

#define ADC_SAMPLE_RATE_HZ          1300000000.0
#define DAC_SAMPLE_RATE_HZ          2600000000.0
#define ADC_TEST_CAPTURE_SAMPLES    2032U
#define CAL_EXPECTED_TONE_BIN       547U
#define CAL_EXPECTED_TONE_HZ        \
    ((double)CAL_EXPECTED_TONE_BIN * ADC_SAMPLE_RATE_HZ / \
     (double)ADC_TEST_CAPTURE_SAMPLES)
#define CAL_COHERENCE_TOLERANCE     1.0e-4
#define CAL_FREQUENCY_TOLERANCE_HZ  1000000.0
#define CAL_FRACTIONAL_EPSILON      1.0e-12

#if ADC_TEST_CAPTURE_SAMPLES != 2032U
#error "Coherent ADC test configuration requires 2032 reconstructed samples"
#endif

#endif /* ADC_TEST_CONFIG_H */
