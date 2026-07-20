#ifndef ADC_TEST_CONFIG_H
#define ADC_TEST_CONFIG_H

#define ADC_CONFIGURED_SAMPLE_RATE_HZ 1450000000.0
/* Compatibility name for hardware configuration calculations only. */
#define ADC_SAMPLE_RATE_HZ             ADC_CONFIGURED_SAMPLE_RATE_HZ
#define DAC_SAMPLE_RATE_HZ          2600000000.0
#define ADC_TEST_CAPTURE_SAMPLES    2032U
#define CAL_COHERENCE_TOLERANCE     1.0e-4
#define CAL_FRACTIONAL_EPSILON      1.0e-12

#if ADC_TEST_CAPTURE_SAMPLES != 2032U
#error "ADC DMA reconstruction requires 2032 combined samples"
#endif

#endif /* ADC_TEST_CONFIG_H */
