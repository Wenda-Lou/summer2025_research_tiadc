#ifndef SIM_CONFIG_H
#define SIM_CONFIG_H

#include <stddef.h>
#include <stdint.h>

#include "adc_frame.h"
#include "adc_test_config.h"

#define SIM_DEFAULT_ADC_SAMPLE_RATE_HZ ADC_CONFIGURED_SAMPLE_RATE_HZ
#define SIM_DEFAULT_DAC_SAMPLE_RATE_HZ DAC_SAMPLE_RATE_HZ
#define SIM_DEFAULT_TONE_HZ            50000000.0
#define SIM_REFERENCE_CAPACITY         4064U
#define SIM_ADC_CHANNEL_SAMPLES        ADC_CHANNEL_SAMPLE_COUNT
#define SIM_DMA_BYTES                  ADC_RAW_FRAME_BYTES
#define SIM_ADC_MIN_CODE               (-8192)
#define SIM_ADC_MAX_CODE               8191
#define SIM_OUTPUT_DIR_DEFAULT         "output"

#endif /* SIM_CONFIG_H */
