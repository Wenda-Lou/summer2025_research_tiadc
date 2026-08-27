#ifndef ADC_FRAME_H
#define ADC_FRAME_H

#include <stddef.h>
#include <stdint.h>

#define ADC_RAW_FRAME_BYTES      4096U
#define ADC_RAW_WORD_COUNT       2048U
#define ADC_TRAILING_WORDS       8U
#define ADC_VALID_SAMPLE_COUNT   2032U
#define ADC_WORDS_PER_DMA_BEAT   8U
#define ADC_SAMPLES_PER_CHANNEL_PER_BEAT 4U
#define ADC_CHANNEL_SAMPLE_COUNT (ADC_VALID_SAMPLE_COUNT / 2U)

/*
 * Reconstruct both independent HDL converter streams.  Each DMA beat is:
 *   w0..w3 = ADC0 S0..S3 (Channel A)
 *   w4..w7 = ADC1 S0..S3 (Channel B)
 * Words are little-endian signed, left-aligned 14-bit values and therefore
 * receive one arithmetic right shift by two.  No channel is negated, merged,
 * interleaved, or averaged.
 */
int adc_reconstruct_channels(
    const uint8_t *raw_bytes,
    size_t raw_byte_count,
    int16_t *channel_a,
    size_t channel_a_capacity,
    int16_t *channel_b,
    size_t channel_b_capacity,
    size_t *samples_per_channel
);

/* Compatibility entry point: returns the chronological Channel A stream. */
int adc_reconstruct_frame(
    const uint8_t *raw_bytes,
    size_t raw_byte_count,
    int16_t *output_samples,
    size_t output_capacity,
    size_t *output_count
);

#endif
