#ifndef ADC_FRAME_H
#define ADC_FRAME_H

#include <stddef.h>
#include <stdint.h>

#define ADC_RAW_FRAME_BYTES      4096U
#define ADC_RAW_WORD_COUNT       2048U
#define ADC_TRAILING_WORDS       8U
#define ADC_VALID_SAMPLE_COUNT   2032U

/*
 * Reconstruct one DMA frame using the same format as frame.py:
 *
 * 1. Interpret bytes as little-endian int16.
 * 2. Arithmetic shift right by two bits.
 * 3. Remove the final eight words.
 * 4. Divide each eight-word group into:
 *      positive branch: words 0..3
 *      negative branch: words 4..7
 * 5. Negate the negative branch and interleave both branches.
 */
int adc_reconstruct_frame(
    const uint8_t *raw_bytes,
    size_t raw_byte_count,
    int16_t *output_samples,
    size_t output_capacity,
    size_t *output_count
);

#endif