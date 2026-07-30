#include "sim_dma.h"

#include <string.h>

static void encode_left_aligned_14bit(uint8_t *raw_bytes, size_t word_index, int16_t value)
{
    const int16_t shifted = (int16_t)(value << 2);
    const uint16_t raw = (uint16_t)shifted;

    raw_bytes[2U * word_index] = (uint8_t)(raw & 0xffU);
    raw_bytes[(2U * word_index) + 1U] = (uint8_t)(raw >> 8U);
}

int sim_dma_encode_channels(
    const int16_t *channel_a,
    const int16_t *channel_b,
    size_t sample_count,
    uint8_t *raw_bytes,
    size_t raw_byte_count)
{
    size_t input_index = 0U;
    const size_t beat_count = ADC_VALID_SAMPLE_COUNT / ADC_WORDS_PER_DMA_BEAT;

    if (channel_a == NULL || channel_b == NULL || raw_bytes == NULL) return -1;
    if (sample_count < ADC_CHANNEL_SAMPLE_COUNT) return -2;
    if (raw_byte_count < ADC_RAW_FRAME_BYTES) return -3;

    memset(raw_bytes, 0, raw_byte_count);
    for (size_t beat = 0U; beat < beat_count; ++beat) {
        const size_t base = beat * ADC_WORDS_PER_DMA_BEAT;
        for (size_t sample = 0U; sample < ADC_SAMPLES_PER_CHANNEL_PER_BEAT; ++sample) {
            encode_left_aligned_14bit(raw_bytes, base + sample, channel_a[input_index + sample]);
            encode_left_aligned_14bit(
                raw_bytes,
                base + ADC_SAMPLES_PER_CHANNEL_PER_BEAT + sample,
                channel_b[input_index + sample]);
        }
        input_index += ADC_SAMPLES_PER_CHANNEL_PER_BEAT;
    }

    return input_index == ADC_CHANNEL_SAMPLE_COUNT ? 0 : -4;
}
