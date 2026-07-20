#include "adc_frame.h"

static int16_t decode_left_aligned_14bit(
    const uint8_t *raw_bytes,
    size_t word_index
)
{
    const uint16_t raw_word =
        (uint16_t)raw_bytes[2U * word_index] |
        ((uint16_t)raw_bytes[(2U * word_index) + 1U] << 8U);
    return ((int16_t)raw_word) >> 2;
}

int adc_reconstruct_channels(
    const uint8_t *raw_bytes,
    size_t raw_byte_count,
    int16_t *channel_a,
    size_t channel_a_capacity,
    int16_t *channel_b,
    size_t channel_b_capacity,
    size_t *samples_per_channel
)
{
    const size_t beat_count =
        ADC_VALID_SAMPLE_COUNT / ADC_WORDS_PER_DMA_BEAT;
    size_t output_index = 0U;

    if ((raw_bytes == NULL) || (channel_a == NULL) ||
        (channel_b == NULL) || (samples_per_channel == NULL)) return -1;
    *samples_per_channel = 0U;
    if (raw_byte_count < ADC_VALID_SAMPLE_COUNT * sizeof(uint16_t)) return -2;
    if ((channel_a_capacity < ADC_CHANNEL_SAMPLE_COUNT) ||
        (channel_b_capacity < ADC_CHANNEL_SAMPLE_COUNT)) return -3;

    for (size_t beat = 0U; beat < beat_count; ++beat) {
        const size_t base = beat * ADC_WORDS_PER_DMA_BEAT;
        for (size_t sample = 0U;
             sample < ADC_SAMPLES_PER_CHANNEL_PER_BEAT; ++sample) {
            channel_a[output_index] = decode_left_aligned_14bit(
                raw_bytes, base + sample);
            channel_b[output_index] = decode_left_aligned_14bit(
                raw_bytes, base + ADC_SAMPLES_PER_CHANNEL_PER_BEAT + sample);
            ++output_index;
        }
    }

    if (output_index != ADC_CHANNEL_SAMPLE_COUNT) return -4;
    *samples_per_channel = output_index;
    return 0;
}

int adc_reconstruct_frame(
    const uint8_t *raw_bytes,
    size_t raw_byte_count,
    int16_t *output_samples,
    size_t output_capacity,
    size_t *output_count
)
{
    static int16_t unused_channel_b[ADC_CHANNEL_SAMPLE_COUNT];

    return adc_reconstruct_channels(
        raw_bytes,
        raw_byte_count,
        output_samples,
        output_capacity,
        unused_channel_b,
        ADC_CHANNEL_SAMPLE_COUNT,
        output_count
    );
}
