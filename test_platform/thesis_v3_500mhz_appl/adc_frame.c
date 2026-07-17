#include "adc_frame.h"

#include <limits.h>
#include <string.h>

static int16_t saturate_int16(int32_t value)
{
    if (value > INT16_MAX) {
        return INT16_MAX;
    }

    if (value < INT16_MIN) {
        return INT16_MIN;
    }

    return (int16_t)value;
}

int adc_reconstruct_frame(
    const uint8_t *raw_bytes,
    size_t raw_byte_count,
    int16_t *output_samples,
    size_t output_capacity,
    size_t *output_count
)
{
    int16_t words[ADC_RAW_WORD_COUNT];
    size_t usable_word_count;
    size_t group_count;
    size_t output_index = 0U;
    size_t group;

    if ((raw_bytes == NULL) ||
        (output_samples == NULL) ||
        (output_count == NULL)) {
        return -1;
    }

    *output_count = 0U;

    if (raw_byte_count != ADC_RAW_FRAME_BYTES) {
        return -2;
    }

    if (output_capacity < ADC_VALID_SAMPLE_COUNT) {
        return -3;
    }

    /*
     * Read little-endian signed 16-bit words safely, without assuming
     * raw_bytes is aligned to an int16_t boundary.
     */
    for (size_t i = 0U; i < ADC_RAW_WORD_COUNT; ++i) {
        uint16_t raw_word =
            ((uint16_t)raw_bytes[(2U * i) + 0U]) |
            ((uint16_t)raw_bytes[(2U * i) + 1U] << 8U);

        /*
         * The FPGA data contains the 14-bit ADC value shifted left by
         * two bits. Casting first preserves the signed representation.
         *
         * GCC for the Zynq target performs an arithmetic right shift.
         */
        words[i] = ((int16_t)raw_word) >> 2;
    }

    usable_word_count = ADC_RAW_WORD_COUNT - ADC_TRAILING_WORDS;

    /*
     * Match Python:
     *     usable = samples.size - samples.size % 8
     *
     * 2048 - 8 = 2040
     * 2040 rounded down to a multiple of eight = 2032.
     */
    usable_word_count -= usable_word_count % 8U;
    group_count = usable_word_count / 8U;

    for (group = 0U; group < group_count; ++group) {
        size_t base = group * 8U;

        for (size_t branch_index = 0U; branch_index < 4U; ++branch_index) {
            int32_t positive = words[base + branch_index];
            int32_t negative = -(int32_t)words[base + 4U + branch_index];

            output_samples[output_index++] =
                saturate_int16(positive);

            output_samples[output_index++] =
                saturate_int16(negative);
        }
    }

    if (output_index != ADC_VALID_SAMPLE_COUNT) {
        return -4;
    }

    *output_count = output_index;
    return 0;
}