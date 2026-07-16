#include "reference_buffer.h"

#include <string.h>

static int16_t reference_samples[REFERENCE_MAX_SAMPLES];

static size_t reference_length = 0U;
static size_t expected_length = 0U;
static size_t written_sample_count = 0U;

static uint8_t reference_ready = 0U;

/*
 * A byte-per-sample written map is simple and reliable for the current
 * 2032-sample reconstructed capture size. It prevents finalize() from
 * accepting a reference with missing or duplicated packet regions.
 */
static uint8_t written_map[REFERENCE_MAX_SAMPLES];

void reference_buffer_clear(void)
{
    memset(reference_samples, 0, sizeof(reference_samples));
    memset(written_map, 0, sizeof(written_map));

    reference_length = 0U;
    expected_length = 0U;
    written_sample_count = 0U;
    reference_ready = 0U;
}

reference_buffer_status_t reference_buffer_load(
    const int16_t *samples,
    size_t sample_count
)
{
    if (samples == NULL) {
        return REFERENCE_BUFFER_ERR_NULL;
    }

    if (sample_count == 0U) {
        return REFERENCE_BUFFER_ERR_EMPTY;
    }

    if (sample_count > REFERENCE_MAX_SAMPLES) {
        return REFERENCE_BUFFER_ERR_TOO_LARGE;
    }

    reference_buffer_clear();

    memcpy(
        reference_samples,
        samples,
        sample_count * sizeof(reference_samples[0])
    );

    memset(written_map, 1, sample_count);

    reference_length = sample_count;
    expected_length = sample_count;
    written_sample_count = sample_count;
    reference_ready = 1U;

    return REFERENCE_BUFFER_OK;
}

reference_buffer_status_t reference_buffer_begin(
    size_t expected_sample_count
)
{
    if (expected_sample_count == 0U) {
        return REFERENCE_BUFFER_ERR_EMPTY;
    }

    if (expected_sample_count > REFERENCE_MAX_SAMPLES) {
        return REFERENCE_BUFFER_ERR_TOO_LARGE;
    }

    reference_buffer_clear();
    expected_length = expected_sample_count;

    return REFERENCE_BUFFER_OK;
}

reference_buffer_status_t reference_buffer_write_chunk(
    size_t sample_offset,
    const int16_t *samples,
    size_t sample_count
)
{
    size_t i;

    if (samples == NULL) {
        return REFERENCE_BUFFER_ERR_NULL;
    }

    if (sample_count == 0U) {
        return REFERENCE_BUFFER_ERR_EMPTY;
    }

    if (expected_length == 0U) {
        return REFERENCE_BUFFER_ERR_EMPTY;
    }

    if ((sample_offset >= expected_length) ||
        (sample_count > (expected_length - sample_offset))) {
        return REFERENCE_BUFFER_ERR_INDEX;
    }

    memcpy(
        &reference_samples[sample_offset],
        samples,
        sample_count * sizeof(reference_samples[0])
    );

    for (i = 0U; i < sample_count; ++i) {
        const size_t index = sample_offset + i;

        if (written_map[index] == 0U) {
            written_map[index] = 1U;
            ++written_sample_count;
        }
    }

    reference_ready = 0U;
    reference_length = 0U;

    return REFERENCE_BUFFER_OK;
}

reference_buffer_status_t reference_buffer_finalize(void)
{
    if (expected_length == 0U) {
        return REFERENCE_BUFFER_ERR_EMPTY;
    }

    if (written_sample_count != expected_length) {
        return REFERENCE_BUFFER_ERR_INDEX;
    }

    reference_length = expected_length;
    reference_ready = 1U;

    return REFERENCE_BUFFER_OK;
}

const int16_t *reference_buffer_data(void)
{
    return reference_samples;
}

size_t reference_buffer_length(void)
{
    return reference_length;
}

size_t reference_buffer_expected_length(void)
{
    return expected_length;
}

int reference_buffer_is_ready(void)
{
    return reference_ready != 0U;
}
