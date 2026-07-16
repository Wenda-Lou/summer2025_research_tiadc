#ifndef REFERENCE_BUFFER_H
#define REFERENCE_BUFFER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Maximum number of reference samples stored on the FPGA.
 *
 * One ADC capture contains ADC_CAPTURE_SAMPLES valid int16_t values.
 * Keep this buffer the same size so one uploaded reference frame maps
 * directly to one ADC capture frame.
 */
#ifndef REFERENCE_MAX_SAMPLES
#define REFERENCE_MAX_SAMPLES 2032U
#endif

typedef enum {
    REFERENCE_BUFFER_OK = 0,
    REFERENCE_BUFFER_ERR_NULL = -1,
    REFERENCE_BUFFER_ERR_EMPTY = -2,
    REFERENCE_BUFFER_ERR_TOO_LARGE = -3,
    REFERENCE_BUFFER_ERR_INDEX = -4
} reference_buffer_status_t;

/* Remove the currently stored reference waveform. */
void reference_buffer_clear(void);

/*
 * Copy a complete reference waveform into the internal buffer.
 *
 * This is useful when the transport layer has already assembled the full
 * waveform in another temporary buffer.
 */
reference_buffer_status_t reference_buffer_load(
    const int16_t *samples,
    size_t sample_count
);

/*
 * Begin an incremental upload.
 *
 * Call this before receiving the first UDP reference packet.
 */
reference_buffer_status_t reference_buffer_begin(size_t expected_sample_count);

/*
 * Copy one chunk into the internal buffer at sample_offset.
 *
 * sample_offset and sample_count are measured in int16_t samples, not bytes.
 */
reference_buffer_status_t reference_buffer_write_chunk(
    size_t sample_offset,
    const int16_t *samples,
    size_t sample_count
);

/*
 * Mark the current incremental upload as complete.
 *
 * This succeeds only when all expected samples have been written.
 */
reference_buffer_status_t reference_buffer_finalize(void);

/* Accessors used by the calibration module. */
const int16_t *reference_buffer_data(void);
size_t reference_buffer_length(void);
size_t reference_buffer_expected_length(void);
int reference_buffer_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* REFERENCE_BUFFER_H */
