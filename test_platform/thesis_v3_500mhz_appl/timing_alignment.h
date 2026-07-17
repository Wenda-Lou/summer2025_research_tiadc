#ifndef TIMING_ALIGNMENT_H
#define TIMING_ALIGNMENT_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    int32_t lag_samples;
    float correlation;
} timing_alignment_result_t;

/*
 * Find the integer circular lag that maximizes centered cross-correlation.
 *
 * The returned lag has the same intended meaning as the Python code:
 *
 *     lag, correlation = estimate_circular_lag(reference, signal)
 *     aligned = np.roll(signal, -lag)
 */
int timing_find_circular_lag(
    const int16_t *reference,
    const int16_t *signal,
    size_t sample_count,
    timing_alignment_result_t *result
);

/*
 * Apply the detected lag:
 *
 *     aligned[i] = signal[(i + lag) mod N]
 */
int timing_apply_circular_lag(
    const int16_t *signal,
    size_t sample_count,
    int32_t lag_samples,
    int16_t *aligned_output
);

#endif