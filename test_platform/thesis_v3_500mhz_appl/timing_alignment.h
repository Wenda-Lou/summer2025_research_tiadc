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
 * The returned lag uses the project convention:
 *
 *     aligned[i] = signal[(i + lag) mod N]
 */
int timing_find_circular_lag(
    const int16_t *reference,
    const int16_t *signal,
    size_t sample_count,
    timing_alignment_result_t *result
);

/* Parabolic refinement of the circular-correlation peak at integer_lag. */
int timing_estimate_fractional_lag(
    const int16_t *reference,
    const int16_t *signal,
    size_t sample_count,
    int32_t integer_lag,
    float *fractional_lag
);

#endif
