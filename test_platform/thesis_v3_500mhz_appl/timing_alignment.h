#ifndef TIMING_ALIGNMENT_H
#define TIMING_ALIGNMENT_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    int32_t lag_samples;
    float correlation;
} timing_alignment_result_t;

/*
 * Board-side timing analysis used by ADC reference and calibration flows.
 *
 * After circular alignment, the fitted model is:
 *
 *     reference ~= fitted_scale * aligned_signal + fitted_offset
 *
 * The RMSE is calculated from this fitted aligned signal.
 */
typedef struct {
    int32_t lag_samples;
    float correlation;

    float fitted_scale;
    float fitted_offset;
    float aligned_rmse_codes;

    float signal_mean;
    float signal_rms;
    int16_t signal_min;
    int16_t signal_max;

    size_t sample_count;
} timing_analysis_result_t;

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

/*
 * Run the timing-analysis sequence used by firmware:
 *
 *   circular lag search
 *   -> circular lag application
 *   -> least-squares scale and offset fit
 *   -> aligned fitted RMSE
 *   -> raw signal statistics
 */
int timing_analyze_frame(
    const int16_t *reference,
    const int16_t *signal,
    size_t sample_count,
    int16_t *aligned_output,
    timing_analysis_result_t *result
);

#endif
