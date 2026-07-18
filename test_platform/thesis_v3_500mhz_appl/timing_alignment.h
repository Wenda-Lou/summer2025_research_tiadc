#ifndef TIMING_ALIGNMENT_H
#define TIMING_ALIGNMENT_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    int32_t lag_samples;
    float correlation;
} timing_alignment_result_t;

/*
 * Complete board-side equivalent of the proven Python timing analysis.
 *
 * After circular alignment, the fitted model is:
 *
 *     reference ~= fitted_scale * aligned_signal + fitted_offset
 *
 * The RMSE is calculated from this fitted aligned signal, matching
 * receive_data.py.
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
 * The returned lag matches the Python convention:
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

/*
 * Run the exact timing-analysis sequence used by Python:
 *
 *   circular lag search
 *   -> np.roll(signal, -lag)
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
