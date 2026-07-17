#include "timing_alignment.h"

#include <float.h>
#include <math.h>

#define TIMING_MIN_SAMPLES 16U
#define TIMING_EPSILON     1.0e-20

static size_t wrap_index(int64_t index, size_t sample_count)
{
    int64_t wrapped = index % (int64_t)sample_count;

    if (wrapped < 0) {
        wrapped += (int64_t)sample_count;
    }

    return (size_t)wrapped;
}

int timing_find_circular_lag(
    const int16_t *reference,
    const int16_t *signal,
    size_t sample_count,
    timing_alignment_result_t *result
)
{
    double reference_mean = 0.0;
    double signal_mean = 0.0;
    double reference_power = 0.0;
    double signal_power = 0.0;
    double normalization;
    double best_score = -DBL_MAX;
    size_t best_lag = 0U;

    if ((reference == NULL) ||
        (signal == NULL) ||
        (result == NULL)) {
        return -1;
    }

    if (sample_count < TIMING_MIN_SAMPLES) {
        return -2;
    }

    for (size_t i = 0U; i < sample_count; ++i) {
        reference_mean += (double)reference[i];
        signal_mean += (double)signal[i];
    }

    reference_mean /= (double)sample_count;
    signal_mean /= (double)sample_count;

    for (size_t i = 0U; i < sample_count; ++i) {
        double ref_centered =
            (double)reference[i] - reference_mean;

        double signal_centered =
            (double)signal[i] - signal_mean;

        reference_power += ref_centered * ref_centered;
        signal_power += signal_centered * signal_centered;
    }

    normalization = sqrt(reference_power * signal_power);

    if (normalization <= TIMING_EPSILON) {
        return -3;
    }

    /*
     * score(lag) =
     *     sum(reference[i] * signal[(i + lag) mod N])
     *
     * After finding lag:
     *     aligned[i] = signal[(i + lag) mod N]
     *
     * This corresponds to np.roll(signal, -lag).
     */
    for (size_t lag = 0U; lag < sample_count; ++lag) {
        double score = 0.0;

        for (size_t i = 0U; i < sample_count; ++i) {
            size_t signal_index = i + lag;

            if (signal_index >= sample_count) {
                signal_index -= sample_count;
            }

            score +=
                ((double)reference[i] - reference_mean) *
                ((double)signal[signal_index] - signal_mean);
        }

        if (score > best_score) {
            best_score = score;
            best_lag = lag;
        }
    }

    /*
     * Convert an unsigned circular index into a signed lag:
     *
     * 0 ... N/2       -> positive lag
     * N/2+1 ... N-1   -> negative lag
     */
    if (best_lag <= sample_count / 2U) {
        result->lag_samples = (int32_t)best_lag;
    } else {
        result->lag_samples =
            (int32_t)best_lag - (int32_t)sample_count;
    }

    result->correlation =
        (float)(best_score / normalization);

    return 0;
}

int timing_apply_circular_lag(
    const int16_t *signal,
    size_t sample_count,
    int32_t lag_samples,
    int16_t *aligned_output
)
{
    if ((signal == NULL) || (aligned_output == NULL)) {
        return -1;
    }

    if (sample_count == 0U) {
        return -2;
    }

    for (size_t i = 0U; i < sample_count; ++i) {
        size_t source_index = wrap_index(
            (int64_t)i + (int64_t)lag_samples,
            sample_count
        );

        aligned_output[i] = signal[source_index];
    }

    return 0;
}