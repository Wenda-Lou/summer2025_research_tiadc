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
        const double ref_centered =
            (double)reference[i] - reference_mean;
        const double signal_centered =
            (double)signal[i] - signal_mean;

        reference_power += ref_centered * ref_centered;
        signal_power += signal_centered * signal_centered;
    }

    normalization = sqrt(reference_power * signal_power);

    if (normalization <= TIMING_EPSILON) {
        return -3;
    }

    /*
     * This is mathematically equivalent to:
     *
     * corr = ifft(fft(signal) * conj(fft(reference))).real
     *
     * and provides the same circular lag convention used throughout firmware.
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

        /*
         * np.argmax keeps the first maximum, so replace only when the
         * score is strictly greater.
         */
        if (score > best_score) {
            best_score = score;
            best_lag = lag;
        }
    }

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

int timing_estimate_fractional_lag(
    const int16_t *reference,
    const int16_t *signal,
    size_t sample_count,
    int32_t integer_lag,
    float *fractional_lag
)
{
    double reference_mean = 0.0;
    double signal_mean = 0.0;
    double scores[3] = {0.0, 0.0, 0.0};
    double denominator;
    double fraction;

    if ((reference == NULL) || (signal == NULL) ||
        (fractional_lag == NULL)) {
        return -1;
    }
    if (sample_count < TIMING_MIN_SAMPLES) {
        return -2;
    }
    if ((integer_lag < -(int32_t)(sample_count / 2U)) ||
        (integer_lag > (int32_t)(sample_count / 2U))) {
        return -3;
    }

    for (size_t i = 0U; i < sample_count; ++i) {
        reference_mean += (double)reference[i];
        signal_mean += (double)signal[i];
    }
    reference_mean /= (double)sample_count;
    signal_mean /= (double)sample_count;

    for (int neighbor = -1; neighbor <= 1; ++neighbor) {
        const int32_t lag = integer_lag + neighbor;
        double score = 0.0;

        for (size_t i = 0U; i < sample_count; ++i) {
            const size_t signal_index = wrap_index(
                (int64_t)i + (int64_t)lag,
                sample_count
            );
            score += ((double)reference[i] - reference_mean) *
                     ((double)signal[signal_index] - signal_mean);
        }
        scores[neighbor + 1] = score;
    }

    denominator = scores[0] - (2.0 * scores[1]) + scores[2];
    if (!isfinite(denominator) || fabs(denominator) <= TIMING_EPSILON) {
        *fractional_lag = 0.0f;
        return -4;
    }

    fraction = 0.5 * (scores[0] - scores[2]) / denominator;
    if (!isfinite(fraction) || (fraction < -0.5) || (fraction > 0.5)) {
        *fractional_lag = 0.0f;
        return -5;
    }

    *fractional_lag = (float)fraction;
    return 0;
}
