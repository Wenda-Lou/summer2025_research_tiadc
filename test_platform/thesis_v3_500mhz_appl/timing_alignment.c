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
     * and therefore matches frame.py.
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
        const size_t source_index = wrap_index(
            (int64_t)i + (int64_t)lag_samples,
            sample_count
        );

        aligned_output[i] = signal[source_index];
    }

    return 0;
}

int timing_analyze_frame(
    const int16_t *reference,
    const int16_t *signal,
    size_t sample_count,
    int16_t *aligned_output,
    timing_analysis_result_t *result
)
{
    timing_alignment_result_t alignment;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    double fit_denominator;
    double scale;
    double offset;
    double squared_error_sum = 0.0;
    double signal_sum = 0.0;
    double signal_square_sum = 0.0;
    int16_t signal_min;
    int16_t signal_max;
    int status;

    if ((reference == NULL) ||
        (signal == NULL) ||
        (aligned_output == NULL) ||
        (result == NULL)) {
        return -1;
    }

    if (sample_count < TIMING_MIN_SAMPLES) {
        return -2;
    }

    status = timing_find_circular_lag(
        reference,
        signal,
        sample_count,
        &alignment
    );

    if (status != 0) {
        return -10 + status;
    }

    status = timing_apply_circular_lag(
        signal,
        sample_count,
        alignment.lag_samples,
        aligned_output
    );

    if (status != 0) {
        return -20 + status;
    }

    /*
     * Match receive_data.py exactly:
     *
     * design = np.column_stack((aligned, np.ones(n)))
     * scale, offset = np.linalg.lstsq(design, reference)[0]
     */
    for (size_t i = 0U; i < sample_count; ++i) {
        const double x = (double)aligned_output[i];
        const double y = (double)reference[i];

        sum_x += x;
        sum_y += y;
        sum_xx += x * x;
        sum_xy += x * y;
    }

    fit_denominator =
        ((double)sample_count * sum_xx) - (sum_x * sum_x);

    if (fabs(fit_denominator) <= TIMING_EPSILON) {
        return -3;
    }

    scale =
        (((double)sample_count * sum_xy) - (sum_x * sum_y)) /
        fit_denominator;

    offset =
        (sum_y - (scale * sum_x)) / (double)sample_count;

    signal_min = signal[0];
    signal_max = signal[0];

    for (size_t i = 0U; i < sample_count; ++i) {
        const double fitted =
            scale * (double)aligned_output[i] + offset;
        const double error =
            fitted - (double)reference[i];
        const double raw_signal =
            (double)signal[i];

        squared_error_sum += error * error;
        signal_sum += raw_signal;
        signal_square_sum += raw_signal * raw_signal;

        if (signal[i] < signal_min) {
            signal_min = signal[i];
        }

        if (signal[i] > signal_max) {
            signal_max = signal[i];
        }
    }

    result->lag_samples = alignment.lag_samples;
    result->correlation = alignment.correlation;
    result->fitted_scale = (float)scale;
    result->fitted_offset = (float)offset;
    result->aligned_rmse_codes =
        (float)sqrt(squared_error_sum / (double)sample_count);
    result->signal_mean =
        (float)(signal_sum / (double)sample_count);
    result->signal_rms =
        (float)sqrt(signal_square_sum / (double)sample_count);
    result->signal_min = signal_min;
    result->signal_max = signal_max;
    result->sample_count = sample_count;

    return 0;
}
