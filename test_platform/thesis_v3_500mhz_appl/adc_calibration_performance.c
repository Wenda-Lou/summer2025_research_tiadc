#include "adc_calibration_performance.h"
#include "adc_test_config.h"

#include <float.h>
#include <math.h>
#include <string.h>

static int adc_cal_double_isfinite(double value)
{
    return value == value && value <= DBL_MAX && value >= -DBL_MAX;
}

typedef struct {
    size_t count;
    double mean;
    double m2;
} perf_stat_t;

static void stat_add(perf_stat_t *s, double value)
{
    const double delta = value - s->mean;
    if (!adc_cal_double_isfinite(value)) return;
    ++s->count;
    s->mean += delta / (double)s->count;
    s->m2 += delta * (value - s->mean);
}

static float stat_mean_or_nan(const perf_stat_t *s)
{
    return s != NULL && s->count > 0U ? (float)s->mean : NAN;
}

void adc_cal_perf_default_config(adc_cal_perf_config_t *config)
{
    if (config == NULL) return;
    config->sample_count = 800U;
    config->sample_rate_hz = ADC_CONFIGURED_SAMPLE_RATE_HZ;
    config->expected_fundamental_hz = 350000000.0;
    config->frame_count = ADC_CAL_PERFORMANCE_DEFAULT_FRAMES;
    config->minimum_valid_frames = ADC_CAL_PERFORMANCE_MIN_VALID_FRAMES;
    config->final_gain_correction = 1.0;
    config->final_offset_correction = 0.0;
    config->nominal_system_gain = 1.0;
    config->canonical_channel = 0;
    config->channel_polarity[0] = 1.0;
    config->channel_polarity[1] = 1.0;
    config->initial_relative_skew_samples = NAN;
    config->initial_relative_skew_ps = NAN;
    config->final_relative_skew_samples = NAN;
    config->final_relative_skew_ps = NAN;
    config->baseline_a = NULL;
    config->baseline_b = NULL;
    config->baseline_frame_stride = 0U;
    config->baseline_frame_count = 0U;
    config->frame_results = NULL;
    config->frame_result_capacity = 0U;
}

int adc_cal_perf_sndr_enob_from_tone_fit(
    double amplitude_codes,
    double rmse_codes,
    double *sndr_db,
    double *enob_bits)
{
    double sndr;
    if (sndr_db == NULL || enob_bits == NULL) return -1;
    *sndr_db = NAN;
    *enob_bits = NAN;
    if (!adc_cal_double_isfinite(amplitude_codes) ||
        !adc_cal_double_isfinite(rmse_codes) ||
        amplitude_codes <= DBL_EPSILON || rmse_codes <= DBL_EPSILON) {
        return -2;
    }
    sndr = 10.0 * log10(0.5 * amplitude_codes * amplitude_codes /
                        (rmse_codes * rmse_codes));
    if (!adc_cal_double_isfinite(sndr)) return -3;
    *sndr_db = sndr;
    *enob_bits = (sndr - 1.76) / 6.02;
    return 0;
}

int adc_cal_perf_resolve_channel_polarity(
    int canonical_channel,
    double canonical_reference_polarity,
    double relative_b_over_a_polarity,
    double channel_polarity[2])
{
    if (channel_polarity == NULL ||
        (canonical_channel != 0 && canonical_channel != 1) ||
        !adc_cal_double_isfinite(canonical_reference_polarity) ||
        !adc_cal_double_isfinite(relative_b_over_a_polarity) ||
        fabs(canonical_reference_polarity) != 1.0 ||
        fabs(relative_b_over_a_polarity) != 1.0) {
        return -1;
    }
    channel_polarity[canonical_channel] = canonical_reference_polarity;
    channel_polarity[1 - canonical_channel] =
        canonical_reference_polarity * relative_b_over_a_polarity;
    return 0;
}

void adc_cal_perf_spectral_reset(adc_cal_perf_spectral_metrics_t *metrics)
{
    if (metrics == NULL) return;
    memset(metrics, 0, sizeof(*metrics));
    metrics->sndr_db = NAN;
    metrics->sfdr_db = NAN;
    metrics->thd_db = NAN;
    metrics->enob = NAN;
    metrics->signal_hz = NAN;
    metrics->worst_spur_hz = NAN;
    metrics->signal_power = NAN;
    metrics->noise_distortion_power = NAN;
    metrics->spur_power = NAN;
    metrics->harmonic_power = NAN;
}

static double goertzel_power(
    const double *windowed_samples,
    size_t sample_count,
    size_t bin)
{
    const double omega = 6.28318530717958647692 *
        (double)bin / (double)sample_count;
    const double coefficient = 2.0 * cos(omega);
    double previous = 0.0;
    double previous2 = 0.0;
    for (size_t i = 0U; i < sample_count; ++i) {
        const double current = windowed_samples[i] +
            coefficient * previous - previous2;
        previous2 = previous;
        previous = current;
    }
    {
        double power = previous2 * previous2 + previous * previous -
            coefficient * previous * previous2;
        if (power < 0.0 && power > -1.0e-6) power = 0.0;
        return power;
    }
}

static double blackman_harris7(size_t index, size_t count)
{
    static const double coefficient[] = {
        0.27105140069342,
        -0.43329793923448,
        0.21812299954311,
        -0.06592544638803,
        0.01081174209837,
        -0.00077658482522,
        0.00001388721735
    };
    double value = 0.0;
    for (size_t i = 0U; i < sizeof(coefficient) / sizeof(coefficient[0]); ++i) {
        value += coefficient[i] * cos(
            6.28318530717958647692 * (double)i * (double)index /
            (double)count);
    }
    return value;
}

static double sum_band_power(
    const double *power,
    size_t power_count,
    size_t center,
    size_t guard)
{
    const size_t first = center > guard ? center - guard : 0U;
    size_t last = center + guard;
    double sum = 0.0;
    if (power == NULL || power_count == 0U) return NAN;
    if (last >= power_count) last = power_count - 1U;
    for (size_t bin = first; bin <= last; ++bin) sum += power[bin];
    return sum;
}

static int power_spectrum(
    const double *samples,
    size_t sample_count,
    double *power,
    size_t power_count)
{
    static double windowed[ADC_CAL_PERFORMANCE_MAX_SAMPLES];
    double mean = 0.0;
    double window_square_mean = 0.0;
    double window_scale;

    if (samples == NULL || power == NULL || sample_count == 0U ||
        sample_count > ADC_CAL_PERFORMANCE_MAX_SAMPLES ||
        power_count < sample_count / 2U + 1U) {
        return -1;
    }
    for (size_t i = 0U; i < sample_count; ++i) {
        if (!adc_cal_double_isfinite(samples[i])) return -2;
        mean += samples[i];
    }
    mean /= (double)sample_count;
    for (size_t i = 0U; i < sample_count; ++i) {
        const double w = blackman_harris7(i, sample_count);
        window_square_mean += w * w;
    }
    window_scale = sqrt(window_square_mean / (double)sample_count);
    if (!adc_cal_double_isfinite(window_scale) || window_scale <= DBL_EPSILON) return -3;
    for (size_t i = 0U; i < sample_count; ++i) {
        const double w = blackman_harris7(i, sample_count) / window_scale;
        windowed[i] = (samples[i] - mean) * w;
        if (!adc_cal_double_isfinite(windowed[i])) return -4;
    }
    for (size_t bin = 0U; bin < sample_count / 2U + 1U; ++bin) {
        const double raw_power =
            goertzel_power(windowed, sample_count, bin);
        power[bin] =
            raw_power / ((double)sample_count * (double)sample_count);
        if (!adc_cal_double_isfinite(power[bin]) || power[bin] < 0.0) return -5;
    }
    return 0;
}

int adc_cal_perf_analyze_record(
    const double *samples,
    size_t sample_count,
    double sample_rate_hz,
    adc_cal_perf_spectral_metrics_t *metrics)
{
    static double power[ADC_CAL_PERFORMANCE_MAX_SAMPLES / 2U + 1U];
    const size_t guard = 8U;
    const size_t dc_bins = 12U;
    const size_t n_harmonics = 5U;
    const size_t power_count = sample_count / 2U + 1U;
    size_t signal_bin = 0U;
    size_t spur_bin = 0U;
    double signal_power;
    double noise_and_distortion = 0.0;
    double harmonic_power = 0.0;
    double spur_power;

    if (metrics == NULL) return -1;
    adc_cal_perf_spectral_reset(metrics);
    if (sample_count == 0U || sample_count > ADC_CAL_PERFORMANCE_MAX_SAMPLES ||
        !adc_cal_double_isfinite(sample_rate_hz) || sample_rate_hz <= 0.0 ||
        power_spectrum(samples, sample_count, power, power_count) != 0) {
        return -2;
    }
    for (size_t bin = dc_bins; bin < power_count; ++bin) {
        if (power[bin] > power[signal_bin]) signal_bin = bin;
    }
    if (signal_bin == 0U) return -3;
    signal_power = sum_band_power(power, power_count, signal_bin, guard);
    if (!adc_cal_double_isfinite(signal_power) || signal_power <= DBL_EPSILON) return -4;
    for (size_t bin = dc_bins; bin < power_count; ++bin) {
        const bool in_signal =
            bin + guard >= signal_bin && bin <= signal_bin + guard;
        if (!in_signal) noise_and_distortion += power[bin];
    }
    for (size_t h = 2U; h <= n_harmonics; ++h) {
        size_t harmonic_bin = h * signal_bin;
        const size_t folded_period = 2U * (power_count - 1U);
        if (folded_period == 0U) continue;
        harmonic_bin %= folded_period;
        if (harmonic_bin > power_count - 1U) {
            harmonic_bin = folded_period - harmonic_bin;
        }
        if (harmonic_bin < dc_bins) continue;
        harmonic_power +=
            sum_band_power(power, power_count, harmonic_bin, guard);
    }
    {
        double worst_power = 0.0;
        for (size_t bin = dc_bins; bin < power_count; ++bin) {
            const bool in_signal =
                bin + guard >= signal_bin && bin <= signal_bin + guard;
            if (!in_signal && power[bin] > worst_power) {
                worst_power = power[bin];
                spur_bin = bin;
            }
        }
    }
    spur_power = sum_band_power(power, power_count, spur_bin, guard);
    metrics->signal_bin = signal_bin;
    metrics->worst_spur_bin = spur_bin;
    metrics->signal_hz =
        (double)signal_bin * sample_rate_hz / (double)sample_count;
    metrics->worst_spur_hz =
        (double)spur_bin * sample_rate_hz / (double)sample_count;
    metrics->signal_power = signal_power;
    metrics->noise_distortion_power = noise_and_distortion;
    metrics->spur_power = spur_power;
    metrics->harmonic_power = harmonic_power;
    metrics->sndr_db = noise_and_distortion > 0.0 ?
        (float)(10.0 * log10(signal_power / noise_and_distortion)) :
        INFINITY;
    metrics->sfdr_db = spur_power > 0.0 ?
        (float)(10.0 * log10(signal_power / spur_power)) : INFINITY;
    metrics->thd_db = harmonic_power > 0.0 ?
        (float)(10.0 * log10(harmonic_power / signal_power)) : -INFINITY;
    metrics->enob = (metrics->sndr_db - 1.76f) / 6.02f;
    return adc_cal_double_isfinite(metrics->signal_hz) &&
        adc_cal_double_isfinite(metrics->worst_spur_hz) &&
        adc_cal_double_isfinite(metrics->signal_power) &&
        adc_cal_double_isfinite(metrics->noise_distortion_power) ? 0 : -5;
}

static float correlation(
    const double *a,
    const double *b,
    size_t count)
{
    double mean_a = 0.0;
    double mean_b = 0.0;
    double num = 0.0;
    double pa = 0.0;
    double pb = 0.0;
    for (size_t i = 0U; i < count; ++i) {
        mean_a += a[i];
        mean_b += b[i];
    }
    mean_a /= (double)count;
    mean_b /= (double)count;
    for (size_t i = 0U; i < count; ++i) {
        const double da = a[i] - mean_a;
        const double db = b[i] - mean_b;
        num += da * db;
        pa += da * da;
        pb += db * db;
    }
    return pa > DBL_EPSILON && pb > DBL_EPSILON ?
        (float)(num / sqrt(pa * pb)) : NAN;
}

static void matching_reset(adc_cal_perf_matching_metrics_t *metrics)
{
    if (metrics == NULL) return;
    memset(metrics, 0, sizeof(*metrics));
    metrics->correlation = NAN;
    metrics->waveform_rmse_codes = NAN;
    metrics->residual_dbc = NAN;
    metrics->offset_mismatch_codes = NAN;
    metrics->gain_ratio_b_over_a = NAN;
    metrics->gain_mismatch = NAN;
    metrics->relative_skew_samples = NAN;
    metrics->relative_skew_ps = NAN;
}

static int analyze_matching(
    const double *a,
    const double *b,
    size_t count,
    double polarity_a,
    double polarity_b,
    double relative_skew_samples,
    double relative_skew_ps,
    double applied_gain_correction,
    double applied_offset_correction,
    adc_cal_perf_matching_metrics_t *metrics)
{
    double mean_a = 0.0;
    double mean_b = 0.0;
    double residual_power = 0.0;
    double ac_power_a = 0.0;
    double ac_power_b = 0.0;
    double covariance = 0.0;
    double signal_rms;

    if (metrics == NULL) return -1;
    matching_reset(metrics);
    if (a == NULL || b == NULL || count == 0U ||
        !adc_cal_double_isfinite(polarity_a) ||
        !adc_cal_double_isfinite(polarity_b) ||
        fabs(polarity_a) != 1.0 || fabs(polarity_b) != 1.0 ||
        !adc_cal_double_isfinite(applied_gain_correction) ||
        applied_gain_correction <= 0.0 ||
        !adc_cal_double_isfinite(applied_offset_correction)) return -2;

    for (size_t i = 0U; i < count; ++i) {
        const double normalized_a = polarity_a * a[i];
        const double normalized_b = polarity_b * b[i];
        if (!adc_cal_double_isfinite(normalized_a) ||
            !adc_cal_double_isfinite(normalized_b)) return -3;
        mean_a += normalized_a;
        mean_b += normalized_b;
    }
    mean_a /= (double)count;
    mean_b /= (double)count;
    for (size_t i = 0U; i < count; ++i) {
        const double normalized_a = polarity_a * a[i];
        const double normalized_b = polarity_b * b[i];
        const double centered_a = normalized_a - mean_a;
        const double centered_b = normalized_b - mean_b;
        const double residual = normalized_a - normalized_b;
        residual_power += residual * residual;
        ac_power_a += centered_a * centered_a;
        ac_power_b += centered_b * centered_b;
        covariance += centered_a * centered_b;
    }
    /* The inputs may already carry the shared production gain/offset
     * correction.  That correction cannot change physical A/B matching, so
     * the reported offset mismatch must remove the polarity-weighted
     * artifact (polarity_a - polarity_b) * gain * offset it introduces;
     * with opposite channel polarities a shared offset would otherwise
     * appear twice.  RMSE, correlation and the gain ratio are centered or
     * ratio metrics, so the shared correction cancels in them by
     * construction. */
    metrics->offset_mismatch_codes = (float)(
        (mean_a - mean_b) -
        applied_gain_correction * (polarity_a - polarity_b) *
        applied_offset_correction);
    metrics->waveform_rmse_codes =
        (float)sqrt(residual_power / (double)count);
    if (ac_power_a > DBL_EPSILON && ac_power_b > DBL_EPSILON) {
        metrics->gain_ratio_b_over_a = (float)sqrt(ac_power_b / ac_power_a);
        metrics->gain_mismatch = metrics->gain_ratio_b_over_a - 1.0f;
        metrics->correlation =
            (float)(covariance / sqrt(ac_power_a * ac_power_b));
    }
    signal_rms = sqrt(0.5 * (ac_power_a + ac_power_b) / (double)count);
    if (signal_rms > DBL_EPSILON) {
        metrics->residual_dbc = metrics->waveform_rmse_codes > 0.0f ?
            (float)(20.0 * log10((double)metrics->waveform_rmse_codes /
                                 signal_rms)) : -INFINITY;
    }
    metrics->relative_skew_samples = relative_skew_samples;
    metrics->relative_skew_ps = relative_skew_ps;
    metrics->valid = adc_cal_double_isfinite(metrics->correlation) &&
        adc_cal_double_isfinite(metrics->waveform_rmse_codes) &&
        adc_cal_double_isfinite(metrics->offset_mismatch_codes) &&
        adc_cal_double_isfinite(metrics->gain_ratio_b_over_a) &&
        adc_cal_double_isfinite(metrics->gain_mismatch);
    return metrics->valid ? 0 : -4;
}

int adc_cal_perf_analyze_frame(
    const double *raw_a,
    const double *raw_b,
    const double *cal_a,
    const double *cal_b,
    const double *reference,
    const adc_cal_perf_config_t *config,
    uint32_t frame_number,
    adc_cal_perf_frame_result_t *result)
{
    static double raw_parallel_average[ADC_CAL_PERFORMANCE_MAX_SAMPLES];
    static double cal_parallel_average[ADC_CAL_PERFORMANCE_MAX_SAMPLES];
    static double normalized_cal_a[ADC_CAL_PERFORMANCE_MAX_SAMPLES];
    static double normalized_cal_b[ADC_CAL_PERFORMANCE_MAX_SAMPLES];
    double residual_sum = 0.0;
    double residual_square_sum = 0.0;
    double residual_a_square_sum = 0.0;
    double residual_b_square_sum = 0.0;
    double residual_before_square_sum = 0.0;
    double raw_cal_b_square_sum = 0.0;
    double raw_cal_a_square_sum = 0.0;
    double raw_cal_b_max = 0.0;
    double raw_cal_a_max = 0.0;
    const double *raw_canonical;
    const double *normalized_cal_canonical;

    if (result == NULL) return -1;
    memset(result, 0, sizeof(*result));
    result->frame_number = frame_number;
    result->parallel_average_available = false;
    result->interleaved_metrics_available = false;
    result->cal_a_reference_correlation = NAN;
    result->cal_b_reference_correlation = NAN;
    result->cal_a_reference_rmse_codes = NAN;
    result->cal_b_reference_rmse_codes = NAN;
    result->failure_reason = "not evaluated";
    adc_cal_perf_spectral_reset(&result->raw_a);
    adc_cal_perf_spectral_reset(&result->raw_b);
    adc_cal_perf_spectral_reset(&result->cal_a);
    adc_cal_perf_spectral_reset(&result->cal_b);
    adc_cal_perf_spectral_reset(&result->raw_parallel_average);
    adc_cal_perf_spectral_reset(&result->cal_parallel_average);
    matching_reset(&result->raw_matching);
    matching_reset(&result->cal_matching);
    if (raw_a == NULL || raw_b == NULL || cal_a == NULL || cal_b == NULL ||
        reference == NULL || config == NULL || config->sample_count == 0U ||
        config->sample_count > ADC_CAL_PERFORMANCE_MAX_SAMPLES ||
        (config->canonical_channel != 0 && config->canonical_channel != 1) ||
        !adc_cal_double_isfinite(config->sample_rate_hz) || config->sample_rate_hz <= 0.0) {
        result->failure_reason = "invalid performance input";
        return -2;
    }
    result->sample_count = config->sample_count;
    result->sample_rate_hz = config->sample_rate_hz;
    result->expected_fundamental_hz = config->expected_fundamental_hz;
    raw_canonical = config->canonical_channel == 0 ? raw_a : raw_b;
    normalized_cal_canonical = config->canonical_channel == 0 ?
        normalized_cal_a : normalized_cal_b;
    {
        /* Polarity self-correction: the canonical channel's normalized cal
         * must correlate positively with the reference.  Board evidence
         * (2026-08-19): the Stage-4/timing channel-sign bookkeeping can
         * disagree with the actual signal (polarity[A] = -1 while cal_a is
         * in phase with the reference), which flipped the canonical
         * correlation to -0.998 and inflated the reference RMSE ~40x
         * (759 vs 19 codes).  A global two-channel flip preserves the A/B
         * relation and leaves the spectral metrics and matching unchanged
         * (both are sign-invariant).  NaN correlation (non-finite input)
         * is left untouched and caught by the finite checks below. */
        double polarity_a = config->channel_polarity[0];
        double polarity_b = config->channel_polarity[1];
        for (size_t i = 0U; i < config->sample_count; ++i) {
            normalized_cal_a[i] = polarity_a * cal_a[i];
            normalized_cal_b[i] = polarity_b * cal_b[i];
        }
        if (correlation(normalized_cal_canonical, reference,
                        config->sample_count) < 0.0) {
            polarity_a = -polarity_a;
            polarity_b = -polarity_b;
            for (size_t i = 0U; i < config->sample_count; ++i) {
                normalized_cal_a[i] = -normalized_cal_a[i];
                normalized_cal_b[i] = -normalized_cal_b[i];
            }
        }
        for (size_t i = 0U; i < config->sample_count; ++i) {
            const double before_polarity = config->final_gain_correction *
                (raw_canonical[i] + config->final_offset_correction);
            const double residual_before = before_polarity - reference[i];
            const double a_difference = raw_a[i] - cal_a[i];
            const double residual = normalized_cal_canonical[i] - reference[i];
            const double residual_a = normalized_cal_a[i] - reference[i];
            const double residual_b = normalized_cal_b[i] - reference[i];
            const double b_difference = raw_b[i] - cal_b[i];
            if (!adc_cal_double_isfinite(raw_a[i]) || !adc_cal_double_isfinite(raw_b[i]) ||
                !adc_cal_double_isfinite(cal_a[i]) || !adc_cal_double_isfinite(cal_b[i]) ||
                !adc_cal_double_isfinite(reference[i])) {
                result->failure_reason = "non-finite performance sample";
                return -3;
            }
            residual_sum += residual;
            residual_square_sum += residual * residual;
            residual_a_square_sum += residual_a * residual_a;
            residual_b_square_sum += residual_b * residual_b;
            residual_before_square_sum += residual_before * residual_before;
            raw_parallel_average[i] = 0.5 *
                (polarity_a * raw_a[i] + polarity_b * raw_b[i]);
            cal_parallel_average[i] = 0.5 *
                (normalized_cal_a[i] + normalized_cal_b[i]);
            raw_cal_b_square_sum += b_difference * b_difference;
            raw_cal_a_square_sum += a_difference * a_difference;
            if (fabs(b_difference) > raw_cal_b_max)
                raw_cal_b_max = fabs(b_difference);
            if (fabs(a_difference) > raw_cal_a_max)
                raw_cal_a_max = fabs(a_difference);
        }
    }
    result->mean_residual =
        (float)(residual_sum / (double)config->sample_count);
    result->rmse =
        (float)sqrt(residual_square_sum / (double)config->sample_count);
    result->rmse_before_polarity =
        (float)sqrt(residual_before_square_sum /
                    (double)config->sample_count);
    result->correlation =
        correlation(normalized_cal_canonical, reference,
                    config->sample_count);
    result->cal_a_reference_correlation =
        correlation(normalized_cal_a, reference, config->sample_count);
    result->cal_b_reference_correlation =
        correlation(normalized_cal_b, reference, config->sample_count);
    result->cal_a_reference_rmse_codes = (float)sqrt(
        residual_a_square_sum / (double)config->sample_count);
    result->cal_b_reference_rmse_codes = (float)sqrt(
        residual_b_square_sum / (double)config->sample_count);
    result->correlation_before_polarity =
        correlation(raw_canonical, reference, config->sample_count);
    result->raw_cal_b_rms_difference = (float)sqrt(
        raw_cal_b_square_sum / (double)config->sample_count);
    result->raw_cal_b_max_abs_difference = (float)raw_cal_b_max;
    result->raw_cal_a_rms_difference = (float)sqrt(
        raw_cal_a_square_sum / (double)config->sample_count);
    result->raw_cal_a_max_abs_difference = (float)raw_cal_a_max;
    result->raw_a_address = (uintptr_t)raw_a;
    result->cal_a_address = (uintptr_t)cal_a;
    result->raw_b_address = (uintptr_t)raw_b;
    result->cal_b_address = (uintptr_t)cal_b;
    result->raw_cal_a_identical = raw_cal_a_max <= DBL_EPSILON;
    result->raw_cal_b_identical = raw_cal_b_max <= DBL_EPSILON;
    result->raw_cal_buffers_identical =
        result->raw_cal_a_identical && result->raw_cal_b_identical;
    if (adc_cal_perf_analyze_record(raw_a, config->sample_count,
                                    config->sample_rate_hz,
                                    &result->raw_a) != 0 ||
        adc_cal_perf_analyze_record(raw_b, config->sample_count,
                                    config->sample_rate_hz,
                                    &result->raw_b) != 0 ||
        adc_cal_perf_analyze_record(cal_a, config->sample_count,
                                    config->sample_rate_hz,
                                    &result->cal_a) != 0 ||
        adc_cal_perf_analyze_record(cal_b, config->sample_count,
                                    config->sample_rate_hz,
                                    &result->cal_b) != 0 ||
        adc_cal_perf_analyze_record(raw_parallel_average, config->sample_count,
                                    config->sample_rate_hz,
                                    &result->raw_parallel_average) != 0 ||
        adc_cal_perf_analyze_record(cal_parallel_average, config->sample_count,
                                    config->sample_rate_hz,
                                    &result->cal_parallel_average) != 0 ||
        analyze_matching(raw_a, raw_b, config->sample_count,
                         config->channel_polarity[0],
                         config->channel_polarity[1],
                         config->initial_relative_skew_samples,
                         config->initial_relative_skew_ps,
                         1.0, 0.0,
                         &result->raw_matching) != 0 ||
        analyze_matching(cal_a, cal_b, config->sample_count,
                         config->channel_polarity[0],
                         config->channel_polarity[1],
                         config->final_relative_skew_samples,
                         config->final_relative_skew_ps,
                         config->final_gain_correction,
                         config->final_offset_correction,
                         &result->cal_matching) != 0) {
        result->failure_reason = "spectral performance analysis failed";
        return -4;
    }
    result->sndr_db = result->cal_parallel_average.sndr_db;
    result->sfdr_db = result->cal_parallel_average.sfdr_db;
    result->thd_db = result->cal_parallel_average.thd_db;
    result->enob = result->cal_parallel_average.enob;
    result->normalized_gain = 1.0f;
    result->parallel_average_available = true;
    result->valid = adc_cal_double_isfinite(result->sndr_db) && adc_cal_double_isfinite(result->enob);
    result->failure_reason = result->valid ? "none" :
        "invalid spectral metrics";
    return result->valid ? 0 : -5;
}

int adc_cal_perf_run_batch(
    const adc_cal_perf_config_t *config,
    adc_cal_perf_capture_fn capture,
    void *context,
    adc_cal_perf_batch_result_t *result)
{
    adc_cal_perf_config_t local_config;
    static double raw_a[ADC_CAL_PERFORMANCE_MAX_SAMPLES];
    static double raw_b[ADC_CAL_PERFORMANCE_MAX_SAMPLES];
    static double cal_a[ADC_CAL_PERFORMANCE_MAX_SAMPLES];
    static double cal_b[ADC_CAL_PERFORMANCE_MAX_SAMPLES];
    static double reference[ADC_CAL_PERFORMANCE_MAX_SAMPLES];
    perf_stat_t sndr = {0U, 0.0, 0.0};
    perf_stat_t sfdr = {0U, 0.0, 0.0};
    perf_stat_t thd = {0U, 0.0, 0.0};
    perf_stat_t enob = {0U, 0.0, 0.0};
    perf_stat_t raw_sndr = {0U, 0.0, 0.0};
    perf_stat_t raw_sfdr = {0U, 0.0, 0.0};
    perf_stat_t raw_thd = {0U, 0.0, 0.0};
    perf_stat_t raw_enob = {0U, 0.0, 0.0};
    perf_stat_t residual = {0U, 0.0, 0.0};
    perf_stat_t rmse = {0U, 0.0, 0.0};
    perf_stat_t corr = {0U, 0.0, 0.0};

    if (result == NULL) return -1;
    memset(result, 0, sizeof(*result));
    result->failure_reason = "performance batch not evaluated";
    if (capture == NULL) {
        result->failure_reason = "missing performance capture callback";
        return -2;
    }
    if (config == NULL) {
        adc_cal_perf_default_config(&local_config);
        config = &local_config;
    }
    if (config->frame_count == 0U || config->sample_count == 0U ||
        config->sample_count > ADC_CAL_PERFORMANCE_MAX_SAMPLES ||
        !adc_cal_double_isfinite(config->final_gain_correction) ||
        config->final_gain_correction <= 0.0 ||
        !adc_cal_double_isfinite(config->final_offset_correction) ||
        !adc_cal_double_isfinite(config->nominal_system_gain) ||
        config->nominal_system_gain <= 0.0 ||
        (config->canonical_channel != 0 && config->canonical_channel != 1) ||
        !adc_cal_double_isfinite(config->channel_polarity[0]) ||
        !adc_cal_double_isfinite(config->channel_polarity[1]) ||
        fabs(config->channel_polarity[0]) != 1.0 ||
        fabs(config->channel_polarity[1]) != 1.0 ||
        ((config->baseline_a == NULL) != (config->baseline_b == NULL)) ||
        (config->baseline_a != NULL &&
         (config->baseline_frame_stride < config->sample_count ||
          config->baseline_frame_count < config->frame_count))) {
        result->failure_reason = "invalid performance configuration";
        return -3;
    }
    result->frames_attempted = config->frame_count;
    for (uint32_t frame = 1U; frame <= config->frame_count; ++frame) {
        adc_cal_perf_frame_result_t frame_result;
        const double *analysis_raw_a = raw_a;
        const double *analysis_raw_b = raw_b;
        const char *reason = NULL;
        size_t captured_count = 0U;
        memset(&frame_result, 0, sizeof(frame_result));
        frame_result.frame_number = frame;
        frame_result.failure_reason = "performance frame not evaluated";
        int status = capture(context, raw_a, raw_b, reference,
                             config->sample_count, &captured_count, &reason);
        if (status == 0 && captured_count == config->sample_count) {
            for (size_t i = 0U; i < config->sample_count; ++i) {
                const double expected =
                    config->nominal_system_gain * reference[i];
                cal_a[i] = config->final_gain_correction *
                    (raw_a[i] + config->final_offset_correction);
                cal_b[i] = config->final_gain_correction *
                    (raw_b[i] + config->final_offset_correction);
                reference[i] = expected;
                if (!adc_cal_double_isfinite(cal_a[i]) ||
                    !adc_cal_double_isfinite(cal_b[i]) ||
                    !adc_cal_double_isfinite(reference[i])) {
                    status = -5;
                    reason = "non-finite corrected performance sample";
                    break;
                }
            }
        }
        if (status == 0 && config->baseline_a != NULL) {
            const size_t baseline_offset = (size_t)(frame - 1U) *
                config->baseline_frame_stride;
            analysis_raw_a = config->baseline_a + baseline_offset;
            analysis_raw_b = config->baseline_b + baseline_offset;
        }
        if (status == 0 && captured_count == config->sample_count) {
            status = adc_cal_perf_analyze_frame(
                analysis_raw_a, analysis_raw_b, cal_a, cal_b, reference,
                config, frame,
                &frame_result);
        }
        if (config->frame_results != NULL &&
            (size_t)(frame - 1U) < config->frame_result_capacity) {
            if (status == 0 || frame_result.frame_number == frame) {
                config->frame_results[frame - 1U] = frame_result;
            } else {
                memset(&config->frame_results[frame - 1U], 0,
                       sizeof(config->frame_results[frame - 1U]));
                config->frame_results[frame - 1U].frame_number = frame;
                config->frame_results[frame - 1U].failure_reason =
                    reason != NULL ? reason : "performance capture failed";
            }
        }
        if (status == 0 && frame_result.valid) {
            ++result->frames_valid;
            stat_add(&sndr, frame_result.cal_parallel_average.sndr_db);
            stat_add(&sfdr, frame_result.cal_parallel_average.sfdr_db);
            stat_add(&thd, frame_result.cal_parallel_average.thd_db);
            stat_add(&enob, frame_result.cal_parallel_average.enob);
            stat_add(&raw_sndr, frame_result.raw_parallel_average.sndr_db);
            stat_add(&raw_sfdr, frame_result.raw_parallel_average.sfdr_db);
            stat_add(&raw_thd, frame_result.raw_parallel_average.thd_db);
            stat_add(&raw_enob, frame_result.raw_parallel_average.enob);
            stat_add(&residual, frame_result.mean_residual);
            stat_add(&rmse, frame_result.rmse);
            stat_add(&corr, frame_result.correlation);
        } else {
            (void)reason;
            ++result->frames_rejected;
        }
    }
    result->cal_parallel_average_sndr_db = stat_mean_or_nan(&sndr);
    result->cal_parallel_average_sfdr_db = stat_mean_or_nan(&sfdr);
    result->cal_parallel_average_thd_db = stat_mean_or_nan(&thd);
    result->cal_parallel_average_enob = stat_mean_or_nan(&enob);
    result->raw_parallel_average_sndr_db = stat_mean_or_nan(&raw_sndr);
    result->raw_parallel_average_sfdr_db = stat_mean_or_nan(&raw_sfdr);
    result->raw_parallel_average_thd_db = stat_mean_or_nan(&raw_thd);
    result->raw_parallel_average_enob = stat_mean_or_nan(&raw_enob);
    result->mean_residual = stat_mean_or_nan(&residual);
    result->rmse = stat_mean_or_nan(&rmse);
    result->correlation = stat_mean_or_nan(&corr);
    result->spectral_metrics_valid =
        sndr.count > 0U && enob.count > 0U &&
        adc_cal_double_isfinite(result->cal_parallel_average_sndr_db) &&
        adc_cal_double_isfinite(result->cal_parallel_average_enob);
    result->valid =
        result->frames_valid >= config->minimum_valid_frames &&
        result->spectral_metrics_valid;
    result->failure_reason = result->valid ? "none" :
        result->frames_valid < config->minimum_valid_frames ?
        "insufficient valid performance frames" :
        "invalid aggregate performance metrics";
    return result->valid ? 0 : -4;
}
