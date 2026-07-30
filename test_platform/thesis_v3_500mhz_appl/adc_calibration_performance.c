#include "adc_calibration_performance.h"

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
    config->sample_rate_hz = 1450000000.0;
    config->expected_fundamental_hz = 350000000.0;
    config->frame_count = ADC_CAL_PERFORMANCE_DEFAULT_FRAMES;
    config->minimum_valid_frames = ADC_CAL_PERFORMANCE_MIN_VALID_FRAMES;
    config->final_gain_correction = 1.0;
    config->final_offset_correction = 0.0;
    config->nominal_system_gain = 1.0;
    config->combined_uses_channel_a = false;
    config->frame_results = NULL;
    config->frame_result_capacity = 0U;
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

static int channel_difference_dbc(
    const double *a,
    const double *b,
    size_t sample_count,
    double sample_rate_hz,
    double input_frequency_hz,
    float *difference_dbc,
    float *dc_difference_codes)
{
    static double pa[ADC_CAL_PERFORMANCE_MAX_SAMPLES / 2U + 1U];
    static double pd[ADC_CAL_PERFORMANCE_MAX_SAMPLES / 2U + 1U];
    static double difference[ADC_CAL_PERFORMANCE_MAX_SAMPLES];
    const size_t power_count = sample_count / 2U + 1U;
    const size_t guard = 8U;
    size_t input_bin;
    double carrier;
    double residual;
    double mean_a = 0.0;
    double mean_b = 0.0;

    if (a == NULL || b == NULL || difference_dbc == NULL ||
        dc_difference_codes == NULL || sample_count == 0U ||
        sample_count > ADC_CAL_PERFORMANCE_MAX_SAMPLES ||
        !adc_cal_double_isfinite(sample_rate_hz) || sample_rate_hz <= 0.0 ||
        !adc_cal_double_isfinite(input_frequency_hz) || input_frequency_hz <= 0.0) {
        return -1;
    }
    input_bin = (size_t)lround(
        input_frequency_hz / (sample_rate_hz / (double)sample_count));
    if (input_bin >= power_count) input_bin = power_count - 1U;
    for (size_t i = 0U; i < sample_count; ++i) {
        difference[i] = a[i] - b[i];
        mean_a += a[i];
        mean_b += b[i];
    }
    if (power_spectrum(a, sample_count, pa, power_count) != 0 ||
        power_spectrum(difference, sample_count, pd, power_count) != 0) {
        return -2;
    }
    carrier = sum_band_power(pa, power_count, input_bin, guard);
    residual = sum_band_power(pd, power_count, input_bin, guard);
    *difference_dbc = carrier > 0.0 && residual > 0.0 ?
        (float)(10.0 * log10(residual / carrier)) : -INFINITY;
    *dc_difference_codes =
        (float)(mean_a / (double)sample_count - mean_b / (double)sample_count);
    return 0;
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
    static double raw_combined[ADC_CAL_PERFORMANCE_MAX_SAMPLES];
    static double cal_combined[ADC_CAL_PERFORMANCE_MAX_SAMPLES];
    double residual_sum = 0.0;
    double residual_square_sum = 0.0;

    if (result == NULL) return -1;
    memset(result, 0, sizeof(*result));
    result->frame_number = frame_number;
    result->failure_reason = "not evaluated";
    adc_cal_perf_spectral_reset(&result->raw_a);
    adc_cal_perf_spectral_reset(&result->raw_b);
    adc_cal_perf_spectral_reset(&result->cal_a);
    adc_cal_perf_spectral_reset(&result->cal_b);
    adc_cal_perf_spectral_reset(&result->raw_combined);
    adc_cal_perf_spectral_reset(&result->cal_combined);
    if (raw_a == NULL || raw_b == NULL || cal_a == NULL || cal_b == NULL ||
        reference == NULL || config == NULL || config->sample_count == 0U ||
        config->sample_count > ADC_CAL_PERFORMANCE_MAX_SAMPLES ||
        !adc_cal_double_isfinite(config->sample_rate_hz) || config->sample_rate_hz <= 0.0) {
        result->failure_reason = "invalid performance input";
        return -2;
    }
    result->sample_count = config->sample_count;
    result->sample_rate_hz = config->sample_rate_hz;
    result->expected_fundamental_hz = config->expected_fundamental_hz;
    for (size_t i = 0U; i < config->sample_count; ++i) {
        const double residual = cal_a[i] - reference[i];
        if (!adc_cal_double_isfinite(raw_a[i]) || !adc_cal_double_isfinite(raw_b[i]) ||
            !adc_cal_double_isfinite(cal_a[i]) || !adc_cal_double_isfinite(cal_b[i]) ||
            !adc_cal_double_isfinite(reference[i])) {
            result->failure_reason = "non-finite performance sample";
            return -3;
        }
        residual_sum += residual;
        residual_square_sum += residual * residual;
        if (config->combined_uses_channel_a) {
            raw_combined[i] = raw_a[i];
            cal_combined[i] = cal_a[i];
        } else {
            raw_combined[i] = 0.5 * (raw_a[i] + raw_b[i]);
            cal_combined[i] = 0.5 * (cal_a[i] + cal_b[i]);
        }
    }
    result->mean_residual =
        (float)(residual_sum / (double)config->sample_count);
    result->rmse =
        (float)sqrt(residual_square_sum / (double)config->sample_count);
    result->correlation =
        correlation(cal_a, reference, config->sample_count);
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
        adc_cal_perf_analyze_record(raw_combined, config->sample_count,
                                    config->sample_rate_hz,
                                    &result->raw_combined) != 0 ||
        adc_cal_perf_analyze_record(cal_combined, config->sample_count,
                                    config->sample_rate_hz,
                                    &result->cal_combined) != 0 ||
        channel_difference_dbc(raw_a, raw_b, config->sample_count,
                               config->sample_rate_hz,
                               result->cal_a.signal_hz,
                               &result->raw_difference_dbc,
                               &result->cal_dc_difference_codes) != 0 ||
        channel_difference_dbc(cal_a, cal_b, config->sample_count,
                               config->sample_rate_hz,
                               result->cal_a.signal_hz,
                               &result->cal_difference_dbc,
                               &result->cal_dc_difference_codes) != 0) {
        result->failure_reason = "spectral performance analysis failed";
        return -4;
    }
    result->sndr_db = result->cal_combined.sndr_db;
    result->sfdr_db = result->cal_combined.sfdr_db;
    result->thd_db = result->cal_combined.thd_db;
    result->enob = result->cal_combined.enob;
    result->normalized_gain = 1.0f;
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
        config->nominal_system_gain <= 0.0) {
        result->failure_reason = "invalid performance configuration";
        return -3;
    }
    result->frames_attempted = config->frame_count;
    for (uint32_t frame = 1U; frame <= config->frame_count; ++frame) {
        adc_cal_perf_frame_result_t frame_result;
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
        if (status == 0 && captured_count == config->sample_count) {
            status = adc_cal_perf_analyze_frame(
                raw_a, raw_b, cal_a, cal_b, reference, config, frame,
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
            stat_add(&sndr, frame_result.cal_combined.sndr_db);
            stat_add(&sfdr, frame_result.cal_combined.sfdr_db);
            stat_add(&thd, frame_result.cal_combined.thd_db);
            stat_add(&enob, frame_result.cal_combined.enob);
            stat_add(&raw_sndr, frame_result.raw_combined.sndr_db);
            stat_add(&raw_sfdr, frame_result.raw_combined.sfdr_db);
            stat_add(&raw_thd, frame_result.raw_combined.thd_db);
            stat_add(&raw_enob, frame_result.raw_combined.enob);
            stat_add(&residual, frame_result.mean_residual);
            stat_add(&rmse, frame_result.rmse);
            stat_add(&corr, frame_result.correlation);
        } else {
            (void)reason;
            ++result->frames_rejected;
        }
    }
    result->sndr_db = stat_mean_or_nan(&sndr);
    result->sfdr_db = stat_mean_or_nan(&sfdr);
    result->thd_db = stat_mean_or_nan(&thd);
    result->enob = stat_mean_or_nan(&enob);
    result->raw_sndr_db = stat_mean_or_nan(&raw_sndr);
    result->raw_sfdr_db = stat_mean_or_nan(&raw_sfdr);
    result->raw_thd_db = stat_mean_or_nan(&raw_thd);
    result->raw_enob = stat_mean_or_nan(&raw_enob);
    result->mean_residual = stat_mean_or_nan(&residual);
    result->rmse = stat_mean_or_nan(&rmse);
    result->correlation = stat_mean_or_nan(&corr);
    result->spectral_metrics_valid =
        sndr.count > 0U && enob.count > 0U &&
        adc_cal_double_isfinite(result->sndr_db) && adc_cal_double_isfinite(result->enob);
    result->valid =
        result->frames_valid >= config->minimum_valid_frames &&
        result->spectral_metrics_valid;
    result->failure_reason = result->valid ? "none" :
        result->frames_valid < config->minimum_valid_frames ?
        "insufficient valid performance frames" :
        "invalid aggregate performance metrics";
    return result->valid ? 0 : -4;
}
