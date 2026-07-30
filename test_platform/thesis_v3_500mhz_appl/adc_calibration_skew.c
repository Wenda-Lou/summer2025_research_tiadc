#include "adc_calibration_skew.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>

static int adc_cal_double_isfinite(double value)
{
    return value == value && value <= DBL_MAX && value >= -DBL_MAX;
}

void adc_cal_skew_default_config(adc_cal_skew_config_t *config)
{
    if (config == NULL) return;
    config->minimum_events = ADC_CAL_SKEW_MIN_EVENTS;
    config->sample_rate_hz = 1450000000.0;
    config->max_linear_skew_samples = ADC_CAL_SKEW_MAX_LINEAR_SKEW_SAMPLES;
    config->max_edge_disagreement_samples =
        ADC_CAL_SKEW_MAX_EDGE_DISAGREEMENT_SAMPLES;
}

void adc_cal_skew_result_reset(adc_cal_skew_result_t *result)
{
    if (result == NULL) return;
    memset(result, 0, sizeof(*result));
    result->status = ADC_CAL_SKEW_STATUS_INVALID;
    result->reason = ADC_CAL_SKEW_REASON_NUMERICAL;
    result->channel_a_skew_samples = NAN;
    result->channel_b_skew_samples = NAN;
    result->relative_skew_samples = NAN;
    result->relative_skew_ps = NAN;
    result->rising_skew_samples = NAN;
    result->falling_skew_samples = NAN;
    result->edge_disagreement_samples = NAN;
    result->pulse_energy = NAN;
    result->derivative_energy = NAN;
    result->quality = NAN;
    result->failure_reason = "not evaluated";
}

const char *adc_cal_skew_status_name(adc_cal_skew_status_t status)
{
    switch (status) {
    case ADC_CAL_SKEW_STATUS_PASS: return "PASS";
    case ADC_CAL_SKEW_STATUS_WARNING: return "WARNING";
    case ADC_CAL_SKEW_STATUS_INVALID:
    default:
        return "INVALID";
    }
}

const char *adc_cal_skew_reason_name(adc_cal_skew_reason_t reason)
{
    switch (reason) {
    case ADC_CAL_SKEW_REASON_NONE: return "none";
    case ADC_CAL_SKEW_REASON_CONTEXT: return "context";
    case ADC_CAL_SKEW_REASON_DITHER: return "dither";
    case ADC_CAL_SKEW_REASON_TOO_FEW_EVENTS: return "too few events";
    case ADC_CAL_SKEW_REASON_POLARITY: return "polarity";
    case ADC_CAL_SKEW_REASON_TEMPLATE: return "template";
    case ADC_CAL_SKEW_REASON_DERIVATIVE: return "derivative";
    case ADC_CAL_SKEW_REASON_EDGE_DISAGREEMENT: return "edge disagreement";
    case ADC_CAL_SKEW_REASON_OUTSIDE_LINEAR_RANGE:
        return "outside linear range";
    case ADC_CAL_SKEW_REASON_NUMERICAL:
    default:
        return "numerical";
    }
}

static int estimate_profile_skew(
    const double *profile,
    const double *template_profile,
    const double *derivative_profile,
    size_t count,
    int mask,
    double gain,
    double *skew_samples,
    uint32_t *used_count,
    double *energy)
{
    double numerator = 0.0;
    double denominator = 0.0;
    uint32_t used = 0U;

    if (profile == NULL || template_profile == NULL ||
        derivative_profile == NULL || skew_samples == NULL ||
        used_count == NULL || energy == NULL || count == 0U ||
        !adc_cal_double_isfinite(gain) || fabs(gain) <= DBL_EPSILON) {
        return -1;
    }
    for (size_t i = 0U; i < count; ++i) {
        const double dprime = derivative_profile[i];
        bool select = true;
        if (mask > 0) select = dprime > 0.0;
        else if (mask < 0) select = dprime < 0.0;
        if (!select) continue;
        if (!adc_cal_double_isfinite(profile[i]) || !adc_cal_double_isfinite(template_profile[i]) ||
            !adc_cal_double_isfinite(dprime)) {
            return -2;
        }
        numerator += (profile[i] - gain * template_profile[i]) * dprime;
        denominator += dprime * dprime;
        ++used;
    }
    *used_count = used;
    *energy = denominator;
    if (used == 0U || denominator <= ADC_CAL_SKEW_DERIVATIVE_ENERGY_FLOOR) {
        return -3;
    }
    *skew_samples = numerator / (gain * denominator);
    return adc_cal_double_isfinite(*skew_samples) ? 0 : -4;
}

static int aggregate_profiles(
    const double *a,
    const double *b,
    const double *template_samples,
    size_t sample_count,
    const adc_cal_dither_result_t *events,
    double *profile_a,
    double *profile_b,
    double *template_profile,
    double *derivative_profile,
    size_t *profile_count_out)
{
    double rel_start = -DBL_MAX;
    double rel_end = DBL_MAX;
    int m_first;
    int m_last;
    size_t profile_count;
    static double u_a[ADC_CAL_SKEW_MAX_PROFILE_SAMPLES];
    static double v_a[ADC_CAL_SKEW_MAX_PROFILE_SAMPLES];
    static double u_b[ADC_CAL_SKEW_MAX_PROFILE_SAMPLES];
    static double v_b[ADC_CAL_SKEW_MAX_PROFILE_SAMPLES];
    static double u_t[ADC_CAL_SKEW_MAX_PROFILE_SAMPLES];
    static double v_t[ADC_CAL_SKEW_MAX_PROFILE_SAMPLES];

    for (size_t k = 0U; k < events->accepted_events; ++k) {
        const double left = (double)events->events[k].start -
            events->events[k].center;
        const double right = (double)(events->events[k].end - 1U) -
            events->events[k].center;
        if (left > rel_start) rel_start = left;
        if (right < rel_end) rel_end = right;
    }
    m_first = (int)ceil(rel_start);
    m_last = (int)floor(rel_end);
    if (m_last < m_first) return -1;
    profile_count = (size_t)(m_last - m_first + 1);
    if (profile_count == 0U ||
        profile_count > ADC_CAL_SKEW_MAX_PROFILE_SAMPLES) {
        return -2;
    }
    memset(profile_a, 0,
           ADC_CAL_SKEW_MAX_PROFILE_SAMPLES * sizeof(profile_a[0]));
    memset(profile_b, 0,
           ADC_CAL_SKEW_MAX_PROFILE_SAMPLES * sizeof(profile_b[0]));
    memset(template_profile, 0,
           ADC_CAL_SKEW_MAX_PROFILE_SAMPLES * sizeof(template_profile[0]));
    memset(u_a, 0, sizeof(u_a));
    memset(v_a, 0, sizeof(v_a));
    memset(u_b, 0, sizeof(u_b));
    memset(v_b, 0, sizeof(v_b));
    memset(u_t, 0, sizeof(u_t));
    memset(v_t, 0, sizeof(v_t));
    for (size_t k = 0U; k < events->accepted_events; ++k) {
        const double polarity = events->events[k].polarity;
        for (size_t j = 0U; j < profile_count; ++j) {
            const double position =
                events->events[k].center + (double)(m_first + (int)j);
            double av;
            double bv;
            double tv;
            if (adc_cal_dither_interpolate(a, sample_count, position, &av) != 0 ||
                adc_cal_dither_interpolate(b, sample_count, position, &bv) != 0 ||
                adc_cal_dither_interpolate(template_samples,
                                           sample_count, position, &tv) != 0) {
                return -3;
            }
            u_a[j] += av;
            v_a[j] += polarity * av;
            u_b[j] += bv;
            v_b[j] += polarity * bv;
            u_t[j] += tv;
            v_t[j] += polarity * tv;
        }
    }
    for (size_t j = 0U; j < profile_count; ++j) {
        const double scale = (double)events->accepted_events;
        const double denom = events->separation_denominator;
        if (!adc_cal_double_isfinite(denom) ||
            denom < ADC_CAL_DITHER_DENOMINATOR_FLOOR) {
            return -4;
        }
        u_a[j] /= scale;
        v_a[j] /= scale;
        u_b[j] /= scale;
        v_b[j] /= scale;
        u_t[j] /= scale;
        v_t[j] /= scale;
        profile_a[j] = (v_a[j] - events->mean_polarity * u_a[j]) / denom;
        profile_b[j] = (v_b[j] - events->mean_polarity * u_b[j]) / denom;
        template_profile[j] =
            (v_t[j] - events->mean_polarity * u_t[j]) / denom;
        if (!adc_cal_double_isfinite(profile_a[j]) ||
            !adc_cal_double_isfinite(profile_b[j]) ||
            !adc_cal_double_isfinite(template_profile[j])) {
            return -5;
        }
    }
    for (size_t j = 0U; j < profile_count; ++j) {
        const double previous = j > 0U ? template_profile[j - 1U] :
            template_profile[j];
        const double next = j + 1U < profile_count ? template_profile[j + 1U] :
            template_profile[j];
        derivative_profile[j] = 0.5 * (next - previous);
    }
    *profile_count_out = profile_count;
    return 0;
}

int adc_cal_skew_estimate_from_residuals(
    const double *channel_a_residual,
    const double *channel_b_residual,
    const double *dither_template,
    size_t sample_count,
    const adc_cal_skew_config_t *config,
    adc_cal_skew_result_t *result)
{
    adc_cal_skew_config_t local_config;
    adc_cal_dither_config_t dither_config;
    adc_cal_dither_result_t dither;
    static double profile_a[ADC_CAL_SKEW_MAX_PROFILE_SAMPLES];
    static double profile_b[ADC_CAL_SKEW_MAX_PROFILE_SAMPLES];
    static double template_profile[ADC_CAL_SKEW_MAX_PROFILE_SAMPLES];
    static double derivative_profile[ADC_CAL_SKEW_MAX_PROFILE_SAMPLES];
    size_t profile_count = 0U;
    uint32_t used = 0U;
    double energy = 0.0;
    double skew_a = NAN;
    double skew_b = NAN;
    double rise_a = NAN;
    double rise_b = NAN;
    double fall_a = NAN;
    double fall_b = NAN;
    double gain_a;
    double gain_b;

    if (result == NULL) return -1;
    adc_cal_skew_result_reset(result);
    if (channel_a_residual == NULL || channel_b_residual == NULL ||
        dither_template == NULL) {
        result->reason = ADC_CAL_SKEW_REASON_CONTEXT;
        result->failure_reason = "null skew input";
        return -2;
    }
    if (sample_count == 0U || sample_count > ADC_CAL_DITHER_MAX_EVENTS * 4U) {
        result->reason = ADC_CAL_SKEW_REASON_CONTEXT;
        result->failure_reason = "invalid sample count";
        return -3;
    }
    if (config == NULL) {
        adc_cal_skew_default_config(&local_config);
        config = &local_config;
    }
    if (!adc_cal_double_isfinite(config->sample_rate_hz) || config->sample_rate_hz <= 0.0 ||
        !adc_cal_double_isfinite(config->max_linear_skew_samples) ||
        !adc_cal_double_isfinite(config->max_edge_disagreement_samples)) {
        result->reason = ADC_CAL_SKEW_REASON_CONTEXT;
        result->failure_reason = "invalid skew configuration";
        return -4;
    }
    adc_cal_dither_default_config(&dither_config);
    dither_config.minimum_events = config->minimum_events;
    dither_config.boundary_margin = 1U;
    if (adc_cal_dither_analyze(channel_a_residual, dither_template,
                               sample_count, &dither_config, &dither) != 0) {
        result->reason = dither.accepted_events < config->minimum_events ?
            ADC_CAL_SKEW_REASON_TOO_FEW_EVENTS : ADC_CAL_SKEW_REASON_DITHER;
        result->accepted_events = (uint32_t)dither.accepted_events;
        result->rejected_events = (uint32_t)dither.rejected_events;
        result->failure_reason = adc_cal_dither_status_name(dither.status);
        return -5;
    }
    result->accepted_events = (uint32_t)dither.accepted_events;
    result->rejected_events = (uint32_t)dither.rejected_events;
    if (aggregate_profiles(channel_a_residual, channel_b_residual,
                           dither_template, sample_count, &dither,
                           profile_a, profile_b, template_profile,
                           derivative_profile,
                           &profile_count) != 0) {
        result->reason = ADC_CAL_SKEW_REASON_TEMPLATE;
        result->failure_reason = "event profile aggregation failed";
        return -6;
    }
    gain_a = dither.normalized_projection;
    if (adc_cal_dither_analyze(channel_b_residual, dither_template,
                               sample_count, &dither_config, &dither) != 0) {
        result->reason = ADC_CAL_SKEW_REASON_DITHER;
        result->failure_reason = "channel B dither analysis failed";
        return -7;
    }
    gain_b = dither.normalized_projection;
    if (estimate_profile_skew(profile_a, template_profile, derivative_profile,
                              profile_count, 0, gain_a, &skew_a, &used,
                              &energy) != 0 ||
        estimate_profile_skew(profile_b, template_profile, derivative_profile,
                              profile_count, 0, gain_b, &skew_b, &used,
                              &energy) != 0 ||
        estimate_profile_skew(profile_a, template_profile, derivative_profile,
                              profile_count, 1, gain_a, &rise_a, &used,
                              &energy) != 0 ||
        estimate_profile_skew(profile_b, template_profile, derivative_profile,
                              profile_count, 1, gain_b, &rise_b, &used,
                              &energy) != 0 ||
        estimate_profile_skew(profile_a, template_profile, derivative_profile,
                              profile_count, -1, gain_a, &fall_a, &used,
                              &energy) != 0 ||
        estimate_profile_skew(profile_b, template_profile, derivative_profile,
                              profile_count, -1, gain_b, &fall_b, &used,
                              &energy) != 0) {
        result->reason = ADC_CAL_SKEW_REASON_DERIVATIVE;
        result->failure_reason = "skew derivative projection failed";
        return -8;
    }
    result->channel_a_skew_samples = skew_a;
    result->channel_b_skew_samples = skew_b;
    result->relative_skew_samples = skew_b - skew_a;
    result->rising_skew_samples = rise_b - rise_a;
    result->falling_skew_samples = fall_b - fall_a;
    result->edge_disagreement_samples =
        fabs(result->rising_skew_samples - result->falling_skew_samples);
    result->relative_skew_ps =
        result->relative_skew_samples * 1.0e12 / config->sample_rate_hz;
    result->derivative_energy = energy;
    result->pulse_energy = dither.template_energy;
    result->quality = dither.quality;
    if (!adc_cal_double_isfinite(result->relative_skew_samples) ||
        !adc_cal_double_isfinite(result->edge_disagreement_samples)) {
        result->reason = ADC_CAL_SKEW_REASON_NUMERICAL;
        result->failure_reason = "non-finite skew estimate";
        return -9;
    }
    if (fabs(result->relative_skew_samples) >
        config->max_linear_skew_samples) {
        result->reason = ADC_CAL_SKEW_REASON_OUTSIDE_LINEAR_RANGE;
        result->failure_reason = "skew outside supported linear range";
        return -10;
    }
    if (result->edge_disagreement_samples >
        config->max_edge_disagreement_samples) {
        result->reason = ADC_CAL_SKEW_REASON_EDGE_DISAGREEMENT;
        result->status = ADC_CAL_SKEW_STATUS_WARNING;
        result->failure_reason = "edge estimates disagree";
        return -11;
    }
    result->valid = 1;
    result->status = ADC_CAL_SKEW_STATUS_PASS;
    result->reason = ADC_CAL_SKEW_REASON_NONE;
    result->failure_reason = "none";
    return 0;
}
