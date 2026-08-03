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

const char *adc_cal_skew_polarity_name(adc_cal_skew_polarity_t polarity)
{
    switch (polarity) {
    case ADC_CAL_SKEW_POLARITY_SAME: return "SAME";
    case ADC_CAL_SKEW_POLARITY_INVERTED: return "INVERTED";
    case ADC_CAL_SKEW_POLARITY_UNKNOWN:
    default:
        return "UNKNOWN";
    }
}

const char *adc_cal_skew_branch_reason_name(
    adc_cal_skew_branch_reason_t reason)
{
    switch (reason) {
    case ADC_CAL_SKEW_BRANCH_REASON_KNOWN_POLARITY:
        return "known polarity metadata";
    case ADC_CAL_SKEW_BRANCH_REASON_DITHER_AGREEMENT:
        return "valid dither agreement";
    case ADC_CAL_SKEW_BRANCH_REASON_FRAME_CONSISTENCY:
        return "previous-frame consistency";
    case ADC_CAL_SKEW_BRANCH_REASON_PHYSICAL_BOUND:
        return "configured physical skew bound";
    case ADC_CAL_SKEW_BRANCH_REASON_MINIMUM_ABSOLUTE_SKEW:
        return "minimum absolute plausible skew";
    case ADC_CAL_SKEW_BRANCH_REASON_NONE:
    default:
        return "none";
    }
}

void adc_cal_skew_phase_default_config(adc_cal_skew_phase_config_t *config)
{
    if (config == NULL) return;
    config->max_abs_skew_samples = ADC_CAL_SKEW_MAX_LINEAR_SKEW_SAMPLES;
    config->known_polarity = ADC_CAL_SKEW_POLARITY_UNKNOWN;
    config->dither_valid = 0;
    config->dither_skew_samples = NAN;
    config->previous_valid = 0;
    config->previous_skew_samples = NAN;
}

static int phase_candidate_is_better(
    const adc_cal_skew_phase_result_t *result,
    size_t candidate,
    size_t best,
    int use_dither,
    double dither_skew,
    int use_previous,
    double previous_skew)
{
    double candidate_score;
    double best_score;
    double candidate_unwrapped;
    double best_unwrapped;

    if (use_dither) {
        candidate_score = fabs(
            result->candidate_skew_samples[candidate] - dither_skew);
        best_score = fabs(result->candidate_skew_samples[best] - dither_skew);
    }
    else if (use_previous) {
        candidate_score = fabs(
            result->candidate_skew_samples[candidate] - previous_skew);
        best_score = fabs(result->candidate_skew_samples[best] - previous_skew);
    }
    else {
        candidate_score = fabs(result->candidate_skew_samples[candidate]);
        best_score = fabs(result->candidate_skew_samples[best]);
    }
    if (candidate_score < best_score - 1.0e-12) return 1;
    if (candidate_score > best_score + 1.0e-12) return 0;
    /* +pi and -pi describe the same inverted phase family after wrapping.
     * Prefer the adjustment that reaches it without first crossing 2*pi so
     * the diagnostic correction sign remains intuitive. */
    candidate_unwrapped = fabs(result->raw_phase_difference_rad +
        result->candidate_phase_adjustment_rad[candidate]);
    best_unwrapped = fabs(result->raw_phase_difference_rad +
        result->candidate_phase_adjustment_rad[best]);
    return candidate_unwrapped < best_unwrapped;
}

int adc_cal_skew_resolve_tone_phase(
    double channel_a_phase_rad,
    double channel_b_phase_rad,
    double tone_frequency_hz,
    double sample_rate_hz,
    const adc_cal_skew_phase_config_t *config,
    adc_cal_skew_phase_result_t *result)
{
    const double two_pi = 6.28318530717958647692;
    const double pi = 3.14159265358979323846;
    const double adjustments[ADC_CAL_SKEW_PHASE_HYPOTHESES] =
        {0.0, pi, -pi};
    adc_cal_skew_phase_config_t local_config;
    double radians_per_sample;
    int have_in_range = 0;
    int use_dither;
    int use_previous;
    size_t best = SIZE_MAX;

    if (result == NULL) return -1;
    memset(result, 0, sizeof(*result));
    result->raw_phase_difference_rad = NAN;
    result->raw_skew_samples = NAN;
    result->corrected_phase_difference_rad = NAN;
    result->corrected_skew_samples = NAN;
    result->applied_phase_adjustment_rad = NAN;
    result->dither_disagreement_samples = NAN;
    if (!adc_cal_double_isfinite(channel_a_phase_rad) ||
        !adc_cal_double_isfinite(channel_b_phase_rad) ||
        !adc_cal_double_isfinite(tone_frequency_hz) ||
        !adc_cal_double_isfinite(sample_rate_hz) ||
        tone_frequency_hz <= 0.0 || sample_rate_hz <= 0.0 ||
        tone_frequency_hz >= 0.5 * sample_rate_hz) return -2;
    if (config == NULL) {
        adc_cal_skew_phase_default_config(&local_config);
        config = &local_config;
    }
    if (!adc_cal_double_isfinite(config->max_abs_skew_samples) ||
        config->max_abs_skew_samples <= 0.0 ||
        (config->known_polarity != ADC_CAL_SKEW_POLARITY_UNKNOWN &&
         config->known_polarity != ADC_CAL_SKEW_POLARITY_SAME &&
         config->known_polarity != ADC_CAL_SKEW_POLARITY_INVERTED)) return -3;

    radians_per_sample = two_pi * tone_frequency_hz / sample_rate_hz;
    result->raw_phase_difference_rad = remainder(
        channel_b_phase_rad - channel_a_phase_rad, two_pi);
    result->raw_skew_samples =
        result->raw_phase_difference_rad / radians_per_sample;
    for (size_t i = 0U; i < ADC_CAL_SKEW_PHASE_HYPOTHESES; ++i) {
        result->candidate_phase_adjustment_rad[i] = adjustments[i];
        result->candidate_phase_difference_rad[i] = remainder(
            result->raw_phase_difference_rad + adjustments[i], two_pi);
        result->candidate_skew_samples[i] =
            result->candidate_phase_difference_rad[i] / radians_per_sample;
        result->candidate_within_physical_range[i] =
            fabs(result->candidate_skew_samples[i]) <=
                config->max_abs_skew_samples ? 1 : 0;
        if (result->candidate_within_physical_range[i]) have_in_range = 1;
    }
    use_dither = config->dither_valid &&
        adc_cal_double_isfinite(config->dither_skew_samples);
    use_previous = config->previous_valid &&
        adc_cal_double_isfinite(config->previous_skew_samples);

    for (size_t i = 0U; i < ADC_CAL_SKEW_PHASE_HYPOTHESES; ++i) {
        const adc_cal_skew_polarity_t candidate_polarity = i == 0U ?
            ADC_CAL_SKEW_POLARITY_SAME : ADC_CAL_SKEW_POLARITY_INVERTED;
        if (config->known_polarity != ADC_CAL_SKEW_POLARITY_UNKNOWN &&
            candidate_polarity != config->known_polarity) continue;
        if (config->known_polarity == ADC_CAL_SKEW_POLARITY_UNKNOWN &&
            have_in_range && !result->candidate_within_physical_range[i]) {
            continue;
        }
        if (best == SIZE_MAX || phase_candidate_is_better(
                result, i, best, use_dither, config->dither_skew_samples,
                !use_dither && use_previous, config->previous_skew_samples)) {
            best = i;
        }
    }
    if (best == SIZE_MAX) return -4;
    result->selected_candidate = best;
    result->selected_polarity = best == 0U ?
        ADC_CAL_SKEW_POLARITY_SAME : ADC_CAL_SKEW_POLARITY_INVERTED;
    if (config->known_polarity != ADC_CAL_SKEW_POLARITY_UNKNOWN) {
        result->selection_reason = ADC_CAL_SKEW_BRANCH_REASON_KNOWN_POLARITY;
    }
    else if (use_dither) {
        result->selection_reason = ADC_CAL_SKEW_BRANCH_REASON_DITHER_AGREEMENT;
    }
    else if (use_previous) {
        result->selection_reason = ADC_CAL_SKEW_BRANCH_REASON_FRAME_CONSISTENCY;
    }
    else if (have_in_range) {
        result->selection_reason = ADC_CAL_SKEW_BRANCH_REASON_PHYSICAL_BOUND;
    }
    else {
        result->selection_reason =
            ADC_CAL_SKEW_BRANCH_REASON_MINIMUM_ABSOLUTE_SKEW;
    }
    result->applied_phase_adjustment_rad =
        result->candidate_phase_adjustment_rad[best];
    result->corrected_phase_difference_rad =
        result->candidate_phase_difference_rad[best];
    result->corrected_skew_samples = result->candidate_skew_samples[best];
    if (use_dither) {
        result->dither_disagreement_samples = fabs(
            result->corrected_skew_samples - config->dither_skew_samples);
    }
    result->valid = 1;
    return 0;
}

int adc_cal_skew_from_tone_phases(
    double channel_a_phase_rad,
    double channel_b_phase_rad,
    double tone_frequency_hz,
    double sample_rate_hz,
    double *relative_skew_samples)
{
    adc_cal_skew_phase_config_t config;
    adc_cal_skew_phase_result_t result;

    if (relative_skew_samples == NULL) return -1;
    adc_cal_skew_phase_default_config(&config);
    if (adc_cal_skew_resolve_tone_phase(
            channel_a_phase_rad, channel_b_phase_rad,
            tone_frequency_hz, sample_rate_hz, &config, &result) != 0 ||
        !result.valid) return -2;
    *relative_skew_samples = result.corrected_skew_samples;
    return 0;
}

const char *adc_cal_skew_measurement_validity_name(
    adc_cal_skew_measurement_validity_t status)
{
    return status == ADC_CAL_SKEW_MEASUREMENT_VALID ? "VALID" : "INVALID";
}

const char *adc_cal_skew_stability_name(adc_cal_skew_stability_t status)
{
    switch (status) {
    case ADC_CAL_SKEW_STABILITY_STABLE: return "STABLE";
    case ADC_CAL_SKEW_STABILITY_UNSTABLE: return "UNSTABLE";
    case ADC_CAL_SKEW_STABILITY_UNKNOWN:
    default:
        return "UNKNOWN";
    }
}

const char *adc_cal_skew_tolerance_status_name(
    adc_cal_skew_tolerance_status_t status)
{
    switch (status) {
    case ADC_CAL_SKEW_TOLERANCE_IN: return "YES";
    case ADC_CAL_SKEW_TOLERANCE_OUT: return "NO";
    case ADC_CAL_SKEW_TOLERANCE_UNKNOWN:
    default:
        return "UNKNOWN";
    }
}

const char *adc_cal_skew_actuator_status_name(
    adc_cal_skew_actuator_status_t status)
{
    return status == ADC_CAL_SKEW_ACTUATOR_AVAILABLE ?
        "AVAILABLE" : "UNAVAILABLE";
}

const char *adc_cal_skew_correction_status_name(
    adc_cal_skew_correction_status_t status)
{
    switch (status) {
    case ADC_CAL_SKEW_CORRECTION_CONVERGED: return "CONVERGED";
    case ADC_CAL_SKEW_CORRECTION_NOT_CONVERGED: return "NOT CONVERGED";
    case ADC_CAL_SKEW_CORRECTION_SATURATED: return "SATURATED";
    case ADC_CAL_SKEW_CORRECTION_NOT_APPLICABLE:
    default:
        return "NOT APPLICABLE";
    }
}

const char *adc_cal_skew_stage_result_name(
    adc_cal_skew_stage_result_t status)
{
    switch (status) {
    case ADC_CAL_SKEW_STAGE_RESULT_UNSTABLE: return "FAIL - UNSTABLE";
    case ADC_CAL_SKEW_STAGE_RESULT_PASS: return "PASS";
    case ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING:
        return "PASS WITH WARNING";
    case ADC_CAL_SKEW_STAGE_RESULT_CORRECTION_NOT_CONVERGED:
        return "CORRECTION NOT CONVERGED";
    case ADC_CAL_SKEW_STAGE_RESULT_CORRECTION_SATURATED:
        return "CORRECTION SATURATED";
    case ADC_CAL_SKEW_STAGE_RESULT_INVALID:
    default:
        return "FAIL - INVALID";
    }
}

int adc_cal_skew_evaluate_stage_policy(
    const adc_cal_skew_stage_policy_input_t *input,
    adc_cal_skew_stage_policy_result_t *result)
{
    if (result == NULL) return -1;
    memset(result, 0, sizeof(*result));
    result->measurement_validity = ADC_CAL_SKEW_MEASUREMENT_INVALID;
    result->stability = ADC_CAL_SKEW_STABILITY_UNKNOWN;
    result->tolerance_status = ADC_CAL_SKEW_TOLERANCE_UNKNOWN;
    result->actuator_status = ADC_CAL_SKEW_ACTUATOR_UNAVAILABLE;
    result->correction_status = ADC_CAL_SKEW_CORRECTION_NOT_APPLICABLE;
    result->stage_result = ADC_CAL_SKEW_STAGE_RESULT_INVALID;
    result->reason = "skew stage policy input is invalid";
    if (input == NULL) return -2;
    result->actuator_status = input->actuator_available ?
        ADC_CAL_SKEW_ACTUATOR_AVAILABLE :
        ADC_CAL_SKEW_ACTUATOR_UNAVAILABLE;
    if (!adc_cal_double_isfinite(input->maximum_batch_std_samples) ||
        input->maximum_batch_std_samples < 0.0 ||
        !adc_cal_double_isfinite(input->tolerance_samples) ||
        input->tolerance_samples < 0.0 ||
        input->minimum_accepted_frames == 0U) return -3;
    if (!input->primary_estimate_valid ||
        !adc_cal_double_isfinite(input->measured_skew_samples) ||
        input->accepted_frames == 0U) {
        result->reason = "no valid primary skew estimate";
        return 0;
    }
    if (input->accepted_frames < input->minimum_accepted_frames) {
        result->reason = "too few accepted skew frames";
        return 0;
    }
    result->measurement_validity = ADC_CAL_SKEW_MEASUREMENT_VALID;
    if (input->polarity_branch_changes > 0U) {
        result->stability = ADC_CAL_SKEW_STABILITY_UNSTABLE;
        result->stage_result = ADC_CAL_SKEW_STAGE_RESULT_UNSTABLE;
        result->reason = "polarity branch changed between accepted frames";
        return 0;
    }
    if (!adc_cal_double_isfinite(input->batch_std_samples) ||
        input->batch_std_samples > input->maximum_batch_std_samples) {
        result->stability = ADC_CAL_SKEW_STABILITY_UNSTABLE;
        result->stage_result = ADC_CAL_SKEW_STAGE_RESULT_UNSTABLE;
        result->reason = "open-loop skew estimate is unstable";
        return 0;
    }
    result->stability = ADC_CAL_SKEW_STABILITY_STABLE;
    result->tolerance_status =
        fabs(input->measured_skew_samples) <= input->tolerance_samples ?
            ADC_CAL_SKEW_TOLERANCE_IN : ADC_CAL_SKEW_TOLERANCE_OUT;
    if (!input->actuator_available) {
        result->correction_status = ADC_CAL_SKEW_CORRECTION_NOT_APPLICABLE;
        result->stage_result =
            result->tolerance_status == ADC_CAL_SKEW_TOLERANCE_IN &&
            !input->advisory_warning ?
                ADC_CAL_SKEW_STAGE_RESULT_PASS :
                ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING;
        result->pipeline_may_continue = 1;
        result->output_usable = 1;
        result->reason = result->tolerance_status == ADC_CAL_SKEW_TOLERANCE_OUT ?
            "stable measured skew exceeds tolerance; no actuator available" :
            input->advisory_warning ?
                "valid stable measurement with optional cross-check warning" :
                "valid stable open-loop skew measurement";
        return 0;
    }
    if (input->actuator_saturated) {
        result->correction_status = ADC_CAL_SKEW_CORRECTION_SATURATED;
        result->stage_result =
            ADC_CAL_SKEW_STAGE_RESULT_CORRECTION_SATURATED;
        result->reason = "skew correction actuator saturated";
        return 0;
    }
    if (input->correction_applied && input->correction_converged &&
        result->tolerance_status == ADC_CAL_SKEW_TOLERANCE_IN) {
        result->correction_status = ADC_CAL_SKEW_CORRECTION_CONVERGED;
        result->stage_result = ADC_CAL_SKEW_STAGE_RESULT_PASS;
        result->pipeline_may_continue = 1;
        result->output_usable = 1;
        result->reason = "skew correction converged";
        return 0;
    }
    result->correction_status = ADC_CAL_SKEW_CORRECTION_NOT_CONVERGED;
    result->stage_result = ADC_CAL_SKEW_STAGE_RESULT_CORRECTION_NOT_CONVERGED;
    result->reason = "skew correction did not converge";
    return 0;
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
    double relative_template_energy = 0.0;
    double relative_projection = 0.0;

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
    /* The DAC pulse is an event locator, not an analog shape oracle.  The
     * ADC/DAC chain can distort its edges enough that subtracting two
     * independent channel-vs-DAC skew estimates leaves a large common shape
     * bias.  For relative B-A skew, use Channel A's captured event profile as
     * the local template and fit Channel B directly against it. */
    for (size_t j = 0U; j < profile_count; ++j) {
        relative_template_energy += profile_a[j] * profile_a[j];
        relative_projection += profile_b[j] * profile_a[j];
        template_profile[j] = profile_a[j];
    }
    if (relative_template_energy <= ADC_CAL_SKEW_DERIVATIVE_ENERGY_FLOOR) {
        result->reason = ADC_CAL_SKEW_REASON_TEMPLATE;
        result->failure_reason = "Channel A event profile has insufficient energy";
        return -7;
    }
    for (size_t j = 0U; j < profile_count; ++j) {
        const double previous = j > 0U ? template_profile[j - 1U] :
            template_profile[j];
        const double next = j + 1U < profile_count ?
            template_profile[j + 1U] : template_profile[j];
        derivative_profile[j] = 0.5 * (next - previous);
    }
    gain_a = 1.0;
    if (adc_cal_dither_analyze(channel_b_residual, dither_template,
                               sample_count, &dither_config, &dither) != 0) {
        result->reason = ADC_CAL_SKEW_REASON_DITHER;
        result->failure_reason = "channel B dither analysis failed";
        return -7;
    }
    gain_b = relative_projection / relative_template_energy;
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
        result->valid = 1;
        result->status = ADC_CAL_SKEW_STATUS_WARNING;
        result->failure_reason = "skew outside supported linear range";
        return 0;
    }
    if (result->edge_disagreement_samples >
        config->max_edge_disagreement_samples) {
        result->reason = ADC_CAL_SKEW_REASON_EDGE_DISAGREEMENT;
        result->valid = 1;
        result->status = ADC_CAL_SKEW_STATUS_WARNING;
        result->failure_reason = "edge estimates disagree";
        return 0;
    }
    result->valid = 1;
    result->status = ADC_CAL_SKEW_STATUS_PASS;
    result->reason = ADC_CAL_SKEW_REASON_NONE;
    result->failure_reason = "none";
    return 0;
}
