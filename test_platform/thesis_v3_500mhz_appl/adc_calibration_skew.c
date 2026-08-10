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

const char *adc_cal_skew_tone_context_status_name(
    adc_cal_skew_tone_context_status_t status)
{
    switch (status) {
    case ADC_CAL_SKEW_TONE_CONTEXT_VALID: return "VALID";
    case ADC_CAL_SKEW_TONE_CONTEXT_INVALID_SAMPLE_RATE:
        return "INVALID_SAMPLE_RATE";
    case ADC_CAL_SKEW_TONE_CONTEXT_INVALID_TONE_FREQUENCY:
        return "INVALID_TONE_FREQUENCY";
    case ADC_CAL_SKEW_TONE_CONTEXT_FREQUENCY_MISMATCH:
    default: return "FREQUENCY_MISMATCH";
    }
}

adc_cal_skew_tone_context_status_t adc_cal_skew_validate_tone_context(
    double inherited_tone_frequency_hz,
    double fitted_tone_frequency_hz,
    double sample_rate_hz,
    size_t sample_count,
    double maximum_error_bins)
{
    double bin_width_hz;
    if (!adc_cal_double_isfinite(sample_rate_hz) || sample_rate_hz <= 0.0 ||
        sample_count == 0U) {
        return ADC_CAL_SKEW_TONE_CONTEXT_INVALID_SAMPLE_RATE;
    }
    if (!adc_cal_double_isfinite(inherited_tone_frequency_hz) ||
        !adc_cal_double_isfinite(fitted_tone_frequency_hz) ||
        inherited_tone_frequency_hz <= 0.0 || fitted_tone_frequency_hz <= 0.0 ||
        inherited_tone_frequency_hz >= 0.5 * sample_rate_hz ||
        fitted_tone_frequency_hz >= 0.5 * sample_rate_hz ||
        !adc_cal_double_isfinite(maximum_error_bins) ||
        maximum_error_bins < 0.0) {
        return ADC_CAL_SKEW_TONE_CONTEXT_INVALID_TONE_FREQUENCY;
    }
    bin_width_hz = sample_rate_hz / (double)sample_count;
    if (fabs(fitted_tone_frequency_hz - inherited_tone_frequency_hz) >
        maximum_error_bins * bin_width_hz) {
        return ADC_CAL_SKEW_TONE_CONTEXT_FREQUENCY_MISMATCH;
    }
    return ADC_CAL_SKEW_TONE_CONTEXT_VALID;
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

int adc_cal_skew_dither_crosscheck_is_usable(
    const adc_cal_skew_result_t *dither_result,
    double primary_skew_samples,
    double maximum_disagreement_samples,
    double *disagreement_samples)
{
    double disagreement;
    if (disagreement_samples == NULL) return -1;
    *disagreement_samples = NAN;
    if (dither_result == NULL ||
        !adc_cal_double_isfinite(primary_skew_samples) ||
        !adc_cal_double_isfinite(maximum_disagreement_samples) ||
        maximum_disagreement_samples < 0.0) return -2;
    if (!dither_result->valid ||
        !adc_cal_double_isfinite(dither_result->relative_skew_samples)) {
        return 0;
    }
    disagreement = fabs(
        primary_skew_samples - dither_result->relative_skew_samples);
    *disagreement_samples = disagreement;
    if (dither_result->status != ADC_CAL_SKEW_STATUS_PASS) return 0;
    return disagreement <= maximum_disagreement_samples ? 1 : 0;
}

int adc_cal_skew_map_paired_window_i16(
    const int16_t *channel_a,
    const int16_t *channel_b,
    size_t frame_sample_count,
    size_t window_start,
    size_t window_length,
    double common_phase_offset_samples,
    double common_lag_samples,
    double *mapped_a,
    double *mapped_b)
{
    if (channel_a == NULL || channel_b == NULL || mapped_a == NULL ||
        mapped_b == NULL || frame_sample_count < 2U || window_length == 0U ||
        window_length > frame_sample_count || window_start >= frame_sample_count ||
        !adc_cal_double_isfinite(common_phase_offset_samples) ||
        !adc_cal_double_isfinite(common_lag_samples)) return -1;
    for (size_t i = 0U; i < window_length; ++i) {
        double position = fmod((double)(window_start + i) +
            common_phase_offset_samples + common_lag_samples,
            (double)frame_sample_count);
        size_t lower;
        size_t upper;
        double fraction;
        if (position < 0.0) position += (double)frame_sample_count;
        lower = (size_t)floor(position);
        upper = lower + 1U;
        if (upper >= frame_sample_count) upper = 0U;
        fraction = position - (double)lower;
        mapped_a[i] = (1.0 - fraction) * (double)channel_a[lower] +
            fraction * (double)channel_a[upper];
        mapped_b[i] = (1.0 - fraction) * (double)channel_b[lower] +
            fraction * (double)channel_b[upper];
        if (!adc_cal_double_isfinite(mapped_a[i]) ||
            !adc_cal_double_isfinite(mapped_b[i])) return -2;
    }
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
        if (!input->measurement_required) {
            result->stage_result = ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING;
            result->pipeline_may_continue = 1;
            result->output_usable = 1;
            result->reason =
                "optional open-loop skew measurement unavailable";
        } else {
            result->reason =
                "mandatory skew measurement has no valid primary estimate";
        }
        return 0;
    }
    if (input->accepted_frames < input->minimum_accepted_frames) {
        if (!input->measurement_required) {
            result->stage_result = ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING;
            result->pipeline_may_continue = 1;
            result->output_usable = 1;
            result->reason =
                "optional open-loop skew has too few accepted frames";
        } else {
            result->reason =
                "mandatory skew measurement has too few accepted frames";
        }
        return 0;
    }
    result->measurement_validity = ADC_CAL_SKEW_MEASUREMENT_VALID;
    if (input->polarity_branch_changes > 0U) {
        result->stability = ADC_CAL_SKEW_STABILITY_UNSTABLE;
        result->stage_result = input->measurement_required ?
            ADC_CAL_SKEW_STAGE_RESULT_UNSTABLE :
            ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING;
        result->pipeline_may_continue = input->measurement_required ? 0 : 1;
        result->output_usable = input->measurement_required ? 0 : 1;
        result->reason = "polarity branch changed between accepted frames";
        return 0;
    }
    if (!adc_cal_double_isfinite(input->batch_std_samples) ||
        input->batch_std_samples > input->maximum_batch_std_samples) {
        result->stability = ADC_CAL_SKEW_STABILITY_UNSTABLE;
        result->stage_result = input->measurement_required ?
            ADC_CAL_SKEW_STAGE_RESULT_UNSTABLE :
            ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING;
        result->pipeline_may_continue = input->measurement_required ? 0 : 1;
        result->output_usable = input->measurement_required ? 0 : 1;
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

void adc_cal_skew_loop_default_config(adc_cal_skew_loop_config_t *config)
{
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));
    config->skew_closed_loop_enable = 0;
    config->skew_tolerance_samples = 0.01;
    config->skew_required_consecutive_passes = 2U;
    config->skew_max_iterations = 10U;
    config->skew_controller_gain = 0.5;
    config->skew_max_steps_per_iteration = 4;
    config->skew_register_min = 0;
    config->skew_register_max = 255;
    config->skew_actuator_step_samples = 0.0;
    config->skew_actuator_polarity = 0;
    config->skew_minimum_accepted_frames = 3U;
    config->skew_maximum_batch_std_samples = 0.02;
    config->skew_characterization_step_tolerance_fraction = 0.35;
}

const char *adc_cal_skew_loop_status_name(adc_cal_skew_loop_status_t status)
{
    switch (status) {
    case ADC_CAL_SKEW_LOOP_MEASUREMENT_ONLY: return "MEASUREMENT ONLY";
    case ADC_CAL_SKEW_LOOP_CONVERGED: return "CONVERGED";
    case ADC_CAL_SKEW_LOOP_NOT_CONVERGED: return "NOT CONVERGED";
    case ADC_CAL_SKEW_LOOP_SATURATED: return "SATURATED";
    case ADC_CAL_SKEW_LOOP_ACTUATOR_UNAVAILABLE:
        return "ACTUATOR UNAVAILABLE";
    case ADC_CAL_SKEW_LOOP_FAILED:
    default: return "FAILED";
    }
}

static int adc_cal_skew_loop_config_valid(
    const adc_cal_skew_loop_config_t *config)
{
    return config != NULL &&
        adc_cal_double_isfinite(config->skew_tolerance_samples) &&
        config->skew_tolerance_samples >= 0.0 &&
        config->skew_required_consecutive_passes > 0U &&
        config->skew_max_iterations > 0U &&
        adc_cal_double_isfinite(config->skew_controller_gain) &&
        config->skew_controller_gain > 0.0 &&
        config->skew_controller_gain <= 1.0 &&
        config->skew_max_steps_per_iteration > 0 &&
        config->skew_register_min < config->skew_register_max &&
        config->skew_minimum_accepted_frames > 0U &&
        adc_cal_double_isfinite(config->skew_maximum_batch_std_samples) &&
        config->skew_maximum_batch_std_samples >= 0.0 &&
        adc_cal_double_isfinite(
            config->skew_characterization_step_tolerance_fraction) &&
        config->skew_characterization_step_tolerance_fraction >= 0.0;
}

static int adc_cal_skew_measurement_valid(
    const adc_cal_skew_loop_config_t *config,
    const adc_cal_skew_batch_measurement_t *measurement)
{
    return measurement != NULL && measurement->valid &&
        adc_cal_double_isfinite(measurement->skew_samples) &&
        adc_cal_double_isfinite(measurement->batch_std_samples) &&
        measurement->accepted_frames >=
            config->skew_minimum_accepted_frames &&
        measurement->batch_std_samples <=
            config->skew_maximum_batch_std_samples;
}

int adc_cal_skew_plan_update(
    double measured_skew_samples,
    int current_register,
    const adc_cal_skew_loop_config_t *config,
    int *requested_steps,
    int *applied_steps,
    int *new_register,
    int *saturated)
{
    double signed_step;
    double requested;
    int request;
    int applied;
    int64_t target;
    if (requested_steps == NULL || applied_steps == NULL ||
        new_register == NULL || saturated == NULL ||
        !adc_cal_skew_loop_config_valid(config) ||
        !adc_cal_double_isfinite(measured_skew_samples) ||
        !adc_cal_double_isfinite(config->skew_actuator_step_samples) ||
        config->skew_actuator_step_samples <= 0.0 ||
        (config->skew_actuator_polarity != 1 &&
         config->skew_actuator_polarity != -1) ||
        current_register < config->skew_register_min ||
        current_register > config->skew_register_max) return -1;
    signed_step = config->skew_actuator_step_samples *
        (double)config->skew_actuator_polarity;
    requested = -config->skew_controller_gain * measured_skew_samples /
        signed_step;
    if (!adc_cal_double_isfinite(requested) ||
        requested > (double)INT32_MAX || requested < (double)INT32_MIN)
        return -2;
    request = (int)lround(requested);
    /* A fractional-gain controller can round to zero one actuator step before
     * convergence.  Permit a single final step only when the characterized
     * linear model predicts that it will land inside the requested tolerance.
     * This avoids both a false resolution failure and a two-code oscillation
     * when the actuator is genuinely too coarse. */
    if (request == 0 &&
        fabs(measured_skew_samples) > config->skew_tolerance_samples) {
        const int finish_direction = requested > 0.0 ? 1 : -1;
        const double projected_skew = measured_skew_samples +
            (double)finish_direction * signed_step;
        if (fabs(projected_skew) <= config->skew_tolerance_samples)
            request = finish_direction;
    }
    applied = request;
    if (applied > config->skew_max_steps_per_iteration)
        applied = config->skew_max_steps_per_iteration;
    if (applied < -config->skew_max_steps_per_iteration)
        applied = -config->skew_max_steps_per_iteration;
    target = (int64_t)current_register + (int64_t)applied;
    *saturated = 0;
    if (target > config->skew_register_max) {
        target = config->skew_register_max;
        *saturated = 1;
    } else if (target < config->skew_register_min) {
        target = config->skew_register_min;
        *saturated = 1;
    }
    applied = (int)target - current_register;
    *requested_steps = request;
    *applied_steps = applied;
    *new_register = (int)target;
    return 0;
}

static int adc_cal_skew_verified_write(
    const adc_cal_skew_loop_config_t *config,
    const adc_cal_skew_loop_io_t *io,
    int expected_current,
    int new_value)
{
    int current = 0;
    int readback = 0;
    if (new_value < config->skew_register_min ||
        new_value > config->skew_register_max ||
        io->read_register(io->context, &current) != 0 ||
        current != expected_current ||
        io->write_register(io->context, new_value) != 0 ||
        io->read_register(io->context, &readback) != 0 ||
        readback != new_value) return -1;
    return 0;
}

int adc_cal_skew_run_closed_loop(
    const adc_cal_skew_loop_config_t *config,
    const adc_cal_skew_loop_io_t *io,
    double sample_rate_hz,
    adc_cal_skew_loop_result_t *result)
{
    adc_cal_skew_loop_config_t active;
    adc_cal_skew_batch_measurement_t measurement;
    adc_cal_skew_batch_measurement_t stepped;
    adc_cal_skew_batch_measurement_t stepped_repeat;
    int current_register = 0;
    int characterization_register;
    int characterization_direction = 1;
    double observed;
    double observed_repeat;
    if (result == NULL) return -1;
    memset(result, 0, sizeof(*result));
    result->status = ADC_CAL_SKEW_LOOP_FAILED;
    result->initial_skew_samples = NAN;
    result->final_skew_samples = NAN;
    result->best_skew_samples = NAN;
    result->final_batch_std_samples = NAN;
    result->observed_step_samples = NAN;
    result->observed_step_ps = NAN;
    result->initial_register = -1;
    result->final_register = -1;
    result->failure_reason = "closed-loop configuration is invalid";
    if (!adc_cal_skew_loop_config_valid(config) || io == NULL ||
        io->measure_batch == NULL || !adc_cal_double_isfinite(sample_rate_hz) ||
        sample_rate_hz <= 0.0) return -2;
    memset(&measurement, 0, sizeof(measurement));
    if (io->measure_batch(io->context, &measurement) != 0 ||
        !adc_cal_skew_measurement_valid(config, &measurement)) {
        result->failure_reason = measurement.reason != NULL ?
            measurement.reason : "invalid primary skew estimate";
        return -3;
    }
    result->initial_skew_samples = measurement.skew_samples;
    result->final_skew_samples = measurement.skew_samples;
    result->best_skew_samples = measurement.skew_samples;
    result->final_batch_std_samples = measurement.batch_std_samples;
    result->accepted_frames += measurement.accepted_frames;
    result->rejected_frames += measurement.rejected_frames;
    if (!config->skew_closed_loop_enable) {
        result->status = ADC_CAL_SKEW_LOOP_MEASUREMENT_ONLY;
        result->failure_reason = "closed-loop skew correction is disabled";
        return 0;
    }
    if (io->read_register == NULL || io->write_register == NULL) {
        result->status = ADC_CAL_SKEW_LOOP_ACTUATOR_UNAVAILABLE;
        result->failure_reason = "ACTUATOR_UNAVAILABLE";
        return 0;
    }
    /* Measurement validity is deliberately established before actuator
     * preparation.  Board backends may write a neutral/default code while
     * preparing, so an invalid estimator must never reach this callback. */
    if (io->prepare_actuator != NULL &&
        io->prepare_actuator(io->context) != 0) {
        result->status = ADC_CAL_SKEW_LOOP_ACTUATOR_UNAVAILABLE;
        result->failure_reason = "ACTUATOR_UNAVAILABLE";
        return 0;
    }
    if (io->prepare_actuator != NULL) {
        adc_cal_skew_batch_measurement_t prepared_baseline;
        memset(&prepared_baseline, 0, sizeof(prepared_baseline));
        if (io->measure_batch(io->context, &prepared_baseline) != 0 ||
            !adc_cal_skew_measurement_valid(config, &prepared_baseline)) {
            result->failure_reason = prepared_baseline.reason != NULL ?
                prepared_baseline.reason :
                "post-preparation skew baseline is invalid";
            return -4;
        }
        /* Never characterize an actuator step against a measurement taken
         * before the backend was prepared. Preparation can change clock-mode
         * state even when it selects the same nominal register code. */
        measurement = prepared_baseline;
        result->initial_skew_samples = measurement.skew_samples;
        result->final_skew_samples = measurement.skew_samples;
        result->best_skew_samples = measurement.skew_samples;
        result->final_batch_std_samples = measurement.batch_std_samples;
        result->accepted_frames += measurement.accepted_frames;
        result->rejected_frames += measurement.rejected_frames;
    }
    if (io->read_register(io->context, &current_register) != 0 ||
        current_register < config->skew_register_min ||
        current_register > config->skew_register_max) {
        result->failure_reason = "actuator register read failed or is out of range";
        return -5;
    }
    result->initial_register = current_register;
    result->final_register = current_register;
    if (current_register == config->skew_register_max)
        characterization_direction = -1;
    characterization_register = current_register + characterization_direction;
    if (adc_cal_skew_verified_write(
            config, io, current_register, characterization_register) != 0) {
        result->failure_reason = "actuator characterization write/readback failed";
        return -6;
    }
    memset(&stepped, 0, sizeof(stepped));
    if (io->measure_batch(io->context, &stepped) != 0 ||
        !adc_cal_skew_measurement_valid(config, &stepped)) {
        (void)adc_cal_skew_verified_write(
            config, io, characterization_register, current_register);
        result->failure_reason = stepped.reason != NULL ? stepped.reason :
            "actuator characterization measurement is invalid";
        return -7;
    }
    result->accepted_frames += stepped.accepted_frames;
    result->rejected_frames += stepped.rejected_frames;
    observed = (stepped.skew_samples - measurement.skew_samples) /
        (double)characterization_direction;
    if (adc_cal_skew_verified_write(
            config, io, characterization_register, current_register) != 0) {
        result->failure_reason = "actuator restoration write/readback failed";
        return -8;
    }
    if (adc_cal_skew_verified_write(
            config, io, current_register, characterization_register) != 0) {
        result->failure_reason = "repeat characterization write/readback failed";
        return -8;
    }
    memset(&stepped_repeat, 0, sizeof(stepped_repeat));
    if (io->measure_batch(io->context, &stepped_repeat) != 0 ||
        !adc_cal_skew_measurement_valid(config, &stepped_repeat)) {
        (void)adc_cal_skew_verified_write(
            config, io, characterization_register, current_register);
        result->failure_reason = stepped_repeat.reason != NULL ?
            stepped_repeat.reason :
            "repeat actuator characterization measurement is invalid";
        return -8;
    }
    result->accepted_frames += stepped_repeat.accepted_frames;
    result->rejected_frames += stepped_repeat.rejected_frames;
    observed_repeat = (stepped_repeat.skew_samples - measurement.skew_samples) /
        (double)characterization_direction;
    if (adc_cal_skew_verified_write(
            config, io, characterization_register, current_register) != 0) {
        result->failure_reason = "repeat characterization restoration failed";
        return -8;
    }
    if (!adc_cal_double_isfinite(observed) || fabs(observed) <= 1.0e-9) {
        result->failure_reason = "actuator characterization produced no measurable step";
        return -9;
    }
    if (!adc_cal_double_isfinite(observed_repeat) ||
        fabs(observed_repeat) <= 1.0e-9 ||
        observed * observed_repeat <= 0.0 ||
        fabs(observed_repeat - observed) / fabs(observed) >
            config->skew_characterization_step_tolerance_fraction) {
        result->failure_reason = "actuator step response is not repeatable";
        return -9;
    }
    observed = 0.5 * (observed + observed_repeat);
    result->observed_step_samples = observed;
    result->observed_step_ps = observed * 1.0e12 / sample_rate_hz;
    result->actuator_polarity = observed > 0.0 ? 1 : -1;
    result->actuator_step_samples = fabs(observed);
    if (config->skew_actuator_polarity != 0 &&
        config->skew_actuator_polarity != result->actuator_polarity) {
        result->failure_reason = "measured actuator polarity disagrees with configuration";
        return -10;
    }
    if (config->skew_actuator_step_samples > 0.0) {
        const double relative_error = fabs(
            fabs(observed) - config->skew_actuator_step_samples) /
            config->skew_actuator_step_samples;
        if (relative_error >
            config->skew_characterization_step_tolerance_fraction) {
            result->failure_reason =
                "measured actuator step disagrees with configuration";
            return -11;
        }
    }
    result->characterization_valid = 1;
    active = *config;
    active.skew_actuator_polarity = result->actuator_polarity;
    active.skew_actuator_step_samples = result->actuator_step_samples;

    for (uint32_t iteration = 1U;
         iteration <= active.skew_max_iterations; ++iteration) {
        int requested_steps = 0;
        int applied_steps = 0;
        int new_register = current_register;
        int saturated = 0;
        result->iterations_completed = iteration;
        result->final_skew_samples = measurement.skew_samples;
        result->final_batch_std_samples = measurement.batch_std_samples;
        if (fabs(measurement.skew_samples) <
            fabs(result->best_skew_samples))
            result->best_skew_samples = measurement.skew_samples;
        if (fabs(measurement.skew_samples) <=
            active.skew_tolerance_samples) {
            ++result->consecutive_passes;
            /* An iteration row records a controller decision, not a DMA
             * request.  When this decision establishes convergence, the
             * loop returns without collecting a post-decision batch.  It is
             * therefore valid for the final iteration row to have no
             * same-numbered capture group. */
            if (io->report_iteration != NULL)
                io->report_iteration(io->context, iteration, &measurement,
                    current_register, current_register, 0, 0,
                    active.skew_actuator_step_samples,
                    result->best_skew_samples, 0,
                    result->consecutive_passes,
                    result->consecutive_passes >=
                        active.skew_required_consecutive_passes);
            if (result->consecutive_passes >=
                active.skew_required_consecutive_passes) {
                result->status = ADC_CAL_SKEW_LOOP_CONVERGED;
                result->final_register = current_register;
                result->total_register_change = current_register -
                    result->initial_register;
                result->failure_reason = "none";
                return 0;
            }
        } else {
            result->consecutive_passes = 0U;
            if (adc_cal_skew_plan_update(
                    measurement.skew_samples, current_register, &active,
                    &requested_steps, &applied_steps, &new_register,
                    &saturated) != 0) {
                result->failure_reason = "unsafe or nonfinite controller update";
                return -12;
            }
            if (applied_steps == 0) {
                if (io->report_iteration != NULL)
                    io->report_iteration(io->context, iteration, &measurement,
                        current_register, current_register,
                        requested_steps, 0,
                        active.skew_actuator_step_samples,
                        result->best_skew_samples,
                        saturated, 0U, 0);
                result->status = saturated ? ADC_CAL_SKEW_LOOP_SATURATED :
                    ADC_CAL_SKEW_LOOP_NOT_CONVERGED;
                result->saturated = saturated;
                result->failure_reason = saturated ?
                    "required correction exceeds actuator range" :
                    "actuator resolution cannot reduce residual skew";
                return 0;
            }
            if (adc_cal_skew_verified_write(
                    &active, io, current_register, new_register) != 0) {
                result->failure_reason = "actuator update write/readback failed";
                return -13;
            }
            current_register = new_register;
            if (io->report_iteration != NULL)
                io->report_iteration(io->context, iteration, &measurement,
                    current_register - applied_steps, current_register,
                    requested_steps, applied_steps,
                    active.skew_actuator_step_samples,
                    result->best_skew_samples, saturated, 0U, 0);
            result->final_register = current_register;
            result->correction_applied = 1;
            result->saturated |= saturated;
        }
        if (iteration == active.skew_max_iterations) break;
        memset(&measurement, 0, sizeof(measurement));
        if (io->measure_batch(io->context, &measurement) != 0 ||
            !adc_cal_skew_measurement_valid(&active, &measurement)) {
            result->failure_reason = measurement.reason != NULL ?
                measurement.reason : "post-update skew estimate is invalid";
            return -14;
        }
        result->accepted_frames += measurement.accepted_frames;
        result->rejected_frames += measurement.rejected_frames;
    }
    result->final_register = current_register;
    result->total_register_change = current_register - result->initial_register;
    result->status = result->saturated ? ADC_CAL_SKEW_LOOP_SATURATED :
        ADC_CAL_SKEW_LOOP_NOT_CONVERGED;
    result->failure_reason = result->saturated ?
        "actuator saturated before convergence" :
        "closed-loop iteration limit reached";
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
