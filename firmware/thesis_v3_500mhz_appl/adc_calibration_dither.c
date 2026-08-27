#include "adc_calibration_dither.h"

#include <float.h>
#include <math.h>
#include <string.h>

static int adc_cal_double_isfinite(double value)
{
    return value == value && value <= DBL_MAX && value >= -DBL_MAX;
}

static double dither_abs(double value)
{
    return value < 0.0 ? -value : value;
}

static size_t dither_circular_distance(
    size_t first,
    size_t second,
    size_t count)
{
    size_t distance = first > second ? first - second : second - first;
    if (distance > count - distance) distance = count - distance;
    return distance;
}

static double dither_wrap_position(double position, double period)
{
    double wrapped;
    if (!adc_cal_double_isfinite(position) ||
        !adc_cal_double_isfinite(period) || period <= DBL_EPSILON) {
        return NAN;
    }
    wrapped = fmod(position, period);
    if (wrapped < 0.0) wrapped += period;
    return wrapped;
}

static int dither_periodic_equivalent(
    size_t candidate_index,
    size_t reference_index,
    size_t score_count,
    double event_spacing_samples,
    double exclusion_width_samples)
{
    const double raw_delta =
        (double)candidate_index - (double)reference_index;
    const int first_frame_offset = raw_delta > 0.0 ? -1 : 0;
    const int last_frame_offset = raw_delta < 0.0 ? 1 : 0;

    if (!adc_cal_double_isfinite(event_spacing_samples) ||
        event_spacing_samples <= 0.0 ||
        event_spacing_samples >= (double)score_count) {
        return 0;
    }
    for (int frame_offset = first_frame_offset;
         frame_offset <= last_frame_offset;
         ++frame_offset) {
        const double lifted_delta = raw_delta +
            (double)frame_offset * (double)score_count;
        const double multiple = floor(
            dither_abs(lifted_delta) / event_spacing_samples + 0.5);
        if (multiple >= 1.0 &&
            dither_abs(dither_abs(lifted_delta) -
                       multiple * event_spacing_samples) <=
                exclusion_width_samples) {
            return 1;
        }
    }
    return 0;
}

static adc_cal_dither_confidence_t dither_classify_peak_ratio(
    double peak_ratio,
    const adc_cal_dither_validation_config_t *config)
{
    if (!adc_cal_double_isfinite(peak_ratio) || config == NULL) {
        return ADC_CAL_DITHER_CONFIDENCE_INVALID;
    }
    return peak_ratio >= config->strong_peak_ratio ?
        ADC_CAL_DITHER_CONFIDENCE_STRONG :
        ADC_CAL_DITHER_CONFIDENCE_WEAK;
}

void adc_cal_dither_default_config(adc_cal_dither_config_t *config)
{
    if (config == NULL) return;
    config->threshold_fraction = ADC_CAL_DITHER_DEFAULT_THRESHOLD_FRACTION;
    config->minimum_events = ADC_CAL_DITHER_DEFAULT_MIN_EVENTS;
    config->boundary_margin = 1U;
}

void adc_cal_dither_peak_default_config(adc_cal_dither_peak_config_t *config)
{
    if (config == NULL) return;
    config->local_exclusion_samples =
        ADC_CAL_DITHER_DEFAULT_PEAK_GUARD_SAMPLES;
    config->periodic_exclusion_width_samples =
        ADC_CAL_DITHER_DEFAULT_PERIODIC_EXCLUSION_WIDTH;
    config->maximum_candidates = ADC_CAL_DITHER_DEFAULT_CANDIDATE_COUNT;
}

void adc_cal_dither_joint_default_config(
    adc_cal_dither_joint_config_t *config)
{
    if (config == NULL) return;
    config->channel_tolerance_samples = 1.0;
    config->existing_tolerance_samples = 1.0;
    config->channel_residual_penalty_weight = 0.25;
    config->existing_residual_penalty_weight = 0.25;
    config->margin_weight = 0.05;
    config->confidence_weight = 0.05;
    config->polarity_policy = ADC_CAL_DITHER_POLARITY_UNCONSTRAINED;
    config->use_expected_origin = 0;
    config->expected_origin_samples = NAN;
}

void adc_cal_dither_validation_default_config(
    adc_cal_dither_validation_config_t *config)
{
    if (config == NULL) return;
    config->minimum_complete_events = ADC_CAL_DITHER_DEFAULT_MIN_EVENTS;
    config->channel_disagreement_tolerance_samples = 1.0;
    config->existing_disagreement_tolerance_samples = 1.0;
    config->weak_peak_ratio = ADC_CAL_DITHER_DEFAULT_WEAK_PEAK_RATIO;
    config->strong_peak_ratio = ADC_CAL_DITHER_DEFAULT_STRONG_PEAK_RATIO;
}

const char *adc_cal_dither_status_name(adc_cal_dither_status_t status)
{
    switch (status) {
    case ADC_CAL_DITHER_OK: return "PASS";
    case ADC_CAL_DITHER_ERR_NULL: return "NULL_INPUT";
    case ADC_CAL_DITHER_ERR_SAMPLE_COUNT: return "SAMPLE_COUNT";
    case ADC_CAL_DITHER_ERR_NO_ENERGY: return "NO_ENERGY";
    case ADC_CAL_DITHER_ERR_NO_EVENTS: return "NO_EVENTS";
    case ADC_CAL_DITHER_ERR_TOO_FEW_EVENTS: return "TOO_FEW_EVENTS";
    case ADC_CAL_DITHER_ERR_POLARITY: return "POLARITY";
    case ADC_CAL_DITHER_ERR_NUMERICAL: return "NUMERICAL";
    default: return "UNKNOWN";
    }
}

const char *adc_cal_dither_confidence_name(
    adc_cal_dither_confidence_t confidence)
{
    switch (confidence) {
    case ADC_CAL_DITHER_CONFIDENCE_STRONG: return "STRONG";
    case ADC_CAL_DITHER_CONFIDENCE_WEAK: return "WEAK";
    case ADC_CAL_DITHER_CONFIDENCE_INVALID:
    default: return "INVALID";
    }
}

const char *adc_cal_dither_recommendation_name(
    adc_cal_dither_recommendation_t recommendation)
{
    switch (recommendation) {
    case ADC_CAL_DITHER_RECOMMEND_ACCEPT: return "ACCEPT";
    case ADC_CAL_DITHER_RECOMMEND_ACCEPT_WITH_WARNING:
        return "ACCEPT WITH WARNING";
    case ADC_CAL_DITHER_RECOMMEND_REJECT:
    default: return "REJECT";
    }
}

const char *adc_cal_dither_validation_reason_name(
    adc_cal_dither_validation_reason_t reason)
{
    switch (reason) {
    case ADC_CAL_DITHER_VALIDATION_NONE: return "NONE";
    case ADC_CAL_DITHER_VALIDATION_SELECTED_CHANNEL:
        return "SELECTED_CHANNEL_INVALID";
    case ADC_CAL_DITHER_VALIDATION_CHANNEL_AVAILABILITY:
        return "AVAILABLE_CHANNEL_INVALID";
    case ADC_CAL_DITHER_VALIDATION_NO_CONSISTENT_PAIR:
        return "NO_EXISTING_CONSISTENT_PAIR";
    case ADC_CAL_DITHER_VALIDATION_CHANNEL_DISAGREEMENT:
        return "CHANNEL_DISAGREEMENT";
    case ADC_CAL_DITHER_VALIDATION_EXISTING_DISAGREEMENT:
        return "EXISTING_DITHER_DISAGREEMENT";
    case ADC_CAL_DITHER_VALIDATION_TOO_FEW_EVENTS:
        return "TOO_FEW_COMPLETE_EVENTS";
    case ADC_CAL_DITHER_VALIDATION_EVENT_INDICES:
        return "EVENT_INDICES_INVALID";
    case ADC_CAL_DITHER_VALIDATION_NUMERICAL: return "NUMERICAL";
    case ADC_CAL_DITHER_VALIDATION_PEAK_BELOW_WEAK:
        return "PEAK_RATIO_BELOW_WEAK_THRESHOLD";
    case ADC_CAL_DITHER_VALIDATION_PEAK_BELOW_STRONG:
        return "PEAK_RATIO_BELOW_STRONG_THRESHOLD";
    case ADC_CAL_DITHER_VALIDATION_EXISTING_TIMING:
        return "EXISTING_TIMING_INVALID";
    case ADC_CAL_DITHER_VALIDATION_TONE_FIT: return "TONE_FIT_INVALID";
    case ADC_CAL_DITHER_VALIDATION_WINDOW: return "WINDOW_INVALID";
    default: return "UNKNOWN";
    }
}

int adc_cal_dither_select_independent_peaks(
    const double *scores,
    size_t score_count,
    double event_spacing_samples,
    const adc_cal_dither_peak_config_t *config,
    adc_cal_dither_peak_result_t *result)
{
    adc_cal_dither_peak_config_t local_config;
    double best_abs = -1.0;
    double raw_second_abs = -1.0;
    double independent_second_abs = -1.0;
    int use_periodic_exclusion;

    if (result == NULL) return ADC_CAL_DITHER_ERR_NULL;
    memset(result, 0, sizeof(*result));
    result->best_peak = NAN;
    result->raw_second_peak = NAN;
    result->independent_second_peak = NAN;
    result->raw_peak_ratio = NAN;
    result->independent_peak_ratio = NAN;
    if (scores == NULL) return ADC_CAL_DITHER_ERR_NULL;
    if (score_count < 2U) return ADC_CAL_DITHER_ERR_SAMPLE_COUNT;
    if (config == NULL) {
        adc_cal_dither_peak_default_config(&local_config);
        config = &local_config;
    }
    if (!adc_cal_double_isfinite(config->periodic_exclusion_width_samples) ||
        config->periodic_exclusion_width_samples < 0.0) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    for (size_t i = 0U; i < score_count; ++i) {
        const double magnitude = dither_abs(scores[i]);
        if (!adc_cal_double_isfinite(scores[i])) {
            return ADC_CAL_DITHER_ERR_NUMERICAL;
        }
        if (magnitude > best_abs) {
            best_abs = magnitude;
            result->best_index = i;
        }
    }
    use_periodic_exclusion =
        adc_cal_double_isfinite(event_spacing_samples) &&
        event_spacing_samples > 0.0 &&
        event_spacing_samples < (double)score_count;
    result->periodic_exclusion_applied = use_periodic_exclusion;
    for (size_t i = 0U; i < score_count; ++i) {
        const double magnitude = dither_abs(scores[i]);
        int periodic_equivalent = 0;
        if (dither_circular_distance(i, result->best_index, score_count) <=
            config->local_exclusion_samples) {
            continue;
        }
        if (magnitude > raw_second_abs) {
            raw_second_abs = magnitude;
            result->raw_second_index = i;
        }
        if (use_periodic_exclusion) {
            const double raw_delta =
                (double)i - (double)result->best_index;
            const int first_frame_offset = raw_delta > 0.0 ? -1 : 0;
            const int last_frame_offset = raw_delta < 0.0 ? 1 : 0;
            for (int frame_offset = first_frame_offset;
                 frame_offset <= last_frame_offset && !periodic_equivalent;
                 ++frame_offset) {
                const double lifted_delta = raw_delta +
                    (double)frame_offset * (double)score_count;
                const double multiple = floor(
                    dither_abs(lifted_delta) /
                        event_spacing_samples + 0.5);
                if (multiple >= 1.0 &&
                    dither_abs(dither_abs(lifted_delta) -
                               multiple * event_spacing_samples) <=
                        config->periodic_exclusion_width_samples) {
                    periodic_equivalent = 1;
                }
            }
        }
        if (!periodic_equivalent && magnitude > independent_second_abs) {
            independent_second_abs = magnitude;
            result->independent_second_index = i;
        }
    }
    if (raw_second_abs < 0.0) return ADC_CAL_DITHER_ERR_NO_EVENTS;
    result->best_peak = scores[result->best_index];
    result->raw_second_peak = scores[result->raw_second_index];
    if (independent_second_abs >= 0.0) {
        result->independent_second_peak =
            scores[result->independent_second_index];
    } else {
        independent_second_abs = 0.0;
        result->independent_second_index = result->best_index;
        result->independent_second_peak = 0.0;
    }
    if (raw_second_abs > DBL_EPSILON) {
        result->raw_peak_ratio = best_abs / raw_second_abs;
    } else {
        result->raw_peak_ratio = DBL_MAX;
    }
    if (independent_second_abs > DBL_EPSILON) {
        result->independent_peak_ratio = best_abs / independent_second_abs;
    } else {
        result->independent_peak_ratio = DBL_MAX;
    }
    result->valid = adc_cal_double_isfinite(result->best_peak) &&
        adc_cal_double_isfinite(result->raw_second_peak) &&
        adc_cal_double_isfinite(result->independent_second_peak) &&
        adc_cal_double_isfinite(result->raw_peak_ratio) &&
        adc_cal_double_isfinite(result->independent_peak_ratio);
    return result->valid ? 0 : ADC_CAL_DITHER_ERR_NUMERICAL;
}

int adc_cal_dither_find_peak_candidates(
    const double *scores,
    size_t score_count,
    double event_spacing_samples,
    const adc_cal_dither_peak_config_t *peak_config,
    const adc_cal_dither_validation_config_t *confidence_config,
    adc_cal_dither_peak_candidate_t *candidates,
    size_t candidate_capacity,
    size_t *candidate_count)
{
    adc_cal_dither_peak_config_t local_peak_config;
    adc_cal_dither_validation_config_t local_confidence_config;
    size_t limit;
    size_t global_strongest_index = 0U;
    double global_strongest_abs = -1.0;

    if (candidate_count == NULL) return ADC_CAL_DITHER_ERR_NULL;
    *candidate_count = 0U;
    if (scores == NULL || candidates == NULL) return ADC_CAL_DITHER_ERR_NULL;
    if (score_count < 3U || candidate_capacity == 0U) {
        return ADC_CAL_DITHER_ERR_SAMPLE_COUNT;
    }
    if (peak_config == NULL) {
        adc_cal_dither_peak_default_config(&local_peak_config);
        peak_config = &local_peak_config;
    }
    if (confidence_config == NULL) {
        adc_cal_dither_validation_default_config(&local_confidence_config);
        confidence_config = &local_confidence_config;
    }
    if (!adc_cal_double_isfinite(
            peak_config->periodic_exclusion_width_samples) ||
        peak_config->periodic_exclusion_width_samples < 0.0 ||
        peak_config->maximum_candidates == 0U ||
        !adc_cal_double_isfinite(confidence_config->weak_peak_ratio) ||
        !adc_cal_double_isfinite(confidence_config->strong_peak_ratio) ||
        confidence_config->weak_peak_ratio <= 1.0 ||
        confidence_config->strong_peak_ratio <
            confidence_config->weak_peak_ratio) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    for (size_t i = 0U; i < score_count; ++i) {
        if (!adc_cal_double_isfinite(scores[i])) {
            return ADC_CAL_DITHER_ERR_NUMERICAL;
        }
        if (dither_abs(scores[i]) > global_strongest_abs) {
            global_strongest_abs = dither_abs(scores[i]);
            global_strongest_index = i;
        }
    }

    memset(candidates, 0,
           candidate_capacity * sizeof(candidates[0]));
    limit = peak_config->maximum_candidates < candidate_capacity ?
        peak_config->maximum_candidates : candidate_capacity;

    while (*candidate_count < limit) {
        size_t best_index = 0U;
        double best_abs = -1.0;
        int found = 0;

        for (size_t i = 0U; i < score_count; ++i) {
            const size_t left = i == 0U ? score_count - 1U : i - 1U;
            const size_t right = i + 1U == score_count ? 0U : i + 1U;
            const double magnitude = dither_abs(scores[i]);
            int locally_suppressed = 0;
            /* Retain peaks, not merely the strongest bins left after
             * suppression.  Interpolating a slope bin can place the parabola
             * outside +/-0.5 and previously forced its fraction back to 0. */
            if (magnitude < dither_abs(scores[left]) ||
                magnitude < dither_abs(scores[right]) ||
                (magnitude == dither_abs(scores[left]) &&
                 magnitude == dither_abs(scores[right]))) {
                continue;
            }
            for (size_t selected = 0U;
                 selected < *candidate_count;
                 ++selected) {
                if (dither_circular_distance(
                        i, candidates[selected].index, score_count) <=
                    peak_config->local_exclusion_samples) {
                    locally_suppressed = 1;
                    break;
                }
            }
            if (!locally_suppressed && magnitude > best_abs) {
                best_abs = magnitude;
                best_index = i;
                found = 1;
            }
        }
        if (!found || best_abs <= DBL_EPSILON) break;

        {
            adc_cal_dither_peak_candidate_t *candidate =
                &candidates[*candidate_count];
            const size_t left = best_index == 0U ?
                score_count - 1U : best_index - 1U;
            const size_t right = best_index + 1U == score_count ?
                0U : best_index + 1U;
            const double y0 = dither_abs(scores[left]);
            const double y1 = dither_abs(scores[best_index]);
            const double y2 = dither_abs(scores[right]);
            const double denominator = y0 - 2.0 * y1 + y2;
            double fraction = 0.0;
            double raw_second_abs = -1.0;
            double independent_second_abs = -1.0;
            double background_sum = 0.0;
            double background_square_sum = 0.0;
            size_t background_count = 0U;
            size_t raw_second_index = best_index;
            size_t independent_second_index = best_index;

            if (dither_abs(denominator) > 1.0e-18) {
                fraction = 0.5 * (y0 - y2) / denominator;
                if (!adc_cal_double_isfinite(fraction) ||
                    fraction < -0.5 || fraction > 0.5) {
                    fraction = 0.0;
                }
            }
            candidate->index = best_index;
            candidate->fractional_offset_samples = fraction;
            candidate->lag_samples = best_index <= score_count / 2U ?
                (double)best_index + fraction :
                (double)best_index - (double)score_count + fraction;
            candidate->coarse_index = candidate->index;
            candidate->coarse_fractional_offset_samples =
                candidate->fractional_offset_samples;
            candidate->coarse_lag_samples = candidate->lag_samples;
            candidate->wrapped_origin_samples = dither_wrap_position(
                -candidate->lag_samples, (double)score_count);
            candidate->peak = scores[best_index];
            candidate->absolute_peak = best_abs;
            candidate->global_strongest_index = global_strongest_index;
            candidate->global_strongest_peak =
                scores[global_strongest_index];
            candidate->global_strongest_absolute_peak =
                global_strongest_abs;

            for (size_t i = 0U; i < score_count; ++i) {
                const double magnitude = dither_abs(scores[i]);
                if (dither_circular_distance(
                        i, best_index, score_count) <=
                    peak_config->local_exclusion_samples) {
                    continue;
                }
                background_sum += magnitude;
                background_square_sum += magnitude * magnitude;
                ++background_count;
                if (magnitude > raw_second_abs) {
                    raw_second_abs = magnitude;
                    raw_second_index = i;
                }
                if (!dither_periodic_equivalent(
                        i, best_index, score_count, event_spacing_samples,
                        peak_config->periodic_exclusion_width_samples) &&
                    magnitude > independent_second_abs) {
                    independent_second_abs = magnitude;
                    independent_second_index = i;
                }
            }

            candidate->raw_second_peak = raw_second_abs >= 0.0 ?
                scores[raw_second_index] : 0.0;
            candidate->independent_second_peak =
                independent_second_abs >= 0.0 ?
                scores[independent_second_index] : 0.0;
            candidate->raw_peak_ratio = raw_second_abs > DBL_EPSILON ?
                best_abs / raw_second_abs : DBL_MAX;
            candidate->independent_peak_ratio =
                independent_second_abs > DBL_EPSILON ?
                best_abs / independent_second_abs : DBL_MAX;
            candidate->margin = 0.0;
            if (background_count > 1U) {
                const double mean =
                    background_sum / (double)background_count;
                double variance =
                    background_square_sum / (double)background_count -
                    mean * mean;
                if (variance < 0.0 && variance > -1.0e-18) variance = 0.0;
                if (variance > DBL_EPSILON) {
                    candidate->margin =
                        (best_abs - mean) / sqrt(variance);
                }
            }
            candidate->confidence = dither_classify_peak_ratio(
                candidate->independent_peak_ratio, confidence_config);
            candidate->valid =
                adc_cal_double_isfinite(candidate->lag_samples) &&
                adc_cal_double_isfinite(candidate->wrapped_origin_samples) &&
                adc_cal_double_isfinite(candidate->peak) &&
                adc_cal_double_isfinite(candidate->absolute_peak) &&
                adc_cal_double_isfinite(candidate->global_strongest_peak) &&
                adc_cal_double_isfinite(
                    candidate->global_strongest_absolute_peak) &&
                adc_cal_double_isfinite(candidate->raw_second_peak) &&
                adc_cal_double_isfinite(candidate->independent_second_peak) &&
                adc_cal_double_isfinite(candidate->raw_peak_ratio) &&
                adc_cal_double_isfinite(candidate->independent_peak_ratio) &&
                adc_cal_double_isfinite(candidate->margin) &&
                candidate->confidence != ADC_CAL_DITHER_CONFIDENCE_INVALID;
            if (!candidate->valid) return ADC_CAL_DITHER_ERR_NUMERICAL;
        }
        ++(*candidate_count);
    }

    return *candidate_count > 0U ? 0 : ADC_CAL_DITHER_ERR_NO_EVENTS;
}

int adc_cal_dither_refine_candidate_lags(
    const double *edge_scores,
    size_t score_count,
    size_t search_radius,
    adc_cal_dither_peak_candidate_t *candidates,
    size_t candidate_count)
{
    if (edge_scores == NULL || candidates == NULL) {
        return ADC_CAL_DITHER_ERR_NULL;
    }
    if (score_count < 3U || search_radius >= score_count / 2U) {
        return ADC_CAL_DITHER_ERR_SAMPLE_COUNT;
    }
    for (size_t i = 0U; i < score_count; ++i) {
        if (!adc_cal_double_isfinite(edge_scores[i])) {
            return ADC_CAL_DITHER_ERR_NUMERICAL;
        }
    }
    for (size_t candidate_index = 0U;
         candidate_index < candidate_count;
         ++candidate_index) {
        adc_cal_dither_peak_candidate_t *candidate =
            &candidates[candidate_index];
        size_t best_index;
        size_t best_distance = 0U;
        double best_abs;
        double fraction = 0.0;
        if (!candidate->valid || candidate->coarse_index >= score_count) {
            continue;
        }
        best_index = candidate->coarse_index;
        best_abs = dither_abs(edge_scores[best_index]);
        for (int direction = -1; direction <= 1; direction += 2) {
            for (size_t distance = 1U;
                 distance <= search_radius;
                 ++distance) {
                int64_t raw_index = (int64_t)candidate->coarse_index +
                    (int64_t)direction * (int64_t)distance;
                size_t index;
                double magnitude;
                raw_index %= (int64_t)score_count;
                if (raw_index < 0) raw_index += (int64_t)score_count;
                index = (size_t)raw_index;
                magnitude = dither_abs(edge_scores[index]);
                if (magnitude > best_abs ||
                    (magnitude == best_abs && distance < best_distance)) {
                    best_abs = magnitude;
                    best_index = index;
                    best_distance = distance;
                }
            }
        }
        {
            const size_t left = best_index == 0U ?
                score_count - 1U : best_index - 1U;
            const size_t right = best_index + 1U == score_count ?
                0U : best_index + 1U;
            const double y0 = dither_abs(edge_scores[left]);
            const double y1 = dither_abs(edge_scores[best_index]);
            const double y2 = dither_abs(edge_scores[right]);
            const double denominator = y0 - 2.0 * y1 + y2;
            if (dither_abs(denominator) > 1.0e-18) {
                fraction = 0.5 * (y0 - y2) / denominator;
                if (!adc_cal_double_isfinite(fraction) ||
                    fraction < -0.5 || fraction > 0.5) {
                    fraction = candidate->coarse_fractional_offset_samples;
                    best_index = candidate->coarse_index;
                }
            } else {
                fraction = candidate->coarse_fractional_offset_samples;
                best_index = candidate->coarse_index;
            }
        }
        candidate->index = best_index;
        candidate->fractional_offset_samples = fraction;
        candidate->lag_samples = best_index <= score_count / 2U ?
            (double)best_index + fraction :
            (double)best_index - (double)score_count + fraction;
        candidate->wrapped_origin_samples = dither_wrap_position(
            -candidate->lag_samples, (double)score_count);
        if (!adc_cal_double_isfinite(candidate->lag_samples) ||
            !adc_cal_double_isfinite(candidate->wrapped_origin_samples)) {
            return ADC_CAL_DITHER_ERR_NUMERICAL;
        }
    }
    return 0;
}

int adc_cal_dither_validate_detection(
    const adc_cal_dither_validation_input_t *input,
    const adc_cal_dither_validation_config_t *config,
    adc_cal_dither_validation_result_t *result)
{
    adc_cal_dither_validation_config_t local_config;
    if (result == NULL) return ADC_CAL_DITHER_ERR_NULL;
    memset(result, 0, sizeof(*result));
    result->confidence = ADC_CAL_DITHER_CONFIDENCE_INVALID;
    result->recommendation = ADC_CAL_DITHER_RECOMMEND_REJECT;
    result->reason = ADC_CAL_DITHER_VALIDATION_NUMERICAL;
    if (input == NULL) return ADC_CAL_DITHER_ERR_NULL;
    if (config == NULL) {
        adc_cal_dither_validation_default_config(&local_config);
        config = &local_config;
    }
    if (!adc_cal_double_isfinite(config->channel_disagreement_tolerance_samples) ||
        !adc_cal_double_isfinite(config->existing_disagreement_tolerance_samples) ||
        !adc_cal_double_isfinite(config->weak_peak_ratio) ||
        !adc_cal_double_isfinite(config->strong_peak_ratio) ||
        config->channel_disagreement_tolerance_samples < 0.0 ||
        config->existing_disagreement_tolerance_samples < 0.0 ||
        config->weak_peak_ratio <= 1.0 ||
        config->strong_peak_ratio < config->weak_peak_ratio) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    result->confidence = dither_classify_peak_ratio(
        input->independent_peak_ratio, config);
    if (!input->selected_channel_valid) {
        result->reason = ADC_CAL_DITHER_VALIDATION_SELECTED_CHANNEL;
        return 0;
    }
    if ((input->channel_a_available && !input->channel_a_valid) ||
        (input->channel_b_available && !input->channel_b_valid)) {
        result->reason = ADC_CAL_DITHER_VALIDATION_CHANNEL_AVAILABILITY;
        return 0;
    }
    if (input->joint_pair_required && !input->joint_pair_valid) {
        result->reason = ADC_CAL_DITHER_VALIDATION_NO_CONSISTENT_PAIR;
        return 0;
    }
    if (!adc_cal_double_isfinite(input->channel_disagreement_samples) ||
        dither_abs(input->channel_disagreement_samples) >
            config->channel_disagreement_tolerance_samples) {
        result->reason = ADC_CAL_DITHER_VALIDATION_CHANNEL_DISAGREEMENT;
        return 0;
    }
    if (input->existing_comparison_required) {
        if (!adc_cal_double_isfinite(input->existing_disagreement_samples) ||
            dither_abs(input->existing_disagreement_samples) >
                config->existing_disagreement_tolerance_samples) {
            result->reason = ADC_CAL_DITHER_VALIDATION_EXISTING_DISAGREEMENT;
            return 0;
        }
    }
    if (input->complete_event_count < config->minimum_complete_events) {
        result->reason = ADC_CAL_DITHER_VALIDATION_TOO_FEW_EVENTS;
        return 0;
    }
    if (!input->event_indices_valid) {
        result->reason = ADC_CAL_DITHER_VALIDATION_EVENT_INDICES;
        return 0;
    }
    if (!input->numerical_values_finite ||
        !adc_cal_double_isfinite(input->independent_peak_ratio)) {
        result->reason = ADC_CAL_DITHER_VALIDATION_NUMERICAL;
        return 0;
    }
    result->structural_valid = 1;
    if (result->confidence == ADC_CAL_DITHER_CONFIDENCE_STRONG) {
        result->recommendation = ADC_CAL_DITHER_RECOMMEND_ACCEPT;
        result->reason = ADC_CAL_DITHER_VALIDATION_NONE;
    } else if (result->confidence == ADC_CAL_DITHER_CONFIDENCE_WEAK) {
        if (input->independent_peak_ratio < config->weak_peak_ratio) {
            /* Peak uniqueness is advisory once independent structural checks
             * (joint A/B consistency, timing-family agreement, event mapping,
             * and finite numerics) have all passed.  A constrained candidate
             * can legitimately be smaller than an isolated invalid peak. */
            result->recommendation =
                ADC_CAL_DITHER_RECOMMEND_ACCEPT_WITH_WARNING;
            result->reason = ADC_CAL_DITHER_VALIDATION_PEAK_BELOW_WEAK;
        } else {
            result->recommendation =
                ADC_CAL_DITHER_RECOMMEND_ACCEPT_WITH_WARNING;
            result->reason = ADC_CAL_DITHER_VALIDATION_PEAK_BELOW_STRONG;
        }
    } else {
        result->structural_valid = 0;
        result->recommendation = ADC_CAL_DITHER_RECOMMEND_REJECT;
        result->reason = ADC_CAL_DITHER_VALIDATION_NUMERICAL;
    }
    return 0;
}

int adc_cal_dither_summarize_events(
    const double *template_samples,
    size_t sample_count,
    size_t window_start,
    size_t window_length,
    double threshold_fraction,
    adc_cal_dither_event_summary_t *summary)
{
    double peak = 0.0;
    double previous_center = NAN;
    double spacing_sum = 0.0;
    size_t spacing_count = 0U;
    size_t index = 0U;
    size_t window_end;
    if (summary == NULL) return ADC_CAL_DITHER_ERR_NULL;
    memset(summary, 0, sizeof(*summary));
    summary->spacing_samples = NAN;
    summary->first_complete_center = NAN;
    summary->last_complete_center = NAN;
    if (template_samples == NULL) return ADC_CAL_DITHER_ERR_NULL;
    if (sample_count == 0U || window_start >= sample_count ||
        window_length == 0U || window_length > sample_count - window_start) {
        return ADC_CAL_DITHER_ERR_SAMPLE_COUNT;
    }
    if (!adc_cal_double_isfinite(threshold_fraction) ||
        threshold_fraction <= 0.0 || threshold_fraction >= 1.0) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    window_end = window_start + window_length;
    for (size_t i = 0U; i < sample_count; ++i) {
        if (!adc_cal_double_isfinite(template_samples[i])) {
            return ADC_CAL_DITHER_ERR_NUMERICAL;
        }
        if (dither_abs(template_samples[i]) > peak) {
            peak = dither_abs(template_samples[i]);
        }
    }
    if (peak <= DBL_EPSILON) return ADC_CAL_DITHER_ERR_NO_ENERGY;
    while (index < sample_count) {
        size_t start;
        size_t end;
        double weighted = 0.0;
        double weight = 0.0;
        double center;
        if (dither_abs(template_samples[index]) < peak * threshold_fraction) {
            ++index;
            continue;
        }
        start = index;
        while (index < sample_count &&
               dither_abs(template_samples[index]) >=
                   peak * threshold_fraction) {
            const double magnitude = dither_abs(template_samples[index]);
            weighted += (double)index * magnitude;
            weight += magnitude;
            ++index;
        }
        end = index;
        center = weighted / weight;
        ++summary->total_count;
        if (adc_cal_double_isfinite(previous_center)) {
            spacing_sum += center - previous_center;
            ++spacing_count;
        }
        previous_center = center;
        if (start >= window_start && end <= window_end) {
            if (summary->complete_count == 0U) {
                summary->first_complete_center = center;
            }
            summary->last_complete_center = center;
            ++summary->complete_count;
        } else if (start < window_end && end > window_start) {
            ++summary->partial_count;
        }
    }
    if (summary->total_count == 0U) return ADC_CAL_DITHER_ERR_NO_EVENTS;
    if (spacing_count > 0U) {
        summary->spacing_samples = spacing_sum / (double)spacing_count;
    }
    summary->valid = 1;
    return 0;
}

int adc_cal_dither_window_is_valid(
    size_t complete_event_count,
    size_t minimum_complete_events,
    int event_indices_valid,
    int event_family_coherent)
{
    return minimum_complete_events > 0U &&
        complete_event_count >= minimum_complete_events &&
        event_indices_valid && event_family_coherent;
}

int adc_cal_dither_compare_periodic_origins(
    double first_origin_samples,
    double second_origin_samples,
    double event_spacing_samples,
    double frame_period_samples,
    adc_cal_dither_periodic_difference_t *difference)
{
    double best_abs = DBL_MAX;
    int first_frame_offset = 0;
    int last_frame_offset = 0;

    if (difference == NULL) return ADC_CAL_DITHER_ERR_NULL;
    memset(difference, 0, sizeof(*difference));
    difference->raw_difference_samples = NAN;
    difference->signed_difference_samples = NAN;
    difference->absolute_difference_samples = NAN;
    if (!adc_cal_double_isfinite(first_origin_samples) ||
        !adc_cal_double_isfinite(second_origin_samples) ||
        !adc_cal_double_isfinite(event_spacing_samples) ||
        event_spacing_samples <= DBL_EPSILON) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    difference->raw_difference_samples =
        first_origin_samples - second_origin_samples;
    if (!adc_cal_double_isfinite(frame_period_samples) ||
        frame_period_samples < 0.0) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    if (frame_period_samples > DBL_EPSILON) {
        if (difference->raw_difference_samples > 0.0) {
            first_frame_offset = -1;
        } else if (difference->raw_difference_samples < 0.0) {
            last_frame_offset = 1;
        }
    }
    for (int frame_offset = first_frame_offset;
         frame_offset <= last_frame_offset;
         ++frame_offset) {
        const double lifted = difference->raw_difference_samples +
            (double)frame_offset * frame_period_samples;
        const double quotient = lifted / event_spacing_samples;
        double nearest_offset;
        double reduced;
        double reduced_abs;
        if (!adc_cal_double_isfinite(quotient) ||
            quotient > (double)INT32_MAX || quotient < (double)INT32_MIN) {
            return ADC_CAL_DITHER_ERR_NUMERICAL;
        }
        nearest_offset = quotient >= 0.0 ?
            floor(quotient + 0.5) : ceil(quotient - 0.5);
        reduced = lifted - nearest_offset * event_spacing_samples;
        if (reduced > 0.5 * event_spacing_samples) {
            reduced -= event_spacing_samples;
            nearest_offset += 1.0;
        } else if (reduced < -0.5 * event_spacing_samples) {
            reduced += event_spacing_samples;
            nearest_offset -= 1.0;
        }
        if (nearest_offset > (double)INT32_MAX ||
            nearest_offset < (double)INT32_MIN) {
            return ADC_CAL_DITHER_ERR_NUMERICAL;
        }
        reduced_abs = dither_abs(reduced);
        if (reduced_abs < best_abs ||
            (reduced_abs == best_abs &&
             dither_abs((double)frame_offset) <
                 dither_abs((double)difference->frame_offset))) {
            best_abs = reduced_abs;
            difference->frame_offset = (int32_t)frame_offset;
            difference->event_offset = (int32_t)nearest_offset;
            difference->signed_difference_samples = reduced;
            difference->absolute_difference_samples = reduced_abs;
        }
    }
    difference->valid =
        adc_cal_double_isfinite(difference->signed_difference_samples) &&
        adc_cal_double_isfinite(difference->absolute_difference_samples);
    return difference->valid ? 0 : ADC_CAL_DITHER_ERR_NUMERICAL;
}

int adc_cal_dither_compare_periodic_lags(
    double first_lag_samples,
    double second_lag_samples,
    double event_spacing_samples,
    double frame_period_samples,
    adc_cal_dither_periodic_difference_t *difference)
{
    return adc_cal_dither_compare_periodic_origins(
        first_lag_samples, second_lag_samples,
        event_spacing_samples, frame_period_samples, difference);
}

int adc_cal_dither_resolve_tone_cycle(
    double ambiguous_lag_samples,
    double tone_period_samples,
    double dither_origin_samples,
    double event_spacing_samples,
    double frame_period_samples,
    double maximum_residual_samples,
    double *resolved_lag_samples,
    int32_t *tone_cycle_offset,
    adc_cal_dither_periodic_difference_t *residual)
{
    adc_cal_dither_periodic_difference_t best_difference;
    double best_lag = NAN;
    int32_t best_cycle = 0;
    int found = 0;
    int32_t maximum_cycles;
    if (resolved_lag_samples == NULL || tone_cycle_offset == NULL ||
        residual == NULL) return ADC_CAL_DITHER_ERR_NULL;
    *resolved_lag_samples = NAN;
    *tone_cycle_offset = 0;
    memset(residual, 0, sizeof(*residual));
    residual->signed_difference_samples = NAN;
    residual->absolute_difference_samples = NAN;
    if (!adc_cal_double_isfinite(ambiguous_lag_samples) ||
        !adc_cal_double_isfinite(tone_period_samples) ||
        tone_period_samples <= DBL_EPSILON ||
        !adc_cal_double_isfinite(dither_origin_samples) ||
        !adc_cal_double_isfinite(event_spacing_samples) ||
        event_spacing_samples <= DBL_EPSILON ||
        !adc_cal_double_isfinite(frame_period_samples) ||
        frame_period_samples <= DBL_EPSILON ||
        !adc_cal_double_isfinite(maximum_residual_samples) ||
        maximum_residual_samples < 0.0) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    maximum_cycles = (int32_t)ceil(
        0.5 * frame_period_samples / tone_period_samples) + 1;
    for (int32_t cycle = -maximum_cycles;
         cycle <= maximum_cycles; ++cycle) {
        const double candidate_lag = ambiguous_lag_samples +
            (double)cycle * tone_period_samples;
        double candidate_origin;
        adc_cal_dither_periodic_difference_t difference;
        if (candidate_lag < -0.5 * frame_period_samples ||
            candidate_lag >= 0.5 * frame_period_samples) continue;
        if (adc_cal_dither_lag_to_wrapped_origin(
                candidate_lag, frame_period_samples,
                &candidate_origin) != 0 ||
            adc_cal_dither_compare_periodic_origins(
                candidate_origin, dither_origin_samples,
                event_spacing_samples, frame_period_samples,
                &difference) != 0) continue;
        if (difference.absolute_difference_samples >
            maximum_residual_samples) continue;
        if (!found ||
            dither_abs((double)cycle) < dither_abs((double)best_cycle) ||
            (dither_abs((double)cycle) == dither_abs((double)best_cycle) &&
             difference.absolute_difference_samples <
                 best_difference.absolute_difference_samples)) {
            found = 1;
            best_lag = candidate_lag;
            best_cycle = cycle;
            best_difference = difference;
        }
    }
    if (!found) return ADC_CAL_DITHER_ERR_NO_EVENTS;
    *resolved_lag_samples = best_lag;
    *tone_cycle_offset = best_cycle;
    *residual = best_difference;
    return 0;
}

int adc_cal_dither_lag_to_wrapped_origin(
    double lag_samples,
    double frame_period_samples,
    double *wrapped_origin_samples)
{
    if (wrapped_origin_samples == NULL) return ADC_CAL_DITHER_ERR_NULL;
    *wrapped_origin_samples = dither_wrap_position(
        -lag_samples, frame_period_samples);
    return adc_cal_double_isfinite(*wrapped_origin_samples) ?
        0 : ADC_CAL_DITHER_ERR_NUMERICAL;
}

static double dither_correlation_observable(
    const double *samples,
    size_t sample_count,
    size_t index,
    adc_cal_dither_correlation_mode_t mode)
{
    double value = samples[index];
    if (mode == ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY) {
        if (index < 2U || index + 2U >= sample_count) {
            value = 0.0;
        } else {
            /* Central difference after [1 2 1]/4 smoothing:
             * (x[n+2] + 2x[n+1] - 2x[n-1] - x[n-2]) / 8. */
            value = (samples[index + 2U] +
                     2.0 * samples[index + 1U] -
                     2.0 * samples[index - 1U] -
                     samples[index - 2U]) / 8.0;
        }
    }
    if (mode == ADC_CAL_DITHER_CORRELATION_ENERGY ||
        mode == ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY) {
        value *= value;
    }
    return value;
}

int adc_cal_dither_compute_circular_scores(
    const double *template_samples,
    const double *capture_samples,
    size_t sample_count,
    adc_cal_dither_correlation_mode_t mode,
    double *scores)
{
    double template_mean = 0.0;
    double capture_mean = 0.0;
    double template_power = 0.0;
    double capture_power = 0.0;
    double normalization;

    if (template_samples == NULL || capture_samples == NULL ||
        scores == NULL) {
        return ADC_CAL_DITHER_ERR_NULL;
    }
    if (sample_count < 2U) return ADC_CAL_DITHER_ERR_SAMPLE_COUNT;
    if (mode != ADC_CAL_DITHER_CORRELATION_SIGNED &&
        mode != ADC_CAL_DITHER_CORRELATION_ENERGY &&
        mode != ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }

    for (size_t i = 0U; i < sample_count; ++i) {
        double template_value;
        double capture_value;
        if (!adc_cal_double_isfinite(template_samples[i]) ||
            !adc_cal_double_isfinite(capture_samples[i])) {
            return ADC_CAL_DITHER_ERR_NUMERICAL;
        }
        template_value = dither_correlation_observable(
            template_samples, sample_count, i, mode);
        capture_value = dither_correlation_observable(
            capture_samples, sample_count, i, mode);
        template_mean += template_value;
        capture_mean += capture_value;
    }
    template_mean /= (double)sample_count;
    capture_mean /= (double)sample_count;

    for (size_t i = 0U; i < sample_count; ++i) {
        double template_value = dither_correlation_observable(
            template_samples, sample_count, i, mode);
        double capture_value = dither_correlation_observable(
            capture_samples, sample_count, i, mode);
        template_value -= template_mean;
        capture_value -= capture_mean;
        template_power += template_value * template_value;
        capture_power += capture_value * capture_value;
    }
    normalization = sqrt(template_power * capture_power);
    if (!adc_cal_double_isfinite(normalization) ||
        normalization <= DBL_EPSILON) {
        return ADC_CAL_DITHER_ERR_NO_ENERGY;
    }

    /* This is direct circular cross-correlation, not convolution:
     * score[lag] = sum_i T[i] C[(i + lag) mod N].  Consequently no
     * L-1 or (L-1)/2 finite-template delay is present. */
    for (size_t lag = 0U; lag < sample_count; ++lag) {
        double numerator = 0.0;
        for (size_t i = 0U; i < sample_count; ++i) {
            size_t capture_index = i + lag;
            double template_value;
            double capture_value;
            if (capture_index >= sample_count) capture_index -= sample_count;
            template_value = dither_correlation_observable(
                template_samples, sample_count, i, mode);
            capture_value = dither_correlation_observable(
                capture_samples, sample_count, capture_index, mode);
            numerator += (template_value - template_mean) *
                (capture_value - capture_mean);
        }
        scores[lag] = numerator / normalization;
    }
    return 0;
}

int adc_cal_dither_compute_timing_scores(
    const double *template_samples,
    const double *capture_samples,
    size_t sample_count,
    double energy_weight,
    double edge_weight,
    double *scores)
{
    double template_mean = 0.0;
    double capture_mean = 0.0;
    double template_power = 0.0;
    double capture_power = 0.0;
    double normalization;
    double weight_sum;
    int status;

    if (!adc_cal_double_isfinite(energy_weight) || energy_weight < 0.0 ||
        !adc_cal_double_isfinite(edge_weight) || edge_weight < 0.0) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    weight_sum = energy_weight + edge_weight;
    if (!adc_cal_double_isfinite(weight_sum) || weight_sum <= DBL_EPSILON) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    status = adc_cal_dither_compute_circular_scores(
        template_samples, capture_samples, sample_count,
        ADC_CAL_DITHER_CORRELATION_ENERGY, scores);
    if (status != 0) return status;
    energy_weight /= weight_sum;
    edge_weight /= weight_sum;

    for (size_t i = 0U; i < sample_count; ++i) {
        const double template_value = dither_correlation_observable(
            template_samples, sample_count, i,
            ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY);
        const double capture_value = dither_correlation_observable(
            capture_samples, sample_count, i,
            ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY);
        template_mean += template_value;
        capture_mean += capture_value;
    }
    template_mean /= (double)sample_count;
    capture_mean /= (double)sample_count;
    for (size_t i = 0U; i < sample_count; ++i) {
        const double template_value = dither_correlation_observable(
            template_samples, sample_count, i,
            ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY) - template_mean;
        const double capture_value = dither_correlation_observable(
            capture_samples, sample_count, i,
            ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY) - capture_mean;
        template_power += template_value * template_value;
        capture_power += capture_value * capture_value;
    }
    normalization = sqrt(template_power * capture_power);
    if (!adc_cal_double_isfinite(normalization) ||
        normalization <= DBL_EPSILON) {
        return ADC_CAL_DITHER_ERR_NO_ENERGY;
    }
    for (size_t lag = 0U; lag < sample_count; ++lag) {
        double numerator = 0.0;
        for (size_t i = 0U; i < sample_count; ++i) {
            size_t capture_index = i + lag;
            double template_value;
            double capture_value;
            if (capture_index >= sample_count) capture_index -= sample_count;
            template_value = dither_correlation_observable(
                template_samples, sample_count, i,
                ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY) - template_mean;
            capture_value = dither_correlation_observable(
                capture_samples, sample_count, capture_index,
                ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY) - capture_mean;
            numerator += template_value * capture_value;
        }
        scores[lag] = energy_weight * scores[lag] +
            edge_weight * numerator / normalization;
    }
    return 0;
}

int adc_cal_dither_template_anchor_delay(
    const double *template_samples,
    size_t sample_count,
    adc_cal_dither_correlation_mode_t mode,
    double *delay_samples)
{
    double mean = 0.0;
    double power = 0.0;

    if (delay_samples == NULL) return ADC_CAL_DITHER_ERR_NULL;
    *delay_samples = NAN;
    if (template_samples == NULL) return ADC_CAL_DITHER_ERR_NULL;
    if (sample_count < 2U) return ADC_CAL_DITHER_ERR_SAMPLE_COUNT;
    if (mode != ADC_CAL_DITHER_CORRELATION_SIGNED &&
        mode != ADC_CAL_DITHER_CORRELATION_ENERGY &&
        mode != ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    for (size_t i = 0U; i < sample_count; ++i) {
        double value = template_samples[i];
        if (!adc_cal_double_isfinite(value)) {
            return ADC_CAL_DITHER_ERR_NUMERICAL;
        }
        if (mode == ADC_CAL_DITHER_CORRELATION_ENERGY) value *= value;
        if (mode == ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY) {
            value = dither_correlation_observable(
                template_samples, sample_count, i, mode);
        }
        mean += value;
    }
    mean /= (double)sample_count;
    for (size_t i = 0U; i < sample_count; ++i) {
        double value = template_samples[i];
        if (mode == ADC_CAL_DITHER_CORRELATION_ENERGY) value *= value;
        if (mode == ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY) {
            value = dither_correlation_observable(
                template_samples, sample_count, i, mode);
        }
        value -= mean;
        power += value * value;
    }
    if (!adc_cal_double_isfinite(power) || power <= DBL_EPSILON) {
        return ADC_CAL_DITHER_ERR_NO_ENERGY;
    }
    /* At lag zero the normalized autocorrelation is exactly one.  The
     * Cauchy-Schwarz bound prevents a larger value at any other lag, and the
     * first-maximum rule selects zero even for a perfectly periodic train.
     * Circular neighbors are symmetric, so parabolic refinement is also 0. */
    *delay_samples = 0.0;
    return 0;
}

int adc_cal_dither_expected_lag(
    double existing_lag_samples,
    const adc_cal_dither_lag_offsets_t *offsets,
    double *expected_lag_samples)
{
    if (offsets == NULL || expected_lag_samples == NULL) {
        return ADC_CAL_DITHER_ERR_NULL;
    }
    *expected_lag_samples = existing_lag_samples +
        offsets->reference_anchor_offset_samples +
        offsets->resampling_delay_samples +
        offsets->template_anchor_delay_samples +
        offsets->reconstruction_offset_samples +
        offsets->window_coordinate_offset_samples;
    return adc_cal_double_isfinite(existing_lag_samples) &&
        adc_cal_double_isfinite(offsets->reference_anchor_offset_samples) &&
        adc_cal_double_isfinite(offsets->resampling_delay_samples) &&
        adc_cal_double_isfinite(offsets->template_anchor_delay_samples) &&
        adc_cal_double_isfinite(offsets->reconstruction_offset_samples) &&
        adc_cal_double_isfinite(offsets->window_coordinate_offset_samples) &&
        adc_cal_double_isfinite(*expected_lag_samples) ?
        0 : ADC_CAL_DITHER_ERR_NUMERICAL;
}

int adc_cal_dither_dac_position_to_adc_position(
    double dac_position_samples,
    double dac_samples_per_adc_sample,
    double selected_dac_phase_samples,
    double interpolation_delay_dac_samples,
    double *adc_position_samples)
{
    if (adc_position_samples == NULL) return ADC_CAL_DITHER_ERR_NULL;
    *adc_position_samples = NAN;
    if (!adc_cal_double_isfinite(dac_position_samples) ||
        !adc_cal_double_isfinite(dac_samples_per_adc_sample) ||
        dac_samples_per_adc_sample <= DBL_EPSILON ||
        !adc_cal_double_isfinite(selected_dac_phase_samples) ||
        !adc_cal_double_isfinite(interpolation_delay_dac_samples)) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    /* Reference reconstruction samples raw DAC coordinate
     * adc_index * ratio + selected_phase + interpolation_delay. */
    *adc_position_samples =
        (dac_position_samples - selected_dac_phase_samples -
         interpolation_delay_dac_samples) /
        dac_samples_per_adc_sample;
    return adc_cal_double_isfinite(*adc_position_samples) ?
        0 : ADC_CAL_DITHER_ERR_NUMERICAL;
}

int adc_cal_dither_resample_dac_reference(
    const int16_t *raw_dac_samples,
    size_t raw_sample_count,
    double dac_samples_per_adc_sample,
    double dac_phase_samples,
    int16_t *adc_reference_samples,
    size_t adc_sample_count)
{
    if (raw_dac_samples == NULL || adc_reference_samples == NULL) {
        return ADC_CAL_DITHER_ERR_NULL;
    }
    if (raw_sample_count < 2U || adc_sample_count == 0U) {
        return ADC_CAL_DITHER_ERR_SAMPLE_COUNT;
    }
    if (!adc_cal_double_isfinite(dac_samples_per_adc_sample) ||
        dac_samples_per_adc_sample <= DBL_EPSILON ||
        !adc_cal_double_isfinite(dac_phase_samples)) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    for (size_t i = 0U; i < adc_sample_count; ++i) {
        double position = fmod(
            (double)i * dac_samples_per_adc_sample + dac_phase_samples,
            (double)raw_sample_count);
        size_t index0;
        size_t index1;
        double fraction;
        long interpolated;
        if (position < 0.0) position += (double)raw_sample_count;
        index0 = (size_t)floor(position);
        index1 = index0 + 1U;
        if (index1 >= raw_sample_count) index1 = 0U;
        fraction = position - (double)index0;
        interpolated = lround(
            (1.0 - fraction) * (double)raw_dac_samples[index0] +
            fraction * (double)raw_dac_samples[index1]);
        if (interpolated > INT16_MAX) interpolated = INT16_MAX;
        if (interpolated < INT16_MIN) interpolated = INT16_MIN;
        adc_reference_samples[i] = (int16_t)interpolated;
    }
    return 0;
}

int adc_cal_dither_rate_ratio_from_tone_bins(
    double nominal_dac_adc_ratio,
    double reference_tone_bin,
    double captured_tone_bin,
    double maximum_relative_adjustment,
    double *matched_dac_adc_ratio)
{
    double candidate;
    double relative_adjustment;
    if (matched_dac_adc_ratio == NULL) return ADC_CAL_DITHER_ERR_NULL;
    *matched_dac_adc_ratio = NAN;
    if (!adc_cal_double_isfinite(nominal_dac_adc_ratio) ||
        nominal_dac_adc_ratio <= DBL_EPSILON ||
        !adc_cal_double_isfinite(reference_tone_bin) ||
        reference_tone_bin <= DBL_EPSILON ||
        !adc_cal_double_isfinite(captured_tone_bin) ||
        captured_tone_bin <= DBL_EPSILON ||
        !adc_cal_double_isfinite(maximum_relative_adjustment) ||
        maximum_relative_adjustment < 0.0) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    candidate = nominal_dac_adc_ratio *
        captured_tone_bin / reference_tone_bin;
    relative_adjustment = dither_abs(
        candidate / nominal_dac_adc_ratio - 1.0);
    if (!adc_cal_double_isfinite(candidate) ||
        candidate <= DBL_EPSILON ||
        relative_adjustment > maximum_relative_adjustment) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    *matched_dac_adc_ratio = candidate;
    return 0;
}

int adc_cal_dither_reference_event_phase(
    double reference_event_position_samples,
    double reference_origin_samples,
    double event_spacing_samples,
    double *event_phase_samples)
{
    if (event_phase_samples == NULL) return ADC_CAL_DITHER_ERR_NULL;
    *event_phase_samples = dither_wrap_position(
        reference_event_position_samples - reference_origin_samples,
        event_spacing_samples);
    return adc_cal_double_isfinite(*event_phase_samples) ?
        0 : ADC_CAL_DITHER_ERR_NUMERICAL;
}

int adc_cal_dither_map_reference_position(
    double reference_position_samples,
    double fixed_window_start_samples,
    double lag_samples,
    double frame_period_samples,
    adc_cal_dither_coordinate_mapping_t *mapping)
{
    double wraps;
    if (mapping == NULL) return ADC_CAL_DITHER_ERR_NULL;
    memset(mapping, 0, sizeof(*mapping));
    mapping->reference_position_samples = NAN;
    mapping->window_relative_position_samples = NAN;
    mapping->capture_unwrapped_position_samples = NAN;
    mapping->capture_wrapped_position_samples = NAN;
    if (!adc_cal_double_isfinite(reference_position_samples) ||
        !adc_cal_double_isfinite(fixed_window_start_samples) ||
        !adc_cal_double_isfinite(lag_samples) ||
        !adc_cal_double_isfinite(frame_period_samples) ||
        frame_period_samples <= DBL_EPSILON) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    mapping->reference_position_samples = reference_position_samples;
    mapping->window_relative_position_samples =
        reference_position_samples - fixed_window_start_samples;
    mapping->capture_unwrapped_position_samples =
        reference_position_samples + lag_samples;
    mapping->capture_wrapped_position_samples = dither_wrap_position(
        mapping->capture_unwrapped_position_samples,
        frame_period_samples);
    wraps = floor(
        mapping->capture_unwrapped_position_samples /
        frame_period_samples);
    if (!adc_cal_double_isfinite(wraps) ||
        wraps > (double)INT32_MAX || wraps < (double)INT32_MIN) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    mapping->capture_frame_wraps = (int32_t)wraps;
    mapping->valid =
        adc_cal_double_isfinite(mapping->window_relative_position_samples) &&
        adc_cal_double_isfinite(mapping->capture_wrapped_position_samples);
    return mapping->valid ? 0 : ADC_CAL_DITHER_ERR_NUMERICAL;
}

int adc_cal_dither_build_tone_removed_residual(
    const double *capture_samples,
    const double *refined_tone_samples,
    size_t sample_count,
    double *dither_residual)
{
    if (capture_samples == NULL || refined_tone_samples == NULL ||
        dither_residual == NULL) {
        return ADC_CAL_DITHER_ERR_NULL;
    }
    if (sample_count == 0U) return ADC_CAL_DITHER_ERR_SAMPLE_COUNT;
    for (size_t i = 0U; i < sample_count; ++i) {
        if (!adc_cal_double_isfinite(capture_samples[i]) ||
            !adc_cal_double_isfinite(refined_tone_samples[i])) {
            return ADC_CAL_DITHER_ERR_NUMERICAL;
        }
        /* The synthesized dither is used to de-bias the tone fit only.
         * Subtracting it here would erase the signal being aligned. */
        dither_residual[i] =
            capture_samples[i] - refined_tone_samples[i];
    }
    return 0;
}

int adc_cal_dither_select_joint_candidate_pair(
    const adc_cal_dither_peak_candidate_t *channel_a_candidates,
    size_t channel_a_count,
    const adc_cal_dither_peak_candidate_t *channel_b_candidates,
    size_t channel_b_count,
    double event_spacing_samples,
    double frame_period_samples,
    const adc_cal_dither_joint_config_t *config,
    adc_cal_dither_joint_result_t *result)
{
    adc_cal_dither_joint_config_t local_config;
    double max_a = 0.0;
    double max_b = 0.0;
    int found = 0;

    if (result == NULL) return ADC_CAL_DITHER_ERR_NULL;
    memset(result, 0, sizeof(*result));
    result->joint_score = -DBL_MAX;
    result->channel_a_existing_residual_samples = NAN;
    result->channel_b_existing_residual_samples = NAN;
    result->consensus_origin_samples = NAN;
    result->consensus_lag_samples = NAN;
    if (channel_a_candidates == NULL || channel_b_candidates == NULL) {
        return ADC_CAL_DITHER_ERR_NULL;
    }
    if (channel_a_count == 0U || channel_b_count == 0U) {
        return ADC_CAL_DITHER_ERR_NO_EVENTS;
    }
    if (config == NULL) {
        adc_cal_dither_joint_default_config(&local_config);
        config = &local_config;
    }
    if (!adc_cal_double_isfinite(event_spacing_samples) ||
        event_spacing_samples <= DBL_EPSILON ||
        !adc_cal_double_isfinite(frame_period_samples) ||
        frame_period_samples <= DBL_EPSILON ||
        !adc_cal_double_isfinite(config->channel_tolerance_samples) ||
        config->channel_tolerance_samples < 0.0 ||
        !adc_cal_double_isfinite(config->existing_tolerance_samples) ||
        config->existing_tolerance_samples < 0.0 ||
        !adc_cal_double_isfinite(
            config->channel_residual_penalty_weight) ||
        config->channel_residual_penalty_weight < 0.0 ||
        !adc_cal_double_isfinite(
            config->existing_residual_penalty_weight) ||
        config->existing_residual_penalty_weight < 0.0 ||
        !adc_cal_double_isfinite(config->margin_weight) ||
        config->margin_weight < 0.0 ||
        !adc_cal_double_isfinite(config->confidence_weight) ||
        config->confidence_weight < 0.0 ||
        (config->use_expected_origin &&
         !adc_cal_double_isfinite(config->expected_origin_samples))) {
        return ADC_CAL_DITHER_ERR_NUMERICAL;
    }
    for (size_t i = 0U; i < channel_a_count; ++i) {
        if (channel_a_candidates[i].valid &&
            channel_a_candidates[i].absolute_peak > max_a) {
            max_a = channel_a_candidates[i].absolute_peak;
        }
    }
    for (size_t i = 0U; i < channel_b_count; ++i) {
        if (channel_b_candidates[i].valid &&
            channel_b_candidates[i].absolute_peak > max_b) {
            max_b = channel_b_candidates[i].absolute_peak;
        }
    }
    if (max_a <= DBL_EPSILON || max_b <= DBL_EPSILON) {
        return ADC_CAL_DITHER_ERR_NO_ENERGY;
    }

    for (size_t a = 0U; a < channel_a_count; ++a) {
        if (!channel_a_candidates[a].valid) continue;
        for (size_t b = 0U; b < channel_b_count; ++b) {
            adc_cal_dither_periodic_difference_t channel_difference;
            adc_cal_dither_periodic_difference_t existing_a_difference;
            adc_cal_dither_periodic_difference_t existing_b_difference;
            const int same_polarity =
                channel_a_candidates[a].peak *
                channel_b_candidates[b].peak >= 0.0;
            double existing_a_residual = 0.0;
            double existing_b_residual = 0.0;
            int existing_consistent = 1;
            double margin_a;
            double margin_b;
            double score;

            if (!channel_b_candidates[b].valid) continue;
            if ((config->polarity_policy == ADC_CAL_DITHER_POLARITY_SAME &&
                 !same_polarity) ||
                (config->polarity_policy ==
                     ADC_CAL_DITHER_POLARITY_INVERTED &&
                 same_polarity)) {
                continue;
            }
            if (adc_cal_dither_compare_periodic_origins(
                    channel_a_candidates[a].wrapped_origin_samples,
                    channel_b_candidates[b].wrapped_origin_samples,
                    event_spacing_samples, frame_period_samples,
                    &channel_difference) != 0 ||
                channel_difference.absolute_difference_samples >
                    config->channel_tolerance_samples) {
                continue;
            }
            if (config->use_expected_origin) {
                if (adc_cal_dither_compare_periodic_origins(
                        channel_a_candidates[a].wrapped_origin_samples,
                        config->expected_origin_samples,
                        event_spacing_samples, frame_period_samples,
                        &existing_a_difference) != 0 ||
                    adc_cal_dither_compare_periodic_origins(
                        channel_b_candidates[b].wrapped_origin_samples,
                        config->expected_origin_samples,
                        event_spacing_samples, frame_period_samples,
                        &existing_b_difference) != 0) {
                    continue;
                }
                existing_a_residual =
                    existing_a_difference.absolute_difference_samples;
                existing_b_residual =
                    existing_b_difference.absolute_difference_samples;
                existing_consistent =
                    existing_a_residual <=
                        config->existing_tolerance_samples &&
                    existing_b_residual <=
                        config->existing_tolerance_samples;
                /* Existing timing ranks and validates an otherwise valid A/B
                 * pair, but must not redefine channel consistency.  Return
                 * the best periodic A/B pair even when the independent
                 * existing estimator disagrees, then report that disagreement
                 * through existing_consistent and the residual fields. */
            }
            margin_a = channel_a_candidates[a].margin /
                (1.0 + dither_abs(channel_a_candidates[a].margin));
            margin_b = channel_b_candidates[b].margin /
                (1.0 + dither_abs(channel_b_candidates[b].margin));
            score =
                channel_a_candidates[a].absolute_peak / max_a +
                channel_b_candidates[b].absolute_peak / max_b +
                config->margin_weight * (margin_a + margin_b) -
                config->channel_residual_penalty_weight *
                    channel_difference.absolute_difference_samples /
                    (config->channel_tolerance_samples > DBL_EPSILON ?
                     config->channel_tolerance_samples : 1.0);
            if (channel_a_candidates[a].confidence ==
                    ADC_CAL_DITHER_CONFIDENCE_STRONG) {
                score += config->confidence_weight;
            }
            if (channel_b_candidates[b].confidence ==
                    ADC_CAL_DITHER_CONFIDENCE_STRONG) {
                score += config->confidence_weight;
            }
            if (config->use_expected_origin) {
                score -= config->existing_residual_penalty_weight *
                    0.5 * (existing_a_residual + existing_b_residual) /
                    (config->existing_tolerance_samples > DBL_EPSILON ?
                     config->existing_tolerance_samples : 1.0);
            }

            if (!found || score > result->joint_score ||
                (score == result->joint_score &&
                 channel_difference.absolute_difference_samples <
                     result->channel_difference.
                         absolute_difference_samples)) {
                found = 1;
                result->channel_a_candidate = a;
                result->channel_b_candidate = b;
                result->channel_difference = channel_difference;
                result->channel_a_existing_residual_samples =
                    config->use_expected_origin ?
                    existing_a_residual : NAN;
                result->channel_b_existing_residual_samples =
                    config->use_expected_origin ?
                    existing_b_residual : NAN;
                result->existing_consistent = existing_consistent;
                result->same_polarity = same_polarity;
                result->joint_score = score;
            }
        }
    }
    if (found) {
        const adc_cal_dither_peak_candidate_t *selected_a =
            &channel_a_candidates[result->channel_a_candidate];
        const adc_cal_dither_peak_candidate_t *selected_b =
            &channel_b_candidates[result->channel_b_candidate];
        const double weight_a = selected_a->absolute_peak;
        const double weight_b = selected_b->absolute_peak;
        const double equivalent_b_origin =
            selected_b->wrapped_origin_samples +
            (double)result->channel_difference.event_offset *
                event_spacing_samples -
            (double)result->channel_difference.frame_offset *
                frame_period_samples;
        result->consensus_origin_samples = dither_wrap_position(
            (weight_a * selected_a->wrapped_origin_samples +
             weight_b * equivalent_b_origin) / (weight_a + weight_b),
            frame_period_samples);
        result->consensus_lag_samples = -result->consensus_origin_samples;
        if (result->consensus_lag_samples <=
            -0.5 * frame_period_samples) {
            result->consensus_lag_samples += frame_period_samples;
        }
        if (!adc_cal_double_isfinite(result->consensus_origin_samples) ||
            !adc_cal_double_isfinite(result->consensus_lag_samples)) {
            return ADC_CAL_DITHER_ERR_NUMERICAL;
        }
    }
    result->valid = found;
    return found ? 0 : ADC_CAL_DITHER_ERR_NO_EVENTS;
}

void adc_cal_dither_result_reset(adc_cal_dither_result_t *result)
{
    if (result == NULL) return;
    memset(result, 0, sizeof(*result));
    result->valid = 0;
    result->status = ADC_CAL_DITHER_ERR_NUMERICAL;
    result->mean_polarity = NAN;
    result->separation_denominator = NAN;
    result->template_projection = NAN;
    result->template_energy = NAN;
    result->derivative_projection = NAN;
    result->derivative_energy = NAN;
    result->normalized_projection = NAN;
    result->quality = NAN;
    result->peak_abs_template = NAN;
}

int adc_cal_dither_interpolate(
    const double *samples,
    size_t count,
    double position,
    double *value)
{
    size_t lower;
    double fraction;
    if (samples == NULL || value == NULL || count == 0U ||
        !adc_cal_double_isfinite(position)) {
        return -1;
    }
    if (position < 0.0 || position > (double)(count - 1U)) {
        return -2;
    }
    lower = (size_t)floor(position);
    if (lower + 1U >= count) {
        *value = samples[count - 1U];
        return adc_cal_double_isfinite(*value) ? 0 : -3;
    }
    fraction = position - (double)lower;
    *value = (1.0 - fraction) * samples[lower] +
             fraction * samples[lower + 1U];
    return adc_cal_double_isfinite(*value) ? 0 : -3;
}

int adc_cal_dither_find_events(
    const double *template_samples,
    size_t sample_count,
    const adc_cal_dither_config_t *config,
    adc_cal_dither_result_t *result)
{
    adc_cal_dither_config_t local_config;
    double peak = 0.0;
    double threshold;
    size_t index = 0U;

    if (result == NULL) return ADC_CAL_DITHER_ERR_NULL;
    adc_cal_dither_result_reset(result);
    if (template_samples == NULL) {
        result->status = ADC_CAL_DITHER_ERR_NULL;
        return result->status;
    }
    if (sample_count == 0U || sample_count > ADC_CAL_DITHER_MAX_EVENTS * 4U) {
        result->status = ADC_CAL_DITHER_ERR_SAMPLE_COUNT;
        return result->status;
    }
    if (config == NULL) {
        adc_cal_dither_default_config(&local_config);
        config = &local_config;
    }
    result->mean_polarity = 0.0;
    if (!adc_cal_double_isfinite(config->threshold_fraction) ||
        config->threshold_fraction <= 0.0 ||
        config->threshold_fraction >= 1.0) {
        result->status = ADC_CAL_DITHER_ERR_NUMERICAL;
        return result->status;
    }
    for (size_t i = 0U; i < sample_count; ++i) {
        if (!adc_cal_double_isfinite(template_samples[i])) {
            result->status = ADC_CAL_DITHER_ERR_NUMERICAL;
            return result->status;
        }
        if (dither_abs(template_samples[i]) > peak) {
            peak = dither_abs(template_samples[i]);
        }
    }
    result->peak_abs_template = peak;
    if (peak <= DBL_EPSILON) {
        result->status = ADC_CAL_DITHER_ERR_NO_ENERGY;
        return result->status;
    }
    threshold = peak * config->threshold_fraction;
    while (index < sample_count) {
        size_t start;
        size_t end;
        double weighted = 0.0;
        double weight = 0.0;
        double signed_sum = 0.0;
        if (dither_abs(template_samples[index]) < threshold) {
            ++index;
            continue;
        }
        start = index;
        while (index < sample_count &&
               dither_abs(template_samples[index]) >= threshold) {
            const double a = dither_abs(template_samples[index]);
            weighted += (double)index * a;
            weight += a;
            signed_sum += template_samples[index];
            ++index;
        }
        end = index;
        ++result->detected_events;
        if (start < config->boundary_margin ||
            end + config->boundary_margin > sample_count ||
            result->accepted_events >= ADC_CAL_DITHER_MAX_EVENTS) {
            ++result->rejected_events;
            continue;
        }
        result->events[result->accepted_events].start = start;
        result->events[result->accepted_events].end = end;
        result->events[result->accepted_events].center =
            weight > DBL_EPSILON ? weighted / weight :
            0.5 * ((double)start + (double)(end - 1U));
        result->events[result->accepted_events].polarity =
            signed_sum >= 0.0 ? 1.0 : -1.0;
        result->mean_polarity +=
            result->events[result->accepted_events].polarity;
        ++result->accepted_events;
    }
    if (result->detected_events == 0U) {
        result->status = ADC_CAL_DITHER_ERR_NO_EVENTS;
        return result->status;
    }
    if (result->accepted_events < config->minimum_events) {
        result->status = ADC_CAL_DITHER_ERR_TOO_FEW_EVENTS;
        return result->status;
    }
    result->mean_polarity /= (double)result->accepted_events;
    result->separation_denominator =
        1.0 - result->mean_polarity * result->mean_polarity;
    if (!adc_cal_double_isfinite(result->separation_denominator) ||
        result->separation_denominator < ADC_CAL_DITHER_DENOMINATOR_FLOOR) {
        result->status = ADC_CAL_DITHER_ERR_POLARITY;
        return result->status;
    }
    result->status = ADC_CAL_DITHER_OK;
    return 0;
}

int adc_cal_dither_analyze(
    const double *samples,
    const double *template_samples,
    size_t sample_count,
    const adc_cal_dither_config_t *config,
    adc_cal_dither_result_t *result)
{
    double sample_mean = 0.0;
    double template_mean = 0.0;
    double sample_energy = 0.0;
    int status;

    if (result == NULL) return ADC_CAL_DITHER_ERR_NULL;
    status = adc_cal_dither_find_events(
        template_samples, sample_count, config, result);
    if (status != 0) return status;
    if (samples == NULL) {
        result->status = ADC_CAL_DITHER_ERR_NULL;
        return result->status;
    }
    result->template_projection = 0.0;
    result->template_energy = 0.0;
    result->derivative_projection = 0.0;
    result->derivative_energy = 0.0;
    result->normalized_projection = NAN;
    result->quality = NAN;
    for (size_t i = 0U; i < sample_count; ++i) {
        if (!adc_cal_double_isfinite(samples[i]) || !adc_cal_double_isfinite(template_samples[i])) {
            result->status = ADC_CAL_DITHER_ERR_NUMERICAL;
            return result->status;
        }
        sample_mean += samples[i];
        template_mean += template_samples[i];
    }
    sample_mean /= (double)sample_count;
    template_mean /= (double)sample_count;
    for (size_t i = 0U; i < sample_count; ++i) {
        const double s = samples[i] - sample_mean;
        const double t = template_samples[i] - template_mean;
        const double previous = i > 0U ? template_samples[i - 1U] :
            template_samples[i];
        const double next = i + 1U < sample_count ? template_samples[i + 1U] :
            template_samples[i];
        const double derivative = 0.5 * (next - previous);
        result->template_projection += s * t;
        result->template_energy += t * t;
        result->derivative_projection += s * derivative;
        result->derivative_energy += derivative * derivative;
        sample_energy += s * s;
    }
    if (result->template_energy <= DBL_EPSILON ||
        sample_energy <= DBL_EPSILON) {
        result->status = ADC_CAL_DITHER_ERR_NO_ENERGY;
        return result->status;
    }
    result->normalized_projection =
        result->template_projection / result->template_energy;
    result->quality = result->template_projection /
        sqrt(result->template_energy * sample_energy);
    result->valid = adc_cal_double_isfinite(result->normalized_projection) &&
        adc_cal_double_isfinite(result->quality);
    result->status = result->valid ? ADC_CAL_DITHER_OK :
        ADC_CAL_DITHER_ERR_NUMERICAL;
    return result->valid ? 0 : result->status;
}

int adc_cal_dither_polarize_template(
    const double *residual_a,
    size_t sample_count,
    const adc_cal_dither_result_t *template_events,
    const double *aligned_template,
    double *polarized_template)
{
    if (residual_a == NULL || template_events == NULL ||
        aligned_template == NULL || polarized_template == NULL) {
        return -1;
    }
    if (template_events->accepted_events == 0U) return -2;
    for (size_t k = 0U; k < template_events->accepted_events; ++k) {
        const size_t start = template_events->events[k].start;
        const size_t end = template_events->events[k].end;
        const double template_sign = template_events->events[k].polarity;
        double capture_sum = 0.0;
        double capture_sign;
        if (start >= end || end > sample_count) return -3;
        for (size_t i = start; i < end; ++i) {
            capture_sum += residual_a[i];
        }
        capture_sign = capture_sum >= 0.0 ? 1.0 : -1.0;
        for (size_t i = start; i < end; ++i) {
            polarized_template[i] = capture_sign * template_sign *
                aligned_template[i];
        }
    }
    return 0;
}
