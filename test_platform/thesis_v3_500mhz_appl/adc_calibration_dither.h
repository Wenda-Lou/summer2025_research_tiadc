#ifndef ADC_CALIBRATION_DITHER_H
#define ADC_CALIBRATION_DITHER_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ADC_CAL_DITHER_MAX_EVENTS
#define ADC_CAL_DITHER_MAX_EVENTS 512U
#endif

#ifndef ADC_CAL_DITHER_DEFAULT_THRESHOLD_FRACTION
#define ADC_CAL_DITHER_DEFAULT_THRESHOLD_FRACTION 0.25
#endif

#ifndef ADC_CAL_DITHER_DEFAULT_MIN_EVENTS
#define ADC_CAL_DITHER_DEFAULT_MIN_EVENTS 2U
#endif

#ifndef ADC_CAL_DITHER_DENOMINATOR_FLOOR
#define ADC_CAL_DITHER_DENOMINATOR_FLOOR 0.05
#endif

#ifndef ADC_CAL_DITHER_DEFAULT_PEAK_GUARD_SAMPLES
#define ADC_CAL_DITHER_DEFAULT_PEAK_GUARD_SAMPLES 8U
#endif

#ifndef ADC_CAL_DITHER_DEFAULT_PERIODIC_EXCLUSION_WIDTH
#define ADC_CAL_DITHER_DEFAULT_PERIODIC_EXCLUSION_WIDTH 8.0
#endif

#ifndef ADC_CAL_DITHER_DEFAULT_WEAK_PEAK_RATIO
#define ADC_CAL_DITHER_DEFAULT_WEAK_PEAK_RATIO 1.10
#endif

#ifndef ADC_CAL_DITHER_DEFAULT_STRONG_PEAK_RATIO
#define ADC_CAL_DITHER_DEFAULT_STRONG_PEAK_RATIO 1.25
#endif

#ifndef ADC_CAL_DITHER_DEFAULT_CANDIDATE_COUNT
#define ADC_CAL_DITHER_DEFAULT_CANDIDATE_COUNT 8U
#endif

typedef enum {
    ADC_CAL_DITHER_OK = 0,
    ADC_CAL_DITHER_ERR_NULL = -1,
    ADC_CAL_DITHER_ERR_SAMPLE_COUNT = -2,
    ADC_CAL_DITHER_ERR_NO_ENERGY = -3,
    ADC_CAL_DITHER_ERR_NO_EVENTS = -4,
    ADC_CAL_DITHER_ERR_TOO_FEW_EVENTS = -5,
    ADC_CAL_DITHER_ERR_POLARITY = -6,
    ADC_CAL_DITHER_ERR_NUMERICAL = -7
} adc_cal_dither_status_t;

typedef struct {
    size_t start;
    size_t end;
    double center;
    double polarity;
} adc_cal_dither_event_t;

typedef struct {
    double threshold_fraction;
    size_t minimum_events;
    size_t boundary_margin;
} adc_cal_dither_config_t;

typedef struct {
    int valid;
    adc_cal_dither_status_t status;
    size_t detected_events;
    size_t accepted_events;
    size_t rejected_events;
    double mean_polarity;
    double separation_denominator;
    double template_projection;
    double template_energy;
    double derivative_projection;
    double derivative_energy;
    double normalized_projection;
    double quality;
    double peak_abs_template;
    adc_cal_dither_event_t events[ADC_CAL_DITHER_MAX_EVENTS];
} adc_cal_dither_result_t;

typedef enum {
    ADC_CAL_DITHER_CONFIDENCE_INVALID = 0,
    ADC_CAL_DITHER_CONFIDENCE_WEAK,
    ADC_CAL_DITHER_CONFIDENCE_STRONG
} adc_cal_dither_confidence_t;

typedef enum {
    ADC_CAL_DITHER_RECOMMEND_REJECT = 0,
    ADC_CAL_DITHER_RECOMMEND_ACCEPT_WITH_WARNING,
    ADC_CAL_DITHER_RECOMMEND_ACCEPT
} adc_cal_dither_recommendation_t;

typedef enum {
    ADC_CAL_DITHER_VALIDATION_NONE = 0,
    ADC_CAL_DITHER_VALIDATION_SELECTED_CHANNEL,
    ADC_CAL_DITHER_VALIDATION_CHANNEL_AVAILABILITY,
    ADC_CAL_DITHER_VALIDATION_NO_CONSISTENT_PAIR,
    ADC_CAL_DITHER_VALIDATION_CHANNEL_DISAGREEMENT,
    ADC_CAL_DITHER_VALIDATION_EXISTING_DISAGREEMENT,
    ADC_CAL_DITHER_VALIDATION_TOO_FEW_EVENTS,
    ADC_CAL_DITHER_VALIDATION_EVENT_INDICES,
    ADC_CAL_DITHER_VALIDATION_NUMERICAL,
    ADC_CAL_DITHER_VALIDATION_PEAK_BELOW_WEAK,
    ADC_CAL_DITHER_VALIDATION_PEAK_BELOW_STRONG
} adc_cal_dither_validation_reason_t;

typedef struct {
    size_t local_exclusion_samples;
    double periodic_exclusion_width_samples;
    size_t maximum_candidates;
} adc_cal_dither_peak_config_t;

typedef struct {
    int valid;
    size_t best_index;
    size_t raw_second_index;
    size_t independent_second_index;
    double best_peak;
    double raw_second_peak;
    double independent_second_peak;
    double raw_peak_ratio;
    double independent_peak_ratio;
    int periodic_exclusion_applied;
} adc_cal_dither_peak_result_t;

typedef struct {
    size_t minimum_complete_events;
    double channel_disagreement_tolerance_samples;
    double existing_disagreement_tolerance_samples;
    double weak_peak_ratio;
    double strong_peak_ratio;
} adc_cal_dither_validation_config_t;

typedef struct {
    int selected_channel_valid;
    int channel_a_available;
    int channel_b_available;
    int channel_a_valid;
    int channel_b_valid;
    int joint_pair_required;
    int joint_pair_valid;
    int existing_comparison_required;
    double channel_disagreement_samples;
    double existing_disagreement_samples;
    size_t complete_event_count;
    int event_indices_valid;
    int numerical_values_finite;
    double independent_peak_ratio;
} adc_cal_dither_validation_input_t;

typedef struct {
    int structural_valid;
    adc_cal_dither_confidence_t confidence;
    adc_cal_dither_recommendation_t recommendation;
    adc_cal_dither_validation_reason_t reason;
} adc_cal_dither_validation_result_t;

typedef struct {
    int valid;
    size_t complete_count;
    size_t partial_count;
    size_t total_count;
    double spacing_samples;
    double first_complete_center;
    double last_complete_center;
} adc_cal_dither_event_summary_t;

typedef struct {
    int valid;
    double raw_difference_samples;
    int32_t frame_offset;
    int32_t event_offset;
    double signed_difference_samples;
    double absolute_difference_samples;
} adc_cal_dither_periodic_difference_t;

typedef struct {
    int valid;
    size_t coarse_index;
    double coarse_fractional_offset_samples;
    double coarse_lag_samples;
    size_t index;
    /* lag aligns template[i] with capture[i + lag]. */
    double fractional_offset_samples;
    double lag_samples;
    /* wrapped_origin = (-lag) mod frame_period. */
    double wrapped_origin_samples;
    double peak;
    double absolute_peak;
    size_t global_strongest_index;
    double global_strongest_peak;
    double global_strongest_absolute_peak;
    double raw_second_peak;
    double independent_second_peak;
    double raw_peak_ratio;
    double independent_peak_ratio;
    double margin;
    adc_cal_dither_confidence_t confidence;
} adc_cal_dither_peak_candidate_t;

typedef enum {
    ADC_CAL_DITHER_POLARITY_UNCONSTRAINED = 0,
    ADC_CAL_DITHER_POLARITY_SAME,
    ADC_CAL_DITHER_POLARITY_INVERTED
} adc_cal_dither_polarity_policy_t;

typedef struct {
    double channel_tolerance_samples;
    double existing_tolerance_samples;
    double channel_residual_penalty_weight;
    double existing_residual_penalty_weight;
    double margin_weight;
    double confidence_weight;
    adc_cal_dither_polarity_policy_t polarity_policy;
    int use_expected_origin;
    double expected_origin_samples;
} adc_cal_dither_joint_config_t;

typedef struct {
    int valid;
    size_t channel_a_candidate;
    size_t channel_b_candidate;
    adc_cal_dither_periodic_difference_t channel_difference;
    double channel_a_existing_residual_samples;
    double channel_b_existing_residual_samples;
    int existing_consistent;
    int same_polarity;
    double joint_score;
    double consensus_origin_samples;
    double consensus_lag_samples;
} adc_cal_dither_joint_result_t;

typedef struct {
    int valid;
    /* Full selected-reference coordinate, before fixed-window subtraction. */
    double reference_position_samples;
    double window_relative_position_samples;
    /* capture_unwrapped = reference_position + lag. */
    double capture_unwrapped_position_samples;
    double capture_wrapped_position_samples;
    int32_t capture_frame_wraps;
} adc_cal_dither_coordinate_mapping_t;

typedef enum {
    /* Preserve the independently randomized +/- event polarities. */
    ADC_CAL_DITHER_CORRELATION_SIGNED = 0,
    /* Correlate centered sample energy so each event polarity is immaterial. */
    ADC_CAL_DITHER_CORRELATION_ENERGY,
    /* Correlate centered squared interior derivatives.  Frame endpoints are
     * zeroed so a truncated nonperiodic DAC segment cannot create a synthetic
     * wrap edge.  This emphasizes the raised-cosine timing edges. */
    ADC_CAL_DITHER_CORRELATION_EDGE_ENERGY
} adc_cal_dither_correlation_mode_t;

#define ADC_CAL_DITHER_DEFAULT_ENERGY_SCORE_WEIGHT 0.35
#define ADC_CAL_DITHER_DEFAULT_EDGE_SCORE_WEIGHT   0.65
#define ADC_CAL_DITHER_DEFAULT_EDGE_REFINE_RADIUS  2U

typedef struct {
    /* All terms are full-frame ADC samples and are added to existing_lag. */
    double reference_anchor_offset_samples;
    double resampling_delay_samples;
    double template_anchor_delay_samples;
    double reconstruction_offset_samples;
    double window_coordinate_offset_samples;
} adc_cal_dither_lag_offsets_t;

void adc_cal_dither_default_config(adc_cal_dither_config_t *config);
void adc_cal_dither_peak_default_config(adc_cal_dither_peak_config_t *config);
void adc_cal_dither_validation_default_config(
    adc_cal_dither_validation_config_t *config);
const char *adc_cal_dither_status_name(adc_cal_dither_status_t status);
const char *adc_cal_dither_confidence_name(
    adc_cal_dither_confidence_t confidence);
const char *adc_cal_dither_recommendation_name(
    adc_cal_dither_recommendation_t recommendation);
const char *adc_cal_dither_validation_reason_name(
    adc_cal_dither_validation_reason_t reason);
void adc_cal_dither_result_reset(adc_cal_dither_result_t *result);

int adc_cal_dither_select_independent_peaks(
    const double *scores,
    size_t score_count,
    double event_spacing_samples,
    const adc_cal_dither_peak_config_t *config,
    adc_cal_dither_peak_result_t *result);

int adc_cal_dither_find_peak_candidates(
    const double *scores,
    size_t score_count,
    double event_spacing_samples,
    const adc_cal_dither_peak_config_t *peak_config,
    const adc_cal_dither_validation_config_t *confidence_config,
    adc_cal_dither_peak_candidate_t *candidates,
    size_t candidate_capacity,
    size_t *candidate_count);

/* Preserve coarse candidate ranking and locally replace only each candidate's
 * integer/fractional lag with the closest smoothed-edge maximum. */
int adc_cal_dither_refine_candidate_lags(
    const double *edge_scores,
    size_t score_count,
    size_t search_radius,
    adc_cal_dither_peak_candidate_t *candidates,
    size_t candidate_count);

void adc_cal_dither_joint_default_config(
    adc_cal_dither_joint_config_t *config);

int adc_cal_dither_select_joint_candidate_pair(
    const adc_cal_dither_peak_candidate_t *channel_a_candidates,
    size_t channel_a_count,
    const adc_cal_dither_peak_candidate_t *channel_b_candidates,
    size_t channel_b_count,
    double event_spacing_samples,
    double frame_period_samples,
    const adc_cal_dither_joint_config_t *config,
    adc_cal_dither_joint_result_t *result);

int adc_cal_dither_validate_detection(
    const adc_cal_dither_validation_input_t *input,
    const adc_cal_dither_validation_config_t *config,
    adc_cal_dither_validation_result_t *result);

int adc_cal_dither_summarize_events(
    const double *template_samples,
    size_t sample_count,
    size_t window_start,
    size_t window_length,
    double threshold_fraction,
    adc_cal_dither_event_summary_t *summary);

/* Minimize |a + m*frame_period - (b + k*event_spacing)| for canonical
 * origins in [0, frame_period). A positive frame_period checks only the two
 * equivalent circular representations (m = 0 and the adjacent-frame lift),
 * not arbitrary frame multiples. Zero limits the comparison to the requested
 * |a - (b + k*event_spacing)| form. */
int adc_cal_dither_compare_periodic_origins(
    double first_origin_samples,
    double second_origin_samples,
    double event_spacing_samples,
    double frame_period_samples,
    adc_cal_dither_periodic_difference_t *difference);

/* Lift a high-precision but sine-cycle-ambiguous lag by an integer number of
 * tone periods so that it agrees with the detected periodic impulse family. */
int adc_cal_dither_resolve_tone_cycle(
    double ambiguous_lag_samples,
    double tone_period_samples,
    double dither_origin_samples,
    double event_spacing_samples,
    double frame_period_samples,
    double maximum_residual_samples,
    double *resolved_lag_samples,
    int32_t *tone_cycle_offset,
    adc_cal_dither_periodic_difference_t *residual);

int adc_cal_dither_lag_to_wrapped_origin(
    double lag_samples,
    double frame_period_samples,
    double *wrapped_origin_samples);

/* scores[lag] aligns template[i] with capture[i + lag].  ENERGY mode is
 * invariant to the independent random +/- polarity assigned to each event. */
int adc_cal_dither_compute_circular_scores(
    const double *template_samples,
    const double *capture_samples,
    size_t sample_count,
    adc_cal_dither_correlation_mode_t mode,
    double *scores);

/* Blend the robust pulse-energy score with the sharper edge-energy score.
 * Weights must be finite, nonnegative, and have a positive sum. */
int adc_cal_dither_compute_timing_scores(
    const double *template_samples,
    const double *capture_samples,
    size_t sample_count,
    double energy_weight,
    double edge_weight,
    double *scores);

/* Measure the detector's correlation-index anchor against the template
 * itself.  Direct circular cross-correlation has zero convolution delay. */
int adc_cal_dither_template_anchor_delay(
    const double *template_samples,
    size_t sample_count,
    adc_cal_dither_correlation_mode_t mode,
    double *delay_samples);

/* expected = existing + reference_anchor + resampling + template_anchor
 *                    + reconstruction + window_coordinate. */
int adc_cal_dither_expected_lag(
    double existing_lag_samples,
    const adc_cal_dither_lag_offsets_t *offsets,
    double *expected_lag_samples);

int adc_cal_dither_dac_position_to_adc_position(
    double dac_position_samples,
    double dac_samples_per_adc_sample,
    double selected_dac_phase_samples,
    double interpolation_delay_dac_samples,
    double *adc_position_samples);

/* Reconstruct one ADC-rate reference phase from a periodic raw DAC waveform
 * using the same circular linear interpolation used by the target command. */
int adc_cal_dither_resample_dac_reference(
    const int16_t *raw_dac_samples,
    size_t raw_sample_count,
    double dac_samples_per_adc_sample,
    double dac_phase_samples,
    int16_t *adc_reference_samples,
    size_t adc_sample_count);

int adc_cal_dither_reference_event_phase(
    double reference_event_position_samples,
    double reference_origin_samples,
    double event_spacing_samples,
    double *event_phase_samples);

int adc_cal_dither_map_reference_position(
    double reference_position_samples,
    double fixed_window_start_samples,
    double lag_samples,
    double frame_period_samples,
    adc_cal_dither_coordinate_mapping_t *mapping);

int adc_cal_dither_build_tone_removed_residual(
    const double *capture_samples,
    const double *refined_tone_samples,
    size_t sample_count,
    double *dither_residual);

int adc_cal_dither_find_events(
    const double *template_samples,
    size_t sample_count,
    const adc_cal_dither_config_t *config,
    adc_cal_dither_result_t *result);

int adc_cal_dither_analyze(
    const double *samples,
    const double *template_samples,
    size_t sample_count,
    const adc_cal_dither_config_t *config,
    adc_cal_dither_result_t *result);

int adc_cal_dither_interpolate(
    const double *samples,
    size_t count,
    double position,
    double *value);

#ifdef __cplusplus
}
#endif

#endif /* ADC_CALIBRATION_DITHER_H */
