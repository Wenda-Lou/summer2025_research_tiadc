#ifndef ADC_CALIBRATION_SKEW_H
#define ADC_CALIBRATION_SKEW_H

#include "adc_calibration_dither.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ADC_CAL_SKEW_MAX_PROFILE_SAMPLES
#define ADC_CAL_SKEW_MAX_PROFILE_SAMPLES 128U
#endif

#ifndef ADC_CAL_SKEW_MIN_EVENTS
#define ADC_CAL_SKEW_MIN_EVENTS 3U
#endif

#ifndef ADC_CAL_SKEW_MAX_LINEAR_SKEW_SAMPLES
#define ADC_CAL_SKEW_MAX_LINEAR_SKEW_SAMPLES 0.25
#endif

#ifndef ADC_CAL_SKEW_MAX_EDGE_DISAGREEMENT_SAMPLES
#define ADC_CAL_SKEW_MAX_EDGE_DISAGREEMENT_SAMPLES 0.03
#endif

#ifndef ADC_CAL_SKEW_DERIVATIVE_ENERGY_FLOOR
#define ADC_CAL_SKEW_DERIVATIVE_ENERGY_FLOOR 1.0e-6
#endif

typedef enum {
    ADC_CAL_SKEW_STATUS_INVALID = 0,
    ADC_CAL_SKEW_STATUS_PASS,
    ADC_CAL_SKEW_STATUS_WARNING
} adc_cal_skew_status_t;

typedef enum {
    ADC_CAL_SKEW_REASON_NONE = 0,
    ADC_CAL_SKEW_REASON_CONTEXT,
    ADC_CAL_SKEW_REASON_DITHER,
    ADC_CAL_SKEW_REASON_TOO_FEW_EVENTS,
    ADC_CAL_SKEW_REASON_POLARITY,
    ADC_CAL_SKEW_REASON_TEMPLATE,
    ADC_CAL_SKEW_REASON_DERIVATIVE,
    ADC_CAL_SKEW_REASON_EDGE_DISAGREEMENT,
    ADC_CAL_SKEW_REASON_OUTSIDE_LINEAR_RANGE,
    ADC_CAL_SKEW_REASON_NUMERICAL
} adc_cal_skew_reason_t;

typedef enum {
    ADC_CAL_SKEW_TONE_CONTEXT_VALID = 0,
    ADC_CAL_SKEW_TONE_CONTEXT_INVALID_SAMPLE_RATE,
    ADC_CAL_SKEW_TONE_CONTEXT_INVALID_TONE_FREQUENCY,
    ADC_CAL_SKEW_TONE_CONTEXT_FREQUENCY_MISMATCH
} adc_cal_skew_tone_context_status_t;

typedef enum {
    ADC_CAL_SKEW_POLARITY_UNKNOWN = 0,
    ADC_CAL_SKEW_POLARITY_SAME = 1,
    ADC_CAL_SKEW_POLARITY_INVERTED = -1
} adc_cal_skew_polarity_t;

typedef enum {
    ADC_CAL_SKEW_BRANCH_REASON_NONE = 0,
    ADC_CAL_SKEW_BRANCH_REASON_KNOWN_POLARITY,
    ADC_CAL_SKEW_BRANCH_REASON_DITHER_AGREEMENT,
    ADC_CAL_SKEW_BRANCH_REASON_FRAME_CONSISTENCY,
    ADC_CAL_SKEW_BRANCH_REASON_PHYSICAL_BOUND,
    ADC_CAL_SKEW_BRANCH_REASON_MINIMUM_ABSOLUTE_SKEW
} adc_cal_skew_branch_reason_t;

#define ADC_CAL_SKEW_PHASE_HYPOTHESES 3U

typedef struct {
    double max_abs_skew_samples;
    adc_cal_skew_polarity_t known_polarity;
    int dither_valid;
    double dither_skew_samples;
    int previous_valid;
    double previous_skew_samples;
} adc_cal_skew_phase_config_t;

typedef struct {
    int valid;
    double raw_phase_difference_rad;
    double raw_skew_samples;
    double candidate_phase_adjustment_rad[ADC_CAL_SKEW_PHASE_HYPOTHESES];
    double candidate_phase_difference_rad[ADC_CAL_SKEW_PHASE_HYPOTHESES];
    double candidate_skew_samples[ADC_CAL_SKEW_PHASE_HYPOTHESES];
    int candidate_within_physical_range[ADC_CAL_SKEW_PHASE_HYPOTHESES];
    size_t selected_candidate;
    adc_cal_skew_polarity_t selected_polarity;
    adc_cal_skew_branch_reason_t selection_reason;
    double applied_phase_adjustment_rad;
    double corrected_phase_difference_rad;
    double corrected_skew_samples;
    double dither_disagreement_samples;
} adc_cal_skew_phase_result_t;

typedef enum {
    ADC_CAL_SKEW_MEASUREMENT_INVALID = 0,
    ADC_CAL_SKEW_MEASUREMENT_VALID
} adc_cal_skew_measurement_validity_t;

typedef enum {
    ADC_CAL_SKEW_STABILITY_UNKNOWN = 0,
    ADC_CAL_SKEW_STABILITY_UNSTABLE,
    ADC_CAL_SKEW_STABILITY_STABLE
} adc_cal_skew_stability_t;

typedef enum {
    ADC_CAL_SKEW_TOLERANCE_UNKNOWN = 0,
    ADC_CAL_SKEW_TOLERANCE_IN,
    ADC_CAL_SKEW_TOLERANCE_OUT
} adc_cal_skew_tolerance_status_t;

typedef enum {
    ADC_CAL_SKEW_ACTUATOR_UNAVAILABLE = 0,
    ADC_CAL_SKEW_ACTUATOR_AVAILABLE
} adc_cal_skew_actuator_status_t;

typedef enum {
    ADC_CAL_SKEW_CORRECTION_NOT_APPLICABLE = 0,
    ADC_CAL_SKEW_CORRECTION_CONVERGED,
    ADC_CAL_SKEW_CORRECTION_NOT_CONVERGED,
    ADC_CAL_SKEW_CORRECTION_SATURATED
} adc_cal_skew_correction_status_t;

typedef enum {
    ADC_CAL_SKEW_STAGE_RESULT_INVALID = 0,
    ADC_CAL_SKEW_STAGE_RESULT_UNSTABLE,
    ADC_CAL_SKEW_STAGE_RESULT_PASS,
    ADC_CAL_SKEW_STAGE_RESULT_PASS_WITH_WARNING,
    ADC_CAL_SKEW_STAGE_RESULT_CORRECTION_NOT_CONVERGED,
    ADC_CAL_SKEW_STAGE_RESULT_CORRECTION_SATURATED
} adc_cal_skew_stage_result_t;

typedef struct {
    int measurement_required;
    int primary_estimate_valid;
    double measured_skew_samples;
    uint32_t accepted_frames;
    uint32_t minimum_accepted_frames;
    double batch_std_samples;
    double maximum_batch_std_samples;
    uint32_t polarity_branch_changes;
    double tolerance_samples;
    int advisory_warning;
    int actuator_available;
    int correction_applied;
    int correction_converged;
    int actuator_saturated;
} adc_cal_skew_stage_policy_input_t;

typedef struct {
    adc_cal_skew_measurement_validity_t measurement_validity;
    adc_cal_skew_stability_t stability;
    adc_cal_skew_tolerance_status_t tolerance_status;
    adc_cal_skew_actuator_status_t actuator_status;
    adc_cal_skew_correction_status_t correction_status;
    adc_cal_skew_stage_result_t stage_result;
    int pipeline_may_continue;
    int output_usable;
    const char *reason;
} adc_cal_skew_stage_policy_result_t;

typedef enum {
    ADC_CAL_SKEW_LOOP_MEASUREMENT_ONLY = 0,
    ADC_CAL_SKEW_LOOP_CONVERGED,
    ADC_CAL_SKEW_LOOP_NOT_CONVERGED,
    ADC_CAL_SKEW_LOOP_SATURATED,
    ADC_CAL_SKEW_LOOP_ACTUATOR_UNAVAILABLE,
    ADC_CAL_SKEW_LOOP_FAILED
} adc_cal_skew_loop_status_t;

/* A positive polarity means that increasing the Channel-B register increases
 * measured B-A skew.  A negative polarity means it decreases measured skew. */
typedef struct {
    int skew_closed_loop_enable;
    double skew_tolerance_samples;
    uint32_t skew_required_consecutive_passes;
    uint32_t skew_max_iterations;
    double skew_controller_gain;
    int skew_max_steps_per_iteration;
    int skew_register_min;
    int skew_register_max;
    double skew_actuator_step_samples;
    int skew_actuator_polarity;
    uint32_t skew_minimum_accepted_frames;
    double skew_maximum_batch_std_samples;
    double skew_characterization_step_tolerance_fraction;
} adc_cal_skew_loop_config_t;

typedef struct {
    int valid;
    double skew_samples;
    double batch_std_samples;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    const char *reason;
} adc_cal_skew_batch_measurement_t;

typedef int (*adc_cal_skew_measure_batch_fn)(
    void *context, adc_cal_skew_batch_measurement_t *measurement);
typedef int (*adc_cal_skew_actuator_prepare_fn)(void *context);
typedef int (*adc_cal_skew_register_read_fn)(void *context, int *value);
typedef int (*adc_cal_skew_register_write_fn)(void *context, int value);
typedef void (*adc_cal_skew_iteration_report_fn)(
    void *context,
    uint32_t iteration,
    const adc_cal_skew_batch_measurement_t *measurement,
    int old_register,
    int new_register,
    int requested_steps,
    int applied_steps,
    double actuator_step_samples,
    double best_skew_samples,
    int saturated,
    uint32_t consecutive_passes,
    int converged);

typedef struct {
    void *context;
    adc_cal_skew_measure_batch_fn measure_batch;
    adc_cal_skew_actuator_prepare_fn prepare_actuator;
    adc_cal_skew_register_read_fn read_register;
    adc_cal_skew_register_write_fn write_register;
    adc_cal_skew_iteration_report_fn report_iteration;
} adc_cal_skew_loop_io_t;

typedef struct {
    adc_cal_skew_loop_status_t status;
    int characterization_valid;
    double observed_step_samples;
    double observed_step_ps;
    int actuator_polarity;
    double actuator_step_samples;
    double initial_skew_samples;
    double final_skew_samples;
    double best_skew_samples;
    double final_batch_std_samples;
    int initial_register;
    int final_register;
    int total_register_change;
    uint32_t iterations_completed;
    uint32_t consecutive_passes;
    uint32_t accepted_frames;
    uint32_t rejected_frames;
    int correction_applied;
    int saturated;
    const char *failure_reason;
} adc_cal_skew_loop_result_t;

typedef struct {
    size_t minimum_events;
    double sample_rate_hz;
    double max_linear_skew_samples;
    double max_edge_disagreement_samples;
} adc_cal_skew_config_t;

typedef struct {
    int valid;
    adc_cal_skew_status_t status;
    adc_cal_skew_reason_t reason;
    double channel_a_skew_samples;
    double channel_b_skew_samples;
    double relative_skew_samples;
    double relative_skew_ps;
    double rising_skew_samples;
    double falling_skew_samples;
    double edge_disagreement_samples;
    double pulse_energy;
    double derivative_energy;
    double quality;
    uint32_t accepted_events;
    uint32_t rejected_events;
    const char *failure_reason;
} adc_cal_skew_result_t;

void adc_cal_skew_default_config(adc_cal_skew_config_t *config);
void adc_cal_skew_result_reset(adc_cal_skew_result_t *result);
const char *adc_cal_skew_status_name(adc_cal_skew_status_t status);
const char *adc_cal_skew_reason_name(adc_cal_skew_reason_t reason);
const char *adc_cal_skew_tone_context_status_name(
    adc_cal_skew_tone_context_status_t status);
adc_cal_skew_tone_context_status_t adc_cal_skew_validate_tone_context(
    double inherited_tone_frequency_hz,
    double fitted_tone_frequency_hz,
    double sample_rate_hz,
    size_t sample_count,
    double maximum_error_bins);
const char *adc_cal_skew_polarity_name(adc_cal_skew_polarity_t polarity);
const char *adc_cal_skew_branch_reason_name(
    adc_cal_skew_branch_reason_t reason);

void adc_cal_skew_phase_default_config(adc_cal_skew_phase_config_t *config);
int adc_cal_skew_resolve_tone_phase(
    double channel_a_phase_rad,
    double channel_b_phase_rad,
    double tone_frequency_hz,
    double sample_rate_hz,
    const adc_cal_skew_phase_config_t *config,
    adc_cal_skew_phase_result_t *result);

const char *adc_cal_skew_measurement_validity_name(
    adc_cal_skew_measurement_validity_t status);
const char *adc_cal_skew_stability_name(adc_cal_skew_stability_t status);
const char *adc_cal_skew_tolerance_status_name(
    adc_cal_skew_tolerance_status_t status);
const char *adc_cal_skew_actuator_status_name(
    adc_cal_skew_actuator_status_t status);
const char *adc_cal_skew_correction_status_name(
    adc_cal_skew_correction_status_t status);
const char *adc_cal_skew_stage_result_name(
    adc_cal_skew_stage_result_t status);
int adc_cal_skew_evaluate_stage_policy(
    const adc_cal_skew_stage_policy_input_t *input,
    adc_cal_skew_stage_policy_result_t *result);

void adc_cal_skew_loop_default_config(adc_cal_skew_loop_config_t *config);
const char *adc_cal_skew_loop_status_name(adc_cal_skew_loop_status_t status);
int adc_cal_skew_plan_update(
    double measured_skew_samples,
    int current_register,
    const adc_cal_skew_loop_config_t *config,
    int *requested_steps,
    int *applied_steps,
    int *new_register,
    int *saturated);
int adc_cal_skew_run_closed_loop(
    const adc_cal_skew_loop_config_t *config,
    const adc_cal_skew_loop_io_t *io,
    double sample_rate_hz,
    adc_cal_skew_loop_result_t *result);

int adc_cal_skew_from_tone_phases(
    double channel_a_phase_rad,
    double channel_b_phase_rad,
    double tone_frequency_hz,
    double sample_rate_hz,
    double *relative_skew_samples);

/* Returns 1 only when the dither estimator passed all of its own checks and
 * agrees with the independent primary estimate. Returns 0 for a finite but
 * warning/discordant dither estimate and a negative value for invalid input. */
int adc_cal_skew_dither_crosscheck_is_usable(
    const adc_cal_skew_result_t *dither_result,
    double primary_skew_samples,
    double maximum_disagreement_samples,
    double *disagreement_samples);

/* Map a single circular coordinate window onto a same-frame physical A/B
 * pair.  Both channels always use identical source indices and interpolation
 * weights, preserving their relative phase. */
int adc_cal_skew_map_paired_window_i16(
    const int16_t *channel_a,
    const int16_t *channel_b,
    size_t frame_sample_count,
    size_t window_start,
    size_t window_length,
    double common_phase_offset_samples,
    double common_lag_samples,
    double *mapped_a,
    double *mapped_b);

int adc_cal_skew_estimate_from_residuals(
    const double *channel_a_residual,
    const double *channel_b_residual,
    const double *dither_template,
    size_t sample_count,
    const adc_cal_skew_config_t *config,
    adc_cal_skew_result_t *result);

#ifdef __cplusplus
}
#endif

#endif /* ADC_CALIBRATION_SKEW_H */
