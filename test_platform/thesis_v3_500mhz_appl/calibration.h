#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * calibration.h
 *
 * Pure calibration algorithm module.
 *
 * This module deliberately contains no DMA, UART, SPI, Ethernet, or AXI-register
 * access. The existing project modules remain responsible for acquisition and
 * hardware control. The caller supplies one aligned ADC frame and its matching
 * DAC reference frame.
 */

typedef enum {
    CALIBRATION_OK = 0,
    CALIBRATION_ERR_NULL = -1,
    CALIBRATION_ERR_SAMPLE_COUNT = -2,
    CALIBRATION_ERR_ZERO_REFERENCE_POWER = -3,
    CALIBRATION_ERR_INVALID_CONFIG = -4
} calibration_status_t;

typedef enum {
    CALIBRATION_STAGE_OFFSET = 0,
    CALIBRATION_STAGE_GAIN,
    CALIBRATION_STAGE_COMPLETE,
    CALIBRATION_STAGE_FAILED
} calibration_stage_t;

#define CALIBRATION_OFFSET_MAX_ACCEPTED_ITERATIONS     20U
#define CALIBRATION_OFFSET_MAX_REJECTED_FRAMES         10U
#define CALIBRATION_OFFSET_TOLERANCE_CODES             1.0f
#define CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES   2U
#define CALIBRATION_OFFSET_BATCH_SIZE                  30U
#define CALIBRATION_OFFSET_UPDATE_STEP                 0.25f
#define CALIBRATION_OFFSET_NEAR_UPDATE_STEP            0.15f
#define CALIBRATION_OFFSET_NEAR_RESIDUAL_CODES         5.0f
#define CALIBRATION_OFFSET_FILTER_ALPHA                0.8f
#define CALIBRATION_OFFSET_ENTER_TOLERANCE_CODES       1.0f
#define CALIBRATION_OFFSET_EXIT_TOLERANCE_CODES        1.5f
#define CALIBRATION_OFFSET_MAX_STANDARD_ERROR_CODES    1.5f
#define CALIBRATION_OFFSET_MAX_UPDATE_CODES            0.25f
#define CALIBRATION_OFFSET_VERIFICATION_TARGET_CODES   2.0f
#define CALIBRATION_OFFSET_VERIFICATION_PROVISIONAL_LIMIT_CODES 5.0f
#define CALIBRATION_OFFSET_VERIFICATION_MAX_BATCHES    3U
#define CALIBRATION_OFFSET_MIN_ACCEPTED_FRAMES         21U
#define CALIBRATION_OFFSET_SCORE_RMSE_WEIGHT           0.1f
#define CALIBRATION_OFFSET_SCORE_STDERR_WEIGHT         0.5f
#define CALIBRATION_OFFSET_MIN_IMPROVEMENT_CODES       0.1f
#define CALIBRATION_OFFSET_MIN_BATCHES_BEFORE_STALL    10U
#define CALIBRATION_OFFSET_STALL_UPDATE_THRESHOLD_CODES 0.1f
#define CALIBRATION_OFFSET_NO_IMPROVEMENT_LIMIT        5U
#define CALIBRATION_OFFSET_MAX_ABS_CORRECTION_CODES    4096.0f
#define CALIBRATION_ADC_MIN_CODE                       (-8192)
#define CALIBRATION_ADC_MAX_CODE                       8191

#define CALIBRATION_GAIN_MAX_ACCEPTED_ITERATIONS       20U
#define CALIBRATION_GAIN_MAX_REJECTED_FRAMES           10U
#define CALIBRATION_GAIN_BATCH_SIZE                    30U
#define CALIBRATION_GAIN_TOLERANCE                     0.01f
#define CALIBRATION_GAIN_MAX_STANDARD_ERROR            0.005f
#define CALIBRATION_GAIN_REQUIRED_CONVERGED_FRAMES     2U
#define CALIBRATION_GAIN_UPDATE_STEP                   0.25f
#define CALIBRATION_GAIN_NEAR_UPDATE_STEP              0.15f
#define CALIBRATION_GAIN_NEAR_ERROR                    0.02f
#define CALIBRATION_GAIN_FILTER_ALPHA                  0.8f
#define CALIBRATION_GAIN_NO_IMPROVEMENT_LIMIT          5U
#define CALIBRATION_GAIN_MIN_IMPROVEMENT               0.001f
#define CALIBRATION_GAIN_RMSE_STOP_ABS_CODES           0.25f
#define CALIBRATION_GAIN_RMSE_STOP_RELATIVE            0.01f
#define CALIBRATION_GAIN_CORRECTION_MIN                0.5f
/* Firmware multiplier, not AD9695 register 0x1910.  Eight covers the
 * observed ~6.14 correction while corrected-sample clipping remains guarded. */
#define CALIBRATION_GAIN_CORRECTION_MAX                8.0f
#define CALIBRATION_GAIN_UPDATE_TOLERANCE              0.01f
#define CALIBRATION_GAIN_EFFECT_EPSILON                 0.002f
#define CALIBRATION_GAIN_EFFECT_SIGMA_MULTIPLIER        2.0f
#define CALIBRATION_GAIN_NO_EFFECT_LIMIT                3U
#define CALIBRATION_GAIN_BASELINE_TOLERANCE             0.10f

typedef enum {
    CALIBRATION_OFFSET_LOOP_IDLE = 0,
    CALIBRATION_OFFSET_LOOP_RUNNING,
    CALIBRATION_OFFSET_LOOP_CONTROLLER_CONVERGED,
    CALIBRATION_OFFSET_LOOP_VERIFYING,
    CALIBRATION_OFFSET_LOOP_PASS,
    CALIBRATION_OFFSET_LOOP_BEST_AVAILABLE,
    CALIBRATION_OFFSET_LOOP_NOT_CONVERGED,
    CALIBRATION_OFFSET_LOOP_FAILED
} calibration_offset_loop_status_t;

typedef enum {
    CALIBRATION_OFFSET_RESULT_NONE = 0,
    CALIBRATION_OFFSET_RESULT_CONVERGED,
    CALIBRATION_OFFSET_RESULT_PROVISIONAL,
    CALIBRATION_OFFSET_RESULT_FAILED
} calibration_offset_result_t;

typedef enum {
    CALIBRATION_GAIN_LOOP_IDLE = 0,
    CALIBRATION_GAIN_LOOP_RUNNING,
    CALIBRATION_GAIN_LOOP_PASS,
    CALIBRATION_GAIN_LOOP_BEST_AVAILABLE,
    CALIBRATION_GAIN_LOOP_NOT_CONVERGED,
    CALIBRATION_GAIN_LOOP_FAILED
} calibration_gain_loop_status_t;

typedef struct {
    /* Maximum iterations allowed for each stage. */
    uint32_t max_offset_iterations;
    uint32_t max_gain_iterations;

    /*
     * Convergence tolerances:
     *   offset_tolerance_codes: absolute residual mean error in ADC codes.
     *   gain_tolerance_ratio:   absolute fractional gain error.
     *                           Example: 0.001f = 0.1 percent.
     */
    float offset_tolerance_codes;
    float gain_tolerance_ratio;

    /*
     * Update step sizes in the interval (0, 1].
     * Smaller values reduce oscillation but require more captures.
     */
    float offset_step;
    float gain_step;

    /*
     * Safety limits for the accumulated correction values.
     * Gain is represented as a linear multiplier.
     */
    float min_gain_correction;
    float max_gain_correction;
    float min_offset_correction;
    float max_offset_correction;
} calibration_config_t;

typedef struct {
    float adc_mean;
    float reference_mean;

    /*
     * Least-squares model:
     *     adc ~= measured_gain * reference + measured_offset
     */
    float measured_gain;
    float measured_offset;

    /*
     * Residual errors after applying the current correction estimate.
     */
    float offset_error_codes;
    float gain_error_ratio;

    float adc_rms_ac;
    float reference_rms_ac;
    /* Raw aligned error: adc - reference. */
    float mae_codes;
    float rmse_codes;
    /* Residual after fitting adc ~= gain * reference + offset. */
    float fitted_rmse_codes;
    float fitted_mae_codes;
    float correlation;
} calibration_metrics_t;

typedef struct {
    calibration_config_t config;
    calibration_stage_t stage;

    /*
     * Correction model applied to raw ADC samples:
     *     corrected = raw * gain_correction + offset_correction
     */
    float offset_correction;
    float gain_correction;

    calibration_metrics_t metrics;

    uint32_t offset_iterations;
    uint32_t gain_iterations;

    uint8_t offset_converged;
    uint8_t gain_converged;
} calibration_state_t;

typedef struct {
    float offset_correction;
    float gain_correction;

    uint32_t accepted_frame_count;
    uint32_t rejected_frame_count;
    uint32_t convergence_count;
    uint32_t batch_iteration_count;

    calibration_offset_loop_status_t final_status;
    calibration_offset_result_t stage_result;

    float latest_correlation;
    float latest_mean_residual;
    float latest_fitted_offset;
    float latest_fitted_gain;
    float latest_rmse;
    float latest_raw_mean;
    float latest_corrected_mean;

    float best_abs_residual;
    float best_residual;
    float best_offset_correction;
    float latest_residual_stddev;
    float latest_residual_min;
    float latest_residual_max;
    float latest_batch_rmse;
    float best_rmse;
    float best_filtered_residual;
    float best_raw_residual;
    float best_score;
    float filtered_residual;
    float latest_standard_error;
    float latest_controller_gain;
    uint32_t no_improvement_count;
    uint8_t filtered_residual_valid;
    uint8_t restored_best_solution;
    uint8_t termination_reason;
    float verification_residual;
    float verification_correlation;
    float verification_stddev;
    float verification_standard_error;
    float verification_rmse;
    uint32_t verification_accepted_frames;
    uint32_t verification_batch_count;
    uint8_t verification_valid;
    uint8_t verification_status;
    uint8_t controller_converged;

    int8_t calibration_channel;
} calibration_offset_loop_state_t;

typedef struct {
    float gain_correction;
    float fixed_offset_correction;
    float initial_gain_correction;
    float final_requested_gain_correction;
    float initial_measured_gain;
    float previous_measured_gain;
    float best_measured_gain;
    float last_applied_gain_delta;
    float nominal_system_gain;
    float latest_raw_system_gain;
    float initial_normalized_gain;
    uint8_t nominal_system_gain_valid;
    uint32_t accepted_frame_count;
    uint32_t rejected_frame_count;
    uint32_t convergence_count;
    calibration_gain_loop_status_t final_status;
    float latest_fitted_gain;
    float latest_gain_error;
    float latest_fitted_offset;
    float latest_correlation;
    float latest_rmse;
    float latest_waveform_rmse;
    float latest_waveform_rmse_improvement;
    float previous_waveform_rmse;
    uint8_t have_previous_waveform_rmse;
    uint32_t batch_iteration_count;
    uint32_t no_improvement_count;
    float filtered_gain_error;
    float latest_gain_stddev;
    float latest_gain_standard_error;
    float latest_controller_gain;
    float best_gain_correction;
    float best_gain_error;
    float best_rmse;
    float best_score;
    uint8_t filtered_gain_error_valid;
    uint8_t restored_best_solution;
    uint8_t termination_reason;
    uint8_t saturation_occurred;
    uint8_t coefficient_changed;
    uint8_t initial_measured_gain_valid;
    uint8_t previous_measured_gain_valid;
    uint8_t no_observable_effect_count;
    float previous_gain_standard_error;
    float post_gain_residual;
    float post_gain_residual_stddev;
    float post_gain_residual_standard_error;
    uint32_t post_gain_residual_frames;
    uint8_t post_gain_residual_valid;
    const char *failure_reason;
    int8_t calibration_channel;
} calibration_gain_loop_state_t;

/* Populate a conservative default configuration. */
void calibration_default_config(calibration_config_t *config);

/* Initialize state using the supplied configuration. */
calibration_status_t calibration_init(
    calibration_state_t *state,
    const calibration_config_t *config
);

/*
 * Analyze one aligned ADC/reference frame using the state's current correction.
 *
 * adc_samples and reference_samples must correspond sample-for-sample.
 * A minimum of two samples is required.
 */
calibration_status_t calibration_analyze_frame(
    calibration_state_t *state,
    const int16_t *adc_samples,
    const int16_t *reference_samples,
    size_t sample_count
);

/*
 * Update only the active stage:
 *   OFFSET -> modifies offset_correction
 *   GAIN   -> modifies gain_correction
 *
 * The state automatically advances from offset to gain and then to complete
 * when the configured tolerances are satisfied.
 */
calibration_status_t calibration_update(calibration_state_t *state);

/* Convenience function: analyze the frame, then update the active stage. */
calibration_status_t calibration_process_frame(
    calibration_state_t *state,
    const int16_t *adc_samples,
    const int16_t *reference_samples,
    size_t sample_count
);

/* Apply the current correction estimate to one raw ADC code. */
float calibration_apply_sample(
    const calibration_state_t *state,
    int16_t raw_adc_sample
);

/* Returns nonzero only after both offset and gain stages converge. */
int calibration_is_complete(const calibration_state_t *state);

/* Human-readable stage name for UART/debug output. */
const char *calibration_stage_name(calibration_stage_t stage);

/* Persistent software offset-loop state. */
void calibration_offset_loop_reset(void);
calibration_offset_loop_state_t *calibration_offset_loop_state(void);
const char *calibration_offset_loop_status_name(
    calibration_offset_loop_status_t status
);

void calibration_gain_loop_reset(void);
calibration_gain_loop_state_t *calibration_gain_loop_state(void);
const char *calibration_gain_loop_status_name(
    calibration_gain_loop_status_t status
);
float calibration_software_gain_correction(void);
float calibration_software_offset_correction(void);
int calibration_set_software_gain_correction(float value);
int calibration_set_software_offset_correction(float value);
int8_t calibration_channel_selection(void);
int calibration_set_channel_selection(int8_t channel);
void calibration_all_loops_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_H */
