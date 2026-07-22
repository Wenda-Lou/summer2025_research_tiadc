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
#define CALIBRATION_OFFSET_REQUIRED_CONVERGED_FRAMES   3U
#define CALIBRATION_OFFSET_UPDATE_STEP                 0.5f
#define CALIBRATION_OFFSET_MAX_ABS_CORRECTION_CODES    4096.0f
#define CALIBRATION_ADC_MIN_CODE                       (-8192)
#define CALIBRATION_ADC_MAX_CODE                       8191

#define CALIBRATION_GAIN_MAX_ACCEPTED_ITERATIONS       20U
#define CALIBRATION_GAIN_MAX_REJECTED_FRAMES           10U
#define CALIBRATION_GAIN_TOLERANCE                     0.005f
#define CALIBRATION_GAIN_REQUIRED_CONVERGED_FRAMES     3U
#define CALIBRATION_GAIN_UPDATE_STEP                   0.5f
#define CALIBRATION_GAIN_CORRECTION_MIN                0.5f
#define CALIBRATION_GAIN_CORRECTION_MAX                2.0f
#define CALIBRATION_GAIN_FITTED_MIN                    0.05f

typedef enum {
    CALIBRATION_OFFSET_LOOP_IDLE = 0,
    CALIBRATION_OFFSET_LOOP_RUNNING,
    CALIBRATION_OFFSET_LOOP_PASS,
    CALIBRATION_OFFSET_LOOP_NOT_CONVERGED,
    CALIBRATION_OFFSET_LOOP_FAILED
} calibration_offset_loop_status_t;

typedef enum {
    CALIBRATION_GAIN_LOOP_IDLE = 0,
    CALIBRATION_GAIN_LOOP_RUNNING,
    CALIBRATION_GAIN_LOOP_PASS,
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

    calibration_offset_loop_status_t final_status;

    float latest_correlation;
    float latest_fitted_offset;
    float latest_fitted_gain;
    float latest_rmse;
    float latest_raw_mean;
    float latest_corrected_mean;

    int8_t calibration_channel;
} calibration_offset_loop_state_t;

typedef struct {
    float gain_correction;
    float fixed_offset_correction;
    uint32_t accepted_frame_count;
    uint32_t rejected_frame_count;
    uint32_t convergence_count;
    calibration_gain_loop_status_t final_status;
    float latest_fitted_gain;
    float latest_gain_error;
    float latest_fitted_offset;
    float latest_correlation;
    float latest_rmse;
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

calibration_gain_loop_state_t *calibration_gain_loop_state(void);
const char *calibration_gain_loop_status_name(
    calibration_gain_loop_status_t status
);
float calibration_software_gain_correction(void);
float calibration_software_offset_correction(void);
int calibration_set_software_gain_correction(float value);
int calibration_set_software_offset_correction(float value);
void calibration_all_loops_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_H */
