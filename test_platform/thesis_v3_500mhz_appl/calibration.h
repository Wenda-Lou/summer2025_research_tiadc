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
     *     corrected = (raw + offset_correction) * gain_correction
     */
    float offset_correction;
    float gain_correction;

    calibration_metrics_t metrics;

    uint32_t offset_iterations;
    uint32_t gain_iterations;

    uint8_t offset_converged;
    uint8_t gain_converged;
} calibration_state_t;

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

#ifdef __cplusplus
}
#endif

#endif /* CALIBRATION_H */
