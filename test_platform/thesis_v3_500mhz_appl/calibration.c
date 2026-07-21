#include "calibration.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define CALIBRATION_MIN_SAMPLES 2U
#define CALIBRATION_EPSILON     1.0e-20

static calibration_offset_loop_state_t g_offset_loop_state;

static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int config_is_valid(const calibration_config_t *config)
{
    if (config == NULL) {
        return 0;
    }

    if ((config->max_offset_iterations == 0U) ||
        (config->max_gain_iterations == 0U)) {
        return 0;
    }

    if ((config->offset_tolerance_codes < 0.0f) ||
        (config->gain_tolerance_ratio < 0.0f)) {
        return 0;
    }

    if ((config->offset_step <= 0.0f) || (config->offset_step > 1.0f) ||
        (config->gain_step <= 0.0f) || (config->gain_step > 1.0f)) {
        return 0;
    }

    if ((config->min_gain_correction <= 0.0f) ||
        (config->max_gain_correction < config->min_gain_correction) ||
        (config->max_offset_correction < config->min_offset_correction)) {
        return 0;
    }

    return 1;
}

void calibration_default_config(calibration_config_t *config)
{
    if (config == NULL) {
        return;
    }

    config->max_offset_iterations = 20U;
    config->max_gain_iterations = 20U;

    config->offset_tolerance_codes = 0.5f;
    config->gain_tolerance_ratio = 0.001f;

    config->offset_step = 0.5f;
    config->gain_step = 0.25f;

    config->min_gain_correction = 0.5f;
    config->max_gain_correction = 2.0f;

    config->min_offset_correction = -8192.0f;
    config->max_offset_correction = 8192.0f;
}

calibration_status_t calibration_init(
    calibration_state_t *state,
    const calibration_config_t *config
)
{
    if ((state == NULL) || (config == NULL)) {
        return CALIBRATION_ERR_NULL;
    }

    if (!config_is_valid(config)) {
        return CALIBRATION_ERR_INVALID_CONFIG;
    }

    memset(state, 0, sizeof(*state));
    state->config = *config;
    state->stage = CALIBRATION_STAGE_OFFSET;
    state->offset_correction = 0.0f;
    state->gain_correction = 1.0f;

    return CALIBRATION_OK;
}

calibration_status_t calibration_analyze_frame(
    calibration_state_t *state,
    const int16_t *adc_samples,
    const int16_t *reference_samples,
    size_t sample_count
)
{
    size_t i;
    double sum_adc = 0.0;
    double sum_ref = 0.0;
    double mean_adc;
    double mean_ref;

    double sum_ref_ac2 = 0.0;
    double sum_adc_ac2 = 0.0;
    double sum_cross = 0.0;
    double sum_error2 = 0.0;
    double sum_absolute_error = 0.0;
    double sum_fitted_error2 = 0.0;
    double sum_fitted_absolute_error = 0.0;

    double measured_gain;
    double measured_offset;
    double corrected_mean_error;
    double corrected_gain;
    double denom_corr;

    if ((state == NULL) || (adc_samples == NULL) ||
        (reference_samples == NULL)) {
        return CALIBRATION_ERR_NULL;
    }

    if (sample_count < CALIBRATION_MIN_SAMPLES) {
        return CALIBRATION_ERR_SAMPLE_COUNT;
    }

    /*
     * First pass: calculate means after applying the current correction to ADC.
     */
    for (i = 0U; i < sample_count; ++i) {
        const double corrected_adc =
            ((double)adc_samples[i] + (double)state->offset_correction) *
            (double)state->gain_correction;

        sum_adc += corrected_adc;
        sum_ref += (double)reference_samples[i];
    }

    mean_adc = sum_adc / (double)sample_count;
    mean_ref = sum_ref / (double)sample_count;

    /*
     * Second pass: centered least-squares gain, RMS, correlation, and RMSE.
     */
    for (i = 0U; i < sample_count; ++i) {
        const double corrected_adc =
            ((double)adc_samples[i] + (double)state->offset_correction) *
            (double)state->gain_correction;
        const double reference = (double)reference_samples[i];

        const double adc_ac = corrected_adc - mean_adc;
        const double ref_ac = reference - mean_ref;
        const double error = corrected_adc - reference;

        sum_adc_ac2 += adc_ac * adc_ac;
        sum_ref_ac2 += ref_ac * ref_ac;
        sum_cross += ref_ac * adc_ac;
        sum_error2 += error * error;
        sum_absolute_error += fabs(error);
    }

    if (sum_ref_ac2 <= CALIBRATION_EPSILON) {
        return CALIBRATION_ERR_ZERO_REFERENCE_POWER;
    }

    measured_gain = sum_cross / sum_ref_ac2;
    measured_offset = mean_adc - (measured_gain * mean_ref);

    /*
     * The active correction has already been applied above. Therefore:
     *   desired residual gain = 1
     *   desired residual mean error = 0
     */
    corrected_gain = measured_gain;
    corrected_mean_error = mean_adc - mean_ref;

    /*
     * Third pass: residual after removing the best-fit gain and offset.
     * Keep this separate from raw aligned RMSE, which measures adc-reference.
     */
    for (i = 0U; i < sample_count; ++i) {
        const double corrected_adc =
            ((double)adc_samples[i] + (double)state->offset_correction) *
            (double)state->gain_correction;
        const double fitted_adc =
            measured_gain * (double)reference_samples[i] + measured_offset;
        const double fitted_error = corrected_adc - fitted_adc;

        sum_fitted_error2 += fitted_error * fitted_error;
        sum_fitted_absolute_error += fabs(fitted_error);
    }

    denom_corr = sqrt(sum_ref_ac2 * sum_adc_ac2);

    state->metrics.adc_mean = (float)mean_adc;
    state->metrics.reference_mean = (float)mean_ref;
    state->metrics.measured_gain = (float)measured_gain;
    state->metrics.measured_offset = (float)measured_offset;
    state->metrics.offset_error_codes = (float)corrected_mean_error;
    state->metrics.gain_error_ratio = (float)(corrected_gain - 1.0);
    state->metrics.adc_rms_ac =
        (float)sqrt(sum_adc_ac2 / (double)sample_count);
    state->metrics.reference_rms_ac =
        (float)sqrt(sum_ref_ac2 / (double)sample_count);
    state->metrics.rmse_codes =
        (float)sqrt(sum_error2 / (double)sample_count);
    state->metrics.mae_codes =
        (float)(sum_absolute_error / (double)sample_count);
    state->metrics.fitted_rmse_codes =
        (float)sqrt(sum_fitted_error2 / (double)sample_count);
    state->metrics.fitted_mae_codes =
        (float)(sum_fitted_absolute_error / (double)sample_count);

    if (denom_corr > CALIBRATION_EPSILON) {
        state->metrics.correlation = (float)(sum_cross / denom_corr);
    } else {
        state->metrics.correlation = 0.0f;
    }

    return CALIBRATION_OK;
}

calibration_status_t calibration_update(calibration_state_t *state)
{
    float update;

    if (state == NULL) {
        return CALIBRATION_ERR_NULL;
    }

    if (!config_is_valid(&state->config)) {
        state->stage = CALIBRATION_STAGE_FAILED;
        return CALIBRATION_ERR_INVALID_CONFIG;
    }

    switch (state->stage) {
    case CALIBRATION_STAGE_OFFSET:
        if (fabsf(state->metrics.offset_error_codes) <=
            state->config.offset_tolerance_codes) {
            state->offset_converged = 1U;
            state->stage = CALIBRATION_STAGE_GAIN;
            return CALIBRATION_OK;
        }

        if (state->offset_iterations >=
            state->config.max_offset_iterations) {
            state->stage = CALIBRATION_STAGE_FAILED;
            return CALIBRATION_OK;
        }

        /*
         * corrected = (raw + offset_correction) * gain_correction
         *
         * To reduce a positive output mean error, decrease offset_correction.
         * Divide by gain so the requested output-code change is expressed at
         * the adder input.
         */
        update = state->config.offset_step *
                 state->metrics.offset_error_codes /
                 state->gain_correction;

        state->offset_correction = clamp_float(
            state->offset_correction - update,
            state->config.min_offset_correction,
            state->config.max_offset_correction
        );

        ++state->offset_iterations;
        break;

    case CALIBRATION_STAGE_GAIN:
        if (fabsf(state->metrics.gain_error_ratio) <=
            state->config.gain_tolerance_ratio) {
            state->gain_converged = 1U;
            state->stage = CALIBRATION_STAGE_COMPLETE;
            return CALIBRATION_OK;
        }

        if (state->gain_iterations >= state->config.max_gain_iterations) {
            state->stage = CALIBRATION_STAGE_FAILED;
            return CALIBRATION_OK;
        }

        /*
         * measured_gain is the residual slope after applying the existing
         * correction. The exact next multiplier would be:
         *
         *     gain_correction /= measured_gain
         *
         * Blend toward that value using gain_step to reduce oscillation.
         */
        if (fabsf(state->metrics.measured_gain) <= FLT_EPSILON) {
            state->stage = CALIBRATION_STAGE_FAILED;
            return CALIBRATION_ERR_ZERO_REFERENCE_POWER;
        }

        update = state->gain_correction /
                 state->metrics.measured_gain;

        state->gain_correction += state->config.gain_step *
                                  (update - state->gain_correction);

        state->gain_correction = clamp_float(
            state->gain_correction,
            state->config.min_gain_correction,
            state->config.max_gain_correction
        );

        ++state->gain_iterations;
        break;

    case CALIBRATION_STAGE_COMPLETE:
    case CALIBRATION_STAGE_FAILED:
    default:
        break;
    }

    return CALIBRATION_OK;
}

calibration_status_t calibration_process_frame(
    calibration_state_t *state,
    const int16_t *adc_samples,
    const int16_t *reference_samples,
    size_t sample_count
)
{
    calibration_status_t status;

    status = calibration_analyze_frame(
        state,
        adc_samples,
        reference_samples,
        sample_count
    );

    if (status != CALIBRATION_OK) {
        if (state != NULL) {
            state->stage = CALIBRATION_STAGE_FAILED;
        }
        return status;
    }

    return calibration_update(state);
}

float calibration_apply_sample(
    const calibration_state_t *state,
    int16_t raw_adc_sample
)
{
    if (state == NULL) {
        return (float)raw_adc_sample;
    }

    return ((float)raw_adc_sample + state->offset_correction) *
           state->gain_correction;
}

int calibration_is_complete(const calibration_state_t *state)
{
    return (state != NULL) &&
           (state->stage == CALIBRATION_STAGE_COMPLETE);
}

const char *calibration_stage_name(calibration_stage_t stage)
{
    switch (stage) {
    case CALIBRATION_STAGE_OFFSET:
        return "offset";
    case CALIBRATION_STAGE_GAIN:
        return "gain";
    case CALIBRATION_STAGE_COMPLETE:
        return "complete";
    case CALIBRATION_STAGE_FAILED:
        return "failed";
    default:
        return "unknown";
    }
}

void calibration_offset_loop_reset(void)
{
    memset(&g_offset_loop_state, 0, sizeof(g_offset_loop_state));
    g_offset_loop_state.gain_correction = 1.0f;
    g_offset_loop_state.calibration_channel = -1;
    g_offset_loop_state.final_status = CALIBRATION_OFFSET_LOOP_IDLE;
}

calibration_offset_loop_state_t *calibration_offset_loop_state(void)
{
    if (g_offset_loop_state.gain_correction == 0.0f) {
        calibration_offset_loop_reset();
    }

    return &g_offset_loop_state;
}

const char *calibration_offset_loop_status_name(
    calibration_offset_loop_status_t status
)
{
    switch (status) {
    case CALIBRATION_OFFSET_LOOP_IDLE:
        return "IDLE";
    case CALIBRATION_OFFSET_LOOP_RUNNING:
        return "RUNNING";
    case CALIBRATION_OFFSET_LOOP_PASS:
        return "PASS";
    case CALIBRATION_OFFSET_LOOP_NOT_CONVERGED:
        return "NOT CONVERGED";
    case CALIBRATION_OFFSET_LOOP_FAILED:
        return "FAILED";
    default:
        return "UNKNOWN";
    }
}
