#include "adc_calibration_pipeline.h"

#include <string.h>

static void pipeline_print_stage(
    const adc_cal_pipeline_callbacks_t *callbacks,
    uint32_t stage_number,
    const char *stage_name)
{
    if (callbacks != NULL && callbacks->print_stage_header != NULL) {
        callbacks->print_stage_header(
            callbacks->context, stage_number, stage_name);
    }
}

static void pipeline_print_summary(
    const adc_cal_pipeline_callbacks_t *callbacks,
    const adc_cal_pipeline_state_t *state)
{
    if (callbacks != NULL && callbacks->print_summary != NULL) {
        callbacks->print_summary(callbacks->context, state);
    }
}

void adc_cal_pipeline_mark_performance_not_run(adc_cal_pipeline_state_t *state)
{
    if (state == NULL) return;
    state->performance_measurement_available = false;
    state->performance_valid = false;
    state->performance_failure_reason = NULL;
}

void adc_cal_pipeline_reset(adc_cal_pipeline_state_t *state)
{
    if (state == NULL) return;
    memset(state, 0, sizeof(*state));
    state->stage = ADC_CAL_PIPELINE_STAGE_IDLE;
    state->failed_stage = ADC_CAL_PIPELINE_STAGE_IDLE;
    state->overall_result = ADC_CAL_PIPELINE_RESULT_NONE;
    state->offset_result = ADC_CAL_PIPELINE_OFFSET_NONE;
    state->calibration_channel = -1;
    state->canonical_reference_phase = -1;
}

void adc_cal_pipeline_fail(
    adc_cal_pipeline_state_t *state,
    adc_cal_pipeline_stage_t failed_stage,
    const char *reason)
{
    if (state == NULL) return;
    state->active = false;
    state->valid = false;
    state->output_valid = false;
    state->stage = ADC_CAL_PIPELINE_STAGE_FAILED;
    state->failed_stage = failed_stage;
    state->overall_result = ADC_CAL_PIPELINE_RESULT_FAILED;
    state->failure_reason = reason;
}

const char *adc_cal_pipeline_stage_name(adc_cal_pipeline_stage_t stage)
{
    switch (stage) {
    case ADC_CAL_PIPELINE_STAGE_IDLE: return "IDLE";
    case ADC_CAL_PIPELINE_STAGE_TIMING: return "TIMING";
    case ADC_CAL_PIPELINE_STAGE_OFFSET: return "OFFSET";
    case ADC_CAL_PIPELINE_STAGE_GAIN: return "GAIN";
    case ADC_CAL_PIPELINE_STAGE_GAIN_VERIFY: return "GAIN_VERIFY";
    case ADC_CAL_PIPELINE_STAGE_SKEW: return "SKEW";
    case ADC_CAL_PIPELINE_STAGE_PERFORMANCE: return "PERFORMANCE";
    case ADC_CAL_PIPELINE_STAGE_COMPLETE: return "COMPLETE";
    case ADC_CAL_PIPELINE_STAGE_FAILED: return "FAILED";
    default: return "UNKNOWN";
    }
}

const char *adc_cal_pipeline_result_name(adc_cal_pipeline_result_t result)
{
    switch (result) {
    case ADC_CAL_PIPELINE_RESULT_PASS: return "PASS";
    case ADC_CAL_PIPELINE_RESULT_PROVISIONAL: return "PROVISIONAL";
    case ADC_CAL_PIPELINE_RESULT_FAILED: return "FAILED";
    case ADC_CAL_PIPELINE_RESULT_NONE:
    default:
        return "NOT COMPLETE";
    }
}

const char *adc_cal_pipeline_performance_status(
    const adc_cal_pipeline_state_t *state)
{
    if (state == NULL) return "NOT RUN";
    if (state->performance_measurement_available) {
        return state->performance_valid ? "VALID" : "INVALID";
    }
    if (state->active &&
        state->stage == ADC_CAL_PIPELINE_STAGE_PERFORMANCE) {
        return "RUNNING";
    }
    return "NOT RUN";
}

int adc_cal_pipeline_run_timing(
    adc_cal_pipeline_state_t *state,
    const adc_cal_pipeline_callbacks_t *callbacks,
    const adc_cal_pipeline_run_config_t *config)
{
    const char *reason = NULL;
    if (state == NULL || callbacks == NULL || config == NULL ||
        callbacks->run_timing == NULL) {
        return -1;
    }
    adc_cal_pipeline_reset(state);
    state->active = true;
    state->stage = ADC_CAL_PIPELINE_STAGE_TIMING;
    pipeline_print_stage(callbacks, 1U, "Timing Alignment");
    if (callbacks->run_timing(callbacks->context, state, config, &reason) != 0 ||
        !state->timing_pass) {
        if (callbacks->invalidate_timing != NULL) {
            callbacks->invalidate_timing(callbacks->context);
        }
        adc_cal_pipeline_fail(
            state,
            ADC_CAL_PIPELINE_STAGE_TIMING,
            reason != NULL ? reason : "timing alignment did not pass");
        pipeline_print_summary(callbacks, state);
        return -2;
    }
    state->active = false;
    pipeline_print_summary(callbacks, state);
    return 0;
}

int adc_cal_pipeline_run_offset(
    adc_cal_pipeline_state_t *state,
    const adc_cal_pipeline_callbacks_t *callbacks)
{
    const char *reason = NULL;
    if (state == NULL || callbacks == NULL || callbacks->run_offset == NULL ||
        !state->timing_pass) {
        return -1;
    }
    state->active = true;
    state->stage = ADC_CAL_PIPELINE_STAGE_OFFSET;
    pipeline_print_stage(callbacks, 2U, "Offset Calibration");
    if (callbacks->run_offset(callbacks->context, state, &reason) != 0 ||
        !state->offset_pass) {
        if (callbacks->invalidate_gain_input != NULL) {
            callbacks->invalidate_gain_input(callbacks->context);
        }
        adc_cal_pipeline_fail(
            state,
            ADC_CAL_PIPELINE_STAGE_OFFSET,
            reason != NULL ? reason : "offset result is unusable");
        pipeline_print_summary(callbacks, state);
        return -2;
    }
    state->active = false;
    pipeline_print_summary(callbacks, state);
    return 0;
}

int adc_cal_pipeline_run_gain(
    adc_cal_pipeline_state_t *state,
    const adc_cal_pipeline_callbacks_t *callbacks)
{
    const char *reason = NULL;
    if (state == NULL || callbacks == NULL || callbacks->run_gain == NULL ||
        !state->timing_pass || !state->offset_pass) {
        return -1;
    }
    state->active = true;
    state->stage = ADC_CAL_PIPELINE_STAGE_GAIN;
    pipeline_print_stage(callbacks, 3U, "Gain Calibration");
    if (callbacks->run_gain(callbacks->context, state, &reason) != 0 ||
        !state->gain_pass || !state->gain_verification_pass ||
        !state->output_valid) {
        if (callbacks->invalidate_gain_input != NULL) {
            callbacks->invalidate_gain_input(callbacks->context);
        }
        adc_cal_pipeline_fail(
            state,
            ADC_CAL_PIPELINE_STAGE_GAIN,
            reason != NULL ?
                reason : "gain did not converge or publish a verified output");
        pipeline_print_summary(callbacks, state);
        return -2;
    }
    state->active = false;
    pipeline_print_summary(callbacks, state);
    return 0;
}

int adc_cal_pipeline_run_skew(
    adc_cal_pipeline_state_t *state,
    const adc_cal_pipeline_callbacks_t *callbacks)
{
    const char *reason = NULL;
    if (state == NULL || callbacks == NULL || callbacks->run_skew == NULL ||
        !state->output_valid) {
        return -1;
    }
    state->active = true;
    state->stage = ADC_CAL_PIPELINE_STAGE_SKEW;
    pipeline_print_stage(callbacks, 4U, "Open-Loop Skew Measurement");
    if (callbacks->run_skew(callbacks->context, state, &reason) != 0) {
        adc_cal_pipeline_fail(
            state,
            ADC_CAL_PIPELINE_STAGE_SKEW,
            reason != NULL ? reason : "skew measurement failed");
        pipeline_print_summary(callbacks, state);
        return -2;
    }
    state->active = false;
    pipeline_print_summary(callbacks, state);
    return 0;
}

int adc_cal_pipeline_run_performance(
    adc_cal_pipeline_state_t *state,
    const adc_cal_pipeline_callbacks_t *callbacks)
{
    const char *reason = NULL;
    if (state == NULL || callbacks == NULL ||
        callbacks->run_performance == NULL || !state->output_valid) {
        return -1;
    }
    state->active = true;
    state->stage = ADC_CAL_PIPELINE_STAGE_PERFORMANCE;
    pipeline_print_stage(callbacks, 5U, "Performance Measurement");
    if (callbacks->run_performance(callbacks->context, state, &reason) != 0) {
        state->performance_measurement_available = true;
        state->performance_valid = false;
        state->performance_failure_reason =
            reason != NULL ? reason : "performance measurement failed";
    }
    state->active = false;
    pipeline_print_summary(callbacks, state);
    return state->performance_valid ? 0 : -2;
}

int adc_cal_pipeline_run_all(
    adc_cal_pipeline_state_t *state,
    const adc_cal_pipeline_callbacks_t *callbacks,
    const adc_cal_pipeline_run_config_t *config)
{
    const char *reason = NULL;

    if (state == NULL || callbacks == NULL || config == NULL ||
        callbacks->run_timing == NULL ||
        callbacks->run_offset == NULL ||
        callbacks->run_gain == NULL ||
        callbacks->run_skew == NULL ||
        callbacks->run_performance == NULL) {
        return -1;
    }

    adc_cal_pipeline_reset(state);
    state->active = true;
    state->stage = ADC_CAL_PIPELINE_STAGE_TIMING;

    if (callbacks->prepare != NULL &&
        callbacks->prepare(callbacks->context, config, &reason) != 0) {
        if (callbacks->invalidate_timing != NULL) {
            callbacks->invalidate_timing(callbacks->context);
        }
        adc_cal_pipeline_fail(
            state,
            ADC_CAL_PIPELINE_STAGE_TIMING,
            reason != NULL ? reason : "calibration preflight failed");
        pipeline_print_summary(callbacks, state);
        return -2;
    }

    pipeline_print_stage(callbacks, 1U, "Timing Alignment");
    if (callbacks->run_timing(callbacks->context, state, config, &reason) != 0 ||
        !state->timing_pass) {
        if (callbacks->invalidate_timing != NULL) {
            callbacks->invalidate_timing(callbacks->context);
        }
        adc_cal_pipeline_fail(
            state,
            ADC_CAL_PIPELINE_STAGE_TIMING,
            reason != NULL ? reason : "timing alignment did not pass");
        pipeline_print_summary(callbacks, state);
        return -3;
    }

    state->stage = ADC_CAL_PIPELINE_STAGE_OFFSET;
    pipeline_print_stage(callbacks, 2U, "Offset Calibration");
    if (callbacks->run_offset(callbacks->context, state, &reason) != 0 ||
        !state->offset_pass) {
        if (callbacks->invalidate_gain_input != NULL) {
            callbacks->invalidate_gain_input(callbacks->context);
        }
        adc_cal_pipeline_fail(
            state,
            ADC_CAL_PIPELINE_STAGE_OFFSET,
            reason != NULL ? reason : "offset result is unusable");
        pipeline_print_summary(callbacks, state);
        return -4;
    }

    state->stage = ADC_CAL_PIPELINE_STAGE_GAIN;
    pipeline_print_stage(callbacks, 3U, "Gain Calibration");
    if (callbacks->run_gain(callbacks->context, state, &reason) != 0 ||
        !state->gain_pass || !state->gain_verification_pass ||
        !state->output_valid) {
        if (callbacks->invalidate_gain_input != NULL) {
            callbacks->invalidate_gain_input(callbacks->context);
        }
        adc_cal_pipeline_fail(
            state,
            ADC_CAL_PIPELINE_STAGE_GAIN,
            reason != NULL ?
                reason : "gain did not converge or publish a verified output");
        pipeline_print_summary(callbacks, state);
        return -5;
    }

    state->stage = ADC_CAL_PIPELINE_STAGE_SKEW;
    pipeline_print_stage(callbacks, 4U, "Open-Loop Skew Measurement");
    if (callbacks->run_skew(callbacks->context, state, &reason) != 0) {
        adc_cal_pipeline_fail(
            state,
            ADC_CAL_PIPELINE_STAGE_SKEW,
            reason != NULL ? reason : "skew measurement failed");
        pipeline_print_summary(callbacks, state);
        return -6;
    }

    state->stage = ADC_CAL_PIPELINE_STAGE_PERFORMANCE;
    pipeline_print_stage(callbacks, 5U, "Performance Measurement");
    if (callbacks->run_performance(callbacks->context, state, &reason) != 0) {
        state->performance_measurement_available = true;
        state->performance_valid = false;
        state->performance_failure_reason =
            reason != NULL ? reason : "performance measurement failed";
    }

    state->active = false;
    state->stage = ADC_CAL_PIPELINE_STAGE_COMPLETE;
    state->overall_result =
        (state->offset_result == ADC_CAL_PIPELINE_OFFSET_PROVISIONAL ||
         !state->skew_pass) ?
            ADC_CAL_PIPELINE_RESULT_PROVISIONAL :
            ADC_CAL_PIPELINE_RESULT_PASS;
    pipeline_print_summary(callbacks, state);
    return 0;
}
