#ifndef ADC_CALIBRATION_PIPELINE_H
#define ADC_CALIBRATION_PIPELINE_H

#include "adc_calibration_skew.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ADC_CAL_PIPELINE_STAGE_IDLE = 0,
    ADC_CAL_PIPELINE_STAGE_TIMING,
    ADC_CAL_PIPELINE_STAGE_OFFSET,
    ADC_CAL_PIPELINE_STAGE_GAIN,
    ADC_CAL_PIPELINE_STAGE_GAIN_VERIFY,
    ADC_CAL_PIPELINE_STAGE_SKEW,
    ADC_CAL_PIPELINE_STAGE_PERFORMANCE,
    ADC_CAL_PIPELINE_STAGE_COMPLETE,
    ADC_CAL_PIPELINE_STAGE_FAILED
} adc_cal_pipeline_stage_t;

typedef enum {
    ADC_CAL_PIPELINE_RESULT_NONE = 0,
    ADC_CAL_PIPELINE_RESULT_PASS,
    ADC_CAL_PIPELINE_RESULT_PROVISIONAL,
    ADC_CAL_PIPELINE_RESULT_FAILED
} adc_cal_pipeline_result_t;

typedef enum {
    ADC_CAL_PIPELINE_OFFSET_NONE = 0,
    ADC_CAL_PIPELINE_OFFSET_CONVERGED,
    ADC_CAL_PIPELINE_OFFSET_PROVISIONAL,
    ADC_CAL_PIPELINE_OFFSET_FAILED
} adc_cal_pipeline_offset_result_t;

typedef struct {
    bool active;
    bool valid;
    bool timing_pass;
    bool offset_pass;
    bool gain_pass;
    bool gain_verification_pass;
    bool skew_pass;
    bool skew_warning;
    bool skew_correction_applied;
    bool output_valid;
    bool performance_measurement_available;
    bool performance_valid;
    adc_cal_pipeline_stage_t stage;
    adc_cal_pipeline_stage_t failed_stage;
    adc_cal_pipeline_result_t overall_result;
    adc_cal_pipeline_offset_result_t offset_result;
    const char *failure_reason;
    const char *performance_failure_reason;
    int8_t calibration_channel;
    int8_t canonical_reference_phase;
    size_t fixed_window_start;
    size_t fixed_window_length;
    int32_t expected_lag;
    float timing_mean_correlation;
    float offset_correction;
    float offset_verification_error;
    float gain_correction;
    float nominal_system_gain;
    float final_normalized_gain;
    float gain_verification_error;
    double final_relative_skew_samples;
    double final_relative_skew_ps;
    adc_cal_skew_stage_policy_result_t skew_policy;
    uint32_t stage_iteration;
} adc_cal_pipeline_state_t;

typedef struct {
    uint32_t timing_frame_count;
} adc_cal_pipeline_run_config_t;

typedef struct {
    void *context;
    int (*prepare)(void *context, const adc_cal_pipeline_run_config_t *config,
                   const char **reason);
    int (*run_timing)(void *context, adc_cal_pipeline_state_t *state,
                      const adc_cal_pipeline_run_config_t *config,
                      const char **reason);
    int (*run_offset)(void *context, adc_cal_pipeline_state_t *state,
                      const char **reason);
    int (*run_gain)(void *context, adc_cal_pipeline_state_t *state,
                    const char **reason);
    int (*run_skew)(void *context, adc_cal_pipeline_state_t *state,
                    const char **reason);
    int (*run_performance)(void *context, adc_cal_pipeline_state_t *state,
                           const char **reason);
    void (*invalidate_timing)(void *context);
    void (*invalidate_gain_input)(void *context);
    void (*print_stage_header)(void *context, uint32_t stage_number,
                               const char *stage_name);
    void (*print_summary)(void *context, const adc_cal_pipeline_state_t *state);
} adc_cal_pipeline_callbacks_t;

void adc_cal_pipeline_reset(adc_cal_pipeline_state_t *state);
void adc_cal_pipeline_mark_performance_not_run(adc_cal_pipeline_state_t *state);
void adc_cal_pipeline_fail(adc_cal_pipeline_state_t *state,
                           adc_cal_pipeline_stage_t failed_stage,
                           const char *reason);
const char *adc_cal_pipeline_stage_name(adc_cal_pipeline_stage_t stage);
const char *adc_cal_pipeline_result_name(adc_cal_pipeline_result_t result);
const char *adc_cal_pipeline_performance_status(
    const adc_cal_pipeline_state_t *state);

int adc_cal_pipeline_run_all(
    adc_cal_pipeline_state_t *state,
    const adc_cal_pipeline_callbacks_t *callbacks,
    const adc_cal_pipeline_run_config_t *config);
int adc_cal_pipeline_run_timing(
    adc_cal_pipeline_state_t *state,
    const adc_cal_pipeline_callbacks_t *callbacks,
    const adc_cal_pipeline_run_config_t *config);
int adc_cal_pipeline_run_offset(
    adc_cal_pipeline_state_t *state,
    const adc_cal_pipeline_callbacks_t *callbacks);
int adc_cal_pipeline_run_gain(
    adc_cal_pipeline_state_t *state,
    const adc_cal_pipeline_callbacks_t *callbacks);
int adc_cal_pipeline_run_skew(
    adc_cal_pipeline_state_t *state,
    const adc_cal_pipeline_callbacks_t *callbacks);
int adc_cal_pipeline_run_performance(
    adc_cal_pipeline_state_t *state,
    const adc_cal_pipeline_callbacks_t *callbacks);

#ifdef __cplusplus
}
#endif

#endif /* ADC_CALIBRATION_PIPELINE_H */
