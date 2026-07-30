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
